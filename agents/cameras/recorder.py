"""Record the dashboard's live camera feeds to one MP4 per camera.

The calibration dashboard owns the cameras (see configs/vision.yaml) and pushes every feed
over its `/ws` websocket: binary messages `[1, camera index] + JPEG` (about stream_hz per
camera, stream_width wide, with the dashboard's overlays when they are switched on), plus
JSON state messages that carry the camera names. One thread reads the socket and keeps the
newest frame per camera; another writes the newest frame of each camera at a fixed `fps`,
repeating it while the feed stalls, so a video's length matches the wall-clock time from its
`started_at`. A dropped connection is reopened until the recording stops.

The websocket client is a minimal RFC 6455 reader on the standard library (no TLS, no
extensions), enough for the local dashboard.
"""

import base64
import contextlib
import hashlib
import json
import logging
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from types import TracebackType
from typing import Any, BinaryIO
from urllib.parse import urlsplit

import cv2
import numpy as np

from agents.claude_cli.session import RecorderError

log = logging.getLogger(__name__)

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
FRAME_MSG = 1  # first byte of a camera frame message; the second is the camera index


class CameraError(RecorderError):
    """The dashboard cannot be reached, or a camera sends no frames."""


class WebSocket:
    """Blocking websocket client: connect, then `recv()` whole messages."""

    def __init__(self, url: str, timeout_s: float = 5.0) -> None:
        parts = urlsplit(url)
        if parts.scheme != "ws":
            raise CameraError(f"only ws:// urls are supported: {url}")
        host, port = parts.hostname or "localhost", parts.port or 80
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.file: BinaryIO = self.sock.makefile("rb")
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET {parts.path or '/'} HTTP/1.1\r\nHost: {host}:{port}\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        status = self.file.readline().decode(errors="replace").strip()
        headers = {}
        while (line := self.file.readline().decode(errors="replace").strip()) != "":
            name, _, value = line.partition(":")
            headers[name.strip().lower()] = value.strip()
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        if " 101 " not in f"{status} " or headers.get("sec-websocket-accept") != accept:
            self.close()
            raise CameraError(f"{url}: websocket handshake failed ({status})")

    def recv(self) -> tuple[int, bytes]:
        """The next text (1) or binary (2) message as (opcode, payload); answers pings."""
        opcode, parts = 0, []
        while True:
            b0, b1 = self._read(2)
            op, n = b0 & 0x0F, b1 & 0x7F
            if n == 126:
                (n,) = struct.unpack("!H", self._read(2))
            elif n == 127:
                (n,) = struct.unpack("!Q", self._read(8))
            mask = self._read(4) if b1 & 0x80 else None
            data = self._read(n)
            if mask:
                data = bytes(c ^ mask[i % 4] for i, c in enumerate(data))
            if op == 8:
                raise ConnectionError("the dashboard closed the websocket")
            if op == 9:
                self._send(10, data)
                continue
            if op == 10:
                continue
            if op != 0:
                opcode = op
            parts.append(data)
            if b0 & 0x80:  # FIN
                return opcode, b"".join(parts)

    def close(self) -> None:
        with contextlib.suppress(OSError):
            self._send(8, b"")
        self.file.close()
        self.sock.close()

    def _read(self, n: int) -> bytes:
        data = self.file.read(n)
        if data is None or len(data) < n:
            raise ConnectionError("the websocket closed")
        return data

    def _send(self, opcode: int, payload: bytes) -> None:
        mask = os.urandom(4)  # client frames must be masked
        masked = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        self.sock.sendall(bytes([0x80 | opcode, 0x80 | len(payload)]) + mask + masked)


@dataclass
class Track:
    """One camera's video."""

    index: int
    name: str | None = None
    file: Path | None = None
    size: tuple[int, int] | None = None
    started_at: datetime | None = None
    frames_received: int = 0
    frames_written: int = 0
    repeated: int = 0  # written again because no new frame came in time
    writer: Any = None
    latest: tuple[int, bytes] | None = None  # (sequence, JPEG)
    written_seq: int = -1
    decoded: Any = field(default=None, repr=False)

    def summary(self, fps: float) -> dict[str, Any]:
        duration = self.frames_written / fps
        return {
            "camera": self.index,
            "name": self.name,
            "file": self.file.name if self.file else None,
            "size": list(self.size) if self.size else None,
            "started_at": self.started_at.isoformat() if self.started_at else None,
            "duration_s": round(duration, 2),
            "frames_received": self.frames_received,
            "frames_written": self.frames_written,
            "frames_repeated": self.repeated,
        }


class CameraRecorder:
    """Context manager: records `cameras` from the dashboard into `out_dir` while it is open."""

    def __init__(
        self,
        dashboard_url: str,
        out_dir: Path,
        cameras: list[int],
        *,
        fps: float = 15.0,
        start_timeout_s: float = 10.0,
        fourcc: str = "mp4v",
    ) -> None:
        base = dashboard_url.rstrip("/")
        self.ws_url = base.replace("http://", "ws://", 1) + "/ws"
        self.out_dir = out_dir
        self.fps = fps
        self.start_timeout_s = start_timeout_s
        self.fourcc = fourcc
        self.tracks = {i: Track(i) for i in cameras}
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.threads: list[threading.Thread] = []
        self.started_at: datetime | None = None
        self.stopped_at: datetime | None = None
        self.reconnects = 0
        self.errors: list[str] = []

    def __enter__(self) -> "CameraRecorder":
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.stop()

    def start(self) -> None:
        """Connect and wait until every camera has sent a frame; raise CameraError if not."""
        ws = self._connect()  # fail fast when the dashboard is down
        self.started_at = datetime.now(timezone.utc)
        self.threads = [
            threading.Thread(target=self._read_loop, args=(ws,), daemon=True, name="cam-read"),
            threading.Thread(target=self._write_loop, daemon=True, name="cam-write"),
        ]
        for t in self.threads:
            t.start()
        deadline = time.monotonic() + self.start_timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                if all(t.frames_written for t in self.tracks.values()):
                    break
            time.sleep(0.05)
        else:
            missing = [i for i, t in self.tracks.items() if not t.frames_written]
            self.stop()
            last = f" ({self.errors[-1]})" if self.errors else ""
            raise CameraError(
                f"no frames from camera(s) {missing} within {self.start_timeout_s:.0f} s "
                f"on {self.ws_url}{last}"
            )
        names = ", ".join(f"{t.index}={t.name or '?'}" for t in self.tracks.values())
        log.info("recording cameras %s at %.0f fps into %s", names, self.fps, self.out_dir)

    def stop(self) -> None:
        if self.stopping.is_set():
            return
        self.stopping.set()
        for t in self.threads:
            t.join(timeout=5)
        with self.lock:
            for track in self.tracks.values():
                if track.writer is not None:
                    track.writer.release()
                    track.writer = None
        self.stopped_at = datetime.now(timezone.utc)

    def summary(self) -> dict[str, Any]:
        with self.lock:
            return {
                "source": self.ws_url,
                "fps": self.fps,
                "codec": self.fourcc,
                "started_at": self.started_at.isoformat() if self.started_at else None,
                "stopped_at": self.stopped_at.isoformat() if self.stopped_at else None,
                "reconnects": self.reconnects,
                "errors": self.errors[-20:],
                "cameras": [t.summary(self.fps) for t in self.tracks.values()],
            }

    # -- threads ----------------------------------------------------------------------

    def _connect(self) -> WebSocket:
        try:
            return WebSocket(self.ws_url)
        except OSError as exc:
            raise CameraError(f"cannot reach the dashboard at {self.ws_url}: {exc}") from exc

    def _read_loop(self, ws: WebSocket | None) -> None:
        seq = 0
        while not self.stopping.is_set():
            try:
                if ws is None:
                    ws = self._connect()
                    self.reconnects += 1
                opcode, data = ws.recv()
            except (OSError, ConnectionError, CameraError) as exc:
                if ws is not None:
                    ws.close()
                    ws = None
                if self.stopping.is_set():
                    break
                self._error(f"camera feed: {exc}; reconnecting")
                self.stopping.wait(1.0)
                continue
            if opcode == 1:
                self._on_state(data)
            elif len(data) > 2 and data[0] == FRAME_MSG and data[1] in self.tracks:
                seq += 1
                with self.lock:
                    track = self.tracks[data[1]]
                    track.latest = (seq, data[2:])
                    track.frames_received += 1
        if ws is not None:
            ws.close()

    def _on_state(self, data: bytes) -> None:
        try:
            cams = json.loads(data).get("cams") or []
        except (json.JSONDecodeError, AttributeError):
            return
        with self.lock:
            for i, cam in enumerate(cams):
                if i in self.tracks and isinstance(cam, dict) and cam.get("name"):
                    self.tracks[i].name = str(cam["name"])

    def _write_loop(self) -> None:
        dt = 1.0 / self.fps
        tick = time.monotonic()
        while not self.stopping.is_set():
            try:
                with self.lock:
                    for track in self.tracks.values():
                        self._write(track)
            except CameraError as exc:  # no writer for this format: nothing more to record
                self._error(str(exc))
                return
            tick += dt
            delay = tick - time.monotonic()
            if delay < -1.0:  # fell far behind (machine busy): skip ahead instead of bursting
                tick = time.monotonic()
            self.stopping.wait(max(delay, 0.0))

    def _write(self, track: Track) -> None:
        """Write the newest frame of `track`, or the previous one again; caller holds the lock."""
        if track.latest is None:
            return  # nothing yet: the video starts with the camera's first frame
        seq, jpeg = track.latest
        if seq != track.written_seq:
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self._error(f"camera {track.index}: undecodable frame", locked=True)
                return
            track.decoded, track.written_seq = img, seq
        elif track.decoded is None:
            return
        else:
            track.repeated += 1
        img = track.decoded
        if track.writer is None:
            self._open(track, img)
        h, w = img.shape[:2]
        if (w, h) != track.size:
            img = cv2.resize(img, track.size, interpolation=cv2.INTER_AREA)
        track.writer.write(img)
        track.frames_written += 1

    def _open(self, track: Track, img: Any) -> None:
        h, w = img.shape[:2]
        track.size = (w, h)
        track.file = self.out_dir / f"camera{track.index}.mp4"
        track.writer = cv2.VideoWriter(
            str(track.file), cv2.VideoWriter.fourcc(*self.fourcc), self.fps, (w, h)
        )
        if not track.writer.isOpened():
            raise CameraError(f"cannot open a {self.fourcc} video writer for {track.file}")
        track.started_at = datetime.now(timezone.utc)

    def _error(self, msg: str, locked: bool = False) -> None:
        log.warning(msg)
        if locked:
            self.errors.append(msg)
        else:
            with self.lock:
                self.errors.append(msg)
