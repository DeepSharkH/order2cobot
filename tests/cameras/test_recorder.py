import base64
import hashlib
import json
import socket
import struct
import threading
import time
from collections.abc import Iterator
from pathlib import Path

import cv2
import numpy as np
import pytest

from agents.cameras import CameraError, CameraRecorder
from agents.cameras.recorder import WS_GUID


def ws_frame(opcode: int, payload: bytes) -> bytes:
    n = len(payload)
    if n < 126:
        head = bytes([0x80 | opcode, n])
    elif n < 1 << 16:
        head = bytes([0x80 | opcode, 126]) + struct.pack("!H", n)
    else:
        head = bytes([0x80 | opcode, 127]) + struct.pack("!Q", n)
    return head + payload


def jpeg(color: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((48, 64, 3), color, np.uint8))
    assert ok
    return buf.tobytes()


class FakeDashboard:
    """Accepts websocket clients and sends a state message plus frames for `cameras`."""

    def __init__(self, cameras: list[int], hz: float = 50.0) -> None:
        self.cameras, self.hz = cameras, hz
        self.server = socket.create_server(("127.0.0.1", 0))
        self.url = f"http://127.0.0.1:{self.server.getsockname()[1]}"
        self.stop = threading.Event()
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self) -> None:
        self.server.settimeout(0.2)
        while not self.stop.is_set():
            try:
                conn, _ = self.server.accept()
            except OSError:
                continue
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn: socket.socket) -> None:
        request = b""
        while b"\r\n\r\n" not in request:
            request += conn.recv(4096)
        key = next(
            line.split(":", 1)[1].strip()
            for line in request.decode().split("\r\n")
            if line.lower().startswith("sec-websocket-key")
        )
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        conn.sendall(
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n".encode()
        )
        state = {"type": "state", "cams": [{"name": f"cam {i}"} for i in range(2)]}
        try:
            conn.sendall(ws_frame(1, json.dumps(state).encode()))
            conn.sendall(ws_frame(9, b"hi"))  # a ping the client must answer
            n = 0
            while not self.stop.is_set():
                for i in self.cameras:
                    conn.sendall(ws_frame(2, bytes([1, i]) + jpeg(40 * i + n % 40)))
                conn.sendall(ws_frame(2, bytes([2, 0]) + b"cloud"))  # ignored
                n += 1
                time.sleep(1 / self.hz)
        except OSError:
            pass
        finally:
            conn.close()

    def close(self) -> None:
        self.stop.set()
        self.server.close()


@pytest.fixture
def dashboard() -> Iterator[FakeDashboard]:
    d = FakeDashboard([0, 1])
    yield d
    d.close()


def test_records_one_video_per_camera(tmp_path: Path, dashboard: FakeDashboard) -> None:
    with CameraRecorder(dashboard.url, tmp_path, [0, 1], fps=20) as rec:
        time.sleep(1.0)
    summary = rec.summary()

    assert summary["errors"] == [] and summary["reconnects"] == 0
    for cam in summary["cameras"]:
        assert cam["name"] == f"cam {cam['camera']}" and cam["size"] == [64, 48]
        assert cam["frames_received"] > 20 and cam["frames_written"] >= 15
        video = cv2.VideoCapture(str(tmp_path / cam["file"]))
        assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) == cam["frames_written"]
        assert video.get(cv2.CAP_PROP_FPS) == 20


def test_missing_camera_fails_to_start(tmp_path: Path) -> None:
    d = FakeDashboard([0])
    try:
        with pytest.raises(CameraError, match=r"camera\(s\) \[1\]"):
            CameraRecorder(d.url, tmp_path, [0, 1], start_timeout_s=0.5).start()
    finally:
        d.close()


def test_unreachable_dashboard_fails_to_start(tmp_path: Path) -> None:
    with socket.create_server(("127.0.0.1", 0)) as s:
        port = s.getsockname()[1]
    with pytest.raises(CameraError, match="cannot reach"):
        CameraRecorder(f"http://127.0.0.1:{port}", tmp_path, [0]).start()
