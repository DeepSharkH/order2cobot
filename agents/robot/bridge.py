"""Async MCP client for the AI Agent Bridge (Streamable HTTP, JSON-RPC over POST)."""

import json
import ssl
from types import TracebackType
from typing import Any

import httpx

from agents.robot.config import RobotConfig

PROTOCOL_VERSION = "2025-06-18"


class BridgeError(RuntimeError):
    """A bridge or Robot API call failed."""


def _ssl_context(cert_path: str) -> ssl.SSLContext:
    try:
        ctx = ssl.create_default_context(cafile=cert_path)
    except (OSError, ssl.SSLError) as exc:
        raise BridgeError(f"cannot load robot certificate {cert_path}: {exc}") from exc
    # The robot's self-signed cert has no subjectAltName, only a common name.
    ctx.hostname_checks_common_name = True
    return ctx


class Bridge:
    """One MCP session with the bridge; use as `async with Bridge(...) as bridge`."""

    def __init__(self, config: RobotConfig, client_name: str) -> None:
        self.config = config
        self.client_name = client_name
        self._http = httpx.AsyncClient(
            verify=_ssl_context(str(config.cert_path)),
            timeout=config.request_timeout_s,
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json, text/event-stream",
            },
        )
        self._next_id = 1

    async def __aenter__(self) -> "Bridge":
        try:
            r = await self._post(
                "initialize",
                {
                    "protocolVersion": PROTOCOL_VERSION,
                    "capabilities": {},
                    "clientInfo": {"name": self.client_name, "version": "0.1"},
                },
            )
            self._http.headers["Mcp-Session-Id"] = r.headers["Mcp-Session-Id"]
            await self._http.post(
                self.config.bridge_url,
                json={"jsonrpc": "2.0", "method": "notifications/initialized"},
            )
        except BaseException:
            await self._http.aclose()
            raise
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        await self._http.aclose()

    async def _post(self, method: str, params: dict[str, Any]) -> httpx.Response:
        msg = {"jsonrpc": "2.0", "id": self._next_id, "method": method, "params": params}
        self._next_id += 1
        try:
            r = await self._http.post(self.config.bridge_url, json=msg)
            r.raise_for_status()
        except httpx.HTTPError as exc:
            raise BridgeError(f"{method}: {exc}") from exc
        return r

    async def rpc(self, method: str, params: dict[str, Any]) -> dict[str, Any]:
        body = (await self._post(method, params)).text
        if "data:" in body:  # SSE framing
            body = [line[5:].strip() for line in body.splitlines() if line.startswith("data:")][-1]
        result: dict[str, Any] = json.loads(body)["result"]
        return result

    async def list_tools(self) -> list[dict[str, Any]]:
        tools: list[dict[str, Any]] = (await self.rpc("tools/list", {}))["tools"]
        return tools

    async def call_raw(self, tool: str, args: dict[str, Any]) -> tuple[str, bool]:
        """Call a tool; return (text, is_error) without raising, for relaying to a model."""
        res = await self.rpc("tools/call", {"name": tool, "arguments": args})
        text = "\n".join(
            c.get("text", "") for c in res.get("content", []) if c.get("type") == "text"
        )
        try:
            ok = json.loads(text).get("status") == "success"
        except (json.JSONDecodeError, AttributeError):
            ok = not res.get("isError")
        return text, bool(res.get("isError", False)) or not ok

    async def call(self, tool: str, **args: Any) -> Any:
        """Call a tool and return its `data`; raise BridgeError on any non-success status."""
        text, failed = await self.call_raw(tool, args)
        if failed:
            # A tier refusal comes back as status "disabled", not "error".
            try:
                msg = json.loads(text).get("error", {}).get("message")
            except (json.JSONDecodeError, AttributeError):
                msg = None
            raise BridgeError(f"{tool}: {msg or text[:800]}")
        data = json.loads(text)
        return data.get("data", data)

    async def program_names(self) -> set[str]:
        return {p["name"] for p in (await self.call("program_list"))["programs"]}

    async def download_urpx(self, name: str) -> bytes:
        """Fetch a program as .urpx bytes through the read-only Robot API."""
        url = f"{self.config.robot_api_url.rstrip('/')}/programs/v1/{name}"
        try:
            r = await self._http.get(url, headers={"Accept": "*/*"})
            r.raise_for_status()
        except httpx.HTTPError as exc:
            raise BridgeError(f"download {name}: {exc}") from exc
        return r.content
