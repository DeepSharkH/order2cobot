"""Minimal MCP (Streamable HTTP) client for the UR AI Agent Bridge URCap."""
import json
import os

import requests
import urllib3

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

DEFAULT_URL = os.environ.get(
    "UR_BRIDGE_URL", "https://192.168.2.12/universal-robots/ai-agent-bridge/mcp/v1/")


class BridgeError(RuntimeError):
    pass


class URBridge:
    def __init__(self, url=DEFAULT_URL, client_name="order2cobot-calib", timeout=30):
        self.url = url
        self.timeout = timeout
        self.s = requests.Session()
        self.s.verify = False  # robot uses a self-signed certificate
        self.s.headers.update({"Content-Type": "application/json",
                               "Accept": "application/json, text/event-stream"})
        self._id = 0
        self.client_name = client_name
        self.initialize()

    def _post(self, method, params=None, notify=False):
        msg = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            msg["params"] = params
        if not notify:
            self._id += 1
            msg["id"] = self._id
        r = self.s.post(self.url, data=json.dumps(msg), timeout=self.timeout)
        if "Mcp-Session-Id" in r.headers:
            self.s.headers["Mcp-Session-Id"] = r.headers["Mcp-Session-Id"]
        r.raise_for_status()
        if notify or not r.content:
            return None
        if r.headers.get("Content-Type", "").startswith("text/event-stream"):
            data = [l[5:].strip() for l in r.text.splitlines() if l.startswith("data:")]
            resp = json.loads(data[-1])
        else:
            resp = r.json()
        if "error" in resp:
            raise BridgeError(resp["error"])
        return resp["result"]

    def initialize(self):
        self.s.headers.pop("Mcp-Session-Id", None)
        res = self._post("initialize", {
            "protocolVersion": "2025-06-18", "capabilities": {},
            "clientInfo": {"name": self.client_name, "version": "0.1"}})
        self._post("notifications/initialized", notify=True)
        return res

    def list_tools(self):
        return self._post("tools/list", {})["tools"]

    def call(self, name, **args):
        """Call a tool; returns parsed JSON content if possible, else text."""
        res = self._post("tools/call", {"name": name, "arguments": args})
        if res.get("structuredContent") is not None:
            out = res["structuredContent"]
        else:
            texts = [c.get("text", "") for c in res.get("content", []) if c.get("type") == "text"]
            txt = "\n".join(texts)
            try:
                out = json.loads(txt)
            except ValueError:
                out = txt
        if res.get("isError"):
            raise BridgeError(out)
        return out
