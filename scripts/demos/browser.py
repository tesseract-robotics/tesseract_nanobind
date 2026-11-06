"""Headless Chrome over the DevTools protocol, with a static file server beside it."""

from __future__ import annotations

import base64
import functools
import http.server
import io
import json
import socket
import subprocess
import tempfile
import threading
import time
import urllib.request
from collections.abc import Generator
from contextlib import contextmanager
from pathlib import Path
from types import TracebackType
from typing import Any

from PIL import Image
from websockets.sync.client import connect
from websockets.sync.connection import Connection

REPO = Path(__file__).resolve().parents[2]
CHROME = Path("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome")
CHROME_START_TIMEOUT_S = 15.0
START_POLL_S = 0.25
# How long one wait for a browser event blocks before looking at the clock again [s].
PUMP_POLL_S = 0.1
# Pending connections the file server holds [-]. A docs page asks for its posters at once; 5,
# the http.server default, dropped 12 requests in 30 loads of planning.md, and this dropped 0.
LISTEN_BACKLOG = 128
_NEXT_PAINT = "new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))"


def scratch_dir() -> Path:
    """The gitignored working directory, which a fresh clone lacks."""
    path = REPO / ".scratch"
    path.mkdir(exist_ok=True)
    return path


class BrowserError(RuntimeError):
    """Chrome is missing, did not start, or refused a command."""


class StaticServer(http.server.ThreadingHTTPServer):
    """A local file server that keeps up with a whole page's requests."""

    request_queue_size = LISTEN_BACKLOG


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:  # noqa: A002 (the base's name)
        pass


@contextmanager
def serve(root: Path) -> Generator[int]:
    """Serve `root` for the context; yields the port."""
    handler = functools.partial(_QuietHandler, directory=str(root))
    with StaticServer(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield server.server_address[1]
        finally:
            server.shutdown()
            thread.join()


class Page:
    """One Chrome tab. Events that arrive between replies collect in `events`."""

    def __init__(self, socket_: Connection) -> None:
        self._socket = socket_
        self._next_id = 0
        self.events: list[dict] = []

    def call(self, method: str, **params: object) -> dict:
        self._next_id += 1
        self._socket.send(json.dumps({"id": self._next_id, "method": method, "params": params}))
        while True:
            reply = json.loads(self._socket.recv())
            if reply.get("id") == self._next_id:
                if "error" in reply:
                    raise BrowserError(f"{method}: {reply['error']}")
                return reply["result"]
            self.events.append(reply)

    def evaluate(self, expression: str, *, wait: bool = False) -> Any:  # What the page returned.
        result = self.call(
            "Runtime.evaluate", expression=expression, returnByValue=True, awaitPromise=wait
        )
        if "exceptionDetails" in result:
            raise BrowserError(f"page script failed: {result['exceptionDetails']}")
        return result["result"].get("value")

    def pump(self, seconds: float) -> None:
        """Collect events for `seconds`."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                self.events.append(json.loads(self._socket.recv(timeout=PUMP_POLL_S)))
            except TimeoutError:
                continue

    def click(self, x: float, y: float) -> None:
        self.call("Input.dispatchMouseEvent", type="mouseMoved", x=x, y=y)
        for kind in ("mousePressed", "mouseReleased"):
            self.call("Input.dispatchMouseEvent", type=kind, x=x, y=y, button="left", clickCount=1)

    def screenshot(self) -> Image.Image:
        self.evaluate(_NEXT_PAINT, wait=True)
        data = self.call("Page.captureScreenshot", format="png")["data"]
        return Image.open(io.BytesIO(base64.b64decode(data))).convert("RGB")


class Browser:
    """A static file server over `root` and headless Chrome, for as long as the context lasts."""

    def __init__(self, root: Path) -> None:
        if not CHROME.exists():
            raise BrowserError(f"Chrome not found at {CHROME}")
        self._root = root

    def __enter__(self) -> Browser:
        self._serving = serve(self._root)
        self.port = self._serving.__enter__()
        self._cdp_port = _free_port()
        self._profile = tempfile.TemporaryDirectory(dir=scratch_dir(), prefix="chrome-")
        self._chrome = subprocess.Popen(
            [
                str(CHROME),
                "--headless=new",
                "--no-first-run",
                "--use-angle=metal",
                "--enable-gpu",
                "--ignore-gpu-blocklist",
                f"--remote-debugging-port={self._cdp_port}",
                f"--user-data-dir={self._profile.name}",
                "about:blank",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._chrome.terminate()
        self._chrome.wait()
        self._serving.__exit__(exc_type, exc, tb)
        self._profile.cleanup()

    @contextmanager
    def page(self) -> Generator[Page]:
        """The tab, for one connection."""
        with connect(self._page_socket(), max_size=None) as socket_:
            yield Page(socket_)

    def _page_socket(self) -> str:
        deadline = time.monotonic() + CHROME_START_TIMEOUT_S
        url = f"http://127.0.0.1:{self._cdp_port}/json"
        while True:
            try:
                with urllib.request.urlopen(url) as reply:
                    targets = json.load(reply)
                return next(t for t in targets if t["type"] == "page")["webSocketDebuggerUrl"]
            except OSError as error:
                if time.monotonic() > deadline:
                    raise BrowserError(f"Chrome did not start: {error}") from error
                time.sleep(START_POLL_S)


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]
