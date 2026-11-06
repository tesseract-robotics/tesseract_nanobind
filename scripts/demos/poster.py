"""Capture each recording's poster image from the static client. `pixi run demo-posters`.

Seeks the real player in headless Chrome, so a poster shows what a click on it will play.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path
from types import TracebackType

from lock import DemoLock, read_lock, sha256_bytes, write_lock
from manifest import Entry, load_manifest, player_query
from PIL import Image
from websockets.sync.client import ClientConnection, connect

REPO = Path(__file__).resolve().parents[2]
DEMOS_DIR = REPO / "docs/demos"
CHROME = Path("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome")
# Player size in the docs [CSS px]: the content column at 16:9, as the page's CSS lays it out.
PLAYER_WIDTH_PX = 688
PLAYER_HEIGHT_PX = 387
# The player's width on a 3840x2160 screen at 2x [px], measured: the box is 757 CSS px there.
LARGEST_BOX_PHYSICAL_PX = 1514
# Render density [-]: enough for that box, so a poster is never stretched next to the sharp
# live player, which renders at the screen's own density.
DEVICE_SCALE = 3.0
# Where on the timeline the poster is taken [-]: late enough that the motion has happened.
POSTER_FRACTION = 0.7
# How long the client gets to download and decode a recording [s].
LOAD_TIMEOUT_S = 90.0
LOAD_POLL_S = 0.25
CHROME_START_TIMEOUT_S = 15.0
# Settling time for the player after a pause click [s].
PAUSE_SETTLE_S = 0.3

_PLAYER_STATE = """(() => {
  const handle = document.querySelector('[role=slider]');
  if (handle === null) return null;
  const track = handle.closest('.mantine-Slider-root').getBoundingClientRect();
  let bar = handle.closest('.mantine-Slider-root');
  while (bar !== null && bar.querySelector('button') === null) bar = bar.parentElement;
  const button = bar.querySelector('button').getBoundingClientRect();
  const panel = bar.getBoundingClientRect();
  return {
    duration_s: Number(handle.getAttribute('aria-valuemax')),
    track: [track.x, track.y, track.width, track.height],
    pause: [button.x + button.width / 2, button.y + button.height / 2],
    bar_top_px: panel.y,
  };
})()"""
_NEXT_PAINT = "new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))"


class PosterCaptureError(RuntimeError):
    """A poster cannot be made: no client, or no timeline."""


def encode_png(frame: Image.Image) -> bytes:
    """Encode the poster frame."""
    out = io.BytesIO()
    frame.save(out, format="PNG", optimize=True)
    return out.getvalue()


def require_timeline(state: dict, module: str) -> None:
    """Check the player reports a timeline to seek along.

    Raises:
        PosterCaptureError: the timeline has no duration.
    """
    if not state["duration_s"] > 0.0:
        raise PosterCaptureError(f"{module}: nothing to seek along")


class _Page:
    """One Chrome tab over the DevTools protocol."""

    def __init__(self, socket_: ClientConnection) -> None:
        self._socket = socket_
        self._next_id = 0

    def call(self, method: str, **params: object) -> dict:
        self._next_id += 1
        self._socket.send(json.dumps({"id": self._next_id, "method": method, "params": params}))
        while True:
            reply = json.loads(self._socket.recv())
            if reply.get("id") == self._next_id:
                if "error" in reply:
                    raise PosterCaptureError(f"{method}: {reply['error']}")
                return reply["result"]

    def evaluate(self, expression: str, *, wait: bool = False) -> object:
        result = self.call(
            "Runtime.evaluate", expression=expression, returnByValue=True, awaitPromise=wait
        )
        if "exceptionDetails" in result:
            raise PosterCaptureError(f"page script failed: {result['exceptionDetails']}")
        return result["result"].get("value")

    def click(self, x: float, y: float) -> None:
        self.call("Input.dispatchMouseEvent", type="mouseMoved", x=x, y=y)
        for kind in ("mousePressed", "mouseReleased"):
            self.call("Input.dispatchMouseEvent", type=kind, x=x, y=y, button="left", clickCount=1)

    def screenshot(self) -> Image.Image:
        self.evaluate(_NEXT_PAINT, wait=True)
        data = self.call("Page.captureScreenshot", format="png")["data"]
        return Image.open(io.BytesIO(base64.b64decode(data))).convert("RGB")


class _Browser:
    """A static file server and headless Chrome, for as long as the context lasts."""

    def __init__(self, root: Path) -> None:
        if not CHROME.exists():
            raise PosterCaptureError(f"Chrome not found at {CHROME}")
        self._root = root

    def __enter__(self) -> _Browser:
        self._port, self._cdp_port = _free_port(), _free_port()
        self._profile = tempfile.TemporaryDirectory(dir=REPO / ".scratch", prefix="poster-")
        self._server = subprocess.Popen(
            [sys.executable, "-m", "http.server", str(self._port), "--bind", "127.0.0.1"],
            cwd=self._root,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
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
        self._server.terminate()
        self._chrome.wait()
        self._server.wait()
        self._profile.cleanup()

    def capture(self, entry: Entry) -> bytes:
        """Seek `entry`'s recording to the poster frame and capture it.

        Raises:
            PosterCaptureError: the player never appeared, or the entry has no camera.
        """
        if entry.camera is None:
            raise PosterCaptureError(f"{entry.module} has no camera")
        query = player_query(entry.module, entry.camera)
        url = f"http://127.0.0.1:{self._port}/client/index.html?{query}"
        with connect(self._page_socket(), max_size=None) as socket_:
            page = _Page(socket_)
            page.call("Page.enable")
            page.call(
                "Emulation.setDeviceMetricsOverride",
                width=PLAYER_WIDTH_PX,
                height=PLAYER_HEIGHT_PX,
                deviceScaleFactor=DEVICE_SCALE,
                mobile=False,
            )
            page.call("Page.navigate", url=url)
            state = _wait_for_player(page, entry.module)
            require_timeline(state, entry.module)
            page.click(*state["pause"])
            time.sleep(PAUSE_SETTLE_S)
            x, y, width, height = state["track"]
            # The control bar stays off the poster: the page draws it on click.
            scene_px = round(state["bar_top_px"] * DEVICE_SCALE)
            page.click(x + POSTER_FRACTION * width, y + height / 2)
            frame = page.screenshot()
        return encode_png(frame.crop((0, 0, frame.width, scene_px)))

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
                    raise PosterCaptureError(f"Chrome did not start: {error}") from error
                time.sleep(LOAD_POLL_S)


def _wait_for_player(page: _Page, module: str) -> dict:
    deadline = time.monotonic() + LOAD_TIMEOUT_S
    while time.monotonic() < deadline:
        state = page.evaluate(_PLAYER_STATE)
        if isinstance(state, dict) and state["duration_s"] > 0.0:
            return state
        time.sleep(LOAD_POLL_S)
    raise PosterCaptureError(f"{module}: no timeline after {LOAD_TIMEOUT_S} s")


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main(argv: list[str] | None = None) -> int:
    """Capture posters for recorded entries, and add their hashes to the lock."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="*", default=[], help="entries to capture; default all")
    args = parser.parse_args(argv)
    lock = read_lock()
    posters = dict(lock.posters)
    posters_dir = DEMOS_DIR / "posters"
    posters_dir.mkdir(parents=True, exist_ok=True)
    with _Browser(DEMOS_DIR) as browser:
        for entry in load_manifest():
            if entry.module not in lock.recordings or (args.only and entry.module not in args.only):
                continue
            data = browser.capture(entry)
            (posters_dir / f"{entry.module}.png").write_bytes(data)
            posters[entry.module] = sha256_bytes(data)
            print(f"{entry.module}: {len(data) / 1e6:.2f} MB")
    write_lock(DemoLock.build(lock.viser_version, lock.client_sha256, lock.recordings, posters))
    return 0


if __name__ == "__main__":
    sys.exit(main())
