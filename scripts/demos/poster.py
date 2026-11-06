"""Capture each recording's poster image from the static client. `pixi run demo-posters`.

Seeks the real player in headless Chrome, so a poster shows what a click on it will play.
"""

from __future__ import annotations

import argparse
import io
import sys
import time
from pathlib import Path

from browser import Browser, BrowserError, Page
from lock import DemoLock, read_lock, sha256_bytes, write_lock
from manifest import Entry, load_manifest, player_query
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
DEMOS_DIR = REPO / "docs/demos"
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
# Settling time for the player after a pause click [s].
PAUSE_SETTLE_S = 0.3
# A static scene has no timeline to say it has loaded: it counts as drawn once two frames this
# far apart [s] are identical, within the budget [s]. Meshes decode after the progress bar goes.
STILL_SETTLE_S = 1.0
STILL_TIMEOUT_S = 30.0

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
    loading: document.querySelector('.mantine-Progress-root') !== null,
  };
})()"""
# With no control bar the scene-tree button floats over the scene, on a tile of its own. Both
# stay off the poster. The first such button is the floating one; the second sits in the tree.
_HIDE_FLOATING_BUTTON = """(() => {
  document.querySelector('button[aria-label="Show scene tree"]').parentElement.style.visibility =
    'hidden';
})()"""


class PosterCaptureError(BrowserError):
    """A poster cannot be made: no timeline, or a recording that never loads."""


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


def require_still(state: dict, module: str) -> None:
    """Check a static entry's recording really has no timeline.

    Raises:
        PosterCaptureError: the recording moves.
    """
    if state["duration_s"] > 0.0:
        raise PosterCaptureError(f"{module}: marked static, yet it plays {state['duration_s']} s")


def player_ready(state: dict | None, *, static: bool) -> bool:
    """Whether the player has its recording: a timeline, or for a static entry no progress bar."""
    if state is None:
        return False
    return not state["loading"] if static else state["duration_s"] > 0.0


def capture(browser: Browser, entry: Entry) -> bytes:
    """Seek `entry`'s recording to the poster frame and capture it.

    Raises:
        PosterCaptureError: the player never appeared, or the entry has no camera.
    """
    if entry.camera is None:
        raise PosterCaptureError(f"{entry.module} has no camera")
    query = player_query(entry.module, entry.camera)
    url = f"http://127.0.0.1:{browser.port}/client/index.html?{query}"
    with browser.page() as page:
        page.call("Page.enable")
        page.call(
            "Emulation.setDeviceMetricsOverride",
            width=PLAYER_WIDTH_PX,
            height=PLAYER_HEIGHT_PX,
            deviceScaleFactor=DEVICE_SCALE,
            mobile=False,
        )
        page.call("Page.navigate", url=url)
        state = _wait_for_player(page, entry)
        if entry.static:
            require_still(state, entry.module)
            return encode_png(_settled_frame(page, entry.module))
        require_timeline(state, entry.module)
        page.click(*state["pause"])
        time.sleep(PAUSE_SETTLE_S)
        x, y, width, height = state["track"]
        # The control bar stays off the poster: the page draws it on click.
        scene_px = round(state["bar_top_px"] * DEVICE_SCALE)
        page.click(x + POSTER_FRACTION * width, y + height / 2)
        frame = page.screenshot()
    return encode_png(frame.crop((0, 0, frame.width, scene_px)))


def _wait_for_player(page: Page, entry: Entry) -> dict:
    deadline = time.monotonic() + LOAD_TIMEOUT_S
    while time.monotonic() < deadline:
        state = page.evaluate(_PLAYER_STATE)
        if player_ready(state, static=entry.static):
            return state
        time.sleep(LOAD_POLL_S)
    raise PosterCaptureError(f"{entry.module}: no recording after {LOAD_TIMEOUT_S} s")


def _settled_frame(page: Page, module: str) -> Image.Image:
    page.evaluate(_HIDE_FLOATING_BUTTON)
    deadline = time.monotonic() + STILL_TIMEOUT_S
    frame = page.screenshot()
    while time.monotonic() < deadline:
        time.sleep(STILL_SETTLE_S)
        previous, frame = frame, page.screenshot()
        if frame.tobytes() == previous.tobytes():
            return frame
    raise PosterCaptureError(f"{module}: still changing after {STILL_TIMEOUT_S} s")


def to_capture(
    modules: list[str], recorded: set[str], posed: set[str], *, only: list[str]
) -> list[str]:
    """The entries to capture: the named ones, or else every recording without a poster.

    A person may have reviewed an existing poster, and a re-record already drops its entry.
    """
    return [m for m in modules if m in recorded and (m in only if only else m not in posed)]


def main(argv: list[str] | None = None) -> int:
    """Capture posters for recorded entries, and add their hashes to the lock."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--only", nargs="*", default=[], help="entries to capture; default those without a poster"
    )
    args = parser.parse_args(argv)
    lock = read_lock()
    posters = dict(lock.posters)
    posters_dir = DEMOS_DIR / "posters"
    posters_dir.mkdir(parents=True, exist_ok=True)
    entries = {entry.module: entry for entry in load_manifest()}
    wanted = to_capture(list(entries), set(lock.recordings), set(posters), only=args.only)
    with Browser(DEMOS_DIR) as browser:
        for module in wanted:
            entry = entries[module]
            data = capture(browser, entry)
            (posters_dir / f"{entry.module}.png").write_bytes(data)
            posters[entry.module] = sha256_bytes(data)
            print(f"{entry.module}: {len(data) / 1e6:.2f} MB")
    write_lock(DemoLock.build(lock.viser_version, lock.client_sha256, lock.recordings, posters))
    return 0


if __name__ == "__main__":
    sys.exit(main())
