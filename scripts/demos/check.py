"""Drive every player in the built site. `pixi run demo-check`.

Loads each page in headless Chrome, clicks the player, and checks it plays, pauses, scrubs and
orbits with no failed request for a demo file. Serves `site/` under the Pages subpath, or
checks the deployed site with `--url`.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from browser import Browser, Page
from manifest import Entry, Status, load_manifest
from numpy.typing import NDArray
from PIL import Image
from players import HERO_MODULE

REPO = Path(__file__).resolve().parents[2]
SITE_DIR = REPO / "site"
# The Pages project path: a root-relative URL that works locally but not deployed shows here.
SUBPATH = "tesseract_nanobind"
DEMO_URL_PART = "/demos/"
VIEWPORT_WIDTH_PX = 1400
VIEWPORT_HEIGHT_PX = 900
HTTP_ERROR = 400
# How long a recording gets to download and decode [s].
LOAD_TIMEOUT_S = 90.0
LOAD_POLL_S = 0.25
# Watch this long for the timeline to move, or to stay put when paused [s].
PLAY_WATCH_S = 1.5
SETTLE_S = 1.0
# Where on the timeline the scrub lands [-], and how far off it may land: one timeline tick
# rounds to the recording's frames, so 5 % of the track is generous without hiding a dead click.
SCRUB_FRACTION = 0.7
SCRUB_ATOL = 0.05
# A channel difference below this is noise, not a change [8-bit levels].
PIXEL_NOISE_LEVELS = 8
# A drag across the scene moves at least this share of the player's pixels [-]; car_seat
# measured 15 %, and the emptiest scene is well above this.
MIN_ORBIT_CHANGE = 0.01
# A paused or static scene left alone changes at most this share [-]: measured 0.
MAX_IDLE_CHANGE = 0.001
ORBIT_DRAG_X_PX = 220.0
ORBIT_DRAG_Y_PX = 40.0
ORBIT_STEPS = 12
ORBIT_STEP_S = 0.03
WHEEL_DELTA_PX = 300.0

_PLAYER_BUTTON = """(() => {{
  const e = document.querySelector('button.demo-player[data-src*="recordings/{module}.viser"]');
  if (e === null) return null;
  e.scrollIntoView({{block: 'center'}});
  const r = e.getBoundingClientRect();
  return [r.x + r.width / 2, r.y + r.height / 2];
}})()"""
_IFRAME = "document.querySelector('iframe.demo-player')"
_IFRAME_RECT = f"""(() => {{
  const r = {_IFRAME}.getBoundingClientRect();
  return [r.x, r.y, r.width, r.height];
}})()"""
_PLAYER_STATE = f"""(() => {{
  const frame = {_IFRAME};
  if (frame === null || frame.contentDocument === null) return null;
  const doc = frame.contentDocument;
  const handle = doc.querySelector('[role=slider]');
  if (handle === null || doc.querySelector('.mantine-Progress-root') !== null) return null;
  const root = handle.closest('.mantine-Slider-root');
  const track = root.getBoundingClientRect();
  let bar = root;
  while (bar !== null && bar.querySelector('button') === null) bar = bar.parentElement;
  const button = bar.querySelector('button').getBoundingClientRect();
  return {{
    now_s: Number(handle.getAttribute('aria-valuenow')),
    duration_s: Number(handle.getAttribute('aria-valuemax')),
    track: [track.x, track.y, track.width, track.height],
    pause: [button.x + button.width / 2, button.y + button.height / 2],
  }};
}})()"""


class PlayerCheckError(RuntimeError):
    """A player failed a check; the message names which."""


@dataclass
class Report:
    """What one player did."""

    label: str
    failures: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def page_url(page: str) -> str:
    """A docs source path as its URL path, relative to the site root."""
    return page.removesuffix(".md") + "/"


def failed_demo_requests(events: list[dict]) -> list[str]:
    """Every demo file the browser failed to fetch, from its Network events."""
    urls: dict[str, str] = {}
    failed: list[str] = []
    for event in events:
        method, params = event.get("method"), event.get("params", {})
        if method == "Network.requestWillBeSent":
            urls[params["requestId"]] = params["request"]["url"]
        elif method == "Network.responseReceived":
            response = params["response"]
            if response["status"] >= HTTP_ERROR and DEMO_URL_PART in response["url"]:
                failed.append(f"{response['status']} {response['url']}")
        elif method == "Network.loadingFailed":
            url = urls.get(params["requestId"], "")
            if DEMO_URL_PART in url:
                failed.append(f"{params['errorText']} {url}")
    return failed


def changed_fraction(before: NDArray[np.uint8], after: NDArray[np.uint8]) -> float:
    """The share of pixels where any channel moved by more than noise."""
    difference = np.abs(before.astype(np.int16) - after.astype(np.int16))
    return float((difference > PIXEL_NOISE_LEVELS).any(axis=-1).mean())


def _player_pixels(page: Page, rect: list[float]) -> NDArray[np.uint8]:
    x, y, width, height = rect
    frame = page.screenshot().crop((int(x), int(y), int(x + width), int(y + height)))
    return np.asarray(Image.Image.convert(frame, "RGB"), dtype=np.uint8)


def _drag(page: Page, x0: float, y0: float) -> None:
    x1, y1 = x0 + ORBIT_DRAG_X_PX, y0 + ORBIT_DRAG_Y_PX
    mouse = "Input.dispatchMouseEvent"
    page.call(mouse, type="mouseMoved", x=x0, y=y0)
    page.call(mouse, type="mousePressed", x=x0, y=y0, button="left", clickCount=1, buttons=1)
    for step in range(1, ORBIT_STEPS + 1):
        t = step / ORBIT_STEPS
        page.call(
            mouse,
            type="mouseMoved",
            x=x0 + (x1 - x0) * t,
            y=y0 + (y1 - y0) * t,
            button="left",
            buttons=1,
        )
        page.pump(ORBIT_STEP_S)
    page.call(mouse, type="mouseReleased", x=x1, y=y1, button="left", clickCount=1)


def _wait_for_state(page: Page) -> dict:
    deadline = time.monotonic() + LOAD_TIMEOUT_S
    while time.monotonic() < deadline:
        state = page.evaluate(_PLAYER_STATE)
        if isinstance(state, dict):
            return state
        page.pump(LOAD_POLL_S)
    raise PlayerCheckError(f"no player after {LOAD_TIMEOUT_S} s")


def _check_timeline(page: Page, rect: list[float], state: dict, report: Report) -> None:
    first = page.evaluate(_PLAYER_STATE)
    page.pump(PLAY_WATCH_S)
    second = page.evaluate(_PLAYER_STATE)
    if second["now_s"] == first["now_s"]:
        report.failures.append(f"does not play: stuck at {first['now_s']} s")
    page.click(rect[0] + state["pause"][0], rect[1] + state["pause"][1])
    page.pump(SETTLE_S)
    paused = page.evaluate(_PLAYER_STATE)["now_s"]
    page.pump(PLAY_WATCH_S)
    if page.evaluate(_PLAYER_STATE)["now_s"] != paused:
        report.failures.append("does not pause")
    x, y, width, height = state["track"]
    page.click(rect[0] + x + SCRUB_FRACTION * width, rect[1] + y + height / 2)
    page.pump(SETTLE_S)
    landed = page.evaluate(_PLAYER_STATE)["now_s"] / state["duration_s"]
    if abs(landed - SCRUB_FRACTION) > SCRUB_ATOL:
        report.failures.append(f"scrub to {SCRUB_FRACTION} landed at {landed:.3f}")


def _check_player(
    page: Page, base: str, path: str, module: str, static: bool, report: Report
) -> None:
    page.events.clear()
    page.call("Page.navigate", url=base + path)
    page.pump(SETTLE_S)
    button = page.evaluate(_PLAYER_BUTTON.format(module=module))
    if button is None:
        raise PlayerCheckError(f"no player for {module} on {path or '/'}")
    page.pump(SETTLE_S)
    page.click(button[0], button[1])
    state = _wait_for_state(page)
    report.notes.append(f"{state['duration_s']:.1f} s")
    page.pump(SETTLE_S)
    rect = page.evaluate(_IFRAME_RECT)
    if static:
        if state["duration_s"] > 0.0:
            report.failures.append(f"marked static, plays {state['duration_s']} s")
    elif state["duration_s"] <= 0.0:
        report.failures.append("has no timeline")
    else:
        _check_timeline(page, rect, state, report)
    still = _player_pixels(page, rect)
    page.pump(SETTLE_S)
    idle = changed_fraction(still, _player_pixels(page, rect))
    if idle > MAX_IDLE_CHANGE:
        report.failures.append(f"moves when left alone: {idle:.2%}")
    # Above the middle: the control bar sits below it.
    _drag(page, rect[0] + rect[2] / 2, rect[1] + rect[3] * 0.45)
    page.pump(SETTLE_S)
    orbited = _player_pixels(page, rect)
    orbit = changed_fraction(still, orbited)
    report.notes.append(f"orbit {orbit:.1%}")
    if orbit < MIN_ORBIT_CHANGE:
        report.failures.append(f"does not orbit: {orbit:.2%}")
    scroll_before = page.evaluate("window.scrollY")
    x, y = rect[0] + rect[2] / 2, rect[1] + rect[3] * 0.45
    page.call(
        "Input.dispatchMouseEvent", type="mouseWheel", x=x, y=y, deltaX=0, deltaY=WHEEL_DELTA_PX
    )
    page.pump(SETTLE_S)
    report.notes.append(
        f"wheel: page scroll {page.evaluate('window.scrollY') - scroll_before:+.0f} px, "
        f"scene {changed_fraction(orbited, _player_pixels(page, rect)):.1%}"
    )
    report.failures.extend(failed_demo_requests(page.events))
    report.failures.extend(
        f"exception: {event['params']['exceptionDetails']['text']}"
        for event in page.events
        if event.get("method") == "Runtime.exceptionThrown"
    )


def _targets(entries: list[Entry], *, preview: bool) -> list[tuple[str, str, str, bool]]:
    shown = [e for e in entries if preview or e.status is Status.PUBLISHED]
    players = [(e.module, page_url(e.page), e.module, e.static) for e in shown]
    hero = next((e for e in shown if e.module == HERO_MODULE), None)
    return players + ([("home", "", HERO_MODULE, False)] if hero else [])


def main(argv: list[str] | None = None) -> int:
    """Check every player; exit 1 if any fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", help="check this deployed site root, not a local site/")
    parser.add_argument("--preview", action="store_true", help="include entries awaiting review")
    parser.add_argument("--only", nargs="*", default=[], help="labels to check; default all")
    args = parser.parse_args(argv)
    targets = [
        t
        for t in _targets(load_manifest(), preview=args.preview)
        if not args.only or t[0] in args.only
    ]
    reports = []
    with tempfile.TemporaryDirectory(dir=REPO / ".scratch", prefix="serve-") as root:
        (Path(root) / SUBPATH).symlink_to(SITE_DIR)
        with Browser(Path(root)) as browser, browser.page() as page:
            base = args.url or f"http://127.0.0.1:{browser.port}/{SUBPATH}/"
            for method in ("Page.enable", "Network.enable", "Runtime.enable"):
                page.call(method)
            page.call(
                "Emulation.setDeviceMetricsOverride",
                width=VIEWPORT_WIDTH_PX,
                height=VIEWPORT_HEIGHT_PX,
                deviceScaleFactor=1.0,
                mobile=False,
            )
            for label, path, module, static in targets:
                report = Report(label)
                try:
                    _check_player(page, base, path, module, static, report)
                except PlayerCheckError as error:
                    report.failures.append(str(error))
                verdict = "FAIL" if report.failures else "ok"
                print(
                    f"{verdict:4} {label}: {'; '.join(report.notes + report.failures)}", flush=True
                )
                reports.append(report)
    return 1 if any(report.failures for report in reports) else 0


if __name__ == "__main__":
    sys.exit(main())
