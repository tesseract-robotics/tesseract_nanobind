"""The generated gallery page: players only for what a person reviewed, reasons for the rest."""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/gallery"))

import lock  # noqa: E402
import manifest  # noqa: E402
import page  # noqa: E402

CAMERA = manifest.Camera((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), 0.6)
SHOWN = manifest.Entry("shown", "Shown", "It plays.", manifest.Status.PUBLISHED, None, CAMERA, None)
IN_REVIEW = manifest.Entry(
    "in_review", "In review", "Blurb.", manifest.Status.EXCLUDED, "Awaiting review.", CAMERA, None
)
LEFT_OUT = manifest.Entry(
    "left_out", "Left out", "Blurb.", manifest.Status.EXCLUDED, "Looks wrong.", None, None
)
ENTRIES = [SHOWN, IN_REVIEW, LEFT_OUT]
LOCK = lock.GalleryLock.build(
    "1.1.1",
    "client",
    {
        module: lock.RecordingLock(lock.BuildIdentity("0.35", "1.1.1", "abc"), "d", 2_000_000)
        for module in ("shown", "in_review")
    },
)


def test_a_published_entry_gets_a_click_to_load_player_with_its_identity():
    text = page.render(ENTRIES, LOCK)
    assert 'data-src="../../demos/client/index.html?playbackPath=../recordings/shown.viser' in text
    assert "0.35" in text
    assert "viser 1.1.1" in text
    assert "<iframe" not in text


def test_an_excluded_entry_gets_no_player_and_states_its_reason():
    text = page.render(ENTRIES, LOCK)
    assert "recordings/in_review.viser" not in text
    assert "recordings/left_out.viser" not in text
    assert "Looks wrong." in text
    assert "Awaiting review." in text


def test_a_preview_shows_what_is_recorded_and_awaiting_review_and_says_so():
    text = page.render(ENTRIES, LOCK, preview=True)
    assert "recordings/in_review.viser" in text
    assert "Awaiting review." in text
    assert "recordings/left_out.viser" not in text


def test_a_published_entry_without_a_recording_fails_loudly():
    empty = lock.GalleryLock.build("1.1.1", "client", {})
    try:
        page.render(ENTRIES, empty)
    except page.MissingRecordingError as error:
        assert "shown" in str(error)
    else:
        raise AssertionError("expected MissingRecordingError")
