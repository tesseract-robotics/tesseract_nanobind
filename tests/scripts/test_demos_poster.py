"""The poster: one still image from the recording."""

import io
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import poster  # noqa: E402


def test_the_poster_frame_is_inside_the_timeline():
    assert 0.0 < poster.POSTER_FRACTION < 1.0


def test_a_frame_encodes_to_a_still_png():
    frame = Image.fromarray(np.full((20, 30, 3), 128, dtype=np.uint8))
    with Image.open(io.BytesIO(poster.encode_png(frame))) as image:
        assert image.format == "PNG"
        assert image.size == (30, 20)
        assert not getattr(image, "is_animated", False)


def test_the_player_state_without_a_timeline_raises():
    with pytest.raises(poster.PosterCaptureError):
        poster.require_timeline({"duration_s": 0.0}, "m")


def test_the_poster_is_not_stretched_on_a_4k_screen():
    assert poster.PLAYER_WIDTH_PX * poster.DEVICE_SCALE >= poster.LARGEST_BOX_PHYSICAL_PX


def test_a_still_with_a_timeline_raises():
    # A manifest entry says static, yet the recording moves: one of the two is wrong.
    with pytest.raises(poster.PosterCaptureError):
        poster.require_still({"duration_s": 2.0}, "m")


@pytest.mark.parametrize(
    ("state", "static", "ready"),
    [
        (None, False, False),
        (None, True, False),
        ({"duration_s": 0.0, "loading": False}, False, False),
        ({"duration_s": 3.0, "loading": False}, False, True),
        ({"duration_s": 0.0, "loading": True}, True, False),
        ({"duration_s": 0.0, "loading": False}, True, True),
    ],
)
def test_when_the_player_counts_as_loaded(state, static, ready):
    assert poster.player_ready(state, static=static) is ready


def test_a_bare_run_leaves_reviewed_posters_alone():
    recorded, posed = {"new", "done"}, {"done"}
    assert poster.to_capture(["new", "done", "unrecorded"], recorded, posed, only=[]) == ["new"]


def test_naming_an_entry_recaptures_it():
    assert poster.to_capture(["new", "done"], {"new", "done"}, {"done"}, only=["done"]) == ["done"]
