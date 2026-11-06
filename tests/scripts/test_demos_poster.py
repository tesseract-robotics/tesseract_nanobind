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
