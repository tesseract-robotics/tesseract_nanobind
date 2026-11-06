"""Poster frames: where they fall on the timeline, and the file they make."""

import io
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import poster  # noqa: E402

# Tolerance on a timeline fraction [-]; float64 round-off.
FRACTION_ATOL = 1e-12


def test_frames_span_the_whole_timeline_at_the_poster_rate():
    fractions = poster.frame_fractions(10.0)
    # 10 s at POSTER_SPEEDUP times speed, POSTER_FPS_HZ frames per second, and the end.
    assert len(fractions) == round(10.0 / poster.POSTER_SPEEDUP * poster.POSTER_FPS_HZ) + 1
    assert fractions[0] == 0.0
    assert fractions[-1] == pytest.approx(1.0, abs=FRACTION_ATOL)
    assert list(fractions) == sorted(fractions)


def test_a_very_short_timeline_still_has_a_start_and_an_end():
    assert list(poster.frame_fractions(0.01)) == [0.0, 1.0]


def test_a_timeline_without_duration_raises():
    with pytest.raises(poster.PosterCaptureError):
        poster.frame_fractions(0.0)


def test_frames_encode_to_a_looping_animated_webp():
    frames = [Image.fromarray(np.full((20, 30, 3), value, dtype=np.uint8)) for value in (0, 255)]
    with Image.open(io.BytesIO(poster.encode_webp(frames))) as image:
        assert image.format == "WEBP"
        assert image.is_animated
        assert image.n_frames == len(frames)
        assert image.size == (30, 20)
        assert image.info["loop"] == 0
        image.seek(1)
        image.load()
        assert image.info["duration"] == round(1000.0 / poster.POSTER_FPS_HZ)


def test_no_frames_raises():
    with pytest.raises(poster.PosterCaptureError):
        poster.encode_webp([])
