"""KrlProfile and the KRL frame binding: validating factories."""

from __future__ import annotations

import math

import pytest

from tesseract_robotics.emitters.core.errors import InvalidFrameBindingError, InvalidProfileError
from tesseract_robotics.emitters.krl import KrlProfile, krl_frames


def test_defaults_sane() -> None:
    p = KrlProfile.build()
    assert p.cp_speed_mms == 250.0
    assert 0.0 < p.ptp_speed_percent <= 100.0
    assert p.approximation_mm is None


@pytest.mark.parametrize(
    "kwargs",
    [
        {"ptp_speed_percent": 150.0},
        {"cp_speed_mms": -1.0},
        {"cp_speed_mms": math.nan},
        {"approximation_mm": 0.0},
        {"approximation_mm": math.inf},
    ],
)
def test_rejects_out_of_range(kwargs: dict[str, float]) -> None:
    with pytest.raises(InvalidProfileError):
        KrlProfile.build(**kwargs)


def test_is_frozen() -> None:
    with pytest.raises(AttributeError):
        KrlProfile.build().cp_speed_mms = 1.0  # type: ignore[misc]


@pytest.mark.parametrize(("bases", "tools"), [({"b": 33}, {}), ({}, {"t": 0}), ({}, {"t": 17})])
def test_frame_indices_validated(bases: dict[str, int], tools: dict[str, int]) -> None:
    with pytest.raises(InvalidFrameBindingError):
        krl_frames(bases=bases, tools=tools)
