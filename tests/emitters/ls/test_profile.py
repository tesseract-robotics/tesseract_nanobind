"""LsProfile and the LS frame binding: validating factories."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import InvalidFrameBindingError, InvalidProfileError
from tesseract_robotics.emitters.ls import LsProfile, ls_frames


def test_defaults_sane() -> None:
    p = LsProfile.build()
    assert p.linear_speed_mms == 100.0 and p.joint_speed_percent == 100.0
    assert p.termination == "FINE"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"joint_speed_percent": 150.0},
        {"linear_speed_mms": -1.0},
        {"termination": ""},
        {"termination": "CNT101"},
        {"termination": "FINE ; L P[9]"},
    ],
)
def test_rejects_out_of_range(kwargs: dict[str, object]) -> None:
    with pytest.raises(InvalidProfileError):
        LsProfile.build(**kwargs)  # type: ignore[arg-type]


def test_is_frozen() -> None:
    with pytest.raises(AttributeError):
        LsProfile.build().linear_speed_mms = 1.0  # type: ignore[misc]


@pytest.mark.parametrize(("uf", "ut"), [({"b": 10}, {}), ({}, {"t": 11}), ({}, {"t": -1})])
def test_frame_numbers_validated(uf: dict[str, int], ut: dict[str, int]) -> None:
    with pytest.raises(InvalidFrameBindingError):
        ls_frames(user_frames=uf, tools=ut)
