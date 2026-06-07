"""LsProfile: frozen motion params with a validating .build() factory."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.ls.profile import LsProfile


def test_defaults_sane() -> None:
    p = LsProfile.build()
    assert p.linear_speed_mms == 100.0
    assert 0.0 < p.joint_speed_percent <= 100.0
    assert p.termination == "FINE"


def test_rejects_out_of_range_percent() -> None:
    with pytest.raises(ValueError):
        LsProfile.build(joint_speed_percent=150.0)


def test_rejects_negative_speed() -> None:
    with pytest.raises(ValueError):
        LsProfile.build(linear_speed_mms=-1.0)


def test_rejects_empty_termination() -> None:
    with pytest.raises(ValueError):
        LsProfile.build(termination="")


def test_is_frozen() -> None:
    with pytest.raises(Exception):
        LsProfile.build().linear_speed_mms = 1.0  # type: ignore[misc]
