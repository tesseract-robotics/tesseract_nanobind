"""JbiProfile: frozen motion params with a validating .build() factory."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.jbi.profile import JbiProfile


def test_defaults_sane() -> None:
    p = JbiProfile.build()
    assert p.linear_speed_mms == 100.0
    assert 0.0 < p.joint_speed_percent <= 100.0
    assert p.tool == 0


def test_rejects_out_of_range_percent() -> None:
    with pytest.raises(ValueError):
        JbiProfile.build(joint_speed_percent=150.0)


def test_rejects_zero_percent() -> None:
    with pytest.raises(ValueError):
        JbiProfile.build(joint_speed_percent=0.0)


def test_rejects_negative_speed() -> None:
    with pytest.raises(ValueError):
        JbiProfile.build(linear_speed_mms=-1.0)


def test_rejects_negative_tool() -> None:
    with pytest.raises(ValueError):
        JbiProfile.build(tool=-1)


def test_is_frozen() -> None:
    with pytest.raises(Exception):
        JbiProfile.build().linear_speed_mms = 1.0  # type: ignore[misc]
