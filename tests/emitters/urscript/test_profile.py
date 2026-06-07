"""UrScriptProfile: frozen motion params with a validating .build() factory."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.urscript.profile import UrScriptProfile


def test_defaults_sane() -> None:
    p = UrScriptProfile.build()
    assert p.linear_speed_ms == 0.25
    assert p.linear_accel_mss == 1.2
    assert p.joint_speed_rads == 1.05
    assert p.joint_accel_radss == 1.4
    assert p.blend_radius_m == 0.0


def test_rejects_non_positive_linear_speed() -> None:
    with pytest.raises(ValueError):
        UrScriptProfile.build(linear_speed_ms=0.0)


def test_rejects_non_positive_joint_accel() -> None:
    with pytest.raises(ValueError):
        UrScriptProfile.build(joint_accel_radss=-1.0)


def test_rejects_negative_blend() -> None:
    with pytest.raises(ValueError):
        UrScriptProfile.build(blend_radius_m=-0.01)


def test_is_frozen() -> None:
    with pytest.raises(Exception):
        UrScriptProfile.build().linear_speed_ms = 1.0  # type: ignore[misc]
