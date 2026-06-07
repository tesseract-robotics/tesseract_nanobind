"""KrlProfile: frozen motion params with a validating .build() factory."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.krl.profile import KrlProfile


def test_defaults_sane() -> None:
    p = KrlProfile.build()
    assert p.cp_speed_mms == 250.0
    assert 0.0 < p.ptp_speed_percent <= 100.0


def test_rejects_out_of_range_ptp() -> None:
    with pytest.raises(ValueError):
        KrlProfile.build(ptp_speed_percent=150.0)


def test_rejects_negative_speed() -> None:
    with pytest.raises(ValueError):
        KrlProfile.build(cp_speed_mms=-1.0)


def test_is_frozen() -> None:
    with pytest.raises(Exception):
        KrlProfile.build().cp_speed_mms = 1.0  # type: ignore[misc]
