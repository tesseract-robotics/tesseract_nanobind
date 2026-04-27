"""Tests for ConstantTCPSpeedParameterization + composite profile bindings."""

import pytest

from tesseract.tesseract_time_parameterization import (
    ConstantTCPSpeedCompositeProfile,
    ConstantTCPSpeedParameterization,
    TimeParameterization,
)


class TestConstantTCPSpeedParameterization:
    def test_default_constructor(self):
        tp = ConstantTCPSpeedParameterization()
        assert tp is not None
        assert tp.getName() == "ConstantTCPSpeed"

    def test_custom_name(self):
        tp = ConstantTCPSpeedParameterization("MyTCP")
        assert tp.getName() == "MyTCP"

    def test_is_time_parameterization(self):
        tp = ConstantTCPSpeedParameterization()
        assert isinstance(tp, TimeParameterization)


class TestConstantTCPSpeedCompositeProfile:
    def test_default_construction(self):
        p = ConstantTCPSpeedCompositeProfile()
        assert p is not None
        # Defaults (from C++ header)
        assert p.max_translational_velocity == pytest.approx(1.0)
        assert p.max_rotational_velocity == pytest.approx(1.0)
        assert p.max_translational_acceleration == pytest.approx(1.0)
        assert p.max_rotational_acceleration == pytest.approx(1.0)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_full_constructor(self):
        p = ConstantTCPSpeedCompositeProfile(
            max_translational_velocity=0.5,
            max_rotational_velocity=0.8,
            max_translational_acceleration=1.5,
            max_rotational_acceleration=2.0,
            max_velocity_scaling_factor=0.75,
            max_acceleration_scaling_factor=0.6,
        )
        assert p.max_translational_velocity == pytest.approx(0.5)
        assert p.max_rotational_velocity == pytest.approx(0.8)
        assert p.max_translational_acceleration == pytest.approx(1.5)
        assert p.max_rotational_acceleration == pytest.approx(2.0)
        assert p.max_velocity_scaling_factor == pytest.approx(0.75)
        assert p.max_acceleration_scaling_factor == pytest.approx(0.6)

    def test_constructor_scaling_defaults(self):
        # scaling factors default to 1.0 when omitted
        p = ConstantTCPSpeedCompositeProfile(0.2, 0.3, 0.4, 0.5)
        assert p.max_translational_velocity == pytest.approx(0.2)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_member_assignment(self):
        p = ConstantTCPSpeedCompositeProfile()
        p.max_translational_velocity = 0.123
        p.max_velocity_scaling_factor = 0.5
        assert p.max_translational_velocity == pytest.approx(0.123)
        assert p.max_velocity_scaling_factor == pytest.approx(0.5)
