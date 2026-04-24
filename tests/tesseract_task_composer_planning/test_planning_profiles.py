"""Tests for tesseract_task_composer planning-node profile bindings."""

import pytest

from tesseract_robotics.tesseract_task_composer_planning import (
    ContactCheckProfile,
    FixStateBoundsProfile,
    FixStateCollisionProfile,
    KinematicLimitsCheckProfile,
    MinLengthProfile,
    ProfileSwitchProfile,
    UpsampleTrajectoryProfile,
)


# ---- MinLengthProfile ------------------------------------------------------

class TestMinLengthProfile:
    def test_default(self):
        p = MinLengthProfile()
        assert p.min_length == 10

    def test_custom(self):
        p = MinLengthProfile(42)
        assert p.min_length == 42

    def test_assignment(self):
        p = MinLengthProfile()
        p.min_length = 5
        assert p.min_length == 5


# ---- UpsampleTrajectoryProfile --------------------------------------------

class TestUpsampleTrajectoryProfile:
    def test_default(self):
        p = UpsampleTrajectoryProfile()
        assert p.longest_valid_segment_length == pytest.approx(0.1)

    def test_custom(self):
        p = UpsampleTrajectoryProfile(0.05)
        assert p.longest_valid_segment_length == pytest.approx(0.05)

    def test_assignment(self):
        p = UpsampleTrajectoryProfile()
        p.longest_valid_segment_length = 0.25
        assert p.longest_valid_segment_length == pytest.approx(0.25)


# ---- ProfileSwitchProfile --------------------------------------------------

class TestProfileSwitchProfile:
    def test_default(self):
        p = ProfileSwitchProfile()
        assert p.return_value == 1

    def test_custom(self):
        p = ProfileSwitchProfile(7)
        assert p.return_value == 7

    def test_assignment(self):
        p = ProfileSwitchProfile()
        p.return_value = -1
        assert p.return_value == -1


# ---- KinematicLimitsCheckProfile ------------------------------------------

class TestKinematicLimitsCheckProfile:
    def test_default(self):
        p = KinematicLimitsCheckProfile()
        assert p.check_position is True
        assert p.check_velocity is True
        assert p.check_acceleration is True

    def test_custom(self):
        p = KinematicLimitsCheckProfile(
            check_position=False, check_velocity=True, check_acceleration=False
        )
        assert p.check_position is False
        assert p.check_velocity is True
        assert p.check_acceleration is False

    def test_assignment(self):
        p = KinematicLimitsCheckProfile()
        p.check_position = False
        assert p.check_position is False


# ---- FixStateBoundsProfile -------------------------------------------------

class TestFixStateBoundsProfile:
    def test_default(self):
        p = FixStateBoundsProfile()
        assert p.mode == FixStateBoundsProfile.Settings.ALL

    def test_settings_enum_values(self):
        # All enum values exposed
        assert FixStateBoundsProfile.Settings.START_ONLY is not None
        assert FixStateBoundsProfile.Settings.END_ONLY is not None
        assert FixStateBoundsProfile.Settings.ALL is not None
        assert FixStateBoundsProfile.Settings.DISABLED is not None

    def test_construct_with_mode(self):
        p = FixStateBoundsProfile(FixStateBoundsProfile.Settings.START_ONLY)
        assert p.mode == FixStateBoundsProfile.Settings.START_ONLY

    def test_mutable_fields(self):
        p = FixStateBoundsProfile()
        p.max_deviation_global = 0.25
        p.upper_bounds_reduction = 0.01
        p.lower_bounds_reduction = 0.01
        assert p.max_deviation_global == pytest.approx(0.25)
        assert p.upper_bounds_reduction == pytest.approx(0.01)
        assert p.lower_bounds_reduction == pytest.approx(0.01)


# ---- FixStateCollisionProfile ---------------------------------------------

class TestFixStateCollisionProfile:
    def test_default(self):
        p = FixStateCollisionProfile()
        assert p.mode == FixStateCollisionProfile.Settings.ALL
        assert p.jiggle_factor == pytest.approx(0.02)
        assert p.sampling_attempts == 100
        assert p.update_workspace is False

    def test_settings_enum_values(self):
        s = FixStateCollisionProfile.Settings
        assert s.START_ONLY is not None
        assert s.END_ONLY is not None
        assert s.INTERMEDIATE_ONLY is not None
        assert s.ALL is not None
        assert s.ALL_EXCEPT_START is not None
        assert s.ALL_EXCEPT_END is not None
        assert s.DISABLED is not None

    def test_correction_method_enum_values(self):
        cm = FixStateCollisionProfile.CorrectionMethod
        assert cm.NONE is not None
        assert cm.TRAJOPT is not None
        assert cm.RANDOM_SAMPLER is not None

    def test_default_correction_workflow(self):
        p = FixStateCollisionProfile()
        workflow = list(p.correction_workflow)
        assert len(workflow) == 2
        assert workflow[0] == FixStateCollisionProfile.CorrectionMethod.TRAJOPT
        assert workflow[1] == FixStateCollisionProfile.CorrectionMethod.RANDOM_SAMPLER

    def test_set_correction_workflow(self):
        p = FixStateCollisionProfile()
        p.correction_workflow = [FixStateCollisionProfile.CorrectionMethod.RANDOM_SAMPLER]
        workflow = list(p.correction_workflow)
        assert len(workflow) == 1
        assert workflow[0] == FixStateCollisionProfile.CorrectionMethod.RANDOM_SAMPLER

    def test_construct_with_mode(self):
        p = FixStateCollisionProfile(FixStateCollisionProfile.Settings.START_ONLY)
        assert p.mode == FixStateCollisionProfile.Settings.START_ONLY

    def test_mutable_scalar_fields(self):
        p = FixStateCollisionProfile()
        p.jiggle_factor = 0.05
        p.sampling_attempts = 200
        p.update_workspace = True
        assert p.jiggle_factor == pytest.approx(0.05)
        assert p.sampling_attempts == 200
        assert p.update_workspace is True


# ---- ContactCheckProfile ---------------------------------------------------

class TestContactCheckProfile:
    def test_default(self):
        p = ContactCheckProfile()
        assert p is not None

    def test_custom_constructor(self):
        # Second ctor signature: (longest_valid_segment_length, contact_distance)
        p = ContactCheckProfile(0.05, 0.01)
        assert p is not None

    def test_has_config_members(self):
        p = ContactCheckProfile()
        # Verify the bound struct members are accessible
        assert p.contact_manager_config is not None
        assert p.collision_check_config is not None
