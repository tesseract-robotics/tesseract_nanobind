"""Tests for tesseract_task_composer planning-node profile bindings."""

from pathlib import Path

import numpy as np
import pytest

import tesseract_robotics.tesseract_motion_planners_trajopt as trajopt
import tesseract_robotics.tesseract_task_composer_planning as tcp_module
from tesseract_robotics.tesseract_collision import CollisionCheckConfig
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    BasicTrustRegionSQPParameters,
    TrajOptJointWaypointConfig,
)
from tesseract_robotics.tesseract_task_composer_planning import (
    ContactCheckProfile,
    FixStateBoundsProfile,
    FixStateCollisionProfile,
    KinematicLimitsCheckProfile,
    MinLengthProfile,
    ProfileSwitchProfile,
    UpsampleTrajectoryProfile,
)
from tesseract_robotics.trajopt_ifopt import CollisionCoeffData

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


# ---- FixStateCollisionProfile TrajOpt fields (#326) ------------------------


class TestFixStateCollisionProfileTrajOptFields:
    def test_opt_params_round_trip(self):
        p = FixStateCollisionProfile()
        p.opt_params.max_iter = 7
        assert p.opt_params.max_iter == 7
        assert isinstance(p.opt_params, BasicTrustRegionSQPParameters)

    @pytest.mark.parametrize(
        "field", ["trajopt_joint_constraint_config", "trajopt_joint_cost_config"]
    )
    def test_joint_waypoint_config_round_trip(self, field):
        p = FixStateCollisionProfile()
        cfg = getattr(p, field)
        assert isinstance(cfg, TrajOptJointWaypointConfig)
        cfg.coeff = np.array([3.0])
        assert np.array_equal(getattr(p, field).coeff, [3.0])

    @pytest.mark.parametrize("field", ["collision_constraint_coeff", "collision_cost_coeff"])
    def test_collision_coeff_round_trip(self, field):
        p = FixStateCollisionProfile()
        coeff = getattr(p, field)
        assert isinstance(coeff, CollisionCoeffData)
        coeff.setDefaultCollisionCoeff(42.0)
        assert getattr(p, field).getDefaultCollisionCoeff() == 42.0

    def test_osqp_settings_round_trip(self):
        p = FixStateCollisionProfile()
        s = p.osqp_settings
        assert isinstance(s, trajopt.OSQPSettings)
        s.max_iter = 123
        p.osqp_settings = s
        assert p.osqp_settings.max_iter == 123

    def test_stub_names_imported_types(self):
        stub = (
            Path(tcp_module.__file__).parent / "_tesseract_task_composer_planning.pyi"
        ).read_text()
        assert "tesseract_motion_planners_trajopt" in stub
        assert '"sco::' not in stub
        assert '"trajopt_common::' not in stub
        assert '"tesseract::motion_planners::' not in stub


# ---- value equality (#261) --------------------------------------------------

# (class, mutate): mutate changes one bound field so two instances differ.
TCP_VALUE_TYPES = [
    (
        ContactCheckProfile,
        lambda p: setattr(p.collision_check_config, "longest_valid_segment_length", 0.123),
    ),
    (FixStateBoundsProfile, lambda p: setattr(p, "max_deviation_global", 0.5)),
    (FixStateCollisionProfile, lambda p: setattr(p, "jiggle_factor", 0.5)),
    (KinematicLimitsCheckProfile, lambda p: setattr(p, "check_velocity", False)),
    (MinLengthProfile, lambda p: setattr(p, "min_length", 99)),
    (ProfileSwitchProfile, lambda p: setattr(p, "return_value", 7)),
    (UpsampleTrajectoryProfile, lambda p: setattr(p, "longest_valid_segment_length", 0.7)),
]


class TestValueEquality:
    @pytest.mark.parametrize(("cls", "mutate"), TCP_VALUE_TYPES)
    def test_equality(self, cls, mutate):
        a, b = cls(), cls()
        assert a == b
        assert not (a != b)
        mutate(b)
        assert a != b
        assert not (a == b)

    @pytest.mark.parametrize(("cls", "mutate"), TCP_VALUE_TYPES)
    def test_unhashable(self, cls, mutate):
        with pytest.raises(TypeError):
            hash(cls())


# ---- createKey (#273) -------------------------------------------------------

TCP_PROFILES = [cls for cls, _ in TCP_VALUE_TYPES]


class TestCreateKey:
    @pytest.mark.parametrize("cls", TCP_PROFILES)
    def test_create_key_matches_get_key(self, cls):
        assert cls.createKey() == cls().getKey()

    def test_keys_differ(self):
        assert len({cls.createKey() for cls in TCP_PROFILES}) == len(TCP_PROFILES)


# ---- from_yaml (#331) -------------------------------------------------------


def _replace_collision_check_config(profile, **fields):
    # A YAML key replaces the whole sub-struct, starting from that struct's own defaults
    # (upstream `n.as<CollisionCheckConfig>()`): type DISCRETE, not the profile's LVS_DISCRETE.
    config = CollisionCheckConfig()
    for name, value in fields.items():
        setattr(config, name, value)
    profile.collision_check_config = config


TCP_YAML = [
    (
        ContactCheckProfile,
        "collision_check_config:\n  longest_valid_segment_length: 0.123\n",
        lambda p: _replace_collision_check_config(p, longest_valid_segment_length=0.123),
    ),
    (
        FixStateBoundsProfile,
        "max_deviation_global: 0.5\n",
        lambda p: setattr(p, "max_deviation_global", 0.5),
    ),
    (FixStateCollisionProfile, "jiggle_factor: 0.5\n", lambda p: setattr(p, "jiggle_factor", 0.5)),
    (
        KinematicLimitsCheckProfile,
        "check_velocity: false\n",
        lambda p: setattr(p, "check_velocity", False),
    ),
    (MinLengthProfile, "min_length: 99\n", lambda p: setattr(p, "min_length", 99)),
    (ProfileSwitchProfile, "return_value: 7\n", lambda p: setattr(p, "return_value", 7)),
    (
        UpsampleTrajectoryProfile,
        "longest_valid_segment_length: 0.7\n",
        lambda p: setattr(p, "longest_valid_segment_length", 0.7),
    ),
]


class TestFromYaml:
    @pytest.mark.parametrize(("cls", "config", "mutate"), TCP_YAML)
    def test_one_field(self, cls, config, mutate):
        expected = cls()
        mutate(expected)
        profile = cls.from_yaml(config)
        assert profile == expected
        assert profile != cls()

    @pytest.mark.parametrize("cls", TCP_PROFILES)
    def test_malformed_yaml_raises_value_error(self, cls):
        with pytest.raises(ValueError):
            cls.from_yaml("key: [")

    def test_wrong_value_type_raises(self):
        with pytest.raises(RuntimeError):
            MinLengthProfile.from_yaml("min_length: abc\n")
