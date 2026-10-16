"""Tests for tesseract_motion_planners_trajopt bindings."""

from pathlib import Path

import numpy as np
import pytest

import tesseract_robotics.tesseract_motion_planners_trajopt as trajopt
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import MotionPlanner, PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

# TrajOpt imports - skip tests if not available
try:
    from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        ProfileDictionary_addTrajOptCompositeProfile,
        ProfileDictionary_addTrajOptPlanProfile,
        TrajOptCollisionConfig,
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultPlanProfile,
        TrajOptMotionPlanner,
    )

    TRAJOPT_AVAILABLE = True
except ImportError:
    TRAJOPT_AVAILABLE = False


TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


@pytest.fixture
def lbr_iiwa_environment():
    """Load LBR IIWA robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = Path(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
        ).getFilePath()
    )
    srdf_path = Path(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
        ).getFilePath()
    )

    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize LBR IIWA"

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    manip_info.manipulator_ik_solver = "KDLInvKinChainLMA"

    joint_names = list(t_env.getJointGroup("manipulator").getJointNames())
    return t_env, manip_info, joint_names


class TestTrajOptProfiles:
    """Test TrajOpt profile types."""

    def test_default_plan_profile(self):
        profile = TrajOptDefaultPlanProfile()
        assert profile is not None
        # Test base class methods
        assert profile.getKey() is not None

    def test_default_composite_profile(self):
        profile = TrajOptDefaultCompositeProfile()
        assert profile is not None
        # Test base class methods
        assert profile.getKey() is not None
        # Test default attributes
        assert hasattr(profile, "smooth_velocities")
        assert hasattr(profile, "smooth_accelerations")
        assert hasattr(profile, "smooth_jerks")
        assert hasattr(profile, "collision_cost_config")

    def test_default_composite_profile_all_attributes(self):
        """Test all TrajOptDefaultCompositeProfile attributes."""
        profile = TrajOptDefaultCompositeProfile()

        # Smoothing attributes
        profile.smooth_velocities = True
        profile.smooth_accelerations = True
        profile.smooth_jerks = True
        assert profile.smooth_velocities is True
        assert profile.smooth_accelerations is True
        assert profile.smooth_jerks is True

        # Singularity avoidance
        profile.avoid_singularity = True
        profile.avoid_singularity_coeff = 5.0
        assert profile.avoid_singularity is True
        assert profile.avoid_singularity_coeff == 5.0

        # Note: longest_valid_segment_* and contact_test_type removed in 0.33
        # These are now part of nested CollisionCheckConfig

        # Collision configs
        assert hasattr(profile, "collision_cost_config")
        assert hasattr(profile, "collision_constraint_config")

    def test_trajopt_collision_config(self):
        """Test TrajOptCollisionConfig (0.33 API)."""
        config = TrajOptCollisionConfig()
        assert config is not None
        config.enabled = True
        config.collision_margin_buffer = 0.025
        assert config.enabled is True
        assert config.collision_margin_buffer == 0.025

    def test_trajopt_collision_config_attributes(self):
        """Test TrajOptCollisionConfig attributes."""
        config = TrajOptCollisionConfig()
        config.enabled = False
        config.collision_margin_buffer = 0.01
        config.max_num_cnt = 5

        assert config.enabled is False
        assert config.collision_margin_buffer == 0.01
        assert config.max_num_cnt == 5

    def test_collision_evaluator_type_enum(self):
        """Test CollisionEvaluatorType enum values (now in tesseract_collision)."""
        assert CollisionEvaluatorType.NONE is not None
        assert CollisionEvaluatorType.DISCRETE is not None
        assert CollisionEvaluatorType.LVS_DISCRETE is not None
        assert CollisionEvaluatorType.CONTINUOUS is not None
        assert CollisionEvaluatorType.LVS_CONTINUOUS is not None

    def test_composite_profile_with_collision_configs(self):
        """Test setting collision configs on composite profile."""
        profile = TrajOptDefaultCompositeProfile()

        # Create and configure collision cost
        cost_config = TrajOptCollisionConfig()
        cost_config.enabled = True
        cost_config.collision_margin_buffer = 0.025
        profile.collision_cost_config = cost_config

        # Create and configure collision constraint
        constraint_config = TrajOptCollisionConfig()
        constraint_config.enabled = False
        constraint_config.collision_margin_buffer = 0.01
        profile.collision_constraint_config = constraint_config

        # Verify assignment
        assert profile.collision_cost_config.enabled is True
        assert profile.collision_constraint_config.enabled is False


class TestTrajOptMotionPlanner:
    """Test TrajOpt motion planner."""

    def test_constructor(self):
        planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
        assert planner is not None
        assert planner.getName() == TRAJOPT_DEFAULT_NAMESPACE

    def test_terminate_and_clear(self):
        """Test terminate() and clear() methods."""
        planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
        # These should not raise exceptions
        planner.terminate()
        planner.clear()


class TestTrajOptPlanning:
    """Integration test for TrajOpt motion planning."""

    def test_trajopt_freespace_joint_cart(self, lbr_iiwa_environment):
        """Test TrajOpt planning from joint to cartesian waypoint."""
        env, manip_info, joint_names = lbr_iiwa_environment

        # Start at joint position
        wp1 = JointWaypoint(joint_names, np.array([0, 0, 0, -1.57, 0, 0, 0], dtype=np.float64))
        # End at cartesian position
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(-0.2, 0.4, 0.2)
            * Quaterniond.from_xyzw(0, 1.0, 0, 0)
        )

        start_instruction = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "TEST_PROFILE",
        )
        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "TEST_PROFILE",
        )

        program = CompositeInstruction("TEST_PROFILE")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        # Interpolate the program first (TrajOpt requires a seed trajectory)
        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        # Setup TrajOpt profiles
        plan_profile = TrajOptDefaultPlanProfile()
        composite_profile = TrajOptDefaultCompositeProfile()

        profiles = ProfileDictionary()
        # Use helper functions for cross-module profile registration
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile
        )
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile
        )

        # Create planner and request
        test_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        # Solve
        response = test_planner.solve(request)
        assert response.successful, f"TrajOpt planning failed: {response.message}"

        # Verify results - iterate through instructions
        results = response.results
        count = 0
        for instr in results:
            if instr.isMoveInstruction():
                move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
                wp = move_instr.getWaypoint()
                if wp.isStateWaypoint():
                    state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                    assert len(state_wp.getNames()) == 7  # 7 joints for IIWA
                    assert isinstance(state_wp.getPosition(), np.ndarray)
                    assert len(state_wp.getPosition()) == 7
                count += 1
        assert count > 0, "Expected at least one move instruction in results"

    def test_trajopt_with_collision_config(self, lbr_iiwa_environment):
        """Test TrajOpt planning with collision configuration."""
        env, manip_info, joint_names = lbr_iiwa_environment

        # Simple motion
        wp1 = JointWaypoint(joint_names, np.array([0, 0, 0, -1.0, 0, 0, 0], dtype=np.float64))
        wp2 = JointWaypoint(joint_names, np.array([0.5, 0, 0, -1.0, 0, 0, 0], dtype=np.float64))

        start_instruction = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "COLLISION_TEST",
        )
        plan_f1 = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "COLLISION_TEST",
        )

        program = CompositeInstruction("COLLISION_TEST")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        # Setup profiles with collision config
        plan_profile = TrajOptDefaultPlanProfile()
        composite_profile = TrajOptDefaultCompositeProfile()

        # Configure collision cost (0.33 API)
        collision_cost = TrajOptCollisionConfig()
        collision_cost.enabled = True
        collision_cost.collision_margin_buffer = 0.025
        composite_profile.collision_cost_config = collision_cost

        # Configure smoothing
        composite_profile.smooth_velocities = True
        composite_profile.smooth_accelerations = True
        composite_profile.smooth_jerks = False

        profiles = ProfileDictionary()
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "COLLISION_TEST", plan_profile
        )
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "COLLISION_TEST", composite_profile
        )

        test_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        response = test_planner.solve(request)
        assert response.successful, (
            f"TrajOpt planning with collision config failed: {response.message}"
        )


def _joint_request(env, manip_info, joint_names, profile_name):
    """Two-waypoint joint move on the IIWA, interpolated as a TrajOpt seed."""
    program = CompositeInstruction(profile_name)
    program.setManipulatorInfo(manip_info)
    for q in ([0, 0, 0, -1.0, 0, 0, 0], [0.5, 0, 0, -1.0, 0, 0, 0]):
        wp = JointWaypoint(joint_names, np.array(q, dtype=np.float64))
        instr = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp), MoveInstructionType_FREESPACE, profile_name
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(instr))
    request = PlannerRequest()
    request.instructions = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)
    request.env = env
    return request


def _trajopt_profiles(profile_name, solver_profile=None):
    profiles = ProfileDictionary()
    trajopt.ProfileDictionary_addTrajOptMoveProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, profile_name, trajopt.TrajOptDefaultMoveProfile()
    )
    trajopt.ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, profile_name, trajopt.TrajOptDefaultCompositeProfile()
    )
    if solver_profile is not None:
        trajopt.ProfileDictionary_addTrajOptSolverProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, profile_name, solver_profile
        )
    return profiles


class TestOSQPSettings:
    """#306: OSQPSettings and TrajOptOSQPSolverProfile.settings."""

    def test_profile_holds_trajopt_defaults(self):
        # trajopt_sco OSQPModelConfig::setDefaultOSQPSettings, not osqp's or zeros
        s = trajopt.TrajOptOSQPSolverProfile().settings
        assert isinstance(s, trajopt.OSQPSettings)
        assert s.eps_abs == 1e-4
        assert s.eps_rel == 1e-6
        assert s.max_iter == 8192
        assert s.polishing == 1
        assert s.adaptive_rho == 1
        assert s.check_dualgap == 0

    def test_default_constructor_fills_osqp_defaults(self):
        s = trajopt.OSQPSettings()
        assert s.max_iter > 0
        assert s.eps_abs > 0
        assert s.rho > 0

    def test_assign_settings_round_trips(self):
        p = trajopt.TrajOptOSQPSolverProfile()
        s = p.settings
        s.eps_abs = 1e-5
        p.settings = s
        assert p.settings.eps_abs == 1e-5

    def test_equality(self):
        a = trajopt.TrajOptOSQPSolverProfile().settings
        b = trajopt.TrajOptOSQPSolverProfile().settings
        assert a == b
        b.max_iter = 100
        assert a != b
        with pytest.raises(TypeError):
            hash(a)

    def test_plan_with_python_assigned_settings(self, lbr_iiwa_environment):
        env, manip_info, joint_names = lbr_iiwa_environment
        solver = trajopt.TrajOptOSQPSolverProfile()
        s = solver.settings
        s.max_iter = 4000
        s.eps_abs = 1e-5
        solver.settings = s
        request = _joint_request(env, manip_info, joint_names, "OSQP_SETTINGS")
        request.profiles = _trajopt_profiles("OSQP_SETTINGS", solver)
        response = trajopt.TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE).solve(request)
        assert response.successful, response.message


class TestComputeLongestValidSegmentLength:
    """#307: one case per branch of the upstream definition."""

    # extent = ||upper - lower|| = ||(3, 4)|| = 5
    LIMITS = np.array([[0.0, 3.0], [0.0, 4.0]])

    @pytest.mark.parametrize(
        ("fraction", "length", "expected"),
        [
            (0.1, 0.2, 0.2),  # both positive: min(0.1 * 5, 0.2)
            (0.1, 1.0, 0.5),  # both positive: min(0.1 * 5, 1.0)
            (0.1, 0.0, 0.5),  # fraction only: 0.1 * 5
            (0.0, 0.3, 0.3),  # length only
            (0.0, 0.0, 0.05),  # neither: LONGEST_VALID_SEGMENT_FRACTION_DEFAULT (0.01) * 5
        ],
    )
    def test_branches(self, fraction, length, expected):
        result = trajopt.TrajOptDefaultCompositeProfile.computeLongestValidSegmentLength(
            self.LIMITS, fraction, length
        )
        assert result == pytest.approx(expected)


class TestTrajOptMotionPlannerBase:
    """#299: TrajOptMotionPlanner is a MotionPlanner."""

    def test_isinstance(self):
        assert isinstance(trajopt.TrajOptMotionPlanner("x"), MotionPlanner)

    def test_inherited_methods(self):
        planner = trajopt.TrajOptMotionPlanner("x")
        assert planner.getName() == "x"
        planner.terminate()
        planner.clear()
        cloned = planner.clone()
        assert type(cloned) is trajopt.TrajOptMotionPlanner
        assert cloned.getName() == "x"

    def test_base_solve_dispatches(self, lbr_iiwa_environment):
        env, manip_info, joint_names = lbr_iiwa_environment
        request = _joint_request(env, manip_info, joint_names, "BASE_SOLVE")
        request.profiles = _trajopt_profiles("BASE_SOLVE")
        planner = trajopt.TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
        response = MotionPlanner.solve(planner, request)
        assert response.successful, response.message


class TestTrajOptProfileHooks:
    """#339: TrajOptSolverProfile.createOptimizationParameters."""

    def test_create_optimization_parameters_copies_opt_params(self):
        p = trajopt.TrajOptOSQPSolverProfile()
        p.opt_params.max_iter = 17
        params = p.createOptimizationParameters()
        assert isinstance(params, trajopt.BasicTrustRegionSQPParameters)
        assert params.max_iter == 17
        assert params == p.opt_params


# (class, mutate) pairs for #261: mutate changes one field so the instances differ.
TRAJOPT_VALUE_TYPES = [
    ("TrajOptCartesianWaypointConfig", lambda x: setattr(x, "enabled", not x.enabled)),
    ("TrajOptJointWaypointConfig", lambda x: setattr(x, "enabled", not x.enabled)),
    ("TrajOptDefaultMoveProfile", lambda x: setattr(x.joint_cost_config, "enabled", True)),
    ("TrajOptDefaultCompositeProfile", lambda x: setattr(x, "smooth_jerks", not x.smooth_jerks)),
    ("TrajOptOSQPSolverProfile", lambda x: setattr(x, "update_workspace", True)),
    ("BasicTrustRegionSQPParameters", lambda x: setattr(x, "max_iter", 3)),
]


class TestTrajOptValueEquality:
    """#261: __eq__/__ne__ compare by value; the types are unhashable."""

    @pytest.mark.parametrize(("cls_name", "mutate"), TRAJOPT_VALUE_TYPES)
    def test_equality(self, cls_name, mutate):
        cls = getattr(trajopt, cls_name)
        a, b = cls(), cls()
        assert a == b
        assert not (a != b)
        mutate(b)
        assert a != b
        assert not (a == b)

    @pytest.mark.parametrize(("cls_name", "mutate"), TRAJOPT_VALUE_TYPES)
    def test_unhashable(self, cls_name, mutate):
        with pytest.raises(TypeError):
            hash(getattr(trajopt, cls_name)())


class TestTrajOptCreateKey:
    """#273: static createKey() matches the instance key."""

    @pytest.mark.parametrize(
        "cls_name",
        ["TrajOptDefaultMoveProfile", "TrajOptDefaultCompositeProfile", "TrajOptOSQPSolverProfile"],
    )
    def test_create_key_matches_get_key(self, cls_name):
        cls = getattr(trajopt, cls_name)
        assert cls.createKey() == cls().getKey()

    def test_keys_differ_between_profile_kinds(self):
        keys = {
            trajopt.TrajOptMoveProfile.createKey(),
            trajopt.TrajOptCompositeProfile.createKey(),
            trajopt.TrajOptSolverProfile.createKey(),
        }
        assert len(keys) == 3


class TestTrajOptFromYaml:
    """#331: from_yaml on the three TrajOpt profiles reads the C++ YAML config."""

    @pytest.mark.parametrize(
        ("cls_name", "config", "mutate"),
        [
            (
                "TrajOptDefaultMoveProfile",
                "joint_cost_config:\n  enabled: true\n",
                lambda p: setattr(p.joint_cost_config, "enabled", True),
            ),
            (
                "TrajOptDefaultCompositeProfile",
                "avoid_singularity_coeff: 7.5\n",
                lambda p: setattr(p, "avoid_singularity_coeff", 7.5),
            ),
            (
                "TrajOptOSQPSolverProfile",
                "update_workspace: true\n",
                lambda p: setattr(p, "update_workspace", True),
            ),
        ],
    )
    def test_one_field(self, cls_name, config, mutate):
        cls = getattr(trajopt, cls_name)
        expected = cls()
        mutate(expected)
        profile = cls.from_yaml(config)
        assert profile == expected
        assert profile != cls()

    def test_osqp_settings_from_yaml(self):
        profile = trajopt.TrajOptOSQPSolverProfile.from_yaml("settings:\n  max_iter: 321\n")
        assert profile.settings.max_iter == 321

    @pytest.mark.parametrize(
        "cls_name",
        ["TrajOptDefaultMoveProfile", "TrajOptDefaultCompositeProfile", "TrajOptOSQPSolverProfile"],
    )
    def test_malformed_yaml_raises_value_error(self, cls_name):
        with pytest.raises(ValueError):
            getattr(trajopt, cls_name).from_yaml("key: [")

    def test_wrong_value_type_raises(self):
        with pytest.raises(RuntimeError, match="Failed to parse yaml config"):
            trajopt.TrajOptDefaultCompositeProfile.from_yaml("avoid_singularity_coeff: abc\n")
