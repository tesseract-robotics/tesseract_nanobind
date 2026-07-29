"""Tests for tesseract_motion_planners_trajopt_ifopt bindings."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    ProfileDictionary_addTrajOptIfoptCompositeProfile,
    ProfileDictionary_addTrajOptIfoptPlanProfile,
    ProfileDictionary_addTrajOptIfoptSolverProfile,
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultPlanProfile,
    TrajOptIfoptMotionPlanner,
    TrajOptIfoptOSQPSolverProfile,
    TrajOptIfoptSolverProfile,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.trajopt_sqp import SQPParameters

TRAJOPT_IFOPT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"


@pytest.fixture
def kuka_iiwa_environment():
    """Load KUKA IIWA robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
        ).getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
        ).getFilePath()
    )

    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize KUKA IIWA"

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    joint_names = list(t_env.getJointGroup("manipulator").getJointNames())
    return t_env, manip_info, joint_names


class TestTrajOptIfoptProfiles:
    """Test TrajOptIfopt profile types."""

    def test_default_plan_profile(self):
        profile = TrajOptIfoptDefaultPlanProfile()
        assert profile is not None
        assert profile.getKey() is not None

    def test_default_composite_profile(self):
        profile = TrajOptIfoptDefaultCompositeProfile()
        assert profile is not None
        # Check default values
        assert hasattr(profile, "smooth_velocities")
        assert hasattr(profile, "smooth_accelerations")
        assert hasattr(profile, "smooth_jerks")

    def test_osqp_solver_profile(self):
        profile = TrajOptIfoptOSQPSolverProfile()
        assert profile is not None
        assert profile.getKey() is not None

    def test_osqp_solver_profile_settings(self):
        """Test OSQP settings setters forward to qp_settings."""
        profile = TrajOptIfoptOSQPSolverProfile()
        profile.setPolish(False)
        profile.setWarmStart(False)
        profile.setAdaptiveRho(False)
        profile.setAdaptiveRhoInterval(25)
        profile.setMaxIteration(4096)
        profile.setAbsoluteTolerance(1e-5)
        profile.setRelativeTolerance(1e-7)
        profile.setVerbosity(False)

    def test_solver_profile_opt_params_defaults(self):
        """opt_params is exposed and reports the C++ SQP defaults."""
        profile = TrajOptIfoptOSQPSolverProfile()
        assert isinstance(profile.opt_params, SQPParameters)
        assert profile.opt_params.max_iterations == 50
        assert profile.opt_params.max_qp_solver_failures == 3

    def test_solver_profile_opt_params_on_base_class(self):
        """opt_params is bound on the base profile, not just the OSQP subclass."""
        assert hasattr(TrajOptIfoptSolverProfile, "opt_params")

    def test_solver_profile_opt_params_in_place(self):
        """In-place writes to opt_params reach the profile (reference, not copy)."""
        profile = TrajOptIfoptOSQPSolverProfile()
        profile.opt_params.max_iterations = 5
        profile.opt_params.max_qp_solver_failures = 1
        profile.opt_params.initial_trust_box_size = 0.05
        profile.opt_params.max_time = 2.5

        assert profile.opt_params.max_iterations == 5
        assert profile.opt_params.max_qp_solver_failures == 1
        assert profile.opt_params.initial_trust_box_size == 0.05
        assert profile.opt_params.max_time == 2.5

    def test_solver_profile_opt_params_assignment(self):
        """Assigning a whole SQPParameters copies its values onto the profile."""
        params = SQPParameters()
        params.max_iterations = 11
        params.trust_shrink_ratio = 0.2
        params.trust_expand_ratio = 2.0
        params.cnt_tolerance = 1e-3
        params.max_merit_coeff_increases = 2
        params.merit_coeff_increase_ratio = 5.0
        params.initial_merit_error_coeff = 20.0
        params.inflate_constraints_individually = False

        profile = TrajOptIfoptOSQPSolverProfile()
        profile.opt_params = params

        assert profile.opt_params.max_iterations == 11
        assert profile.opt_params.trust_shrink_ratio == 0.2
        assert profile.opt_params.trust_expand_ratio == 2.0
        assert profile.opt_params.cnt_tolerance == 1e-3
        assert profile.opt_params.max_merit_coeff_increases == 2
        assert profile.opt_params.merit_coeff_increase_ratio == 5.0
        assert profile.opt_params.initial_merit_error_coeff == 20.0
        assert profile.opt_params.inflate_constraints_individually is False

        # Assignment copies: later edits to the source do not leak into the profile.
        params.max_iterations = 99
        assert profile.opt_params.max_iterations == 11

    def test_composite_profile_collision_config_defaults(self):
        """Both collision configs are exposed as trajopt_common::TrajOptCollisionConfig."""
        profile = TrajOptIfoptDefaultCompositeProfile()

        for config in (profile.collision_cost_config, profile.collision_constraint_config):
            assert isinstance(config, TrajOptCollisionConfig)
            assert config.enabled is True
            assert config.collision_margin_buffer == 0.01
            # max_num_cnt is the ifopt-only fixed constraint-row cap
            assert config.max_num_cnt > 0

    def test_composite_profile_collision_config_in_place(self):
        """In-place writes to the collision configs reach the profile."""
        profile = TrajOptIfoptDefaultCompositeProfile()

        profile.collision_cost_config.enabled = False
        profile.collision_cost_config.collision_margin_buffer = 0.025
        profile.collision_cost_config.max_num_cnt = 7

        assert profile.collision_cost_config.enabled is False
        assert profile.collision_cost_config.collision_margin_buffer == 0.025
        assert profile.collision_cost_config.max_num_cnt == 7

    def test_composite_profile_collision_configs_are_independent(self):
        """Cost and constraint configs are separate members."""
        profile = TrajOptIfoptDefaultCompositeProfile()

        profile.collision_cost_config.max_num_cnt = 3
        profile.collision_constraint_config.max_num_cnt = 9

        assert profile.collision_cost_config.max_num_cnt == 3
        assert profile.collision_constraint_config.max_num_cnt == 9

    def test_composite_profile_collision_config_assignment(self):
        """Assigning whole configs copies their values onto the profile."""
        cost_config = TrajOptCollisionConfig(0.05, 20.0)
        cost_config.enabled = True
        cost_config.collision_margin_buffer = 0.02
        cost_config.max_num_cnt = 4

        constraint_config = TrajOptCollisionConfig()
        constraint_config.enabled = False
        constraint_config.collision_margin_buffer = 0.005
        constraint_config.max_num_cnt = 1

        profile = TrajOptIfoptDefaultCompositeProfile()
        profile.collision_cost_config = cost_config
        profile.collision_constraint_config = constraint_config

        assert profile.collision_cost_config.enabled is True
        assert profile.collision_cost_config.collision_margin_buffer == 0.02
        assert profile.collision_cost_config.max_num_cnt == 4
        assert profile.collision_constraint_config.enabled is False
        assert profile.collision_constraint_config.collision_margin_buffer == 0.005
        assert profile.collision_constraint_config.max_num_cnt == 1

        # Assignment copies: later edits to the source do not leak into the profile.
        cost_config.max_num_cnt = 99
        assert profile.collision_cost_config.max_num_cnt == 4

    def test_composite_profile_collision_config_nested_members(self):
        """The nested margin/check/coeff configs are reachable through the profile."""
        profile = TrajOptIfoptDefaultCompositeProfile()
        config = profile.collision_constraint_config

        config.contact_manager_config.default_margin = 0.03
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        config.collision_check_config.longest_valid_segment_length = 0.05
        config.collision_coeff_data.setDefaultCollisionCoeff(15.0)

        assert profile.collision_constraint_config.contact_manager_config.default_margin == 0.03
        assert (
            profile.collision_constraint_config.collision_check_config.type
            == CollisionEvaluatorType.LVS_DISCRETE
        )
        assert (
            profile.collision_constraint_config.collision_check_config.longest_valid_segment_length
            == 0.05
        )
        assert (
            profile.collision_constraint_config.collision_coeff_data.getDefaultCollisionCoeff()
            == 15.0
        )

    def test_composite_profile_modify_collision_objects_scoping(self):
        """modify_object_enabled scoping is settable through the collision config."""
        profile = TrajOptIfoptDefaultCompositeProfile()
        profile.collision_cost_config.contact_manager_config.modify_object_enabled = {
            "link_1": False,
            "link_2": True,
        }

        enabled = profile.collision_cost_config.contact_manager_config.modify_object_enabled
        assert enabled == {"link_1": False, "link_2": True}

    def test_add_profiles_to_dictionary(self):
        """Test adding TrajOptIfopt profiles to ProfileDictionary."""
        profiles = ProfileDictionary()

        plan_profile = TrajOptIfoptDefaultPlanProfile()
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        solver_profile = TrajOptIfoptOSQPSolverProfile()

        # Add profiles using helper functions
        ProfileDictionary_addTrajOptIfoptPlanProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", plan_profile
        )
        ProfileDictionary_addTrajOptIfoptCompositeProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", composite_profile
        )
        ProfileDictionary_addTrajOptIfoptSolverProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", solver_profile
        )

        assert profiles is not None


class TestTrajOptIfoptPlanner:
    """Test TrajOptIfopt planner functionality."""

    def test_planner_creation(self):
        """Test creating TrajOptIfopt planner."""
        planner = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE)
        assert planner.getName() == TRAJOPT_IFOPT_NAMESPACE

    def test_joint_to_joint_planning(self, kuka_iiwa_environment):
        """Test planning from joint state to joint state."""
        t_env, manip_info, joint_names = kuka_iiwa_environment

        # Set initial state
        start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
        end_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        t_env.setState(joint_names, start_pos)

        # Create waypoints
        wp1 = JointWaypoint(joint_names, start_pos)
        wp2 = JointWaypoint(joint_names, end_pos)

        # Create instructions
        start_instr = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT"
        )
        end_instr = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT"
        )

        # Create program
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(end_instr))

        # Interpolate program
        interpolated = generateInterpolatedProgram(program, t_env, 3.14, 1.0, 3.14, 30)

        # Create profiles with proper velocity coefficients
        plan_profile = TrajOptIfoptDefaultPlanProfile()
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        # Initialize coefficients to match DOF count (7 for KUKA IIWA)
        n_joints = len(joint_names)
        composite_profile.velocity_coeff = np.ones(n_joints)
        composite_profile.acceleration_coeff = np.ones(n_joints)
        composite_profile.jerk_coeff = np.ones(n_joints)
        solver_profile = TrajOptIfoptOSQPSolverProfile()

        profiles = ProfileDictionary()
        ProfileDictionary_addTrajOptIfoptPlanProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", plan_profile
        )
        ProfileDictionary_addTrajOptIfoptCompositeProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", composite_profile
        )
        ProfileDictionary_addTrajOptIfoptSolverProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", solver_profile
        )

        # Create request
        request = PlannerRequest()
        request.instructions = interpolated
        request.env = t_env
        request.profiles = profiles

        # Solve
        planner = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE)
        response = planner.solve(request)

        assert response.successful, f"Planning failed: {response.message}"
        assert response.results is not None

    def test_planning_with_collision_configs_and_opt_params(self, kuka_iiwa_environment):
        """Plan with the newly bound collision configs and SQP parameters set."""
        t_env, manip_info, joint_names = kuka_iiwa_environment

        start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
        end_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        t_env.setState(joint_names, start_pos)

        wp1 = JointWaypoint(joint_names, start_pos)
        wp2 = JointWaypoint(joint_names, end_pos)
        start_instr = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT"
        )
        end_instr = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT"
        )

        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(end_instr))

        interpolated = generateInterpolatedProgram(program, t_env, 3.14, 1.0, 3.14, 30)

        n_joints = len(joint_names)
        plan_profile = TrajOptIfoptDefaultPlanProfile()
        composite_profile = TrajOptIfoptDefaultCompositeProfile()
        composite_profile.velocity_coeff = np.ones(n_joints)
        composite_profile.acceleration_coeff = np.ones(n_joints)
        composite_profile.jerk_coeff = np.ones(n_joints)

        # Collision as a constraint only, with a bounded number of constraint rows
        # per timestep (max_num_cnt is honoured by trajopt_ifopt only).
        composite_profile.collision_cost_config.enabled = False
        composite_profile.collision_constraint_config.enabled = True
        composite_profile.collision_constraint_config.max_num_cnt = 3
        composite_profile.collision_constraint_config.collision_margin_buffer = 0.01
        composite_profile.collision_constraint_config.contact_manager_config.default_margin = 0.01

        solver_profile = TrajOptIfoptOSQPSolverProfile()
        solver_profile.opt_params.max_iterations = 20
        solver_profile.opt_params.max_qp_solver_failures = 5
        solver_profile.opt_params.initial_trust_box_size = 0.1

        profiles = ProfileDictionary()
        ProfileDictionary_addTrajOptIfoptPlanProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", plan_profile
        )
        ProfileDictionary_addTrajOptIfoptCompositeProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", composite_profile
        )
        ProfileDictionary_addTrajOptIfoptSolverProfile(
            profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", solver_profile
        )

        request = PlannerRequest()
        request.instructions = interpolated
        request.env = t_env
        request.profiles = profiles

        planner = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE)
        response = planner.solve(request)

        assert response.successful, f"Planning failed: {response.message}"
        assert response.results is not None
