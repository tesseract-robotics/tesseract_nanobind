"""Tests for tesseract_motion_planners_trajopt_ifopt bindings."""

import gc
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    InstructionPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import AddLinkCommand, Environment
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_motion_planners import MotionPlanner, PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    ProfileDictionary_addTrajOptIfoptCompositeProfile,
    ProfileDictionary_addTrajOptIfoptMoveProfile,
    ProfileDictionary_addTrajOptIfoptPlanProfile,
    ProfileDictionary_addTrajOptIfoptSolverProfile,
    ProfileOverrideNotDispatchedError,
    TrajOptIfoptCartesianWaypointConfig,
    TrajOptIfoptCompositeProfile,
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultMoveProfile,
    TrajOptIfoptDefaultPlanProfile,
    TrajOptIfoptJointWaypointConfig,
    TrajOptIfoptMotionPlanner,
    TrajOptIfoptMoveProfile,
    TrajOptIfoptNodeLayoutChangedError,
    TrajOptIfoptOSQPSolverProfile,
    TrajOptIfoptProfileReturnTypeError,
    TrajOptIfoptSolverProfile,
    TrajOptIfoptTermInfos,
    TrajOptIfoptWaypointInfo,
    TrajOptIfoptWaypointInfoWithoutNodeError,
    createCollisionConstraints,
    createJointVelocityConstraint,
)
from tesseract_robotics.tesseract_scene_graph import Collision, Joint, JointType, Link
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.trajopt_sqp import SQPCallback, SQPParameters

TRAJOPT_IFOPT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"


@pytest.fixture
def kuka_iiwa_environment():
    """Load KUKA IIWA robot environment for testing."""
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


START_POS = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
END_POS = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])


def _joint_move(joint_names, position):
    """A FREESPACE MoveInstructionPoly to a constrained joint waypoint."""
    waypoint = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joint_names, position))
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(waypoint, MoveInstructionType_FREESPACE, "DEFAULT")
    )


def _interpolated_program(t_env, manip_info, joint_names):
    """START_POS -> END_POS, interpolated as in the planning tests above."""
    t_env.setState(joint_names, START_POS)
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(_joint_move(joint_names, START_POS))
    program.appendMoveInstruction(_joint_move(joint_names, END_POS))
    return generateInterpolatedProgram(program, t_env, 3.14, 1.0, 3.14, 30)


def _move_instructions(composite):
    """The move instructions of `composite` in order, as the planner flattens them."""
    moves = []
    for instruction in composite:
        if instruction.isCompositeInstruction():
            moves.extend(_move_instructions(InstructionPoly_as_CompositeInstruction(instruction)))
        elif instruction.isMoveInstruction():
            moves.append(InstructionPoly_as_MoveInstructionPoly(instruction))
    return moves


def _smoothing_composite_profile(n_joints):
    """Velocity, acceleration and jerk smoothing (upstream enables velocity only)."""
    profile = TrajOptIfoptDefaultCompositeProfile()
    profile.smooth_velocities = True
    profile.smooth_accelerations = True
    profile.smooth_jerks = True
    profile.velocity_coeff = np.ones(n_joints)
    profile.acceleration_coeff = np.ones(n_joints)
    profile.jerk_coeff = np.ones(n_joints)
    return profile


def _request(t_env, manip_info, joint_names, solver_profile):
    profiles = ProfileDictionary()
    ProfileDictionary_addTrajOptIfoptPlanProfile(
        profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", TrajOptIfoptDefaultPlanProfile()
    )
    ProfileDictionary_addTrajOptIfoptCompositeProfile(
        profiles,
        TRAJOPT_IFOPT_NAMESPACE,
        "DEFAULT",
        _smoothing_composite_profile(len(joint_names)),
    )
    ProfileDictionary_addTrajOptIfoptSolverProfile(
        profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", solver_profile
    )
    request = PlannerRequest()
    request.instructions = _interpolated_program(t_env, manip_info, joint_names)
    request.env = t_env
    request.profiles = profiles
    return request


class _RecordingCallback(SQPCallback):
    """Records the SQP iteration of every call; returns `keep_going`."""

    def __init__(self, keep_going=True):
        super().__init__()
        self.keep_going = keep_going
        self.iterations = []

    def execute(self, problem, sqp_results):
        self.iterations.append(sqp_results.overall_iteration)
        return self.keep_going


class TestTrajOptIfoptSolverCallbacks:
    """`TrajOptIfoptSolverProfile.callbacks` reach the planner's solver (gh-308)."""

    def test_create_optimization_callbacks_returns_assigned_list(self):
        callback = _RecordingCallback()
        profile = TrajOptIfoptOSQPSolverProfile()
        profile.callbacks = [callback]
        callbacks = profile.createOptimizationCallbacks()
        assert len(callbacks) == 1
        assert callbacks[0] is callback

    def test_none_callback_is_rejected(self):
        profile = TrajOptIfoptOSQPSolverProfile()
        with pytest.raises(TypeError, match=r"callbacks\[0\] is None"):
            profile.callbacks = [None]

    def test_callback_called_every_iteration(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        callback = _RecordingCallback()
        solver_profile = TrajOptIfoptOSQPSolverProfile()
        solver_profile.callbacks = [callback]

        response = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(
            _request(t_env, manip_info, joint_names, solver_profile)
        )

        assert response.successful, response.message
        assert callback.iterations, "the planner never called the Python callback"
        # every SQP iteration the solver reached called the callback at least once
        assert set(range(1, max(callback.iterations) + 1)) <= set(callback.iterations)

    def test_callback_returning_false_called_once_per_iteration_up_to_max(
        self, kuka_iiwa_environment
    ):
        # Whether False stops the solve is upstream's (trajopt 0.35.0 keeps iterating, see
        # docs/user-guide/low-level-sqp.md); the binding guarantees the calls and their arguments.
        t_env, manip_info, joint_names = kuka_iiwa_environment
        seen = []

        class Recorder(SQPCallback):
            def execute(self, problem, sqp_results):
                # `problem` is valid only during the call: check it here, never keep it.
                seen.append(
                    (
                        sqp_results.overall_iteration,
                        isinstance(problem, tsqp.QPProblem),
                        isinstance(sqp_results, tsqp.SQPResults),
                    )
                )
                return False

        max_iterations = 5
        solver_profile = TrajOptIfoptOSQPSolverProfile()
        solver_profile.opt_params.max_iterations = max_iterations
        solver_profile.callbacks = [Recorder()]

        TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(
            _request(t_env, manip_info, joint_names, solver_profile)
        )

        iterations = [iteration for iteration, _, _ in seen]
        assert iterations, "the planner never called the Python callback"
        # one call per SQP iteration the solver reached, never past max_iterations
        assert iterations == sorted(set(iterations))
        assert 1 <= iterations[0] and iterations[-1] <= max_iterations
        assert all(is_problem and is_results for _, is_problem, is_results in seen)


class TestTrajOptIfoptConstraintFactories:
    """The planner's term factories from trajopt_ifopt_utils.h (gh-309)."""

    def test_joint_velocity_constraint_matches_hand_built(self, kuka_iiwa_environment):
        t_env, _, joint_names = kuka_iiwa_environment
        manip = t_env.getKinematicGroup("manipulator")
        states = [START_POS, 0.5 * (START_POS + END_POS), END_POS]
        nodes = ti.createNodesVariables(
            "trajectory", joint_names, states, ti.toBounds(manip.getLimits().joint_limits)
        )
        variables = [node.getVar("joints") for node in nodes.getNodes()]
        target = np.zeros(len(joint_names))
        coeffs = np.ones(len(joint_names))

        factory_built = createJointVelocityConstraint(target, variables, coeffs)
        hand_built = ti.JointVelConstraint(target, variables, coeffs, "Velocity")
        problem = tsqp.TrajOptQPProblem(nodes)
        problem.addConstraintSet(factory_built)
        problem.addConstraintSet(hand_built)
        problem.setup()

        np.testing.assert_array_equal(factory_built.getValues(), hand_built.getValues())
        np.testing.assert_array_equal(factory_built.getCoefficients(), hand_built.getCoefficients())
        assert [(b.getLower(), b.getUpper()) for b in factory_built.getBounds()] == [
            (b.getLower(), b.getUpper()) for b in hand_built.getBounds()
        ]

    def test_empty_variables_are_rejected(self):
        with pytest.raises(ValueError, match="vars is empty"):
            createJointVelocityConstraint(np.zeros(7), [], np.ones(7))

    def test_collision_constraints_flag_the_colliding_step(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        manip = t_env.getKinematicGroup("manipulator")
        # An obstacle cube centred on the tool at END_POS: that step collides, START_POS does not.
        obstacle = Link("obstacle")
        collision = Collision()
        collision.geometry = Box(0.1, 0.1, 0.1)
        obstacle.addCollision(collision)
        joint = Joint("obstacle_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "base_link"
        joint.child_link_name = "obstacle"
        joint.parent_to_joint_origin_transform = manip.calcFwdKin(END_POS)["tool0"]
        assert t_env.applyCommand(AddLinkCommand(obstacle, joint))

        nodes = ti.createNodesVariables(
            "trajectory",
            joint_names,
            [START_POS, END_POS],
            ti.toBounds(manip.getLimits().joint_limits),
        )
        variables = [node.getVar("joints") for node in nodes.getNodes()]
        config = TrajOptCollisionConfig(0.05, 20.0)

        constraints = createCollisionConstraints(variables, t_env, manip_info, config, [], False)
        assert len(constraints) == 2  # DISCRETE: one set per unfixed step
        problem = tsqp.TrajOptQPProblem(nodes)
        for constraint in constraints:
            problem.addConstraintSet(constraint)
        problem.setup()

        free, colliding = constraints
        assert np.all(free.getValues() <= 0.0)
        assert np.max(colliding.getValues()) > 0.0

    def test_fixed_steps_get_no_collision_constraint(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        manip = t_env.getKinematicGroup("manipulator")
        nodes = ti.createNodesVariables(
            "trajectory",
            joint_names,
            [START_POS, END_POS, START_POS],
            ti.toBounds(manip.getLimits().joint_limits),
        )
        variables = [node.getVar("joints") for node in nodes.getNodes()]
        constraints = createCollisionConstraints(
            variables, t_env, manip_info, TrajOptCollisionConfig(0.05, 20.0), [0, 2], False
        )
        assert len(constraints) == 1


class TestTrajOptIfoptMotionPlannerBase:
    """`TrajOptIfoptMotionPlanner` is a `MotionPlanner` (gh-299)."""

    def test_isinstance(self):
        assert isinstance(TrajOptIfoptMotionPlanner("x"), MotionPlanner)

    def test_base_solve_dispatches(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        planner = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE)
        via_base = MotionPlanner.solve(
            planner, _request(t_env, manip_info, joint_names, TrajOptIfoptOSQPSolverProfile())
        )
        direct = planner.solve(
            _request(t_env, manip_info, joint_names, TrajOptIfoptOSQPSolverProfile())
        )
        assert via_base.successful, via_base.message
        assert via_base.message == direct.message
        assert len(via_base.results) == len(direct.results)


class TestTrajOptIfoptProfileHooks:
    """The profiles' `create` hooks and the structs they return (gh-339)."""

    @staticmethod
    def _move_profile():
        profile = TrajOptIfoptDefaultMoveProfile()
        profile.joint_cost_config.enabled = False
        profile.joint_constraint_config.enabled = True
        return profile

    def test_move_profile_create(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        info = self._move_profile().create(
            _joint_move(joint_names, START_POS), manip_info, t_env, 3
        )
        assert isinstance(info, TrajOptIfoptWaypointInfo)
        assert isinstance(info.term_infos, TrajOptIfoptTermInfos)
        assert info.fixed is True  # constrained, untoleranced joint waypoint
        assert info.node.getName() == "Node_3"
        np.testing.assert_array_equal(info.node.getVar("position").value(), START_POS)
        assert len(info.term_infos.constraints) == 1
        assert len(info.term_infos.squared_costs) == 0
        assert len(info.term_infos.absolute_costs) == 0
        assert len(info.term_infos.hinge_costs) == 0

    def test_node_keeps_waypoint_info_alive(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        info = self._move_profile().create(
            _joint_move(joint_names, START_POS), manip_info, t_env, 5
        )
        refs = sys.getrefcount(info)
        node = info.node
        assert sys.getrefcount(info) == refs + 1
        del info
        gc.collect()
        assert node.getName() == "Node_5"
        np.testing.assert_array_equal(node.getVar("position").value(), START_POS)

    def test_node_is_read_only(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        info = self._move_profile().create(
            _joint_move(joint_names, START_POS), manip_info, t_env, 0
        )
        with pytest.raises(AttributeError):
            info.node = ti.Node("other")

    def test_term_set_outlives_waypoint_info(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        info = self._move_profile().create(
            _joint_move(joint_names, START_POS), manip_info, t_env, 0
        )
        constraint = info.term_infos.constraints[0]
        del info
        gc.collect()
        assert len(constraint.getCoefficients()) == len(joint_names)

    def test_python_constraint_set_survives_term_infos(self):
        class Tagged(ti.ConstraintSet):
            def __init__(self):
                super().__init__("tagged", 1)
                self.tag = "mine"

            def getValues(self):
                return np.zeros(1)

            def getBounds(self):
                return [ti.Bounds(0.0, 0.0)]

            def fillJacobianBlock(self, var_set, jac_block):
                pass

        infos = TrajOptIfoptTermInfos()
        infos.constraints = [Tagged()]
        gc.collect()
        constraint = infos.constraints[0]
        assert type(constraint) is Tagged
        assert constraint.tag == "mine"
        del infos
        gc.collect()
        assert type(constraint) is Tagged
        assert constraint.tag == "mine"
        np.testing.assert_array_equal(constraint.getValues(), np.zeros(1))

    def test_default_waypoint_info_has_no_node(self):
        info = TrajOptIfoptWaypointInfo()
        assert info.node is None
        assert info.fixed is False

    def test_composite_profile_create(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        program = _interpolated_program(t_env, manip_info, joint_names)
        moves = _move_instructions(program)
        move_profile = self._move_profile()
        infos = [move_profile.create(move, manip_info, t_env, i) for i, move in enumerate(moves)]
        nodes = [info.node for info in infos]
        fixed = [i for i, info in enumerate(infos) if info.fixed]
        assert fixed == [0, len(moves) - 1]

        composite = _smoothing_composite_profile(len(joint_names))
        terms = composite.create(manip_info, t_env, nodes, fixed)

        # The planner adds these sets as they are; the factories build the same sets.
        variables = [node.getVar("position") for node in nodes]
        expected_constraints = createCollisionConstraints(
            variables, t_env, manip_info, composite.collision_constraint_config, fixed, False
        )
        expected_hinge = createCollisionConstraints(
            variables, t_env, manip_info, composite.collision_cost_config, fixed, False
        )
        assert len(terms.constraints) == len(expected_constraints) == len(moves) - 2
        assert len(terms.hinge_costs) == len(expected_hinge) == len(moves) - 2
        assert len(terms.squared_costs) == 3  # velocity, acceleration, jerk smoothing
        assert len(terms.absolute_costs) == 0

    def test_composite_profile_rejects_none_node(self, kuka_iiwa_environment):
        t_env, manip_info, _ = kuka_iiwa_environment
        with pytest.raises(TypeError, match=r"nodes\[0\] is None"):
            TrajOptIfoptDefaultCompositeProfile().create(manip_info, t_env, [None], [])

    def test_solver_profile_create(self):
        profile = TrajOptIfoptOSQPSolverProfile()
        profile.opt_params.max_iterations = 7
        solver = profile.create()
        assert isinstance(solver, tsqp.TrustRegionSQPSolver)
        assert solver.params.max_iterations == 7

    def test_hooks_on_base_classes(self):
        assert "create" in TrajOptIfoptMoveProfile.__dict__
        assert "create" in TrajOptIfoptCompositeProfile.__dict__
        assert "create" in TrajOptIfoptSolverProfile.__dict__


class TestTrajOptIfoptValueEquality:
    """Value `__eq__` on configs and profiles; unhashable (gh-261)."""

    @pytest.mark.parametrize(
        ("cls", "mutate"),
        [
            (TrajOptIfoptCartesianWaypointConfig, lambda x: setattr(x, "enabled", not x.enabled)),
            (TrajOptIfoptJointWaypointConfig, lambda x: setattr(x, "enabled", not x.enabled)),
            (
                TrajOptIfoptDefaultMoveProfile,
                lambda x: setattr(x.joint_cost_config, "enabled", not x.joint_cost_config.enabled),
            ),
            (
                TrajOptIfoptDefaultCompositeProfile,
                lambda x: setattr(x, "smooth_jerks", not x.smooth_jerks),
            ),
            (TrajOptIfoptOSQPSolverProfile, lambda x: setattr(x.opt_params, "max_iterations", 7)),
            (TrajOptIfoptOSQPSolverProfile, lambda x: x.setPolish(False)),
        ],
    )
    def test_equality(self, cls, mutate):
        a, b = cls(), cls()
        assert a == b
        mutate(b)
        assert a != b
        with pytest.raises(TypeError):
            hash(a)


class TestTrajOptIfoptProfileCreateKey:
    """Static `createKey()` is the key the profile carries (gh-273)."""

    @pytest.mark.parametrize(
        "cls",
        [
            TrajOptIfoptDefaultMoveProfile,
            TrajOptIfoptDefaultCompositeProfile,
            TrajOptIfoptOSQPSolverProfile,
        ],
    )
    def test_create_key_matches_instance_key(self, cls):
        assert cls.createKey() == cls().getKey()

    def test_keys_differ_between_profile_kinds(self):
        keys = {
            TrajOptIfoptMoveProfile.createKey(),
            TrajOptIfoptCompositeProfile.createKey(),
            TrajOptIfoptSolverProfile.createKey(),
        }
        assert len(keys) == 3


class TestTrajOptIfoptProfileFromYaml:
    """`from_yaml` reads a profile with the C++ YAML constructor (gh-331)."""

    def test_move_profile(self):
        expected = TrajOptIfoptDefaultMoveProfile()
        expected.joint_cost_config.enabled = not expected.joint_cost_config.enabled
        text = f"joint_cost_config:\n  enabled: {str(expected.joint_cost_config.enabled).lower()}\n"
        assert TrajOptIfoptDefaultMoveProfile.from_yaml(text) == expected

    def test_composite_profile(self):
        expected = TrajOptIfoptDefaultCompositeProfile()
        expected.smooth_jerks = not expected.smooth_jerks
        text = f"smooth_jerks: {str(expected.smooth_jerks).lower()}\n"
        assert TrajOptIfoptDefaultCompositeProfile.from_yaml(text) == expected

    def test_osqp_solver_profile(self):
        expected = TrajOptIfoptOSQPSolverProfile()
        expected.opt_params.max_iterations = 7
        text = "opt_params:\n  max_iterations: 7\n"
        assert TrajOptIfoptOSQPSolverProfile.from_yaml(text) == expected

    def test_malformed_yaml_raises_value_error(self):
        with pytest.raises(ValueError):
            TrajOptIfoptDefaultCompositeProfile.from_yaml("smooth_jerks: [true")

    def test_wrong_value_type_raises_the_constructor_error(self):
        with pytest.raises(RuntimeError, match="Failed to parse yaml config"):
            TrajOptIfoptDefaultCompositeProfile.from_yaml("smooth_jerks: not-a-bool\n")


# ---- Python-authored profiles (gh-340) -------------------------------------


class _ZeroCost(ti.ConstraintSet):
    """A one-row squared cost that is always zero; counts `getValues` calls."""

    def __init__(self):
        super().__init__("python_zero_cost", 1)
        self.value_calls = 0

    def getValues(self):
        self.value_calls += 1
        return np.zeros(1)

    def getBounds(self):
        return [ti.Bounds(0.0, 0.0)]

    def getJacobian(self):
        return scipy.sparse.csr_matrix((1, self.getVariables().getRows()))

    def update(self):
        return self.getRows()

    def getCoefficients(self):
        return np.ones(1)


class _ProfileError(Exception):
    """Raised by a Python profile hook; a type no binding translates to."""


class _DelegatingMoveProfile(TrajOptIfoptMoveProfile):
    def __init__(self):
        super().__init__()
        self.indices = []
        self._inner = TrajOptIfoptDefaultMoveProfile()
        self._inner.joint_cost_config.enabled = False
        self._inner.joint_constraint_config.enabled = True

    def create(self, move_instruction, composite_manip_info, env, index):
        self.indices.append(index)
        return self._inner.create(move_instruction, composite_manip_info, env, index)


class _AddingCompositeProfile(TrajOptIfoptCompositeProfile):
    """The smoothing profile's terms plus one Python cost."""

    def __init__(self, n_joints):
        super().__init__()
        self.cost = _ZeroCost()
        self.seen = None
        self._inner = _smoothing_composite_profile(n_joints)

    def create(self, composite_manip_info, env, nodes, fixed_indices):
        self.seen = (len(nodes), list(fixed_indices))
        terms = self._inner.create(composite_manip_info, env, nodes, fixed_indices)
        terms.squared_costs = [*terms.squared_costs, self.cost]
        return terms


def _python_profile_request(t_env, manip_info, joint_names, move=None, composite=None):
    request = _request(t_env, manip_info, joint_names, TrajOptIfoptOSQPSolverProfile())
    if move is not None:
        ProfileDictionary_addTrajOptIfoptMoveProfile(
            request.profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", move
        )
    if composite is not None:
        ProfileDictionary_addTrajOptIfoptCompositeProfile(
            request.profiles, TRAJOPT_IFOPT_NAMESPACE, "DEFAULT", composite
        )
    return request


# s: a child loads an environment and runs one TrajOpt-Ifopt solve (the task-composer child also
# loads the pipeline plugins, about 5 s); generous for a loaded CI runner
CHILD_TIMEOUT_S = 300.0

_CHILD_PREAMBLE = textwrap.dedent(
    """
    import gc

    import numpy as np
    import scipy.sparse

    from tesseract_robotics import trajopt_ifopt as ti
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        TrajOptIfoptCompositeProfile,
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptDefaultMoveProfile,
        TrajOptIfoptMoveProfile,
    )

    released = []

    class ZeroCost(ti.ConstraintSet):
        def __init__(self):
            super().__init__("python_zero_cost", 1)
            self.value_calls = 0

        def __del__(self):
            released.append(self.value_calls)

        def getValues(self):
            self.value_calls += 1
            return np.zeros(1)

        def getBounds(self):
            return [ti.Bounds(0.0, 0.0)]

        def getJacobian(self):
            return scipy.sparse.csr_matrix((1, self.getVariables().getRows()))

        def update(self):
            return self.getRows()

        def getCoefficients(self):
            return np.ones(1)
    """
)

# The planning API runs TrajOptIfoptPipeline on the task composer's worker threads: the Python
# composite profile's create, the Python cost's calls and, once the result is dropped, the
# cost's release all happen off the main thread.
_TASK_COMPOSER_RELEASE = textwrap.dedent(
    """
    from tesseract_robotics.planning import (
        JointTarget,
        MotionProgram,
        Robot,
        TaskComposer,
        create_trajopt_ifopt_default_profiles,
    )
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        ProfileDictionary_addTrajOptIfoptCompositeProfile,
    )

    class AddingComposite(TrajOptIfoptCompositeProfile):
        def __init__(self):
            super().__init__()
            self.calls = 0
            self._inner = TrajOptIfoptDefaultCompositeProfile()

        def create(self, composite_manip_info, env, nodes, fixed_indices):
            self.calls += 1
            terms = self._inner.create(composite_manip_info, env, nodes, fixed_indices)
            terms.squared_costs = [*terms.squared_costs, ZeroCost()]
            return terms

    robot = Robot.from_tesseract_support("abb_irb2400")
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .move_to(JointTarget([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        .move_to(JointTarget([0.5, 0.2, -0.3, 0.4, 0.6, -0.2]))
    )
    profiles = create_trajopt_ifopt_default_profiles()
    composite = AddingComposite()
    ProfileDictionary_addTrajOptIfoptCompositeProfile(
        profiles, "TrajOptIfoptMotionPlannerTask", "DEFAULT", composite
    )
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptIfoptPipeline", profiles=profiles)
    assert result.successful, result.message
    assert composite.calls == 1, composite.calls
    del result, profiles, composite
    gc.collect()
    assert len(released) == 1 and released[0] > 0, released
    print("OK")
    """
)

# A move profile that keeps its waypoint info would keep a Node the planner has taken; the
# trampoline refuses such an info before the planner moves the node out.
_KEPT_WAYPOINT_INFO = textwrap.dedent(
    """
    from pathlib import Path

    from tesseract_robotics.tesseract_command_language import (
        CompositeInstruction,
        JointWaypoint,
        JointWaypointPoly_wrap_JointWaypoint,
        MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction,
        MoveInstructionType_FREESPACE,
        ProfileDictionary,
    )
    from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
    from tesseract_robotics.tesseract_environment import Environment
    from tesseract_robotics.tesseract_motion_planners import PlannerRequest
    from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        ProfileDictionary_addTrajOptIfoptCompositeProfile,
        ProfileDictionary_addTrajOptIfoptMoveProfile,
        ProfileDictionary_addTrajOptIfoptSolverProfile,
        TrajOptIfoptMotionPlanner,
        TrajOptIfoptOSQPSolverProfile,
        TrajOptIfoptWaypointInfoInUseError,
    )

    class Keeper(TrajOptIfoptMoveProfile):
        def __init__(self, keep):
            super().__init__()
            self.keep = keep
            self.kept = []
            self._inner = TrajOptIfoptDefaultMoveProfile()

        def create(self, move_instruction, composite_manip_info, env, index):
            info = self._inner.create(move_instruction, composite_manip_info, env, index)
            if self.keep == "info":
                self.kept.append(info)
            elif self.keep == "node":
                self.kept.append(info.node)
            return info

    locator = GeneralResourceLocator()
    support = "package://tesseract/support/urdf/lbr_iiwa_14_r820"
    env = Environment()
    assert env.init(
        Path(locator.locateResource(support + ".urdf").getFilePath()),
        Path(locator.locateResource(support + ".srdf").getFilePath()),
        locator,
    )
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    joints = list(env.getJointGroup("manipulator").getJointNames())

    def move(position):
        return MoveInstructionPoly_wrap_MoveInstruction(
            MoveInstruction(
                JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joints, np.array(position))),
                MoveInstructionType_FREESPACE,
                "DEFAULT",
            )
        )

    for keep in ("info", "node"):
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(move([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]))
        program.appendMoveInstruction(move([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0]))
        profiles = ProfileDictionary()
        keeper = Keeper(keep)
        ns = "TrajOptIfoptMotionPlannerTask"
        ProfileDictionary_addTrajOptIfoptMoveProfile(profiles, ns, "DEFAULT", keeper)
        ProfileDictionary_addTrajOptIfoptCompositeProfile(
            profiles, ns, "DEFAULT", TrajOptIfoptDefaultCompositeProfile()
        )
        ProfileDictionary_addTrajOptIfoptSolverProfile(
            profiles, ns, "DEFAULT", TrajOptIfoptOSQPSolverProfile()
        )
        request = PlannerRequest()
        request.env = env
        request.instructions = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 5)
        request.profiles = profiles
        try:
            TrajOptIfoptMotionPlanner(ns).solve(request)
        except TrajOptIfoptWaypointInfoInUseError as error:
            assert "create" in str(error), str(error)
        else:
            raise AssertionError(f"keeping the {keep} was not refused")
        del request, profiles
        gc.collect()
        # what the override kept is still whole: the planner never took its node
        if keep == "info":
            assert keeper.kept[0].node.getName() == "Node_0"
        else:
            assert keeper.kept[0].getName() == "Node_0"
    print("OK")
    """
)


# create is pure virtual on both bases: super().create from a Python override must raise, not
# dispatch back into the override.
_SUPER_CREATE = textwrap.dedent(
    """
    from pathlib import Path

    from tesseract_robotics.tesseract_command_language import (
        JointWaypoint,
        JointWaypointPoly_wrap_JointWaypoint,
        MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction,
        MoveInstructionType_FREESPACE,
    )
    from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
    from tesseract_robotics.tesseract_environment import Environment
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        TrajOptIfoptPureVirtualCallError,
    )

    class MoveCallsUp(TrajOptIfoptMoveProfile):
        def create(self, move_instruction, composite_manip_info, env, index):
            return super().create(move_instruction, composite_manip_info, env, index)

    class CompositeCallsUp(TrajOptIfoptCompositeProfile):
        def create(self, composite_manip_info, env, nodes, fixed_indices):
            return super().create(composite_manip_info, env, nodes, fixed_indices)

    locator = GeneralResourceLocator()
    support = "package://tesseract/support/urdf/lbr_iiwa_14_r820"
    env = Environment()
    assert env.init(
        Path(locator.locateResource(support + ".urdf").getFilePath()),
        Path(locator.locateResource(support + ".srdf").getFilePath()),
        locator,
    )
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    joints = list(env.getJointGroup("manipulator").getJointNames())
    move = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joints, np.zeros(len(joints)))),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
    )
    for call in (
        lambda: MoveCallsUp().create(move, manip_info, env, 0),
        lambda: CompositeCallsUp().create(manip_info, env, [], []),
    ):
        try:
            call()
        except TrajOptIfoptPureVirtualCallError as error:
            assert isinstance(error, NotImplementedError)
            assert "create" in str(error), str(error)
        else:
            raise AssertionError("super().create did not raise")
    print("OK")
    """
)


def _run_child(body):
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD_PREAMBLE + body],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
    )
    traceback_at = proc.stderr.find("Traceback")
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[traceback_at:] if traceback_at >= 0 else proc.stderr}"
    )
    assert "OK" in proc.stdout.splitlines(), proc.stdout


class TestTrajOptIfoptPythonProfiles:
    """Python subclasses of the move and composite profiles plan (gh-340)."""

    def test_python_composite_profile_adds_a_constraint_set(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        composite = _AddingCompositeProfile(len(joint_names))
        request = _python_profile_request(t_env, manip_info, joint_names, composite=composite)
        n_moves = len(_move_instructions(request.instructions))
        response = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)
        assert response.successful, response.message
        assert composite.seen == (n_moves, [0, n_moves - 1])
        assert composite.cost.value_calls > 0

    def test_python_move_profile_plans(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment
        move = _DelegatingMoveProfile()
        request = _python_profile_request(t_env, manip_info, joint_names, move=move)
        n_moves = len(_move_instructions(request.instructions))
        response = TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)
        assert response.successful, response.message
        assert move.indices == list(range(n_moves))

    def test_move_create_exception_propagates(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class Raising(TrajOptIfoptMoveProfile):
            def create(self, move_instruction, composite_manip_info, env, index):
                raise _ProfileError("from move create")

        request = _python_profile_request(t_env, manip_info, joint_names, move=Raising())
        with pytest.raises(_ProfileError, match="from move create"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    def test_composite_create_exception_propagates(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class Raising(TrajOptIfoptCompositeProfile):
            def create(self, composite_manip_info, env, nodes, fixed_indices):
                raise _ProfileError("from composite create")

        request = _python_profile_request(t_env, manip_info, joint_names, composite=Raising())
        with pytest.raises(_ProfileError, match="from composite create"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    def test_move_create_without_a_node_raises(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class NoNode(TrajOptIfoptMoveProfile):
            def create(self, move_instruction, composite_manip_info, env, index):
                return TrajOptIfoptWaypointInfo()

        request = _python_profile_request(t_env, manip_info, joint_names, move=NoNode())
        with pytest.raises(TrajOptIfoptWaypointInfoWithoutNodeError, match="node"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    def test_move_create_wrong_return_type_raises(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class WrongType(TrajOptIfoptMoveProfile):
            def create(self, move_instruction, composite_manip_info, env, index):
                return None

        request = _python_profile_request(t_env, manip_info, joint_names, move=WrongType())
        with pytest.raises(TrajOptIfoptProfileReturnTypeError, match="TrajOptIfoptWaypointInfo"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    def test_hook_without_override_raises(self, kuka_iiwa_environment):
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class Bare(TrajOptIfoptCompositeProfile):
            pass

        request = _python_profile_request(t_env, manip_info, joint_names, composite=Bare())
        with pytest.raises(RuntimeError, match="pure virtual"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    @pytest.mark.parametrize("mutation", ["addVar", "setVariables"])
    def test_composite_create_changing_nodes_raises(self, kuka_iiwa_environment, mutation):
        # the planner laid its variables out and seeded them before create: a grown node is a
        # meaningless layout, and changed values would be silently overwritten
        t_env, manip_info, joint_names = kuka_iiwa_environment

        class Mutating(TrajOptIfoptCompositeProfile):
            def __init__(self):
                super().__init__()
                self._inner = _smoothing_composite_profile(len(joint_names))

            def create(self, composite_manip_info, env, nodes, fixed_indices):
                if mutation == "addVar":
                    nodes[1].addVar("extra", ["e0", "e1"], np.ones(2), [ti.Bounds(-1.0, 1.0)] * 2)
                else:
                    nodes[1].setVariables(np.full(len(nodes) * len(joint_names), 0.25))
                return self._inner.create(composite_manip_info, env, nodes, fixed_indices)

        request = _python_profile_request(t_env, manip_info, joint_names, composite=Mutating())
        with pytest.raises(TrajOptIfoptNodeLayoutChangedError, match="node 1"):
            TrajOptIfoptMotionPlanner(TRAJOPT_IFOPT_NAMESPACE).solve(request)

    def test_super_create_raises_pure_virtual_call_error(self):
        _run_child(_SUPER_CREATE)

    def test_kept_waypoint_info_is_refused(self):
        _run_child(_KEPT_WAYPOINT_INFO)

    def test_python_constraint_set_released_after_task_composer_solve(self):
        _run_child(_TASK_COMPOSER_RELEASE)


class TestTrajOptIfoptConcreteProfileOverrides:
    """A hook override on a concrete profile is refused, not silently ignored (gh-340)."""

    @pytest.mark.parametrize(
        ("concrete", "hook", "base"),
        [
            (TrajOptIfoptDefaultMoveProfile, "create", "TrajOptIfoptMoveProfile"),
            (TrajOptIfoptDefaultCompositeProfile, "create", "TrajOptIfoptCompositeProfile"),
            (TrajOptIfoptOSQPSolverProfile, "create", "fields"),
            (TrajOptIfoptOSQPSolverProfile, "createOptimizationCallbacks", "callbacks"),
        ],
    )
    def test_hook_override_raises(self, concrete, hook, base):
        with pytest.raises(ProfileOverrideNotDispatchedError, match=base):
            type("Overrides", (concrete,), {hook: lambda self, *args: None})

    def test_subclass_without_hooks_is_allowed(self):
        class Configured(TrajOptIfoptDefaultCompositeProfile):
            pass

        assert isinstance(Configured(), TrajOptIfoptCompositeProfile)

    def test_abstract_base_override_is_allowed(self):
        assert issubclass(_AddingCompositeProfile, TrajOptIfoptCompositeProfile)
