"""Tests for tesseract_motion_planners_trajopt_ifopt bindings."""

import numpy as np
import pytest

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

# TrajOptIfopt imports
try:
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        ProfileDictionary_addTrajOptIfoptCompositeProfile,
        ProfileDictionary_addTrajOptIfoptPlanProfile,
        ProfileDictionary_addTrajOptIfoptSolverProfile,
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptDefaultPlanProfile,
        TrajOptIfoptMotionPlanner,
        TrajOptIfoptOSQPSolverProfile,
    )

    TRAJOPT_IFOPT_AVAILABLE = True
except ImportError:
    TRAJOPT_IFOPT_AVAILABLE = False


TRAJOPT_IFOPT_NAMESPACE = "TrajOptIfoptMotionPlannerTask"


@pytest.fixture
def kuka_iiwa_environment():
    """Load KUKA IIWA robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(
        locator.locateResource(
            "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
        ).getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource(
            "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"
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


@pytest.mark.skipif(not TRAJOPT_IFOPT_AVAILABLE, reason="TrajOptIfopt not available")
class TestTrajOptIfoptProfiles:
    """Test TrajOptIfopt profile types."""

    def test_default_plan_profile(self):
        profile = TrajOptIfoptDefaultPlanProfile()
        assert profile is not None
        assert profile.getKey() is not None
        assert TrajOptIfoptDefaultPlanProfile.getStaticKey() is not None

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


@pytest.mark.skipif(not TRAJOPT_IFOPT_AVAILABLE, reason="TrajOptIfopt not available")
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
        cur_state = t_env.getState()
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
