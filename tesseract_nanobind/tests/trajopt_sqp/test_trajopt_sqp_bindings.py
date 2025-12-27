"""Tests for low-level trajopt_sqp bindings (SQP solver API)."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import Environment

# Low-level SQP imports
try:
    from tesseract_robotics import trajopt_ifopt as ti
    from tesseract_robotics import trajopt_sqp as tsqp

    SQP_AVAILABLE = True
except ImportError:
    SQP_AVAILABLE = False


@pytest.fixture
def kuka_setup():
    """Load KUKA IIWA robot environment."""
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

    env = Environment()
    assert env.init(urdf_path, srdf_path, locator)

    manip = env.getKinematicGroup("manipulator")
    joint_names = list(manip.getJointNames())
    joint_limits = manip.getLimits().joint_limits

    return env, manip, joint_names, joint_limits


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestIfoptBaseClasses:
    """Test ifopt base class bindings."""

    def test_bounds_creation(self):
        """Test ifopt.Bounds creation."""
        from tesseract_robotics import ifopt

        # Bounded
        b = ifopt.Bounds(-1.0, 1.0)
        assert b.lower == -1.0
        assert b.upper == 1.0

    def test_variable_set_interface(self, kuka_setup):
        """Test JointPosition inherits from VariableSet."""
        _, _, joint_names, joint_limits = kuka_setup

        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "test_var")

        # VariableSet interface
        assert var.GetRows() == len(joint_names)


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestTrajOptIfoptTypes:
    """Test trajopt_ifopt constraint and variable types."""

    def test_joint_position(self, kuka_setup):
        """Test JointPosition variable creation."""
        _, _, joint_names, joint_limits = kuka_setup

        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")

        assert var.GetRows() == len(joint_names)

        # Test SetBounds
        var.SetBounds(joint_limits)

    def test_interpolate(self):
        """Test interpolate utility function."""
        start = np.array([0.0, 0.0, 0.0])
        end = np.array([1.0, 2.0, 3.0])
        steps = 5

        states = ti.interpolate(start, end, steps)

        assert len(states) == steps
        np.testing.assert_array_almost_equal(states[0], start)
        np.testing.assert_array_almost_equal(states[-1], end)

    def test_to_bounds(self, kuka_setup):
        """Test toBounds conversion."""
        _, _, _, joint_limits = kuka_setup

        bounds = ti.toBounds(joint_limits)
        assert len(bounds) == joint_limits.shape[0]

    def test_cart_pos_info(self, kuka_setup):
        """Test CartPosInfo creation."""
        _, manip, _, _ = kuka_setup

        info = ti.CartPosInfo()
        info.manip = manip
        info.source_frame = "tool0"
        info.target_frame = "world"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset = Isometry3d.Identity()
        info.type = ti.CartPosInfoType.TARGET_ACTIVE
        info.indices = np.array([0, 1, 2, 3, 4, 5], dtype=np.int32)

        assert info.source_frame == "tool0"

    def test_collision_config(self):
        """Test TrajOptCollisionConfig creation."""
        config = ti.TrajOptCollisionConfig(0.1, 10.0)
        assert config is not None


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestTrajOptSQPTypes:
    """Test trajopt_sqp solver types."""

    def test_osqp_solver_creation(self):
        """Test OSQPEigenSolver creation."""
        solver = tsqp.OSQPEigenSolver()
        assert solver is not None

    def test_trust_region_solver_creation(self):
        """Test TrustRegionSQPSolver creation."""
        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        assert solver is not None

        # Check params access
        assert hasattr(solver, "params")
        solver.params.initial_trust_box_size = 0.1

    def test_ifopt_qp_problem(self, kuka_setup):
        """Test IfoptQPProblem creation."""
        _, _, joint_names, joint_limits = kuka_setup

        problem = tsqp.IfoptQPProblem()

        # Add a variable set
        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)

        problem.addVariableSet(var)

    def test_cost_penalty_types(self):
        """Test CostPenaltyType enum."""
        assert tsqp.CostPenaltyType.SQUARED is not None
        assert tsqp.CostPenaltyType.ABSOLUTE is not None
        assert tsqp.CostPenaltyType.HINGE is not None


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestSQPIntegration:
    """Integration tests for SQP solver."""

    def test_simple_optimization(self, kuka_setup):
        """Test a simple optimization problem."""
        _, manip, joint_names, joint_limits = kuka_setup

        # Start and target positions
        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        steps = 5

        # Interpolate initial trajectory
        initial_states = ti.interpolate(start_pos, target_pos, steps)

        # Create problem
        problem = tsqp.IfoptQPProblem()

        # Add joint position variables
        vars_list = []
        for i, state in enumerate(initial_states):
            var = ti.JointPosition(state, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)
            problem.addVariableSet(var)

        # Add start position constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(start_pos, [vars_list[0]], home_coeffs, "Home")
        problem.addConstraintSet(home_constraint)

        # Add velocity cost
        vel_target = np.zeros(len(joint_names))
        vel_cost = ti.JointVelConstraint(vel_target, vars_list, np.ones(1), "Velocity")
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)

        # Setup and solve
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

        solver.solve(problem)
        results = solver.getResults()

        assert results.best_var_vals is not None
        # Result includes joint vars + slack vars for constraints
        assert len(results.best_var_vals) >= len(joint_names) * steps
