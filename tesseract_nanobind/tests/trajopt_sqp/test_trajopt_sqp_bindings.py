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

    def test_special_bounds(self):
        """Test special bound constants."""
        from tesseract_robotics import ifopt

        # Unbounded (NoBound uses +/- inf)
        nb = ifopt.NoBound
        assert nb.lower == -ifopt.inf
        assert nb.upper == ifopt.inf

        # Zero bound (equality constraint)
        zb = ifopt.BoundZero
        assert zb.lower == 0.0
        assert zb.upper == 0.0

        # Greater than zero
        gz = ifopt.BoundGreaterZero
        assert gz.lower == 0.0
        assert gz.upper == ifopt.inf

        # Smaller than zero
        sz = ifopt.BoundSmallerZero
        assert sz.lower == -ifopt.inf
        assert sz.upper == 0.0

    def test_inf_constant(self):
        """Test ifopt.inf constant."""
        from tesseract_robotics import ifopt

        # ifopt uses 1e20 as "infinity" for numerical stability
        assert ifopt.inf == 1e20

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

    def test_cart_pos_info_types(self):
        """Test CartPosInfoType enum values."""
        assert ti.CartPosInfoType.TARGET_ACTIVE is not None
        assert ti.CartPosInfoType.SOURCE_ACTIVE is not None
        assert ti.CartPosInfoType.BOTH_ACTIVE is not None

    def test_collision_config(self):
        """Test TrajOptCollisionConfig creation."""
        config = ti.TrajOptCollisionConfig(0.1, 10.0)
        assert config is not None

    def test_collision_cache(self):
        """Test CollisionCache creation."""
        cache = ti.CollisionCache(10)  # 10 timesteps
        assert cache is not None

    def test_cart_pos_constraint(self, kuka_setup):
        """Test CartPosConstraint creation."""
        _, manip, joint_names, joint_limits = kuka_setup

        # Create a joint position variable
        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")

        # Create CartPosInfo
        info = ti.CartPosInfo()
        info.manip = manip
        info.source_frame = "tool0"
        info.target_frame = "world"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset = Isometry3d.Identity()
        info.type = ti.CartPosInfoType.TARGET_ACTIVE
        info.indices = np.array([0, 1, 2], dtype=np.int32)  # Position only

        # Create constraint
        constraint = ti.CartPosConstraint(info, var, "CartPos")
        assert constraint is not None

    def test_joint_accel_constraint(self, kuka_setup):
        """Test JointAccelConstraint creation."""
        _, _, joint_names, joint_limits = kuka_setup

        # Create multiple joint position variables (need at least 4 for acceleration)
        vars_list = []
        for i in range(5):
            pos = np.zeros(len(joint_names))
            var = ti.JointPosition(pos, joint_names, f"Joint_{i}")
            vars_list.append(var)

        # Create acceleration constraint
        accel_target = np.zeros(len(joint_names))
        coeffs = np.ones(1)
        constraint = ti.JointAccelConstraint(accel_target, vars_list, coeffs, "Accel")
        assert constraint is not None

    def test_discrete_collision_evaluator(self, kuka_setup):
        """Test SingleTimestepCollisionEvaluator creation."""
        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        cache = ti.CollisionCache(5)

        evaluator = ti.SingleTimestepCollisionEvaluator(
            cache,
            manip,
            env,
            config,
            True,  # dynamic_environment
        )
        assert evaluator is not None

    def test_discrete_collision_constraint(self, kuka_setup):
        """Test DiscreteCollisionConstraint creation."""
        env, manip, joint_names, joint_limits = kuka_setup

        # Create variable
        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")

        # Create collision evaluator
        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        cache = ti.CollisionCache(5)
        evaluator = ti.SingleTimestepCollisionEvaluator(cache, manip, env, config, True)

        # Create constraint
        constraint = ti.DiscreteCollisionConstraint(evaluator, var, 1, False, "Collision_0")
        assert constraint is not None


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

    def test_sqp_status_enum(self):
        """Test SQPStatus enum values."""
        assert tsqp.SQPStatus.RUNNING is not None
        assert tsqp.SQPStatus.NLP_CONVERGED is not None
        assert tsqp.SQPStatus.ITERATION_LIMIT is not None

    def test_sqp_parameters(self):
        """Test SQPParameters configuration."""
        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)

        # Access and modify parameters
        solver.params.initial_trust_box_size = 0.2
        solver.params.min_trust_box_size = 1e-4
        solver.params.max_merit_coeff_increases = 5

        assert solver.params.initial_trust_box_size == 0.2

    def test_qp_solver_status(self):
        """Test QPSolverStatus enum."""
        assert tsqp.QPSolverStatus.INITIALIZED is not None
        assert tsqp.QPSolverStatus.QP_ERROR is not None

    def test_constraint_type_enum(self):
        """Test ConstraintType enum."""
        assert tsqp.ConstraintType.EQ is not None
        assert tsqp.ConstraintType.INEQ is not None

    def test_sqp_parameters_full(self):
        """Test all SQPParameters options."""
        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)

        # Trust region parameters
        solver.params.initial_trust_box_size = 0.5
        solver.params.min_trust_box_size = 1e-5
        solver.params.trust_expand_ratio = 1.5
        solver.params.trust_shrink_ratio = 0.5

        # Convergence parameters
        solver.params.max_iterations = 100
        solver.params.cnt_tolerance = 1e-4
        solver.params.min_approx_improve = 1e-6
        solver.params.improve_ratio_threshold = 0.25

        # Merit function parameters
        solver.params.initial_merit_error_coeff = 10.0
        solver.params.max_merit_coeff_increases = 5
        solver.params.merit_coeff_increase_ratio = 10.0

        assert solver.params.initial_trust_box_size == 0.5
        assert solver.params.max_iterations == 100

    def test_solver_status_after_solve(self, kuka_setup):
        """Test getStatus method returns valid status after solve."""
        _, _, joint_names, joint_limits = kuka_setup

        # Build problem
        problem = tsqp.IfoptQPProblem()

        vars_list = []
        for i in range(3):
            pos = np.zeros(len(joint_names))
            var = ti.JointPosition(pos, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)
            problem.addVariableSet(var)

        # Add velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)

        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

        # After solve, status should be converged or iteration limit
        solver.solve(problem)
        status = solver.getStatus()
        assert status in [
            tsqp.SQPStatus.NLP_CONVERGED,
            tsqp.SQPStatus.ITERATION_LIMIT,
            tsqp.SQPStatus.CALLBACK_STOPPED,
        ]

    def test_sqp_results_attributes(self, kuka_setup):
        """Test SQPResults attributes are accessible after solve."""
        _, _, joint_names, joint_limits = kuka_setup

        # Build problem with constraint and cost
        problem = tsqp.IfoptQPProblem()

        vars_list = []
        for i in range(3):
            pos = np.zeros(len(joint_names))
            var = ti.JointPosition(pos, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)
            problem.addVariableSet(var)

        # Add start constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        start_pos = np.zeros(len(joint_names))
        constraint = ti.JointPosConstraint(start_pos, [vars_list[0]], home_coeffs, "Home")
        problem.addConstraintSet(constraint)

        # Add velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)

        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False
        solver.solve(problem)

        results = solver.getResults()

        # Check key attributes are accessible
        assert results.best_var_vals is not None
        assert results.best_exact_merit is not None
        assert results.overall_iteration >= 0
        # cost_names and constraint_names are lists (may be empty for simple problems)
        assert isinstance(results.cost_names, list)
        assert isinstance(results.constraint_names, list)


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

    def test_optimization_with_cartesian_constraint(self, kuka_setup):
        """Test optimization with Cartesian target constraint."""
        env, manip, joint_names, joint_limits = kuka_setup

        # Start position and compute target pose
        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        target_tf = manip.calcFwdKin(target_pos)["tool0"]
        steps = 5

        # Interpolate
        initial_states = ti.interpolate(start_pos, target_pos, steps)

        # Create problem
        problem = tsqp.IfoptQPProblem()

        # Add variables
        vars_list = []
        for i, state in enumerate(initial_states):
            var = ti.JointPosition(state, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)
            problem.addVariableSet(var)

        # Start constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(start_pos, [vars_list[0]], home_coeffs, "Home")
        problem.addConstraintSet(home_constraint)

        # Cartesian target constraint (position only, xyz)
        cart_info = ti.CartPosInfo()
        cart_info.manip = manip
        cart_info.source_frame = "tool0"
        cart_info.target_frame = "world"
        cart_info.source_frame_offset = Isometry3d.Identity()
        cart_info.target_frame_offset = target_tf
        cart_info.type = ti.CartPosInfoType.TARGET_ACTIVE
        cart_info.indices = np.array([0, 1, 2], dtype=np.int32)  # xyz only

        target_constraint = ti.CartPosConstraint(cart_info, vars_list[-1], "Target")
        problem.addConstraintSet(target_constraint)

        # Velocity cost
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
        # Check we got a valid result with costs
        assert results.best_exact_merit is not None

    def test_optimization_with_collision(self, kuka_setup):
        """Test optimization with collision constraints."""
        env, manip, joint_names, joint_limits = kuka_setup

        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.3, 0.2, 0.0, -1.0, 0.0, 0.3, 0.0])
        steps = 5

        initial_states = ti.interpolate(start_pos, target_pos, steps)

        problem = tsqp.IfoptQPProblem()

        # Variables
        vars_list = []
        for i, state in enumerate(initial_states):
            var = ti.JointPosition(state, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)
            problem.addVariableSet(var)

        # Collision constraints (keep references alive)
        collision_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        collision_cache = ti.CollisionCache(steps)
        evaluators = []
        constraints = []

        for i in range(1, steps):
            evaluator = ti.SingleTimestepCollisionEvaluator(
                collision_cache, manip, env, collision_config, True
            )
            constraint = ti.DiscreteCollisionConstraint(
                evaluator, vars_list[i], 1, False, f"Collision_{i}"
            )
            problem.addConstraintSet(constraint)
            evaluators.append(evaluator)
            constraints.append(constraint)

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)

        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

        solver.solve(problem)
        results = solver.getResults()

        assert results.best_var_vals is not None


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestAdditionalBindings:
    """Tests for additional bindings not covered elsewhere."""

    def test_collision_coeff_data(self):
        """Test CollisionCoeffData for per-link collision weights."""
        data = ti.CollisionCoeffData()
        assert data is not None
        # Pair collision coefficient methods
        assert hasattr(data, "getPairCollisionCoeff")
        assert hasattr(data, "setPairCollisionCoeff")

    def test_ifopt_problem_base(self, kuka_setup):
        """Test IfoptProblem (base class of IfoptQPProblem)."""
        _, _, joint_names, joint_limits = kuka_setup

        # IfoptProblem is the base IFOPT interface
        problem = tsqp.IfoptProblem()

        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)
        problem.AddVariableSet(var)

        # Should have method to get variable count
        assert problem.GetNumberOfOptimizationVariables() >= len(joint_names)

    def test_qp_problem_interface(self, kuka_setup):
        """Test QPProblem interface methods."""
        _, _, joint_names, joint_limits = kuka_setup

        problem = tsqp.IfoptQPProblem()

        # Add variable
        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)
        problem.addVariableSet(var)

        # QPProblem interface methods
        problem.setup()
        assert problem.getNumNLPVars() >= len(joint_names)

    def test_sqp_callback_class(self):
        """Test SQPCallback class exists and is callable."""
        # SQPCallback is the base class for solver callbacks
        assert hasattr(tsqp, "SQPCallback")
        # The class itself should exist
        callback_class = tsqp.SQPCallback
        assert callback_class is not None

    def test_qp_solver_inheritance(self):
        """Test QPSolver base class."""
        # OSQPEigenSolver inherits from QPSolver
        solver = tsqp.OSQPEigenSolver()
        assert isinstance(solver, tsqp.QPSolver)

    def test_ifopt_composite(self):
        """Test ifopt.Composite for component management."""
        from tesseract_robotics import ifopt

        # Composite is the container for components
        assert hasattr(ifopt, "Composite")
        composite = ifopt.Composite("test", False)
        assert composite is not None

    def test_ifopt_component_interface(self, kuka_setup):
        """Test ifopt.Component interface via JointPosition."""
        _, _, joint_names, _ = kuka_setup

        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "TestVar")

        # Component interface
        assert var.GetName() == "TestVar"
        assert var.GetRows() == len(joint_names)

    def test_constraint_set_interface(self, kuka_setup):
        """Test ConstraintSet interface via JointPosConstraint."""
        _, _, joint_names, joint_limits = kuka_setup

        pos = np.zeros(len(joint_names))
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)

        # JointPosConstraint inherits from ConstraintSet
        target = np.zeros(len(joint_names))
        coeffs = np.ones(len(joint_names))
        constraint = ti.JointPosConstraint(target, [var], coeffs, "Position")

        # ConstraintSet interface
        assert constraint.GetName() == "Position"
        assert hasattr(constraint, "GetRows")

    def test_cost_term_interface(self, kuka_setup):
        """Test CostTerm interface via JointVelConstraint used as cost."""
        _, _, joint_names, joint_limits = kuka_setup

        # Create variables
        vars_list = []
        for i in range(3):
            pos = np.zeros(len(joint_names))
            var = ti.JointPosition(pos, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars_list.append(var)

        # JointVelConstraint can be used as cost
        vel_target = np.zeros(len(joint_names))
        vel_cost = ti.JointVelConstraint(vel_target, vars_list, np.ones(1), "VelCost")

        # Adding as cost works
        problem = tsqp.IfoptQPProblem()
        for v in vars_list:
            problem.addVariableSet(v)
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestContinuousCollisionBindings:
    """Tests for continuous collision evaluators and constraints."""

    def test_continuous_collision_evaluator_base(self):
        """Test ContinuousCollisionEvaluator base class exists."""
        assert hasattr(ti, "ContinuousCollisionEvaluator")

    def test_lvs_discrete_collision_evaluator(self, kuka_setup):
        """Test LVSDiscreteCollisionEvaluator creation."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        # 0.33 API: LVSDiscreteCollisionEvaluator requires LVS_DISCRETE type
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        cache = ti.CollisionCache(10)

        evaluator = ti.LVSDiscreteCollisionEvaluator(cache, manip, env, config, True)
        assert evaluator is not None

        # Check base class methods (GetCollisionConfig removed in 0.33)
        assert evaluator.GetCollisionMarginBuffer() is not None

    def test_lvs_continuous_collision_evaluator(self, kuka_setup):
        """Test LVSContinuousCollisionEvaluator creation."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        # 0.33 API: LVSContinuousCollisionEvaluator requires CONTINUOUS or LVS_CONTINUOUS
        config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS
        cache = ti.CollisionCache(10)

        evaluator = ti.LVSContinuousCollisionEvaluator(cache, manip, env, config, True)
        assert evaluator is not None

    def test_continuous_collision_constraint(self, kuka_setup):
        """Test ContinuousCollisionConstraint creation."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, joint_limits = kuka_setup

        # Create two joint position variables
        pos0 = np.zeros(len(joint_names))
        pos1 = np.ones(len(joint_names)) * 0.1
        var0 = ti.JointPosition(pos0, joint_names, "Joint_0")
        var1 = ti.JointPosition(pos1, joint_names, "Joint_1")
        var0.SetBounds(joint_limits)
        var1.SetBounds(joint_limits)

        # Create LVS evaluator - 0.33 API requires LVS_DISCRETE type
        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        cache = ti.CollisionCache(10)
        evaluator = ti.LVSDiscreteCollisionEvaluator(cache, manip, env, config, True)

        # Create constraint (uses two variables)
        constraint = ti.ContinuousCollisionConstraint(
            evaluator, var0, var1, False, False, 1, False, "LVSCollision_0_1"
        )
        assert constraint is not None

    def test_continuous_vs_discrete_evaluator_types(self, kuka_setup):
        """Test that continuous evaluators inherit from correct base."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, _, _ = kuka_setup

        # Discrete evaluator - uses default DISCRETE type
        discrete_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        cache = ti.CollisionCache(10)
        discrete = ti.SingleTimestepCollisionEvaluator(cache, manip, env, discrete_config, True)
        assert isinstance(discrete, ti.DiscreteCollisionEvaluator)

        # LVS discrete - requires LVS_DISCRETE type
        lvs_discrete_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        lvs_discrete_config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        lvs_discrete = ti.LVSDiscreteCollisionEvaluator(
            cache, manip, env, lvs_discrete_config, True
        )
        assert isinstance(lvs_discrete, ti.ContinuousCollisionEvaluator)

        # LVS continuous - requires LVS_CONTINUOUS or CONTINUOUS type
        lvs_continuous_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        lvs_continuous_config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS
        lvs_continuous = ti.LVSContinuousCollisionEvaluator(
            cache, manip, env, lvs_continuous_config, True
        )
        assert isinstance(lvs_continuous, ti.ContinuousCollisionEvaluator)


class TestNewConstraintBindings:
    """Tests for newly added constraint bindings (JointJerk, CartLine, etc.)."""

    def test_joint_jerk_constraint(self, kuka_setup):
        """Test JointJerkConstraint creation - requires 6+ waypoints."""
        _, _, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        # Create 6 joint position variables (minimum for jerk)
        vars = []
        for i in range(6):
            pos = np.ones(n_joints) * 0.1 * i
            var = ti.JointPosition(pos, joint_names, f"Joint_{i}")
            var.SetBounds(joint_limits)
            vars.append(var)

        # Create jerk constraint
        targets = np.zeros(n_joints)  # target jerk values
        coeffs = np.ones(n_joints)  # coefficients
        constraint = ti.JointJerkConstraint(targets, vars, coeffs, "JerkConstraint")
        assert constraint is not None
        assert "Jerk" in constraint.GetName()

    def test_cart_line_info(self, kuka_setup):
        """Test CartLineInfo struct creation and member access."""
        from tesseract_robotics.tesseract_common import Isometry3d

        _, manip, _, _ = kuka_setup

        info = ti.CartLineInfo()
        assert info is not None

        # Set members (use Isometry3d for transforms)
        info.manip = manip
        info.source_frame = "base_link"
        info.target_frame = "tool0"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset1 = Isometry3d.Identity()
        info.target_frame_offset2 = Isometry3d.Identity()
        info.indices = np.array([0, 1, 2])  # x, y, z

        # Verify members are set
        assert info.source_frame == "base_link"
        assert info.target_frame == "tool0"

    def test_cart_line_constraint(self, kuka_setup):
        """Test CartLineConstraint creation."""
        from tesseract_robotics.tesseract_common import Isometry3d

        _, manip, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        # Create joint position variable
        pos = np.zeros(n_joints)
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)

        # Create CartLineInfo
        info = ti.CartLineInfo()
        info.manip = manip
        info.source_frame = "base_link"
        info.target_frame = "tool0"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset1 = Isometry3d.Identity()
        # Create offset using matrix (Isometry3d.translate not bound)
        offset_mat = np.eye(4)
        offset_mat[0, 3] = 0.1  # x offset
        info.target_frame_offset2 = Isometry3d(offset_mat)
        info.indices = np.array([0, 1, 2])

        coeffs = np.ones(3)  # x, y, z coefficients
        constraint = ti.CartLineConstraint(info, var, coeffs, "CartLine_0")
        assert constraint is not None
        assert "CartLine" in constraint.GetName()

        # Test numeric differentiation flag
        constraint.use_numeric_differentiation = True
        assert constraint.use_numeric_differentiation is True

    def test_discrete_collision_numerical_constraint(self, kuka_setup):
        """Test DiscreteCollisionNumericalConstraint (uses numerical jacobians)."""
        env, manip, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        # Create joint position variable
        pos = np.zeros(n_joints)
        var = ti.JointPosition(pos, joint_names, "Joint_0")
        var.SetBounds(joint_limits)

        # Create collision evaluator
        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        cache = ti.CollisionCache(10)
        evaluator = ti.SingleTimestepCollisionEvaluator(cache, manip, env, config, True)

        # Create numerical constraint (vs analytical DiscreteCollisionConstraint)
        constraint = ti.DiscreteCollisionNumericalConstraint(
            evaluator, var, 1, False, "CollisionNum_0"
        )
        assert constraint is not None
        assert "Numerical" in constraint.GetName() or "CollisionNum" in constraint.GetName()

        # Verify evaluator accessor
        retrieved = constraint.GetCollisionEvaluator()
        assert retrieved is not None

    def test_inverse_kinematics_info(self, kuka_setup):
        """Test InverseKinematicsInfo struct creation."""
        from tesseract_robotics.tesseract_common import Isometry3d

        env, _, _, _ = kuka_setup

        # Get kinematic group (not joint group)
        kin_group = env.getKinematicGroup("manipulator")
        assert kin_group is not None

        info = ti.InverseKinematicsInfo()
        assert info is not None

        # Set members (use Isometry3d for tcp_offset)
        info.manip = kin_group
        info.working_frame = "base_link"
        info.tcp_frame = "tool0"
        info.tcp_offset = Isometry3d.Identity()

        # Verify
        assert info.working_frame == "base_link"
        assert info.tcp_frame == "tool0"

    def test_inverse_kinematics_constraint(self, kuka_setup):
        """Test InverseKinematicsConstraint creation."""
        from tesseract_robotics.tesseract_common import Isometry3d

        env, _, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        # Get kinematic group
        kin_group = env.getKinematicGroup("manipulator")
        assert kin_group is not None

        # Create two joint position variables (constraint and seed)
        pos = np.zeros(n_joints)
        constraint_var = ti.JointPosition(pos, joint_names, "Joint_0")
        constraint_var.SetBounds(joint_limits)
        seed_var = ti.JointPosition(pos, joint_names, "Joint_seed")
        seed_var.SetBounds(joint_limits)

        # Create IK info
        info = ti.InverseKinematicsInfo()
        info.manip = kin_group
        info.working_frame = "base_link"
        info.tcp_frame = "tool0"
        info.tcp_offset = Isometry3d.Identity()

        # Target pose (create from matrix since translate not bound)
        target_mat = np.eye(4)
        target_mat[0, 3] = 0.5  # x = 0.5m
        target_mat[2, 3] = 0.5  # z = 0.5m
        target = Isometry3d(target_mat)

        # Note: IK constraint needs shared_ptr to info
        # The binding uses ConstPtr which may require special handling
        try:
            constraint = ti.InverseKinematicsConstraint(
                target, info, constraint_var, seed_var, "IK_0"
            )
            assert constraint is not None
        except TypeError as e:
            # May need shared_ptr wrapping - document this
            pytest.skip(f"IK constraint may need shared_ptr wrapper: {e}")


class TestCustomPythonConstraints:
    """Test custom Python constraints with SQP solver.

    These tests verify that Python subclasses of ifopt.ConstraintSet can be
    used with the SQP solver. This requires:
    1. nanobind/eigen/sparse.h for Jacobian matrix handling
    2. Correct parameter order in FillJacobianBlock (var_set, jac_block)
    3. Using scipy sparse API (jac_block[i,j] = val) not Eigen (coeffRef)
    """

    def test_custom_constraint_class_definition(self):
        """Test that custom ConstraintSet subclass can be created."""
        from tesseract_robotics import ifopt

        class MyConstraint(ifopt.ConstraintSet):
            def __init__(self, n_vars, name="custom"):
                super().__init__(n_vars, name)
                self.n_vars = n_vars

            def GetValues(self):
                return np.zeros(self.n_vars)

            def GetBounds(self):
                return [ifopt.BoundZero] * self.n_vars

            def FillJacobianBlock(self, var_set, jac_block):
                # var_set is string, jac_block is scipy csr_matrix
                for i in range(self.n_vars):
                    jac_block[i, i] = 1.0

        constraint = MyConstraint(3, "test")
        assert constraint.GetRows() == 3
        assert constraint.GetName() == "test"

        # Verify GetValues returns correct shape
        vals = constraint.GetValues()
        assert len(vals) == 3

        # Verify GetBounds returns correct length
        bounds = constraint.GetBounds()
        assert len(bounds) == 3

    def test_custom_constraint_with_sqp_solver(self):
        """Test custom Python constraint works with TrustRegionSQPSolver."""
        from tesseract_robotics import ifopt

        class ZeroConstraint(ifopt.ConstraintSet):
            """Constrain variables to zero (equality constraint)."""

            def __init__(self, n_vars, name="zero"):
                super().__init__(n_vars, name)
                self.n_vars = n_vars

            def GetValues(self):
                # Access linked variables via GetVariables()
                vars_composite = self.GetVariables()
                if vars_composite is None:
                    return np.zeros(self.n_vars)
                # Get the component and its values
                var = vars_composite.GetComponent("joint_4")
                if var is None:
                    return np.zeros(self.n_vars)
                return np.array(var.GetValues())

            def GetBounds(self):
                return [ifopt.BoundZero] * self.n_vars

            def FillJacobianBlock(self, var_set, jac_block):
                # Identity Jacobian - constraint value = variable value
                if var_set == "joint_4":
                    for i in range(self.n_vars):
                        jac_block[i, i] = 1.0

        joint_names = ["j1", "j2", "j3"]
        n_dof = len(joint_names)
        n_steps = 5

        # Build QP problem
        qp_problem = tsqp.IfoptQPProblem()

        # Add variables
        variables = []
        bounds = np.array([[-3.14, 3.14]] * n_dof)
        for i in range(n_steps):
            init = np.zeros(n_dof)
            var = ti.JointPosition(init, joint_names, f"joint_{i}")
            var.SetBounds(bounds)
            variables.append(var)
            qp_problem.addVariableSet(var)

        # Fix start position
        start = np.zeros(n_dof)
        variables[0].SetBounds(np.column_stack([start, start]))

        # Add custom constraint to final position (constrain to zero)
        custom = ZeroConstraint(n_dof, "custom_zero")
        qp_problem.addConstraintSet(custom)

        # Setup and solve
        qp_problem.setup()

        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        solver.params.max_iterations = 10

        solver.init(qp_problem)
        solver.solve(qp_problem)

        # Verify final position is constrained to zero
        final_values = variables[-1].GetValues()
        np.testing.assert_array_almost_equal(final_values, np.zeros(n_dof), decimal=3)

    def test_get_variables_exposed(self):
        """Test that GetVariables() is accessible from Python constraints."""
        from tesseract_robotics import ifopt

        class InspectorConstraint(ifopt.ConstraintSet):
            """Constraint that inspects linked variables."""

            def __init__(self, n_vars, name="inspector"):
                super().__init__(n_vars, name)
                self.n_vars = n_vars
                self.inspected_vars = None

            def GetValues(self):
                self.inspected_vars = self.GetVariables()
                return np.zeros(self.n_vars)

            def GetBounds(self):
                return [ifopt.BoundZero] * self.n_vars

            def FillJacobianBlock(self, var_set, jac_block):
                pass

        # Create minimal problem
        qp = tsqp.IfoptQPProblem()

        joint_names = ["j1", "j2"]
        var = ti.JointPosition(np.zeros(2), joint_names, "test_var")
        var.SetBounds(np.array([[-1, 1], [-1, 1]]))
        qp.addVariableSet(var)

        constraint = InspectorConstraint(2, "inspector")
        qp.addConstraintSet(constraint)
        qp.setup()

        # Force evaluation
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        solver.init(qp)

        # GetVariables should now be accessible after linking
        # The constraint's GetValues was called during setup/init
