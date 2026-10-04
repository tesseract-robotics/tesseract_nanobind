"""Tests for low-level trajopt_sqp bindings (SQP solver API).

Updated for 0.34 API:
- JointPosition replaced by createNodesVariables() -> Node -> Var
- CollisionCache removed (now internal)
- CartPosInfo removed; CartPosConstraint takes direct params
- ifopt module removed; types now in trajopt_ifopt
- TrajOptQPProblem(variables) takes constraint and cost sets directly
- evaluateTotalExactCost() -> getTotalExactCost() (no args)
- ConstraintType enum removed
- Bounds uses getLower()/getUpper() instead of .lower/.upper
"""

import subprocess
import sys

import numpy as np
import pytest
import scipy.sparse

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import Environment


def _make_nodes_variables(joint_names, joint_limits, initial_states, name="trajectory"):
    """Helper: create NodesVariables from joint states and limits.

    Returns (nodes_variables, vars_list) where vars_list is the list of Var refs.
    """
    bounds = ti.toBounds(joint_limits)
    nodes_variables = ti.createNodesVariables(name, list(joint_names), list(initial_states), bounds)
    vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
    return nodes_variables, vars_list


def _make_problem(nodes_variables, constraints=None, costs=None):
    """Helper: a TrajOptQPProblem with the given constraint sets and SQUARED cost sets."""
    problem = tsqp.TrajOptQPProblem(nodes_variables)
    for c in constraints or []:
        problem.addConstraintSet(c)
    for c in costs or []:
        problem.addCostSet(c, tsqp.CostPenaltyType.SQUARED)
    return problem


@pytest.fixture
def kuka_setup():
    """Load KUKA IIWA robot environment."""
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

    env = Environment()
    assert env.init(urdf_path, srdf_path, locator)

    manip = env.getKinematicGroup("manipulator")
    joint_names = [jid.name() for jid in manip.getJointIds()]
    joint_limits = manip.getLimits().joint_limits

    return env, manip, joint_names, joint_limits


class TestIfoptBaseClasses:
    """Test ifopt base class bindings (now in trajopt_ifopt)."""

    def test_bounds_creation(self):
        """Test Bounds creation via trajopt_ifopt."""
        b = ti.Bounds(-1.0, 1.0)
        assert b.getLower() == -1.0
        assert b.getUpper() == 1.0

    def test_bounds_types(self):
        """Test BoundsType enum values."""
        assert ti.BoundsType.RANGE_BOUND is not None
        assert ti.BoundsType.EQUALITY is not None
        assert ti.BoundsType.LOWER_BOUND is not None
        assert ti.BoundsType.UPPER_BOUND is not None
        assert ti.BoundsType.UNBOUNDED is not None

    def test_bounds_type_detection(self):
        """Test that Bounds correctly detects type from values."""
        # Range bound
        b = ti.Bounds(-1.0, 1.0)
        assert b.getType() == ti.BoundsType.RANGE_BOUND

    def test_var_interface(self, kuka_setup):
        """Test Var from NodesVariables has expected interface."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        var = vars_list[0]
        assert var.size() == len(joint_names)
        assert var.name is not None


class TestTrajOptIfoptTypes:
    """Test trajopt_ifopt constraint and variable types."""

    def test_nodes_variables_creation(self, kuka_setup):
        """Test NodesVariables factory creation."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        assert len(vars_list) == 1
        assert vars_list[0].size() == len(joint_names)

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

    def test_collision_config(self):
        """Test TrajOptCollisionConfig creation."""
        config = ti.TrajOptCollisionConfig(0.1, 10.0)
        assert config is not None

    def test_cart_pos_constraint(self, kuka_setup):
        """Test CartPosConstraint creation (0.34: direct params, no CartPosInfo)."""
        _, manip, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        constraint = ti.CartPosConstraint(
            vars_list[0],
            manip,
            "tool0",
            "base_link",
            Isometry3d.Identity(),
            Isometry3d.Identity(),
            "CartPos",
        )
        assert constraint is not None

    def test_joint_accel_constraint(self, kuka_setup):
        """Test JointAccelConstraint creation."""
        _, _, joint_names, joint_limits = kuka_setup

        # Need at least 4 waypoints for acceleration
        states = [np.zeros(len(joint_names)) for _ in range(5)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        accel_target = np.zeros(len(joint_names))
        coeffs = np.ones(1)
        constraint = ti.JointAccelConstraint(accel_target, vars_list, coeffs, "Accel")
        assert constraint is not None

    def test_discrete_collision_evaluator(self, kuka_setup):
        """Test SingleTimestepCollisionEvaluator creation (0.34: no cache)."""
        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, config, True)
        assert evaluator is not None

    def test_discrete_collision_constraint(self, kuka_setup):
        """Test DiscreteCollisionConstraint creation."""
        env, manip, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, config, True)

        constraint = ti.DiscreteCollisionConstraint(
            evaluator, vars_list[0], 1, False, "Collision_0"
        )
        assert constraint is not None


class TestTrajOptSQPTypes:
    """Test trajopt_sqp solver types."""

    def test_osqp_solver_creation(self):
        """Test OSQPEigenSolver creation."""
        solver = tsqp.OSQPEigenSolver()
        assert solver is not None

    def test_osqp_solver_settings(self):
        """Test OSQP settings setters forward to solver_->settings()."""
        solver = tsqp.OSQPEigenSolver()
        solver.setPolish(False)
        solver.setWarmStart(False)
        solver.setAdaptiveRho(False)
        solver.setMaxIteration(4096)
        solver.setAbsoluteTolerance(1e-5)
        solver.setRelativeTolerance(1e-7)
        solver.setVerbosity(False)

    def test_trust_region_solver_creation(self):
        """Test TrustRegionSQPSolver creation."""
        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        assert solver is not None

        # Check params access
        assert hasattr(solver, "params")
        solver.params.initial_trust_box_size = 0.1

    def test_cost_penalty_types(self):
        """CostPenaltyType's members exist; TestTrajOptQPProblemPenaltyCosts tests what ABSOLUTE
        and HINGE costs do: bounds checks, exact values, bookkeeping and two trajopt 0.35.0
        defects."""
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

        # Build problem with 3 waypoints
        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem = _make_problem(nv, costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

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

        # Build problem
        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        # Start constraint on first waypoint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        start_pos = np.zeros(len(joint_names))
        constraint = ti.JointPosConstraint(start_pos, vars_list[0], home_coeffs, "Home")

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem = _make_problem(nv, constraints=[constraint], costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False
        solver.solve(problem)

        results = solver.getResults()

        assert results.best_var_vals is not None
        assert results.best_exact_merit is not None
        assert results.overall_iteration >= 0
        assert isinstance(results.cost_names, list)
        assert isinstance(results.constraint_names, list)


class TestSQPIntegration:
    """Integration tests for SQP solver."""

    def test_simple_optimization(self, kuka_setup):
        """Test a simple optimization problem."""
        _, manip, joint_names, joint_limits = kuka_setup

        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        steps = 5

        initial_states = ti.interpolate(start_pos, target_pos, steps)
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, initial_states)

        # Start position constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(start_pos, vars_list[0], home_coeffs, "Home")

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem = _make_problem(nv, constraints=[home_constraint], costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

        solver.solve(problem)
        results = solver.getResults()

        assert results.best_var_vals is not None
        assert len(results.best_var_vals) >= len(joint_names) * steps

    def test_optimization_with_cartesian_constraint(self, kuka_setup):
        """Test optimization with Cartesian target constraint."""
        env, manip, joint_names, joint_limits = kuka_setup

        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        target_tf = manip.calcFwdKin(target_pos)["tool0"]
        steps = 5

        initial_states = ti.interpolate(start_pos, target_pos, steps)
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, initial_states)

        problem = tsqp.TrajOptQPProblem(nv)

        # Start constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(start_pos, vars_list[0], home_coeffs, "Home")
        problem.addConstraintSet(home_constraint)

        # Cartesian target constraint (0.34: direct params)
        target_constraint = ti.CartPosConstraint(
            vars_list[-1],
            manip,
            "tool0",
            "base_link",
            Isometry3d.Identity(),
            target_tf,
            "Target",
        )
        problem.addConstraintSet(target_constraint)

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
        assert results.best_exact_merit is not None

    def test_optimization_with_collision(self, kuka_setup):
        """Test optimization with collision constraints."""
        env, manip, joint_names, joint_limits = kuka_setup

        start_pos = np.zeros(len(joint_names))
        target_pos = np.array([0.3, 0.2, 0.0, -1.0, 0.0, 0.3, 0.0])
        steps = 5

        initial_states = ti.interpolate(start_pos, target_pos, steps)
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, initial_states)

        problem = tsqp.TrajOptQPProblem(nv)

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)

        # Collision constraints (0.34: no cache)
        collision_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        evaluators = []
        constraints = []

        for i in range(1, steps):
            evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, collision_config, True)
            constraint = ti.DiscreteCollisionConstraint(
                evaluator, vars_list[i], 1, False, f"Collision_{i}"
            )
            problem.addConstraintSet(constraint)
            evaluators.append(evaluator)
            constraints.append(constraint)

        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False

        solver.solve(problem)
        results = solver.getResults()

        assert results.best_var_vals is not None


class TestAdditionalBindings:
    """Tests for additional bindings not covered elsewhere."""

    def test_collision_coeff_data(self):
        """Test CollisionCoeffData for per-link collision weights."""
        data = ti.CollisionCoeffData()
        assert data is not None
        assert hasattr(data, "getPairCollisionCoeff")
        assert hasattr(data, "setPairCollisionCoeff")

    def test_qp_problem_interface(self, kuka_setup):
        """Test QPProblem interface methods."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        problem = tsqp.TrajOptQPProblem(nv)
        problem.setup()
        assert problem.getNumNLPVars() >= len(joint_names)

    def test_sqp_callback_class(self):
        """Test SQPCallback class exists and is callable."""
        assert hasattr(tsqp, "SQPCallback")
        callback_class = tsqp.SQPCallback
        assert callback_class is not None

    def test_qp_solver_inheritance(self):
        """Test QPSolver base class."""
        solver = tsqp.OSQPEigenSolver()
        assert isinstance(solver, tsqp.QPSolver)

    def test_component_interface(self, kuka_setup):
        """Test Component interface via Var from NodesVariables."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states, name="TestVar")

        # Var has name and size
        var = vars_list[0]
        assert var.name is not None
        assert var.size() == len(joint_names)

    def test_constraint_set_interface(self, kuka_setup):
        """Test ConstraintSet interface via JointPosConstraint."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        target = np.zeros(len(joint_names))
        coeffs = np.ones(len(joint_names))
        constraint = ti.JointPosConstraint(target, vars_list[0], coeffs, "Position")

        assert constraint.getName() == "Position"
        assert hasattr(constraint, "getRows")

    def test_cost_term_interface(self, kuka_setup):
        """Test CostTerm interface via JointVelConstraint used as cost."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        vel_target = np.zeros(len(joint_names))
        vel_cost = ti.JointVelConstraint(vel_target, vars_list, np.ones(1), "VelCost")

        # Adding as cost to QP problem works
        problem = tsqp.TrajOptQPProblem(nv)
        problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)
        assert problem is not None

    def test_get_total_exact_cost(self, kuka_setup):
        """Test getTotalExactCost() (renamed from evaluateTotalExactCost in 0.34)."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem = _make_problem(nv, costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False
        solver.solve(problem)

        # 0.34 API: getTotalExactCost() with no args
        cost = problem.getTotalExactCost()
        assert cost is not None
        assert isinstance(cost, float)

    def test_get_exact_costs(self, kuka_setup):
        """Test getExactCosts() (renamed from evaluateExactCosts in 0.34)."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        problem = _make_problem(nv, costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False
        solver.solve(problem)

        # 0.34 API: getExactCosts() with no args
        costs = problem.getExactCosts()
        assert costs is not None


class TestContinuousCollisionBindings:
    """Tests for continuous collision evaluators and constraints."""

    def test_continuous_collision_evaluator_base(self):
        """Test ContinuousCollisionEvaluator base class exists."""
        assert hasattr(ti, "ContinuousCollisionEvaluator")

    def test_lvs_discrete_collision_evaluator(self, kuka_setup):
        """Test LVSDiscreteCollisionEvaluator creation (0.34: no cache)."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE

        evaluator = ti.LVSDiscreteCollisionEvaluator(manip, env, config, True)
        assert evaluator is not None
        assert evaluator.getCollisionMarginBuffer() is not None

    def test_lvs_continuous_collision_evaluator(self, kuka_setup):
        """Test LVSContinuousCollisionEvaluator creation (0.34: no cache)."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, _ = kuka_setup

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS

        evaluator = ti.LVSContinuousCollisionEvaluator(manip, env, config, True)
        assert evaluator is not None

    def test_continuous_collision_constraint(self, kuka_setup):
        """Test ContinuousCollisionConstraint creation."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, joint_names, joint_limits = kuka_setup

        states = [
            np.zeros(len(joint_names)),
            np.ones(len(joint_names)) * 0.1,
        ]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        evaluator = ti.LVSDiscreteCollisionEvaluator(manip, env, config, True)

        constraint = ti.ContinuousCollisionConstraint(
            evaluator, vars_list[0], vars_list[1], False, False, 1, False, "LVSCollision_0_1"
        )
        assert constraint is not None

    def test_continuous_vs_discrete_evaluator_types(self, kuka_setup):
        """Test that continuous evaluators inherit from correct base."""
        from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

        env, manip, _, _ = kuka_setup

        # Discrete evaluator
        discrete_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        discrete = ti.SingleTimestepCollisionEvaluator(manip, env, discrete_config, True)
        assert isinstance(discrete, ti.DiscreteCollisionEvaluator)

        # LVS discrete
        lvs_discrete_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        lvs_discrete_config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        lvs_discrete = ti.LVSDiscreteCollisionEvaluator(manip, env, lvs_discrete_config, True)
        assert isinstance(lvs_discrete, ti.ContinuousCollisionEvaluator)

        # LVS continuous
        lvs_continuous_config = ti.TrajOptCollisionConfig(0.05, 20.0)
        lvs_continuous_config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS
        lvs_continuous = ti.LVSContinuousCollisionEvaluator(manip, env, lvs_continuous_config, True)
        assert isinstance(lvs_continuous, ti.ContinuousCollisionEvaluator)


class TestNewConstraintBindings:
    """Tests for newly added constraint bindings (JointJerk, CartLine, etc.)."""

    def test_joint_jerk_constraint(self, kuka_setup):
        """Test JointJerkConstraint creation - requires 6+ waypoints."""
        _, _, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        states = [np.ones(n_joints) * 0.1 * i for i in range(6)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        targets = np.zeros(n_joints)
        coeffs = np.ones(n_joints)
        constraint = ti.JointJerkConstraint(targets, vars_list, coeffs, "JerkConstraint")
        assert constraint is not None
        assert "Jerk" in constraint.getName()

    def test_cart_line_info(self, kuka_setup):
        """Test CartLineInfo struct creation and member access."""
        _, manip, _, _ = kuka_setup

        info = ti.CartLineInfo()
        assert info is not None

        info.manip = manip
        info.source_frame = "base_link"
        info.target_frame = "tool0"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset1 = Isometry3d.Identity()
        info.target_frame_offset2 = Isometry3d.Identity()
        info.indices = np.array([0, 1, 2])

        assert info.source_frame == "base_link"
        assert info.target_frame == "tool0"

    def test_cart_line_constraint(self, kuka_setup):
        """Test CartLineConstraint creation."""
        _, manip, joint_names, joint_limits = kuka_setup
        n_joints = len(joint_names)

        states = [np.zeros(n_joints)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        info = ti.CartLineInfo()
        info.manip = manip
        info.source_frame = "base_link"
        info.target_frame = "tool0"
        info.source_frame_offset = Isometry3d.Identity()
        info.target_frame_offset1 = Isometry3d.Identity()
        offset_mat = np.eye(4)
        offset_mat[0, 3] = 0.1
        info.target_frame_offset2 = Isometry3d(offset_mat)
        info.indices = np.array([0, 1, 2])

        coeffs = np.ones(3)
        constraint = ti.CartLineConstraint(info, vars_list[0], coeffs, "CartLine_0")
        assert constraint is not None
        assert "CartLine" in constraint.getName()

        constraint.use_numeric_differentiation = True
        assert constraint.use_numeric_differentiation is True

    def test_discrete_collision_numerical_constraint(self, kuka_setup):
        """Test DiscreteCollisionNumericalConstraint (uses numerical jacobians)."""
        env, manip, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        config = ti.TrajOptCollisionConfig(0.05, 20.0)
        evaluator = ti.SingleTimestepCollisionEvaluator(manip, env, config, True)

        constraint = ti.DiscreteCollisionNumericalConstraint(
            evaluator, vars_list[0], 1, False, "CollisionNum_0"
        )
        assert constraint is not None
        assert "Numerical" in constraint.getName() or "CollisionNum" in constraint.getName()

        retrieved = constraint.getCollisionEvaluator()
        assert retrieved is not None

    def test_inverse_kinematics_info(self, kuka_setup):
        """Test InverseKinematicsInfo struct creation."""
        env, _, _, _ = kuka_setup

        kin_group = env.getKinematicGroup("manipulator")
        assert kin_group is not None

        info = ti.InverseKinematicsInfo()
        assert info is not None

        info.manip = kin_group
        info.working_frame = "base_link"
        info.tcp_frame = "tool0"
        info.tcp_offset = Isometry3d.Identity()

        assert info.working_frame == "base_link"
        assert info.tcp_frame == "tool0"

    def test_inverse_kinematics_constraint(self, kuka_setup):
        """Test InverseKinematicsConstraint creation."""
        env, _, joint_names, joint_limits = kuka_setup

        kin_group = env.getKinematicGroup("manipulator")
        assert kin_group is not None

        # Need 2 waypoints: constraint_var + seed_var
        states = [np.zeros(len(joint_names)), np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        info = ti.InverseKinematicsInfo()
        info.manip = kin_group
        info.working_frame = "base_link"
        info.tcp_frame = "tool0"
        info.tcp_offset = Isometry3d.Identity()

        target_mat = np.eye(4)
        target_mat[0, 3] = 0.5
        target_mat[2, 3] = 0.5
        target = Isometry3d(target_mat)

        constraint = ti.InverseKinematicsConstraint(
            target, info, vars_list[0], vars_list[1], "IK_0"
        )
        assert constraint is not None


class TestConstraintSetInterface:
    """Test ConstraintSet interface via built-in constraints.

    In 0.34, ConstraintSet is no longer subclassable from Python
    (the ifopt module with trampoline classes was removed). These tests
    verify the ConstraintSet interface works correctly via C++ constraints.
    """

    def test_constraint_set_base_class(self):
        """Test that ConstraintSet class exists in trajopt_ifopt."""
        assert hasattr(ti, "ConstraintSet")
        assert ti.ConstraintSet is not None

    def test_constraint_set_via_joint_pos(self, kuka_setup):
        """Test ConstraintSet interface via JointPosConstraint."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        target = np.zeros(len(joint_names))
        coeffs = np.ones(len(joint_names))
        constraint = ti.JointPosConstraint(target, vars_list[0], coeffs, "Position")

        # ConstraintSet interface
        assert isinstance(constraint, ti.ConstraintSet)
        assert constraint.getName() == "Position"
        assert constraint.getRows() > 0

    def test_constraint_used_as_cost_with_sqp(self, kuka_setup):
        """Test JointVelConstraint as cost in SQP optimization."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names)) for _ in range(5)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        # Add position constraint on first waypoint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(
            np.zeros(len(joint_names)), vars_list[0], home_coeffs, "Home"
        )

        # Velocity as cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "VelCost"
        )

        problem = _make_problem(nv, constraints=[home_constraint], costs=[vel_cost])
        problem.setup()

        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        solver.params.max_iterations = 10
        solver.verbose = False
        solver.solve(problem)

        status = solver.getStatus()
        assert status in [
            tsqp.SQPStatus.NLP_CONVERGED,
            tsqp.SQPStatus.ITERATION_LIMIT,
        ]


# ---------------------------------------------------------------------------
# TrajOptQPProblem (tesseract_nanobind#145)
# ---------------------------------------------------------------------------

# Linear residuals make the Gauss-Newton model exact, so exact/model = 1 up to float64
# cancellation (about 1e-11 at this step); defects of the kind trajopt#595 reports (a merit
# that ignores the cost coefficient c, a model counted once per cost term of N) read 1/c or 1/N.
MODEL_ROUND_OFF = 1e-8
MODEL_STEP = 1e-3
# OSQP's absolute tolerance as the trajopt_sqp wrapper sets it: a QP solution is exact to this.
OSQP_ABSOLUTE_TOLERANCE = 1e-4

_TRAJOPT_QP_TEARDOWN_SCRIPT = """\
import numpy as np
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
nodes = ti.createNodesVariables(
    "trajectory", ["j0"], [np.array([float(k)]) for k in range(6)],
    ti.toBounds(np.array([[-10.0, 10.0]])),
)
vars_list = [node.getVar("joints") for node in nodes.getNodes()]
problem = tsqp.TrajOptQPProblem(nodes)
problem.addConstraintSet(ti.JointPosConstraint(np.zeros(1), vars_list[0], np.ones(1), "start"))
problem.addCostSet(
    ti.JointVelConstraint(np.zeros(1), vars_list, np.full(1, 10.0), "vel"),
    tsqp.CostPenaltyType.SQUARED,
)
problem.setup()
solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
solver.solve(problem)
print("OK:", solver.getStatus().name)
"""


class _PinCost(ti.ConstraintSet):
    """Squared cost (x_k - target)^2 on one variable: a linear residual written in Python."""

    def __init__(self, index: int, target: float):
        super().__init__("python_pin", 1)
        self._index = index
        self._target = target

    def getValues(self) -> np.ndarray:
        x = np.array(self.getVariables().getValues())
        return np.array([x[self._index] - self._target])

    def getBounds(self) -> list:
        return [ti.Bounds(0.0, 0.0)]

    def getJacobian(self):
        n = len(self.getVariables().getValues())
        return scipy.sparse.csr_matrix(([1.0], ([0], [self._index])), shape=(1, n))

    def update(self) -> int:
        return self.getRows()

    def getCoefficients(self) -> np.ndarray:
        return np.ones(1)


def _one_joint_problem(values, specs):
    """A set-up TrajOptQPProblem over one joint, one squared cost per (class, coeff)."""
    nodes = ti.createNodesVariables(
        "trajectory",
        ["j0"],
        [np.array([v]) for v in values],
        ti.toBounds(np.array([[-100.0, 100.0]])),
    )
    vars_list = [node.getVar("joints") for node in nodes.getNodes()]
    problem = tsqp.TrajOptQPProblem(nodes)
    for k, (cls, coeff) in enumerate(specs):
        problem.addCostSet(
            cls(np.zeros(1), vars_list, np.full(1, coeff), f"cost_{k}"),
            tsqp.CostPenaltyType.SQUARED,
        )
    problem.setup()
    return problem


class TestTrajOptQPProblem:
    """TrajOptQPProblem: the QP problem tesseract_planning's TrajOpt-Ifopt planner builds."""

    SPECS = [
        (ti.JointVelConstraint, 10.0),
        (ti.JointAccelConstraint, 1.0),
        (ti.JointJerkConstraint, 2000.0),
    ]

    def test_is_a_qp_problem(self, kuka_setup):
        _, _, joint_names, joint_limits = kuka_setup
        nv, _ = _make_nodes_variables(joint_names, joint_limits, [np.zeros(len(joint_names))] * 3)
        assert isinstance(tsqp.TrajOptQPProblem(nv), tsqp.QPProblem)

    def test_solve_holds_the_pinned_start(self, kuka_setup):
        _, _, joint_names, joint_limits = kuka_setup
        start = np.zeros(len(joint_names))
        target = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
        nv, vars_list = _make_nodes_variables(
            joint_names, joint_limits, ti.interpolate(start, target, 5)
        )
        problem = tsqp.TrajOptQPProblem(nv)
        home = ti.JointPosConstraint(start, vars_list[0], np.full(len(joint_names), 5.0), "Home")
        problem.addConstraintSet(home)
        vel = ti.JointVelConstraint(np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity")
        problem.addCostSet(vel, tsqp.CostPenaltyType.SQUARED)
        problem.setup()
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())

        solver.solve(problem)

        assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
        results = solver.getResults()
        assert len(results.merit_error_coeffs) == 1  # one merit unit per constraint set
        np.testing.assert_allclose(
            np.array(vars_list[0].value()), start, atol=OSQP_ABSOLUTE_TOLERANCE
        )

    def test_model_matches_the_exact_cost(self):
        rng = np.random.default_rng(0)
        x0 = np.cumsum(rng.normal(size=8))
        x1 = x0 + MODEL_STEP * rng.normal(size=8)
        base = _one_joint_problem(x0, self.SPECS)
        base.convexify()
        trial = _one_joint_problem(x1, self.SPECS)
        exact = trial.getExactCosts().sum() - base.getExactCosts().sum()
        predicted = base.evaluateConvexCosts(x1).sum() - base.evaluateConvexCosts(x0).sum()
        assert exact / predicted == pytest.approx(1.0, abs=MODEL_ROUND_OFF)

    def test_total_exact_cost_is_the_sum(self):
        x0 = np.cumsum(np.random.default_rng(0).normal(size=8))
        problem = _one_joint_problem(x0, self.SPECS)
        costs = problem.getExactCosts()
        assert len(costs) == len(self.SPECS)
        assert problem.getTotalExactCost() == pytest.approx(costs.sum(), rel=MODEL_ROUND_OFF)

    def test_python_cost_set(self):
        nodes = ti.createNodesVariables(
            "trajectory",
            ["j0"],
            [np.array([1.95]) for _ in range(4)],
            ti.toBounds(np.array([[-10.0, 10.0]])),
        )
        problem = tsqp.TrajOptQPProblem(nodes)
        problem.addCostSet(_PinCost(index=2, target=2.0), tsqp.CostPenaltyType.SQUARED)
        problem.setup()
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())

        solver.solve(problem)

        assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
        assert nodes.getValues()[2] == pytest.approx(2.0, abs=OSQP_ABSOLUTE_TOLERANCE)

    def test_squared_cost_needs_equality_bounds(self):
        nodes = ti.createNodesVariables(
            "trajectory", ["j0"], [np.zeros(1)] * 3, ti.toBounds(np.array([[-10.0, 10.0]]))
        )
        var = nodes.getNodes()[1].getVar("joints")
        upper_limited = ti.JointPosConstraint([ti.Bounds(-np.inf, 1.0)], var, np.ones(1), "upper")
        problem = tsqp.TrajOptQPProblem(nodes)
        with pytest.raises(RuntimeError, match="equality bounds"):
            problem.addCostSet(upper_limited, tsqp.CostPenaltyType.SQUARED)

    def test_interpreter_teardown_without_ordered_del(self):
        """Module globals die in the interpreter's order, not C++'s ownership order."""
        proc = subprocess.run(
            [sys.executable, "-c", _TRAJOPT_QP_TEARDOWN_SCRIPT],
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0, (
            f"interpreter teardown died (rc={proc.returncode}, SIGSEGV is -11/139): "
            f"{proc.stderr[-500:]}"
        )
        assert "OK: NLP_CONVERGED" in proc.stdout


# ---------------------------------------------------------------------------
# ABSOLUTE and HINGE cost sets on TrajOptQPProblem (tesseract_nanobind#149 review)
# ---------------------------------------------------------------------------

# "trajopt 0.35.0" in this section is trajopt as every tesseract-robotics-nanobind 0.35.0.x wheel
# bundles it. No release has TrajOptQPProblem yet (0.35.0.8, the latest, binds only
# IfoptQPProblem); the behaviour pinned here was measured on #149's development builds
# 0.35.0.9.dev13–dev16, before the merge. trajopt#592's fix reaches a wheel only once one is built
# on a newer trajopt.
# Every joint value below is dyadic, so each residual, violation and sum is exact in float64
# and exact costs compare with ==.
PENALTY_JOINT_LIMIT = 10.0  # rad, symmetric; no variable bound is active below
SEED_NODE_1 = 1.5  # rad, node 1's joint value at the seed; nodes 0 and 2 sit at 0
# rad: how far the seed misses each penalty cost below. It must exceed initial_trust_box_size /
# improve_ratio_threshold = 0.1 / 0.25 = 0.4: trajopt 0.35.0 reads a penalty cost's model as 0, so
# a trial's ratio is box / SEED_VIOLATION = 0.2 < 0.25 and the seed stalls; at 0.4 or less the
# first step is taken.
SEED_VIOLATION = 0.5
ABSOLUTE_TARGET = SEED_NODE_1 + SEED_VIOLATION  # rad: the ABSOLUTE row x_1 = 2
HINGE_UPPER = SEED_NODE_1 - SEED_VIOLATION  # rad: the HINGE row x_1 <= 1
# rad: a violation the default first trust box cuts by more than a quarter (box /
# SMALL_VIOLATION = 0.1 / 0.25 = 0.4 >= improve_ratio_threshold = 0.25), so even trajopt 0.35.0
# removes it: the contrast to SEED_VIOLATION.
SMALL_VIOLATION = 0.25
OFF_SEED_NODE_1 = 2.25  # rad, a second point: past the ABSOLUTE target, further past HINGE_UPPER
LARGE_COEFF = 10.0  # a cost coefficient other than 1: an exact cost that drops it reads 0.5, not 5


def _penalty_problem(node_1, cost_sets, constraint_sets=()):
    """A set-up TrajOptQPProblem over three nodes: node 1 at the joint values node_1, the
    other two at 0. Each (make, penalty_type) of cost_sets adds make(vars_list) as a cost, each
    make of constraint_sets make(vars_list) as a constraint.

    Returns:
        (nodes, problem).
    """
    n_joints = len(node_1)
    zeros = np.zeros(n_joints)
    nodes, vars_list = _make_nodes_variables(
        [f"j{k}" for k in range(n_joints)],
        np.tile([-PENALTY_JOINT_LIMIT, PENALTY_JOINT_LIMIT], (n_joints, 1)),
        [zeros, np.array(node_1, dtype=float), zeros],
    )
    problem = tsqp.TrajOptQPProblem(nodes)
    for make, penalty_type in cost_sets:
        problem.addCostSet(make(vars_list), penalty_type)
    for make in constraint_sets:
        problem.addConstraintSet(make(vars_list))
    problem.setup()
    return nodes, problem


def _absolute_cost(targets, coeff=1.0, name="absolute"):
    """make(vars_list): equality rows x_1j = targets[j] on node 1, each weighted by coeff."""
    return lambda vars_list: ti.JointPosConstraint(
        np.array(targets, dtype=float), vars_list[1], np.full(len(targets), coeff), name
    )


def _hinge_cost(bounds, coeff=1.0, name="hinge"):
    """make(vars_list): one one-sided (lower, upper) row per joint of node 1, weighted by coeff."""
    return lambda vars_list: ti.JointPosConstraint(
        [ti.Bounds(lower, upper) for lower, upper in bounds],
        vars_list[1],
        np.full(len(bounds), coeff),
        name,
    )


def _seed_cost(penalty_type, coeff, violation=SEED_VIOLATION):
    """(make, penalty_type): a node-1 cost the seed violates by `violation`: the target
    SEED_NODE_1 + violation (ABSOLUTE) or the upper bound SEED_NODE_1 - violation (HINGE). The
    default puts them at ABSOLUTE_TARGET and HINGE_UPPER."""
    if penalty_type == tsqp.CostPenaltyType.ABSOLUTE:
        return _absolute_cost((SEED_NODE_1 + violation,), coeff), penalty_type
    return _hinge_cost(((-np.inf, SEED_NODE_1 - violation),), coeff), penalty_type


_PENALTY_TYPES = pytest.mark.parametrize(
    "penalty_type",
    [tsqp.CostPenaltyType.ABSOLUTE, tsqp.CostPenaltyType.HINGE],
    ids=["absolute", "hinge"],
)
_COEFFS = pytest.mark.parametrize("coeff", [1.0, LARGE_COEFF], ids=["c1", "c10"])


class TestTrajOptQPProblemPenaltyCosts:
    """ABSOLUTE and HINGE cost sets on TrajOptQPProblem.

    QP layout (trajopt 0.35.0, trajopt_qp_problem.cpp:28, :798-822): each ABSOLUTE row adds two
    slack variables, each HINGE row one, after the NLP variables; the QP prices a slack at its
    row's coefficient (:798). This branch links a trajopt that includes
    tesseract-robotics/trajopt#592, which fixes the two trajopt 0.35.0 penalty defects of
    tesseract_nanobind#151; `main` (trajopt 0.35.0) still pins the defective behaviour.
    """

    def test_absolute_cost_needs_equality_bounds(self):
        nodes, vars_list = _make_nodes_variables(
            ["j0"], np.array([[-PENALTY_JOINT_LIMIT, PENALTY_JOINT_LIMIT]]), [np.zeros(1)] * 3
        )
        upper_limited = ti.JointPosConstraint(
            [ti.Bounds(-np.inf, HINGE_UPPER)], vars_list[1], np.ones(1), "upper"
        )
        problem = tsqp.TrajOptQPProblem(nodes)
        with pytest.raises(RuntimeError, match="absolute cost must have equality bounds"):
            problem.addCostSet(upper_limited, tsqp.CostPenaltyType.ABSOLUTE)

    def test_hinge_cost_needs_inequality_bounds(self):
        nodes, vars_list = _make_nodes_variables(
            ["j0"], np.array([[-PENALTY_JOINT_LIMIT, PENALTY_JOINT_LIMIT]]), [np.zeros(1)] * 3
        )
        pinned = ti.JointPosConstraint(
            np.array([ABSOLUTE_TARGET]), vars_list[1], np.ones(1), "pinned"
        )
        problem = tsqp.TrajOptQPProblem(nodes)
        with pytest.raises(RuntimeError, match="hinge cost must have inequality bounds"):
            problem.addCostSet(pinned, tsqp.CostPenaltyType.HINGE)

    def test_absolute_exact_cost_is_the_summed_absolute_error(self):
        """At c = 1 the exact cost is the sum over rows of |e|, one row below its target and
        one above: e = (-0.5, +0.25) reads 0.75, where the signed sum is -0.25 and the squared
        sum 0.3125 (trajopt_qp_problem.cpp:1002-1016)."""
        _, problem = _penalty_problem(
            (SEED_NODE_1, OFF_SEED_NODE_1),
            [(_absolute_cost((ABSOLUTE_TARGET, ABSOLUTE_TARGET)), tsqp.CostPenaltyType.ABSOLUTE)],
        )
        assert problem.getExactCosts().tolist() == [0.75]

    @pytest.mark.parametrize(
        ("bound", "node_1", "expected"),
        [
            ((-np.inf, HINGE_UPPER), (1.5, 1.25), 0.75),  # both rows above: 0.5 + 0.25
            ((-np.inf, HINGE_UPPER), (0.5, 1.0), 0.0),  # one row inside, one on the bound
            ((HINGE_UPPER, np.inf), (0.5, 0.75), 0.75),  # both rows below: 0.5 + 0.25
            ((HINGE_UPPER, np.inf), (1.5, 1.0), 0.0),  # one row inside, one on the bound
        ],
        ids=["upper-violated", "upper-satisfied", "lower-violated", "lower-satisfied"],
    )
    def test_hinge_exact_cost_is_the_summed_violation(self, bound, node_1, expected):
        """At c = 1 the exact cost sums each row's distance outside its one-sided bound, and is
        0 when every row holds (trajopt_qp_problem.cpp:1002-1016)."""
        _, problem = _penalty_problem(
            node_1, [(_hinge_cost((bound, bound)), tsqp.CostPenaltyType.HINGE)]
        )
        assert problem.getExactCosts().tolist() == [expected]

    def test_cost_terms_are_reported_by_penalty_type(self):
        """One exact cost per term, in the order squared, hinge, absolute, whatever the order
        they were added in: setup() concatenates the squared, then the hinge, then the absolute
        sets (trajopt_qp_problem.cpp:569-577). getTotalExactCost() is their sum."""
        _, problem = _penalty_problem(
            (SEED_NODE_1,),
            [
                (_absolute_cost((OFF_SEED_NODE_1,)), tsqp.CostPenaltyType.ABSOLUTE),
                (_hinge_cost(((-np.inf, HINGE_UPPER),)), tsqp.CostPenaltyType.HINGE),
                (
                    lambda vars_list: ti.JointVelConstraint(
                        np.zeros(1), vars_list, np.ones(1), "squared"
                    ),
                    tsqp.CostPenaltyType.SQUARED,
                ),
            ],
        )
        assert problem.getNLPCostNames() == ["squared", "hinge", "absolute"]
        assert problem.getNumNLPCosts() == 3
        # squared: velocities (1.5, -1.5), 2.25 + 2.25; hinge: 1.5 - 1; absolute: |1.5 - 2.25|,
        # the target at OFF_SEED_NODE_1
        assert problem.getExactCosts().tolist() == [4.5, 0.5, 0.75]
        assert problem.getTotalExactCost() == 5.75

    @_COEFFS
    @_PENALTY_TYPES
    def test_zero_slack_model_is_the_exact_cost(self, penalty_type, coeff):
        """With its slack entries at 0, a penalty cost's convex model is the slack-free linear
        model, so on a linear residual it equals the exact cost anywhere: at the
        convexification point and away from it. trajopt 0.35.0 reads the slack columns (zeros
        here) and leaves the coefficient out of both sides; trajopt#592 reads only the NLP block
        and weights both sides by it."""
        make, penalty_type = _seed_cost(penalty_type, coeff)
        _, base = _penalty_problem((SEED_NODE_1,), [(make, penalty_type)])
        base.convexify()
        n_slack = base.getNumQPVars() - base.getNumNLPVars()
        assert n_slack > 0

        for node_1 in (SEED_NODE_1, OFF_SEED_NODE_1):
            _, at_point = _penalty_problem((node_1,), [(make, penalty_type)])
            qp_point = np.concatenate([[0.0, node_1, 0.0], np.zeros(n_slack)])
            model = base.evaluateConvexCosts(qp_point)
            assert model == pytest.approx(at_point.getExactCosts(), abs=MODEL_ROUND_OFF)

    @_COEFFS
    @_PENALTY_TYPES
    def test_penalty_only_cost_the_trust_box_cuts_by_a_quarter_is_reduced(
        self, penalty_type, coeff
    ):
        """A penalty-only cost whose violation the first trust box cuts by
        improve_ratio_threshold is removed: SMALL_VIOLATION = 0.25 <= initial_trust_box_size /
        improve_ratio_threshold = 0.4. On trajopt 0.35.0 the model reads the cost as 0, so the
        first trial's ratio is box / SMALL_VIOLATION = 0.4 >= 0.25, the trial is accepted, and
        the solve reaches the target; after trajopt#592 the model is exact on this linear
        residual and the ratio is 1. The contrast to
        test_penalty_only_cost_beyond_the_trust_box_is_reduced, which stalled on trajopt
        0.35.0 because of the violation's size."""
        nodes, problem = _penalty_problem(
            (SEED_NODE_1,), [_seed_cost(penalty_type, coeff, SMALL_VIOLATION)]
        )
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        params = solver.params
        assert SMALL_VIOLATION <= params.initial_trust_box_size / params.improve_ratio_threshold

        solver.solve(problem)

        assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
        # The position is exact to OSQP_ABSOLUTE_TOLERANCE; after trajopt#592 the exact cost is
        # that residual times coeff, so the cost's tolerance scales with coeff.
        assert problem.getExactCosts().tolist() == pytest.approx(
            [0.0], abs=coeff * OSQP_ABSOLUTE_TOLERANCE
        )
        node_1 = nodes.getValues()[1]
        if penalty_type == tsqp.CostPenaltyType.ABSOLUTE:
            assert node_1 == pytest.approx(
                SEED_NODE_1 + SMALL_VIOLATION, abs=OSQP_ABSOLUTE_TOLERANCE
            )
        else:
            assert node_1 <= SEED_NODE_1 - SMALL_VIOLATION + OSQP_ABSOLUTE_TOLERANCE

    @_COEFFS
    @_PENALTY_TYPES
    def test_penalty_only_cost_beyond_the_trust_box_is_reduced(self, penalty_type, coeff):
        """An ABSOLUTE- or HINGE-only cost whose violation the first trust box cannot cut by
        improve_ratio_threshold is still removed.

        trajopt#592 evaluates penalty rows on the slack-free linear model, so the model is exact
        on this linear residual: the predicted improvement equals the exact one and every trial
        within the box is accepted. trajopt 0.35.0 read the slack columns too, predicted the whole
        cost away, rejected every trial (ratio box / SEED_VIOLATION = 0.2 < 0.25) and reported
        NLP_CONVERGED at the seed (tesseract_nanobind#151).
        """
        nodes, problem = _penalty_problem((SEED_NODE_1,), [_seed_cost(penalty_type, coeff)])
        solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
        params = solver.params
        assert SEED_VIOLATION > params.initial_trust_box_size / params.improve_ratio_threshold

        solver.solve(problem)

        results = solver.getResults()
        assert results.new_approx_costs == pytest.approx(
            np.array(results.new_costs), abs=MODEL_ROUND_OFF
        )
        node_1 = nodes.getValues()[1]
        if penalty_type == tsqp.CostPenaltyType.ABSOLUTE:
            assert node_1 == pytest.approx(ABSOLUTE_TARGET, abs=OSQP_ABSOLUTE_TOLERANCE)
        else:
            assert node_1 <= HINGE_UPPER + OSQP_ABSOLUTE_TOLERANCE
        assert problem.getExactCosts().tolist() == pytest.approx([0.0], abs=OSQP_ABSOLUTE_TOLERANCE)

    @_PENALTY_TYPES
    def test_penalty_exact_cost_is_weighted_by_the_coefficient(self, penalty_type):
        """getExactCosts() weights a penalty cost's row violations by the row coefficient, as
        the QP prices each row's slack: at c = 10 the exact cost reads 10 * 0.5 = 5. trajopt
        0.35.0 summed them unweighted and read 0.5 (tesseract_nanobind#151, fixed by
        trajopt#592)."""
        _, problem = _penalty_problem((SEED_NODE_1,), [_seed_cost(penalty_type, LARGE_COEFF)])
        assert problem.getExactCosts().tolist() == [LARGE_COEFF * SEED_VIOLATION]


# ---------------------------------------------------------------------------
# Convex evaluators take the QP solution vector (tesseract_nanobind#149 review)
# ---------------------------------------------------------------------------

# rad: a constraint row x_0 = 0.5 on node 0, which the seed (x_0 = 0) violates by 0.5. It gives
# evaluateConvexConstraintViolations a set to report and the QP two more slack variables.
START_TARGET = 0.5
# s; a child imports the bindings and evaluates in about 1-2 s. A hang must fail the test, not
# the session, and 60 s leaves room for a loaded runner.
EVALUATOR_CHILD_TIMEOUT_S = 60.0
CONVEX_EVALUATORS = (
    "evaluateConvexCosts",
    "evaluateTotalConvexCost",
    "evaluateConvexConstraintViolations",
)


def _start_pin(vars_list):
    """The constraint x_0 = START_TARGET on node 0."""
    return ti.JointPosConstraint(np.array([START_TARGET]), vars_list[0], np.ones(1), "start")


_BEFORE_CONVEXIFY_SCRIPT = f"""\
import numpy as np
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
nodes = ti.createNodesVariables(
    "trajectory", ["j0"], [np.array([v]) for v in (0.0, {SEED_NODE_1}, 0.0)],
    ti.toBounds(np.array([[-{PENALTY_JOINT_LIMIT}, {PENALTY_JOINT_LIMIT}]])),
)
vars_list = [node.getVar("joints") for node in nodes.getNodes()]
problem = tsqp.TrajOptQPProblem(nodes)
problem.addCostSet(
    ti.JointVelConstraint(np.zeros(1), vars_list, np.ones(1), "vel"),
    tsqp.CostPenaltyType.SQUARED,
)
problem.setup()
for name in {CONVEX_EVALUATORS!r}:
    for var_vals in (np.zeros(0), np.zeros(3)):
        try:
            getattr(problem, name)(var_vals)
        except ValueError as exc:
            print(f"RAISED {{name}} {{len(var_vals)}}: {{exc}}", flush=True)
        else:
            print(f"RETURNED {{name}} {{len(var_vals)}}", flush=True)
"""


class TestConvexEvaluatorArguments:
    """evaluateConvexCosts, evaluateTotalConvexCost and evaluateConvexConstraintViolations take
    the QP solution vector of the last convexify(): getNumQPVars() entries, the NLP variables
    followed by the slack variables (trajopt_qp_problem.cpp:28).

    trajopt 0.35.0 multiplies a penalty cost's full QP rows, slack columns included, into
    var_vals unchecked (trajopt_qp_problem.cpp:187-188), so an NLP-sized var_vals was read past
    its end: on a #149 development build from before this guard, through a view of the first
    three entries of a longer buffer, an absolute cost of 0.5 read 3.5, a hinge one 0.0,
    depending on the entries after the view. Before the first convexify() there is no model and getNumQPVars() is 0; an
    empty or NLP-sized var_vals then segfaulted on a squared cost. The binding raises ValueError
    in both cases, for all three evaluators, although trajopt 0.35.0 reads only the NLP block in
    two of them: one contract.
    """

    @pytest.mark.parametrize("evaluator", CONVEX_EVALUATORS)
    @_PENALTY_TYPES
    def test_nlp_sized_var_vals_raises(self, evaluator, penalty_type):
        _, problem = _penalty_problem(
            (SEED_NODE_1,), [_seed_cost(penalty_type, 1.0)], constraint_sets=[_start_pin]
        )
        problem.convexify()
        nlp_point = np.array([0.0, SEED_NODE_1, 0.0])
        assert problem.getNumNLPVars() == len(nlp_point) < problem.getNumQPVars()

        with pytest.raises(ValueError, match=r"getNumQPVars\(\)"):
            getattr(problem, evaluator)(nlp_point)

    @pytest.mark.parametrize("evaluator", CONVEX_EVALUATORS)
    @_PENALTY_TYPES
    def test_over_long_var_vals_raises(self, evaluator, penalty_type):
        """The rule is exactly getNumQPVars() entries: one more raises too, although trajopt
        0.35.0 would read only the leading ones and return a value."""
        _, problem = _penalty_problem(
            (SEED_NODE_1,), [_seed_cost(penalty_type, 1.0)], constraint_sets=[_start_pin]
        )
        problem.convexify()
        over_long = np.zeros(problem.getNumQPVars() + 1)

        with pytest.raises(ValueError, match=r"getNumQPVars\(\)"):
            getattr(problem, evaluator)(over_long)

    @_PENALTY_TYPES
    def test_qp_sized_var_vals_is_accepted(self, penalty_type):
        """At the convexification point with its slacks at 0, each evaluator reads the exact
        values there: SEED_VIOLATION for the cost, and START_TARGET for the start pin, which
        the seed's x_0 = 0 misses by that much."""
        _, problem = _penalty_problem(
            (SEED_NODE_1,), [_seed_cost(penalty_type, 1.0)], constraint_sets=[_start_pin]
        )
        problem.convexify()
        n_slack = problem.getNumQPVars() - problem.getNumNLPVars()
        qp_point = np.concatenate([[0.0, SEED_NODE_1, 0.0], np.zeros(n_slack)])

        costs = problem.evaluateConvexCosts(qp_point)
        total = problem.evaluateTotalConvexCost(qp_point)
        violations = problem.evaluateConvexConstraintViolations(qp_point)

        assert costs == pytest.approx(problem.getExactCosts(), abs=MODEL_ROUND_OFF)
        assert total == pytest.approx(problem.getTotalExactCost(), abs=MODEL_ROUND_OFF)
        exact = problem.getExactConstraintViolations()
        assert violations.raw == pytest.approx(exact.raw, abs=MODEL_ROUND_OFF)
        assert violations.weighted == pytest.approx(exact.weighted, abs=MODEL_ROUND_OFF)
        assert costs.tolist() == [SEED_VIOLATION]
        assert violations.raw.tolist() == [START_TARGET]

    def test_before_convexify_raises(self):
        """No model before the first convexify(): every evaluator raises, for an empty and an
        NLP-sized var_vals alike. Runs in a child process: the call used to segfault, and a
        regression must fail this test, not the session."""
        proc = subprocess.run(
            [sys.executable, "-c", _BEFORE_CONVEXIFY_SCRIPT],
            capture_output=True,
            text=True,
            check=False,
            timeout=EVALUATOR_CHILD_TIMEOUT_S,
        )
        assert proc.returncode == 0, (
            f"child died (rc={proc.returncode}, SIGSEGV is -11/139): {proc.stderr[-500:]}"
        )
        outcomes = [ln for ln in proc.stdout.splitlines() if ln.startswith(("RAISED", "RETURNED"))]
        assert len(outcomes) == 2 * len(CONVEX_EVALUATORS), proc.stdout[-500:]
        for outcome in outcomes:
            assert outcome.startswith("RAISED") and "convexify()" in outcome, outcome
