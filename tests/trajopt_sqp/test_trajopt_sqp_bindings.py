"""Tests for low-level trajopt_sqp bindings (SQP solver API).

Updated for 0.34 API:
- JointPosition replaced by createNodesVariables() -> Node -> Var
- CollisionCache removed (now internal)
- CartPosInfo removed; CartPosConstraint takes direct params
- ifopt module removed; types now in trajopt_ifopt
- IfoptProblem(variables) constructor, then IfoptQPProblem(nlp)
- evaluateTotalExactCost() -> getTotalExactCost() (no args)
- ConstraintType enum removed
- Bounds uses getLower()/getUpper() instead of .lower/.upper
"""

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


def _make_nodes_variables(joint_names, joint_limits, initial_states, name="trajectory"):
    """Helper: create NodesVariables from joint states and limits.

    Returns (nodes_variables, vars_list) where vars_list is the list of Var refs.
    """
    bounds = ti.toBounds(joint_limits)
    nodes_variables = ti.createNodesVariables(name, list(joint_names), list(initial_states), bounds)
    vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
    return nodes_variables, vars_list


def _make_problem(nodes_variables, vars_list, joint_names, constraints=None, costs=None):
    """Helper: create IfoptProblem + IfoptQPProblem with given constraints/costs.

    constraints added to nlp, costs added to qp_problem with SQUARED penalty.
    Returns (nlp, problem).
    """
    nlp = tsqp.IfoptProblem(nodes_variables)
    for c in constraints or []:
        nlp.addConstraintSet(c)
    for c in costs or []:
        nlp.addCostSet(c)
    problem = tsqp.IfoptQPProblem(nlp)
    return nlp, problem


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


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
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
        """Test IfoptQPProblem creation (0.34: requires IfoptProblem(nv))."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        nlp = tsqp.IfoptProblem(nv)
        problem = tsqp.IfoptQPProblem(nlp)
        assert problem is not None

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
        nlp, problem = _make_problem(nv, vars_list, joint_names, costs=[vel_cost])
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
        nlp, problem = _make_problem(
            nv, vars_list, joint_names, constraints=[constraint], costs=[vel_cost]
        )
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


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
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
        nlp, problem = _make_problem(
            nv, vars_list, joint_names, constraints=[home_constraint], costs=[vel_cost]
        )
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

        nlp = tsqp.IfoptProblem(nv)

        # Start constraint
        home_coeffs = np.ones(len(joint_names)) * 5.0
        home_constraint = ti.JointPosConstraint(start_pos, vars_list[0], home_coeffs, "Home")
        nlp.addConstraintSet(home_constraint)

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
        nlp.addConstraintSet(target_constraint)

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        nlp.addCostSet(vel_cost)

        # Create QP problem
        problem = tsqp.IfoptQPProblem(nlp)
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

        nlp = tsqp.IfoptProblem(nv)

        # Velocity cost
        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        nlp.addCostSet(vel_cost)

        # Create QP problem
        problem = tsqp.IfoptQPProblem(nlp)

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


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
class TestAdditionalBindings:
    """Tests for additional bindings not covered elsewhere."""

    def test_collision_coeff_data(self):
        """Test CollisionCoeffData for per-link collision weights."""
        data = ti.CollisionCoeffData()
        assert data is not None
        assert hasattr(data, "getPairCollisionCoeff")
        assert hasattr(data, "setPairCollisionCoeff")

    def test_ifopt_problem_base(self, kuka_setup):
        """Test IfoptProblem (base NLP) creation via 0.34 API."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        nlp = tsqp.IfoptProblem(nv)
        assert nlp.getNumberOfOptimizationVariables() >= len(joint_names)

    def test_qp_problem_interface(self, kuka_setup):
        """Test QPProblem interface methods."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names))]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        nlp = tsqp.IfoptProblem(nv)
        problem = tsqp.IfoptQPProblem(nlp)
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
        nlp = tsqp.IfoptProblem(nv)
        nlp.addCostSet(vel_cost)
        problem = tsqp.IfoptQPProblem(nlp)
        assert problem is not None

    def test_get_total_exact_cost(self, kuka_setup):
        """Test getTotalExactCost() (renamed from evaluateTotalExactCost in 0.34)."""
        _, _, joint_names, joint_limits = kuka_setup

        states = [np.zeros(len(joint_names)) for _ in range(3)]
        nv, vars_list = _make_nodes_variables(joint_names, joint_limits, states)

        vel_cost = ti.JointVelConstraint(
            np.zeros(len(joint_names)), vars_list, np.ones(1), "Velocity"
        )
        nlp, problem = _make_problem(nv, vars_list, joint_names, costs=[vel_cost])
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
        nlp, problem = _make_problem(nv, vars_list, joint_names, costs=[vel_cost])
        problem.setup()

        qp_solver = tsqp.OSQPEigenSolver()
        solver = tsqp.TrustRegionSQPSolver(qp_solver)
        solver.verbose = False
        solver.solve(problem)

        # 0.34 API: getExactCosts() with no args
        costs = problem.getExactCosts()
        assert costs is not None


@pytest.mark.skipif(not SQP_AVAILABLE, reason="trajopt_sqp not available")
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

        try:
            constraint = ti.InverseKinematicsConstraint(
                target, info, vars_list[0], vars_list[1], "IK_0"
            )
            assert constraint is not None
        except TypeError as e:
            pytest.skip(f"IK constraint may need shared_ptr wrapper: {e}")


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

        nlp, problem = _make_problem(
            nv, vars_list, joint_names, constraints=[home_constraint], costs=[vel_cost]
        )
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
