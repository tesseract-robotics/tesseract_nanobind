/**
 * @file trajopt_sqp_bindings.cpp
 * @brief nanobind bindings for trajopt_sqp solver classes
 *
 * Exposes the TrustRegionSQPSolver for incremental optimization (real-time planning).
 * Key feature: stepSQPSolver() allows running a single SQP iteration.
 */

#include "tesseract_nb.h"

// OsqpEigen - must be included BEFORE trajopt_sqp headers to complete forward declaration
#include <OsqpEigen/Solver.hpp>

// trajopt_sqp headers
#include <trajopt_sqp/types.h>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/sqp_callback.h>

// ifopt headers for types used in QPProblem interface
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/problem.h>

namespace tsqp = trajopt_sqp;

// Trampoline for SQPCallback (allow Python subclasses)
class PySQPCallback : public tsqp::SQPCallback {
public:
    NB_TRAMPOLINE(tsqp::SQPCallback, 1);

    bool execute(const tsqp::QPProblem& problem, const tsqp::SQPResults& sqp_results) override {
        NB_OVERRIDE_PURE(execute, problem, sqp_results);
    }
};

NB_MODULE(_trajopt_sqp, m) {
    m.doc() = "trajopt_sqp Python bindings - SQP solver for trajectory optimization";

    // ========== Enums ==========

    nb::enum_<tsqp::ConstraintType>(m, "ConstraintType", "Type of constraint")
        .value("EQ", tsqp::ConstraintType::EQ, "Equality constraint")
        .value("INEQ", tsqp::ConstraintType::INEQ, "Inequality constraint");

    nb::enum_<tsqp::CostPenaltyType>(m, "CostPenaltyType", "Penalty type for cost terms")
        .value("SQUARED", tsqp::CostPenaltyType::SQUARED, "Squared penalty (L2)")
        .value("ABSOLUTE", tsqp::CostPenaltyType::ABSOLUTE, "Absolute penalty (L1)")
        .value("HINGE", tsqp::CostPenaltyType::HINGE, "Hinge penalty");

    nb::enum_<tsqp::SQPStatus>(m, "SQPStatus", "Status codes for SQP optimization")
        .value("RUNNING", tsqp::SQPStatus::RUNNING, "Optimization is currently running")
        .value("NLP_CONVERGED", tsqp::SQPStatus::NLP_CONVERGED, "NLP successfully converged")
        .value("ITERATION_LIMIT", tsqp::SQPStatus::ITERATION_LIMIT, "Reached iteration limit")
        .value("PENALTY_ITERATION_LIMIT", tsqp::SQPStatus::PENALTY_ITERATION_LIMIT, "Reached penalty iteration limit")
        .value("OPT_TIME_LIMIT", tsqp::SQPStatus::OPT_TIME_LIMIT, "Reached time limit")
        .value("QP_SOLVER_ERROR", tsqp::SQPStatus::QP_SOLVER_ERROR, "QP solver failed")
        .value("CALLBACK_STOPPED", tsqp::SQPStatus::CALLBACK_STOPPED, "Stopped by callback");

    nb::enum_<tsqp::QPSolverStatus>(m, "QPSolverStatus", "Status of QP solver")
        .value("UNITIALIZED", tsqp::QPSolverStatus::UNITIALIZED, "Solver not initialized")
        .value("INITIALIZED", tsqp::QPSolverStatus::INITIALIZED, "Solver initialized")
        .value("QP_ERROR", tsqp::QPSolverStatus::QP_ERROR, "QP solver error");

    // ========== SQPParameters ==========

    nb::class_<tsqp::SQPParameters>(m, "SQPParameters", "Parameters controlling SQP optimization")
        .def(nb::init<>())
        .def_rw("improve_ratio_threshold", &tsqp::SQPParameters::improve_ratio_threshold,
                "Minimum ratio exact_improve/approx_improve to accept step (default: 0.25)")
        .def_rw("min_trust_box_size", &tsqp::SQPParameters::min_trust_box_size,
                "NLP converges if trust region smaller than this (default: 1e-4)")
        .def_rw("min_approx_improve", &tsqp::SQPParameters::min_approx_improve,
                "NLP converges if approx_merit_improve smaller than this (default: 1e-4)")
        .def_rw("min_approx_improve_frac", &tsqp::SQPParameters::min_approx_improve_frac,
                "NLP converges if approx_merit_improve/best_exact_merit < this")
        .def_rw("max_iterations", &tsqp::SQPParameters::max_iterations,
                "Max number of QP calls allowed (default: 50)")
        .def_rw("trust_shrink_ratio", &tsqp::SQPParameters::trust_shrink_ratio,
                "Trust region scale factor when shrinking (default: 0.1)")
        .def_rw("trust_expand_ratio", &tsqp::SQPParameters::trust_expand_ratio,
                "Trust region scale factor when expanding (default: 1.5)")
        .def_rw("cnt_tolerance", &tsqp::SQPParameters::cnt_tolerance,
                "Constraint violation tolerance (default: 1e-4)")
        .def_rw("max_merit_coeff_increases", &tsqp::SQPParameters::max_merit_coeff_increases,
                "Max times constraints will be inflated (default: 5)")
        .def_rw("max_qp_solver_failures", &tsqp::SQPParameters::max_qp_solver_failures,
                "Max QP solver failures before abort (default: 3)")
        .def_rw("merit_coeff_increase_ratio", &tsqp::SQPParameters::merit_coeff_increase_ratio,
                "Scale factor for constraint inflation (default: 10)")
        .def_rw("max_time", &tsqp::SQPParameters::max_time,
                "Max optimization time in seconds")
        .def_rw("initial_merit_error_coeff", &tsqp::SQPParameters::initial_merit_error_coeff,
                "Initial constraint scaling coefficient (default: 10)")
        .def_rw("inflate_constraints_individually", &tsqp::SQPParameters::inflate_constraints_individually,
                "If true, only violated constraints are inflated (default: true)")
        .def_rw("initial_trust_box_size", &tsqp::SQPParameters::initial_trust_box_size,
                "Initial trust region size (default: 0.1)")
        .def_rw("log_results", &tsqp::SQPParameters::log_results, "Enable logging (unused)")
        .def_rw("log_dir", &tsqp::SQPParameters::log_dir, "Log directory (unused)");

    // ========== SQPResults ==========

    nb::class_<tsqp::SQPResults>(m, "SQPResults", "Results and state from SQP optimization")
        .def(nb::init<>())
        .def(nb::init<Eigen::Index, Eigen::Index, Eigen::Index>(),
             "num_vars"_a, "num_cnts"_a, "num_costs"_a)
        .def_rw("best_exact_merit", &tsqp::SQPResults::best_exact_merit,
                "Lowest cost ever achieved")
        .def_rw("new_exact_merit", &tsqp::SQPResults::new_exact_merit,
                "Cost achieved this iteration")
        .def_rw("best_approx_merit", &tsqp::SQPResults::best_approx_merit,
                "Lowest convexified cost ever achieved")
        .def_rw("new_approx_merit", &tsqp::SQPResults::new_approx_merit,
                "Convexified cost this iteration")
        .def_rw("best_var_vals", &tsqp::SQPResults::best_var_vals,
                "Variable values for best_exact_merit")
        .def_rw("new_var_vals", &tsqp::SQPResults::new_var_vals,
                "Variable values this iteration")
        .def_rw("approx_merit_improve", &tsqp::SQPResults::approx_merit_improve,
                "Convexified cost improvement this iteration")
        .def_rw("exact_merit_improve", &tsqp::SQPResults::exact_merit_improve,
                "Exact cost improvement this iteration")
        .def_rw("merit_improve_ratio", &tsqp::SQPResults::merit_improve_ratio,
                "Cost improvement as ratio of total cost")
        .def_rw("box_size", &tsqp::SQPResults::box_size,
                "Trust region box size (var_vals +/- box_size)")
        .def_rw("merit_error_coeffs", &tsqp::SQPResults::merit_error_coeffs,
                "Coefficients weighting constraint violations")
        .def_rw("best_constraint_violations", &tsqp::SQPResults::best_constraint_violations,
                "Constraint violations for best solution (positive = violation)")
        .def_rw("new_constraint_violations", &tsqp::SQPResults::new_constraint_violations,
                "Constraint violations this iteration")
        .def_rw("best_approx_constraint_violations", &tsqp::SQPResults::best_approx_constraint_violations,
                "Convexified constraint violations for best solution")
        .def_rw("new_approx_constraint_violations", &tsqp::SQPResults::new_approx_constraint_violations,
                "Convexified constraint violations this iteration")
        .def_rw("best_costs", &tsqp::SQPResults::best_costs,
                "Cost values for best solution")
        .def_rw("new_costs", &tsqp::SQPResults::new_costs,
                "Cost values this iteration")
        .def_rw("best_approx_costs", &tsqp::SQPResults::best_approx_costs,
                "Convexified costs for best solution")
        .def_rw("new_approx_costs", &tsqp::SQPResults::new_approx_costs,
                "Convexified costs this iteration")
        .def_rw("constraint_names", &tsqp::SQPResults::constraint_names,
                "Names of constraint sets")
        .def_rw("cost_names", &tsqp::SQPResults::cost_names,
                "Names of cost terms")
        .def_rw("penalty_iteration", &tsqp::SQPResults::penalty_iteration)
        .def_rw("convexify_iteration", &tsqp::SQPResults::convexify_iteration)
        .def_rw("trust_region_iteration", &tsqp::SQPResults::trust_region_iteration)
        .def_rw("overall_iteration", &tsqp::SQPResults::overall_iteration)
        .def("print", &tsqp::SQPResults::print, "Print results to console");

    // ========== QPSolver (abstract base) ==========

    nb::class_<tsqp::QPSolver>(m, "QPSolver",
        "Abstract base class for QP solvers")
        .def("init", &tsqp::QPSolver::init, "num_vars"_a, "num_cnts"_a,
             "Initialize the QP solver")
        .def("clear", &tsqp::QPSolver::clear, "Clear the QP solver")
        .def("solve", &tsqp::QPSolver::solve, "Solve the QP")
        .def("getSolution", &tsqp::QPSolver::getSolution, "Get the solution vector")
        .def("getSolverStatus", &tsqp::QPSolver::getSolverStatus, "Get solver status")
        .def_rw("verbosity", &tsqp::QPSolver::verbosity, "Verbosity level (0 = silent)");

    // ========== OSQPEigenSolver ==========
    nb::class_<tsqp::OSQPEigenSolver, tsqp::QPSolver>(
        m, "OSQPEigenSolver", "OSQP-based QP solver")
        .def(nb::init<>())
        .def("init", &tsqp::OSQPEigenSolver::init, "num_vars"_a, "num_cnts"_a)
        .def("clear", &tsqp::OSQPEigenSolver::clear)
        .def("solve", &tsqp::OSQPEigenSolver::solve)
        .def("getSolution", &tsqp::OSQPEigenSolver::getSolution)
        .def("getSolverStatus", &tsqp::OSQPEigenSolver::getSolverStatus)
        .def("updateGradient", &tsqp::OSQPEigenSolver::updateGradient, "gradient"_a)
        .def("updateLowerBound", &tsqp::OSQPEigenSolver::updateLowerBound, "lower_bound"_a)
        .def("updateUpperBound", &tsqp::OSQPEigenSolver::updateUpperBound, "upper_bound"_a)
        .def("updateBounds", &tsqp::OSQPEigenSolver::updateBounds,
             "lower_bound"_a, "upper_bound"_a);

    // ========== QPProblem (abstract base) ==========

    nb::class_<tsqp::QPProblem>(m, "QPProblem",
        "Abstract base class for QP problems (convexified NLP)")
        .def("addVariableSet", &tsqp::QPProblem::addVariableSet, "variable_set"_a,
             "Add a set of optimization variables")
        .def("addConstraintSet", &tsqp::QPProblem::addConstraintSet, "constraint_set"_a,
             "Add a set of constraints")
        .def("addCostSet", &tsqp::QPProblem::addCostSet,
             "constraint_set"_a, "penalty_type"_a,
             "Add a cost term with specified penalty type")
        .def("setup", &tsqp::QPProblem::setup,
             "Setup the QP problem (call after adding all sets)")
        .def("getVariableValues", &tsqp::QPProblem::getVariableValues,
             "Get current optimization variable values")
        .def("convexify", &tsqp::QPProblem::convexify,
             "Run the full convexification routine")
        .def("evaluateTotalConvexCost", &tsqp::QPProblem::evaluateTotalConvexCost,
             "var_vals"_a, "Evaluate convexified cost at given point")
        .def("evaluateConvexCosts", &tsqp::QPProblem::evaluateConvexCosts,
             "var_vals"_a, "Evaluate individual convexified costs")
        .def("evaluateTotalExactCost", &tsqp::QPProblem::evaluateTotalExactCost,
             "var_vals"_a, "Evaluate exact (non-convexified) cost")
        .def("evaluateExactCosts", &tsqp::QPProblem::evaluateExactCosts,
             "var_vals"_a, "Evaluate individual exact costs")
        .def("getExactCosts", &tsqp::QPProblem::getExactCosts,
             "Get current exact costs")
        .def("evaluateConvexConstraintViolations", &tsqp::QPProblem::evaluateConvexConstraintViolations,
             "var_vals"_a, "Evaluate convexified constraint violations")
        .def("evaluateExactConstraintViolations", &tsqp::QPProblem::evaluateExactConstraintViolations,
             "var_vals"_a, "Evaluate exact constraint violations")
        .def("getExactConstraintViolations", &tsqp::QPProblem::getExactConstraintViolations,
             "Get current exact constraint violations")
        .def("scaleBoxSize", &tsqp::QPProblem::scaleBoxSize, "scale"_a,
             "Uniformly scale the trust region box size")
        .def("setBoxSize", &tsqp::QPProblem::setBoxSize, "box_size"_a,
             "Set trust region box size")
        .def("setConstraintMeritCoeff", &tsqp::QPProblem::setConstraintMeritCoeff,
             "merit_coeff"_a, "Set constraint merit coefficients")
        .def("getBoxSize", nb::overload_cast<>(&tsqp::QPProblem::getBoxSize, nb::const_),
             "Get trust region box size")
        .def("print", &tsqp::QPProblem::print, "Print problem to console")
        .def("getNumNLPVars", &tsqp::QPProblem::getNumNLPVars,
             "Number of NLP variables")
        .def("getNumNLPConstraints", &tsqp::QPProblem::getNumNLPConstraints,
             "Number of NLP constraints")
        .def("getNumNLPCosts", &tsqp::QPProblem::getNumNLPCosts,
             "Number of NLP cost terms")
        .def("getNumQPVars", &tsqp::QPProblem::getNumQPVars,
             "Number of QP variables (includes slack)")
        .def("getNumQPConstraints", &tsqp::QPProblem::getNumQPConstraints,
             "Number of QP constraints")
        .def("getNLPConstraintNames", &tsqp::QPProblem::getNLPConstraintNames,
             nb::rv_policy::reference_internal, "Get constraint names")
        .def("getNLPCostNames", &tsqp::QPProblem::getNLPCostNames,
             nb::rv_policy::reference_internal, "Get cost names");

    // ========== IfoptQPProblem ==========

    nb::class_<tsqp::IfoptQPProblem, tsqp::QPProblem>(
        m, "IfoptQPProblem",
        "QP problem wrapper for ifopt::Problem (general NLP)")
        .def(nb::init<>())
        .def(nb::init<std::shared_ptr<ifopt::Problem>>(), "nlp"_a,
             "Construct from ifopt Problem")
        .def("addVariableSet", &tsqp::IfoptQPProblem::addVariableSet, "variable_set"_a)
        .def("addConstraintSet", &tsqp::IfoptQPProblem::addConstraintSet, "constraint_set"_a)
        .def("addCostSet", &tsqp::IfoptQPProblem::addCostSet,
             "constraint_set"_a, "penalty_type"_a)
        .def("setup", &tsqp::IfoptQPProblem::setup)
        .def("convexify", &tsqp::IfoptQPProblem::convexify)
        .def("print", &tsqp::IfoptQPProblem::print);

    // ========== TrajOptQPProblem ==========
    // NOTE: TrajOptQPProblem uses PIMPL with internal Implementation struct
    // that's not exposed in headers. We cannot bind it directly because
    // nanobind needs to see complete types for move/copy operations.
    // Use IfoptQPProblem or create TrajOptQPProblem via C++ factory functions.
    // For online planning, IfoptQPProblem is sufficient.

    // ========== SQPCallback ==========
    // NOTE: PySQPCallback trampoline allows Python subclasses
    nb::class_<tsqp::SQPCallback, PySQPCallback>(
        m, "SQPCallback", "Base class for SQP optimization callbacks")
        .def(nb::init<>())
        .def("execute", &tsqp::SQPCallback::execute, "problem"_a, "sqp_results"_a,
             "Called during SQP. Return false to stop optimization.");

    // ========== TrustRegionSQPSolver ==========

    nb::class_<tsqp::TrustRegionSQPSolver>(
        m, "TrustRegionSQPSolver",
        "Trust region SQP solver for trajectory optimization.\n\n"
        "Key methods for real-time/incremental optimization:\n"
        "  - init(qp_prob): Initialize solver with problem\n"
        "  - stepSQPSolver(): Run ONE SQP iteration (for real-time control)\n"
        "  - getResults(): Get current optimization results\n"
        "  - setBoxSize(): Control trust region size")
        .def(nb::init<std::shared_ptr<tsqp::QPSolver>>(), "qp_solver"_a,
             "Construct with a QP solver (e.g., OSQPEigenSolver)")
        .def("init", &tsqp::TrustRegionSQPSolver::init, "qp_prob"_a,
             "Initialize the solver with a QP problem for incremental solving")
        .def("solve", &tsqp::TrustRegionSQPSolver::solve, "qp_prob"_a,
             "Run complete SQP optimization")
        .def("stepSQPSolver", &tsqp::TrustRegionSQPSolver::stepSQPSolver,
             "Run a SINGLE SQP convexification step.\n\n"
             "This is the key method for real-time/online planning.\n"
             "Returns True if QP solve converged (does not mean SQP converged).\n\n"
             "Typical usage:\n"
             "  solver.init(problem)\n"
             "  while solver.getStatus() == SQPStatus.RUNNING:\n"
             "      solver.stepSQPSolver()\n"
             "      # Check constraints, update environment, etc.")
        .def("verifySQPSolverConvergence", &tsqp::TrustRegionSQPSolver::verifySQPSolverConvergence,
             "Check if SQP constraints are satisfied")
        .def("adjustPenalty", &tsqp::TrustRegionSQPSolver::adjustPenalty,
             "Increase penalty on constraints when SQP reports convergence but constraints violated")
        .def("runTrustRegionLoop", &tsqp::TrustRegionSQPSolver::runTrustRegionLoop,
             "Run trust region loop (adjusts box size)")
        .def("solveQPProblem", &tsqp::TrustRegionSQPSolver::solveQPProblem,
             "Solve current QP problem, store results, call callbacks")
        .def("setBoxSize", &tsqp::TrustRegionSQPSolver::setBoxSize, "box_size"_a,
             "Set trust region box size (for online planning control)")
        .def("callCallbacks", &tsqp::TrustRegionSQPSolver::callCallbacks,
             "Call all registered callbacks. Returns false if any callback returned false.")
        .def("printStepInfo", &tsqp::TrustRegionSQPSolver::printStepInfo,
             "Print info about current optimization state")
        .def("registerCallback", &tsqp::TrustRegionSQPSolver::registerCallback,
             "callback"_a, "Register an optimization callback")
        .def("getStatus", &tsqp::TrustRegionSQPSolver::getStatus,
             nb::rv_policy::reference_internal, "Get current SQP status")
        .def("getResults", &tsqp::TrustRegionSQPSolver::getResults,
             nb::rv_policy::reference_internal, "Get current SQP results")
        .def_rw("verbose", &tsqp::TrustRegionSQPSolver::verbose,
                "If true, print debug info to console")
        .def_rw("params", &tsqp::TrustRegionSQPSolver::params,
                "SQP parameters (modify before calling init/solve)")
        .def_rw("qp_solver", &tsqp::TrustRegionSQPSolver::qp_solver,
                "The QP solver used internally")
        .def_rw("qp_problem", &tsqp::TrustRegionSQPSolver::qp_problem,
                "The current QP problem");

    // ========== ifopt types (minimal bindings for use with QPProblem) ==========
    // NOTE: ifopt::VariableSet and ifopt::ConstraintSet are already bound in the
    // _ifopt module. We only bind ifopt::Problem here since it's not in _ifopt.

    // ifopt::Problem - needed to construct IfoptQPProblem
    nb::class_<ifopt::Problem>(m, "IfoptProblem",
        "ifopt::Problem - generic NLP with variables, costs, constraints")
        .def(nb::init<>())
        .def("AddVariableSet", &ifopt::Problem::AddVariableSet, "variable_set"_a)
        .def("AddConstraintSet", &ifopt::Problem::AddConstraintSet, "constraint_set"_a)
        .def("AddCostSet", &ifopt::Problem::AddCostSet, "cost_set"_a)
        .def("GetNumberOfOptimizationVariables", &ifopt::Problem::GetNumberOfOptimizationVariables)
        .def("GetNumberOfConstraints", &ifopt::Problem::GetNumberOfConstraints)
        .def("GetVariableValues", &ifopt::Problem::GetVariableValues)
        .def("PrintCurrent", &ifopt::Problem::PrintCurrent);
}
