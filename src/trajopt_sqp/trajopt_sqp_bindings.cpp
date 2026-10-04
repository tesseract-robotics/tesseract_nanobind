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
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/sqp_callback.h>

// trajopt_ifopt headers for types used in QPProblem interface
#include <trajopt_ifopt/core/component.h>
#include <trajopt_ifopt/core/constraint_set.h>

namespace tsqp = trajopt_sqp;

namespace {
// The convex evaluators read var_vals in the layout of the last convexify(): the NLP variables
// followed by the slack variables, getNumQPVars() entries, as trajopt documents ("Should be size
// num_qp_vars", qp_problem.h:59, :67) but does not check. trajopt 0.35.0 (every 0.35.0.x wheel)
// multiplies a penalty cost's full QP rows into var_vals (trajopt_qp_problem.cpp:187-188), so a
// shorter vector is read past its end; before the first convexify() getNumQPVars() is 0 and an
// empty or NLP-sized vector segfaults on a squared cost and reads garbage otherwise. The binding
// owns the Python boundary, so validate here and fail loud (std::invalid_argument -> ValueError).
// One contract for all three evaluators, although trajopt 0.35.0 reads only the NLP block in two.
void validate_qp_solution(const tsqp::QPProblem& problem,
                          const Eigen::Ref<const Eigen::VectorXd>& var_vals,
                          const char* method)
{
    const Eigen::Index n_qp = problem.getNumQPVars();
    if (n_qp < problem.getNumNLPVars())
        throw std::invalid_argument(std::string(method) +
                                    ": the problem has no convex model yet; call convexify() first");
    if (var_vals.size() != n_qp)
        throw std::invalid_argument(std::string(method) + ": var_vals has " +
                                    std::to_string(var_vals.size()) +
                                    " entries; it must be the QP solution vector of getNumQPVars() = " +
                                    std::to_string(n_qp) +
                                    " entries, the NLP variables followed by the slack variables");
}
}  // namespace

// Trampoline for SQPCallback (allow Python subclasses)
class PySQPCallback : public tsqp::SQPCallback {
public:
    NB_TRAMPOLINE(tsqp::SQPCallback, 1);

    bool execute(const tsqp::QPProblem& problem, const tsqp::SQPResults& sqp_results) override {
        // Mirrors nanobind 2.12.0 trampoline.h NB_OVERRIDE_PURE_NAME, with explicit argument
        // policies: its default for const& arguments is a copy, which aborts on a non-copyable
        // problem (TrajOptQPProblem) and deep-copied every other problem on every trial. The
        // problem goes by reference (Python gets the object passed to solve()); a problem with
        // no Python instance (none reachable today) would arrive as a non-owning view valid
        // only for the duration of the call. The results stay a copy, a per-trial snapshot.
        nanobind::detail::ticket nb_ticket(nb_trampoline, "execute", true);  // takes the GIL
        return nb::cast<bool>(nb_trampoline.base().attr(nb_ticket.key)(
            nb::cast(problem, nb::rv_policy::reference),
            nb::cast(sqp_results, nb::rv_policy::copy)));
    }
};

// trajopt 0.35.0 defaults TrajOptQPProblem's move constructor in its header, where the PIMPL
// Implementation is incomplete, so that constructor compiles only inside
// trajopt_qp_problem.cpp. nanobind's class_ instantiates the move constructor of every
// move-constructible type it binds (to return it by value). This subclass adds no state and no
// overrides and deletes copy and move; Python sees it as TrajOptQPProblem. Bind
// tsqp::TrajOptQPProblem directly once trajopt defaults the move out of line.
class TrajOptQPProblemBinding final : public tsqp::TrajOptQPProblem {
public:
    using tsqp::TrajOptQPProblem::TrajOptQPProblem;
    TrajOptQPProblemBinding(const TrajOptQPProblemBinding&) = delete;
    TrajOptQPProblemBinding& operator=(const TrajOptQPProblemBinding&) = delete;
    TrajOptQPProblemBinding(TrajOptQPProblemBinding&&) = delete;
    TrajOptQPProblemBinding& operator=(TrajOptQPProblemBinding&&) = delete;
    ~TrajOptQPProblemBinding() override = default;
};

NB_MODULE(_trajopt_sqp, m) {
    m.doc() = "trajopt_sqp Python bindings - SQP solver for trajectory optimization";

    // Import trajopt_ifopt module for cross-module type resolution
    nb::module_::import_("tesseract_robotics.trajopt_ifopt._trajopt_ifopt");

    // ========== Enums ==========

    nb::enum_<tsqp::CostPenaltyType>(m, "CostPenaltyType", "Penalty type for cost terms")
        .value("SQUARED", tsqp::CostPenaltyType::kSquared, "Squared penalty (L2)")
        .value("ABSOLUTE", tsqp::CostPenaltyType::kAbsolute, "Absolute penalty (L1)")
        .value("HINGE", tsqp::CostPenaltyType::kHinge, "Hinge penalty");

    nb::enum_<tsqp::SQPStatus>(m, "SQPStatus", "Status codes for SQP optimization")
        .value("RUNNING", tsqp::SQPStatus::kRunning, "Optimization is currently running")
        .value("NLP_CONVERGED", tsqp::SQPStatus::kConverged, "NLP successfully converged")
        .value("ITERATION_LIMIT", tsqp::SQPStatus::kIterationLimit, "Reached iteration limit")
        .value("PENALTY_ITERATION_LIMIT", tsqp::SQPStatus::kPenaltyIterationLimit, "Reached penalty iteration limit")
        .value("OPT_TIME_LIMIT", tsqp::SQPStatus::kTimeLimit, "Reached time limit")
        .value("QP_SOLVER_ERROR", tsqp::SQPStatus::kQPSolveFailed, "QP solver failed")
        .value("CALLBACK_STOPPED", tsqp::SQPStatus::kStoppedByCallback, "Stopped by callback");

    nb::enum_<tsqp::QPSolverStatus>(m, "QPSolverStatus", "Status of QP solver")
        .value("UNITIALIZED", tsqp::QPSolverStatus::kUninitialized, "Solver not initialized")
        .value("INITIALIZED", tsqp::QPSolverStatus::kInitialized, "Solver initialized")
        .value("QP_ERROR", tsqp::QPSolverStatus::kFailed, "QP solver error");

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

    nb::class_<tsqp::ConstraintViolations>(m, "ConstraintViolations",
        "Per merit unit constraint violations (non-negative; 0 = satisfied)")
        .def(nb::init<>())
        .def_rw("raw", &tsqp::ConstraintViolations::raw,
                "Unweighted violation per merit unit; its sum is compared against cnt_tolerance")
        .def_rw("weighted", &tsqp::ConstraintViolations::weighted,
                "Violation times row weight; the merit charges weighted.dot(merit_error_coeffs)");

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
             "lower_bound"_a, "upper_bound"_a)
        // OSQP settings — forwarded to solver_->settings()
        // Defaults (set in C++ constructor): polish=true, warmStart=true,
        // adaptiveRho=true, maxIter=8192, absTol=1e-4, relTol=1e-6
        .def("setPolish", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setPolish(v); }, "polish"_a,
            "Enable solution polishing (default: true)")
        .def("setWarmStart", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setWarmStart(v); }, "warm_start"_a,
            "Enable warm-starting (default: true)")
        .def("setAdaptiveRho", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setAdaptiveRho(v); }, "adaptive_rho"_a,
            "Enable adaptive step size (default: true)")
        .def("setMaxIteration", [](tsqp::OSQPEigenSolver& self, int v) {
            self.solver_->settings()->setMaxIteration(v); }, "max_iter"_a,
            "Max OSQP iterations per QP solve (default: 8192)")
        .def("setAbsoluteTolerance", [](tsqp::OSQPEigenSolver& self, double v) {
            self.solver_->settings()->setAbsoluteTolerance(v); }, "abs_tol"_a,
            "Absolute convergence tolerance (default: 1e-4)")
        .def("setRelativeTolerance", [](tsqp::OSQPEigenSolver& self, double v) {
            self.solver_->settings()->setRelativeTolerance(v); }, "rel_tol"_a,
            "Relative convergence tolerance (default: 1e-6)")
        .def("setVerbosity", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setVerbosity(v); }, "verbose"_a,
            "Enable OSQP console output (default: false)");

    // ========== QPProblem (abstract base) ==========

    nb::class_<tsqp::QPProblem>(m, "QPProblem",
        "Abstract base class for QP problems (convexified NLP)")
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
        .def("evaluateTotalConvexCost",
             [](const tsqp::QPProblem& self, const Eigen::Ref<const Eigen::VectorXd>& var_vals) {
                 validate_qp_solution(self, var_vals, "evaluateTotalConvexCost");
                 return self.evaluateTotalConvexCost(var_vals);
             },
             "var_vals"_a,
             "Evaluate the convexified total cost at var_vals.\n\n"
             "Args:\n"
             "    var_vals: A point of the QP built by the last convexify(): getNumQPVars()\n"
             "        entries, the NLP variables followed by the slack variables, as in a QP\n"
             "        solution. To evaluate at an NLP point, append zeros for the slacks.\n\n"
             "Returns:\n"
             "    The sum of evaluateConvexCosts(var_vals).\n\n"
             "Raises:\n"
             "    ValueError: if var_vals does not have getNumQPVars() entries, or before the\n"
             "        first convexify().")
        .def("evaluateConvexCosts",
             [](const tsqp::QPProblem& self, const Eigen::Ref<const Eigen::VectorXd>& var_vals) {
                 validate_qp_solution(self, var_vals, "evaluateConvexCosts");
                 return self.evaluateConvexCosts(var_vals);
             },
             "var_vals"_a,
             "Evaluate each cost term of the convexified problem at var_vals.\n\n"
             "Args:\n"
             "    var_vals: A point of the QP built by the last convexify(): getNumQPVars()\n"
             "        entries, the NLP variables followed by the slack variables, as in a QP\n"
             "        solution. To evaluate at an NLP point, append zeros for the slacks.\n\n"
             "Returns:\n"
             "    One convexified cost per cost term, in getNLPCostNames() order.\n\n"
             "Raises:\n"
             "    ValueError: if var_vals does not have getNumQPVars() entries, or before the\n"
             "        first convexify().")
        .def("getTotalExactCost", &tsqp::QPProblem::getTotalExactCost,
             "Get exact (non-convexified) total cost")
        .def("getExactCosts", &tsqp::QPProblem::getExactCosts,
             "Get current exact costs")
        .def("evaluateConvexConstraintViolations",
             [](const tsqp::QPProblem& self, const Eigen::Ref<const Eigen::VectorXd>& var_vals) {
                 validate_qp_solution(self, var_vals, "evaluateConvexConstraintViolations");
                 return self.evaluateConvexConstraintViolations(var_vals);
             },
             "var_vals"_a,
             "Evaluate the convexified constraint violations at var_vals.\n\n"
             "Args:\n"
             "    var_vals: A point of the QP built by the last convexify(): getNumQPVars()\n"
             "        entries, the NLP variables followed by the slack variables, as in a QP\n"
             "        solution. To evaluate at an NLP point, append zeros for the slacks.\n\n"
             "Returns:\n"
             "    One violation per constraint set, in getNLPConstraintNames() order; 0 where\n"
             "    satisfied.\n\n"
             "Raises:\n"
             "    ValueError: if var_vals does not have getNumQPVars() entries, or before the\n"
             "        first convexify().")
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

    // ========== TrajOptQPProblem ==========
    // The QP problem tesseract_planning's TrajOpt-Ifopt planner builds
    // (trajopt_ifopt_motion_planner.cpp): costs and constraints go straight in. Its exact merit
    // weights each squared cost row by its coefficient, as its convex model does, and counts
    // every cost term once; it keeps one merit coefficient per constraint set. nanobind binds
    // TrajOptQPProblemBinding (above): trajopt 0.35.0 defaults the move constructor in its
    // header, where the PIMPL type is incomplete.
    nb::class_<TrajOptQPProblemBinding, tsqp::QPProblem>(
        m, "TrajOptQPProblem",
        "QP problem over trajopt_ifopt variables: the problem tesseract_planning's\n"
        "TrajOpt-Ifopt planner builds. Costs and constraints are added directly.")
        .def(nb::init<std::shared_ptr<trajopt_ifopt::Variables>>(), "variables"_a,
             "Construct over the optimization variables (e.g. createNodesVariables(...))")
        .def("addConstraintSet", &tsqp::TrajOptQPProblem::addConstraintSet, "constraint_set"_a)
        .def("addCostSet", &tsqp::TrajOptQPProblem::addCostSet,
             "constraint_set"_a, "penalty_type"_a)
        .def("setup", &tsqp::TrajOptQPProblem::setup)
        .def("convexify", &tsqp::TrajOptQPProblem::convexify)
        .def("print", &tsqp::TrajOptQPProblem::print);

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
             "Run complete SQP optimization", nb::call_guard<nb::gil_scoped_release>())
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
}
