/**
 * @file trajopt_sqp.cpp
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
#include <trajopt_sqp/piqp_solver.h>
#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/sqp_callback.h>

// trajopt_ifopt headers for types used in QPProblem interface
#include <trajopt_ifopt/core/component.h>
#include <trajopt_ifopt/core/constraint_set.h>

namespace tsqp = trajopt_sqp;

namespace {
// A convex evaluator called with a var_vals that is not getNumQPVars() long
struct QPSolutionSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A convex evaluator called before the first convexify(): there is no convex model to evaluate
struct NotConvexifiedError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// The convex evaluators take a point of the QP the last convexify() built: the NLP variables
// followed by the slack variables, getNumQPVars() entries, as trajopt documents
// (qp_problem.h:61, :69, :92) but does not check. trajopt reads only the NLP block
// (var_vals.head(n_nlp_vars), trajopt_qp_problem.cpp:246, :309, since trajopt#592), so a vector
// shorter than getNumNLPVars() is read past its end. Before the first convexify() there is no
// model: a squared cost segfaults, and a penalty cost or a constraint reads 0 whatever its
// violation. Measured on trajopt a504b22 with this check off, in child interpreters. The binding
// owns the Python boundary, so it validates here and fails loud (NotConvexifiedError,
// QPSolutionSizeError, both ValueError): one contract for all three evaluators, trajopt's
// documented size and not the NLP-sized read it does today.
void validate_qp_solution(const tsqp::QPProblem& problem,
                          const Eigen::Ref<const Eigen::VectorXd>& var_vals,
                          const char* method)
{
    const Eigen::Index n_qp = problem.getNumQPVars();
    if (n_qp < problem.getNumNLPVars())
        throw NotConvexifiedError(std::string(method) +
                                  ": the problem has no convex model yet; call convexify() first");
    if (var_vals.size() != n_qp)
        throw QPSolutionSizeError(std::string(method) + ": var_vals has " + std::to_string(var_vals.size()) +
                                  " entries; it must be the QP solution vector of getNumQPVars() = " +
                                  std::to_string(n_qp) +
                                  " entries, the NLP variables followed by the slack variables");
}

// QPProblem::setVariables(const double* x) reads getNumNLPVars() doubles through an Eigen::Map
// without a length (trajopt_qp_problem.cpp:702, trajopt 0.35.0).
struct VariablesSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// OSQPEigenSolver's update methods size their input against init()'s (num_vars, num_cnts) with
// assert() only, or not at all (osqp_eigen_solver.cpp, trajopt 0.35.0), and setWarmStart writes
// the problem's variables into an init()-sized vector: a mismatch reads or writes out of range.
struct QPSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// (num_vars, num_cnts) from the last init(). OSQPEigenSolver keeps them private; its OSQP data
// holds the same two numbers. It is the only QPSolver: QPSolver has no Python trampoline.
std::pair<Eigen::Index, Eigen::Index> qp_solver_size(const tsqp::QPSolver& self) {
    const auto* osqp = dynamic_cast<const tsqp::OSQPEigenSolver*>(&self);
    if (osqp == nullptr)
        throw std::logic_error("QPSolver: the binding sizes only OSQPEigenSolver");
    const auto* data = osqp->solver_->data()->getData();
    return { static_cast<Eigen::Index>(data->n), static_cast<Eigen::Index>(data->m) };
}

void require_size(const char* method, const char* what, Eigen::Index size, const char* expected_name,
                  Eigen::Index expected) {
    if (size != expected)
        throw QPSizeError(std::string(method) + ": " + what + " has " + std::to_string(size) +
                          "; init() set " + expected_name + " = " + std::to_string(expected));
}

bool set_warm_start_checked(tsqp::QPSolver& self, const tsqp::QPProblem& qp_problem) {
    const auto [n_vars, n_cnts] = qp_solver_size(self);
    require_size("setWarmStart", "the problem's getNumQPVars()", qp_problem.getNumQPVars(), "num_vars", n_vars);
    require_size("setWarmStart", "the problem's getNumQPConstraints()", qp_problem.getNumQPConstraints(),
                 "num_cnts", n_cnts);
    return self.setWarmStart(qp_problem);
}

// The convex model's sparse matrices; a copy, compressed for nanobind's caster
trajopt_ifopt::Jacobian compressed(const trajopt_ifopt::Jacobian& matrix) {
    trajopt_ifopt::Jacobian copy = matrix;
    copy.makeCompressed();
    return copy;
}
}  // namespace

// A QPProblem used after the SQPCallback call it was handed to has returned
struct QPProblemExpiredError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// The problem an SQPCallback receives when it has no Python object: one a planner built in C++
// (TrajOptIfoptMotionPlanner), freed when its solve returns. Python owns this view, at its own
// address, so nanobind's instance map never holds an entry at the problem's address; expire()
// cuts the view off when the call returns, so a callback that stored it gets
// QPProblemExpiredError instead of reading freed memory.
class QPProblemView final : public tsqp::QPProblem {
public:
    explicit QPProblemView(const tsqp::QPProblem& problem)
        // The callback API passes const&; Python reaches the mutators (setVariables, ...) on the
        // solver's problem as it does through the reference cast of a Python-owned problem.
        : problem_(const_cast<tsqp::QPProblem*>(&problem)) {}

    void expire() { problem_ = nullptr; }

    void addConstraintSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set) override {
        live().addConstraintSet(std::move(constraint_set));
    }
    void addCostSet(std::shared_ptr<trajopt_ifopt::ConstraintSet> constraint_set,
                    tsqp::CostPenaltyType penalty_type) override {
        live().addCostSet(std::move(constraint_set), penalty_type);
    }
    void setup() override { live().setup(); }
    void setVariables(const double* x) override { live().setVariables(x); }
    Eigen::VectorXd getVariableValues() const override { return live().getVariableValues(); }
    void convexify() override { live().convexify(); }
    double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override {
        return live().evaluateTotalConvexCost(var_vals);
    }
    Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override {
        return live().evaluateConvexCosts(var_vals);
    }
    double getTotalExactCost() const override { return live().getTotalExactCost(); }
    Eigen::VectorXd getExactCosts() const override { return live().getExactCosts(); }
    tsqp::ConstraintViolations
    evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) const override {
        return live().evaluateConvexConstraintViolations(var_vals);
    }
    tsqp::ConstraintViolations getExactConstraintViolations() const override {
        return live().getExactConstraintViolations();
    }
    void scaleBoxSize(double& scale) override { live().scaleBoxSize(scale); }
    void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override { live().setBoxSize(box_size); }
    void setConstraintMeritCoeff(const Eigen::Ref<const Eigen::VectorXd>& merit_coeff) override {
        live().setConstraintMeritCoeff(merit_coeff);
    }
    void setSumConstraintSetViolations(bool sum) override { live().setSumConstraintSetViolations(sum); }
    void print() const override { live().print(); }
    Eigen::Index getNumNLPVars() const override { return live().getNumNLPVars(); }
    Eigen::Index getNumNLPConstraints() const override { return live().getNumNLPConstraints(); }
    Eigen::Index getNumNLPCosts() const override { return live().getNumNLPCosts(); }
    Eigen::Index getNumQPVars() const override { return live().getNumQPVars(); }
    Eigen::Index getNumQPConstraints() const override { return live().getNumQPConstraints(); }
    const std::vector<std::string>& getNLPConstraintNames() const override { return live().getNLPConstraintNames(); }
    const std::vector<std::string>& getNLPCostNames() const override { return live().getNLPCostNames(); }
    const Eigen::VectorXd& getBoxSize() const override { return live().getBoxSize(); }
    const Eigen::VectorXd& getConstraintMeritCoeff() const override { return live().getConstraintMeritCoeff(); }
    const trajopt_ifopt::Jacobian& getHessian() const override { return live().getHessian(); }
    const Eigen::VectorXd& getGradient() const override { return live().getGradient(); }
    const trajopt_ifopt::Jacobian& getConstraintMatrix() const override { return live().getConstraintMatrix(); }
    const Eigen::VectorXd& getBoundsLower() const override { return live().getBoundsLower(); }
    const Eigen::VectorXd& getBoundsUpper() const override { return live().getBoundsUpper(); }

private:
    tsqp::QPProblem& live() const {
        if (problem_ == nullptr)
            throw QPProblemExpiredError(
                "QPProblem: the problem an SQPCallback received is valid only during that call; the "
                "solve that owned it has returned");
        return *problem_;
    }

    tsqp::QPProblem* problem_;
};

// Trampoline for SQPCallback (allow Python subclasses)
class PySQPCallback : public tsqp::SQPCallback {
public:
    NB_TRAMPOLINE(tsqp::SQPCallback, 1);

    bool execute(const tsqp::QPProblem& problem, const tsqp::SQPResults& sqp_results) override {
        // Mirrors nanobind 2.12.0 trampoline.h NB_OVERRIDE_PURE_NAME, with explicit argument
        // policies: its default for const& arguments is a copy, which aborts on a non-copyable
        // problem (TrajOptQPProblem) and deep-copied every other problem on every trial. A problem
        // with a Python object (one passed to solve() from Python) arrives as that object. One
        // without (built by a planner in C++) arrives as a QPProblemView that expires when the
        // call returns. The results stay a copy, a per-trial snapshot.
        nanobind::detail::ticket nb_ticket(nb_trampoline, "execute", true);  // takes the GIL
        nb::object results = nb::cast(sqp_results, nb::rv_policy::copy);
        nb::object existing = nb::find(problem);
        if (existing.is_valid())
            return nb::cast<bool>(nb_trampoline.base().attr(nb_ticket.key)(existing, results));
        nb::object view_object = nb::cast(std::make_unique<QPProblemView>(problem));
        auto& view = nb::cast<QPProblemView&>(view_object);
        struct Expire {  // also when the callback raises
            QPProblemView& view;
            ~Expire() { view.expire(); }
        } expire{ view };
        return nb::cast<bool>(nb_trampoline.base().attr(nb_ticket.key)(view_object, results));
    }
};

NB_MODULE(trajopt_sqp, m) {
    m.doc() = "trajopt_sqp Python bindings - SQP solver for trajectory optimization";
    register_python_error_translator();  // SQPCallback trampoline

    nb::exception<QPProblemExpiredError>(m, "QPProblemExpiredError", PyExc_RuntimeError)
        .attr("__doc__") = "A QPProblemView used after the SQPCallback call it was passed to returned.";
    nb::exception<VariablesSizeError>(m, "VariablesSizeError", PyExc_ValueError)
        .attr("__doc__") = "QPProblem.setVariables with a vector that is not getNumNLPVars() long.";
    nb::exception<QPSizeError>(m, "QPSizeError", PyExc_ValueError)
        .attr("__doc__") = "QPSolver input whose size is not the (num_vars, num_cnts) of its init().";
    nb::exception<QPSolutionSizeError>(m, "QPSolutionSizeError", PyExc_ValueError)
        .attr("__doc__") = "A convex evaluator's var_vals that is not getNumQPVars() long.";
    nb::exception<NotConvexifiedError>(m, "NotConvexifiedError", PyExc_ValueError)
        .attr("__doc__") = "A convex evaluator called before the problem's first convexify().";

    // Import trajopt_ifopt module for cross-module type resolution
    nb::module_::import_("tesseract_robotics.trajopt_ifopt");

    // ========== Enums ==========

    nb::enum_<tsqp::CostPenaltyType>(m, "CostPenaltyType", "Penalty type for cost terms")
        .value("SQUARED", tsqp::CostPenaltyType::kSquared, "Squared penalty (L2)")
        .value("ABSOLUTE", tsqp::CostPenaltyType::kAbsolute, "Absolute penalty (L1)")
        .value("HINGE", tsqp::CostPenaltyType::kHinge, "Hinge penalty");

    nb::enum_<tsqp::SQPStatus>(m, "SQPStatus", "Status codes for SQP optimization")
        .value("RUNNING", tsqp::SQPStatus::kRunning, "Optimization is currently running")
        .value("CONVERGED", tsqp::SQPStatus::kConverged, "Optimization converged")
        .value("ITERATION_LIMIT", tsqp::SQPStatus::kIterationLimit, "Reached iteration limit")
        .value("PENALTY_ITERATION_LIMIT", tsqp::SQPStatus::kPenaltyIterationLimit, "Reached penalty iteration limit")
        .value("TIME_LIMIT", tsqp::SQPStatus::kTimeLimit, "Reached time limit")
        .value("QP_SOLVE_FAILED", tsqp::SQPStatus::kQPSolveFailed, "QP solver failed")
        .value("STOPPED_BY_CALLBACK", tsqp::SQPStatus::kStoppedByCallback, "Stopped by callback");

    m.def("toString", nb::overload_cast<tsqp::SQPStatus>(&tsqp::toString), "status"_a,
          "trajopt's text for a solver status (e.g. 'SQP_CONVERGED')");

    nb::enum_<tsqp::QPSolverStatus>(m, "QPSolverStatus", "Status of QP solver")
        .value("UNINITIALIZED", tsqp::QPSolverStatus::kUninitialized, "Solver not initialized")
        .value("INITIALIZED", tsqp::QPSolverStatus::kInitialized, "Solver initialized")
        .value("FAILED", tsqp::QPSolverStatus::kFailed, "QP solver failed");

    // ========== SQPParameters ==========

    nb::class_<tsqp::SQPParameters> sqp_parameters(m, "SQPParameters", "Parameters controlling SQP optimization");
    bind_value_equality(sqp_parameters);
    sqp_parameters
        .def(nb::init<>())
        .def_rw("improve_ratio_threshold", &tsqp::SQPParameters::improve_ratio_threshold,
                "Minimum ratio exact_improve/approx_improve to accept step (default: 0.25)")
        .def_rw("min_trust_box_size", &tsqp::SQPParameters::min_trust_box_size,
                "NLP converges if trust region smaller than this (default: 1e-4)")
        .def_rw("min_approx_improve", &tsqp::SQPParameters::min_approx_improve,
                "NLP converges if approx_merit_improve smaller than this (default: 1e-4)")
        .def_rw("min_approx_improve_frac", &tsqp::SQPParameters::min_approx_improve_frac,
                "NLP converges if approx_merit_improve/best_exact_merit < this")
        .def_rw("max_iter", &tsqp::SQPParameters::max_iter,
                "Max convexifications per penalty iteration; rejected trust-region steps do not count (default: 50)")
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
        .def_rw("sum_constraint_set_violations", &tsqp::SQPParameters::sum_constraint_set_violations,
                "If true, cnt_tolerance is tested against a set's summed row violations; if false, against its "
                "largest row. Applied by TrustRegionSQPSolver.init() (default: true)")
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
        // The QP: minimise x'Hx + g'x s.t. lower <= A x <= upper. Each update checks its size
        // against init(num_vars, num_cnts) and raises QPSizeError otherwise.
        .def("updateHessianMatrix",
             [](tsqp::QPSolver& self, const trajopt_ifopt::Jacobian& hessian) {
                 const auto [n_vars, n_cnts] = qp_solver_size(self);
                 require_size("updateHessianMatrix", "hessian rows", hessian.rows(), "num_vars", n_vars);
                 require_size("updateHessianMatrix", "hessian columns", hessian.cols(), "num_vars", n_vars);
                 return self.updateHessianMatrix(hessian);
             },
             "hessian"_a, "Set the Hessian H (sparse, num_vars square) of the objective x'Hx + g'x")
        .def("updateGradient",
             [](tsqp::QPSolver& self, const Eigen::Ref<const Eigen::VectorXd>& gradient) {
                 require_size("updateGradient", "gradient", gradient.size(), "num_vars",
                              qp_solver_size(self).first);
                 return self.updateGradient(gradient);
             },
             "gradient"_a, "Set the gradient g (num_vars)")
        .def("updateLowerBound",
             [](tsqp::QPSolver& self, const Eigen::Ref<const Eigen::VectorXd>& lower_bound) {
                 require_size("updateLowerBound", "lower_bound", lower_bound.size(), "num_cnts",
                              qp_solver_size(self).second);
                 return self.updateLowerBound(lower_bound);
             },
             "lower_bound"_a, "Set the constraint lower bounds (num_cnts)")
        .def("updateUpperBound",
             [](tsqp::QPSolver& self, const Eigen::Ref<const Eigen::VectorXd>& upper_bound) {
                 require_size("updateUpperBound", "upper_bound", upper_bound.size(), "num_cnts",
                              qp_solver_size(self).second);
                 return self.updateUpperBound(upper_bound);
             },
             "upper_bound"_a, "Set the constraint upper bounds (num_cnts)")
        .def("updateBounds",
             [](tsqp::QPSolver& self, const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                const Eigen::Ref<const Eigen::VectorXd>& upper_bound) {
                 const Eigen::Index n_cnts = qp_solver_size(self).second;
                 require_size("updateBounds", "lower_bound", lower_bound.size(), "num_cnts", n_cnts);
                 require_size("updateBounds", "upper_bound", upper_bound.size(), "num_cnts", n_cnts);
                 return self.updateBounds(lower_bound, upper_bound);
             },
             "lower_bound"_a, "upper_bound"_a, "Set both constraint bounds (num_cnts each)")
        .def("updateLinearConstraintsMatrix",
             [](tsqp::QPSolver& self, const trajopt_ifopt::Jacobian& matrix) {
                 const auto [n_vars, n_cnts] = qp_solver_size(self);
                 require_size("updateLinearConstraintsMatrix", "matrix rows", matrix.rows(), "num_cnts", n_cnts);
                 require_size("updateLinearConstraintsMatrix", "matrix columns", matrix.cols(), "num_vars", n_vars);
                 return self.updateLinearConstraintsMatrix(matrix);
             },
             "linear_constraints_matrix"_a, "Set the constraint matrix A (sparse, num_cnts x num_vars)")
        .def("setWarmStart", &set_warm_start_checked, "qp_problem"_a,
             "Seed the next solve from the problem's variables (and its slacks); a no-op while "
             "the solver's warm-start option is off")
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
        // OSQP settings — forwarded to solver_->settings()
        // Defaults (set in C++ constructor): polish=true, warmStart=true,
        // adaptiveRho=true, maxIter=8192, absTol=1e-4, relTol=1e-6
        .def("setPolish", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setPolish(v); }, "polish"_a,
            "Enable solution polishing (default: true)")
        .def("setWarmStart", [](tsqp::OSQPEigenSolver& self, bool v) {
            self.solver_->settings()->setWarmStart(v); }, "warm_start"_a,
            "Enable warm-starting (default: true)")
        // The bool forwarder above hides QPSolver.setWarmStart(problem); bind it here too
        .def("setWarmStart", &set_warm_start_checked, "qp_problem"_a,
             "Seed the next solve from the problem's variables (and its slacks); a no-op while "
             "warm-starting is off")
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
        .def("setVariables",
             [](tsqp::QPProblem& self, const Eigen::Ref<const Eigen::VectorXd>& x) {
                 if (x.size() != self.getNumNLPVars())
                     throw VariablesSizeError("setVariables: x has " + std::to_string(x.size()) +
                                              " entries; the problem has getNumNLPVars() = " +
                                              std::to_string(self.getNumNLPVars()) + " variables");
                 self.setVariables(x.data());
             },
             "x"_a,
             "Set the NLP variables and update every component (a TrajOptQPProblem also recounts "
             "its dynamic rows). Raises VariablesSizeError unless len(x) == getNumNLPVars().")
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
             "    QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.\n"
             "    NotConvexifiedError: before the first convexify().")
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
             "    QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.\n"
             "    NotConvexifiedError: before the first convexify().")
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
             "    QPSolutionSizeError: if var_vals does not have getNumQPVars() entries.\n"
             "    NotConvexifiedError: before the first convexify().")
        .def("getExactConstraintViolations", &tsqp::QPProblem::getExactConstraintViolations,
             "Get current exact constraint violations")
        .def("setSumConstraintSetViolations", &tsqp::QPProblem::setSumConstraintSetViolations, "sum"_a,
             "Reduce a merit unit's row violations to their sum (True) or to the largest (False) in "
             "ConstraintViolations.raw. TrustRegionSQPSolver.init() overwrites this with "
             "SQPParameters.sum_constraint_set_violations.")
        .def("scaleBoxSize", &tsqp::QPProblem::scaleBoxSize, "scale"_a,
             "Uniformly scale the trust region box size")
        .def("setBoxSize", &tsqp::QPProblem::setBoxSize, "box_size"_a,
             "Set trust region box size")
        .def("setConstraintMeritCoeff", &tsqp::QPProblem::setConstraintMeritCoeff,
             "merit_coeff"_a, "Set constraint merit coefficients")
        .def("getBoxSize", nb::overload_cast<>(&tsqp::QPProblem::getBoxSize, nb::const_),
             "Get trust region box size")
        .def("getConstraintMeritCoeff", &tsqp::QPProblem::getConstraintMeritCoeff,
             "Get the constraint merit coefficients, one per constraint set")
        // The convex model the last convexify() built: x'Hx + g'x (no 1/2, as evaluateTotalConvexCost
        // and OSQPEigenSolver read H) over getNumQPVars() variables, s.t. lower <= A x <= upper over
        // getNumQPConstraints() rows
        .def("getHessian", [](const tsqp::QPProblem& self) { return compressed(self.getHessian()); },
             "The QP Hessian (sparse, getNumQPVars() square), from the last convexify()")
        .def("getGradient", &tsqp::QPProblem::getGradient,
             "The QP gradient (getNumQPVars()), from the last convexify()")
        .def("getConstraintMatrix",
             [](const tsqp::QPProblem& self) { return compressed(self.getConstraintMatrix()); },
             "The QP constraint matrix (sparse, getNumQPConstraints() x getNumQPVars()), from the "
             "last convexify()")
        .def("getBoundsLower", &tsqp::QPProblem::getBoundsLower,
             "The QP constraint lower bounds (getNumQPConstraints()), from the last convexify()")
        .def("getBoundsUpper", &tsqp::QPProblem::getBoundsUpper,
             "The QP constraint upper bounds (getNumQPConstraints()), from the last convexify()")
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
    // weights each cost row by its coefficient, as its convex model does, and counts
    // every cost term once; it keeps one merit coefficient per constraint set.
    nb::class_<tsqp::TrajOptQPProblem, tsqp::QPProblem>(
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
    nb::class_<QPProblemView, tsqp::QPProblem>(
        m, "QPProblemView",
        "The problem an SQPCallback receives from a solve whose problem has no Python object (one a\n"
        "planner built in C++). Valid only during that call; afterwards every method raises\n"
        "QPProblemExpiredError.");

    nb::class_<tsqp::SQPCallback, PySQPCallback>(
        m, "SQPCallback", "Base class for SQP optimization callbacks")
        .def(nb::init<>())
        .def("execute", &tsqp::SQPCallback::execute, "problem"_a, "sqp_results"_a,
             "Called during SQP. Return false to stop optimization. problem is valid only during "
             "the call: a problem passed to solve() from Python arrives as that object; one a "
             "planner built in C++ arrives as a QPProblemView, which raises QPProblemExpiredError "
             "once the call has returned. sqp_results is a copy the callback may keep.");

    // ========== PIQP ==========
    nb::enum_<piqp::KKTSolver>(m, "PIQPKKTSolver", "PIQP's KKT system solver")
        .value("dense_cholesky", piqp::KKTSolver::dense_cholesky)
        .value("sparse_ldlt", piqp::KKTSolver::sparse_ldlt)
        .value("sparse_ldlt_eq_cond", piqp::KKTSolver::sparse_ldlt_eq_cond)
        .value("sparse_ldlt_ineq_cond", piqp::KKTSolver::sparse_ldlt_ineq_cond)
        .value("sparse_ldlt_cond", piqp::KKTSolver::sparse_ldlt_cond)
        .value("sparse_multistage", piqp::KKTSolver::sparse_multistage);

    nb::class_<piqp::Settings<double>>(m, "PIQPSettings", "PIQP solver settings (piqp::Settings<double>)")
        .def(nb::init<>(), "PIQP's own defaults")
        .def_rw("rho_init", &piqp::Settings<double>::rho_init, "Initial primal proximal penalty")
        .def_rw("delta_init", &piqp::Settings<double>::delta_init, "Initial augmented Lagrangian penalty")
        .def_rw("eps_abs", &piqp::Settings<double>::eps_abs, "Absolute tolerance")
        .def_rw("eps_rel", &piqp::Settings<double>::eps_rel, "Relative tolerance")
        .def_rw("check_duality_gap", &piqp::Settings<double>::check_duality_gap, "Also stop on the duality gap")
        .def_rw("eps_duality_gap_abs", &piqp::Settings<double>::eps_duality_gap_abs, "Absolute duality gap tolerance")
        .def_rw("eps_duality_gap_rel", &piqp::Settings<double>::eps_duality_gap_rel, "Relative duality gap tolerance")
        .def_rw("infeasibility_threshold", &piqp::Settings<double>::infeasibility_threshold, "Threshold for the infeasibility detection")
        .def_rw("reg_lower_limit", &piqp::Settings<double>::reg_lower_limit, "Lower limit of the regularization")
        .def_rw("reg_finetune_lower_limit", &piqp::Settings<double>::reg_finetune_lower_limit, "Fine-tune lower limit of the regularization")
        .def_rw("reg_finetune_primal_update_threshold", &piqp::Settings<double>::reg_finetune_primal_update_threshold, "Primal updates before fine-tuning the regularization")
        .def_rw("reg_finetune_dual_update_threshold", &piqp::Settings<double>::reg_finetune_dual_update_threshold, "Dual updates before fine-tuning the regularization")
        .def_rw("max_iter", &piqp::Settings<double>::max_iter, "Maximum number of iterations")
        .def_rw("max_factor_retires", &piqp::Settings<double>::max_factor_retires, "Maximum number of factorization retries")
        .def_rw("preconditioner_scale_cost", &piqp::Settings<double>::preconditioner_scale_cost, "Scale the cost in the preconditioner")
        .def_rw("preconditioner_reuse_on_update", &piqp::Settings<double>::preconditioner_reuse_on_update, "Reuse the preconditioner when the problem is updated")
        .def_rw("preconditioner_iter", &piqp::Settings<double>::preconditioner_iter, "Preconditioner iterations")
        .def_rw("tau", &piqp::Settings<double>::tau, "Maximum step size fraction to the boundary")
        .def_rw("kkt_solver", &piqp::Settings<double>::kkt_solver, "KKT system solver")
        .def_rw("iterative_refinement_always_enabled", &piqp::Settings<double>::iterative_refinement_always_enabled, "Always run iterative refinement")
        .def_rw("iterative_refinement_eps_abs", &piqp::Settings<double>::iterative_refinement_eps_abs, "Iterative refinement absolute tolerance")
        .def_rw("iterative_refinement_eps_rel", &piqp::Settings<double>::iterative_refinement_eps_rel, "Iterative refinement relative tolerance")
        .def_rw("iterative_refinement_max_iter", &piqp::Settings<double>::iterative_refinement_max_iter, "Iterative refinement maximum iterations")
        .def_rw("iterative_refinement_min_improvement_rate", &piqp::Settings<double>::iterative_refinement_min_improvement_rate, "Iterative refinement minimum improvement rate")
        .def_rw("iterative_refinement_static_regularization_eps", &piqp::Settings<double>::iterative_refinement_static_regularization_eps, "Iterative refinement static regularization")
        .def_rw("iterative_refinement_static_regularization_rel", &piqp::Settings<double>::iterative_refinement_static_regularization_rel, "Iterative refinement relative static regularization")
        .def_rw("verbose", &piqp::Settings<double>::verbose, "Print solver progress")
        .def_rw("compute_timings", &piqp::Settings<double>::compute_timings, "Measure the solver timings");

    nb::class_<tsqp::PIQPSolver, tsqp::QPSolver>(m, "PIQPSolver", "PIQP-based (interior point) QP solver")
        .def(nb::init<>(), "With trajopt's default PIQP settings")
        .def_rw("settings", &tsqp::PIQPSolver::settings, "PIQP settings, applied when the solver is next set up")
        .def_static("defaultSettings", [] {
            piqp::Settings<double> settings;
            tsqp::PIQPSolver::setDefaultPIQPSettings(settings);
            return settings;
        }, "The PIQP settings trajopt uses by default")
        .def("reusedSetup", &tsqp::PIQPSolver::reusedSetup,
             "Whether the last solve reused the previous setup (only the vectors changed)");

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
