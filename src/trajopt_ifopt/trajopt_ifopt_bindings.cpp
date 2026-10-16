/**
 * @file trajopt_ifopt_bindings.cpp
 * @brief nanobind bindings for trajopt_ifopt variables, constraints, and costs
 *
 * NOTE: Default arguments for Eigen types (Isometry3d, Matrix) cause std::bad_cast
 * at module load time. All Eigen defaults are avoided - use default constructor
 * and set members instead.
 */

#include "tesseract_nb.h"

// trajopt_ifopt core (replaces ifopt base classes)
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/component.h>
#include <trajopt_ifopt/core/constraint_set.h>

// trajopt_ifopt variable sets (new variable system)
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <tesseract/common/joint_state.h>

// trajopt_ifopt constraints
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>

// trajopt_common
#include <trajopt_common/collision_types.h>

// tesseract_collision for CollisionCheckConfig (used by TrajOptCollisionConfig)
#include <tesseract/collision/types.h>

// tesseract dependencies
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/environment/environment.h>

namespace ti = trajopt_ifopt;
namespace tc = trajopt_common;

// Trampoline for ConstraintSet (enable Python subclassing)
// Pure virtuals from Component: getValues(), getBounds()
// Pure virtuals from Differentiable: update(), getCoefficients(), getJacobian()
class PyConstraintSet : public ti::ConstraintSet {
public:
    NB_TRAMPOLINE(ti::ConstraintSet, 5);

    // getNonZeros() is a Jacobian reservation hint; trajopt's default -1 means unset, and a
    // Python set cannot set the protected non_zeros_. TrajOptQPProblem reserves the sum of its
    // cost hints, so a Python cost set on its own asked for SIZE_MAX. 0 is a valid hint:
    // storage grows as needed.
    PyConstraintSet(std::string name, int n_constraints)
        : ti::ConstraintSet(std::move(name), n_constraints) {
        non_zeros_ = 0;
    }

    PyConstraintSet(std::string name, bool dynamic)
        : ti::ConstraintSet(std::move(name), dynamic) {
        non_zeros_ = 0;
    }

    // Expose protected getVariables() for Python access
    using ti::ConstraintSet::getVariables;

    Eigen::VectorXd getValues() const override {
        NB_OVERRIDE_PURE(getValues);
    }

    std::vector<ti::Bounds> getBounds() const override {
        NB_OVERRIDE_PURE(getBounds);
    }

    int update() override {
        NB_OVERRIDE_PURE(update);
    }

    Eigen::VectorXd getCoefficients() const override {
        NB_OVERRIDE_PURE(getCoefficients);
    }

    ti::Jacobian getJacobian() const override {
        NB_OVERRIDE_PURE(getJacobian);
    }
};

// Every built-in constraint sizes its Jacobian from the linked variables
// (`Jacobian jac(rows_, variables_->getRows())`, trajopt 0.35.0) without a null check, so
// getJacobian() before linkWithVariables() dereferenced a null pointer (#202). No other bound
// member reads variables_.
struct ConstraintNotLinkedError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// CartLineConstraint sizes its Jacobian by indices.size() rows but fills six whatever that size,
// in the numeric and the analytic branch (cartesian_line_constraint.cpp:179-188, :238-247,
// trajopt 0.35.0), so fewer than six indices write past the matrix (#237, trajopt#618).
// Remove with the check when the pinned trajopt has the fix (#238).
struct CartLineIndicesUnsupportedError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// toTrajArray sizes its matrix by the first Var and assigns every Var as a row unchecked
struct VarSizeMismatchError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A constraint writes its Jacobian columns at its Vars' indices into a matrix as wide as the linked
// variable set, unchecked: a Var outside that set (standalone, or from another set) puts columns
// out of range. Linking rejects it (VarCheckedConstraint below). The collision constraints also
// key their contact cache on the set's hash and read it through
// position_var->getParent()->getParent() unchecked (discrete_collision_constraint.cpp:69,
// trajopt 0.35.0), so they reject a Var outside any NodesVariables already when built.
struct VarNotInVariableSetError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// NodesVariables(name, nodes) takes each Node over from its Python object
struct NodeOwnershipError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// calcForwardNumJac sizes the Jacobian by len(f(x)) and subtracts every later f(x + h e_i) from
// f(x) unchecked: an output of another length reads or writes out of range.
struct NumericalJacobianSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A built-in constraint, plus the Vars it was built on, checked when it is linked to a variable
// set: from linkWithVariables, and from TrajOptQPProblem.addConstraintSet / addCostSet, which link
// in C++. The constraints keep their Vars private, so the binding keeps its own reference.
// ConstraintSet::linkWithVariables stores the set, then calls the private virtual
// initVariableDependedQuantities, a no-op that no built-in constraint overrides (trajopt 0.35.0).
template <class Constraint>
class VarCheckedConstraint final : public Constraint {
public:
    template <class... Args>
    explicit VarCheckedConstraint(std::vector<std::shared_ptr<const ti::Var>> vars, Args&&... args)
        : Constraint(std::forward<Args>(args)...), vars_(std::move(vars)) {}

    const std::vector<std::shared_ptr<const ti::Var>>& vars() const { return vars_; }

private:
    void initVariableDependedQuantities(const ti::Variables::Ptr& variables) override {
        for (const auto& var : vars_) {
            if (var->getParent() != nullptr && var->getParent()->getParent() == variables.get())
                continue;
            this->variables_.reset();  // leave the constraint unlinked, not linked to the wrong set
            throw VarNotInVariableSetError(
                "linkWithVariables: constraint set '" + this->getName() + "' uses Var '" +
                var->getIdentifier() + "', which is not in variable set '" + variables->getName() +
                "'; take its Vars from the set it is linked to (or added to a problem with)");
        }
    }

    std::vector<std::shared_ptr<const ti::Var>> vars_;
};

using CartPosConstraintBinding = VarCheckedConstraint<ti::CartPosConstraint>;
using JointPosConstraintBinding = VarCheckedConstraint<ti::JointPosConstraint>;
using JointVelConstraintBinding = VarCheckedConstraint<ti::JointVelConstraint>;
using JointAccelConstraintBinding = VarCheckedConstraint<ti::JointAccelConstraint>;
using JointJerkConstraintBinding = VarCheckedConstraint<ti::JointJerkConstraint>;
using CartLineConstraintBinding = VarCheckedConstraint<ti::CartLineConstraint>;
using InverseKinematicsConstraintBinding = VarCheckedConstraint<ti::InverseKinematicsConstraint>;
using DiscreteCollisionConstraintBinding = VarCheckedConstraint<ti::DiscreteCollisionConstraint>;
using DiscreteCollisionConstraintDBinding = VarCheckedConstraint<ti::DiscreteCollisionConstraintD>;
using DiscreteCollisionNumericalConstraintBinding = VarCheckedConstraint<ti::DiscreteCollisionNumericalConstraint>;
using ContinuousCollisionConstraintBinding = VarCheckedConstraint<ti::ContinuousCollisionConstraint>;
using ContinuousCollisionConstraintDBinding = VarCheckedConstraint<ti::ContinuousCollisionConstraintD>;
using ContinuousCollisionNumericalConstraintBinding =
    VarCheckedConstraint<ti::ContinuousCollisionNumericalConstraint>;

// Var -> Node -> NodesVariables are raw C++ pointers, so the Python objects carry the lifetimes:
// a Var read from a Node keeps that Node's Python object alive (keep_alive<0,1>), a Node read from
// a set keeps the set's, and a Node handed over to a set keeps the set alive (NodesVariables
// __init__). With that chain, the Node a Var's getParent() reaches is alive.
//
// Handing a Node to NodesVariables relinquishes its Python object, as nanobind's unique_ptr caster
// does, but nanobind keeps that object in its instance map until the caller drops it, and returns
// it for the same pointer: unusable. So the set's getters return the Node's registered object only
// when it is usable, and otherwise a fresh reference to the Node that keeps owner alive.
nb::object node_object(const ti::Node* node, nb::handle owner) {
    if (node == nullptr)
        return nb::none();
    nb::object found = nb::find(node);
    if (found.is_valid() && nb::inst_ready(found))
        return found;
    return nb::inst_reference(nb::type<ti::Node>(), const_cast<ti::Node*>(node), owner);
}

// NodesVariables takes its Nodes by unique_ptr. All are checked before any is taken over, so a
// rejected call leaves every Node with its caller.
// nb_type_relinquish_ownership / nb_type_restore_ownership are nanobind::detail API
// (nanobind/nb_lib.h, nanobind 2.12.0 as pinned in pixi.lock), the calls nanobind's own unique_ptr
// caster makes. tests/trajopt_ifopt/test_variable_set_lifetimes.py pins their behaviour
// (test_a_kept_node_is_unusable_after_handover, test_handing_the_same_node_over_twice_raises,
// test_result_keeps_its_owner_alive[set_getters_while_caller_keeps_node]); rerun it on a bump.
std::vector<std::unique_ptr<ti::Node>> take_over_nodes(const std::vector<nb::handle>& nodes) {
    std::set<PyObject*> seen;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const nb::handle h = nodes[i];
        if (!nb::isinstance<ti::Node>(h))
            throw nb::type_error(("NodesVariables: nodes[" + std::to_string(i) + "] is not a Node").c_str());
        const auto [ready, owned] = nb::inst_state(h);
        if (!ready)
            throw NodeOwnershipError("NodesVariables: nodes[" + std::to_string(i) +
                                     "] was handed to a variable set already");
        const std::string name = nb::inst_ptr<ti::Node>(h)->getName();
        if (!owned)
            throw NodeOwnershipError("NodesVariables: nodes[" + std::to_string(i) + "] '" + name +
                                     "' belongs to a variable set; build a new Node");
        if (!seen.insert(h.ptr()).second)
            throw NodeOwnershipError("NodesVariables: Node '" + name + "' appears twice in nodes");
    }
    std::vector<std::unique_ptr<ti::Node>> owned_nodes;
    owned_nodes.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (!nb::detail::nb_type_relinquish_ownership(nodes[i].ptr(), true)) {
            for (std::size_t j = 0; j < i; ++j)
                nb::detail::nb_type_restore_ownership(nodes[j].ptr(), true);
            for (auto& node : owned_nodes)
                node.release();  // back with their Python objects
            throw NodeOwnershipError("NodesVariables: nodes[" + std::to_string(i) +
                                     "] cannot be taken over from Python");
        }
        owned_nodes.emplace_back(nb::inst_ptr<ti::Node>(nodes[i]));
    }
    return owned_nodes;
}

const std::shared_ptr<const ti::Var>& require_in_variable_set(const std::shared_ptr<const ti::Var>& var,
                                                              const char* caller, const char* arg) {
    if (var->getParent() == nullptr || var->getParent()->getParent() == nullptr)
        throw VarNotInVariableSetError(
            std::string(caller) + ": " + arg + " '" + var->getIdentifier() +
            "' is not in a variable set; the collision constraints read the set's hash. Take the "
            "Var from a NodesVariables (createNodesVariables, or Nodes handed to NodesVariables)");
    return var;
}

// forward-difference step of calcNumericalConstraintGradient, its C++ default (ifopt_utils.h:108)
constexpr double NUMERICAL_GRADIENT_EPSILON = 1e-8;

// rows of a twist error: 3 linear + 3 angular
constexpr std::size_t TWIST_SIZE = 6;

// trajopt 0.35.0 checks a bounds vector's length against the rows it describes with assert()
// (compiled out) or a log line, then indexes by row: a wrong length reads or writes out of range.
struct BoundsSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// getClosestValidPoint / calcBoundsErrors / calcBoundsViolations index bounds by input entry
void require_one_bound_per_value(const Eigen::Ref<const Eigen::VectorXd>& input,
                                 const std::vector<ti::Bounds>& bounds, const char* function) {
    if (bounds.size() != static_cast<std::size_t>(input.size()))
        throw BoundsSizeError(std::string(function) + ": input has " + std::to_string(input.size()) +
                              " entries; got " + std::to_string(bounds.size()) + " bounds");
}

template <class Constraint>
void set_bounds_checked(Constraint& self, const std::vector<ti::Bounds>& bounds) {
    const auto rows = static_cast<std::size_t>(self.getRows());
    if (bounds.size() != rows)
        throw BoundsSizeError("setBounds: constraint set '" + self.getName() + "' has " +
                              std::to_string(rows) + " row(s); got " + std::to_string(bounds.size()) +
                              " bounds");
    self.setBounds(bounds);
}

ti::Variables::Ptr require_linked(const ti::ConstraintSet& self, const char* method = "getJacobian") {
    // getVariables() is protected; the trampoline's using-declaration names it,
    // and the member pointer is ConstraintSet's, so this is valid for any set.
    constexpr auto get_variables = &PyConstraintSet::getVariables;
    ti::Variables::Ptr variables = (self.*get_variables)();
    if (!variables)
        throw ConstraintNotLinkedError(std::string(method) + ": constraint set '" + self.getName() +
                                       "' is not linked to variables; call "
                                       "linkWithVariables first, or add it to a problem");
    return variables;
}

void require_six_indices(const ti::CartLineConstraint& self, const char* method) {
    const auto n_indices = static_cast<std::size_t>(self.getInfo().indices.size());
    if (n_indices != TWIST_SIZE)
        throw CartLineIndicesUnsupportedError(
            std::string(method) + ": constraint set '" + self.getName() + "' has " +
            std::to_string(n_indices) + " indices; trajopt 0.35.0 writes " +
            std::to_string(TWIST_SIZE) + " Jacobian rows whatever their number and "
            "corrupts memory with fewer (tesseract_nanobind#237)");
}

// calcJacobianBlock(jac_block, joint_vals) fills a caller-sized Jacobian: getJacobian() sizes it
// (rows, linked variables' rows) and reserves getNonZeros() first (trajopt 0.35.0, e.g.
// cartesian_position_constraint.cpp:378-382). It reads n_dof entries of joint_vals unchecked.
struct JointValuesSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

template <class Constraint>
ti::Jacobian jacobian_block_checked(const Constraint& self,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                    Eigen::Index n_dof) {
    const ti::Variables::Ptr variables = require_linked(self, "calcJacobianBlock");
    if (joint_vals.size() != n_dof)
        throw JointValuesSizeError("calcJacobianBlock: constraint set '" + self.getName() + "' has " +
                                   std::to_string(n_dof) + " joints; joint_vals has " +
                                   std::to_string(joint_vals.size()) + " entries");
    ti::Jacobian jacobian(self.getRows(), variables->getRows());
    jacobian.reserve(self.getNonZeros());
    self.calcJacobianBlock(jacobian, joint_vals);
    jacobian.makeCompressed();
    return jacobian;
}

ti::Jacobian jacobian_checked(const ti::Differentiable& self) {
    // Bound on Differentiable too (#287); every Differentiable in trajopt 0.35.0 is a ConstraintSet
    if (const auto* constraint_set = dynamic_cast<const ti::ConstraintSet*>(&self))
        require_linked(*constraint_set);
    // The collision constraints (analytic and numerical, discrete and continuous)
    // assemble their Jacobian with coeffRef while a contact is active, which
    // leaves it uncompressed; nanobind's Eigen caster only returns compressed
    // sparse matrices.
    ti::Jacobian jacobian = self.getJacobian();
    jacobian.makeCompressed();
    return jacobian;
}

NB_MODULE(_trajopt_ifopt, m) {
    m.doc() = "trajopt_ifopt Python bindings - variables, constraints, and costs for IFOPT-based trajectory optimization";

    // JointTrajectory (toJointTrajectory)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    // Import tesseract_collision for CollisionCheckConfig type
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");
    // JointGroup / KinematicGroup and Environment: constructor arguments and Info fields
    nb::module_::import_("tesseract_robotics.tesseract_kinematics._tesseract_kinematics");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");

    nb::exception<ConstraintNotLinkedError>(m, "ConstraintNotLinkedError", PyExc_RuntimeError)
        .attr("__doc__") = "getJacobian() on a constraint set that is not linked to variables yet.";
    nb::exception<CartLineIndicesUnsupportedError>(m, "CartLineIndicesUnsupportedError", PyExc_ValueError)
        .attr("__doc__") = "CartLineConstraint.getJacobian() with fewer than 6 indices (#237).";
    nb::exception<BoundsSizeError>(m, "BoundsSizeError", PyExc_ValueError)
        .attr("__doc__") = "A bounds list whose length is not the number of rows or values it bounds.";
    nb::exception<JointValuesSizeError>(m, "JointValuesSizeError", PyExc_ValueError)
        .attr("__doc__") = "joint_vals whose length is not the constraint's number of joints.";
    nb::exception<VarSizeMismatchError>(m, "VarSizeMismatchError", PyExc_ValueError)
        .attr("__doc__") = "toTrajArray over Vars of different sizes.";
    nb::exception<VarNotInVariableSetError>(m, "VarNotInVariableSetError", PyExc_ValueError)
        .attr("__doc__") = "A constraint linked to a variable set that does not hold its Vars, or a collision "
                           "constraint built on a Var that is not in a NodesVariables.";
    nb::exception<NodeOwnershipError>(m, "NodeOwnershipError", PyExc_ValueError)
        .attr("__doc__") = "A Node handed to NodesVariables that is in a variable set already, or twice.";
    nb::exception<NumericalJacobianSizeError>(m, "NumericalJacobianSizeError", PyExc_ValueError)
        .attr("__doc__") = "calcForwardNumJac over a function whose output length changes between calls.";

    // ConstraintSet's trampoline is called from _trajopt_sqp (TrajOptQPProblem, the solver)
    register_python_error_translator();

    // ========== BoundsType enum ==========
    nb::enum_<ti::BoundsType>(m, "BoundsType")
        .value("UNBOUNDED", ti::BoundsType::kUnbounded)
        .value("EQUALITY", ti::BoundsType::kEquality)
        .value("LOWER_BOUND", ti::BoundsType::kLowerBound)
        .value("UPPER_BOUND", ti::BoundsType::kUpperBound)
        .value("RANGE_BOUND", ti::BoundsType::kRangeBound);

    // ========== RangeBoundHandling enum ==========
    nb::enum_<ti::RangeBoundHandling>(m, "RangeBoundHandling")
        .value("KEEP_AS_IS", ti::RangeBoundHandling::kKeepAsIs)
        .value("SPLIT_TO_TWO_INEQUALITIES", ti::RangeBoundHandling::kSplitToTwoInequalities);

    // ========== Bounds ==========
    nb::class_<ti::Bounds>(m, "Bounds")
        .def(nb::init<double, double>(), "lower"_a = 0.0, "upper"_a = 0.0,
             "Create bounds with lower and upper limits")
        .def("set", &ti::Bounds::set, "lower"_a, "upper"_a,
             "Set both lower and upper bounds")
        .def("setLower", &ti::Bounds::setLower, "lower"_a,
             "Set the lower bound")
        .def("getLower", &ti::Bounds::getLower,
             "Get the lower bound value")
        .def("setUpper", &ti::Bounds::setUpper, "upper"_a,
             "Set the upper bound")
        .def("getUpper", &ti::Bounds::getUpper,
             "Get the upper bound value")
        .def("getType", &ti::Bounds::getType,
             "Get the cached bound classification")
        // C++ declares both operators returning void; nb::self += double() would hand Python
        // that void (None) and rebind the name, so these return self.
        .def("__iadd__", [](ti::Bounds& self, double scalar) -> ti::Bounds& { self += scalar; return self; },
             "scalar"_a, nb::rv_policy::reference,
             "Shift both limits up by scalar and reclassify the bounds type")
        .def("__isub__", [](ti::Bounds& self, double scalar) -> ti::Bounds& { self -= scalar; return self; },
             "scalar"_a, nb::rv_policy::reference,
             "Shift both limits down by scalar and reclassify the bounds type");

    // ========== Component (base class) ==========
    nb::class_<ti::Component>(m, "Component")
        .def("getName", &ti::Component::getName,
             "Get the component name")
        .def("getRows", &ti::Component::getRows,
             "Get the number of rows")
        .def("getValues", &ti::Component::getValues,
             "Get the current values (variables, or constraint / cost values)")
        .def("getBounds", &ti::Component::getBounds,
             "Get the bounds, one per row");

    // ========== Variables (inherits Component) ==========
    nb::class_<ti::Variables, ti::Component>(m, "Variables")
        .def("setVariables", &ti::Variables::setVariables, "x"_a,
             "Set the variable values from a flat decision vector")
        .def("getHash", &ti::Variables::getHash,
             "Get the variable hash for caching");

    // ========== Differentiable (inherits Component) ==========
    nb::class_<ti::Differentiable, ti::Component> differentiable(m, "Differentiable");

    nb::enum_<ti::Differentiable::Mode>(differentiable, "Mode")
        .value("STACK_ROWS", ti::Differentiable::Mode::kStackRows, "One row per value")
        .value("SUM_TO_SCALAR", ti::Differentiable::Mode::kSumToScalar, "All values summed into one row");

    differentiable
        .def("getJacobian", &jacobian_checked,
             "Get the Jacobian (sparse matrix); a constraint set must be linked to variables first")
        .def("isDynamic", &ti::Differentiable::isDynamic,
             "Whether this component can change its number of rows")
        .def("isScalar", &ti::Differentiable::isScalar,
             "Whether this component is scalar or stacked")
        .def("getNonZeros", &ti::Differentiable::getNonZeros,
             "Estimated number of non-zero entries in the Jacobian")
        .def("update", &ti::Differentiable::update,
             "Recompute sizing/state for dynamic-sized components")
        .def("getCoefficients", &ti::Differentiable::getCoefficients,
             "Get the coefficient vector");

    // ========== ConstraintSet (inherits Differentiable, with Python trampoline) ==========
    nb::class_<ti::ConstraintSet, ti::Differentiable, PyConstraintSet>(m, "ConstraintSet")
        .def(nb::init<std::string, int>(), "name"_a, "n_constraints"_a,
             "Create a constraint set with name and number of constraints")
        .def("linkWithVariables", &ti::ConstraintSet::linkWithVariables, "x"_a,
             "Connect the constraint with the optimization variables")
        .def("getVariables", &PyConstraintSet::getVariables,
             "Get the linked optimization variables (protected in C++, exposed via trampoline)")
        .def("getJacobian", &jacobian_checked,
             "Get the constraint Jacobian (sparse matrix)")
        .def("update", &ti::ConstraintSet::update,
             "Recompute sizing/state for dynamic-sized components")
        .def("getCoefficients", &ti::ConstraintSet::getCoefficients,
             "Get the coefficient vector");

    // ========== Var (variable segment) ==========
    // A standalone Var (no parent Node) has no variable set: a constraint built on it raises
    // VarNotInVariableSetError when linked, and a collision constraint already when built.
    nb::class_<ti::Var>(m, "Var")
        .def("__init__",
             [](ti::Var* self, Eigen::Index index, std::string name, double value, ti::Bounds bounds) {
                 new (self) ti::Var(index, std::move(name), value, bounds);
             },
             "index"_a, "name"_a, "value"_a, "bounds"_a = ti::NoBound,
             "Create a standalone scalar variable at index of the decision vector; a value outside "
             "bounds is clamped into them")
        .def("__init__",
             [](ti::Var* self, Eigen::Index index, std::string identifier, std::vector<std::string> names,
                const Eigen::VectorXd& values, const std::vector<ti::Bounds>& bounds) {
                 new (self) ti::Var(index, std::move(identifier), std::move(names), values, bounds);
             },
             "index"_a, "identifier"_a, "names"_a, "values"_a, "bounds"_a,
             "Create a standalone vector variable starting at index of the decision vector; values "
             "outside bounds are clamped into them")
        .def("getIdentifier", &ti::Var::getIdentifier,
             "Get the identifier for the variable segment")
        .def("getParent",
             [](nb::pointer_and_handle<ti::Var> self) { return node_object(self.p->getParent(), self.h); },
             nb::sig("def getParent(self) -> Node | None"),
             "The Node holding this variable, or None for a standalone Var")
        .def("getIndex", &ti::Var::getIndex,
             "Get the starting index of this variable segment")
        .def("size", &ti::Var::size,
             "Get the number of elements in this variable segment")
        .def("value", &ti::Var::value,
             "Get the current variable values")
        .def("name", &ti::Var::name,
             "Get the variable names")
        .def("bounds", &ti::Var::bounds,
             "Get the variable bounds")
        .def("setVariables", &ti::Var::setVariables, "x"_a,
             "Set the variable values from the full decision vector");

    // ========== Node (group of related variables) ==========
    // Built on the heap, not in the Python object: NodesVariables takes Nodes by unique_ptr, and
    // nanobind can hand only a C++-allocated instance over to a unique_ptr.
    nb::class_<ti::Node>(m, "Node")
        .def(nb::new_([](std::string node_name) { return std::make_unique<ti::Node>(std::move(node_name)); }),
             "node_name"_a = "Node",
             "Create a node with an optional name")
        .def("getName", &ti::Node::getName,
             "Get the name of this node")
        // No keep_alive: a Node in a set keeps the set alive already, and tying the set back to the
        // Node would be a cycle the garbage collector cannot see.
        .def("getParent", &ti::Node::getParent, nb::rv_policy::reference,
             nb::sig("def getParent(self) -> NodesVariables | None"),
             "The NodesVariables holding this node, or None before it is handed to one")
        .def("addVar",
             nb::overload_cast<const std::string&, double, ti::Bounds>(&ti::Node::addVar),
             "name"_a, "value"_a, "bounds"_a = ti::NoBound, nb::keep_alive<0, 1>(),
             "Add a scalar variable to this node")
        .def("addVar",
             nb::overload_cast<const std::string&, const std::vector<std::string>&,
                               const Eigen::VectorXd&, const std::vector<ti::Bounds>&>(&ti::Node::addVar),
             "name"_a, "child_names"_a, "values"_a, "bounds"_a, nb::keep_alive<0, 1>(),
             "Add a vector-valued variable to this node")
        .def("hasVar", &ti::Node::hasVar, "name"_a,
             "Check whether this node has a variable by name")
        .def("getVar", &ti::Node::getVar, "name"_a, nb::keep_alive<0, 1>(),
             "Get a variable by name")
        .def("getValues", &ti::Node::getValues,
             "Get all variable values as a single vector")
        .def("getBounds", &ti::Node::getBounds,
             "Get all variable bounds")
        .def("size", &ti::Node::size,
             "Get total number of scalar decision variables")
        .def("setVariables", &ti::Node::setVariables, "x"_a,
             "Update all variable values from the full decision vector");

    // ========== NodesVariables (main variable container, replaces JointPosition) ==========
    // The constructor takes each Node by unique_ptr: the Python Node objects hand over ownership
    // and become unusable (take_over_nodes). Moving the Nodes instead would leave each Var's parent
    // pointer on the moved-from Python object. Each handed-over object keeps the set alive, so a
    // Var read from the Node before the handover keeps its parent chain.
    nb::class_<ti::NodesVariables, ti::Variables>(m, "NodesVariables")
        .def("__init__",
             [](nb::pointer_and_handle<ti::NodesVariables> self, std::string variable_name,
                const std::vector<nb::handle>& nodes) {
                 new (self.p) ti::NodesVariables(std::move(variable_name), take_over_nodes(nodes));
                 for (const nb::handle node : nodes)
                     nb::detail::keep_alive(node.ptr(), self.h.ptr());
             },
             "variable_name"_a, "nodes"_a,
             nb::sig("def __init__(self, variable_name: str, nodes: collections.abc.Sequence[Node]) -> None"),
             "Create the variable set from its nodes, in order; takes ownership of the nodes, which "
             "become unusable through the caller's references (read them back with getNode / "
             "getNodes). Raises NodeOwnershipError for a Node given twice or already in a set")
        .def("getNode",
             [](nb::pointer_and_handle<ti::NodesVariables> self, std::size_t opt_idx) {
                 return node_object(self.p->getNode(opt_idx).get(), self.h);
             },
             "opt_idx"_a, nb::sig("def getNode(self, opt_idx: int) -> Node"),
             "Get node based on index")
        .def("getNodes",
             [](nb::pointer_and_handle<ti::NodesVariables> self) {
                 nb::list nodes;
                 for (const auto& node : self.p->getNodes())
                     nodes.append(node_object(node.get(), self.h));
                 return nodes;
             },
             nb::sig("def getNodes(self) -> list[Node]"),
             "Get all nodes")
        .def("getDim", &ti::NodesVariables::getDim,
             "Get the dimensions of every node");

    // ========== Factory for NodesVariables ==========
    // The one-call form of the NodesVariables constructor: one Node with one vector Var
    // ("joints") per waypoint, built on the C++ side.
    m.def("createNodesVariables",
          [](const std::string& variable_name,
             const std::vector<std::string>& joint_names,
             const std::vector<Eigen::VectorXd>& initial_values,
             const std::vector<ti::Bounds>& bounds) -> std::shared_ptr<ti::NodesVariables> {
              std::vector<std::unique_ptr<ti::Node>> nodes;
              nodes.reserve(initial_values.size());
              for (size_t i = 0; i < initial_values.size(); ++i) {
                  auto node = std::make_unique<ti::Node>(variable_name + "_" + std::to_string(i));
                  node->addVar("joints", joint_names, initial_values[i], bounds);
                  nodes.push_back(std::move(node));
              }
              return std::make_shared<ti::NodesVariables>(variable_name, std::move(nodes));
          },
          "variable_name"_a, "joint_names"_a, "initial_values"_a, "bounds"_a,
          "Create NodesVariables with one vector Var per node.\n\n"
          "Args:\n"
          "    variable_name: Name for the variable set\n"
          "    joint_names: Joint name for each DOF\n"
          "    initial_values: List of initial joint value vectors (one per waypoint)\n"
          "    bounds: Joint bounds (shared across all waypoints)");

    // ========== Utility functions ==========
    m.def("interpolate", &ti::interpolate, "start"_a, "end"_a, "steps"_a,
          "Interpolate between two joint vectors, returning 'steps' waypoints");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::MatrixX2d>&>(&ti::toBounds),
          "limits"_a, "Convert Nx2 matrix [lower, upper] to vector of Bounds");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                         const Eigen::Ref<const Eigen::VectorXd>&>(&ti::toBounds),
          "lower_limits"_a, "upper_limits"_a, "Convert lower/upper limit vectors to vector of Bounds");

    m.def("getClosestValidPoint",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "getClosestValidPoint");
              return ti::getClosestValidPoint(input, bounds);
          },
          "input"_a, "bounds"_a, "input clamped into bounds, entry by entry");

    m.def("calcBoundsErrors",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "calcBoundsErrors");
              Eigen::VectorXd out(input.size());
              ti::calcBoundsErrors(out, input, bounds);
              return out;
          },
          "input"_a, "bounds"_a,
          "Signed bound error per entry: input - lower below the lower bound (negative), "
          "input - upper above the upper bound (positive), else 0");

    m.def("calcBoundsViolations",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "calcBoundsViolations");
              Eigen::VectorXd out(input.size());
              ti::calcBoundsViolations(out, input, bounds);
              return out;
          },
          "input"_a, "bounds"_a, "Bound violation per entry: the magnitude of calcBoundsErrors");

    // Numerical Jacobians, to check an analytic getJacobian (a Python ConstraintSet's, say). Both
    // call back into Python (constraint_set's methods, f), so neither releases the GIL.
    m.def("calcNumericalConstraintGradient",
          [](ti::Variables& variables, ti::ConstraintSet& constraint_set, double epsilon) {
              ti::Jacobian jacobian = ti::calcNumericalConstraintGradient(variables, constraint_set, epsilon);
              jacobian.makeCompressed();  // assembled with coeffRef
              return jacobian;
          },
          "variables"_a, "constraint_set"_a, "epsilon"_a = NUMERICAL_GRADIENT_EPSILON,
          "Forward-difference Jacobian of constraint_set's values over variables (sparse matrix). "
          "Perturbs each variable in turn through variables.setVariables, calling update() and "
          "getValues(), and restores the variables afterwards.");
    m.def("calcForwardNumJac",
          [](const ti::ErrorCalculator& f, const Eigen::Ref<const Eigen::VectorXd>& x, double epsilon) {
              std::optional<Eigen::Index> rows;  // len(f(x)), from the first call
              const ti::ErrorCalculator checked = [&f, &rows](const Eigen::Ref<const Eigen::VectorXd>& at) {
                  Eigen::VectorXd values = f(at);
                  if (!rows)
                      rows = values.size();
                  else if (values.size() != *rows)
                      throw NumericalJacobianSizeError(
                          "calcForwardNumJac: f returned " + std::to_string(*rows) +
                          " values at x and " + std::to_string(values.size()) + " at a perturbed x");
                  return values;
              };
              return ti::calcForwardNumJac(checked, x, epsilon);
          },
          "f"_a, "x"_a, "epsilon"_a,
          "Forward-difference Jacobian of f at x (sparse matrix, len(f(x)) x len(x)); raises "
          "NumericalJacobianSizeError if f's output length changes");

    // ========== Trajectory conversion (trajopt_utils.h) ==========
    m.def("toTrajArray",
          [](const std::vector<std::shared_ptr<const ti::Var>>& joint_positions) {
              require_non_null(joint_positions, "toTrajArray", "joint_positions", "a Var");
              for (std::size_t i = 1; i < joint_positions.size(); ++i)
                  if (joint_positions[i]->size() != joint_positions.front()->size())
                      throw VarSizeMismatchError(
                          "toTrajArray: joint_positions[" + std::to_string(i) + "] has " +
                          std::to_string(joint_positions[i]->size()) + " values; joint_positions[0] has " +
                          std::to_string(joint_positions.front()->size()));
              return ti::toTrajArray(joint_positions);
          },
          "joint_positions"_a,
          "The Vars' values as a waypoints x joints matrix, one row per Var");
    m.def("toJointTrajectory",
          [](const std::vector<std::shared_ptr<const ti::Var>>& joint_positions) {
              return ti::toJointTrajectory(
                  require_non_null(joint_positions, "toJointTrajectory", "joint_positions", "a Var"));
          },
          "joint_positions"_a,
          "A JointTrajectory with one JointState (the Var's names and values) per Var");

    // ========== CartPosConstraint ==========
    // Constructor takes individual parameters (CartPosInfo struct removed)
    //
    // range_bound_handling defaults to kSplitToTwoInequalities, matching the C++ ctor
    // default: every row whose bounds are a true range [lb, ub] is emitted as two
    // one-sided rows, g(x) >= lb and g(x) <= ub, so the row count grows accordingly.
    // kKeepAsIs leaves such a row as a single [lb, ub] row instead - the form to pass
    // when reading back an asymmetric band. Equality and one-sided bounds are unaffected
    // either way.
    nb::class_<CartPosConstraintBinding, ti::ConstraintSet>(m, "CartPosConstraint")
        .def("__init__", [](CartPosConstraintBinding* self,
                            std::shared_ptr<const ti::Var> position_var,
                            std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                            std::string source_frame,
                            std::string target_frame,
                            const Eigen::Isometry3d& source_frame_offset,
                            const Eigen::Isometry3d& target_frame_offset,
                            const std::string& name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) CartPosConstraintBinding({position_var}, position_var, manip,
                     std::move(source_frame), std::move(target_frame),
                     source_frame_offset, target_frame_offset,
                     name, range_bound_handling);
             },
             "position_var"_a, "manip"_a,
             "source_frame"_a, "target_frame"_a,
             "source_frame_offset"_a, "target_frame_offset"_a,
             "name"_a = "CartPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create Cartesian position constraint")
        .def("__init__", [](CartPosConstraintBinding* self,
                            std::shared_ptr<const ti::Var> position_var,
                            const Eigen::VectorXd& coeffs,
                            const std::vector<ti::Bounds>& bounds,
                            std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                            std::string source_frame,
                            std::string target_frame,
                            const Eigen::Isometry3d& source_frame_offset,
                            const Eigen::Isometry3d& target_frame_offset,
                            const std::string& name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) CartPosConstraintBinding({position_var}, position_var, coeffs, bounds, manip,
                     std::move(source_frame), std::move(target_frame),
                     source_frame_offset, target_frame_offset,
                     name, range_bound_handling);
             },
             "position_var"_a, "coeffs"_a, "bounds"_a, "manip"_a,
             "source_frame"_a, "target_frame"_a,
             "source_frame_offset"_a, "target_frame_offset"_a,
             "name"_a = "CartPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create Cartesian position constraint with per-axis coefficients and "
             "bounds. coeffs and bounds are both length 6, ordered "
             "[x, y, z, rx, ry, rz]; a zero coefficient frees that axis.")
        .def("calcValues", &ti::CartPosConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("calcJacobianBlock",
             [](const CartPosConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 // CartPosConstraint does not expose its manipulator; its Var holds one value per joint
                 return jacobian_block_checked(self, joint_vals, self.vars().front()->size());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables")
        .def("getValues", &ti::CartPosConstraint::getValues,
             "Get current constraint values")
        .def("setTargetPose", &ti::CartPosConstraint::setTargetPose, "target_frame_offset"_a,
             "Set the target pose - critical for online planning!")
        .def("getTargetPose", &ti::CartPosConstraint::getTargetPose,
             "Get the target pose for the constraint")
        .def("getCurrentPose", &ti::CartPosConstraint::getCurrentPose,
             "Get current TCP pose in world frame")
        .def_rw("use_numeric_differentiation", &ti::CartPosConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== JointPosConstraint ==========
    nb::class_<JointPosConstraintBinding, ti::ConstraintSet>(m, "JointPosConstraint")
        .def("__init__",
             [](JointPosConstraintBinding* self, const Eigen::VectorXd& target,
                const std::shared_ptr<const ti::Var>& position_var, const Eigen::VectorXd& coeffs,
                std::string name, ti::RangeBoundHandling range_bound_handling) {
                 new (self) JointPosConstraintBinding({position_var}, target, position_var, coeffs,
                                                      std::move(name), range_bound_handling);
             },
             "target"_a, "position_var"_a, "coeffs"_a, "name"_a = "JointPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create joint position constraint with target values")
        .def("__init__", [](JointPosConstraintBinding* self,
                            const std::vector<ti::Bounds>& bounds,
                            const std::shared_ptr<const ti::Var>& position_var,
                            const Eigen::VectorXd& coeffs,
                            std::string name,
                            ti::RangeBoundHandling range_bound_handling) {
                 // In trajopt 0.35.0 the range split indexes the caller's coeffs argument
                 // instead of the broadcast member coeffs_, so a length-0/1 coeffs is read
                 // past its end (fixed by tesseract-robotics/trajopt#592, merged 2026-09-30,
                 // in no release yet); the binding broadcasts first. Remove the broadcast only
                 // once a release containing #592 is pinned.
                 const auto n_dof = static_cast<Eigen::Index>(bounds.size());
                 Eigen::VectorXd per_joint = coeffs;
                 if (coeffs.size() == 0)
                     per_joint = Eigen::VectorXd::Ones(n_dof);
                 else if (coeffs.size() == 1)
                     per_joint = Eigen::VectorXd::Constant(n_dof, coeffs(0));
                 new (self) JointPosConstraintBinding({position_var}, bounds, position_var, per_joint,
                                                      std::move(name), range_bound_handling);
             },
             "bounds"_a, "position_var"_a, "coeffs"_a, "name"_a = "JointPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create joint position constraint with one bound per joint: an equality, a "
             "one-sided limit or a range. coeffs has length n_dof, 1 (one weight for every "
             "joint) or 0 (weight 1 for every joint). A range becomes two one-sided rows "
             "unless range_bound_handling is KEEP_AS_IS.")
        .def("getValues", &ti::JointPosConstraint::getValues,
             "Get current constraint values");

    // ========== JointVelConstraint ==========
    nb::class_<JointVelConstraintBinding, ti::ConstraintSet>(m, "JointVelConstraint")
        .def("__init__",
             [](JointVelConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointVelConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointVelConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointVel",
             "Create joint velocity constraint with target values")
        .def("getValues", &ti::JointVelConstraint::getValues,
             "Get current constraint values");

    // ========== JointAccelConstraint ==========
    nb::class_<JointAccelConstraintBinding, ti::ConstraintSet>(m, "JointAccelConstraint")
        .def("__init__",
             [](JointAccelConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointAccelConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointAccelConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointAccel",
             "Create joint acceleration constraint with target values")
        .def("getValues", &ti::JointAccelConstraint::getValues,
             "Get current constraint values");

    // ========== trajopt_common::CollisionCoeffData ==========
    nb::class_<tc::CollisionCoeffData> collision_coeff_data(m, "CollisionCoeffData");
    bind_value_equality(collision_coeff_data);
    collision_coeff_data
        .def(nb::init<>())
        .def(nb::init<double>(), "default_collision_coeff"_a)
        .def("setDefaultCollisionCoeff", &tc::CollisionCoeffData::setDefaultCollisionCoeff)
        .def("getDefaultCollisionCoeff", &tc::CollisionCoeffData::getDefaultCollisionCoeff)
        .def("setCollisionCoeff", &tc::CollisionCoeffData::setCollisionCoeff,
             "obj1"_a, "obj2"_a, "collision_coeff"_a)
        .def("getCollisionCoeff", &tc::CollisionCoeffData::getCollisionCoeff,
             "obj1"_a, "obj2"_a)
        .def("getCollisionCoeffPairData", &tc::CollisionCoeffData::getCollisionCoeffPairData,
             "Every pair with a coefficient: {(link_a, link_b): coeff}")
        .def("getPairsWithZeroCoeff", &tc::CollisionCoeffData::getPairsWithZeroCoeff,
             "The pairs whose coefficient is 0 (collision checking switched off)");

    // ========== trajopt_common::TrajOptCollisionConfig ==========
    nb::class_<tc::TrajOptCollisionConfig> trajopt_collision_config(m, "TrajOptCollisionConfig");
    bind_value_equality(trajopt_collision_config);
    trajopt_collision_config
        .def(nb::init<>())
        .def(nb::init<double, double>(), "margin"_a, "coeff"_a)
        .def_rw("enabled", &tc::TrajOptCollisionConfig::enabled)
        .def_rw("contact_manager_config", &tc::TrajOptCollisionConfig::contact_manager_config)
        .def_rw("collision_check_config", &tc::TrajOptCollisionConfig::collision_check_config)
        .def_rw("collision_coeff_data", &tc::TrajOptCollisionConfig::collision_coeff_data)
        .def_rw("collision_margin_buffer", &tc::TrajOptCollisionConfig::collision_margin_buffer)
        .def_rw("max_num_cnt", &tc::TrajOptCollisionConfig::max_num_cnt);

    // ========== trajopt_common collision results (what the evaluators compute) ==========
    nb::class_<tc::LinkGradientResults>(m, "LinkGradientResults")
        .def(nb::init<>())
        .def_rw("has_gradient", &tc::LinkGradientResults::has_gradient)
        .def_rw("gradient", &tc::LinkGradientResults::gradient)
        .def_rw("translation_vector", &tc::LinkGradientResults::translation_vector)
        .def_rw("jacobian", &tc::LinkGradientResults::jacobian)
        .def_rw("scale", &tc::LinkGradientResults::scale)
        .def_rw("cc_type", &tc::LinkGradientResults::cc_type)
        .def("clear", &tc::LinkGradientResults::clear);

    nb::class_<tc::GradientResults>(m, "GradientResults")
        .def(nb::init<>())
        .def_rw("gradients", &tc::GradientResults::gradients)
        .def_rw("cc_gradients", &tc::GradientResults::cc_gradients)
        .def_rw("error", &tc::GradientResults::error)
        .def_rw("error_with_buffer", &tc::GradientResults::error_with_buffer)
        .def("clear", &tc::GradientResults::clear);

    nb::class_<tc::LinkMaxError>(m, "LinkMaxError")
        .def(nb::init<>())
        .def_rw("has_error", &tc::LinkMaxError::has_error)
        .def_rw("error", &tc::LinkMaxError::error)
        .def_rw("error_with_buffer", &tc::LinkMaxError::error_with_buffer)
        .def("getMaxError", &tc::LinkMaxError::getMaxError)
        .def("getMaxErrorWithBuffer", &tc::LinkMaxError::getMaxErrorWithBuffer);

    nb::class_<tc::GradientResultsSet>(m, "GradientResultsSet")
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), "reserve"_a)
        .def_rw("key", &tc::GradientResultsSet::key)
        .def_rw("shape_key", &tc::GradientResultsSet::shape_key)
        .def_rw("coeff", &tc::GradientResultsSet::coeff)
        .def_rw("is_continuous", &tc::GradientResultsSet::is_continuous)
        .def_rw("max_error", &tc::GradientResultsSet::max_error)
        .def_rw("results", &tc::GradientResultsSet::results)
        .def("add", &tc::GradientResultsSet::add, "gradient_result"_a)
        .def("getMaxError", &tc::GradientResultsSet::getMaxError)
        .def("getMaxErrorT0", &tc::GradientResultsSet::getMaxErrorT0)
        .def("getMaxErrorT1", &tc::GradientResultsSet::getMaxErrorT1)
        .def("getMaxErrorWithBuffer", &tc::GradientResultsSet::getMaxErrorWithBuffer)
        .def("getMaxErrorWithBufferT0", &tc::GradientResultsSet::getMaxErrorWithBufferT0)
        .def("getMaxErrorWithBufferT1", &tc::GradientResultsSet::getMaxErrorWithBufferT1);

    nb::class_<tc::CollisionCacheData>(m, "CollisionCacheData")
        .def(nb::init<>())
        .def_rw("contact_results_map", &tc::CollisionCacheData::contact_results_map)
        .def_rw("gradient_results_sets", &tc::CollisionCacheData::gradient_results_sets);

    // ========== DiscreteCollisionEvaluator (base class) ==========
    // The queries are bound here only; virtual dispatch serves every evaluator. getJointGroup
    // returns a reference to the evaluator's own JointGroup, so it keeps the evaluator alive.
    nb::class_<ti::DiscreteCollisionEvaluator>(m, "DiscreteCollisionEvaluator")
        .def("calcCollisions",
             [](ti::DiscreteCollisionEvaluator& self, const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                std::size_t max_allowed) {
                 tc::CollisionCacheData collision_data;
                 self.calcCollisions(collision_data, dof_vals, max_allowed);
                 return collision_data;
             },
             "dof_vals"_a, "max_allowed"_a,
             "Check collisions at dof_vals, keeping at most max_allowed contacts per link pair; "
             "returns the contacts and their gradient result sets")
        .def("getGradient", &ti::DiscreteCollisionEvaluator::getGradient,
             "dofvals"_a, "contact_result"_a,
             "Gradient of one contact at dofvals")
        .def("getJointGroup", &ti::DiscreteCollisionEvaluator::getJointGroup,
             nb::rv_policy::reference_internal)
        .def("getCollisionMarginBuffer", &ti::DiscreteCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::DiscreteCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::DiscreteCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== SingleTimestepCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::SingleTimestepCollisionEvaluator, ti::DiscreteCollisionEvaluator>(m, "SingleTimestepCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== DiscreteCollisionConstraint ==========
    nb::class_<DiscreteCollisionConstraintBinding, ti::ConstraintSet>(m, "DiscreteCollisionConstraint")
        .def("__init__", [](DiscreteCollisionConstraintBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var,
                            int max_num_cnt, bool fixed_sparsity, std::string name) {
                 new (self) DiscreteCollisionConstraintBinding(
                     {var}, std::move(eval), require_in_variable_set(var, "DiscreteCollisionConstraint", "position_var"),
                     max_num_cnt, fixed_sparsity, std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollision")
        .def("getValues", &ti::DiscreteCollisionConstraint::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionConstraint::getCollisionEvaluator);

    // ========== ContinuousCollisionEvaluator (base class) ==========
    nb::class_<ti::ContinuousCollisionEvaluator>(m, "ContinuousCollisionEvaluator")
        .def("calcCollisionData",
             [](ti::ContinuousCollisionEvaluator& self, const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1, bool vars0_fixed, bool vars1_fixed,
                std::size_t max_allowed) {
                 tc::CollisionCacheData collision_data;
                 self.calcCollisionData(collision_data, dof_vals0, dof_vals1, vars0_fixed, vars1_fixed,
                                        max_allowed);
                 return collision_data;
             },
             "dof_vals0"_a, "dof_vals1"_a, "vars0_fixed"_a, "vars1_fixed"_a, "max_allowed"_a,
             "Check collisions along the segment dof_vals0 -> dof_vals1, keeping at most max_allowed "
             "contacts per link pair; returns the contacts and their gradient result sets")
        .def("calcGradientData", &ti::ContinuousCollisionEvaluator::calcGradientData,
             "dof_vals0"_a, "dof_vals1"_a, "contact_results"_a,
             "Gradient of one contact along the segment dof_vals0 -> dof_vals1")
        .def("getJointGroup", &ti::ContinuousCollisionEvaluator::getJointGroup,
             nb::rv_policy::reference_internal)
        .def("getCollisionMarginBuffer", &ti::ContinuousCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::ContinuousCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::ContinuousCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== LVSDiscreteCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSDiscreteCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSDiscreteCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== LVSContinuousCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSContinuousCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSContinuousCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== ContinuousCollisionConstraint ==========
    nb::class_<ContinuousCollisionConstraintBinding, ti::ConstraintSet>(m, "ContinuousCollisionConstraint")
        .def("__init__", [](ContinuousCollisionConstraintBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionConstraint", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionConstraint", "position_var1")};
                 new (self) ContinuousCollisionConstraintBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1, max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision")
        .def("getValues", &ti::ContinuousCollisionConstraint::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionConstraint::getCollisionEvaluator);

    // ========== DiscreteCollisionConstraintD ==========
    // The TrajOpt-Ifopt planner's discrete collision term: one row per contact, recounted by
    // update(), so it has no max_num_cnt / fixed_sparsity.
    nb::class_<DiscreteCollisionConstraintDBinding, ti::ConstraintSet>(m, "DiscreteCollisionConstraintD")
        .def("__init__", [](DiscreteCollisionConstraintDBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var, std::string name) {
                 new (self) DiscreteCollisionConstraintDBinding(
                     {var}, std::move(eval), require_in_variable_set(var, "DiscreteCollisionConstraintD", "position_var"),
                     std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a, "name"_a = "DiscreteCollision",
             "Create a discrete collision constraint with one row per contact (rows change on update())")
        .def("getValues", &ti::DiscreteCollisionConstraintD::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionConstraintDBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionConstraintD::getCollisionEvaluator);

    // ========== ContinuousCollisionConstraintD ==========
    nb::class_<ContinuousCollisionConstraintDBinding, ti::ConstraintSet>(m, "ContinuousCollisionConstraintD")
        .def("__init__", [](ContinuousCollisionConstraintDBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionConstraintD", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionConstraintD", "position_var1")};
                 new (self) ContinuousCollisionConstraintDBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false, "name"_a = "LVSCollision",
             "Create a continuous collision constraint with one row per contact (rows change on update())")
        .def("getValues", &ti::ContinuousCollisionConstraintD::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionConstraintDBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionConstraintD::getCollisionEvaluator);

    // ========== ContinuousCollisionNumericalConstraint ==========
    nb::class_<ContinuousCollisionNumericalConstraintBinding, ti::ConstraintSet>(m, "ContinuousCollisionNumericalConstraint")
        .def("__init__", [](ContinuousCollisionNumericalConstraintBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionNumericalConstraint", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionNumericalConstraint", "position_var1")};
                 new (self) ContinuousCollisionNumericalConstraintBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1,
                                                                       max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision",
             "Create continuous collision constraint with numerical jacobians")
        .def("getValues", &ti::ContinuousCollisionNumericalConstraint::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionNumericalConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionNumericalConstraint::getCollisionEvaluator);

    // ========== JointJerkConstraint ==========
    nb::class_<JointJerkConstraintBinding, ti::ConstraintSet>(m, "JointJerkConstraint")
        .def("__init__",
             [](JointJerkConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointJerkConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointJerkConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointJerk",
             "Create joint jerk constraint (requires 4+ consecutive waypoints)")
        .def("getValues", &ti::JointJerkConstraint::getValues,
             "Get current constraint values");

    // ========== CartLineInfo (struct) ==========
    // The C++ defaults of the trailing Eigen parameters are spelled as overloads (file header note)
    using JointGroupConstPtr = std::shared_ptr<const tesseract::kinematics::JointGroup>;
    nb::class_<ti::CartLineInfo>(m, "CartLineInfo")
        .def(nb::init<>())
        .def(nb::init<JointGroupConstPtr, std::string, std::string, const Eigen::Isometry3d&,
                      const Eigen::Isometry3d&, const Eigen::Isometry3d&, const Eigen::VectorXi&>(),
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a, "source_frame_offset"_a, "indices"_a,
             "Create the info of a line from target_frame_offset1 to target_frame_offset2")
        .def("__init__",
             [](ti::CartLineInfo* self, JointGroupConstPtr manip, std::string source_frame,
                std::string target_frame, const Eigen::Isometry3d& target_frame_offset1,
                const Eigen::Isometry3d& target_frame_offset2, const Eigen::Isometry3d& source_frame_offset) {
                 new (self) ti::CartLineInfo(std::move(manip), std::move(source_frame), std::move(target_frame),
                                             target_frame_offset1, target_frame_offset2, source_frame_offset);
             },
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a, "source_frame_offset"_a,
             "Create the info of a line, constraining all 6 indices")
        .def("__init__",
             [](ti::CartLineInfo* self, JointGroupConstPtr manip, std::string source_frame,
                std::string target_frame, const Eigen::Isometry3d& target_frame_offset1,
                const Eigen::Isometry3d& target_frame_offset2) {
                 new (self) ti::CartLineInfo(std::move(manip), std::move(source_frame), std::move(target_frame),
                                             target_frame_offset1, target_frame_offset2);
             },
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a,
             "Create the info of a line, with an identity source_frame_offset and all 6 indices")
        .def_rw("manip", &ti::CartLineInfo::manip, "The joint group")
        .def_rw("source_frame", &ti::CartLineInfo::source_frame, "TCP frame")
        .def_rw("target_frame", &ti::CartLineInfo::target_frame, "Reference frame")
        .def_rw("source_frame_offset", &ti::CartLineInfo::source_frame_offset, "TCP offset")
        .def_rw("target_frame_offset1", &ti::CartLineInfo::target_frame_offset1, "Line start pose")
        .def_rw("target_frame_offset2", &ti::CartLineInfo::target_frame_offset2, "Line end pose")
        .def_rw("indices", &ti::CartLineInfo::indices,
                "DOF indices to constrain: default {0,1,2,3,4,5}");

    // ========== CartLineConstraint ==========
    nb::class_<CartLineConstraintBinding, ti::ConstraintSet>(m, "CartLineConstraint")
        .def("__init__",
             [](CartLineConstraintBinding* self, ti::CartLineInfo info, std::shared_ptr<const ti::Var> position_var,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) CartLineConstraintBinding({position_var}, std::move(info), position_var, coeffs,
                                                      std::move(name));
             },
             "info"_a, "position_var"_a, "coeffs"_a, "name"_a = "CartLine",
             "Create Cartesian line constraint")
        .def("getValues", &ti::CartLineConstraint::getValues,
             "Get current constraint values")
        .def("setBounds", &set_bounds_checked<CartLineConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getLine", &ti::CartLineConstraint::getLine,
             "The line's two end poses (target_frame_offset1, target_frame_offset2)")
        .def("getInfo", &ti::CartLineConstraint::getInfo, nb::rv_policy::reference_internal,
             "The constraint's CartLineInfo")
        .def("getJacobian", [](const CartLineConstraintBinding& self) {
                 require_linked(self);
                 require_six_indices(self, "getJacobian");
                 return jacobian_checked(self);
             },
             "Get the constraint Jacobian (sparse matrix); raises CartLineIndicesUnsupportedError "
             "unless all 6 indices are set (#237)")
        .def("calcJacobianBlock",
             [](const CartLineConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 require_six_indices(self, "calcJacobianBlock");
                 return jacobian_block_checked(self, joint_vals, self.getInfo().manip->numJoints());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables and all 6 indices (#237)")
        .def("calcValues", &ti::CartLineConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("getLinePoint", &ti::CartLineConstraint::getLinePoint,
             "source_tf"_a, "target_tf1"_a, "target_tf2"_a,
             "Find nearest point on line (uses SLERP for orientation)")
        .def_rw("use_numeric_differentiation", &ti::CartLineConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== DiscreteCollisionNumericalConstraint ==========
    nb::class_<DiscreteCollisionNumericalConstraintBinding, ti::ConstraintSet>(m, "DiscreteCollisionNumericalConstraint")
        .def("__init__", [](DiscreteCollisionNumericalConstraintBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var,
                            int max_num_cnt, bool fixed_sparsity, std::string name) {
                 new (self) DiscreteCollisionNumericalConstraintBinding(
                     {var}, std::move(eval),
                     require_in_variable_set(var, "DiscreteCollisionNumericalConstraint", "position_var"),
                     max_num_cnt, fixed_sparsity, std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollisionNumerical",
             "Create discrete collision constraint with numerical jacobians")
        .def("getValues", &ti::DiscreteCollisionNumericalConstraint::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionNumericalConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("calcValues", &ti::DiscreteCollisionNumericalConstraint::calcValues, "joint_vals"_a)
        .def("calcJacobianBlock",
             [](const DiscreteCollisionNumericalConstraintBinding& self,
                const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(
                     self, joint_vals, self.getCollisionEvaluator()->getJointGroup().numJoints());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables and an update() since the last variable change")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionNumericalConstraint::getCollisionEvaluator);

    // ========== InverseKinematicsInfo (struct) ==========
    using KinematicGroupConstPtr = std::shared_ptr<const tesseract::kinematics::KinematicGroup>;
    nb::class_<ti::InverseKinematicsInfo>(m, "InverseKinematicsInfo")
        .def(nb::init<>())
        .def(nb::init<KinematicGroupConstPtr, std::string, std::string, const Eigen::Isometry3d&>(),
             "manip"_a, "working_frame"_a, "tcp_frame"_a, "tcp_offset"_a,
             "Create IK info; raises if manip has no link tcp_frame")
        .def("__init__",
             [](ti::InverseKinematicsInfo* self, KinematicGroupConstPtr manip, std::string working_frame,
                std::string tcp_frame) {
                 new (self) ti::InverseKinematicsInfo(std::move(manip), std::move(working_frame),
                                                      std::move(tcp_frame));
             },
             "manip"_a, "working_frame"_a, "tcp_frame"_a,
             "Create IK info with an identity tcp_offset; raises if manip has no link tcp_frame")
        .def_rw("manip", &ti::InverseKinematicsInfo::manip, "The kinematic group (with IK solver)")
        .def_rw("working_frame", &ti::InverseKinematicsInfo::working_frame, "Working frame (not currently used)")
        .def_rw("tcp_frame", &ti::InverseKinematicsInfo::tcp_frame, "TCP frame (not currently used)")
        .def_rw("tcp_offset", &ti::InverseKinematicsInfo::tcp_offset, "TCP offset (not currently used)");

    // ========== InverseKinematicsConstraint ==========
    nb::class_<InverseKinematicsConstraintBinding, ti::ConstraintSet>(m, "InverseKinematicsConstraint")
        .def("__init__",
             [](InverseKinematicsConstraintBinding* self, const Eigen::Isometry3d& target_pose,
                ti::InverseKinematicsInfo::ConstPtr kinematic_info, std::shared_ptr<const ti::Var> constraint_var,
                std::shared_ptr<const ti::Var> seed_var, std::string name) {
                 new (self) InverseKinematicsConstraintBinding({constraint_var, seed_var}, target_pose,
                                                               std::move(kinematic_info), constraint_var,
                                                               seed_var, std::move(name));
             },
             "target_pose"_a, "kinematic_info"_a, "constraint_var"_a, "seed_var"_a,
             "name"_a = "InverseKinematics",
             "Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)")
        .def("getValues", &ti::InverseKinematicsConstraint::getValues,
             "Get current constraint values")
        .def("setBounds", &set_bounds_checked<InverseKinematicsConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("setTargetPose", &ti::InverseKinematicsConstraint::setTargetPose, "target_pose"_a,
             "Set the target pose the IK solution is computed for")
        .def("getKinematicInfo", &ti::InverseKinematicsConstraint::getKinematicInfo,
             nb::rv_policy::reference_internal,
             "The constraint's InverseKinematicsInfo");
}
