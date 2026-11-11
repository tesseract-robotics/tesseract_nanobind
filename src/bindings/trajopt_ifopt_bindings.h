#pragma once

// The trajopt_ifopt module's translation units, and what they share: the ConstraintSet
// trampoline, the Var-checked constraint wrappers, the named errors and the argument checks.
// NB_MODULE(_trajopt_ifopt) in trajopt_ifopt.cpp calls bind_collision_terms in registration order.

#include "tesseract_nb.h"
#include "collision_defaults.h"
#include <sstream>

#include <nanobind/stl/tuple.h>
#include <mutex>
#include <type_traits>
#include <unordered_set>

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
#include <trajopt_ifopt/constraints/cartesian_axis_align_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_cone_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>

// trajopt_common
#include <trajopt_common/collision_types.h>
#include <trajopt_common/collision_utils.h>

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

    // getNonZeros() is a Jacobian reservation hint, and a Python set cannot set the protected
    // non_zeros_. trajopt leaves it at -1 (component.h:292) and asserts that a subclass set it
    // (assert(non_zeros_ >= 0), component.cpp:91, trajopt a504b22): a non-negative hint is the
    // subclass's job. Release builds skip the assert, and TrajOptQPProblem reserves the raw sum
    // of its cost hints (trajopt_qp_problem.cpp:944), so -1 asked for SIZE_MAX ("ValueError:
    // vector"). trajopt#599 (clamp negative hints) is open with no PR. 0 is a valid hint:
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
// CartAxisKinematics::toUnitAxis throws a bare std::runtime_error for a zero or non-finite
// axis; every CartAxis entry point checks first, with upstream's criterion, for a ValueError.
struct DegenerateAxisError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

inline const Eigen::Vector3d& require_axis(const Eigen::Vector3d& axis, const std::string& what) {
    if (!axis.allFinite() || !(axis.norm() > 0)) {
        std::ostringstream msg;
        msg << what << " must be finite and non-zero, got [" << axis.transpose() << "]";
        throw DegenerateAxisError(msg.str());
    }
    return axis;
}

struct ConstraintNotLinkedError : std::runtime_error {
    using std::runtime_error::runtime_error;
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

// Whether set holds node; compares node against the set's own (live) Nodes, never dereferences it
inline bool holds(const ti::NodesVariables& set, const ti::Node* node) {
    for (const auto& member : set.getNodes())
        if (member.get() == node)
            return true;
    return false;
}

// Whether set holds var: its Node among the set's own, and var among that Node's. Never
// dereferences var.getParent(), which dangles once the Var's set (and Node) is freed.
inline bool set_holds_var(const ti::NodesVariables& set, const ti::Var& var) {
    for (const auto& node : set.getNodes())
        if (node.get() == var.getParent())
            return node->hasVar(var.getIdentifier()) && node->getVar(var.getIdentifier()).get() == &var;
    return false;
}

// Node.getParent on a Node whose variable set Python does not hold: one a planner built in C++
// (handed to a Python profile's create), freed when its solve returns.
struct NodeParentUnavailableError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// The variable sets built from Python (NodesVariables(...), createNodesVariables): shared-owned,
// and registered while alive. A Node keeps only a raw pointer to its set, which a C++ owner (a
// planner) may free while Python still holds the Node, so Node.getParent hands out a set only from
// this registry, and only one that holds that Node: a freed set's address may be reused.
class NodesVariablesBinding final : public ti::NodesVariables,
                                    public std::enable_shared_from_this<NodesVariablesBinding> {
public:
    NodesVariablesBinding(std::string variable_name, std::vector<std::unique_ptr<ti::Node>> nodes)
        : ti::NodesVariables(std::move(variable_name), std::move(nodes)) {
        const std::lock_guard<std::mutex> lock(mutex());
        live().insert(this);
    }
    ~NodesVariablesBinding() override {
        // a C++ owner (TrajOptQPProblem on a task composer worker) may drop the last reference
        const std::lock_guard<std::mutex> lock(mutex());
        live().erase(this);
    }
    NodesVariablesBinding(const NodesVariablesBinding&) = delete;
    NodesVariablesBinding& operator=(const NodesVariablesBinding&) = delete;
    NodesVariablesBinding(NodesVariablesBinding&&) = delete;
    NodesVariablesBinding& operator=(NodesVariablesBinding&&) = delete;

    // The live Python-built set for which holds_member(set) is true (a full scan; the Var path), kept alive by the result; null if
    // none. A registered set is alive while the lock is held (its destructor waits for the lock), so
    // holds_member may read it; only the match is locked, and released after the lock.
    template <class HoldsMember>
    static std::shared_ptr<ti::NodesVariables> find_live(const HoldsMember& holds_member) {
        std::shared_ptr<const NodesVariablesBinding> set;
        const std::lock_guard<std::mutex> lock(mutex());
        for (const ti::NodesVariables* candidate : live()) {
            if (holds_member(*candidate)) {
                set = static_cast<const NodesVariablesBinding*>(candidate)->weak_from_this().lock();
                break;
            }
        }
        // Python's NodesVariables is mutable, as for a set Python built itself
        return std::const_pointer_cast<NodesVariablesBinding>(set);
    }

    // node's set, kept alive by the result; raises unless it is a live Python-built set holding node
    static std::shared_ptr<ti::NodesVariables> parent_of(const ti::Node& node) {
        std::shared_ptr<const NodesVariablesBinding> set;
        {
            // node is alive (the caller holds it), so its parent pointer is the key: no scan
            const std::lock_guard<std::mutex> lock(mutex());
            const ti::NodesVariables* parent = node.getParent();
            if (live().count(parent) != 0 && holds(*parent, &node))
                set = static_cast<const NodesVariablesBinding*>(parent)->weak_from_this().lock();
        }
        if (!set)
            throw NodeParentUnavailableError(
                "Node.getParent: the variable set of Node '" + node.getName() +
                "' is owned by C++ (a planner's, handed to a profile's create) and is not reachable "
                "from Python; it may be gone already");
        return std::const_pointer_cast<NodesVariablesBinding>(set);
    }

private:
    // Never freed: a set a C++ owner still holds may be destroyed during static destruction
    static std::mutex& mutex() {
        static auto* m = new std::mutex;
        return *m;
    }
    static std::unordered_set<const ti::NodesVariables*>& live() {
        static auto* sets = new std::unordered_set<const ti::NodesVariables*>;
        return *sets;
    }
};

// calcForwardNumJac sizes the Jacobian by len(f(x)) and subtracts every later f(x + h e_i) from
// f(x) unchecked: an output of another length reads or writes out of range.
struct NumericalJacobianSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// The collision constraints' update() keys their contact cache on position_var->getParent()
// ->getParent()->getHash() unchecked (discrete_collision_constraint.cpp:70, :179,
// continuous_collision_constraint.cpp:82, :278, and init() from update(), the numerical ones :67,
// :92, trajopt 0.35.0): no other member reads the Var's parent chain.
template <class Constraint>
constexpr bool reads_var_parent_chain = std::is_same_v<Constraint, ti::DiscreteCollisionConstraint> ||
                                        std::is_same_v<Constraint, ti::DiscreteCollisionConstraintD> ||
                                        std::is_same_v<Constraint, ti::DiscreteCollisionNumericalConstraint> ||
                                        std::is_same_v<Constraint, ti::ContinuousCollisionConstraint> ||
                                        std::is_same_v<Constraint, ti::ContinuousCollisionConstraintD> ||
                                        std::is_same_v<Constraint, ti::ContinuousCollisionNumericalConstraint>;

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

    // A linked constraint's Vars are in its set, which it keeps alive (the link check). Unlinked, a
    // collision constraint's Var may outlive its set (a planner's, kept from a profile's create):
    // update() needs that set alive, found through the registry and held for the call.
    int update() override {
        if constexpr (reads_var_parent_chain<Constraint>) {
            if (!this->variables_) {
                std::vector<std::shared_ptr<ti::NodesVariables>> held;
                for (const auto& var : vars_) {
                    held.push_back(NodesVariablesBinding::find_live(
                        [&var](const ti::NodesVariables& candidate) { return set_holds_var(candidate, *var); }));
                    if (!held.back())
                        throw NodeParentUnavailableError(
                            "update: constraint set '" + this->getName() + "' is not linked, and the "
                            "variable set of its Var '" + var->getIdentifier() + "' is owned by C++ (a "
                            "planner's) or gone; build it on a Var of a set built from Python");
                }
                return Constraint::update();
            }
        }
        return Constraint::update();
    }

private:
    void initVariableDependedQuantities(const ti::Variables::Ptr& variables) override {
        // A Var keeps a raw pointer to its Node, which may be freed (with its set) while the
        // constraint holds the Var, and its address reused: find the Node among the set's own,
        // then the Var among that Node's, and never dereference the Var's pointer.
        const auto* set = dynamic_cast<const ti::NodesVariables*>(variables.get());
        for (const auto& var : vars_) {
            if (set != nullptr && set_holds_var(*set, *var))
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
using CartAxisConeConstraintBinding = VarCheckedConstraint<ti::CartAxisConeConstraint>;
using CartAxisAlignConstraintBinding = VarCheckedConstraint<ti::CartAxisAlignConstraint>;
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
// constructor). With that chain, the Node a Var's getParent() reaches is alive. A Node a C++ owner
// hands to Python (a profile's create) keeps only itself alive: Node.getParent checks its set.
//
// Handing a Node to NodesVariables relinquishes its Python object, as nanobind's unique_ptr caster
// does, but nanobind keeps that object in its instance map until the caller drops it, and returns
// it for the same pointer: unusable. So the set's getters return the Node's registered object only
// when it is usable, and otherwise a fresh reference to the Node that keeps owner alive.
inline nb::object node_object(const ti::Node* node, nb::handle owner) {
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
inline std::vector<std::unique_ptr<ti::Node>> take_over_nodes(const std::vector<nb::handle>& nodes) {
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
        // nanobind::detail API (nanobind/nb_lib.h, nanobind 2.12.0 pinned here); recheck on a bump
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

inline const std::shared_ptr<const ti::Var>& require_in_variable_set(const std::shared_ptr<const ti::Var>& var,
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

// trajopt 0.35.0 checks a bounds vector's length against the rows it describes with assert()
// (compiled out) or a log line, then indexes by row: a wrong length reads or writes out of range.
struct BoundsSizeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// getClosestValidPoint / calcBoundsErrors / calcBoundsViolations index bounds by input entry
inline void require_one_bound_per_value(const Eigen::Ref<const Eigen::VectorXd>& input,
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

inline ti::Variables::Ptr require_linked(const ti::ConstraintSet& self, const char* method = "getJacobian") {
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

// JointVel/Accel/JerkConstraint(bounds, position_vars, coeffs, name, range_bound_handling)
// (trajopt#606): one bound per joint, repeated for every finite-difference row block.
template <class Binding>
inline auto joint_diff_bounds_init(const char* caller) {
    return [caller](Binding* self, const std::vector<ti::Bounds>& bounds,
                    const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                    const Eigen::VectorXd& coeffs, std::string name,
                    ti::RangeBoundHandling range_bound_handling) {
        new (self) Binding(position_vars, bounds,
                           require_non_null(position_vars, caller, "position_vars", "a Var"), coeffs,
                           std::move(name), range_bound_handling);
    };
}

constexpr const char* JOINT_DIFF_BOUNDS_DOC =
    "Create the constraint with one bound per joint, the same for every step: an equality, a "
    "one-sided limit or a range. The rows are undivided finite differences, so the bounds are "
    "per step. coeffs has length n_dof, 1 (one weight for every joint) or 0 (weight 1 for "
    "every joint). A range becomes a lower-bound row followed by an upper-bound row unless "
    "range_bound_handling is KEEP_AS_IS, which TrajOptQPProblem refuses.";

inline ti::Jacobian jacobian_checked(const ti::Differentiable& self) {
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


// The collision evaluators, constraints and their trajopt_common data (trajopt_ifopt_collision.cpp).
void bind_collision_terms(nb::module_& m);
