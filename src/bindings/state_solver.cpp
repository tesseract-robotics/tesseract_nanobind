/**
 * @file state_solver.cpp
 * @brief nanobind bindings for tesseract_state_solver
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <algorithm>
#include <stdexcept>
#include <nanobind/stl/unordered_map.h>

// tesseract_state_solver
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

// tesseract_scene_graph for SceneState and SceneGraph
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>

// tesseract_common
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/eigen_types.h>

namespace tsg = tesseract::scene_graph;
namespace tc = tesseract::common;

namespace {

// gh-219, GH #43: OFKTStateSolver stores a value through nodes_[id] for every given joint
// id (ofkt_state_solver.cpp:226, :246, :268), so an unknown name default-inserts a null node
// and dereferences it; KDLStateSolver logs and skips it. Its vector forms check sizes only with
// assert, and an unknown floating id is std::out_of_range after the joint values were stored.
// The binding owns the Python boundary: validate first, std::invalid_argument -> ValueError.

// The ids not in `allowed`, their names comma-separated, or "" when all are.
template <typename Id>
std::string unknown_names(const std::vector<Id>& ids, const std::vector<Id>& allowed)
{
    std::string unknown;
    for (const auto& id : ids) {
        if (std::find(allowed.begin(), allowed.end(), id) == allowed.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += id.name();
        }
    }
    return unknown;
}

template <typename Map>
std::vector<typename Map::key_type> keys_of(const Map& map)
{
    std::vector<typename Map::key_type> keys;
    keys.reserve(map.size());
    for (const auto& kv : map) keys.push_back(kv.first);
    return keys;
}

// Joint values are set by id on the active joints only: fixed and mimic joints have no node
// value, floating joints take a transform.
void validate_joint_names(const tsg::StateSolver& solver, const std::vector<tc::JointId>& joint_ids,
                          const char* caller)
{
    const std::string unknown = unknown_names(joint_ids, solver.getActiveJointIds());
    if (!unknown.empty())
        throw std::invalid_argument(std::string(caller) + ": unknown or non-active joint names: " + unknown);
}

void validate_names_values(const tsg::StateSolver& solver, const std::vector<tc::JointId>& joint_ids,
                           const Eigen::Ref<const Eigen::VectorXd>& values, const char* caller)
{
    if (static_cast<Eigen::Index>(joint_ids.size()) != values.size())
        throw std::invalid_argument(std::string(caller) + ": joint_ids length (" + std::to_string(joint_ids.size()) +
                                    ") != joint_values length (" + std::to_string(values.size()) + ")");
    validate_joint_names(solver, joint_ids, caller);
}

// A bare joint vector is in getActiveJointIds() order and covers every active joint.
void validate_joint_vector(const tsg::StateSolver& solver, const Eigen::Ref<const Eigen::VectorXd>& values,
                           const char* caller)
{
    const std::size_t n_active = solver.getActiveJointIds().size();
    if (static_cast<Eigen::Index>(n_active) != values.size())
        throw std::invalid_argument(std::string(caller) + ": joint_values length (" + std::to_string(values.size()) +
                                    ") != number of active joints (" + std::to_string(n_active) + ")");
}

void validate_floating_joints(const tsg::StateSolver& solver, const tc::JointIdTransformMap& floating_joint_values,
                              const char* caller)
{
    if (floating_joint_values.empty()) return;  // the common call: skip the id copy
    const std::string unknown = unknown_names(keys_of(floating_joint_values), solver.getFloatingJointIds());
    if (!unknown.empty())
        throw std::invalid_argument(std::string(caller) + ": unknown floating joint names: " + unknown);
}

// getJacobian reads link_map_.at(link_id) (OFKT, std::out_of_range) or logs and throws (KDL).
void require_link(const tsg::StateSolver& solver, const tc::LinkId& link_id, const char* caller)
{
    if (!solver.hasLinkId(link_id))
        throw nb::key_error((std::string(caller) + ": unknown link name: " + link_id.name()).c_str());
}

}  // namespace

NB_MODULE(state_solver, m) {
    m.doc() = "tesseract_state_solver Python bindings";

    // Import common module for the Isometry3d type (else stubs quote the C++ name, which is compiler-specific);
    // LinkId / JointId are registered there too
    nb::module_::import_("tesseract_robotics.common._common");
    // SceneGraph, Joint and Link in the solver constructors and MutableStateSolver (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.scene_graph._scene_graph");

    // Id-keyed transform maps leave C++ as plain dicts keyed by LinkId / JointId; ids hash and
    // compare like their names, so Python can still index them with a str.
    using LinkTransforms = std::unordered_map<tc::LinkId, Eigen::Isometry3d>;
    using JointTransforms = std::unordered_map<tc::JointId, Eigen::Isometry3d>;

    // ========== SceneState ==========
    nb::class_<tsg::SceneState> scene_state(m, "SceneState");
    scene_state
        .def(nb::init<>())
        .def_rw("joints", &tsg::SceneState::joints, nb::rv_policy::copy, "Joint values: joint id -> value [rad or m].")
        .def_rw("floating_joints", &tsg::SceneState::floating_joints, nb::rv_policy::copy, "Floating-joint transforms: joint id -> Isometry3d.")
        .def_prop_rw("link_transforms",
            [](const tsg::SceneState& self) {
                return LinkTransforms(self.link_transforms.begin(), self.link_transforms.end());
            },
            [](tsg::SceneState& self, const LinkTransforms& m) {
                self.link_transforms = tc::LinkIdTransformMap(m.begin(), m.end());
            }, "World transform of every link: link id -> Isometry3d (a copy).")
        .def_prop_rw("joint_transforms",
            [](const tsg::SceneState& self) {
                return JointTransforms(self.joint_transforms.begin(), self.joint_transforms.end());
            },
            [](tsg::SceneState& self, const JointTransforms& m) {
                self.joint_transforms = tc::JointIdTransformMap(m.begin(), m.end());
            }, "World transform of every joint: joint id -> Isometry3d (a copy).")
        .def("getJointValues", &tsg::SceneState::getJointValues, "joint_ids"_a, "The values of `joint_ids`, in that order; an unknown id raises KeyError.")
        // #232: upstream's floating_joints.at(name) (scene_state.cpp:48) throws std::out_of_range
        // for an unknown name; a missing mapping key is KeyError in Python.
        .def("getFloatingJointValues",
            [](const tsg::SceneState& self, const std::vector<tc::JointId>& joint_ids) {
                for (const auto& id : joint_ids)
                    if (self.floating_joints.find(id) == self.floating_joints.end())
                        throw nb::key_error(("getFloatingJointValues: unknown floating joint name: " + id.name()).c_str());
                return self.getFloatingJointValues(joint_ids);
            },
            "joint_ids"_a);
    // Upstream compares the transforms with isApprox(1e-5) (scene_state.cpp:56).
    bind_value_equality(scene_state);

    // ========== StateSolver (abstract base) ==========
    // gh-219: every setState / getState / getJacobian / getLinkTransforms overload under its native
    // name, validated as above.
    // Dispatch: {name: float} matches only the joint-value map, {name: Isometry3d} only the
    // JointIdTransformMap; an empty {} converts to both and reaches the joint-value map first.
    const auto set_state_map = [](tsg::StateSolver& self,
                                  const tsg::SceneState::JointValues& joint_values,
                                  const tc::JointIdTransformMap& floating_joint_values) {
        validate_joint_names(self, keys_of(joint_values), "setState");
        validate_floating_joints(self, floating_joint_values, "setState");
        self.setState(joint_values, floating_joint_values);
    };
    const auto set_state_names_values = [](tsg::StateSolver& self,
                                           const std::vector<tc::JointId>& joint_ids,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                           const tc::JointIdTransformMap& floating_joint_values) {
        validate_names_values(self, joint_ids, joint_values, "setState");
        validate_floating_joints(self, floating_joint_values, "setState");
        self.setState(joint_ids, joint_values, floating_joint_values);
    };

    nb::class_<tsg::StateSolver>(m, "StateSolver")
        .def("setState", [](tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::JointIdTransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "setState");
            validate_floating_joints(self, floating_joint_values, "setState");
            self.setState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("setState", set_state_map, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{}, "Set joint values (a vector in getActiveJointIds order, a {joint: value} map, or ids and values) and optional floating-joint transforms. Unknown joints raise ValueError before anything changes.")
        .def("setState", set_state_names_values,
             "joint_ids"_a, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{}, "Set joint values (a vector in getActiveJointIds order, a {joint: value} map, or ids and values) and optional floating-joint transforms. Unknown joints raise ValueError before anything changes.")
        .def("setState", [](tsg::StateSolver& self, const tc::JointIdTransformMap& floating_joint_values) {
            validate_floating_joints(self, floating_joint_values, "setState");
            self.setState(floating_joint_values);
        }, "floating_joint_values"_a)
        .def("getState", [](const tsg::StateSolver& self) {
            return self.getState();
        }, "The state at the given joint values (forms as setState), or the current state with no arguments, without changing the solver.")
        .def("getState", [](const tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::JointIdTransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("getState", [](const tsg::StateSolver& self,
                            const tsg::SceneState::JointValues& joint_values,
                            const tc::JointIdTransformMap& floating_joint_values) {
            validate_joint_names(self, keys_of(joint_values), "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("getState", [](const tsg::StateSolver& self,
                            const std::vector<tc::JointId>& joint_ids,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::JointIdTransformMap& floating_joint_values) {
            validate_names_values(self, joint_ids, joint_values, "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_ids, joint_values, floating_joint_values);
        }, "joint_ids"_a, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("getState", [](const tsg::StateSolver& self, const tc::JointIdTransformMap& floating_joint_values) {
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(floating_joint_values);
        }, "floating_joint_values"_a)
        // getLinkTransforms(ids, values[, floating]): the C++ out-param returned (out-param rule).
        // One Python overload covers both C++ ones: the 3-argument one is the 4-argument one with
        // the current floating joint values, which an empty map leaves in place.
        .def("getLinkTransforms", [](const tsg::StateSolver& self) {
            return self.getLinkTransforms();
        }, "Every link's world transform, at the current state or at `joint_values` for `joint_ids`.")
        .def("getLinkTransforms", [](const tsg::StateSolver& self,
                                     const std::vector<tc::JointId>& joint_ids,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tc::JointIdTransformMap& floating_joint_values) {
            validate_names_values(self, joint_ids, joint_values, "getLinkTransforms");
            validate_floating_joints(self, floating_joint_values, "getLinkTransforms");
            tc::LinkIdTransformMap link_transforms;
            self.getLinkTransforms(link_transforms, joint_ids, joint_values, floating_joint_values);
            return link_transforms;
        }, "joint_ids"_a, "joint_values"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("getRandomState", &tsg::StateSolver::getRandomState, "A state at uniformly random joint values within the joint limits.")
        // 0.36 dropped the joint-value-map getJacobian (state_solver.h:170-189): vector and ids + vector only
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tc::LinkId& link_id,
                               const tc::JointIdTransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "getJacobian");
            validate_floating_joints(self, floating_joint_values, "getJacobian");
            require_link(self, link_id, "getJacobian");
            return self.getJacobian(joint_values, link_id, floating_joint_values);
        }, "joint_values"_a, "link_id"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const std::vector<tc::JointId>& joint_ids,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tc::LinkId& link_id,
                               const tc::JointIdTransformMap& floating_joint_values) {
            validate_names_values(self, joint_ids, joint_values, "getJacobian");
            validate_floating_joints(self, floating_joint_values, "getJacobian");
            require_link(self, link_id, "getJacobian");
            return self.getJacobian(joint_ids, joint_values, link_id, floating_joint_values);
        }, "joint_ids"_a, "joint_values"_a, "link_id"_a, "floating_joint_values"_a = tc::JointIdTransformMap{})
        // Id getters
        .def("getJointIds", &tsg::StateSolver::getJointIds, "Every joint id of the scene graph.")
        .def("getFloatingJointIds", &tsg::StateSolver::getFloatingJointIds, "The floating joint ids.")
        .def("getActiveJointIds", &tsg::StateSolver::getActiveJointIds, "The active (non-fixed, non-mimic) joint ids, in state-vector order.")
        .def("getBaseLinkId", &tsg::StateSolver::getBaseLinkId, "The root link id.")
        .def("getLinkIds", &tsg::StateSolver::getLinkIds, "Every link id.")
        .def("getActiveLinkIds", &tsg::StateSolver::getActiveLinkIds, "Links moved by an active joint.")
        .def("getStaticLinkIds", &tsg::StateSolver::getStaticLinkIds, "Links no active joint moves.")
        // Link queries
        .def("isActiveLinkId", &tsg::StateSolver::isActiveLinkId, "link_id"_a, "True if an active joint moves `link_id`.")
        .def("hasLinkId", &tsg::StateSolver::hasLinkId, "link_id"_a, "True if `link_id` is a link of the scene graph.")
        // Transform getters
        .def("getLinkTransform", nb::overload_cast<const tc::LinkId&>(&tsg::StateSolver::getLinkTransform, nb::const_),
             "link_id"_a, "The current world transform of `link_id`.")
        .def("getRelativeLinkTransform", &tsg::StateSolver::getRelativeLinkTransform,
             "from_link_id"_a, "to_link_id"_a, "The transform of `to_link_id` in the frame of `from_link_id`, at the current state.")
        .def("getLimits", &tsg::StateSolver::getLimits, "The active joints' position, velocity, acceleration and jerk limits, in getActiveJointIds order.")
        .def("clone", [](const tsg::StateSolver& self) { return self.clone(); }, "An independent copy at the same state.");

    // ========== MutableStateSolver (abstract, inherits StateSolver) ==========
    nb::class_<tsg::MutableStateSolver, tsg::StateSolver>(m, "MutableStateSolver")
        .def("setRevision", &tsg::MutableStateSolver::setRevision, "revision"_a, "Set the revision this solver tracks (the environment's command count).")
        .def("getRevision", &tsg::MutableStateSolver::getRevision, "The revision this solver tracks.")
        .def("addLink", &tsg::MutableStateSolver::addLink, "link"_a, "joint"_a, "Add `link` attached by `joint`; False if the link or joint exists.")
        .def("moveLink", &tsg::MutableStateSolver::moveLink, "joint"_a, "Re-attach a link by replacing its parent joint with `joint`; False on failure.")
        .def("removeLink", &tsg::MutableStateSolver::removeLink, "name"_a, "Remove the link `name` and its subtree; False if unknown.")
        .def("replaceJoint", &tsg::MutableStateSolver::replaceJoint, "joint"_a, "Replace the joint of the same name (same child link); False on failure.")
        .def("removeJoint", &tsg::MutableStateSolver::removeJoint, "name"_a, "Remove the joint `name` and its child subtree; False if unknown.")
        .def("moveJoint", &tsg::MutableStateSolver::moveJoint, "name"_a, "parent_link"_a, "Re-parent the joint `name` onto `parent_link`; False on failure.")
        .def("changeJointOrigin", &tsg::MutableStateSolver::changeJointOrigin, "name"_a, "new_origin"_a, "Set the origin of the joint `name`; False if unknown.")
        .def("changeJointPositionLimits", &tsg::MutableStateSolver::changeJointPositionLimits,
             "name"_a, "lower"_a, "upper"_a, "Set the position limits of the joint `name`; False if unknown.")
        .def("changeJointVelocityLimits", &tsg::MutableStateSolver::changeJointVelocityLimits,
             "name"_a, "limit"_a, "Set the velocity limit of the joint `name`; False if unknown.")
        .def("changeJointAccelerationLimits", &tsg::MutableStateSolver::changeJointAccelerationLimits,
             "name"_a, "limit"_a, "Set the acceleration limit of the joint `name`; False if unknown.")
        .def("changeJointJerkLimits", &tsg::MutableStateSolver::changeJointJerkLimits,
             "name"_a, "limit"_a, "Set the jerk limit of the joint `name`; False if unknown.")
        // False (and a tesseract error) when the joint's parent link is not in the solver, its
        // child is not in scene_graph, or the joint name exists (ofkt_state_solver.cpp:861-886).
        .def("insertSceneGraph", &tsg::MutableStateSolver::insertSceneGraph,
             "scene_graph"_a, "joint"_a, "prefix"_a = "");

    // ========== KDLStateSolver (concrete) ==========
    // Upstream behaviour, kept: floating_joint_values are ignored by every setState / getState /
    // getJacobian overload, and setState(floating_joint_values) / getState(floating_joint_values)
    // throw "not supported" (kdl_state_solver.cpp:78-141, :232). The (scene_graph, data)
    // constructor is not bound: KDLTreeData is a KDL tree type (S8).
    nb::class_<tsg::KDLStateSolver, tsg::StateSolver>(m, "KDLStateSolver",
        "State solver on a KDL tree. Floating joint values are ignored, and setState / getState "
        "with only floating_joint_values raise RuntimeError (upstream: not supported).")
        .def(nb::init<const tsg::SceneGraph&>(), "scene_graph"_a)
        .def("clone", [](const tsg::KDLStateSolver& self) {
            return self.clone();
        }, "An independent copy at the same state.");

    // ========== OFKTStateSolver (concrete, mutable) ==========
    nb::class_<tsg::OFKTStateSolver, tsg::MutableStateSolver>(m, "OFKTStateSolver")
        // `prefix` stays unbound: upstream 0.35.0 looks the prefixed root up in the unprefixed
        // graph (ofkt_state_solver.cpp:1053, :1075) and throws for any non-empty prefix (#247)
        .def(nb::init<const tsg::SceneGraph&>(), "scene_graph"_a)
        // #247: the root link only; grow it with addLink / moveLink
        .def(nb::init<const tc::LinkId&>(), "root_link_id"_a)
        .def("clone", [](const tsg::OFKTStateSolver& self) {
            return self.clone();
        }, "An independent copy at the same state.");
}
