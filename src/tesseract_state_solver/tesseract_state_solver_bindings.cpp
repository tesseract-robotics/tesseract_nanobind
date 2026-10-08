/**
 * @file tesseract_state_solver_bindings.cpp
 * @brief nanobind bindings for tesseract_state_solver
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <algorithm>
#include <stdexcept>

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

// gh-219, GH #43: OFKTStateSolver stores a value through nodes_[name] for every given joint
// name (ofkt_state_solver.cpp:226, :246, :268), so an unknown name default-inserts a null node
// and dereferences it; KDLStateSolver logs and skips it. Its vector forms check sizes only with
// assert, and an unknown floating name is std::out_of_range after the joint values were stored.
// The binding owns the Python boundary: validate first, std::invalid_argument -> ValueError.

// The names not in `allowed`, comma-separated, or "" when all are.
std::string unknown_names(const std::vector<std::string>& names, const std::vector<std::string>& allowed)
{
    std::string unknown;
    for (const auto& name : names) {
        if (std::find(allowed.begin(), allowed.end(), name) == allowed.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += name;
        }
    }
    return unknown;
}

template <typename Map>
std::vector<std::string> keys_of(const Map& map)
{
    std::vector<std::string> keys;
    keys.reserve(map.size());
    for (const auto& kv : map) keys.push_back(kv.first);
    return keys;
}

// Joint values are set by name on the active joints only: fixed and mimic joints have no node
// value, floating joints take a transform.
void validate_joint_names(const tsg::StateSolver& solver, const std::vector<std::string>& names,
                          const char* caller)
{
    const std::string unknown = unknown_names(names, solver.getActiveJointNames());
    if (!unknown.empty())
        throw std::invalid_argument(std::string(caller) + ": unknown or non-active joint names: " + unknown);
}

void validate_names_values(const tsg::StateSolver& solver, const std::vector<std::string>& names,
                           const Eigen::Ref<const Eigen::VectorXd>& values, const char* caller)
{
    if (static_cast<Eigen::Index>(names.size()) != values.size())
        throw std::invalid_argument(std::string(caller) + ": joint_names length (" + std::to_string(names.size()) +
                                    ") != joint_values length (" + std::to_string(values.size()) + ")");
    validate_joint_names(solver, names, caller);
}

// A bare joint vector is in getActiveJointNames() order and covers every active joint.
void validate_joint_vector(const tsg::StateSolver& solver, const Eigen::Ref<const Eigen::VectorXd>& values,
                           const char* caller)
{
    const std::size_t n_active = solver.getActiveJointNames().size();
    if (static_cast<Eigen::Index>(n_active) != values.size())
        throw std::invalid_argument(std::string(caller) + ": joint_values length (" + std::to_string(values.size()) +
                                    ") != number of active joints (" + std::to_string(n_active) + ")");
}

void validate_floating_joints(const tsg::StateSolver& solver, const tc::TransformMap& floating_joint_values,
                              const char* caller)
{
    if (floating_joint_values.empty()) return;  // the common call: skip the name copy
    const std::string unknown = unknown_names(keys_of(floating_joint_values), solver.getFloatingJointNames());
    if (!unknown.empty())
        throw std::invalid_argument(std::string(caller) + ": unknown floating joint names: " + unknown);
}

// getJacobian reads link_map_.at(link_name) (OFKT, std::out_of_range) or logs and throws (KDL).
void require_link(const tsg::StateSolver& solver, const std::string& link_name, const char* caller)
{
    if (!solver.hasLinkName(link_name))
        throw nb::key_error((std::string(caller) + ": unknown link name: " + link_name).c_str());
}

}  // namespace

NB_MODULE(_tesseract_state_solver, m) {
    m.doc() = "tesseract_state_solver Python bindings";

    // Import common module for the Isometry3d type (else stubs quote the C++ name, which is compiler-specific)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    // SceneGraph, Joint and Link in the solver constructors and MutableStateSolver (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph");

    // ========== SceneState ==========
    nb::class_<tsg::SceneState>(m, "SceneState")
        .def(nb::init<>())
        .def_rw("joints", &tsg::SceneState::joints)
        .def_prop_rw("link_transforms",
            [](const tsg::SceneState& self) {
                // Convert AlignedMap to std::map for Python
                std::map<std::string, Eigen::Isometry3d> result;
                for (const auto& p : self.link_transforms) {
                    result[p.first] = p.second;
                }
                return result;
            },
            [](tsg::SceneState& self, const std::map<std::string, Eigen::Isometry3d>& m) {
                self.link_transforms.clear();
                for (const auto& p : m) {
                    self.link_transforms[p.first] = p.second;
                }
            })
        .def_prop_rw("joint_transforms",
            [](const tsg::SceneState& self) {
                std::map<std::string, Eigen::Isometry3d> result;
                for (const auto& p : self.joint_transforms) {
                    result[p.first] = p.second;
                }
                return result;
            },
            [](tsg::SceneState& self, const std::map<std::string, Eigen::Isometry3d>& m) {
                self.joint_transforms.clear();
                for (const auto& p : m) {
                    self.joint_transforms[p.first] = p.second;
                }
            })
        .def("getJointValues", &tsg::SceneState::getJointValues, "joint_names"_a);

    // ========== StateSolver (abstract base) ==========
    // gh-219: every setState / getState / getJacobian / getLinkTransforms overload under its native
    // name, validated as above. The Python-only aliases setStateByMap / setStateByNamesAndValues are
    // bound to the same lambdas as their native forms (kept until a separate removal).
    // Dispatch: {name: float} matches only the joint-value map, {name: Isometry3d} only the
    // TransformMap; an empty {} converts to both and reaches the joint-value map first.
    const auto set_state_map = [](tsg::StateSolver& self,
                                  const std::unordered_map<std::string, double>& joint_values,
                                  const tc::TransformMap& floating_joint_values) {
        validate_joint_names(self, keys_of(joint_values), "setState");
        validate_floating_joints(self, floating_joint_values, "setState");
        self.setState(joint_values, floating_joint_values);
    };
    const auto set_state_names_values = [](tsg::StateSolver& self,
                                           const std::vector<std::string>& joint_names,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                           const tc::TransformMap& floating_joint_values) {
        validate_names_values(self, joint_names, joint_values, "setState");
        validate_floating_joints(self, floating_joint_values, "setState");
        self.setState(joint_names, joint_values, floating_joint_values);
    };

    nb::class_<tsg::StateSolver>(m, "StateSolver")
        .def("setState", [](tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::TransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "setState");
            validate_floating_joints(self, floating_joint_values, "setState");
            self.setState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("setState", set_state_map, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("setState", set_state_names_values,
             "joint_names"_a, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("setState", [](tsg::StateSolver& self, const tc::TransformMap& floating_joint_values) {
            validate_floating_joints(self, floating_joint_values, "setState");
            self.setState(floating_joint_values);
        }, "floating_joint_values"_a)
        .def("setStateByMap", set_state_map, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("setStateByNamesAndValues", set_state_names_values,
             "joint_names"_a, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getState", [](const tsg::StateSolver& self) {
            return self.getState();
        })
        .def("getState", [](const tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::TransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getState", [](const tsg::StateSolver& self,
                            const std::unordered_map<std::string, double>& joint_values,
                            const tc::TransformMap& floating_joint_values) {
            validate_joint_names(self, keys_of(joint_values), "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_values, floating_joint_values);
        }, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getState", [](const tsg::StateSolver& self,
                            const std::vector<std::string>& joint_names,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::TransformMap& floating_joint_values) {
            validate_names_values(self, joint_names, joint_values, "getState");
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(joint_names, joint_values, floating_joint_values);
        }, "joint_names"_a, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getState", [](const tsg::StateSolver& self, const tc::TransformMap& floating_joint_values) {
            validate_floating_joints(self, floating_joint_values, "getState");
            return self.getState(floating_joint_values);
        }, "floating_joint_values"_a)
        // getLinkTransforms(names, values[, floating]): the C++ out-param returned (out-param rule).
        // One Python overload covers both C++ ones: the 3-argument one is the 4-argument one with
        // the current floating joint values, which an empty map leaves in place.
        .def("getLinkTransforms", [](const tsg::StateSolver& self) {
            return self.getLinkTransforms();
        })
        .def("getLinkTransforms", [](const tsg::StateSolver& self,
                                     const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tc::TransformMap& floating_joint_values) {
            validate_names_values(self, joint_names, joint_values, "getLinkTransforms");
            validate_floating_joints(self, floating_joint_values, "getLinkTransforms");
            tc::TransformMap link_transforms;
            self.getLinkTransforms(link_transforms, joint_names, joint_values, floating_joint_values);
            return link_transforms;
        }, "joint_names"_a, "joint_values"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getRandomState", &tsg::StateSolver::getRandomState)
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const std::string& link_name,
                               const tc::TransformMap& floating_joint_values) {
            validate_joint_vector(self, joint_values, "getJacobian");
            validate_floating_joints(self, floating_joint_values, "getJacobian");
            require_link(self, link_name, "getJacobian");
            return self.getJacobian(joint_values, link_name, floating_joint_values);
        }, "joint_values"_a, "link_name"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const std::unordered_map<std::string, double>& joint_values,
                               const std::string& link_name,
                               const tc::TransformMap& floating_joint_values) {
            validate_joint_names(self, keys_of(joint_values), "getJacobian");
            validate_floating_joints(self, floating_joint_values, "getJacobian");
            require_link(self, link_name, "getJacobian");
            return self.getJacobian(joint_values, link_name, floating_joint_values);
        }, "joint_values"_a, "link_name"_a, "floating_joint_values"_a = tc::TransformMap{})
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const std::string& link_name,
                               const tc::TransformMap& floating_joint_values) {
            validate_names_values(self, joint_names, joint_values, "getJacobian");
            validate_floating_joints(self, floating_joint_values, "getJacobian");
            require_link(self, link_name, "getJacobian");
            return self.getJacobian(joint_names, joint_values, link_name, floating_joint_values);
        }, "joint_names"_a, "joint_values"_a, "link_name"_a, "floating_joint_values"_a = tc::TransformMap{})
        // Name getters
        .def("getJointNames", &tsg::StateSolver::getJointNames)
        .def("getFloatingJointNames", &tsg::StateSolver::getFloatingJointNames)
        .def("getActiveJointNames", &tsg::StateSolver::getActiveJointNames)
        .def("getBaseLinkName", &tsg::StateSolver::getBaseLinkName)
        .def("getLinkNames", &tsg::StateSolver::getLinkNames)
        .def("getActiveLinkNames", &tsg::StateSolver::getActiveLinkNames)
        .def("getStaticLinkNames", &tsg::StateSolver::getStaticLinkNames)
        // Link queries
        .def("isActiveLinkName", &tsg::StateSolver::isActiveLinkName, "link_name"_a)
        .def("hasLinkName", &tsg::StateSolver::hasLinkName, "link_name"_a)
        // Transform getters
        .def("getLinkTransform", &tsg::StateSolver::getLinkTransform, "link_name"_a)
        .def("getRelativeLinkTransform", &tsg::StateSolver::getRelativeLinkTransform,
             "from_link_name"_a, "to_link_name"_a)
        .def("getLimits", &tsg::StateSolver::getLimits)
        .def("clone", [](const tsg::StateSolver& self) { return self.clone(); });

    // ========== MutableStateSolver (abstract, inherits StateSolver) ==========
    nb::class_<tsg::MutableStateSolver, tsg::StateSolver>(m, "MutableStateSolver")
        .def("setRevision", &tsg::MutableStateSolver::setRevision, "revision"_a)
        .def("getRevision", &tsg::MutableStateSolver::getRevision)
        .def("addLink", &tsg::MutableStateSolver::addLink, "link"_a, "joint"_a)
        .def("moveLink", &tsg::MutableStateSolver::moveLink, "joint"_a)
        .def("removeLink", &tsg::MutableStateSolver::removeLink, "name"_a)
        .def("replaceJoint", &tsg::MutableStateSolver::replaceJoint, "joint"_a)
        .def("removeJoint", &tsg::MutableStateSolver::removeJoint, "name"_a)
        .def("moveJoint", &tsg::MutableStateSolver::moveJoint, "name"_a, "parent_link"_a)
        .def("changeJointOrigin", &tsg::MutableStateSolver::changeJointOrigin, "name"_a, "new_origin"_a)
        .def("changeJointPositionLimits", &tsg::MutableStateSolver::changeJointPositionLimits,
             "name"_a, "lower"_a, "upper"_a)
        .def("changeJointVelocityLimits", &tsg::MutableStateSolver::changeJointVelocityLimits,
             "name"_a, "limit"_a)
        .def("changeJointAccelerationLimits", &tsg::MutableStateSolver::changeJointAccelerationLimits,
             "name"_a, "limit"_a)
        .def("changeJointJerkLimits", &tsg::MutableStateSolver::changeJointJerkLimits,
             "name"_a, "limit"_a)
        // False (and a console_bridge error) when the joint's parent link is not in the solver, its
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
        });

    // ========== OFKTStateSolver (concrete, mutable) ==========
    nb::class_<tsg::OFKTStateSolver, tsg::MutableStateSolver>(m, "OFKTStateSolver")
        .def(nb::init<const tsg::SceneGraph&>(), "scene_graph"_a)
        .def("clone", [](const tsg::OFKTStateSolver& self) {
            return self.clone();
        });
}
