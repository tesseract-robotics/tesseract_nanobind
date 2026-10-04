/**
 * @file tesseract_state_solver_bindings.cpp
 * @brief nanobind bindings for tesseract_state_solver
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
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

NB_MODULE(_tesseract_state_solver, m) {
    m.doc() = "tesseract_state_solver Python bindings";

    // LinkId / JointId are registered in tesseract_common
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // Id-keyed transform maps leave C++ as plain dicts keyed by LinkId / JointId; ids hash and
    // compare like their names, so Python can still index them with a str.
    using LinkTransforms = std::unordered_map<tc::LinkId, Eigen::Isometry3d>;
    using JointTransforms = std::unordered_map<tc::JointId, Eigen::Isometry3d>;

    // ========== SceneState ==========
    nb::class_<tsg::SceneState>(m, "SceneState")
        .def(nb::init<>())
        .def_rw("joints", &tsg::SceneState::joints)
        .def_prop_rw("link_transforms",
            [](const tsg::SceneState& self) {
                return LinkTransforms(self.link_transforms.begin(), self.link_transforms.end());
            },
            [](tsg::SceneState& self, const LinkTransforms& m) {
                self.link_transforms = tc::LinkIdTransformMap(m.begin(), m.end());
            })
        .def_prop_rw("joint_transforms",
            [](const tsg::SceneState& self) {
                return JointTransforms(self.joint_transforms.begin(), self.joint_transforms.end());
            },
            [](tsg::SceneState& self, const JointTransforms& m) {
                self.joint_transforms = tc::JointIdTransformMap(m.begin(), m.end());
            })
        .def("getJointValues", &tsg::SceneState::getJointValues, "joint_ids"_a);

    // ========== StateSolver (abstract base) ==========
    nb::class_<tsg::StateSolver>(m, "StateSolver")
        // getState methods - multiple overloads with same name for Python compatibility
        .def("getState", [](const tsg::StateSolver& self) {
            return self.getState();
        })
        .def("getState", [](const tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_values);
        }, "joint_values"_a)
        .def("getState", [](const tsg::StateSolver& self,
                            const tsg::SceneState::JointValues& joint_values) {
            return self.getState(joint_values);
        }, "joint_values"_a)
        .def("getState", [](const tsg::StateSolver& self,
                            const std::vector<tc::JointId>& joint_ids,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_ids, joint_values);
        }, "joint_ids"_a, "joint_values"_a)
        .def("getRandomState", &tsg::StateSolver::getRandomState)
        // setState methods
        .def("setState", [](tsg::StateSolver& self, const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_values);
        }, "joint_values"_a)
        .def("setStateByMap", [](tsg::StateSolver& self, const tsg::SceneState::JointValues& joint_values) {
            self.setState(joint_values);
        }, "joint_values"_a)
        .def("setStateByNamesAndValues", [](tsg::StateSolver& self,
                                             const std::vector<tc::JointId>& joint_ids,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_ids, joint_values);
        }, "joint_ids"_a, "joint_values"_a)
        // Jacobian methods
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const tc::LinkId& link_id) {
            return self.getJacobian(joint_values, link_id);
        }, "joint_values"_a, "link_id"_a)
        // Id getters
        .def("getJointIds", &tsg::StateSolver::getJointIds)
        .def("getFloatingJointIds", &tsg::StateSolver::getFloatingJointIds)
        .def("getActiveJointIds", &tsg::StateSolver::getActiveJointIds)
        .def("getBaseLinkId", &tsg::StateSolver::getBaseLinkId)
        .def("getLinkIds", &tsg::StateSolver::getLinkIds)
        .def("getActiveLinkIds", &tsg::StateSolver::getActiveLinkIds)
        .def("getStaticLinkIds", &tsg::StateSolver::getStaticLinkIds)
        // Link queries
        .def("isActiveLinkId", &tsg::StateSolver::isActiveLinkId, "link_id"_a)
        .def("hasLinkId", &tsg::StateSolver::hasLinkId, "link_id"_a)
        // Transform getters
        .def("getLinkTransform", nb::overload_cast<const tc::LinkId&>(&tsg::StateSolver::getLinkTransform, nb::const_),
             "link_id"_a)
        .def("getRelativeLinkTransform", &tsg::StateSolver::getRelativeLinkTransform,
             "from_link_id"_a, "to_link_id"_a)
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
             "name"_a, "limit"_a);

    // ========== KDLStateSolver (concrete) ==========
    nb::class_<tsg::KDLStateSolver, tsg::StateSolver>(m, "KDLStateSolver")
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
