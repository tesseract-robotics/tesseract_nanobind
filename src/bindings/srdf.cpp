/**
 * @file srdf.cpp
 * @brief nanobind bindings for tesseract_srdf
 */

#include "tesseract_nb.h"

#include <tesseract/srdf/srdf_model.h>
#include <tesseract/srdf/kinematics_information.h>
#include <tesseract/srdf/utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/calibration_info.h>
#include <tesseract/common/collision_margin_data.h>

namespace ts = tesseract::srdf;

NB_MODULE(srdf, m) {
    m.doc() = "tesseract_srdf Python bindings";

    // Import common module for the Isometry3d type (else stubs quote the C++ name, which is compiler-specific);
    // LinkId / JointId / LinkIdPair are registered there too
    nb::module_::import_("tesseract_robotics.common._common");
    // SceneGraph in initFile / initString / processSRDFAllowedCollisions (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.scene_graph._scene_graph");

    // KinematicsInformation
    nb::class_<ts::KinematicsInformation>(m, "KinematicsInformation")
        .def(nb::init<>())
        .def_rw("group_names", &ts::KinematicsInformation::group_names, "The names of every group (chain, joint and link groups).")
        .def_rw("chain_groups", &ts::KinematicsInformation::chain_groups, nb::rv_policy::copy, "Chain groups: group name -> list of (base link, tip link).")
        .def_rw("joint_groups", &ts::KinematicsInformation::joint_groups, nb::rv_policy::copy, "Joint groups: group name -> joint names.")
        .def_rw("link_groups", &ts::KinematicsInformation::link_groups, nb::rv_policy::copy, "Link groups: group name -> link names.")
        // kinematics_plugin_info: tesseract::common::KinematicsPluginInfo, registered by the
        // tesseract_common module. Carries the fwd/inv IK-solver plugin config per group so a
        // group added via AddKinematicsInformationCommand resolves through getKinematicGroup().
        .def_rw("kinematics_plugin_info", &ts::KinematicsInformation::kinematics_plugin_info, "The forward / inverse kinematics solver plugins per group.")
        .def_rw("group_states", &ts::KinematicsInformation::group_states, nb::rv_policy::copy, "Named joint states per group: group name -> state name -> joint values.")
        .def_rw("group_tcps", &ts::KinematicsInformation::group_tcps, nb::rv_policy::copy, "Named TCP offsets per group: group name -> TCP link name -> Isometry3d.")
        .def("insert", &ts::KinematicsInformation::insert, "other"_a, "Merge `other` into this; on a name both define, `other`'s entry wins.")
        .def("clear", &ts::KinematicsInformation::clear, "Remove every group, state, TCP and plugin entry.")
        .def("hasGroup", &ts::KinematicsInformation::hasGroup, "group_name"_a, "True if `group_name` is a chain, joint or link group.")
        .def("addChainGroup", &ts::KinematicsInformation::addChainGroup, "group_name"_a, "chain_group"_a, "Add (or replace) the chain group `group_name`.")
        .def("removeChainGroup", &ts::KinematicsInformation::removeChainGroup, "group_name"_a, "Remove the chain group `group_name`, if present.")
        .def("hasChainGroup", &ts::KinematicsInformation::hasChainGroup, "group_name"_a, "True if `group_name` is a chain group.")
        .def("addJointGroup", &ts::KinematicsInformation::addJointGroup, "group_name"_a, "joint_group"_a, "Add (or replace) the joint group `group_name`.")
        .def("removeJointGroup", &ts::KinematicsInformation::removeJointGroup, "group_name"_a, "Remove the joint group `group_name`, if present.")
        .def("hasJointGroup", &ts::KinematicsInformation::hasJointGroup, "group_name"_a, "True if `group_name` is a joint group.")
        .def("addLinkGroup", &ts::KinematicsInformation::addLinkGroup, "group_name"_a, "link_group"_a, "Add (or replace) the link group `group_name`.")
        .def("removeLinkGroup", &ts::KinematicsInformation::removeLinkGroup, "group_name"_a, "Remove the link group `group_name`, if present.")
        .def("hasLinkGroup", &ts::KinematicsInformation::hasLinkGroup, "group_name"_a, "True if `group_name` is a link group.")
        .def("addGroupJointState", &ts::KinematicsInformation::addGroupJointState,
             "group_name"_a, "state_name"_a, "joint_state"_a, "Add (or replace) the named joint state `state_name` of `group_name`.")
        .def("removeGroupJointState", &ts::KinematicsInformation::removeGroupJointState,
             "group_name"_a, "state_name"_a, "Remove the named joint state, if present.")
        .def("hasGroupJointState", &ts::KinematicsInformation::hasGroupJointState,
             "group_name"_a, "state_name"_a, "True if `group_name` has the named joint state `state_name`.")
        .def("addGroupTCP", &ts::KinematicsInformation::addGroupTCP,
             "group_name"_a, "tcp_name"_a, "tcp"_a, "Add (or replace) the named TCP offset `tcp_name` of `group_name`.")
        .def("removeGroupTCP", &ts::KinematicsInformation::removeGroupTCP,
             "group_name"_a, "tcp_name"_a, "Remove the named TCP offset, if present.")
        .def("hasGroupTCP", &ts::KinematicsInformation::hasGroupTCP,
             "group_name"_a, "tcp_name"_a, "True if `group_name` has the named TCP offset `tcp_name`.")
        .def("__eq__", &ts::KinematicsInformation::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &ts::KinematicsInformation::operator!=, TNB_NE_ATTRS);

    // SRDFModel
    nb::class_<ts::SRDFModel>(m, "SRDFModel")
        .def(nb::init<>())
        .def("initFile", &ts::SRDFModel::initFile,
             "scene_graph"_a, "filename"_a, "locator"_a, "Parse the SRDF file `filename` against `scene_graph`, resolving URLs with `locator`; raises on invalid SRDF.")
        .def("initString", &ts::SRDFModel::initString,
             "scene_graph"_a, "xmlstring"_a, "locator"_a, "Parse SRDF text against `scene_graph`, resolving URLs with `locator`; raises on invalid SRDF.")
        .def("saveToFile", &ts::SRDFModel::saveToFile, "file_path"_a, "Write the model as SRDF XML to `file_path`; False if the file cannot be written.")
        .def("clear", &ts::SRDFModel::clear, "Reset the model to empty.")
        .def_rw("name", &ts::SRDFModel::name, "The robot name.")
        .def_rw("version", &ts::SRDFModel::version, "The SRDF format version, (major, minor, patch).")
        .def_rw("kinematics_information", &ts::SRDFModel::kinematics_information, "Groups, group states, TCPs and kinematics plugin configuration.")
        .def_rw("acm", &ts::SRDFModel::acm, "The allowed collision matrix (disable_collisions entries).")
        .def_rw("collision_margin_data", &ts::SRDFModel::collision_margin_data, "Collision margins (default and per link pair).")
        .def_rw("contact_managers_plugin_info", &ts::SRDFModel::contact_managers_plugin_info, "The contact manager plugin configuration.")
        .def_rw("calibration_info", &ts::SRDFModel::calibration_info, "Joint origin calibrations (calibration_config).")
        .def("__eq__", &ts::SRDFModel::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &ts::SRDFModel::operator!=, TNB_NE_ATTRS)
        .def("__repr__", [](const ts::SRDFModel& self) {
            return "SRDFModel('" + self.name + "')";
        });

    // Utility functions
    m.def("processSRDFAllowedCollisions", &ts::processSRDFAllowedCollisions,
          "scene_graph"_a, "srdf_model"_a,
          "Process SRDF allowed collisions and add to scene graph ACM");
}
