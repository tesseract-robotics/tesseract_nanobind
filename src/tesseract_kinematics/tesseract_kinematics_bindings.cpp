/**
 * @file tesseract_kinematics_bindings.cpp
 * @brief nanobind bindings for tesseract_kinematics
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>

// tesseract_state_solver - need full definition for JointGroup
#include <tesseract/state_solver/state_solver.h>

// tesseract_kinematics core
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/types.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/utils.h>

// tesseract_scene_graph
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

// tesseract_common
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/plugin_info.h>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace tk = tesseract::kinematics;
namespace tcommon = tesseract::common;
namespace tsg = tesseract::scene_graph;

// Make KinGroupIKInputs opaque so we can bind it as a class
NB_MAKE_OPAQUE(tk::KinGroupIKInputs)

// Removing a group's last solver makes upstream read (and, when it is the default, write) through
// the map iterator it has just erased (kinematics_plugin_factory.cpp:150-154 and :209-214 @ 0.35.0).
// The binding refuses that call instead of reaching it; drop the guard once a fixed tesseract is
// the minimum version (tesseract-robotics/tesseract#1381).
struct KinematicsPluginRemovalError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

namespace {

using PluginGroups = std::map<std::string, tcommon::PluginInfoContainer>;

// Upstream throws std::runtime_error for an unknown group or solver; a name lookup that misses is
// a KeyError here (the ContactManagersPluginFactory rule, gh-170).
const tcommon::PluginInfoContainer& require_solver(const PluginGroups& groups, const char* kind,
                                                   const std::string& group_name,
                                                   const std::string& solver_name) {
    auto group_it = groups.find(group_name);
    if (group_it == groups.end() || group_it->second.plugins.count(solver_name) == 0)
        throw nb::key_error(("no " + std::string(kind) + " kin solver '" + solver_name + "' for group '" +
                             group_name + "'").c_str());
    return group_it->second;
}

void require_not_last_solver(const tcommon::PluginInfoContainer& group, const std::string& group_name) {
    if (group.plugins.size() == 1)
        throw KinematicsPluginRemovalError("removing the last solver of group '" + group_name +
                                           "' is unsafe in tesseract 0.35.0 (kinematics_plugin_factory.cpp:150-154)");
}

// The link_point and base_link overloads of JointGroup::calcJacobian read link transforms with
// operator[] behind an assert (joint_group.cpp:197-198, :234-235 @ 0.35.0): in a release build an
// unknown name yields an uninitialized Isometry3d. Every overload checks its link names first.
void require_link(const tk::JointGroup& group, const std::string& link_name) {
    if (!group.hasLinkName(link_name))
        throw nb::key_error(("no link '" + link_name + "' in joint group '" + group.getName() + "'").c_str());
}

}  // namespace

NB_MODULE(_tesseract_kinematics, m) {
    m.doc() = "tesseract_kinematics Python bindings";

    // ResourceLocator, Isometry3d live in tesseract_common
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    // SceneGraph and SceneState in JointGroup / createFwdKin / createInvKin (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph");
    nb::module_::import_("tesseract_robotics.tesseract_state_solver._tesseract_state_solver");

    // ========== URParameters ==========
    nb::class_<tk::URParameters>(m, "URParameters")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double, double>(),
             "d1"_a, "a2"_a, "a3"_a, "d4"_a, "d5"_a, "d6"_a)
        .def_rw("d1", &tk::URParameters::d1)
        .def_rw("a2", &tk::URParameters::a2)
        .def_rw("a3", &tk::URParameters::a3)
        .def_rw("d4", &tk::URParameters::d4)
        .def_rw("d5", &tk::URParameters::d5)
        .def_rw("d6", &tk::URParameters::d6);

    // UR parameter constants
    m.attr("UR10Parameters") = tk::UR10Parameters;
    m.attr("UR5Parameters") = tk::UR5Parameters;
    m.attr("UR3Parameters") = tk::UR3Parameters;
    m.attr("UR10eParameters") = tk::UR10eParameters;
    m.attr("UR5eParameters") = tk::UR5eParameters;
    m.attr("UR3eParameters") = tk::UR3eParameters;

    // ========== KinGroupIKInput ==========
    nb::class_<tk::KinGroupIKInput>(m, "KinGroupIKInput")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&, std::string, std::string>(),
             "pose"_a, "working_frame"_a, "tip_link_name"_a)
        .def_rw("pose", &tk::KinGroupIKInput::pose)
        .def_rw("working_frame", &tk::KinGroupIKInput::working_frame)
        .def_rw("tip_link_name", &tk::KinGroupIKInput::tip_link_name);

    // ========== KinGroupIKInputs (vector of KinGroupIKInput) ==========
    nb::class_<tk::KinGroupIKInputs>(m, "KinGroupIKInputs")
        .def(nb::init<>())
        .def("__len__", [](const tk::KinGroupIKInputs& self) { return self.size(); })
        .def("__getitem__", [](const tk::KinGroupIKInputs& self, size_t i) -> tk::KinGroupIKInput {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            return self[i];
        })
        .def("append", [](tk::KinGroupIKInputs& self, const tk::KinGroupIKInput& v) { self.push_back(v); })
        .def("clear", [](tk::KinGroupIKInputs& self) { self.clear(); });

    // ========== ForwardKinematics (abstract) ==========
    nb::class_<tk::ForwardKinematics>(m, "ForwardKinematics")
        .def("calcFwdKin", [](const tk::ForwardKinematics& self,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
            auto result = self.calcFwdKin(joint_angles);
            // Convert TransformMap to std::map for Python
            std::map<std::string, Eigen::Isometry3d> py_result;
            for (const auto& p : result) {
                py_result[p.first] = p.second;
            }
            return py_result;
        }, "joint_angles"_a)
        // Note: In 0.33, calcJacobian has a non-virtual wrapper returning MatrixXd
        .def("calcJacobian", [](const tk::ForwardKinematics& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name) {
            return self.calcJacobian(joint_angles, link_name);
        }, "joint_angles"_a, "link_name"_a)
        .def("getBaseLinkName", &tk::ForwardKinematics::getBaseLinkName)
        .def("getJointNames", &tk::ForwardKinematics::getJointNames)
        .def("getTipLinkNames", &tk::ForwardKinematics::getTipLinkNames)
        .def("numJoints", &tk::ForwardKinematics::numJoints)
        .def("getSolverName", &tk::ForwardKinematics::getSolverName)
        .def("clone", [](const tk::ForwardKinematics& self) { return self.clone(); });

    // ========== InverseKinematics (abstract) ==========
    nb::class_<tk::InverseKinematics>(m, "InverseKinematics")
        .def("calcInvKin", [](const tk::InverseKinematics& self,
                              const std::map<std::string, Eigen::Isometry3d>& tip_link_poses,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            // Convert std::map to TransformMap
            tesseract::common::TransformMap poses;
            for (const auto& p : tip_link_poses) {
                poses[p.first] = p.second;
            }
            return self.calcInvKin(poses, seed);
        }, "tip_link_poses"_a, "seed"_a)
        .def("getJointNames", &tk::InverseKinematics::getJointNames)
        .def("numJoints", &tk::InverseKinematics::numJoints)
        .def("getBaseLinkName", &tk::InverseKinematics::getBaseLinkName)
        .def("getWorkingFrame", &tk::InverseKinematics::getWorkingFrame)
        .def("getTipLinkNames", &tk::InverseKinematics::getTipLinkNames)
        .def("getSolverName", &tk::InverseKinematics::getSolverName)
        .def("clone", [](const tk::InverseKinematics& self) { return self.clone(); });

    // ========== JointGroup ==========
    // One body for calcJacobian(q, link_name, link_point) and its alias calcJacobianWithPoint.
    auto jacobian_at_point = [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name,
                                const Eigen::Vector3d& link_point) {
        require_link(self, link_name);
        return self.calcJacobian(joint_angles, link_name, link_point);
    };
    nb::class_<tk::JointGroup>(m, "JointGroup")
        .def(nb::init<std::string, std::vector<std::string>, const tsg::SceneGraph&, const tsg::SceneState&>(),
             "name"_a, "joint_names"_a, "scene_graph"_a, "scene_state"_a)
        .def("calcFwdKin", [](const tk::JointGroup& self,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
            auto result = self.calcFwdKin(joint_angles);
            std::map<std::string, Eigen::Isometry3d> py_result;
            for (const auto& p : result) {
                py_result[p.first] = p.second;
            }
            return py_result;
        }, "joint_angles"_a)
        // The four C++ overloads in header order (joint_group.h:96, :106, :117, :129). nanobind
        // tells (q, link_name, link_point) from (q, base_link_name, link_name) by the third
        // argument's type. Unknown link names raise KeyError (require_link).
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name) {
            require_link(self, link_name);
            return self.calcJacobian(joint_angles, link_name);
        }, "joint_angles"_a, "link_name"_a)
        .def("calcJacobian", jacobian_at_point, "joint_angles"_a, "link_name"_a, "link_point"_a)
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& base_link_name,
                                const std::string& link_name) {
            require_link(self, base_link_name);
            require_link(self, link_name);
            return self.calcJacobian(joint_angles, base_link_name, link_name);
        }, "joint_angles"_a, "base_link_name"_a, "link_name"_a)
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& base_link_name,
                                const std::string& link_name,
                                const Eigen::Vector3d& link_point) {
            require_link(self, base_link_name);
            require_link(self, link_name);
            return self.calcJacobian(joint_angles, base_link_name, link_name, link_point);
        }, "joint_angles"_a, "base_link_name"_a, "link_name"_a, "link_point"_a)
        .def("calcJacobianWithPoint", jacobian_at_point, "joint_angles"_a, "link_name"_a, "link_point"_a,
             "Alias of `calcJacobian(joint_angles, link_name, link_point)`, the native form.")
        .def("getJointNames", &tk::JointGroup::getJointNames)
        .def("getLinkNames", &tk::JointGroup::getLinkNames)
        .def("getActiveLinkNames", &tk::JointGroup::getActiveLinkNames)
        .def("getStaticLinkNames", &tk::JointGroup::getStaticLinkNames)
        .def("isActiveLinkName", &tk::JointGroup::isActiveLinkName, "link_name"_a)
        .def("hasLinkName", &tk::JointGroup::hasLinkName, "link_name"_a)
        .def("getLimits", &tk::JointGroup::getLimits)
        .def("setLimits", &tk::JointGroup::setLimits, "limits"_a)
        .def("getRedundancyCapableJointIndices", &tk::JointGroup::getRedundancyCapableJointIndices)
        .def("numJoints", &tk::JointGroup::numJoints)
        .def("getBaseLinkName", &tk::JointGroup::getBaseLinkName)
        .def("getName", &tk::JointGroup::getName)
        .def("checkJoints", &tk::JointGroup::checkJoints, "vec"_a);

    // ========== KinematicGroup (extends JointGroup) ==========
    // One body for calcInvKin(list[KinGroupIKInput], seed) and its alias calcInvKinMultiple:
    // KinGroupIKInputs is opaque (NB_MAKE_OPAQUE above), so a list does not convert to it.
    auto inv_kin_from_list = [](const tk::KinematicGroup& self,
                                const std::vector<tk::KinGroupIKInput>& tip_link_poses,
                                const Eigen::Ref<const Eigen::VectorXd>& seed) {
        tk::KinGroupIKInputs inputs(tip_link_poses.begin(), tip_link_poses.end());
        return self.calcInvKin(inputs, seed);
    };
    nb::class_<tk::KinematicGroup, tk::JointGroup>(m, "KinematicGroup")
        // Python has no move: the group takes a clone of inv_kin, so the caller's solver stays
        // usable. The clone runs code from the plugin library of the factory that made inv_kin;
        // keep_alive<1, 4> keeps inv_kin, and through createInvKin's keep_alive<0, 1> that
        // factory, alive as long as the group (gh-72).
        .def("__init__", [](tk::KinematicGroup* self,
                            std::string name,
                            std::vector<std::string> joint_names,
                            const tk::InverseKinematics& inv_kin,
                            const tsg::SceneGraph& scene_graph,
                            const tsg::SceneState& scene_state) {
            new (self) tk::KinematicGroup(std::move(name), std::move(joint_names), inv_kin.clone(),
                                          scene_graph, scene_state);
        }, "name"_a, "joint_names"_a, "inv_kin"_a, "scene_graph"_a, "scene_state"_a, nb::keep_alive<1, 4>())
        .def("calcInvKin", [](const tk::KinematicGroup& self,
                              const tk::KinGroupIKInputs& tip_link_poses,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            return self.calcInvKin(tip_link_poses, seed);
        }, "tip_link_poses"_a, "seed"_a)
        .def("calcInvKin", [](const tk::KinematicGroup& self,
                              const tk::KinGroupIKInput& tip_link_pose,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            return self.calcInvKin(tip_link_pose, seed);
        }, "tip_link_pose"_a, "seed"_a)
        .def("calcInvKin", inv_kin_from_list, "tip_link_poses"_a, "seed"_a)
        .def("calcInvKinMultiple", inv_kin_from_list, "tip_link_poses"_a, "seed"_a,
             "Alias of `calcInvKin(tip_link_poses, seed)` with a list, the native form.")
        .def("getAllValidWorkingFrames", &tk::KinematicGroup::getAllValidWorkingFrames)
        .def("getAllPossibleTipLinkNames", &tk::KinematicGroup::getAllPossibleTipLinkNames)
        .def("getInverseKinematics", &tk::KinematicGroup::getInverseKinematics,
             nb::rv_policy::reference_internal);

    // ========== KinematicsPluginFactory ==========
    nb::class_<tk::KinematicsPluginFactory>(m, "KinematicsPluginFactory")
        .def(nb::init<>())
        // Constructor with config file path and locator. StrictPath (tesseract_nb.h):
        // a `str` is always YAML content (the overload below), never a path.
        .def("__init__", [](tk::KinematicsPluginFactory* self,
                            const tesseract_nb::StrictPath& config_path,
                            const tcommon::ResourceLocator& locator) {
            new (self) tk::KinematicsPluginFactory(config_path.value, locator);
        }, "config_path"_a, "locator"_a)
        // Constructor with string and locator
        .def("__init__", [](tk::KinematicsPluginFactory* self,
                            const std::string& config,
                            const tcommon::ResourceLocator& locator) {
            new (self) tk::KinematicsPluginFactory(config, locator);
        }, "config"_a, "locator"_a)
        .def("addSearchPath", &tk::KinematicsPluginFactory::addSearchPath, "path"_a)
        .def("getSearchPaths", &tk::KinematicsPluginFactory::getSearchPaths)
        .def("addSearchLibrary", &tk::KinematicsPluginFactory::addSearchLibrary, "library_name"_a)
        .def("getSearchLibraries", &tk::KinematicsPluginFactory::getSearchLibraries)
        .def("addFwdKinPlugin", &tk::KinematicsPluginFactory::addFwdKinPlugin,
             "group_name"_a, "solver_name"_a, "plugin_info"_a)
        .def("getFwdKinPlugins", &tk::KinematicsPluginFactory::getFwdKinPlugins)
        .def("removeFwdKinPlugin", [](tk::KinematicsPluginFactory& self, const std::string& group_name,
                                      const std::string& solver_name) {
            const PluginGroups groups = self.getFwdKinPlugins();
            require_not_last_solver(require_solver(groups, "fwd", group_name, solver_name), group_name);
            self.removeFwdKinPlugin(group_name, solver_name);
        }, "group_name"_a, "solver_name"_a,
           "Remove a forward kinematics solver from a group.\n\n"
           "Raises:\n"
           "    KeyError: the group or the solver is unknown.\n"
           "    KinematicsPluginRemovalError: it is the group's last solver.")
        .def("setDefaultFwdKinPlugin", [](tk::KinematicsPluginFactory& self, const std::string& group_name,
                                          const std::string& solver_name) {
            require_solver(self.getFwdKinPlugins(), "fwd", group_name, solver_name);
            self.setDefaultFwdKinPlugin(group_name, solver_name);
        }, "group_name"_a, "solver_name"_a,
           "Set a group's default forward kinematics solver.\n\n"
           "Raises:\n"
           "    KeyError: the group or the solver is unknown.")
        .def("getDefaultFwdKinPlugin", &tk::KinematicsPluginFactory::getDefaultFwdKinPlugin, "group_name"_a)
        .def("addInvKinPlugin", &tk::KinematicsPluginFactory::addInvKinPlugin,
             "group_name"_a, "solver_name"_a, "plugin_info"_a)
        .def("getInvKinPlugins", &tk::KinematicsPluginFactory::getInvKinPlugins)
        .def("removeInvKinPlugin", [](tk::KinematicsPluginFactory& self, const std::string& group_name,
                                      const std::string& solver_name) {
            const PluginGroups groups = self.getInvKinPlugins();
            require_not_last_solver(require_solver(groups, "inv", group_name, solver_name), group_name);
            self.removeInvKinPlugin(group_name, solver_name);
        }, "group_name"_a, "solver_name"_a,
           "Remove an inverse kinematics solver from a group.\n\n"
           "Raises:\n"
           "    KeyError: the group or the solver is unknown.\n"
           "    KinematicsPluginRemovalError: it is the group's last solver.")
        .def("setDefaultInvKinPlugin", [](tk::KinematicsPluginFactory& self, const std::string& group_name,
                                          const std::string& solver_name) {
            require_solver(self.getInvKinPlugins(), "inv", group_name, solver_name);
            self.setDefaultInvKinPlugin(group_name, solver_name);
        }, "group_name"_a, "solver_name"_a,
           "Set a group's default inverse kinematics solver.\n\n"
           "Raises:\n"
           "    KeyError: the group or the solver is unknown.")
        .def("getDefaultInvKinPlugin", &tk::KinematicsPluginFactory::getDefaultInvKinPlugin, "group_name"_a)
        // Create kinematics solvers.
        // keep_alive: the returned solver is instantiated from a plugin whose code
        // lives in a dylib owned by this factory (self), and it retains references
        // into the scene graph/state arguments. If Python frees the factory (the
        // loader dlcloses the plugin dylib) or a scene argument before the solver,
        // the solver's destructor virtual-calls into unmapped pages / dereferences
        // freed memory -> SIGSEGV at teardown (same failure mode as gh-72 for
        // Environment groups). Nurse 0 = returned solver; patients 1 = self/factory
        // (the plugin dylib owner), 4 = scene_graph, 5 = scene_state.
        .def("createFwdKin", [](const tk::KinematicsPluginFactory& self,
                                const std::string& group_name,
                                const std::string& solver_name,
                                const tsg::SceneGraph& scene_graph,
                                const tsg::SceneState& scene_state) {
            return self.createFwdKin(group_name, solver_name, scene_graph, scene_state);
        }, "group_name"_a, "solver_name"_a, "scene_graph"_a, "scene_state"_a,
           nb::keep_alive<0, 1>(), nb::keep_alive<0, 4>(), nb::keep_alive<0, 5>())
        .def("createInvKin", [](const tk::KinematicsPluginFactory& self,
                                const std::string& group_name,
                                const std::string& solver_name,
                                const tsg::SceneGraph& scene_graph,
                                const tsg::SceneState& scene_state) {
            return self.createInvKin(group_name, solver_name, scene_graph, scene_state);
        }, "group_name"_a, "solver_name"_a, "scene_graph"_a, "scene_state"_a,
           nb::keep_alive<0, 1>(), nb::keep_alive<0, 4>(), nb::keep_alive<0, 5>())
        // From an explicit PluginInfo: the same plugin-library and scene ties as above.
        .def("createFwdKin", [](const tk::KinematicsPluginFactory& self,
                                const std::string& solver_name,
                                const tcommon::PluginInfo& plugin_info,
                                const tsg::SceneGraph& scene_graph,
                                const tsg::SceneState& scene_state) {
            return self.createFwdKin(solver_name, plugin_info, scene_graph, scene_state);
        }, "solver_name"_a, "plugin_info"_a, "scene_graph"_a, "scene_state"_a,
           nb::keep_alive<0, 1>(), nb::keep_alive<0, 4>(), nb::keep_alive<0, 5>())
        .def("createInvKin", [](const tk::KinematicsPluginFactory& self,
                                const std::string& solver_name,
                                const tcommon::PluginInfo& plugin_info,
                                const tsg::SceneGraph& scene_graph,
                                const tsg::SceneState& scene_state) {
            return self.createInvKin(solver_name, plugin_info, scene_graph, scene_state);
        }, "solver_name"_a, "plugin_info"_a, "scene_graph"_a, "scene_state"_a,
           nb::keep_alive<0, 1>(), nb::keep_alive<0, 4>(), nb::keep_alive<0, 5>())
        // Same YAML as upstream saveConfig, which ignores a failed ofstream; here a failed open or
        // write raises OSError (FileNotFoundError for a missing directory).
        .def("saveConfig", [](const tk::KinematicsPluginFactory& self, const std::filesystem::path& file_path) {
            errno = 0;
            std::ofstream fout(file_path);
            if (fout) fout << self.getConfig();
            if (fout) fout.close();
            if (!fout) {
                PyErr_SetFromErrnoWithFilename(PyExc_OSError, file_path.string().c_str());
                throw nb::python_error();
            }
        }, "file_path"_a)
        // YAML::Node has no caster: the YAML document as str, like PluginInfo.config.
        .def("getConfig", [](const tk::KinematicsPluginFactory& self) {
            YAML::Emitter out;
            out << self.getConfig();
            return std::string(out.c_str());
        }, "The factory configuration as a YAML document string.");

    nb::exception<KinematicsPluginRemovalError>(m, "KinematicsPluginRemovalError", PyExc_RuntimeError)
        .attr("__doc__") =
        "Refused to remove a group's last kinematics solver: tesseract 0.35.0 then reads and writes "
        "through an erased map iterator (kinematics_plugin_factory.cpp:150-154, tesseract-robotics/tesseract#1381).";

    // ========== Utility functions ==========
    m.def("getRedundantSolutions", [](const Eigen::Ref<const Eigen::VectorXd>& sol,
                                       const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                                       const std::vector<Eigen::Index>& redundancy_capable_joints) {
        return tk::getRedundantSolutions<double>(sol, limits, redundancy_capable_joints);
    }, "sol"_a, "limits"_a, "redundancy_capable_joints"_a,
    "Get redundant solutions for a joint configuration by adding +/- 2*pi to redundancy capable joints");
}
