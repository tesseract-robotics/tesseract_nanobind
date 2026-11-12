/**
 * @file kinematics.cpp
 * @brief nanobind bindings for tesseract_kinematics
 */

#include "tesseract_nb.h"
#include <nanobind/stl/unordered_map.h>
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
#include <tesseract/kinematics/validate.h>

// tesseract_scene_graph
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

// tesseract_common
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/plugin_info.h>

// Plugin anchors: the factory libraries are linked into this module, so the wheel repair tools
// bundle them; the anchors register the bundled copies with the plugin loader at import
#include <tesseract/kinematics/rep_factory.h>
#include <tesseract/kinematics/rop_factory.h>
#include <tesseract/kinematics/kdl/kdl_factories.h>
#include <tesseract/kinematics/opw/opw_factory.h>
#include <tesseract/kinematics/ur/ur_factory.h>
#include <boost_plugin_loader/utils.h>
#include <Eigen/SVD>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
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

// A jacobian with no rows or no columns has no singular values. isNearSingularity reads the
// smallest one regardless (utils.cpp:224), and calcManipulability segfaults on a 6 x 0 jacobian
// (measured at tesseract 4cf644d7).
struct EmptyJacobianError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// JacobiSVD flags a non-finite input and rank() then reads 0, so upstream's rule returns the
// singular answer (volume 0.0, near a singularity): a blown-up FK chain would read as a wrist
// singularity.
struct NonFiniteJacobianError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
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
void require_link(const tk::JointGroup& group, const tcommon::LinkId& link_id) {
    if (!group.hasLinkId(link_id))
        throw nb::key_error(("no link '" + link_id.name() + "' in joint group '" + group.getName() + "'").c_str());
}

// Upstream default arguments, which C++ exposes as no symbol, restated for the keywords; they
// must not drift from the headers.
// [smallest singular value of the Jacobian; its rows mix m/rad and 1/rad] isNearSingularity's
// threshold default (utils.h:129).
constexpr double SINGULARITY_THRESHOLD_DEFAULT = 0.01;
// [m for the translation distance, rad for the angle distance] checkKinematics' tol default
// (validate.h:38); the one value bounds both.
constexpr double CHECK_KINEMATICS_TOL_DEFAULT = 1e-3;

// numericalJacobian reads poses with operator[] (an unknown link inserts an uninitialized
// Isometry3d) and writes jacobian(0..5, i) for every joint value without sizing it (utils.cpp:36-104).
void require_joint_count(Eigen::Index n_values, Eigen::Index n_joints) {
    if (n_values != n_joints)
        throw nb::value_error(("got " + std::to_string(n_values) + " joint values, expected " +
                               std::to_string(n_joints)).c_str());
}

void require_tip_link(const tk::ForwardKinematics& kin, const tcommon::LinkId& link_id) {
    const auto tips = kin.getTipLinkIds();
    if (std::find(tips.begin(), tips.end(), link_id) == tips.end())
        throw nb::key_error(("no tip link '" + link_id.name() + "' in forward kinematics '" + kin.getSolverName() + "'").c_str());
}

// The harmonizers index qs (and position_limits' rows) with every redundancy_capable_joints entry
// unchecked (utils.h:355-359, :375-381); check them all before any write.
void require_indices(const std::vector<Eigen::Index>& indices, Eigen::Index size, const char* what) {
    for (Eigen::Index i : indices)
        if (i < 0 || i >= size)
            throw nb::index_error(("joint index " + std::to_string(i) + " is outside " + what + " (size " +
                                   std::to_string(size) + ")").c_str());
}

// Rows of a geometric jacobian: three linear over three angular.
constexpr Eigen::Index JACOBIAN_ROWS = 6;

void require_columns(Eigen::Index cols, const char* caller) {
    if (cols == 0)
        throw EmptyJacobianError(std::string(caller) + ": the jacobian has no columns");
}

// `which` names the jacobian in the message: "the jacobian", or "jacobian 3" of a stack.
template <typename Derived>
void require_finite(const Eigen::MatrixBase<Derived>& jacobian, const char* caller, const std::string& which) {
    if (jacobian.allFinite())
        return;
    for (Eigen::Index row = 0; row < jacobian.rows(); ++row)
        for (Eigen::Index col = 0; col < jacobian.cols(); ++col)
            if (!std::isfinite(jacobian(row, col))) {
                std::ostringstream ss;
                ss << caller << ": " << which << " is not finite (entry [" << row << ", " << col
                   << "] = " << jacobian(row, col) << ")";
                throw NonFiniteJacobianError(ss.str());
            }
}

// calcManipulability's m.volume from the SVD of the whole jacobian, by upstream's rule
// (calcEllipsoids, utils.cpp:240-250, tesseract#1396): the product of the singular values in
// increasing order at full row rank, exactly 0.0 below it. Pinned bitwise by
// test_calc_manipulability_volume_is_the_full_calls_volume_across_the_rank_threshold.
double svd_volume(const Eigen::JacobiSVD<Eigen::MatrixXd>& svd) {
    return svd.rank() < JACOBIAN_ROWS ? 0.0 : svd.singularValues().reverse().prod();
}

std::string repr_ellipsoid(const tk::ManipulabilityEllipsoid& e) {
    std::ostringstream out;
    const Eigen::IOFormat row(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    out << "ManipulabilityEllipsoid(eigen_values=" << e.eigen_values.transpose().format(row)
        << ", measure=" << e.measure << ", condition=" << e.condition << ", volume=" << e.volume << ")";
    return out.str();
}

}  // namespace

NB_MODULE(kinematics, m) {
    boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::REPInvKinFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
    boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::ROPInvKinFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
    boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::KDLFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
    boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::OPWFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");
    boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::kinematics::URFactoriesAnchor(), "TESSERACT_KINEMATICS_PLUGINS");

    m.doc() = "tesseract_kinematics Python bindings";

    // ResourceLocator, Isometry3d, LinkId / JointId / LinkIdPair live in tesseract_common
    nb::module_::import_("tesseract_robotics.common._common");
    // SceneGraph and SceneState in JointGroup / createFwdKin / createInvKin (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.scene_graph._scene_graph");
    nb::module_::import_("tesseract_robotics.state_solver");

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
        .def(nb::init<const Eigen::Isometry3d&, tcommon::LinkId, tcommon::LinkId>(),
             "pose"_a, "working_frame"_a, "tip_link_id"_a)
        .def_rw("pose", &tk::KinGroupIKInput::pose)
        .def_rw("working_frame", &tk::KinGroupIKInput::working_frame, nb::rv_policy::copy)
        .def_rw("tip_link_id", &tk::KinGroupIKInput::tip_link_id, nb::rv_policy::copy);

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
            return self.calcFwdKin(joint_angles);  // dict[str, Isometry3d], keyed by link name
        }, "joint_angles"_a)
        .def("calcJacobian", [](const tk::ForwardKinematics& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const tcommon::LinkId& link_id) {
            return self.calcJacobian(joint_angles, link_id);
        }, "joint_angles"_a, "link_id"_a)
        .def("getBaseLinkId", &tk::ForwardKinematics::getBaseLinkId)
        .def("getJointIds", &tk::ForwardKinematics::getJointIds)
        .def("getTipLinkIds", &tk::ForwardKinematics::getTipLinkIds)
        .def("numJoints", &tk::ForwardKinematics::numJoints)
        .def("getSolverName", &tk::ForwardKinematics::getSolverName)
        .def("clone", [](const tk::ForwardKinematics& self) { return self.clone(); },
             // gh-72: a clone runs the plugin library's code (its vtable lives there), so it keeps
             // the original, and through it whatever owns that library, alive.
             nb::keep_alive<0, 1>(), "An independent copy; it keeps this object alive (gh-72).");

    // ========== InverseKinematics (abstract) ==========
    // calcInvKin releases the GIL here and on KinematicGroup: numerical solvers such as TRAC-IK
    // run until their timeout (scripts/nogil_entry_points.yaml).
    nb::class_<tk::InverseKinematics>(m, "InverseKinematics")
        .def("calcInvKin", [](const tk::InverseKinematics& self,
                              const std::unordered_map<tcommon::LinkId, Eigen::Isometry3d>& tip_link_poses,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            const tcommon::LinkIdTransformMap poses(tip_link_poses.begin(), tip_link_poses.end());
            return self.calcInvKin(poses, seed);
        }, "tip_link_poses"_a, "seed"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("getJointIds", &tk::InverseKinematics::getJointIds)
        .def("numJoints", &tk::InverseKinematics::numJoints)
        .def("getBaseLinkId", &tk::InverseKinematics::getBaseLinkId)
        .def("getWorkingFrame", &tk::InverseKinematics::getWorkingFrame)
        .def("getTipLinkIds", &tk::InverseKinematics::getTipLinkIds)
        .def("getSolverName", &tk::InverseKinematics::getSolverName)
        .def("clone", [](const tk::InverseKinematics& self) { return self.clone(); },
             // gh-72: a clone runs the plugin library's code (its vtable lives there), so it keeps
             // the original, and through it whatever owns that library, alive.
             nb::keep_alive<0, 1>(), "An independent copy; it keeps this object alive (gh-72).");

    // ========== JointGroup ==========
    // calcJacobian(q, link_id, link_point), checked.
    auto jacobian_at_point = [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const tcommon::LinkId& link_id,
                                const Eigen::Vector3d& link_point) {
        require_link(self, link_id);
        return self.calcJacobian(joint_angles, link_id, link_point);
    };
    nb::class_<tk::JointGroup>(m, "JointGroup")
        .def(nb::init<std::string, std::vector<tcommon::JointId>, const tsg::SceneGraph&, const tsg::SceneState&>(),
             "name"_a, "joint_ids"_a, "scene_graph"_a, "scene_state"_a)
        .def("calcFwdKin", [](const tk::JointGroup& self,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
            return self.calcFwdKin(joint_angles);  // dict[str, Isometry3d], keyed by link name
        }, "joint_angles"_a)
        // The four C++ overloads in header order (joint_group.h:96, :106, :117, :129). nanobind
        // tells (q, link_id, link_point) from (q, base_link_id, link_id) by the third
        // argument's type. Unknown link names raise KeyError (require_link).
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const tcommon::LinkId& link_id) {
            require_link(self, link_id);
            return self.calcJacobian(joint_angles, link_id);
        }, "joint_angles"_a, "link_id"_a)
        .def("calcJacobian", jacobian_at_point, "joint_angles"_a, "link_id"_a, "link_point"_a)
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const tcommon::LinkId& base_link_id,
                                const tcommon::LinkId& link_id) {
            require_link(self, base_link_id);
            require_link(self, link_id);
            return self.calcJacobian(joint_angles, base_link_id, link_id);
        }, "joint_angles"_a, "base_link_id"_a, "link_id"_a)
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const tcommon::LinkId& base_link_id,
                                const tcommon::LinkId& link_id,
                                const Eigen::Vector3d& link_point) {
            require_link(self, base_link_id);
            require_link(self, link_id);
            return self.calcJacobian(joint_angles, base_link_id, link_id, link_point);
        }, "joint_angles"_a, "base_link_id"_a, "link_id"_a, "link_point"_a)
        .def("getJointIds", &tk::JointGroup::getJointIds)
        .def("getLinkIds", &tk::JointGroup::getLinkIds)
        .def("getActiveLinkIds", &tk::JointGroup::getActiveLinkIds)
        .def("getStaticLinkIds", &tk::JointGroup::getStaticLinkIds)
        .def("isActiveLinkId", &tk::JointGroup::isActiveLinkId, "link_id"_a)
        .def("hasLinkId", &tk::JointGroup::hasLinkId, "link_id"_a)
        .def("getLimits", &tk::JointGroup::getLimits)
        .def("setLimits", &tk::JointGroup::setLimits, "limits"_a)
        .def("getRedundancyCapableJointIndices", &tk::JointGroup::getRedundancyCapableJointIndices)
        .def("numJoints", &tk::JointGroup::numJoints)
        .def("getBaseLinkId", &tk::JointGroup::getBaseLinkId)
        .def("getName", &tk::JointGroup::getName)
        .def("checkJoints", &tk::JointGroup::checkJoints, "vec"_a);

    // ========== KinematicGroup (extends JointGroup) ==========
    // calcInvKin(list[KinGroupIKInput], seed): KinGroupIKInputs is opaque (NB_MAKE_OPAQUE above), so a list does not convert to it.
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
                            std::vector<tcommon::JointId> joint_ids,
                            const tk::InverseKinematics& inv_kin,
                            const tsg::SceneGraph& scene_graph,
                            const tsg::SceneState& scene_state) {
            new (self) tk::KinematicGroup(std::move(name), std::move(joint_ids), inv_kin.clone(),
                                          scene_graph, scene_state);
        }, "name"_a, "joint_ids"_a, "inv_kin"_a, "scene_graph"_a, "scene_state"_a, nb::keep_alive<1, 4>())
        .def("calcInvKin", [](const tk::KinematicGroup& self,
                              const tk::KinGroupIKInputs& tip_link_poses,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            return self.calcInvKin(tip_link_poses, seed);
        }, "tip_link_poses"_a, "seed"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("calcInvKin", [](const tk::KinematicGroup& self,
                              const tk::KinGroupIKInput& tip_link_pose,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            return self.calcInvKin(tip_link_pose, seed);
        }, "tip_link_pose"_a, "seed"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("calcInvKin", inv_kin_from_list, "tip_link_poses"_a, "seed"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("getAllValidWorkingFrames", &tk::KinematicGroup::getAllValidWorkingFrames)
        .def("getAllPossibleTipLinkIds", &tk::KinematicGroup::getAllPossibleTipLinkIds)
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
            require_located_file(config_path.value, locator);
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
    nb::exception<EmptyJacobianError>(m, "EmptyJacobianError", PyExc_ValueError)
        .attr("__doc__") = "A jacobian with no rows or no columns: it has no singular values.";
    nb::exception<NonFiniteJacobianError>(m, "NonFiniteJacobianError", PyExc_ValueError)
        .attr("__doc__") =
        "A jacobian with a NaN or infinite entry: its SVD reads rank 0, which is the answer for a singularity.";

    // ========== Utility functions ==========
    m.def("getRedundantSolutions", [](const Eigen::Ref<const Eigen::VectorXd>& sol,
                                       const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                                       const std::vector<Eigen::Index>& redundancy_capable_joints) {
        return tk::getRedundantSolutions<double>(sol, limits, redundancy_capable_joints);
    }, "sol"_a, "limits"_a, "redundancy_capable_joints"_a,
    "Get redundant solutions for a joint configuration by adding +/- 2*pi to redundancy capable joints");

    // numericalJacobian: the 6 x n out-parameter is returned (out-param rule).
    m.def("numericalJacobian", [](const Eigen::Isometry3d& change_base,
                                  const tk::ForwardKinematics& kin,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  const tcommon::LinkId& link_id,
                                  const Eigen::Ref<const Eigen::Vector3d>& link_point) {
        require_joint_count(joint_values.size(), kin.numJoints());
        require_tip_link(kin, link_id);
        Eigen::MatrixXd jacobian(6, joint_values.size());
        tk::numericalJacobian(jacobian, change_base, kin, joint_values, link_id, link_point);
        return jacobian;
    }, "change_base"_a, "kin"_a, "joint_values"_a, "link_id"_a, "link_point"_a,
    "Finite-difference Jacobian of a tip link (step 1e-8 rad per joint, utils.cpp:44).\n\n"
    "Raises:\n"
    "    ValueError: joint_values does not have kin.numJoints() entries.\n"
    "    KeyError: link_id is not a tip link of kin.");
    m.def("numericalJacobian", [](const Eigen::Isometry3d& change_base,
                                  const tk::JointGroup& joint_group,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  const tcommon::LinkId& link_id,
                                  const Eigen::Ref<const Eigen::Vector3d>& link_point) {
        require_joint_count(joint_values.size(), joint_group.numJoints());
        require_link(joint_group, link_id);
        Eigen::MatrixXd jacobian(6, joint_values.size());
        tk::numericalJacobian(jacobian, change_base, joint_group, joint_values, link_id, link_point);
        return jacobian;
    }, "change_base"_a, "joint_group"_a, "joint_values"_a, "link_id"_a, "link_point"_a,
    "Finite-difference Jacobian of a link of joint_group (step 1e-8 rad per joint, utils.cpp:80).\n\n"
    "Raises:\n"
    "    ValueError: joint_values does not have joint_group.numJoints() entries.\n"
    "    KeyError: link_id is not a link of joint_group.");
    m.def("numericalJacobian", [](const tk::JointGroup& joint_group,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  const tcommon::LinkId& base_link_id,
                                  const Eigen::Isometry3d& base_link_offset,
                                  const tcommon::LinkId& link_id,
                                  const Eigen::Isometry3d& link_offset) {
        require_joint_count(joint_values.size(), joint_group.numJoints());
        require_link(joint_group, base_link_id);
        require_link(joint_group, link_id);
        Eigen::MatrixXd jacobian(6, joint_values.size());
        tk::numericalJacobian(jacobian, joint_group, joint_values, base_link_id, base_link_offset, link_id,
                              link_offset);
        return jacobian;
    }, "joint_group"_a, "joint_values"_a, "base_link_id"_a, "base_link_offset"_a, "link_id"_a, "link_offset"_a,
    "Finite-difference Jacobian of link_id relative to base_link_id, in the base link frame.\n\n"
    "Raises:\n"
    "    ValueError: joint_values does not have joint_group.numJoints() entries.\n"
    "    KeyError: base_link_id or link_id is not a link of joint_group.");

    m.def("isNearSingularity", [](const Eigen::Ref<const Eigen::MatrixXd>& jacobian, double threshold) {
        // The smallest singular value is svd.singularValues().tail(1) (utils.cpp:224): none for an empty matrix.
        if (jacobian.rows() == 0 || jacobian.cols() == 0)
            throw EmptyJacobianError("isNearSingularity: the jacobian is empty (" + std::to_string(jacobian.rows()) +
                                     " x " + std::to_string(jacobian.cols()) + ")");
        require_finite(jacobian, "isNearSingularity", "the jacobian");
        return tk::isNearSingularity(jacobian, threshold);
    }, "jacobian"_a, "threshold"_a = SINGULARITY_THRESHOLD_DEFAULT,
    "True when the smallest singular value of jacobian is below threshold.\n\n"
    "Raises:\n"
    "    EmptyJacobianError: jacobian has no rows or no columns.\n"
    "    NonFiniteJacobianError: jacobian has a NaN or infinite entry.");

    // Results, not inputs: read-only fields.
    nb::class_<tk::ManipulabilityEllipsoid>(m, "ManipulabilityEllipsoid")
        .def(nb::init<>())
        .def_ro("eigen_values", &tk::ManipulabilityEllipsoid::eigen_values)
        .def_ro("measure", &tk::ManipulabilityEllipsoid::measure)
        .def_ro("condition", &tk::ManipulabilityEllipsoid::condition)
        .def_ro("volume", &tk::ManipulabilityEllipsoid::volume)
        .def("__repr__", &repr_ellipsoid);
    nb::class_<tk::Manipulability>(m, "Manipulability")
        .def(nb::init<>())
        .def_ro("m", &tk::Manipulability::m)
        .def_ro("m_linear", &tk::Manipulability::m_linear)
        .def_ro("m_angular", &tk::Manipulability::m_angular)
        .def_ro("f", &tk::Manipulability::f)
        .def_ro("f_linear", &tk::Manipulability::f_linear)
        .def_ro("f_angular", &tk::Manipulability::f_angular)
        .def("__repr__", [](const tk::Manipulability& self) {
            return "Manipulability(m=" + repr_ellipsoid(self.m) + ", m_linear=" + repr_ellipsoid(self.m_linear) +
                   ", m_angular=" + repr_ellipsoid(self.m_angular) + ", f=" + repr_ellipsoid(self.f) +
                   ", f_linear=" + repr_ellipsoid(self.f_linear) + ", f_angular=" + repr_ellipsoid(self.f_angular) +
                   ")";
        });

    // A fixed 6 rows in the Ref type: calcManipulability splits topRows(3) / bottomRows(3)
    // (utils.cpp:279-280), so any other row count raises TypeError at the call boundary.
    using Jacobian = Eigen::Matrix<double, JACOBIAN_ROWS, Eigen::Dynamic>;
    m.def("calcManipulability", [](const Eigen::Ref<const Jacobian>& jacobian) {
        require_columns(jacobian.cols(), "calcManipulability");
        require_finite(jacobian, "calcManipulability", "the jacobian");
        return tk::calcManipulability(jacobian);
    }, "jacobian"_a,
    "Manipulability and force ellipsoids of a 6-row jacobian.\n\n"
    "When an ellipsoid's smallest eigenvalue is ~0 (a singular jacobian), its measure and condition\n"
    "are the largest double (sys.float_info.max), as upstream returns them (utils.cpp:256-261).\n\n"
    "Raises:\n"
    "    EmptyJacobianError: jacobian has no columns.\n"
    "    NonFiniteJacobianError: jacobian has a NaN or infinite entry.");
    // calcManipulability decomposes the jacobian and both 3-row blocks and fills six ellipsoids;
    // these two return the one number of the first decomposition.
    m.def("calcManipulabilityVolume", [](const Eigen::Ref<const Jacobian>& jacobian) {
        require_columns(jacobian.cols(), "calcManipulabilityVolume");
        require_finite(jacobian, "calcManipulabilityVolume", "the jacobian");
        return svd_volume(Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian));
    }, "jacobian"_a,
    "Yoshikawa manipulability of a 6-row jacobian: bitwise calcManipulability(jacobian).m.volume,\n"
    "from one SVD instead of three and no ellipsoids.\n\n"
    "Raises:\n"
    "    EmptyJacobianError: jacobian has no columns.\n"
    "    NonFiniteJacobianError: jacobian has a NaN or infinite entry.");
    // (M, 6, n) as numpy holds a stack: C order, row-major 6 x n per jacobian. An input that is
    // strided, Fortran-ordered or not float64 is converted by nanobind first.
    m.def("calcManipulabilityVolume_each", [](
              const nb::ndarray<nb::numpy, const double, nb::shape<-1, JACOBIAN_ROWS, -1>, nb::c_contig>& jacobians) {
        const auto cols = static_cast<Eigen::Index>(jacobians.shape(2));
        require_columns(cols, "calcManipulabilityVolume_each");
        const auto count = static_cast<Eigen::Index>(jacobians.shape(0));
        using RowMajorJacobian = Eigen::Matrix<double, JACOBIAN_ROWS, Eigen::Dynamic, Eigen::RowMajor>;
        Eigen::VectorXd volumes(count);
        Eigen::MatrixXd jacobian(JACOBIAN_ROWS, cols);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JACOBIAN_ROWS, cols);
        for (Eigen::Index i = 0; i < count; ++i) {
            jacobian = Eigen::Map<const RowMajorJacobian>(jacobians.data() + i * JACOBIAN_ROWS * cols,
                                                          JACOBIAN_ROWS, cols);
            require_finite(jacobian, "calcManipulabilityVolume_each", "jacobian " + std::to_string(i));
            volumes(i) = svd_volume(svd.compute(jacobian));
        }
        return volumes;
    }, "jacobians"_a, nb::call_guard<nb::gil_scoped_release>(),
    "Yoshikawa manipulability of each jacobian of an (M, 6, n) stack, (M,); row i is bitwise\n"
    "calcManipulability(jacobians[i]).m.volume. Releases the GIL.\n\n"
    "Raises:\n"
    "    EmptyJacobianError: n is 0.\n"
    "    NonFiniteJacobianError: a jacobian has a NaN or infinite entry; the message names its index.");

    // In place, as #184: writable Eigen::Ref, so an array nanobind would have to convert (float32,
    // read-only, non-contiguous) raises TypeError instead of being written as a lost copy.
    m.def("harmonizeTowardZero", [](tesseract_nb::InPlace<Eigen::VectorXd> qs,
                                    const std::vector<Eigen::Index>& redundancy_capable_joints) {
        require_indices(redundancy_capable_joints, qs.ref.size(), "qs");
        tk::harmonizeTowardZero<double>(qs.ref, redundancy_capable_joints);
    }, "qs"_a, "redundancy_capable_joints"_a,
    "Wrap qs[i] into [-pi, pi) in place for each i in redundancy_capable_joints.\n\n"
    "Raises:\n"
    "    IndexError: an index is outside qs; qs is left unchanged.");
    m.def("harmonizeTowardMedian", [](tesseract_nb::InPlace<Eigen::VectorXd> qs,
                                      const std::vector<Eigen::Index>& redundancy_capable_joints,
                                      const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& position_limits) {
        require_indices(redundancy_capable_joints, qs.ref.size(), "qs");
        require_indices(redundancy_capable_joints, position_limits.rows(), "position_limits");
        tk::harmonizeTowardMedian<double>(qs.ref, redundancy_capable_joints, position_limits);
    }, "qs"_a, "redundancy_capable_joints"_a, "position_limits"_a,
    "Wrap qs[i] into [median - pi, median + pi) in place, median the midpoint of row i of position_limits.\n\n"
    "Raises:\n"
    "    IndexError: an index is outside qs or position_limits; qs is left unchanged.");

    m.def("isValid", [](const std::array<double, 6>& qs) { return tk::isValid<double>(qs); }, "qs"_a,
          "True when all six values are finite.");

    // ========== Sample grids (types.h, utils.h) ==========
    // The building blocks the REP / ROP / RTP solver plugins use to sample a positioner or tool
    // chain; callable without a plugin.
    nb::class_<tk::PositionerSampleResolution>(m, "PositionerSampleResolution",
        "A positioner joint's sample resolution, optionally with its own sampled range "
        "(by default the joint's limits).")
        .def(nb::init<>())
        .def("__init__",
             [](tk::PositionerSampleResolution* self, std::string name, double value, std::optional<double> min,
                std::optional<double> max) {
                 new (self) tk::PositionerSampleResolution{std::move(name), value, min, max};
             },
             "name"_a, "value"_a, "min"_a = nb::none(), "max"_a = nb::none())
        .def_rw("name", &tk::PositionerSampleResolution::name, "Joint name")
        .def_rw("value", &tk::PositionerSampleResolution::value, "Sample resolution")
        .def_rw("min", &tk::PositionerSampleResolution::min, "Lower bound; None: the joint's lower limit")
        .def_rw("max", &tk::PositionerSampleResolution::max, "Upper bound; None: the joint's upper limit");

    m.def("gatherJointLimits", &tk::gatherJointLimits, "scene_graph"_a, "joint_ids"_a,
          "The joints' position limits as an (N, 2) array [lower, upper]; a continuous joint reports "
          "[-pi, pi].\n\n"
          "Raises:\n"
          "    RuntimeError: a joint is not in scene_graph, or a non-continuous joint has no limits.");
    m.def("buildSampleGrid", &tk::buildSampleGrid, "range"_a, "resolution"_a,
          "Per joint i, ceil((range[i, 1] - range[i, 0]) / resolution[i]) + 1 evenly spaced samples "
          "over range[i] (one array per joint, not their product).\n\n"
          "Raises:\n"
          "    RuntimeError: sizes disagree, a bound is non-finite or inverted, a resolution is not "
          "finite and > 0, or a joint would need more than 10 million samples.");
    m.def("computeChainReachUpperBound", &tk::computeChainReachUpperBound, "scene_graph"_a, "base_link_id"_a,
          "tip_link_id"_a,
          "An upper bound, in metres, on the base-to-tip distance over every joint configuration: the "
          "sum of the joint origin offsets along the path plus each prismatic joint's largest "
          "|limit|.\n\n"
          "Raises:\n"
          "    RuntimeError: a link is missing, no path joins them, or a joint on it has unbounded "
          "translation.");

    m.def("checkKinematics", &tk::checkKinematics, "manip"_a, "tol"_a = CHECK_KINEMATICS_TOL_DEFAULT,
          nb::call_guard<nb::gil_scoped_release>(),
          "Round-trip self-check: FK then IK for every working frame and tip link of manip.\n\n"
          "False when an IK solution's translation or angle distance from the FK pose exceeds tol\n"
          "(validate.cpp:38-175). True also when IK found no solution at all: only failed solutions\n"
          "make it False.");
}
