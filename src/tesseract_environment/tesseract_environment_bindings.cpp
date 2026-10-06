/**
 * @file tesseract_environment_bindings.cpp
 * @brief nanobind bindings for tesseract_environment
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/chrono.h>  // getTimestamp / getCurrentStateTimestamp -> datetime.datetime

#include <atomic>
#include <algorithm>  // std::find (setState validation, GH #43)
#include <stdexcept>  // std::invalid_argument

// tesseract_environment
#include <tesseract/environment/environment.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/command.h>
#include <tesseract/environment/commands/add_contact_managers_plugin_info_command.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/environment/commands/add_kinematics_information_command.h>
#include <tesseract/environment/commands/add_scene_graph_command.h>
#include <tesseract/environment/commands/add_trajectory_link_command.h>
#include <tesseract/environment/commands/change_collision_margins_command.h>
#include <tesseract/environment/commands/change_joint_acceleration_limits_command.h>
#include <tesseract/environment/commands/change_joint_origin_command.h>
#include <tesseract/environment/commands/change_joint_position_limits_command.h>
#include <tesseract/environment/commands/change_joint_velocity_limits_command.h>
#include <tesseract/environment/commands/change_link_collision_enabled_command.h>
#include <tesseract/environment/commands/change_link_origin_command.h>
#include <tesseract/environment/commands/change_link_visibility_command.h>
#include <tesseract/environment/commands/modify_allowed_collisions_command.h>
#include <tesseract/environment/commands/move_joint_command.h>
#include <tesseract/environment/commands/move_link_command.h>
#include <tesseract/environment/commands/remove_allowed_collision_link_command.h>
#include <tesseract/environment/commands/remove_joint_command.h>
#include <tesseract/environment/commands/remove_link_command.h>
#include <tesseract/environment/commands/replace_joint_command.h>

// tesseract_scene_graph
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>

// tesseract_common
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/manipulator_info.h>
#include <filesystem>

// tesseract_srdf
#include <tesseract/srdf/srdf_model.h>

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_collision
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

// tesseract_state_solver - need full definition for getStateSolver return type
#include <tesseract/state_solver/state_solver.h>

namespace te = tesseract::environment;
namespace tsg = tesseract::scene_graph;
namespace tc = tesseract::common;
namespace tk = tesseract::kinematics;
namespace tcol = tesseract::collision;

namespace {
using LocatorPtr = std::shared_ptr<const tc::ResourceLocator>;

// checkTrajectory's `std::vector<ContactResultMap>& contacts` out-param is returned
// alongside the summary instead: Python gets (ContactTrajectoryResults, list[ContactResultMap]).
using CheckTrajectoryResult = std::pair<tcol::ContactTrajectoryResults, std::vector<tcol::ContactResultMap>>;

// The trajectory loop is pure C++ and can run for seconds, so the GIL is released for it.
// Released in-body rather than via call_guard so the result is cast back with the GIL held.
template <typename Manager>
CheckTrajectoryResult check_trajectory(Manager& manager,
                                       const tsg::StateSolver& state_solver,
                                       const std::vector<std::string>& joint_names,
                                       const tc::TrajArray& traj,
                                       const tcol::CollisionCheckConfig& config)
{
    CheckTrajectoryResult out;
    nb::gil_scoped_release release;
    out.first = te::checkTrajectory(out.second, manager, state_solver, joint_names, traj, config);
    return out;
}

template <typename Manager>
CheckTrajectoryResult check_trajectory(Manager& manager,
                                       const tk::JointGroup& manip,
                                       const tc::TrajArray& traj,
                                       const tcol::CollisionCheckConfig& config)
{
    CheckTrajectoryResult out;
    nb::gil_scoped_release release;
    out.first = te::checkTrajectory(out.second, manager, manip, traj, config);
    return out;
}

// GH #43: Environment::setState forwards joint names straight into the state
// solver, which dereferences unknown names unchecked -> SIGSEGV with no Python
// traceback. The binding owns the Python boundary, so validate here and fail
// loud (std::invalid_argument -> ValueError). Valid targets are the ACTIVE
// joints: fixed/mimic joints aren't in the solver's map and crash identically.
void validate_set_state_joint_names(const te::Environment& env,
                                    const std::vector<std::string>& names,
                                    const std::string& caller = "setState")
{
    const std::vector<std::string> active = env.getActiveJointNames();
    std::string unknown;
    for (const auto& name : names) {
        if (std::find(active.begin(), active.end(), name) == active.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += name;
        }
    }
    if (!unknown.empty())
        throw std::invalid_argument(caller + ": unknown or non-active joint names: " + unknown);
}

void validate_set_state(const te::Environment& env,
                        const std::vector<std::string>& names,
                        const Eigen::Ref<const Eigen::VectorXd>& values,
                        const std::string& caller = "setState")
{
    if (static_cast<Eigen::Index>(names.size()) != values.size())
        throw std::invalid_argument(caller + ": joint_names length (" + std::to_string(names.size()) +
                                    ") != joint_values length (" + std::to_string(values.size()) + ")");
    validate_set_state_joint_names(env, names, caller);
}

// gh-188: Environment has no floating-joint name accessor; the keys of
// getCurrentFloatingJointValues() are every floating joint. Returns the names that are not
// floating joints, comma-separated, or "" when all are.
std::string unknown_floating_joint_names(const te::Environment& env, const std::vector<std::string>& names)
{
    const tc::TransformMap floating = env.getCurrentFloatingJointValues();
    std::string unknown;
    for (const auto& name : names) {
        if (floating.find(name) == floating.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += name;
        }
    }
    return unknown;
}

// gh-188: the link-keyed getters do not document a miss; raise KeyError before C++ sees one.
void require_link(const te::Environment& env, const std::string& name)
{
    if (!env.getLink(name))
        throw nb::key_error(("Link not found: " + name).c_str());
}

// Test oracle for the GIL guards (gh-134): counts its copies by whether the copying thread holds
// the GIL. Environment::clone copies every find-TCP callback, so a probe attached as one reports
// exactly whether clone's native work ran with the GIL, independent of threads and timing.
struct GilProbeCounts {
    std::atomic<int> with_gil{0};
    std::atomic<int> without_gil{0};
};

struct GilProbeFn {
    std::shared_ptr<GilProbeCounts> counts;

    explicit GilProbeFn(std::shared_ptr<GilProbeCounts> c) : counts(std::move(c)) {}
    GilProbeFn(const GilProbeFn& other) : counts(other.counts) { record(); }
    GilProbeFn(GilProbeFn&&) noexcept = default;
    GilProbeFn& operator=(const GilProbeFn& other) {
        counts = other.counts;
        record();
        return *this;
    }
    GilProbeFn& operator=(GilProbeFn&&) noexcept = default;

    // Not a TCP source: findTCPOffset catches the throw and tries the next callback.
    Eigen::Isometry3d operator()(const tc::ManipulatorInfo&) const {
        throw std::runtime_error("GIL probe is not a TCP source");
    }

    // PyGILState_Check is not in the stable ABI the wheels build against; Ensure reports
    // PyGILState_LOCKED exactly when this thread already held the GIL, and Release restores it.
    void record() const {
        const PyGILState_STATE state = PyGILState_Ensure();
        ++(state == PyGILState_LOCKED ? counts->with_gil : counts->without_gil);
        PyGILState_Release(state);
    }
};

struct GilProbe {
    std::shared_ptr<GilProbeCounts> counts = std::make_shared<GilProbeCounts>();
};
}  // namespace

// Wrapper for Python event callbacks
struct PyEventCallbackFn {
    nb::callable callback;
    void operator()(const te::Event& evt) const {
        // Events fire from applyCommand, which now runs with the GIL released (and from C++
        // threads such as the ROS 2 monitor), so re-enter the interpreter explicitly.
        nb::gil_scoped_acquire gil;
        callback(nb::cast(evt, nb::rv_policy::reference));
    }
};

NB_MODULE(_tesseract_environment, m) {
    // Import collision module for DiscreteContactManager/ContinuousContactManager types
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");
    // Import common module for ContactManagersPluginInfo (AddContactManagersPluginInfoCommand)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    // Import scene_graph module for JointLimits (getJointLimits)
    nb::module_::import_("tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph");
    m.doc() = "tesseract_environment Python bindings";

    // ========== Events enum ==========
    nb::enum_<te::Events>(m, "Events")
        .value("COMMAND_APPLIED", te::Events::COMMAND_APPLIED)
        .value("SCENE_STATE_CHANGED", te::Events::SCENE_STATE_CHANGED);

    // Export enum values with SWIG-compatible naming
    m.attr("Events_COMMAND_APPLIED") = te::Events::COMMAND_APPLIED;
    m.attr("Events_SCENE_STATE_CHANGED") = te::Events::SCENE_STATE_CHANGED;

    // ========== Event base class ==========
    nb::class_<te::Event>(m, "Event")
        .def_ro("type", &te::Event::type);

    // ========== CommandAppliedEvent ==========
    nb::class_<te::CommandAppliedEvent, te::Event>(m, "CommandAppliedEvent")
        .def_ro("revision", &te::CommandAppliedEvent::revision);

    // ========== SceneStateChangedEvent ==========
    nb::class_<te::SceneStateChangedEvent, te::Event>(m, "SceneStateChangedEvent")
        .def_prop_ro("state", [](const te::SceneStateChangedEvent& self) -> const tsg::SceneState& {
            return self.state;
        });

    // ========== Event cast functions (for Python to downcast Event to specific type) ==========
    m.def("cast_CommandAppliedEvent", [](const te::Event& evt) -> const te::CommandAppliedEvent& {
        return static_cast<const te::CommandAppliedEvent&>(evt);
    }, nb::rv_policy::reference, "Cast Event to CommandAppliedEvent");

    m.def("cast_SceneStateChangedEvent", [](const te::Event& evt) -> const te::SceneStateChangedEvent& {
        return static_cast<const te::SceneStateChangedEvent&>(evt);
    }, nb::rv_policy::reference, "Cast Event to SceneStateChangedEvent");

    // ========== checkTrajectory (utils.h) ==========
    // {discrete, continuous} manager x {StateSolver + joint_names, JointGroup}
    m.def("checkTrajectory",
        [](tcol::DiscreteContactManager& manager, const tsg::StateSolver& state_solver,
           const std::vector<std::string>& joint_names, const tc::TrajArray& traj,
           const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, state_solver, joint_names, traj, config);
        }, "manager"_a, "state_solver"_a, "joint_names"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::DiscreteContactManager& manager, const tk::JointGroup& manip,
           const tc::TrajArray& traj, const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, manip, traj, config);
        }, "manager"_a, "manip"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::ContinuousContactManager& manager, const tsg::StateSolver& state_solver,
           const std::vector<std::string>& joint_names, const tc::TrajArray& traj,
           const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, state_solver, joint_names, traj, config);
        }, "manager"_a, "state_solver"_a, "joint_names"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::ContinuousContactManager& manager, const tk::JointGroup& manip,
           const tc::TrajArray& traj, const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, manip, traj, config);
        }, "manager"_a, "manip"_a, "traj"_a, "config"_a);

    // EventCallbackFn wrapper for Python callbacks
    nb::class_<PyEventCallbackFn>(m, "EventCallbackFn")
        .def(nb::init<nb::callable>(), "callback"_a);

    // ========== Command base class ==========
    nb::class_<te::Command>(m, "Command")
        .def("getType", &te::Command::getType);

    // ========== RemoveJointCommand ==========
    nb::class_<te::RemoveJointCommand, te::Command>(m, "RemoveJointCommand")
        .def(nb::init<std::string>(), "joint_name"_a)
        .def("getJointName", &te::RemoveJointCommand::getJointName);

    // ========== AddLinkCommand ==========
    nb::class_<te::AddLinkCommand, te::Command>(m, "AddLinkCommand")
        .def(nb::init<const tsg::Link&, bool>(), "link"_a, "replace_allowed"_a = false)
        .def(nb::init<const tsg::Link&, const tsg::Joint&, bool>(),
             "link"_a, "joint"_a, "replace_allowed"_a = false)
        .def("getLink", &te::AddLinkCommand::getLink)
        .def("getJoint", &te::AddLinkCommand::getJoint)
        .def("replaceAllowed", &te::AddLinkCommand::replaceAllowed);

    // ========== RemoveLinkCommand ==========
    nb::class_<te::RemoveLinkCommand, te::Command>(m, "RemoveLinkCommand")
        .def(nb::init<std::string>(), "link_name"_a)
        .def("getLinkName", &te::RemoveLinkCommand::getLinkName);

    // ========== AddSceneGraphCommand ==========
    nb::class_<te::AddSceneGraphCommand, te::Command>(m, "AddSceneGraphCommand")
        .def(nb::init<const tsg::SceneGraph&, std::string>(), "scene_graph"_a, "prefix"_a = "")
        .def(nb::init<const tsg::SceneGraph&, const tsg::Joint&, std::string>(),
             "scene_graph"_a, "joint"_a, "prefix"_a = "")
        .def("getSceneGraph", &te::AddSceneGraphCommand::getSceneGraph)
        .def("getJoint", &te::AddSceneGraphCommand::getJoint)
        .def("getPrefix", &te::AddSceneGraphCommand::getPrefix);

    // ========== AddTrajectoryLinkCommand ==========
    // Adds a link whose collision geometry sweeps the given trajectory. `Method` is nested
    // (AddTrajectoryLinkCommand.Method.PER_STATE_OBJECTS), without SWIG-style Method_* module
    // constants. It is registered before the ctor so the `method` default renders in the stub.
    // The arity-0 ctor is a serialization ctor and stays unbound. getTrajectory returns a copy:
    // a Python edit must not reach a command already in an environment's history.
    using TrajectoryLinkMethod = te::AddTrajectoryLinkCommand::Method;
    auto add_trajectory_link_command = nb::class_<te::AddTrajectoryLinkCommand, te::Command>(m, "AddTrajectoryLinkCommand");
    nb::enum_<TrajectoryLinkMethod>(add_trajectory_link_command, "Method")
        .value("PER_STATE_OBJECTS", TrajectoryLinkMethod::PER_STATE_OBJECTS)
        .value("PER_STATE_CONVEX_HULL", TrajectoryLinkMethod::PER_STATE_CONVEX_HULL)
        .value("GLOBAL_PER_LINK_CONVEX_HULL", TrajectoryLinkMethod::GLOBAL_PER_LINK_CONVEX_HULL)
        .value("GLOBAL_CONVEX_HULL", TrajectoryLinkMethod::GLOBAL_CONVEX_HULL);
    add_trajectory_link_command
        .def(nb::init<std::string, std::string, tc::JointTrajectory, bool, TrajectoryLinkMethod>(),
             "link_name"_a, "parent_link_name"_a, "trajectory"_a, "replace_allowed"_a = false,
             "method"_a = TrajectoryLinkMethod::PER_STATE_OBJECTS)
        .def("getLinkName", &te::AddTrajectoryLinkCommand::getLinkName)
        .def("getParentLinkName", &te::AddTrajectoryLinkCommand::getParentLinkName)
        .def("getTrajectory", &te::AddTrajectoryLinkCommand::getTrajectory, nb::rv_policy::copy)
        .def("replaceAllowed", &te::AddTrajectoryLinkCommand::replaceAllowed)
        .def("getMethod", &te::AddTrajectoryLinkCommand::getMethod);
    bind_value_equality(add_trajectory_link_command);

    // ========== AddKinematicsInformationCommand ==========
    // Registers a KinematicsInformation (group defs + IK plugin config) into a live env.
    // insert-merges with existing kinematics info, so pre-existing groups keep resolving.
    nb::class_<te::AddKinematicsInformationCommand, te::Command>(m, "AddKinematicsInformationCommand")
        .def(nb::init<>())
        .def(nb::init<tesseract::srdf::KinematicsInformation>(), "kinematics_information"_a)
        .def("getKinematicsInformation", &te::AddKinematicsInformationCommand::getKinematicsInformation);

    // ========== AddContactManagersPluginInfoCommand ==========
    // The arity-0 ctor is a serialization ctor and stays unbound. The getter returns a copy:
    // the C++ getter is a const& into the command, and a Python edit must not mutate a
    // command already in an environment's history.
    auto add_contact_managers_plugin_info_command = nb::class_<te::AddContactManagersPluginInfoCommand, te::Command>(m, "AddContactManagersPluginInfoCommand")
        .def(nb::init<tesseract::common::ContactManagersPluginInfo>(), "contact_managers_plugin_info"_a)
        .def("getContactManagersPluginInfo", &te::AddContactManagersPluginInfoCommand::getContactManagersPluginInfo,
             nb::rv_policy::copy);
    bind_value_equality(add_contact_managers_plugin_info_command);

    // ========== ModifyAllowedCollisionsType enum ==========
    nb::enum_<te::ModifyAllowedCollisionsType>(m, "ModifyAllowedCollisionsType")
        .value("ADD", te::ModifyAllowedCollisionsType::ADD)
        .value("REMOVE", te::ModifyAllowedCollisionsType::REMOVE)
        .value("REPLACE", te::ModifyAllowedCollisionsType::REPLACE);

    // SWIG-compatible enum values
    m.attr("ModifyAllowedCollisionsType_ADD") = te::ModifyAllowedCollisionsType::ADD;
    m.attr("ModifyAllowedCollisionsType_REMOVE") = te::ModifyAllowedCollisionsType::REMOVE;
    m.attr("ModifyAllowedCollisionsType_REPLACE") = te::ModifyAllowedCollisionsType::REPLACE;

    // ========== ModifyAllowedCollisionsCommand ==========
    nb::class_<te::ModifyAllowedCollisionsCommand, te::Command>(m, "ModifyAllowedCollisionsCommand")
        .def(nb::init<tc::AllowedCollisionMatrix, te::ModifyAllowedCollisionsType>(),
             "acm"_a, "type"_a)
        .def("getModifyType", &te::ModifyAllowedCollisionsCommand::getModifyType)
        .def("getAllowedCollisionMatrix", &te::ModifyAllowedCollisionsCommand::getAllowedCollisionMatrix);

    // ========== RemoveAllowedCollisionLinkCommand ==========
    nb::class_<te::RemoveAllowedCollisionLinkCommand, te::Command>(m, "RemoveAllowedCollisionLinkCommand")
        .def(nb::init<std::string>(), "link_name"_a)
        .def("getLinkName", &te::RemoveAllowedCollisionLinkCommand::getLinkName);

    // ========== ChangeJointPositionLimitsCommand ==========
    nb::class_<te::ChangeJointPositionLimitsCommand, te::Command>(m, "ChangeJointPositionLimitsCommand")
        .def(nb::init<std::string, double, double>(), "joint_name"_a, "lower"_a, "upper"_a)
        .def(nb::init<std::unordered_map<std::string, std::pair<double, double>>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointPositionLimitsCommand::getLimits);

    // ========== ChangeJointVelocityLimitsCommand ==========
    nb::class_<te::ChangeJointVelocityLimitsCommand, te::Command>(m, "ChangeJointVelocityLimitsCommand")
        .def(nb::init<std::string, double>(), "joint_name"_a, "limit"_a)
        .def(nb::init<std::unordered_map<std::string, double>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointVelocityLimitsCommand::getLimits);

    // ========== ChangeJointAccelerationLimitsCommand ==========
    nb::class_<te::ChangeJointAccelerationLimitsCommand, te::Command>(m, "ChangeJointAccelerationLimitsCommand")
        .def(nb::init<std::string, double>(), "joint_name"_a, "limit"_a)
        .def(nb::init<std::unordered_map<std::string, double>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointAccelerationLimitsCommand::getLimits);

    // ========== ChangeCollisionMarginsCommand ==========
    // Note: 0.33 API change - uses CollisionMarginPairData and CollisionMarginPairOverrideType
    nb::class_<te::ChangeCollisionMarginsCommand, te::Command>(m, "ChangeCollisionMarginsCommand")
        .def(nb::init<double>(), "default_margin"_a)
        .def(nb::init<tc::CollisionMarginPairData, tc::CollisionMarginPairOverrideType>(),
             "pair_margin_data"_a, "override_type"_a = tc::CollisionMarginPairOverrideType::REPLACE)
        .def("getDefaultCollisionMargin", &te::ChangeCollisionMarginsCommand::getDefaultCollisionMargin)
        .def("getCollisionMarginPairData", &te::ChangeCollisionMarginsCommand::getCollisionMarginPairData)
        .def("getCollisionMarginPairOverrideType", &te::ChangeCollisionMarginsCommand::getCollisionMarginPairOverrideType);

    // ========== ChangeLinkCollisionEnabledCommand ==========
    nb::class_<te::ChangeLinkCollisionEnabledCommand, te::Command>(m, "ChangeLinkCollisionEnabledCommand")
        .def(nb::init<std::string, bool>(), "link_name"_a, "enabled"_a)
        .def("getLinkName", &te::ChangeLinkCollisionEnabledCommand::getLinkName)
        .def("getEnabled", &te::ChangeLinkCollisionEnabledCommand::getEnabled);

    // ========== ChangeLinkVisibilityCommand ==========
    nb::class_<te::ChangeLinkVisibilityCommand, te::Command>(m, "ChangeLinkVisibilityCommand")
        .def(nb::init<std::string, bool>(), "link_name"_a, "visible"_a)
        .def("getLinkName", &te::ChangeLinkVisibilityCommand::getLinkName)
        .def("getEnabled", &te::ChangeLinkVisibilityCommand::getEnabled);

    // ========== ChangeJointOriginCommand ==========
    nb::class_<te::ChangeJointOriginCommand, te::Command>(m, "ChangeJointOriginCommand")
        .def(nb::init<std::string, const Eigen::Isometry3d&>(), "joint_name"_a, "origin"_a)
        .def("getJointName", &te::ChangeJointOriginCommand::getJointName)
        .def("getOrigin", &te::ChangeJointOriginCommand::getOrigin);

    // ========== ChangeLinkOriginCommand ==========
    nb::class_<te::ChangeLinkOriginCommand, te::Command>(m, "ChangeLinkOriginCommand")
        .def(nb::init<std::string, const Eigen::Isometry3d&>(), "link_name"_a, "origin"_a)
        .def("getLinkName", &te::ChangeLinkOriginCommand::getLinkName)
        .def("getOrigin", &te::ChangeLinkOriginCommand::getOrigin);

    // ========== MoveJointCommand ==========
    nb::class_<te::MoveJointCommand, te::Command>(m, "MoveJointCommand")
        .def(nb::init<std::string, std::string>(), "joint_name"_a, "parent_link"_a)
        .def("getJointName", &te::MoveJointCommand::getJointName)
        .def("getParentLink", &te::MoveJointCommand::getParentLink);

    // ========== MoveLinkCommand ==========
    nb::class_<te::MoveLinkCommand, te::Command>(m, "MoveLinkCommand")
        .def(nb::init<const tsg::Joint&>(), "joint"_a)
        .def("getJoint", &te::MoveLinkCommand::getJoint);

    // ========== ReplaceJointCommand ==========
    nb::class_<te::ReplaceJointCommand, te::Command>(m, "ReplaceJointCommand")
        .def(nb::init<const tsg::Joint&>(), "joint"_a)
        .def("getJoint", &te::ReplaceJointCommand::getJoint);

    // ========== Environment ==========
    nb::class_<te::Environment>(m, "Environment")
        .def(nb::init<>())
        // Init methods
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph) {
            return self.init(scene_graph);
        }, "scene_graph"_a)
        // Init with scene_graph + srdf (makes a copy of srdf into shared_ptr)
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph,
                        const tesseract::srdf::SRDFModel& srdf) {
            auto srdf_ptr = std::make_shared<const tesseract::srdf::SRDFModel>(srdf);
            return self.init(scene_graph, srdf_ptr);
        }, "scene_graph"_a, "srdf"_a)
        // Native URDF/SRDF overloads. A `str` is always *content* (the std::string
        // overloads), never a path: the path overloads take StrictPath, which
        // rejects `str`, so a path must be a pathlib.Path / os.PathLike and a mixed
        // (content, path) call matches nothing and raises TypeError.
        .def("init", nb::overload_cast<const std::string&, const LocatorPtr&>(&te::Environment::init),
             "urdf_string"_a, "locator"_a)
        .def("init", [](te::Environment& self, const tesseract_nb::StrictPath& urdf_path,
                        const LocatorPtr& locator) {
            return self.init(urdf_path.value, locator);
        }, "urdf_path"_a, "locator"_a)
        .def("init", nb::overload_cast<const std::string&, const std::string&, const LocatorPtr&>(
                         &te::Environment::init),
             "urdf_string"_a, "srdf_string"_a, "locator"_a)
        .def("init", [](te::Environment& self, const tesseract_nb::StrictPath& urdf_path,
                        const tesseract_nb::StrictPath& srdf_path, const LocatorPtr& locator) {
            return self.init(urdf_path.value, srdf_path.value, locator);
        }, "urdf_path"_a, "srdf_path"_a, "locator"_a)
        // State methods
        .def("isInitialized", &te::Environment::isInitialized)
        .def("reset", &te::Environment::reset)
        .def("clear", &te::Environment::clear)
        .def("getRevision", &te::Environment::getRevision)
        .def("getInitRevision", &te::Environment::getInitRevision)
        // system_clock::time_point -> naive datetime.datetime in local time (nanobind stl/chrono.h)
        .def("getTimestamp", &te::Environment::getTimestamp,
             "Last update time (any change to the environment), as a naive local `datetime.datetime`.")
        .def("getCurrentStateTimestamp", &te::Environment::getCurrentStateTimestamp,
             "Last update time of the current state, as a naive local `datetime.datetime`.")
        .def("getName", &te::Environment::getName)
        .def("setName", &te::Environment::setName, "name"_a)
        // Scene graph
        .def("getSceneGraph", &te::Environment::getSceneGraph)
        // State
        .def("getState", [](const te::Environment& self) {
            return self.getState();
        })
        .def("getStateByMap", [](const te::Environment& self,
                                  const std::unordered_map<std::string, double>& joints) {
            return self.getState(joints);
        }, "joints"_a)
        .def("getStateByNamesAndValues", [](const te::Environment& self,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("setState", [](te::Environment& self,
                            const std::unordered_map<std::string, double>& joints) {
            std::vector<std::string> names;
            names.reserve(joints.size());
            for (const auto& kv : joints) names.push_back(kv.first);
            validate_set_state_joint_names(self, names);  // GH #43
            self.setState(joints);
        }, "joints"_a)
        .def("setStateByNamesAndValues", [](te::Environment& self,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            validate_set_state(self, joint_names, joint_values);  // GH #43
            self.setState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        // setState with (names, values) - SWIG compatibility
        .def("setState", [](te::Environment& self,
                            const std::vector<std::string>& joint_names,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            validate_set_state(self, joint_names, joint_values);  // GH #43
            self.setState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        // Event callbacks
        .def("addEventCallback", [](te::Environment& self, std::size_t hash, const PyEventCallbackFn& fn) {
            self.addEventCallback(hash, fn);
        }, "hash"_a, "fn"_a)
        .def("removeEventCallback", &te::Environment::removeEventCallback, "hash"_a)
        .def("clearEventCallbacks", &te::Environment::clearEventCallbacks)
        // Commands - RemoveJointCommand
        .def("applyCommand", [](te::Environment& self, const te::RemoveJointCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::RemoveJointCommand>(cmd.getJointName());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::AddLinkCommand& cmd) {
            std::shared_ptr<te::Command> cmd_ptr;
            if (cmd.getJoint() != nullptr) {
                cmd_ptr = std::make_shared<te::AddLinkCommand>(*cmd.getLink(), *cmd.getJoint(), cmd.replaceAllowed());
            } else {
                cmd_ptr = std::make_shared<te::AddLinkCommand>(*cmd.getLink(), cmd.replaceAllowed());
            }
            // Building collision shapes for a mesh-heavy link takes seconds; hold no GIL. Python
            // can still be re-entered from inside this region - an event callback fires on every
            // applied command - which is why PyEventCallbackFn re-acquires the GIL.
            // ponytail: deadlocks if a Python event callback is registered AND another Python
            // thread calls a non-releasing Environment method - callbacks fire under the env's
            // unique lock, so this thread would hold that lock while re-acquiring the GIL. Lift by
            // skipping the release when Python callbacks are registered, if that ever bites.
            nb::gil_scoped_release nogil;
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - RemoveLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::RemoveLinkCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::RemoveLinkCommand>(cmd.getLinkName());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddSceneGraphCommand
        .def("applyCommand", [](te::Environment& self, const te::AddSceneGraphCommand& cmd) {
            std::shared_ptr<te::Command> cmd_ptr;
            if (cmd.getJoint() != nullptr) {
                cmd_ptr = std::make_shared<te::AddSceneGraphCommand>(*cmd.getSceneGraph(), *cmd.getJoint(), cmd.getPrefix());
            } else {
                cmd_ptr = std::make_shared<te::AddSceneGraphCommand>(*cmd.getSceneGraph(), cmd.getPrefix());
            }
            nb::gil_scoped_release nogil;  // see AddLinkCommand above
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddTrajectoryLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::AddTrajectoryLinkCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::AddTrajectoryLinkCommand>(cmd);
            // The convex-hull methods build collision geometry for every state.
            nb::gil_scoped_release nogil;  // see AddLinkCommand above
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddKinematicsInformationCommand
        .def("applyCommand", [](te::Environment& self, const te::AddKinematicsInformationCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::AddKinematicsInformationCommand>(cmd.getKinematicsInformation());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddContactManagersPluginInfoCommand
        .def("applyCommand", [](te::Environment& self, const te::AddContactManagersPluginInfoCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::AddContactManagersPluginInfoCommand>(cmd.getContactManagersPluginInfo());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ModifyAllowedCollisionsCommand
        .def("applyCommand", [](te::Environment& self, const te::ModifyAllowedCollisionsCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ModifyAllowedCollisionsCommand>(cmd.getAllowedCollisionMatrix(), cmd.getModifyType());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - RemoveAllowedCollisionLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::RemoveAllowedCollisionLinkCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::RemoveAllowedCollisionLinkCommand>(cmd.getLinkName());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeJointPositionLimitsCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeJointPositionLimitsCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeJointPositionLimitsCommand>(cmd.getLimits());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeJointVelocityLimitsCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeJointVelocityLimitsCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeJointVelocityLimitsCommand>(cmd.getLimits());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeJointAccelerationLimitsCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeJointAccelerationLimitsCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeJointAccelerationLimitsCommand>(cmd.getLimits());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeCollisionMarginsCommand
        // Note: 0.33 API change - uses CollisionMarginPairData and CollisionMarginPairOverrideType
        .def("applyCommand", [](te::Environment& self, const te::ChangeCollisionMarginsCommand& cmd) {
            // Handle both default margin and pair margins
            auto default_margin = cmd.getDefaultCollisionMargin();
            if (default_margin.has_value()) {
                auto cmd_ptr = std::make_shared<te::ChangeCollisionMarginsCommand>(
                    default_margin.value(), cmd.getCollisionMarginPairData(), cmd.getCollisionMarginPairOverrideType());
                return self.applyCommand(cmd_ptr);
            } else {
                auto cmd_ptr = std::make_shared<te::ChangeCollisionMarginsCommand>(
                    cmd.getCollisionMarginPairData(), cmd.getCollisionMarginPairOverrideType());
                return self.applyCommand(cmd_ptr);
            }
        }, "command"_a)
        // Commands - ChangeLinkCollisionEnabledCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeLinkCollisionEnabledCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeLinkCollisionEnabledCommand>(cmd.getLinkName(), cmd.getEnabled());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeLinkVisibilityCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeLinkVisibilityCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeLinkVisibilityCommand>(cmd.getLinkName(), cmd.getEnabled());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeJointOriginCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeJointOriginCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeJointOriginCommand>(cmd.getJointName(), cmd.getOrigin());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ChangeLinkOriginCommand
        .def("applyCommand", [](te::Environment& self, const te::ChangeLinkOriginCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ChangeLinkOriginCommand>(cmd.getLinkName(), cmd.getOrigin());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - MoveJointCommand
        .def("applyCommand", [](te::Environment& self, const te::MoveJointCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::MoveJointCommand>(cmd.getJointName(), cmd.getParentLink());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - MoveLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::MoveLinkCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::MoveLinkCommand>(*cmd.getJoint());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - ReplaceJointCommand
        .def("applyCommand", [](te::Environment& self, const te::ReplaceJointCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::ReplaceJointCommand>(*cmd.getJoint());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // State solver
        .def("getStateSolver", [](const te::Environment& self) {
            return self.getStateSolver();
        })
        // Joint/Link info
        .def("getJointNames", &te::Environment::getJointNames)
        .def("getActiveJointNames", &te::Environment::getActiveJointNames)
        .def("getLinkNames", &te::Environment::getLinkNames)
        .def("getActiveLinkNames", [](const te::Environment& self) {
            return self.getActiveLinkNames();
        })
        .def("getStaticLinkNames", [](const te::Environment& self) {
            return self.getStaticLinkNames();
        })
        .def("getRootLinkName", &te::Environment::getRootLinkName)
        .def("getCurrentJointValues", [](const te::Environment& self) {
            return self.getCurrentJointValues();
        })
        .def("getCurrentJointValuesByNames", [](const te::Environment& self,
                                                 const std::vector<std::string>& joint_names) {
            return self.getCurrentJointValues(joint_names);
        }, "joint_names"_a)
        // Transforms
        .def("getLinkTransform", &te::Environment::getLinkTransform, "link_name"_a)
        .def("getRelativeLinkTransform", &te::Environment::getRelativeLinkTransform,
             "from_link_name"_a, "to_link_name"_a)
        // gh-188: getLinkTransforms. The (names, values[, floating_joints]) overloads return the
        // C++ out-param (accepted out-param rule) and validate like setState (GH #43): they feed
        // the same state-solver path.
        .def("getLinkTransforms", [](const te::Environment& self) {
            return self.getLinkTransforms();
        })
        .def("getLinkTransforms", [](const te::Environment& self,
                                     const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            validate_set_state(self, joint_names, joint_values, "getLinkTransforms");
            tc::TransformMap link_transforms;
            self.getLinkTransforms(link_transforms, joint_names, joint_values);
            return link_transforms;
        }, "joint_names"_a, "joint_values"_a)
        .def("getLinkTransforms", [](const te::Environment& self,
                                     const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tc::TransformMap& floating_joints) {
            validate_set_state(self, joint_names, joint_values, "getLinkTransforms");
            std::vector<std::string> floating_names;
            floating_names.reserve(floating_joints.size());
            for (const auto& kv : floating_joints) floating_names.push_back(kv.first);
            const std::string unknown = unknown_floating_joint_names(self, floating_names);
            if (!unknown.empty())
                throw std::invalid_argument("getLinkTransforms: unknown floating joint names: " + unknown);
            tc::TransformMap link_transforms;
            self.getLinkTransforms(link_transforms, joint_names, joint_values, floating_joints);
            return link_transforms;
        }, "joint_names"_a, "joint_values"_a, "floating_joints"_a)
        .def("getCurrentFloatingJointValues", [](const te::Environment& self) {
            return self.getCurrentFloatingJointValues();
        })
        .def("getCurrentFloatingJointValues", [](const te::Environment& self,
                                                 const std::vector<std::string>& joint_names) {
            const std::string unknown = unknown_floating_joint_names(self, joint_names);
            if (!unknown.empty())
                throw nb::key_error(("Floating joint not found: " + unknown).c_str());
            return self.getCurrentFloatingJointValues(joint_names);
        }, "joint_names"_a)
        // gh-188: name-keyed getters raise KeyError on a miss instead of None / an unasked-for bool.
        // getJointLimits copies: the C++ getter hands out a pointer into the scene graph.
        .def("getJointLimits", [](const te::Environment& self, const std::string& joint_name) {
            auto ptr = self.getJointLimits(joint_name);
            if (!ptr) throw nb::key_error(("Joint not found: " + joint_name).c_str());
            return tsg::JointLimits(*ptr);
        }, "joint_name"_a)
        .def("getLinkCollisionEnabled", [](const te::Environment& self, const std::string& name) {
            require_link(self, name);
            return self.getLinkCollisionEnabled(name);
        }, "name"_a)
        .def("getLinkVisibility", [](const te::Environment& self, const std::string& name) {
            require_link(self, name);
            return self.getLinkVisibility(name);
        }, "name"_a)
        .def("getContactManagersPluginInfo", &te::Environment::getContactManagersPluginInfo)
        // Link/Joint access - dereference shared_ptr for cross-module compatibility
        .def("getLink", [](const te::Environment& self, const std::string& name) -> const tsg::Link& {
            auto ptr = self.getLink(name);
            if (!ptr) throw std::runtime_error("Link not found: " + name);
            return *ptr;
        }, "name"_a, nb::rv_policy::reference_internal)
        .def("getJoint", [](const te::Environment& self, const std::string& name) -> const tsg::Joint& {
            auto ptr = self.getJoint(name);
            if (!ptr) throw std::runtime_error("Joint not found: " + name);
            return *ptr;
        }, "name"_a, nb::rv_policy::reference_internal)
        // Groups
        .def("getGroupNames", &te::Environment::getGroupNames)
        .def("getGroupJointNames", &te::Environment::getGroupJointNames, "group_name"_a)
        // Return unique_ptr directly - nanobind transfers ownership to Python
        // Previously returned a reference which became dangling when the unique_ptr
        // was destroyed at the end of the lambda, causing segfaults after setState()
        //
        // keep_alive<0, 1>: groups/contact managers are built from plugin
        // instances (OPW/KDL/UR kinematics, bullet/fcl collision) whose vtables
        // live in dylibs owned by the Environment's plugin loader. If Python
        // destroys the Environment first, the loader dlcloses those dylibs and
        // the survivors' destructors virtual-call into unmapped pages (gh-72).
        .def("getJointGroup", [](const te::Environment& self, const std::string& group_name) {
            auto ptr = self.getJointGroup(group_name);
            if (!ptr) throw std::runtime_error("Failed to get joint group: " + group_name);
            return ptr;
        }, "group_name"_a, nb::keep_alive<0, 1>())
        .def("getKinematicGroup", [](const te::Environment& self, const std::string& group_name,
                                      const std::string& ik_solver_name) {
            auto ptr = self.getKinematicGroup(group_name, ik_solver_name);
            if (!ptr) throw std::runtime_error("Failed to get kinematic group: " + group_name);
            return ptr;
        }, "group_name"_a, "ik_solver_name"_a = "", nb::keep_alive<0, 1>())
        // TCP
        .def("findTCPOffset", &te::Environment::findTCPOffset, "manip_info"_a)
        // Contact managers
        // GIL released for the same reason as applyCommand: on the first call these build every
        // collision shape in the scene, which is where an SDF- or mesh-heavy environment spends its
        // seconds when no manager was cached for applyCommand to update. No event callbacks fire
        // here and the env is only read-locked, so the applyCommand caveat does not apply.
        .def("getDiscreteContactManager", [](const te::Environment& self) {
            return self.getDiscreteContactManager();
        }, nb::keep_alive<0, 1>(), nb::call_guard<nb::gil_scoped_release>())
        .def("getContinuousContactManager", [](const te::Environment& self) {
            return self.getContinuousContactManager();
        }, nb::keep_alive<0, 1>(), nb::call_guard<nb::gil_scoped_release>())
        .def("setActiveDiscreteContactManager", &te::Environment::setActiveDiscreteContactManager, "name"_a)
        .def("setActiveContinuousContactManager", &te::Environment::setActiveContinuousContactManager, "name"_a)
        .def("clearCachedDiscreteContactManager", &te::Environment::clearCachedDiscreteContactManager)
        .def("clearCachedContinuousContactManager", &te::Environment::clearCachedContinuousContactManager)
        // ACM and collision
        .def("getAllowedCollisionMatrix", &te::Environment::getAllowedCollisionMatrix)
        .def("getCollisionMarginData", &te::Environment::getCollisionMarginData)
        // Locator
        .def("setResourceLocator", &te::Environment::setResourceLocator, "locator"_a)
        .def("getResourceLocator", &te::Environment::getResourceLocator)
        // Kinematics information (from SRDF)
        .def("getKinematicsInformation", &te::Environment::getKinematicsInformation)
        // clone() is a pure C++ deep-copy (no Python callback) and is mutex-serialised
        // against setState() (Environment::clone takes a shared_lock on the same mutex_),
        // so releasing the GIL lets a background thread clone the env WITHOUT blocking the
        // UI thread — the collision scan clones off-thread for a responsive sweep.
        .def("clone", [](const te::Environment& self) { return self.clone(); },
             nb::call_guard<nb::gil_scoped_release>());

    // Private test oracle; see GilProbeFn.
    nb::class_<GilProbe>(m, "_GilProbe")
        .def(nb::init<>())
        .def("attach", [](const GilProbe& self, te::Environment& env) {
            env.addFindTCPOffsetCallback(GilProbeFn(self.counts));
        }, "env"_a, "Register as a find-TCP callback of env, which clone() copies")
        .def("reset", [](const GilProbe& self) {
            self.counts->with_gil = 0;
            self.counts->without_gil = 0;
        })
        .def_prop_ro("copies_with_gil", [](const GilProbe& self) { return self.counts->with_gil.load(); })
        .def_prop_ro("copies_without_gil", [](const GilProbe& self) { return self.counts->without_gil.load(); });
}
