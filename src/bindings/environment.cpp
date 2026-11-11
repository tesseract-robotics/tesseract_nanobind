/**
 * @file environment.cpp
 * @brief nanobind bindings for tesseract_environment
 */

#include "tesseract_nb.h"
#include "environment_bindings.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/chrono.h>  // getTimestamp / getCurrentStateTimestamp -> datetime.datetime

#include <atomic>
#include <algorithm>  // std::find (setState validation, GH #43)
#include <exception>  // std::exception_ptr (_find_tcp_offset_on_native_thread)
#include <stdexcept>  // std::invalid_argument, std::runtime_error (EventTypeError), std::logic_error
#include <string>     // std::to_string (event_type_name)
#include <thread>     // std::thread (_find_tcp_offset_on_native_thread)

// tesseract_environment
#include <tesseract/environment/environment.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/command.h>

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
                                       const std::vector<tc::JointId>& joint_ids,
                                       const tc::TrajArray& traj,
                                       const tcol::CollisionCheckConfig& config)
{
    CheckTrajectoryResult out;
    nb::gil_scoped_release release;
    out.first = te::checkTrajectory(out.second, manager, state_solver, joint_ids, traj, config);
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

// gh-190: checkTrajectorySegment / checkTrajectoryState do `state.at(link)` for every active
// collision object, so a map missing one throws std::out_of_range mid-loop, after some objects
// were already moved. Check up front, with the GIL held, and name every missing link.
template <typename Manager>
void require_active_link_transforms(const Manager& manager,
                                    const tc::LinkIdTransformMap& state,
                                    const char* caller,
                                    const char* arg)
{
    std::string missing;
    for (const auto& id : manager.getActiveCollisionObjects()) {
        if (state.find(id) == state.end()) {
            if (!missing.empty()) missing += ", ";
            missing += id.name();
        }
    }
    if (!missing.empty())
        throw nb::key_error((std::string(caller) + ": " + arg + " has no transform for active links: " +
                             missing).c_str());
}

// Both write into a `ContactResultMap&` and return void; the map is returned instead (out-param
// rule). A fresh map per call: C++ callers that accumulated across calls merge in Python.
// GIL released in-body, as in check_trajectory, so the map is cast back with the GIL held.
template <typename Manager>
tcol::ContactResultMap check_trajectory_state(Manager& manager,
                                              const tc::LinkIdTransformMap& state,
                                              const tcol::ContactRequest& contact_request)
{
    require_active_link_transforms(manager, state, "checkTrajectoryState", "state");
    tcol::ContactResultMap contact_results;
    nb::gil_scoped_release release;
    te::checkTrajectoryState(contact_results, manager, state, contact_request);
    return contact_results;
}

tcol::ContactResultMap check_trajectory_segment(tcol::ContinuousContactManager& manager,
                                                const tc::LinkIdTransformMap& state0,
                                                const tc::LinkIdTransformMap& state1,
                                                const tcol::ContactRequest& contact_request)
{
    require_active_link_transforms(manager, state0, "checkTrajectorySegment", "state0");
    require_active_link_transforms(manager, state1, "checkTrajectorySegment", "state1");
    tcol::ContactResultMap contact_results;
    nb::gil_scoped_release release;
    te::checkTrajectorySegment(contact_results, manager, state0, state1, contact_request);
    return contact_results;
}

// GH #43: Environment::setState forwards joint names straight into the state
// solver, which dereferences unknown names unchecked -> SIGSEGV with no Python
// traceback. The binding owns the Python boundary, so validate here and fail
// loud (std::invalid_argument -> ValueError). Valid targets are the ACTIVE
// joints: fixed/mimic joints aren't in the solver's map and crash identically.
// (M, N) joint states as numpy holds them: one state per row.
using RowMajorXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void validate_set_state_joint_ids(const te::Environment& env,
                                    const std::vector<tc::JointId>& ids,
                                    const std::string& caller = "setState")
{
    const std::vector<tc::JointId> active = env.getActiveJointIds();
    std::string unknown;
    for (const auto& id : ids) {
        if (std::find(active.begin(), active.end(), id) == active.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += id.name();
        }
    }
    if (!unknown.empty())
        throw std::invalid_argument(caller + ": unknown or non-active joint ids: " + unknown);
}

void validate_set_state(const te::Environment& env,
                        const std::vector<tc::JointId>& names,
                        const Eigen::Ref<const Eigen::VectorXd>& values,
                        const std::string& caller = "setState")
{
    if (static_cast<Eigen::Index>(names.size()) != values.size())
        throw std::invalid_argument(caller + ": joint_ids length (" + std::to_string(names.size()) +
                                    ") != joint_values length (" + std::to_string(values.size()) + ")");
    validate_set_state_joint_ids(env, names, caller);
}

// gh-188: Environment has no floating-joint name accessor; the keys of
// getCurrentFloatingJointValues() are every floating joint. Returns the names of the ids that are
// not floating joints, comma-separated, or "" when all are.
std::string unknown_floating_joint_ids(const te::Environment& env, const std::vector<tc::JointId>& ids)
{
    const tc::JointIdTransformMap floating = env.getCurrentFloatingJointValues();
    std::string unknown;
    for (const auto& id : ids) {
        if (floating.find(id) == floating.end()) {
            if (!unknown.empty()) unknown += ", ";
            unknown += id.name();
        }
    }
    return unknown;
}

// Map keys as an id list, for the validators that take ids.
template <typename Map>
std::vector<typename Map::key_type> keys_of(const Map& map)
{
    std::vector<typename Map::key_type> keys;
    keys.reserve(map.size());
    for (const auto& kv : map) keys.push_back(kv.first);
    return keys;
}

// gh-187: the floating_joints argument of getState/setState/getLinkTransforms reaches an unchecked
// floating_joints.at() in the state solver (std::out_of_range -> IndexError). In setState that
// throw lands after the joint values were stored, leaving the solver half-updated. Validate first.
void validate_floating_joints(const te::Environment& env,
                              const tc::JointIdTransformMap& floating_joints,
                              const std::string& caller)
{
    if (floating_joints.empty()) return;  // the common call: skip copying the floating-joint map
    const std::string unknown = unknown_floating_joint_ids(env, keys_of(floating_joints));
    if (!unknown.empty())
        throw std::invalid_argument(caller + ": unknown floating joint names: " + unknown);
}

// gh-187: getActiveLinkNames/getStaticLinkNames/getJointGroup(name, joint_names) accept any joint
// type, so they check scene-graph membership, not the active set (GH #43 validators).
void validate_joint_ids_exist(const te::Environment& env,
                                const std::vector<tc::JointId>& ids,
                                const std::string& caller)
{
    std::string unknown;
    for (const auto& id : ids) {
        if (!env.getJoint(id)) {
            if (!unknown.empty()) unknown += ", ";
            unknown += id.name();
        }
    }
    if (!unknown.empty())
        throw std::invalid_argument(caller + ": unknown joint names: " + unknown);
}

// gh-187: the by-name contact-manager getters return nullptr for a name the factory does not
// know (upstream logs and returns). A miss is a KeyError (gh-188); a registered plugin that
// still fails to build is a RuntimeError. Pure C++, safe under the getters' GIL release.
void require_discrete_contact_manager(const te::Environment& env, const std::string& name)
{
    const auto plugins = env.getContactManagersPluginInfo().discrete_plugin_infos.plugins;
    if (plugins.find(name) == plugins.end())
        throw nb::key_error(("Discrete contact manager not found: " + name).c_str());
}

void require_continuous_contact_manager(const te::Environment& env, const std::string& name)
{
    const auto plugins = env.getContactManagersPluginInfo().continuous_plugin_infos.plugins;
    if (plugins.find(name) == plugins.end())
        throw nb::key_error(("Continuous contact manager not found: " + name).c_str());
}

// gh-188: the link-keyed getters do not document a miss; raise KeyError before C++ sees one.
void require_link(const te::Environment& env, const tc::LinkId& id)
{
    if (!env.getLink(id))
        throw nb::key_error(("Link not found: " + id.name()).c_str());
}

// An SRDF group name, the key of every group getter: a miss is a KeyError (gh-188).
void require_group(const te::Environment& env, const std::string& group_name)
{
    const auto groups = env.getGroupNames();
    if (groups.find(group_name) == groups.end())
        throw nb::key_error(("Group not found: " + group_name).c_str());
}

// getKinematicGroup's (group, solver) key. Upstream throws a bare runtime_error for a group with
// no IK configuration and returns nullptr for an unknown solver name.
void require_ik_solver(const te::Environment& env, const std::string& group_name, const std::string& ik_solver_name)
{
    require_group(env, group_name);
    const auto info = env.getKinematicsInformation().kinematics_plugin_info.inv_plugin_infos;
    const auto group = info.find(group_name);
    if (group == info.end() || group->second.plugins.empty())
        throw nb::key_error(("Group '" + group_name + "' has no inverse kinematics solver").c_str());
    if (!ik_solver_name.empty() && group->second.plugins.find(ik_solver_name) == group->second.plugins.end())
        throw nb::key_error(("Inverse kinematics solver '" + ik_solver_name + "' not found for group '" +
                             group_name + "'").c_str());
}

// gh-186: nanobind's None check covers a direct shared_ptr argument but not the elements of a
// list, so `[cmd, None]` arrives with a null element, which upstream dereferences (SIGSEGV).
using CommandList = std::vector<std::shared_ptr<const te::Command>>;

const CommandList& require_commands(const CommandList& commands, const char* caller)
{
    return require_non_null(commands, caller, "commands", "a Command");  // gh-201 shared helper
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

// cast_*Event used an unchecked static_cast: a mismatched event was undefined behaviour.
// The casts now check the Events tag first and raise EventTypeError (a TypeError).
struct EventTypeError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Enumerator name as Python prints it (Events.<NAME>). Every Events value has a case, so
// -Wswitch flags a value added upstream.
const char* event_type_name(te::Events type) {
    switch (type) {
        case te::Events::COMMAND_APPLIED: return "COMMAND_APPLIED";
        case te::Events::SCENE_STATE_CHANGED: return "SCENE_STATE_CHANGED";
    }
    throw std::logic_error("event_type_name: unhandled Events value " + std::to_string(static_cast<int>(type)));
}

template <typename Derived, te::Events Expected>
const Derived& checked_event_cast(const te::Event& evt, const char* name) {
    if (evt.type != Expected)
        throw EventTypeError(std::string(name) + ": event type is Events." + event_type_name(evt.type) +
                             ", expected Events." + event_type_name(Expected));
    return static_cast<const Derived&>(evt);
}

// The events hand callbacks references into the environment (CommandAppliedEvent::commands,
// SceneStateChangedEvent::state) that die when the callback returns. Python gets an event over
// a copy instead, owned with it, so a callback may keep the event. Members are constructed in
// declaration order: the copy exists before the event binds a reference to it.
struct CommandAppliedEventCopy {
    explicit CommandAppliedEventCopy(const te::CommandAppliedEvent& event)
        : commands(event.commands), event(commands, event.revision) {}
    const std::vector<std::shared_ptr<const te::Command>> commands;
    const te::CommandAppliedEvent event;
};

struct SceneStateChangedEventCopy {
    explicit SceneStateChangedEventCopy(const te::SceneStateChangedEvent& event) : state(event.state), event(state) {}
    const tsg::SceneState state;
    const te::SceneStateChangedEvent event;
};

// The Python object of an owned copy of `event`: the event refers into a copy that a capsule
// owns, and the event's wrapper keeps the capsule alive. Called with the GIL held.
template <typename Copy, typename Event>
nb::object owned_event(const te::Event& event) {
    auto* copy = new Copy(static_cast<const Event&>(event));
    nb::capsule owner(copy, [](void* p) noexcept { delete static_cast<Copy*>(p); });
    return nb::cast(&copy->event, nb::rv_policy::reference_internal, owner);
}

using PyEventCallable = nb::typed<nb::callable, void(te::Event)>;

// gh-183-style Python-backed EventCallbackFn. Upstream fires the callbacks from the thread that
// changed the environment, under its shared lock; every Environment binding runs that change
// without the GIL, and C++ threads (a ROS 2 monitor) change it too, so the call takes the GIL
// itself.
struct PyEventCallbackFn {
    std::shared_ptr<nb::object> fn;

    void operator()(const te::Event& event) const {
        if (!nb::is_alive()) return;  // a C++ thread changing the environment after interpreter exit
        nb::gil_scoped_acquire gil;  // declared first, so it is released after every Python object below
        nb::object py_event;
        switch (event.type) {
            case te::Events::COMMAND_APPLIED:
                py_event = owned_event<CommandAppliedEventCopy, te::CommandAppliedEvent>(event);
                break;
            case te::Events::SCENE_STATE_CHANGED:
                py_event = owned_event<SceneStateChangedEventCopy, te::SceneStateChangedEvent>(event);
                break;
        }
        (*fn)(py_event);
    }
};

// gh-183: Python-backed FindTCPOffsetCallbackFn. Environment::clone copies every find-TCP callback
// with the GIL released (gh-134), and planners call findTCPOffset from task-composer worker threads,
// so neither a copy nor a call may assume the GIL (gil_safe_object, tesseract_nb.h).
using PyFindTCPOffsetCallable = nb::typed<nb::callable, Eigen::Isometry3d(tc::ManipulatorInfo)>;

struct PyFindTCPOffsetFn {
    std::shared_ptr<nb::object> fn;

    // Every throw below counts as "not located": upstream's findTCPOffset wraps each callback in
    // catch (...) and tries the next one (environment.cpp, Implementation::findTCPOffset).
    Eigen::Isometry3d operator()(const tc::ManipulatorInfo& manip_info) const {
        if (!nb::is_alive())
            throw std::runtime_error("find-TCP callback: the Python interpreter has shut down");
        nb::gil_scoped_acquire gil;  // declared first, so it is released after every Python object below
        nb::object ret;
        try {
            ret = (*fn)(manip_info);
        } catch (nb::python_error& e) {
            // Converted while the GIL is held: the Python exception is released here, not in
            // upstream's catch (...) under the environment's lock.
            throw std::runtime_error(std::string("find-TCP callback raised: ") + e.what());
        }
        Eigen::Isometry3d tcp;
        if (!nb::try_cast<Eigen::Isometry3d>(ret, tcp))
            throw std::runtime_error(std::string("find-TCP callback returned ") +
                                     nb::type_name(ret.type()).c_str() + ", not Isometry3d");
        return tcp;
    }
};
}  // namespace

NB_MODULE(environment, m) {
    // Import collision module for DiscreteContactManager/ContinuousContactManager types
    nb::module_::import_("tesseract_robotics.collision");
    // Import common module for ContactManagersPluginInfo (AddContactManagersPluginInfoCommand)
    // and the ContactAllowedValidator base of EnvironmentContactAllowedValidator
    nb::module_::import_("tesseract_robotics.common._common");
    // Import scene_graph module for JointLimits (getJointLimits) and SceneGraph
    // (EnvironmentContactAllowedValidator constructor)
    nb::module_::import_("tesseract_robotics.scene_graph._scene_graph");
    // kinematics (getKinematicGroup) and srdf (getKinematicsInformation) return types
    nb::module_::import_("tesseract_robotics.kinematics");
    nb::module_::import_("tesseract_robotics.srdf");
    m.doc() = "tesseract_environment Python bindings";

    // ========== Events enum ==========
    nb::enum_<te::Events>(m, "Events")
        .value("COMMAND_APPLIED", te::Events::COMMAND_APPLIED)
        .value("SCENE_STATE_CHANGED", te::Events::SCENE_STATE_CHANGED);

    // ========== Event base class ==========
    // A callback receives the most-derived event, over its own copy of the environment data it
    // names (the history, the state): it stays valid after the callback returns.
    nb::class_<te::Event>(m, "Event", "An environment event; `type` names its derived class.")
        .def_ro("type", &te::Event::type, "Which event this is.");

    // ========== CommandAppliedEvent ==========
    // `commands` is the environment's whole history at the event; the property copies the list
    // (polymorphic, see getCommandHistory).
    nb::class_<te::CommandAppliedEvent, te::Event>(m, "CommandAppliedEvent",
        "Fired after commands were applied (applyCommand(s), init, reset).")
        .def_prop_ro("commands", [](const te::CommandAppliedEvent& self) {
            return CommandList(self.commands);
        }, "The environment's command history at the event (a copy).")
        .def_ro("revision", &te::CommandAppliedEvent::revision, "The environment's revision after the commands.");

    // ========== SceneStateChangedEvent ==========
    nb::class_<te::SceneStateChangedEvent, te::Event>(m, "SceneStateChangedEvent",
        "Fired after the current state changed (setState, and every applied command).")
        .def_prop_ro("state", [](const te::SceneStateChangedEvent& self) -> const tsg::SceneState& {
            return self.state;
        }, "The new current state.");

    // ========== Event cast functions (for Python to downcast Event to specific type) ==========
    // Callbacks already receive the most-derived class (Event is polymorphic), so these only
    // check the type. The result is the argument's own Python object (nanobind finds the
    // registered wrapper for the same pointer).
    nb::exception<EventTypeError>(m, "EventTypeError", PyExc_TypeError)
        .attr("__doc__") = "cast_CommandAppliedEvent / cast_SceneStateChangedEvent got an event of the other type.";
    m.def("cast_CommandAppliedEvent", [](const te::Event& evt) -> const te::CommandAppliedEvent& {
        return checked_event_cast<te::CommandAppliedEvent, te::Events::COMMAND_APPLIED>(evt, "cast_CommandAppliedEvent");
    }, "evt"_a, nb::rv_policy::reference,
       "Downcast an Event to CommandAppliedEvent. Raises EventTypeError if evt.type is not COMMAND_APPLIED.");

    m.def("cast_SceneStateChangedEvent", [](const te::Event& evt) -> const te::SceneStateChangedEvent& {
        return checked_event_cast<te::SceneStateChangedEvent, te::Events::SCENE_STATE_CHANGED>(
            evt, "cast_SceneStateChangedEvent");
    }, "evt"_a, nb::rv_policy::reference,
       "Downcast an Event to SceneStateChangedEvent. Raises EventTypeError if evt.type is not SCENE_STATE_CHANGED.");

    // ========== checkTrajectory (utils.h) ==========
    // {discrete, continuous} manager x {StateSolver + joint_ids, JointGroup}
    m.def("checkTrajectory",
        [](tcol::DiscreteContactManager& manager, const tsg::StateSolver& state_solver,
           const std::vector<tc::JointId>& joint_ids, const tc::TrajArray& traj,
           const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, state_solver, joint_ids, traj, config);
        }, "manager"_a, "state_solver"_a, "joint_ids"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::DiscreteContactManager& manager, const tk::JointGroup& manip,
           const tc::TrajArray& traj, const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, manip, traj, config);
        }, "manager"_a, "manip"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::ContinuousContactManager& manager, const tsg::StateSolver& state_solver,
           const std::vector<tc::JointId>& joint_ids, const tc::TrajArray& traj,
           const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, state_solver, joint_ids, traj, config);
        }, "manager"_a, "state_solver"_a, "joint_ids"_a, "traj"_a, "config"_a);
    m.def("checkTrajectory",
        [](tcol::ContinuousContactManager& manager, const tk::JointGroup& manip,
           const tc::TrajArray& traj, const tcol::CollisionCheckConfig& config) {
            return check_trajectory(manager, manip, traj, config);
        }, "manager"_a, "manip"_a, "traj"_a, "config"_a);

    // ========== checkTrajectorySegment / checkTrajectoryState (utils.h) ==========
    // Single-step checks; the ContactResultMap out-param is the return value. A state map missing
    // an active collision link raises KeyError before the manager is touched (gh-190).
    m.def("checkTrajectorySegment", &check_trajectory_segment,
          "manager"_a, "state0"_a, "state1"_a, "contact_request"_a);
    m.def("checkTrajectoryState",
        [](tcol::DiscreteContactManager& manager, const tc::LinkIdTransformMap& state,
           const tcol::ContactRequest& contact_request) {
            return check_trajectory_state(manager, state, contact_request);
        }, "manager"_a, "state"_a, "contact_request"_a);
    m.def("checkTrajectoryState",
        [](tcol::ContinuousContactManager& manager, const tc::LinkIdTransformMap& state,
           const tcol::ContactRequest& contact_request) {
            return check_trajectory_state(manager, state, contact_request);
        }, "manager"_a, "state"_a, "contact_request"_a);

    bind_commands(m);  // environment_commands.cpp

    // gh-187: getState validates like setState (GH #43). Unvalidated, the state solver inserts
    // unknown or fixed joint names into the returned state without complaint and, on a
    // ids/values length mismatch, reads joint_values out of bounds.
    const auto get_state_map = [](const te::Environment& self,
                                  const tsg::SceneState::JointValues& joints,
                                  const tc::JointIdTransformMap& floating_joints) {
        validate_set_state_joint_ids(self, keys_of(joints), "getState");
        validate_floating_joints(self, floating_joints, "getState");
        return self.getState(joints, floating_joints);
    };
    const auto get_state_ids_values = [](const te::Environment& self,
                                         const std::vector<tc::JointId>& joint_ids,
                                         const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                         const tc::JointIdTransformMap& floating_joints) {
        validate_set_state(self, joint_ids, joint_values, "getState");
        validate_floating_joints(self, floating_joints, "getState");
        return self.getState(joint_ids, joint_values, floating_joints);
    };
    // current_state.joints is keyed by exactly the active joints; a miss is std::out_of_range
    // (IndexError) upstream. Raise KeyError like getCurrentFloatingJointValues(joint_ids).
    const auto get_current_joint_values_by_ids = [](const te::Environment& self,
                                                    const std::vector<tc::JointId>& joint_ids) {
        std::string unknown;
        const std::vector<tc::JointId> active = self.getActiveJointIds();
        for (const auto& id : joint_ids) {
            if (std::find(active.begin(), active.end(), id) == active.end()) {
                if (!unknown.empty()) unknown += ", ";
                unknown += id.name();
            }
        }
        if (!unknown.empty())
            throw nb::key_error(("Active joint not found: " + unknown).c_str());
        return self.getCurrentJointValues(joint_ids);
    };

    // ========== EnvironmentContactAllowedValidator ==========
    // Validator Environment installs on every contact manager it creates or clones: a link pair
    // may collide unless the scene graph's ACM allows it. Holds the scene graph by shared_ptr, so
    // it reflects later ACM changes to that graph and keeps it alive. No default constructor: it
    // exists for cereal and leaves scene_graph_ null, which operator() dereferences. __call__ is
    // inherited from the tesseract_common base binding and dispatches virtually.
    nb::class_<te::EnvironmentContactAllowedValidator, tc::ContactAllowedValidator>(
        m, "EnvironmentContactAllowedValidator",
        "The contact validator an Environment installs on its contact managers: a link pair may collide "
        "unless the scene graph's allowed collision matrix allows it.")
        .def(nb::init<std::shared_ptr<const tsg::SceneGraph>>(), "scene_graph"_a,
             "Validate against `scene_graph`'s ACM, kept alive and read at each call.");

    // ========== Environment ==========
    // Every method that reaches the environment runs with the GIL released (NoGil). Upstream
    // fires event callbacks under the environment's shared lock, from the changing thread; a
    // Python callback then waits for the GIL while holding that lock. A thread that waited for
    // the environment's lock while holding the GIL would deadlock against it, writers (unique
    // lock) always and readers too on platforms whose shared mutex queues them behind a waiting
    // writer (docs/developer/threading.md). Validation runs inside the release: it reads the
    // environment, and its errors are plain C++ exceptions, translated after the GIL is back.
    // Arguments are converted before the release and results after it.
    using NoGil = nb::call_guard<nb::gil_scoped_release>;
    auto environment = nb::class_<te::Environment>(m, "Environment",
        "A scene graph with its current state, kinematic groups, contact managers and command history.\n\n"
        "Thread-safe: every method takes the environment's lock without holding the GIL, so Python "
        "threads may share one environment. Event and find-TCP callbacks run on the thread that changed "
        "or queried the environment, under its lock: they must not change it.")
        .def(nb::init<>(), "An empty, uninitialized environment; call `init`.")
        // Init methods. A `str` is always *content* (the std::string overloads), never a path: the
        // path overloads take StrictPath, which rejects `str`, so a path must be a pathlib.Path /
        // os.PathLike and a mixed (content, path) call matches nothing and raises TypeError.
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph) {
            return self.init(scene_graph);
        }, "scene_graph"_a, NoGil(),
           "Initialize from a copy of `scene_graph`, with no SRDF; False on failure.")
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph,
                        const tesseract::srdf::SRDFModel& srdf) {
            return self.init(scene_graph, std::make_shared<const tesseract::srdf::SRDFModel>(srdf));
        }, "scene_graph"_a, "srdf"_a, NoGil(),
           "Initialize from copies of `scene_graph` and `srdf`; False on failure.")
        .def("init", nb::overload_cast<const std::string&, const LocatorPtr&>(&te::Environment::init),
             "urdf_string"_a, "locator"_a, NoGil(),
             "Initialize from URDF text, resolving package:// and file:// URLs with `locator`; False on failure.")
        .def("init", [](te::Environment& self, const tesseract_nb::StrictPath& urdf_path,
                        const LocatorPtr& locator) {
            return self.init(urdf_path.value, locator);
        }, "urdf_path"_a, "locator"_a, NoGil(),
           "Initialize from a URDF file; False on failure.")
        .def("init", nb::overload_cast<const std::string&, const std::string&, const LocatorPtr&>(
                         &te::Environment::init),
             "urdf_string"_a, "srdf_string"_a, "locator"_a, NoGil(),
             "Initialize from URDF and SRDF text; False on failure.")
        .def("init", [](te::Environment& self, const tesseract_nb::StrictPath& urdf_path,
                        const tesseract_nb::StrictPath& srdf_path, const LocatorPtr& locator) {
            return self.init(urdf_path.value, srdf_path.value, locator);
        }, "urdf_path"_a, "srdf_path"_a, "locator"_a, NoGil(),
           "Initialize from URDF and SRDF files; False on failure.")
        .def("init", [](te::Environment& self, const CommandList& commands) {
            return self.init(require_commands(commands, "init"));
        }, "commands"_a, NoGil(),
           "Initialize by replaying `commands`, the inverse of getCommandHistory: the list must start "
           "with an AddSceneGraphCommand. False on failure.")
        // State methods
        .def("isInitialized", &te::Environment::isInitialized, NoGil(), "True once an init call succeeded.")
        .def("reset", &te::Environment::reset, NoGil(),
             "Re-initialize from the init commands, dropping every later command; False on failure.")
        .def("clear", &te::Environment::clear, NoGil(), "Drop everything: the environment is uninitialized.")
        .def("getRevision", &te::Environment::getRevision, NoGil(),
             "The number of commands applied, init's own included.")
        .def("getInitRevision", &te::Environment::getInitRevision, NoGil(),
             "The revision right after init: what reset() returns to.")
        // system_clock::time_point -> naive datetime.datetime in local time (nanobind stl/chrono.h)
        .def("getTimestamp", &te::Environment::getTimestamp, NoGil(),
             "Last update time (any change to the environment), as a naive local `datetime.datetime`.")
        .def("getCurrentStateTimestamp", &te::Environment::getCurrentStateTimestamp, NoGil(),
             "Last update time of the current state, as a naive local `datetime.datetime`.")
        .def("getName", &te::Environment::getName, NoGil(), "The environment's name (the URDF robot name).")
        .def("setName", &te::Environment::setName, "name"_a, NoGil(), "Rename the environment.")
        // Scene graph
        .def("getSceneGraph", &te::Environment::getSceneGraph, NoGil(),
             "The scene graph, shared with the environment: read-only, changed only through commands.")
        // State
        .def("getState", [](const te::Environment& self) { return self.getState(); }, NoGil(),
             "The current state: joint values and every link's world transform.")
        .def("getState", get_state_map, "joints"_a, "floating_joints"_a = tc::JointIdTransformMap{}, NoGil(),
             "The state at the active-joint values `joints` (the rest current), without changing the "
             "environment. Unknown or non-active joints raise ValueError.")
        .def("getState", get_state_ids_values,
             "joint_ids"_a, "joint_values"_a, "floating_joints"_a = tc::JointIdTransformMap{}, NoGil(),
             "The state at `joint_values` for `joint_ids` (the rest current), without changing the environment.")
        .def("getState", [](const te::Environment& self, const tc::JointIdTransformMap& floating_joints) {
            validate_floating_joints(self, floating_joints, "getState");
            return self.getState(floating_joints);
        }, "floating_joints"_a, NoGil(),
           "The state at the floating-joint transforms `floating_joints` (the rest current).")
        // M joint states in one call: (M, N) values of `joint_ids` -> (M, L, 4, 4) world
        // transforms of `link_ids`, row-major. Row m is bitwise
        // `getState(joint_ids, joint_values[m]).link_transforms[link].matrix`; the
        // environment's own state is not moved. The capsule is a Python object: built with the GIL.
        .def("link_transforms_batch", [](const te::Environment& self,
                                         const std::vector<tc::JointId>& joint_ids,
                                         const Eigen::Ref<const RowMajorXd>& joint_values,
                                         const std::vector<tc::LinkId>& link_ids) {
            const std::size_t m = static_cast<std::size_t>(joint_values.rows());
            const std::size_t l = link_ids.size();
            auto data = std::make_unique<double[]>(16 * m * l);
            {
                nb::gil_scoped_release nogil;
                if (static_cast<Eigen::Index>(joint_ids.size()) != joint_values.cols())
                    throw std::invalid_argument("link_transforms_batch: joint_ids length (" +
                                                std::to_string(joint_ids.size()) + ") != joint_values columns (" +
                                                std::to_string(joint_values.cols()) + ")");
                validate_set_state_joint_ids(self, joint_ids, "link_transforms_batch");
                std::string unknown;
                for (const auto& id : link_ids) {
                    if (!self.getLink(id)) {
                        if (!unknown.empty()) unknown += ", ";
                        unknown += id.name();
                    }
                }
                if (!unknown.empty())
                    throw std::invalid_argument("link_transforms_batch: unknown link ids: " + unknown);
                for (std::size_t row = 0; row < m; ++row) {
                    const Eigen::VectorXd values = joint_values.row(static_cast<Eigen::Index>(row)).transpose();
                    const auto state = self.getState(joint_ids, values);
                    for (std::size_t link = 0; link < l; ++link) {
                        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(data.get() + 16 * (row * l + link)) =
                            state.link_transforms.at(link_ids[link]).matrix();
                    }
                }
            }
            double* raw = data.release();
            nb::capsule owner(raw, [](void* p) noexcept { delete[] static_cast<double*>(p); });
            return nb::ndarray<nb::numpy, double, nb::shape<-1, -1, 4, 4>, nb::c_contig>(raw, {m, l, 4, 4}, owner);
        }, "joint_ids"_a, "joint_values"_a, "link_ids"_a,
           "World transforms of `link_ids` at each row of `joint_values` ((M, N) for the N `joint_ids`), "
           "as an (M, L, 4, 4) array; the environment's state is not changed.")
        .def("setState", [](te::Environment& self,
                            const tsg::SceneState::JointValues& joints,
                            const tc::JointIdTransformMap& floating_joints) {
            validate_set_state_joint_ids(self, keys_of(joints));  // GH #43
            validate_floating_joints(self, floating_joints, "setState");
            self.setState(joints, floating_joints);
        }, "joints"_a, "floating_joints"_a = tc::JointIdTransformMap{}, NoGil(),
           "Set the active-joint values `joints` (and floating-joint transforms). Unknown or non-active "
           "joints raise ValueError before anything changes.")
        .def("setState", [](te::Environment& self,
                            const std::vector<tc::JointId>& joint_ids,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                            const tc::JointIdTransformMap& floating_joints) {
            validate_set_state(self, joint_ids, joint_values);  // GH #43
            validate_floating_joints(self, floating_joints, "setState");
            self.setState(joint_ids, joint_values, floating_joints);
        }, "joint_ids"_a, "joint_values"_a, "floating_joints"_a = tc::JointIdTransformMap{}, NoGil(),
           "Set `joint_values` for `joint_ids` (and floating-joint transforms).")
        .def("setState", [](te::Environment& self, const tc::JointIdTransformMap& floating_joints) {
            validate_floating_joints(self, floating_joints, "setState");
            self.setState(floating_joints);
        }, "floating_joints"_a, NoGil(), "Set floating-joint transforms.")
        // Event callbacks: a plain callable, keyed by the environment.
        .def("addEventCallback", [](te::Environment& self, PyEventCallable fn) {
            const PyEventCallbackFn callback{ gil_safe_object(std::move(fn)) };
            nb::gil_scoped_release nogil;
            // The key is the address of the callback's holder, unique while it is registered;
            // a key C++ code already uses is skipped.
            auto key = reinterpret_cast<std::size_t>(callback.fn.get());
            const auto existing = self.getEventCallbacks();
            while (existing.find(key) != existing.end()) ++key;
            self.addEventCallback(key, callback);
            return key;
        }, "fn"_a,
           "Register `fn(event)`, called after each change with a CommandAppliedEvent or a "
           "SceneStateChangedEvent; returns the key for removeEventCallback.\n\n"
           "`fn` runs on the thread that changed the environment, under its lock: it must not change "
           "the environment. The environment keeps `fn` alive; `clone()` does not copy it.")
        .def("removeEventCallback", &te::Environment::removeEventCallback, "key"_a, NoGil(),
             "Unregister the callback addEventCallback returned `key` for; an unknown key is ignored.")
        .def("clearEventCallbacks", &te::Environment::clearEventCallbacks, NoGil(), "Unregister every event callback.")
        // Commands. nanobind's shared_ptr caster keeps a Python-created command alive while the
        // history holds it, and drops it under the GIL: no per-type copy.
        .def("applyCommand", [](te::Environment& self, std::shared_ptr<const te::Command> command) {
            return self.applyCommand(std::move(command));
        }, "command"_a, NoGil(),
           "Apply one command; True on success. The command joins the history (getCommandHistory).")
        .def("applyCommands", [](te::Environment& self, const CommandList& commands) {
            return self.applyCommands(require_commands(commands, "applyCommands"));
        }, "commands"_a, NoGil(),
           "Apply the commands in order; False, and stop, at the first that fails.")
        // Every command applied since construction, init's own included; each element arrives as
        // its bound derived class.
        .def("getCommandHistory", &te::Environment::getCommandHistory, NoGil(),
             "Every command applied since init, init's own included, each as its derived class.")
        .def("getStateSolver", [](const te::Environment& self) { return self.getStateSolver(); }, NoGil(),
             "A new state solver at the current state, independent of the environment.")
        // Joint/Link info
        .def("getJointIds", &te::Environment::getJointIds, NoGil(), "Every joint id.")
        .def("getActiveJointIds", &te::Environment::getActiveJointIds, NoGil(),
             "The active (non-fixed, non-mimic) joint ids, in state-vector order.")
        .def("getLinkIds", &te::Environment::getLinkIds, NoGil(), "Every link id.")
        .def("getActiveLinkIds", [](const te::Environment& self) { return self.getActiveLinkIds(); }, NoGil(),
             "Links moved by an active joint.")
        // gh-187: the joint_ids forms walk the scene graph, which throws a bare runtime_error
        // on an unknown joint; validate for a ValueError naming every unknown joint.
        .def("getActiveLinkIds", [](const te::Environment& self, const std::vector<tc::JointId>& joint_ids) {
            validate_joint_ids_exist(self, joint_ids, "getActiveLinkIds");
            return self.getActiveLinkIds(joint_ids);
        }, "joint_ids"_a, NoGil(), "Links moved by any of `joint_ids`.")
        .def("getStaticLinkIds", [](const te::Environment& self) { return self.getStaticLinkIds(); }, NoGil(),
             "Links no active joint moves.")
        .def("getStaticLinkIds", [](const te::Environment& self, const std::vector<tc::JointId>& joint_ids) {
            validate_joint_ids_exist(self, joint_ids, "getStaticLinkIds");
            return self.getStaticLinkIds(joint_ids);
        }, "joint_ids"_a, NoGil(), "Links none of `joint_ids` moves.")
        .def("getRootLinkId", &te::Environment::getRootLinkId, NoGil(), "The scene graph's root link.")
        .def("getCurrentJointValues", [](const te::Environment& self) { return self.getCurrentJointValues(); },
             NoGil(), "The active joints' current values, in getActiveJointIds order.")
        .def("getCurrentJointValues", get_current_joint_values_by_ids, "joint_ids"_a, NoGil(),
             "The current values of `joint_ids`; a non-active id raises KeyError.")
        // Transforms
        .def("getLinkTransform", [](const te::Environment& self, const tc::LinkId& link_id) {
            require_link(self, link_id);
            return self.getLinkTransform(link_id);
        }, "link_id"_a, NoGil(), "The link's current world transform; an unknown link raises KeyError.")
        .def("getRelativeLinkTransform", [](const te::Environment& self, const tc::LinkId& from_link_id,
                                            const tc::LinkId& to_link_id) {
            require_link(self, from_link_id);
            require_link(self, to_link_id);
            return self.getRelativeLinkTransform(from_link_id, to_link_id);
        }, "from_link_id"_a, "to_link_id"_a, NoGil(),
           "The transform of `to_link_id` in the frame of `from_link_id`; an unknown link raises KeyError.")
        // gh-188: getLinkTransforms. The (ids, values[, floating_joints]) overloads return the
        // C++ out-param (accepted out-param rule) and validate like setState (GH #43): they feed
        // the same state-solver path.
        .def("getLinkTransforms", [](const te::Environment& self) { return self.getLinkTransforms(); }, NoGil(),
             "Every link's current world transform.")
        .def("getLinkTransforms", [](const te::Environment& self,
                                     const std::vector<tc::JointId>& joint_ids,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const tc::JointIdTransformMap& floating_joints) {
            validate_set_state(self, joint_ids, joint_values, "getLinkTransforms");
            validate_floating_joints(self, floating_joints, "getLinkTransforms");
            tc::LinkIdTransformMap link_transforms;
            self.getLinkTransforms(link_transforms, joint_ids, joint_values, floating_joints);
            return link_transforms;
        }, "joint_ids"_a, "joint_values"_a, "floating_joints"_a = tc::JointIdTransformMap{}, NoGil(),
           "Every link's world transform at `joint_values` for `joint_ids`, without changing the environment.")
        .def("getCurrentFloatingJointValues", [](const te::Environment& self) {
            return self.getCurrentFloatingJointValues();
        }, NoGil(), "Every floating joint's current transform.")
        .def("getCurrentFloatingJointValues", [](const te::Environment& self,
                                                 const std::vector<tc::JointId>& joint_ids) {
            const std::string unknown = unknown_floating_joint_ids(self, joint_ids);
            if (!unknown.empty())
                throw nb::key_error(("Floating joint not found: " + unknown).c_str());
            return self.getCurrentFloatingJointValues(joint_ids);
        }, "joint_ids"_a, NoGil(), "The current transforms of `joint_ids`; a non-floating id raises KeyError.")
        // gh-188: id-keyed getters raise KeyError on a miss instead of None / an unasked-for bool.
        // getJointLimits copies: the C++ getter hands out a pointer into the scene graph.
        .def("getJointLimits", [](const te::Environment& self, const tc::JointId& joint_id) {
            auto ptr = self.getJointLimits(joint_id);
            if (!ptr) throw nb::key_error(("Joint not found: " + joint_id.name()).c_str());
            return tsg::JointLimits(*ptr);
        }, "joint_id"_a, NoGil(), "A copy of the joint's limits; an unknown joint raises KeyError.")
        .def("getLinkCollisionEnabled", [](const te::Environment& self, const tc::LinkId& link_id) {
            require_link(self, link_id);
            return self.getLinkCollisionEnabled(link_id);
        }, "link_id"_a, NoGil(), "Whether the link takes part in collision checks; KeyError for an unknown link.")
        .def("getLinkVisibility", [](const te::Environment& self, const tc::LinkId& link_id) {
            require_link(self, link_id);
            return self.getLinkVisibility(link_id);
        }, "link_id"_a, NoGil(), "Whether the link is visible; KeyError for an unknown link.")
        .def("getContactManagersPluginInfo", &te::Environment::getContactManagersPluginInfo, NoGil(),
             "The contact-manager plugin configuration.")
        // The scene graph's own Link / Joint, shared: a command that removes or replaces one
        // leaves the Python object valid (it keeps the old one alive).
        .def("getLink", [](const te::Environment& self, const tc::LinkId& link_id) {
            auto link = self.getLink(link_id);
            if (!link) throw nb::key_error(("Link not found: " + link_id.name()).c_str());
            return link;
        }, "link_id"_a, NoGil(), "The link (shared, read-only); an unknown link raises KeyError.")
        .def("getJoint", [](const te::Environment& self, const tc::JointId& joint_id) {
            auto joint = self.getJoint(joint_id);
            if (!joint) throw nb::key_error(("Joint not found: " + joint_id.name()).c_str());
            return joint;
        }, "joint_id"_a, NoGil(), "The joint (shared, read-only); an unknown joint raises KeyError.")
        // Groups
        .def("getGroupNames", &te::Environment::getGroupNames, NoGil(), "The SRDF group names.")
        .def("getGroupJointIds", [](const te::Environment& self, const std::string& group_name) {
            require_group(self, group_name);
            return self.getGroupJointIds(group_name);
        }, "group_name"_a, NoGil(), "The group's joint ids; an unknown group raises KeyError.")
        // Groups and contact managers come back by unique/shared_ptr, owned by Python.
        // keep_alive<0, 1>: they are built from plugin instances (OPW/KDL/UR kinematics,
        // bullet/fcl collision) whose code lives in dylibs the Environment's plugin loader owns.
        // If Python destroyed the Environment first, the loader would dlclose those dylibs and the
        // survivors' destructors would call into unmapped pages (gh-72).
        .def("getJointGroup", [](const te::Environment& self, const std::string& group_name) {
            require_group(self, group_name);
            return self.getJointGroup(group_name);
        }, "group_name"_a, nb::keep_alive<0, 1>(), NoGil(),
           "The SRDF group as a JointGroup at the current state; an unknown group raises KeyError.")
        .def("getJointGroup", [](const te::Environment& self, const std::string& group_name,
                                 const std::vector<tc::JointId>& joint_ids) {
            validate_joint_ids_exist(self, joint_ids, "getJointGroup");
            return self.getJointGroup(group_name, joint_ids);
        }, "group_name"_a, "joint_ids"_a, nb::keep_alive<0, 1>(), NoGil(),
           "A new JointGroup named `group_name` over `joint_ids`; unknown joints raise ValueError.")
        .def("getKinematicGroup", [](const te::Environment& self, const std::string& group_name,
                                      const std::string& ik_solver_name) {
            require_ik_solver(self, group_name, ik_solver_name);
            auto group = self.getKinematicGroup(group_name, ik_solver_name);
            if (!group)
                throw std::runtime_error("getKinematicGroup: inverse kinematics solver '" + ik_solver_name +
                                         "' of group '" + group_name + "' is configured but failed to load");
            return group;
        }, "group_name"_a, "ik_solver_name"_a = "", nb::keep_alive<0, 1>(), NoGil(),
           "The SRDF group with its IK solver (the group's default when `ik_solver_name` is empty). An "
           "unknown group or solver raises KeyError; a configured solver that fails to load, RuntimeError.")
        // TCP. The callbacks take the GIL themselves.
        .def("findTCPOffset", &te::Environment::findTCPOffset, "manip_info"_a, NoGil(),
             "The TCP offset `manip_info.tcp_offset` names: a transform, a link, a group TCP, or what a "
             "find-TCP callback returns; RuntimeError when none locates it.")
        .def("addFindTCPOffsetCallback", [](te::Environment& self, PyFindTCPOffsetCallable fn) {
            const PyFindTCPOffsetFn callback{ gil_safe_object(std::move(fn)) };
            nb::gil_scoped_release nogil;
            self.addFindTCPOffsetCallback(callback);
        }, "fn"_a,
        "Register `fn(manip_info) -> Isometry3d` as a source of TCP offsets for `findTCPOffset`.\n\n"
        "`findTCPOffset` asks the callbacks, in registration order, only for a `tcp_offset` name that "
        "is neither an `Isometry3d`, a link, nor a group TCP of `manip_info.manipulator`. `clone()` "
        "copies the callbacks, and the environment keeps `fn` alive.\n\n"
        "A callback that raises, or returns anything but an `Isometry3d` (`None` included), counts "
        "as \"not located\": `findTCPOffset` tries the next one and raises `RuntimeError` "
        "(\"Could not find tcp by name ...\") when none returns a transform. The callback's own "
        "exception is not reported (upstream catches it with `catch (...)`).\n\n"
        "`fn` may run on any thread, under this environment's lock: it must not change this "
        "environment, and must not keep it alive itself (a closure over it, or an attribute holding "
        "it): the environment keeps `fn` alive from C++, so the garbage collector cannot see that "
        "cycle.\n\n"
        "Args:\n"
        "    fn: Callable taking the `ManipulatorInfo` and returning the TCP offset.")
        // Contact managers. On the first call these build every collision shape in the scene.
        .def("getDiscreteContactManager", [](const te::Environment& self) {
            return self.getDiscreteContactManager();
        }, nb::keep_alive<0, 1>(), NoGil(), "A clone of the active discrete contact manager, at the current state.")
        .def("getContinuousContactManager", [](const te::Environment& self) {
            return self.getContinuousContactManager();
        }, nb::keep_alive<0, 1>(), NoGil(), "A clone of the active continuous contact manager, at the current state.")
        .def("getDiscreteContactManager", [](const te::Environment& self, const std::string& name) {
            require_discrete_contact_manager(self, name);
            auto manager = self.getDiscreteContactManager(name);
            if (!manager)
                throw std::runtime_error("getDiscreteContactManager: plugin '" + name + "' is registered but failed to load");
            return manager;
        }, "name"_a, nb::keep_alive<0, 1>(), NoGil(),
           "A new discrete contact manager from plugin `name`; an unregistered name raises KeyError.")
        .def("getContinuousContactManager", [](const te::Environment& self, const std::string& name) {
            require_continuous_contact_manager(self, name);
            auto manager = self.getContinuousContactManager(name);
            if (!manager)
                throw std::runtime_error("getContinuousContactManager: plugin '" + name + "' is registered but failed to load");
            return manager;
        }, "name"_a, nb::keep_alive<0, 1>(), NoGil(),
           "A new continuous contact manager from plugin `name`; an unregistered name raises KeyError.")
        .def("setActiveDiscreteContactManager", &te::Environment::setActiveDiscreteContactManager, "name"_a, NoGil(),
             "Make plugin `name` the active discrete contact manager; False if it is not registered.")
        .def("setActiveContinuousContactManager", &te::Environment::setActiveContinuousContactManager, "name"_a,
             NoGil(), "Make plugin `name` the active continuous contact manager; False if it is not registered.")
        .def("clearCachedDiscreteContactManager", &te::Environment::clearCachedDiscreteContactManager, NoGil(),
             "Drop the cached discrete manager: the next call rebuilds it.")
        .def("clearCachedContinuousContactManager", &te::Environment::clearCachedContinuousContactManager, NoGil(),
             "Drop the cached continuous manager: the next call rebuilds it.")
        // ACM and collision
        .def("getAllowedCollisionMatrix", &te::Environment::getAllowedCollisionMatrix, NoGil(),
             "The allowed collision matrix (shared, read-only).")
        .def("getCollisionMarginData", &te::Environment::getCollisionMarginData, NoGil(),
             "The collision margins (shared, read-only).")
        // Locator
        .def("setResourceLocator", &te::Environment::setResourceLocator, "locator"_a, NoGil(),
             "Replace the resource locator commands resolve URLs with.")
        .def("getResourceLocator", &te::Environment::getResourceLocator, NoGil(), "The resource locator.")
        .def("getKinematicsInformation", &te::Environment::getKinematicsInformation, NoGil(),
             "The SRDF kinematics information: groups, group states, TCPs and IK plugin configuration.")
        .def("clone", [](const te::Environment& self) { return self.clone(); }, NoGil(),
             "A deep copy, find-TCP callbacks included and event callbacks not; it compares equal.");
    // operator== (environment.cpp) holds a shared lock on the left operand's mutex. It compares
    // initialized, the revisions, the command types (through the non-virtual Command::operator==),
    // the current state and both timestamps: two environments built separately differ by their
    // timestamps; a clone() compares equal.
    bind_value_equality(environment, NoGil());

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

    // Private test oracle (gh-183): takes ownership of env (a clone returned to Python), then on a
    // std::thread that never had a Python thread state calls findTCPOffset and destroys env. Once the
    // test has dropped the original, that thread holds the last reference to every Python find-TCP
    // callback, so both PyFindTCPOffsetFn's call and its deleter must take the GIL themselves.
    // threading.Thread cannot test this: a Python thread already holds the GIL when it calls in.
    m.def("_find_tcp_offset_on_native_thread",
        [](std::unique_ptr<te::Environment> env, tc::ManipulatorInfo manip_info) {
            Eigen::Isometry3d tcp;
            std::exception_ptr error;
            {
                nb::gil_scoped_release nogil;
                std::thread worker([&] {
                    try {
                        tcp = env->findTCPOffset(manip_info);
                    } catch (...) {
                        error = std::current_exception();
                    }
                    env.reset();
                });
                worker.join();
            }
            if (error) std::rethrow_exception(error);
            return tcp;
        }, "env"_a, "manip_info"_a,
        "Call env.findTCPOffset and destroy env on a native thread; env is unusable afterwards");
}
