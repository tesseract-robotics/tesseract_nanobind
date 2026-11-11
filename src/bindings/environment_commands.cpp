/**
 * @file environment_commands.cpp
 * @brief tesseract_environment: the commands an Environment applies
 */

#include "environment_bindings.h"

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
#include <tesseract/environment/commands/set_active_continuous_contact_manager_command.h>
#include <tesseract/environment/commands/set_active_discrete_contact_manager_command.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/srdf/srdf_model.h>
#include <tesseract/common/manipulator_info.h>

namespace te = tesseract::environment;
namespace tsg = tesseract::scene_graph;
namespace tc = tesseract::common;

void bind_commands(nb::module_& m) {
    // ========== CommandType enum ==========
    nb::enum_<te::CommandType>(m, "CommandType")
        .value("UNINITIALIZED", te::CommandType::UNINITIALIZED)
        .value("ADD_LINK", te::CommandType::ADD_LINK)
        .value("MOVE_LINK", te::CommandType::MOVE_LINK)
        .value("MOVE_JOINT", te::CommandType::MOVE_JOINT)
        .value("REMOVE_LINK", te::CommandType::REMOVE_LINK)
        .value("REMOVE_JOINT", te::CommandType::REMOVE_JOINT)
        .value("CHANGE_LINK_ORIGIN", te::CommandType::CHANGE_LINK_ORIGIN)
        .value("CHANGE_JOINT_ORIGIN", te::CommandType::CHANGE_JOINT_ORIGIN)
        .value("CHANGE_LINK_COLLISION_ENABLED", te::CommandType::CHANGE_LINK_COLLISION_ENABLED)
        .value("CHANGE_LINK_VISIBILITY", te::CommandType::CHANGE_LINK_VISIBILITY)
        .value("MODIFY_ALLOWED_COLLISIONS", te::CommandType::MODIFY_ALLOWED_COLLISIONS)
        .value("REMOVE_ALLOWED_COLLISION_LINK", te::CommandType::REMOVE_ALLOWED_COLLISION_LINK)
        .value("ADD_SCENE_GRAPH", te::CommandType::ADD_SCENE_GRAPH)
        .value("CHANGE_JOINT_POSITION_LIMITS", te::CommandType::CHANGE_JOINT_POSITION_LIMITS)
        .value("CHANGE_JOINT_VELOCITY_LIMITS", te::CommandType::CHANGE_JOINT_VELOCITY_LIMITS)
        .value("CHANGE_JOINT_ACCELERATION_LIMITS", te::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS)
        .value("ADD_KINEMATICS_INFORMATION", te::CommandType::ADD_KINEMATICS_INFORMATION)
        .value("REPLACE_JOINT", te::CommandType::REPLACE_JOINT)
        .value("CHANGE_COLLISION_MARGINS", te::CommandType::CHANGE_COLLISION_MARGINS)
        .value("ADD_CONTACT_MANAGERS_PLUGIN_INFO", te::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO)
        .value("SET_ACTIVE_DISCRETE_CONTACT_MANAGER", te::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER)
        .value("SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER", te::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER)
        .value("ADD_TRAJECTORY_LINK", te::CommandType::ADD_TRAJECTORY_LINK);

    // ========== Command base class ==========
    // Command has a virtual destructor, so a shared_ptr<const Command> returned to Python (command
    // history, CommandAppliedEvent.commands) arrives as its most-derived bound class.
    // operator== compares only the command type. Every derived command binds its own __eq__,
    // and Python dispatches on the most-derived class, so derived commands compare their state.
    auto command = nb::class_<te::Command>(m, "Command")
        .def("getType", &te::Command::getType);
    bind_value_equality(command);

    // ========== RemoveJointCommand ==========
    auto remove_joint_command = nb::class_<te::RemoveJointCommand, te::Command>(m, "RemoveJointCommand")
        .def(nb::init<tc::JointId>(), "joint_id"_a)
        .def("getJointId", &te::RemoveJointCommand::getJointId);
    bind_value_equality(remove_joint_command);

    // ========== AddLinkCommand ==========
    auto add_link_command = nb::class_<te::AddLinkCommand, te::Command>(m, "AddLinkCommand")
        .def(nb::init<const tsg::Link&, bool>(), "link"_a, "replace_allowed"_a = false)
        .def(nb::init<const tsg::Link&, const tsg::Joint&, bool>(),
             "link"_a, "joint"_a, "replace_allowed"_a = false)
        .def("getLink", &te::AddLinkCommand::getLink)
        .def("getJoint", &te::AddLinkCommand::getJoint)
        .def("replaceAllowed", &te::AddLinkCommand::replaceAllowed);
    bind_value_equality(add_link_command);

    // ========== RemoveLinkCommand ==========
    auto remove_link_command = nb::class_<te::RemoveLinkCommand, te::Command>(m, "RemoveLinkCommand")
        .def(nb::init<tc::LinkId>(), "link_id"_a)
        .def("getLinkId", &te::RemoveLinkCommand::getLinkId);
    bind_value_equality(remove_link_command);

    // ========== AddSceneGraphCommand ==========
    auto add_scene_graph_command = nb::class_<te::AddSceneGraphCommand, te::Command>(m, "AddSceneGraphCommand")
        .def(nb::init<const tsg::SceneGraph&, std::string>(), "scene_graph"_a, "prefix"_a = "")
        .def(nb::init<const tsg::SceneGraph&, const tsg::Joint&, std::string>(),
             "scene_graph"_a, "joint"_a, "prefix"_a = "")
        .def("getSceneGraph", &te::AddSceneGraphCommand::getSceneGraph)
        .def("getJoint", &te::AddSceneGraphCommand::getJoint)
        .def("getPrefix", &te::AddSceneGraphCommand::getPrefix);
    bind_value_equality(add_scene_graph_command);

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
        .def(nb::init<tc::LinkId, tc::LinkId, tc::JointTrajectory, bool, TrajectoryLinkMethod>(),
             "link_id"_a, "parent_link_id"_a, "trajectory"_a, "replace_allowed"_a = false,
             "method"_a = TrajectoryLinkMethod::PER_STATE_OBJECTS)
        .def("getLinkId", &te::AddTrajectoryLinkCommand::getLinkId)
        .def("getParentLinkId", &te::AddTrajectoryLinkCommand::getParentLinkId)
        .def("getTrajectory", &te::AddTrajectoryLinkCommand::getTrajectory, nb::rv_policy::copy)
        .def("replaceAllowed", &te::AddTrajectoryLinkCommand::replaceAllowed)
        .def("getMethod", &te::AddTrajectoryLinkCommand::getMethod);
    bind_value_equality(add_trajectory_link_command);

    // ========== AddKinematicsInformationCommand ==========
    // Registers a KinematicsInformation (group defs + IK plugin config) into a live env.
    // insert-merges with existing kinematics info, so pre-existing groups keep resolving.
    auto add_kinematics_information_command = nb::class_<te::AddKinematicsInformationCommand, te::Command>(m, "AddKinematicsInformationCommand")
        .def(nb::init<>())
        .def(nb::init<tesseract::srdf::KinematicsInformation>(), "kinematics_information"_a)
        .def("getKinematicsInformation", &te::AddKinematicsInformationCommand::getKinematicsInformation);
    bind_value_equality(add_kinematics_information_command);

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

    // ========== ModifyAllowedCollisionsCommand ==========
    auto modify_allowed_collisions_command = nb::class_<te::ModifyAllowedCollisionsCommand, te::Command>(m, "ModifyAllowedCollisionsCommand")
        .def(nb::init<tc::AllowedCollisionMatrix, te::ModifyAllowedCollisionsType>(),
             "acm"_a, "type"_a)
        .def("getModifyType", &te::ModifyAllowedCollisionsCommand::getModifyType)
        .def("getAllowedCollisionMatrix", &te::ModifyAllowedCollisionsCommand::getAllowedCollisionMatrix);
    bind_value_equality(modify_allowed_collisions_command);

    // ========== RemoveAllowedCollisionLinkCommand ==========
    auto remove_allowed_collision_link_command = nb::class_<te::RemoveAllowedCollisionLinkCommand, te::Command>(m, "RemoveAllowedCollisionLinkCommand")
        .def(nb::init<tc::LinkId>(), "link_id"_a)
        .def("getLinkId", &te::RemoveAllowedCollisionLinkCommand::getLinkId);
    bind_value_equality(remove_allowed_collision_link_command);

    // ========== ChangeJointPositionLimitsCommand ==========
    auto change_joint_position_limits_command = nb::class_<te::ChangeJointPositionLimitsCommand, te::Command>(m, "ChangeJointPositionLimitsCommand")
        .def(nb::init<tc::JointId, double, double>(), "joint_id"_a, "lower"_a, "upper"_a)
        .def(nb::init<std::unordered_map<tc::JointId, std::pair<double, double>>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointPositionLimitsCommand::getLimits);
    bind_value_equality(change_joint_position_limits_command);

    // ========== ChangeJointVelocityLimitsCommand ==========
    auto change_joint_velocity_limits_command = nb::class_<te::ChangeJointVelocityLimitsCommand, te::Command>(m, "ChangeJointVelocityLimitsCommand")
        .def(nb::init<tc::JointId, double>(), "joint_id"_a, "limit"_a)
        .def(nb::init<std::unordered_map<tc::JointId, double>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointVelocityLimitsCommand::getLimits);
    bind_value_equality(change_joint_velocity_limits_command);

    // ========== ChangeJointAccelerationLimitsCommand ==========
    auto change_joint_acceleration_limits_command = nb::class_<te::ChangeJointAccelerationLimitsCommand, te::Command>(m, "ChangeJointAccelerationLimitsCommand")
        .def(nb::init<tc::JointId, double>(), "joint_id"_a, "limit"_a)
        .def(nb::init<std::unordered_map<tc::JointId, double>>(), "limits"_a)
        .def("getLimits", &te::ChangeJointAccelerationLimitsCommand::getLimits);
    bind_value_equality(change_joint_acceleration_limits_command);

    // ========== ChangeCollisionMarginsCommand ==========
    auto change_collision_margins_command = nb::class_<te::ChangeCollisionMarginsCommand, te::Command>(m, "ChangeCollisionMarginsCommand")
        .def(nb::init<double>(), "default_margin"_a)
        .def(nb::init<tc::CollisionMarginPairData, tc::CollisionMarginPairOverrideType>(),
             "pair_margin_data"_a, "override_type"_a = tc::CollisionMarginPairOverrideType::MODIFY)
        // #247: override_type defaults to MODIFY in both overloads, as upstream
        .def(nb::init<double, tc::CollisionMarginPairData, tc::CollisionMarginPairOverrideType>(),
             "default_margin"_a, "pair_margin_data"_a,
             "override_type"_a = tc::CollisionMarginPairOverrideType::MODIFY)
        .def("getDefaultCollisionMargin", &te::ChangeCollisionMarginsCommand::getDefaultCollisionMargin)
        .def("getCollisionMarginPairData", &te::ChangeCollisionMarginsCommand::getCollisionMarginPairData)
        .def("getCollisionMarginPairOverrideType", &te::ChangeCollisionMarginsCommand::getCollisionMarginPairOverrideType);
    bind_value_equality(change_collision_margins_command);

    // ========== ChangeLinkCollisionEnabledCommand ==========
    auto change_link_collision_enabled_command = nb::class_<te::ChangeLinkCollisionEnabledCommand, te::Command>(m, "ChangeLinkCollisionEnabledCommand")
        .def(nb::init<tc::LinkId, bool>(), "link_id"_a, "enabled"_a)
        .def("getLinkId", &te::ChangeLinkCollisionEnabledCommand::getLinkId)
        .def("getEnabled", &te::ChangeLinkCollisionEnabledCommand::getEnabled);
    bind_value_equality(change_link_collision_enabled_command);

    // ========== ChangeLinkVisibilityCommand ==========
    auto change_link_visibility_command = nb::class_<te::ChangeLinkVisibilityCommand, te::Command>(m, "ChangeLinkVisibilityCommand")
        .def(nb::init<tc::LinkId, bool>(), "link_id"_a, "visible"_a)
        .def("getLinkId", &te::ChangeLinkVisibilityCommand::getLinkId)
        .def("getEnabled", &te::ChangeLinkVisibilityCommand::getEnabled);
    bind_value_equality(change_link_visibility_command);

    // ========== ChangeJointOriginCommand ==========
    auto change_joint_origin_command = nb::class_<te::ChangeJointOriginCommand, te::Command>(m, "ChangeJointOriginCommand")
        .def(nb::init<tc::JointId, const Eigen::Isometry3d&>(), "joint_id"_a, "origin"_a)
        .def("getJointId", &te::ChangeJointOriginCommand::getJointId)
        .def("getOrigin", &te::ChangeJointOriginCommand::getOrigin);
    bind_value_equality(change_joint_origin_command);

    // ========== ChangeLinkOriginCommand ==========
    auto change_link_origin_command = nb::class_<te::ChangeLinkOriginCommand, te::Command>(m, "ChangeLinkOriginCommand")
        .def(nb::init<tc::LinkId, const Eigen::Isometry3d&>(), "link_id"_a, "origin"_a)
        .def("getLinkId", &te::ChangeLinkOriginCommand::getLinkId)
        .def("getOrigin", &te::ChangeLinkOriginCommand::getOrigin);
    bind_value_equality(change_link_origin_command);

    // ========== MoveJointCommand ==========
    auto move_joint_command = nb::class_<te::MoveJointCommand, te::Command>(m, "MoveJointCommand")
        .def(nb::init<tc::JointId, tc::LinkId>(), "joint_id"_a, "parent_link"_a)
        .def("getJointId", &te::MoveJointCommand::getJointId)
        .def("getParentLink", &te::MoveJointCommand::getParentLink);
    bind_value_equality(move_joint_command);

    // ========== MoveLinkCommand ==========
    auto move_link_command = nb::class_<te::MoveLinkCommand, te::Command>(m, "MoveLinkCommand")
        .def(nb::init<const tsg::Joint&>(), "joint"_a)
        .def("getJoint", &te::MoveLinkCommand::getJoint);
    bind_value_equality(move_link_command);

    // ========== ReplaceJointCommand ==========
    auto replace_joint_command = nb::class_<te::ReplaceJointCommand, te::Command>(m, "ReplaceJointCommand")
        .def(nb::init<const tsg::Joint&>(), "joint"_a)
        .def("getJoint", &te::ReplaceJointCommand::getJoint);
    bind_value_equality(replace_joint_command);

    // ========== SetActive{Discrete,Continuous}ContactManagerCommand ==========
    // The arity-0 ctors are serialization ctors and stay unbound.
    auto set_active_discrete_contact_manager_command = nb::class_<te::SetActiveDiscreteContactManagerCommand, te::Command>(m, "SetActiveDiscreteContactManagerCommand")
        .def(nb::init<std::string>(), "active_contact_manager"_a)
        .def("getName", &te::SetActiveDiscreteContactManagerCommand::getName);
    bind_value_equality(set_active_discrete_contact_manager_command);

    auto set_active_continuous_contact_manager_command = nb::class_<te::SetActiveContinuousContactManagerCommand, te::Command>(m, "SetActiveContinuousContactManagerCommand")
        .def(nb::init<std::string>(), "active_contact_manager"_a)
        .def("getName", &te::SetActiveContinuousContactManagerCommand::getName);
    bind_value_equality(set_active_continuous_contact_manager_command);

}
