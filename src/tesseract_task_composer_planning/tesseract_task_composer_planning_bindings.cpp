/**
 * @file tesseract_task_composer_planning_bindings.cpp
 * @brief nanobind bindings for tesseract_task_composer planning-node profiles.
 *
 * The task classes themselves are constructed via YAML + TaskComposerPluginFactory;
 * only their *profiles* need to be tunable from Python, so just the profiles
 * are bound here.
 */

#include "tesseract_nb.h"
#include "tesseract_nb_yaml.h"
#include <nanobind/stl/vector.h>

#include <tesseract/common/profile.h>

#include <tesseract/task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract/task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract/task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract/task_composer/planning/profiles/kinematic_limits_check_profile.h>
#include <tesseract/task_composer/planning/profiles/min_length_profile.h>
#include <tesseract/task_composer/planning/profiles/profile_switch_profile.h>
#include <tesseract/task_composer/planning/profiles/upsample_trajectory_profile.h>

namespace tp = tesseract::task_composer;
namespace tc = tesseract::common;

NB_MODULE(_tesseract_task_composer_planning, m) {
    m.doc() = "tesseract_task_composer planning-node profile Python bindings";

    // Import Profile from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    // Import tesseract_collision for ContactManagerConfig / CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");
    // FixStateCollisionProfile's TrajOpt correction fields: TrajOptJointWaypointConfig,
    // BasicTrustRegionSQPParameters and OSQPSettings (planner module), CollisionCoeffData (trajopt_ifopt)
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners_trajopt._tesseract_motion_planners_trajopt");
    nb::module_::import_("tesseract_robotics.trajopt_ifopt._trajopt_ifopt");

    // ===== ContactCheckProfile =====
    nb::class_<tp::ContactCheckProfile, tc::Profile> contact_check(m, "ContactCheckProfile");
    contact_check
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "longest_valid_segment_length"_a, "contact_distance"_a)
        .def_rw("contact_manager_config", &tp::ContactCheckProfile::contact_manager_config)
        .def_rw("collision_check_config", &tp::ContactCheckProfile::collision_check_config)
        .def_static("createKey", &tc::Profile::createKey<tp::ContactCheckProfile>);
    bind_value_equality(contact_check);
    bind_from_yaml(contact_check,
        "Nested sub-configs are replaced, not merged: a key naming one rebuilds it from that "
        "struct's own defaults, so fields left out lose the profile's values. This is per "
        "struct; here `collision_check_config:` resets type to DISCRETE and "
        "longest_valid_segment_length to 0.005, and `contact_manager_config:` resets "
        "default_margin to None.");

    // ===== FixStateBoundsProfile =====
    nb::class_<tp::FixStateBoundsProfile, tc::Profile> fix_state_bounds(m, "FixStateBoundsProfile");
    nb::enum_<tp::FixStateBoundsProfile::Settings>(fix_state_bounds, "Settings")
        .value("START_ONLY", tp::FixStateBoundsProfile::Settings::START_ONLY)
        .value("END_ONLY", tp::FixStateBoundsProfile::Settings::END_ONLY)
        .value("ALL", tp::FixStateBoundsProfile::Settings::ALL)
        .value("DISABLED", tp::FixStateBoundsProfile::Settings::DISABLED);
    fix_state_bounds
        .def(nb::init<tp::FixStateBoundsProfile::Settings>(),
             "mode"_a = tp::FixStateBoundsProfile::Settings::ALL)
        .def_rw("mode", &tp::FixStateBoundsProfile::mode)
        .def_rw("max_deviation_global", &tp::FixStateBoundsProfile::max_deviation_global)
        .def_rw("upper_bounds_reduction", &tp::FixStateBoundsProfile::upper_bounds_reduction)
        .def_rw("lower_bounds_reduction", &tp::FixStateBoundsProfile::lower_bounds_reduction)
        .def_static("createKey", &tc::Profile::createKey<tp::FixStateBoundsProfile>);
    bind_value_equality(fix_state_bounds);
    bind_from_yaml(fix_state_bounds,
        "No nested sub-configs: every key is a field of the profile itself. Other profiles' "
        "from_yaml behave per struct.");

    // ===== FixStateCollisionProfile =====
    nb::class_<tp::FixStateCollisionProfile, tc::Profile> fix_state_collision(m, "FixStateCollisionProfile");
    nb::enum_<tp::FixStateCollisionProfile::Settings>(fix_state_collision, "Settings")
        .value("START_ONLY", tp::FixStateCollisionProfile::Settings::START_ONLY)
        .value("END_ONLY", tp::FixStateCollisionProfile::Settings::END_ONLY)
        .value("INTERMEDIATE_ONLY", tp::FixStateCollisionProfile::Settings::INTERMEDIATE_ONLY)
        .value("ALL", tp::FixStateCollisionProfile::Settings::ALL)
        .value("ALL_EXCEPT_START", tp::FixStateCollisionProfile::Settings::ALL_EXCEPT_START)
        .value("ALL_EXCEPT_END", tp::FixStateCollisionProfile::Settings::ALL_EXCEPT_END)
        .value("DISABLED", tp::FixStateCollisionProfile::Settings::DISABLED);
    nb::enum_<tp::FixStateCollisionProfile::CorrectionMethod>(fix_state_collision, "CorrectionMethod")
        .value("NONE", tp::FixStateCollisionProfile::CorrectionMethod::NONE)
        .value("TRAJOPT", tp::FixStateCollisionProfile::CorrectionMethod::TRAJOPT)
        .value("RANDOM_SAMPLER", tp::FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER);
    fix_state_collision
        .def(nb::init<tp::FixStateCollisionProfile::Settings>(),
             "mode"_a = tp::FixStateCollisionProfile::Settings::ALL)
        .def_rw("mode", &tp::FixStateCollisionProfile::mode)
        .def_rw("correction_workflow", &tp::FixStateCollisionProfile::correction_workflow)
        .def_rw("jiggle_factor", &tp::FixStateCollisionProfile::jiggle_factor)
        .def_rw("contact_manager_config", &tp::FixStateCollisionProfile::contact_manager_config)
        .def_rw("collision_check_config", &tp::FixStateCollisionProfile::collision_check_config)
        .def_rw("sampling_attempts", &tp::FixStateCollisionProfile::sampling_attempts)
        .def_rw("update_workspace", &tp::FixStateCollisionProfile::update_workspace)
        .def_rw("trajopt_joint_constraint_config", &tp::FixStateCollisionProfile::trajopt_joint_constraint_config,
                "Joint waypoint constraint config of the TrajOpt correction solve")
        .def_rw("trajopt_joint_cost_config", &tp::FixStateCollisionProfile::trajopt_joint_cost_config,
                "Joint waypoint cost config of the TrajOpt correction solve")
        .def_rw("collision_constraint_coeff", &tp::FixStateCollisionProfile::collision_constraint_coeff,
                "Collision constraint coefficients of the TrajOpt correction solve")
        .def_rw("collision_cost_coeff", &tp::FixStateCollisionProfile::collision_cost_coeff,
                "Collision cost coefficients of the TrajOpt correction solve")
        .def_rw("opt_params", &tp::FixStateCollisionProfile::opt_params,
                "Optimization parameters of the TrajOpt correction solve")
        .def_rw("osqp_settings", &tp::FixStateCollisionProfile::osqp_settings,
                "OSQP settings of the TrajOpt correction solve")
        .def_static("createKey", &tc::Profile::createKey<tp::FixStateCollisionProfile>);
    bind_value_equality(fix_state_collision);
    bind_from_yaml(fix_state_collision,
        "Nested sub-configs are replaced, not merged: a key naming one rebuilds it from that "
        "struct's own defaults, so fields left out lose the profile's values. This is per "
        "struct; here `collision_check_config:` resets contact_request.type to ALL, and "
        "`trajopt_joint_constraint_config:` resets coeff to 5. `contact_manager_config:`, "
        "`opt_params:` and `osqp_settings:` keep the profile's defaults.");

    // ===== KinematicLimitsCheckProfile =====
    nb::class_<tp::KinematicLimitsCheckProfile, tc::Profile> kinematic_limits_check(m, "KinematicLimitsCheckProfile");
    kinematic_limits_check
        .def(nb::init<bool, bool, bool>(),
             "check_position"_a = true,
             "check_velocity"_a = true,
             "check_acceleration"_a = true)
        .def_rw("check_position", &tp::KinematicLimitsCheckProfile::check_position)
        .def_rw("check_velocity", &tp::KinematicLimitsCheckProfile::check_velocity)
        .def_rw("check_acceleration", &tp::KinematicLimitsCheckProfile::check_acceleration)
        .def_static("createKey", &tc::Profile::createKey<tp::KinematicLimitsCheckProfile>);
    bind_value_equality(kinematic_limits_check);
    bind_from_yaml(kinematic_limits_check,
        "No nested sub-configs: every key is a field of the profile itself. Other profiles' "
        "from_yaml behave per struct.");

    // ===== MinLengthProfile =====
    nb::class_<tp::MinLengthProfile, tc::Profile> min_length(m, "MinLengthProfile");
    min_length
        .def(nb::init<>())
        .def(nb::init<long>(), "min_length"_a)
        .def_rw("min_length", &tp::MinLengthProfile::min_length)
        .def_static("createKey", &tc::Profile::createKey<tp::MinLengthProfile>);
    bind_value_equality(min_length);
    bind_from_yaml(min_length,
        "No nested sub-configs: every key is a field of the profile itself. Other profiles' "
        "from_yaml behave per struct.");

    // ===== ProfileSwitchProfile =====
    nb::class_<tp::ProfileSwitchProfile, tc::Profile> profile_switch(m, "ProfileSwitchProfile");
    profile_switch
        .def(nb::init<int>(), "return_value"_a = 1)
        .def_rw("return_value", &tp::ProfileSwitchProfile::return_value)
        .def_static("createKey", &tc::Profile::createKey<tp::ProfileSwitchProfile>);
    bind_value_equality(profile_switch);
    bind_from_yaml(profile_switch,
        "No nested sub-configs: every key is a field of the profile itself. Other profiles' "
        "from_yaml behave per struct.");

    // ===== UpsampleTrajectoryProfile =====
    nb::class_<tp::UpsampleTrajectoryProfile, tc::Profile> upsample_trajectory(m, "UpsampleTrajectoryProfile");
    upsample_trajectory
        .def(nb::init<>())
        .def(nb::init<double>(), "longest_valid_segment_length"_a)
        .def_rw("longest_valid_segment_length", &tp::UpsampleTrajectoryProfile::longest_valid_segment_length)
        .def_static("createKey", &tc::Profile::createKey<tp::UpsampleTrajectoryProfile>);
    bind_value_equality(upsample_trajectory);
    bind_from_yaml(upsample_trajectory,
        "No nested sub-configs: every key is a field of the profile itself. Other profiles' "
        "from_yaml behave per struct.");
}
