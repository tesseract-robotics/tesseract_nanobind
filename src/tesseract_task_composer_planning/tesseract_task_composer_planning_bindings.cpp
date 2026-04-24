/**
 * @file tesseract_task_composer_planning_bindings.cpp
 * @brief nanobind bindings for tesseract_task_composer planning-node profiles.
 *
 * The task classes themselves are constructed via YAML + TaskComposerPluginFactory;
 * only their *profiles* need to be tunable from Python, so just the profiles
 * are bound here.
 *
 * FixStateCollisionProfile has members of type OSQPSettings,
 * sco::BasicTrustRegionSQPParameters, trajopt_common::CollisionCoeffData, and
 * TrajOptJointWaypointConfig that are not currently exposed to Python. Those
 * members are skipped; users who need to tune them should build the profile
 * in C++ or configure via YAML. Simple scalar / enum fields are exposed.
 */

#include "tesseract_nb.h"
#include <nanobind/stl/vector.h>

#include <tesseract_common/profile.h>

#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/planning/profiles/kinematic_limits_check_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_task_composer/planning/profiles/profile_switch_profile.h>
#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_task_composer_planning, m) {
    m.doc() = "tesseract_task_composer planning-node profile Python bindings";

    // Import Profile from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    // Import tesseract_collision for ContactManagerConfig / CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ===== ContactCheckProfile =====
    nb::class_<tp::ContactCheckProfile, tc::Profile>(m, "ContactCheckProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "longest_valid_segment_length"_a, "contact_distance"_a)
        .def_rw("contact_manager_config", &tp::ContactCheckProfile::contact_manager_config)
        .def_rw("collision_check_config", &tp::ContactCheckProfile::collision_check_config);

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
        .def_rw("lower_bounds_reduction", &tp::FixStateBoundsProfile::lower_bounds_reduction);

    // ===== FixStateCollisionProfile =====
    // Heavy trajopt-typed members (opt_params, osqp_settings, coeff data,
    // trajopt waypoint configs) are not bound — build those in C++ or YAML.
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
        .def_rw("update_workspace", &tp::FixStateCollisionProfile::update_workspace);

    // ===== KinematicLimitsCheckProfile =====
    nb::class_<tp::KinematicLimitsCheckProfile, tc::Profile>(m, "KinematicLimitsCheckProfile")
        .def(nb::init<bool, bool, bool>(),
             "check_position"_a = true,
             "check_velocity"_a = true,
             "check_acceleration"_a = true)
        .def_rw("check_position", &tp::KinematicLimitsCheckProfile::check_position)
        .def_rw("check_velocity", &tp::KinematicLimitsCheckProfile::check_velocity)
        .def_rw("check_acceleration", &tp::KinematicLimitsCheckProfile::check_acceleration);

    // ===== MinLengthProfile =====
    nb::class_<tp::MinLengthProfile, tc::Profile>(m, "MinLengthProfile")
        .def(nb::init<>())
        .def(nb::init<long>(), "min_length"_a)
        .def_rw("min_length", &tp::MinLengthProfile::min_length);

    // ===== ProfileSwitchProfile =====
    nb::class_<tp::ProfileSwitchProfile, tc::Profile>(m, "ProfileSwitchProfile")
        .def(nb::init<int>(), "return_value"_a = 1)
        .def_rw("return_value", &tp::ProfileSwitchProfile::return_value);

    // ===== UpsampleTrajectoryProfile =====
    nb::class_<tp::UpsampleTrajectoryProfile, tc::Profile>(m, "UpsampleTrajectoryProfile")
        .def(nb::init<>())
        .def(nb::init<double>(), "longest_valid_segment_length"_a)
        .def_rw("longest_valid_segment_length", &tp::UpsampleTrajectoryProfile::longest_valid_segment_length);
}
