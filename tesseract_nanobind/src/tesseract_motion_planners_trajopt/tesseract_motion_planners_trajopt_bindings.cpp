/**
 * @file tesseract_motion_planners_trajopt_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners TrajOpt
 *
 * NOTE: 0.33 API changes:
 * - TrajOptPlanProfile -> TrajOptMoveProfile
 * - TrajOptDefaultPlanProfile -> TrajOptDefaultMoveProfile
 * - CollisionCostConfig/CollisionConstraintConfig -> trajopt_common::TrajOptCollisionConfig
 * - Profile/ProfileDictionary moved to tesseract_common
 * - CollisionEvaluatorType moved to tesseract_collision
 * - contact_test_type, longest_valid_segment_* removed from TrajOptDefaultCompositeProfile
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners TrajOpt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>

// trajopt_sco for optimizer parameters
#include <trajopt_sco/optimizers.hpp>

// trajopt_common for collision config (moved here in 0.33)
#include <trajopt_common/collision_types.h>

// tesseract_collision for CollisionEvaluatorType (moved here in 0.33)
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;
namespace tj = trajopt_common;

NB_MODULE(_tesseract_motion_planners_trajopt, m) {
    m.doc() = "tesseract_motion_planners_trajopt Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for CollisionEvaluatorType (moved there in 0.33)
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Note: CollisionEvaluatorType is in tesseract_collision module - import from there

    // ========== TrajOptCartesianWaypointConfig ==========
    nb::class_<tp::TrajOptCartesianWaypointConfig>(m, "TrajOptCartesianWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptCartesianWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptCartesianWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptCartesianWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptCartesianWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptCartesianWaypointConfig::coeff);

    // ========== TrajOptJointWaypointConfig ==========
    nb::class_<tp::TrajOptJointWaypointConfig>(m, "TrajOptJointWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptJointWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptJointWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptJointWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptJointWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptJointWaypointConfig::coeff);

    // ========== TrajOptCollisionConfig ==========
    // Import from trajopt_ifopt to avoid duplicate registration
    nb::module_::import_("tesseract_robotics.trajopt_ifopt._trajopt_ifopt");

    // ========== TrajOptMoveProfile (base, was TrajOptPlanProfile) ==========
    nb::class_<tp::TrajOptMoveProfile, tc::Profile>(m, "TrajOptMoveProfile")
        .def("getKey", &tp::TrajOptMoveProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptMoveProfile::getStaticKey);

    // SWIG-compatible alias
    m.attr("TrajOptPlanProfile") = m.attr("TrajOptMoveProfile");

    // ========== TrajOptCompositeProfile (base) ==========
    nb::class_<tp::TrajOptCompositeProfile, tc::Profile>(m, "TrajOptCompositeProfile")
        .def("getKey", &tp::TrajOptCompositeProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptCompositeProfile::getStaticKey);

    // ========== TrajOptDefaultMoveProfile (was TrajOptDefaultPlanProfile) ==========
    nb::class_<tp::TrajOptDefaultMoveProfile, tp::TrajOptMoveProfile>(m, "TrajOptDefaultMoveProfile")
        .def(nb::init<>())
        .def_rw("cartesian_cost_config", &tp::TrajOptDefaultMoveProfile::cartesian_cost_config)
        .def_rw("cartesian_constraint_config", &tp::TrajOptDefaultMoveProfile::cartesian_constraint_config)
        .def_rw("joint_cost_config", &tp::TrajOptDefaultMoveProfile::joint_cost_config)
        .def_rw("joint_constraint_config", &tp::TrajOptDefaultMoveProfile::joint_constraint_config);

    // SWIG-compatible alias
    m.attr("TrajOptDefaultPlanProfile") = m.attr("TrajOptDefaultMoveProfile");

    // ========== TrajOptDefaultCompositeProfile ==========
    // Note: contact_test_type, longest_valid_segment_fraction/length removed in 0.33
    nb::class_<tp::TrajOptDefaultCompositeProfile, tp::TrajOptCompositeProfile>(m, "TrajOptDefaultCompositeProfile")
        .def(nb::init<>())
        .def_rw("collision_cost_config", &tp::TrajOptDefaultCompositeProfile::collision_cost_config)
        .def_rw("collision_constraint_config", &tp::TrajOptDefaultCompositeProfile::collision_constraint_config)
        .def_rw("smooth_velocities", &tp::TrajOptDefaultCompositeProfile::smooth_velocities)
        .def_rw("velocity_coeff", &tp::TrajOptDefaultCompositeProfile::velocity_coeff)
        .def_rw("smooth_accelerations", &tp::TrajOptDefaultCompositeProfile::smooth_accelerations)
        .def_rw("acceleration_coeff", &tp::TrajOptDefaultCompositeProfile::acceleration_coeff)
        .def_rw("smooth_jerks", &tp::TrajOptDefaultCompositeProfile::smooth_jerks)
        .def_rw("jerk_coeff", &tp::TrajOptDefaultCompositeProfile::jerk_coeff)
        .def_rw("avoid_singularity", &tp::TrajOptDefaultCompositeProfile::avoid_singularity)
        .def_rw("avoid_singularity_coeff", &tp::TrajOptDefaultCompositeProfile::avoid_singularity_coeff);

    // ========== sco::BasicTrustRegionSQPParameters ==========
    nb::class_<sco::BasicTrustRegionSQPParameters>(m, "BasicTrustRegionSQPParameters")
        .def(nb::init<>())
        .def_rw("improve_ratio_threshold", &sco::BasicTrustRegionSQPParameters::improve_ratio_threshold,
                "Minimum ratio exact_improve/approx_improve to accept step")
        .def_rw("min_trust_box_size", &sco::BasicTrustRegionSQPParameters::min_trust_box_size,
                "If trust region gets any smaller, exit and report convergence")
        .def_rw("min_approx_improve", &sco::BasicTrustRegionSQPParameters::min_approx_improve,
                "If model improves less than this, exit and report convergence")
        .def_rw("min_approx_improve_frac", &sco::BasicTrustRegionSQPParameters::min_approx_improve_frac,
                "If model improves less than this fraction, exit and report convergence")
        .def_rw("max_iter", &sco::BasicTrustRegionSQPParameters::max_iter,
                "The max number of iterations")
        .def_rw("trust_shrink_ratio", &sco::BasicTrustRegionSQPParameters::trust_shrink_ratio,
                "If improvement is less than improve_ratio_threshold, shrink trust region by this ratio")
        .def_rw("trust_expand_ratio", &sco::BasicTrustRegionSQPParameters::trust_expand_ratio,
                "If improvement is greater than improve_ratio_threshold, expand trust region by this ratio")
        .def_rw("cnt_tolerance", &sco::BasicTrustRegionSQPParameters::cnt_tolerance,
                "After convergence of penalty subproblem, if constraint violation is less than this, we're done")
        .def_rw("max_merit_coeff_increases", &sco::BasicTrustRegionSQPParameters::max_merit_coeff_increases,
                "Max number of times that the constraints' cost will be increased")
        .def_rw("max_qp_solver_failures", &sco::BasicTrustRegionSQPParameters::max_qp_solver_failures,
                "Max number of times the QP solver can fail before optimization is aborted")
        .def_rw("merit_coeff_increase_ratio", &sco::BasicTrustRegionSQPParameters::merit_coeff_increase_ratio,
                "Ratio that we increase coeff each time")
        .def_rw("max_time", &sco::BasicTrustRegionSQPParameters::max_time,
                "Max time in seconds that the optimizer will run")
        .def_rw("initial_merit_error_coeff", &sco::BasicTrustRegionSQPParameters::initial_merit_error_coeff,
                "Initial coefficient that is used to scale the constraints")
        .def_rw("inflate_constraints_individually", &sco::BasicTrustRegionSQPParameters::inflate_constraints_individually,
                "If true, merit coeffs will only be inflated for the constraints that failed")
        .def_rw("trust_box_size", &sco::BasicTrustRegionSQPParameters::trust_box_size,
                "Current size of trust region (component-wise)")
        .def_rw("log_results", &sco::BasicTrustRegionSQPParameters::log_results,
                "Log results to file")
        .def_rw("log_dir", &sco::BasicTrustRegionSQPParameters::log_dir,
                "Directory to store log results")
        .def_rw("num_threads", &sco::BasicTrustRegionSQPParameters::num_threads,
                "If greater than one, multi threaded functions are called");

    // ========== TrajOptSolverProfile (base) ==========
    nb::class_<tp::TrajOptSolverProfile, tc::Profile>(m, "TrajOptSolverProfile")
        .def_rw("opt_params", &tp::TrajOptSolverProfile::opt_params,
                "Optimization parameters")
        .def("getKey", &tp::TrajOptSolverProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptSolverProfile::getStaticKey);

    // ========== TrajOptOSQPSolverProfile ==========
    nb::class_<tp::TrajOptOSQPSolverProfile, tp::TrajOptSolverProfile>(m, "TrajOptOSQPSolverProfile")
        .def(nb::init<>())
        .def_rw("update_workspace", &tp::TrajOptOSQPSolverProfile::update_workspace,
                "Update the OSQP workspace for subsequent optimizations, instead of recreating it each time");

    // Helper to add TrajOpt move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptMoveProfile", [](tc::ProfileDictionary& dict,
                                                         const std::string& ns,
                                                         const std::string& profile_name,
                                                         std::shared_ptr<tp::TrajOptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt move profile to ProfileDictionary");

    // SWIG-compatible alias
    m.def("ProfileDictionary_addTrajOptPlanProfile", [](tc::ProfileDictionary& dict,
                                                         const std::string& ns,
                                                         const std::string& profile_name,
                                                         std::shared_ptr<tp::TrajOptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt plan profile to ProfileDictionary (legacy alias)");

    // Helper to add TrajOpt composite profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptCompositeProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptCompositeProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt composite profile to ProfileDictionary");

    // Helper to add TrajOpt solver profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptSolverProfile", [](tc::ProfileDictionary& dict,
                                                           const std::string& ns,
                                                           const std::string& profile_name,
                                                           std::shared_ptr<tp::TrajOptSolverProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt solver profile to ProfileDictionary");

    // ========== TrajOptMotionPlanner ==========
    nb::class_<tp::TrajOptMotionPlanner>(m, "TrajOptMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::TrajOptMotionPlanner::getName)
        .def("solve", &tp::TrajOptMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::TrajOptMotionPlanner::terminate)
        .def("clear", &tp::TrajOptMotionPlanner::clear);
}
