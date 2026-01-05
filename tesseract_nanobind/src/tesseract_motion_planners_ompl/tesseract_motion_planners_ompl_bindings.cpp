/**
 * @file tesseract_motion_planners_ompl_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners OMPL
 *
 * NOTE: 0.33 API changes:
 * - OMPLPlanProfile -> OMPLMoveProfile
 * - OMPLRealVectorPlanProfile -> OMPLRealVectorMoveProfile
 * - Profile/ProfileDictionary moved to tesseract_common
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners OMPL
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/ompl_solver_config.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_ompl, m) {
    m.doc() = "tesseract_motion_planners_ompl Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    auto profile_type = cl_module.attr("Profile");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== OMPLSolverConfig ==========
    nb::class_<tp::OMPLSolverConfig>(m, "OMPLSolverConfig")
        .def(nb::init<>())
        .def_rw("planning_time", &tp::OMPLSolverConfig::planning_time,
            "Max planning time allowed in seconds (default: 5.0)")
        .def_rw("max_solutions", &tp::OMPLSolverConfig::max_solutions,
            "Max number of solutions to find before exiting (default: 10)")
        .def_rw("simplify", &tp::OMPLSolverConfig::simplify,
            "Simplify trajectory (default: false). Ignores n_output_states if true.")
        .def_rw("optimize", &tp::OMPLSolverConfig::optimize,
            "Use all planning time to optimize trajectory (default: true)")
        .def("addPlanner", [](tp::OMPLSolverConfig& self, std::shared_ptr<tp::OMPLPlannerConfigurator> planner) {
            self.planners.push_back(planner);
        }, "planner"_a, "Add a planner configurator (each runs in parallel)")
        .def("clearPlanners", [](tp::OMPLSolverConfig& self) {
            self.planners.clear();
        }, "Clear all planners")
        .def("getNumPlanners", [](const tp::OMPLSolverConfig& self) {
            return self.planners.size();
        }, "Get number of planners (threads)");

    // ========== OMPLPlannerType enum ==========
    nb::enum_<tp::OMPLPlannerType>(m, "OMPLPlannerType")
        .value("SBL", tp::OMPLPlannerType::SBL)
        .value("EST", tp::OMPLPlannerType::EST)
        .value("LBKPIECE1", tp::OMPLPlannerType::LBKPIECE1)
        .value("BKPIECE1", tp::OMPLPlannerType::BKPIECE1)
        .value("KPIECE1", tp::OMPLPlannerType::KPIECE1)
        .value("BiTRRT", tp::OMPLPlannerType::BiTRRT)
        .value("RRT", tp::OMPLPlannerType::RRT)
        .value("RRTConnect", tp::OMPLPlannerType::RRTConnect)
        .value("RRTstar", tp::OMPLPlannerType::RRTstar)
        .value("TRRT", tp::OMPLPlannerType::TRRT)
        .value("PRM", tp::OMPLPlannerType::PRM)
        .value("PRMstar", tp::OMPLPlannerType::PRMstar)
        .value("LazyPRMstar", tp::OMPLPlannerType::LazyPRMstar)
        .value("SPARS", tp::OMPLPlannerType::SPARS);

    // ========== OMPLPlannerConfigurator (base) ==========
    nb::class_<tp::OMPLPlannerConfigurator>(m, "OMPLPlannerConfigurator")
        .def("getType", &tp::OMPLPlannerConfigurator::getType);

    // ========== RRTConnectConfigurator ==========
    nb::class_<tp::RRTConnectConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConnectConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConnectConfigurator::range);

    // ========== RRTstarConfigurator ==========
    nb::class_<tp::RRTstarConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTstarConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTstarConfigurator::range)
        .def_rw("goal_bias", &tp::RRTstarConfigurator::goal_bias)
        .def_rw("delay_collision_checking", &tp::RRTstarConfigurator::delay_collision_checking);

    // ========== SBLConfigurator ==========
    nb::class_<tp::SBLConfigurator, tp::OMPLPlannerConfigurator>(m, "SBLConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::SBLConfigurator::range);

    // ========== OMPLMoveProfile (base, was OMPLPlanProfile) ==========
    nb::class_<tp::OMPLMoveProfile, tc::Profile>(m, "OMPLMoveProfile")
        .def("getKey", &tp::OMPLMoveProfile::getKey)
        .def_static("getStaticKey", &tp::OMPLMoveProfile::getStaticKey);

    // SWIG-compatible alias
    m.attr("OMPLPlanProfile") = m.attr("OMPLMoveProfile");

    // ========== OMPLRealVectorMoveProfile (was OMPLRealVectorPlanProfile) ==========
    nb::class_<tp::OMPLRealVectorMoveProfile, tp::OMPLMoveProfile>(m, "OMPLRealVectorMoveProfile")
        .def(nb::init<>())
        .def_rw("solver_config", &tp::OMPLRealVectorMoveProfile::solver_config,
            "The OMPL parallel planner solver config")
        .def_rw("collision_check_config", &tp::OMPLRealVectorMoveProfile::collision_check_config,
            "The collision check configuration");

    // SWIG-compatible alias
    m.attr("OMPLRealVectorPlanProfile") = m.attr("OMPLRealVectorMoveProfile");

    // Helper to convert OMPLMoveProfile to Profile::ConstPtr for ProfileDictionary
    m.def("OMPLMoveProfile_as_ProfileConstPtr", [](std::shared_ptr<tp::OMPLMoveProfile> profile) -> tc::Profile::ConstPtr {
        return profile;
    }, "profile"_a, "Convert OMPLMoveProfile to Profile::ConstPtr for use with ProfileDictionary.addProfile");

    // Legacy alias
    m.def("OMPLPlanProfile_as_ProfileConstPtr", [](std::shared_ptr<tp::OMPLMoveProfile> profile) -> tc::Profile::ConstPtr {
        return profile;
    }, "profile"_a, "Convert OMPLPlanProfile to Profile::ConstPtr (legacy alias)");

    // Helper to add OMPL move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addOMPLMoveProfile", [](tc::ProfileDictionary& dict,
                                                  const std::string& ns,
                                                  const std::string& profile_name,
                                                  std::shared_ptr<tp::OMPLMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add OMPL move profile to ProfileDictionary");

    // Legacy alias
    m.def("ProfileDictionary_addOMPLProfile", [](tc::ProfileDictionary& dict,
                                                  const std::string& ns,
                                                  const std::string& profile_name,
                                                  std::shared_ptr<tp::OMPLMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add OMPL plan profile to ProfileDictionary (legacy alias)");

    // ========== OMPLMotionPlanner ==========
    nb::class_<tp::OMPLMotionPlanner>(m, "OMPLMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::OMPLMotionPlanner::getName)
        .def("solve", &tp::OMPLMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::OMPLMotionPlanner::terminate)
        .def("clear", &tp::OMPLMotionPlanner::clear);
}
