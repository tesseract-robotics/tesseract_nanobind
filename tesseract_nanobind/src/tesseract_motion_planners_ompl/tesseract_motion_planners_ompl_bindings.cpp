/**
 * @file tesseract_motion_planners_ompl_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners OMPL
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (for Profile base class and ProfileDictionary)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners OMPL (0.33.x: Plan -> Move profile rename)
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_ompl, m) {
    m.doc() = "tesseract_motion_planners_ompl Python bindings";

    // Import tesseract_common for Profile base class (cross-module inheritance)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

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

    // ========== OMPLMoveProfile (base) - renamed from OMPLPlanProfile in 0.33.x ==========
    nb::class_<tp::OMPLMoveProfile, tc::Profile>(m, "OMPLMoveProfile")
        .def("getKey", &tp::OMPLMoveProfile::getKey)
        .def_static("getStaticKey", &tp::OMPLMoveProfile::getStaticKey);

    // Alias for backwards compatibility
    m.attr("OMPLPlanProfile") = m.attr("OMPLMoveProfile");

    // ========== OMPLRealVectorMoveProfile - renamed from OMPLRealVectorPlanProfile in 0.33.x ==========
    nb::class_<tp::OMPLRealVectorMoveProfile, tp::OMPLMoveProfile>(m, "OMPLRealVectorMoveProfile")
        .def(nb::init<>());

    // Alias for backwards compatibility
    m.attr("OMPLRealVectorPlanProfile") = m.attr("OMPLRealVectorMoveProfile");

    // Helper to convert OMPLMoveProfile to Profile::ConstPtr for ProfileDictionary
    m.def("OMPLMoveProfile_as_ProfileConstPtr", [](std::shared_ptr<tp::OMPLMoveProfile> profile) -> tc::Profile::ConstPtr {
        return profile;
    }, "profile"_a, "Convert OMPLMoveProfile to Profile::ConstPtr for use with ProfileDictionary.addProfile");

    // Alias for backwards compatibility
    m.attr("OMPLPlanProfile_as_ProfileConstPtr") = m.attr("OMPLMoveProfile_as_ProfileConstPtr");

    // Helper to add OMPL move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addOMPLProfile", [](tc::ProfileDictionary& dict,
                                                  const std::string& ns,
                                                  const std::string& profile_name,
                                                  std::shared_ptr<tp::OMPLMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add OMPL move profile to ProfileDictionary (cross-module workaround)");

    // ========== OMPLMotionPlanner ==========
    nb::class_<tp::OMPLMotionPlanner>(m, "OMPLMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::OMPLMotionPlanner::getName)
        .def("solve", &tp::OMPLMotionPlanner::solve, "request"_a)
        .def("terminate", &tp::OMPLMotionPlanner::terminate)
        .def("clear", &tp::OMPLMotionPlanner::clear);
}
