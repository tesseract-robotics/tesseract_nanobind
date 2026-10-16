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
#include "tesseract_nb_yaml.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners OMPL
#include <tesseract/motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/ompl_solver_config.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract/collision/types.h>

// OMPL global RNG (seeding for reproducible planning)
#include <ompl/util/RandomNumbers.h>

namespace tp = tesseract::motion_planners;
namespace tc = tesseract::common;

NB_MODULE(_tesseract_motion_planners_ompl, m) {
    m.doc() = "tesseract_motion_planners_ompl Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    auto profile_type = cl_module.attr("Profile");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Import MotionPlanner: base class of OMPLMotionPlanner and clone() return type
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // ========== OMPLSolverConfig ==========
    nb::class_<tp::OMPLSolverConfig> solver_config(m, "OMPLSolverConfig");
    solver_config
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
        }, "Clear all planners");
    // Compares planning_time, max_solutions, simplify, optimize and the planner *count*
    // (upstream ompl_solver_config.cpp), not the configurators themselves.
    bind_value_equality(solver_config);

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
    // create(ompl::base::SpaceInformationPtr) stays unbound: it takes and returns OMPL types.
    nb::class_<tp::OMPLPlannerConfigurator>(m, "OMPLPlannerConfigurator")
        .def("getType", &tp::OMPLPlannerConfigurator::getType);

    // ========== Configurators, one per OMPLPlannerType ==========
    nb::class_<tp::SBLConfigurator, tp::OMPLPlannerConfigurator>(m, "SBLConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::SBLConfigurator::range);

    nb::class_<tp::ESTConfigurator, tp::OMPLPlannerConfigurator>(m, "ESTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::ESTConfigurator::range)
        .def_rw("goal_bias", &tp::ESTConfigurator::goal_bias);

    nb::class_<tp::LBKPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "LBKPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::LBKPIECE1Configurator::range)
        .def_rw("border_fraction", &tp::LBKPIECE1Configurator::border_fraction)
        .def_rw("min_valid_path_fraction", &tp::LBKPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::BKPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "BKPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::BKPIECE1Configurator::range)
        .def_rw("border_fraction", &tp::BKPIECE1Configurator::border_fraction)
        .def_rw("failed_expansion_score_factor", &tp::BKPIECE1Configurator::failed_expansion_score_factor)
        .def_rw("min_valid_path_fraction", &tp::BKPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::KPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "KPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::KPIECE1Configurator::range)
        .def_rw("goal_bias", &tp::KPIECE1Configurator::goal_bias)
        .def_rw("border_fraction", &tp::KPIECE1Configurator::border_fraction)
        .def_rw("failed_expansion_score_factor", &tp::KPIECE1Configurator::failed_expansion_score_factor)
        .def_rw("min_valid_path_fraction", &tp::KPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::BiTRRTConfigurator, tp::OMPLPlannerConfigurator>(m, "BiTRRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::BiTRRTConfigurator::range)
        .def_rw("temp_change_factor", &tp::BiTRRTConfigurator::temp_change_factor)
        .def_rw("cost_threshold", &tp::BiTRRTConfigurator::cost_threshold)
        .def_rw("init_temperature", &tp::BiTRRTConfigurator::init_temperature)
        .def_rw("frontier_threshold", &tp::BiTRRTConfigurator::frontier_threshold)
        .def_rw("frontier_node_ratio", &tp::BiTRRTConfigurator::frontier_node_ratio);

    nb::class_<tp::RRTConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConfigurator::range)
        .def_rw("goal_bias", &tp::RRTConfigurator::goal_bias);

    nb::class_<tp::RRTConnectConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConnectConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConnectConfigurator::range);

    nb::class_<tp::RRTstarConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTstarConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTstarConfigurator::range)
        .def_rw("goal_bias", &tp::RRTstarConfigurator::goal_bias)
        .def_rw("delay_collision_checking", &tp::RRTstarConfigurator::delay_collision_checking);

    nb::class_<tp::TRRTConfigurator, tp::OMPLPlannerConfigurator>(m, "TRRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::TRRTConfigurator::range)
        .def_rw("goal_bias", &tp::TRRTConfigurator::goal_bias)
        .def_rw("temp_change_factor", &tp::TRRTConfigurator::temp_change_factor)
        .def_rw("init_temperature", &tp::TRRTConfigurator::init_temperature)
        .def_rw("frontier_threshold", &tp::TRRTConfigurator::frontier_threshold)
        .def_rw("frontier_node_ratio", &tp::TRRTConfigurator::frontier_node_ratio);

    nb::class_<tp::PRMConfigurator, tp::OMPLPlannerConfigurator>(m, "PRMConfigurator")
        .def(nb::init<>())
        .def_rw("max_nearest_neighbors", &tp::PRMConfigurator::max_nearest_neighbors);

    nb::class_<tp::PRMstarConfigurator, tp::OMPLPlannerConfigurator>(m, "PRMstarConfigurator")
        .def(nb::init<>());

    nb::class_<tp::LazyPRMstarConfigurator, tp::OMPLPlannerConfigurator>(m, "LazyPRMstarConfigurator")
        .def(nb::init<>());

    nb::class_<tp::SPARSConfigurator, tp::OMPLPlannerConfigurator>(m, "SPARSConfigurator")
        .def(nb::init<>())
        .def_rw("max_failures", &tp::SPARSConfigurator::max_failures)
        .def_rw("dense_delta_fraction", &tp::SPARSConfigurator::dense_delta_fraction)
        .def_rw("sparse_delta_fraction", &tp::SPARSConfigurator::sparse_delta_fraction)
        .def_rw("stretch_factor", &tp::SPARSConfigurator::stretch_factor);

    // Bound after the configurators so the stub names them; the element type is
    // shared_ptr<const OMPLPlannerConfigurator>, cast to the bound derived class.
    solver_config.def_rw("planners", &tp::OMPLSolverConfig::planners,
        "Planner configurators, one parallel planner each. The list is copied on access: "
        "assign a new list (or use addPlanner/clearPlanners) to change it.");

    // ========== OMPLMoveProfile (base, was OMPLPlanProfile) ==========
    // createSimpleSetup / createStateExtractor stay unbound: they take or return OMPL types.
    nb::class_<tp::OMPLMoveProfile, tc::Profile>(m, "OMPLMoveProfile")
        .def("getKey", &tp::OMPLMoveProfile::getKey)
        // Every OMPL move profile shares the key of this base (OMPLMoveProfile()'s constructor).
        .def_static("createKey", &tc::Profile::createKey<tp::OMPLMoveProfile>)
        .def("createSolverConfig", &tp::OMPLMoveProfile::createSolverConfig,
            "The solver config the planner uses for this profile");

    // SWIG-compatible alias
    m.attr("OMPLPlanProfile") = m.attr("OMPLMoveProfile");

    // ========== OMPLRealVectorMoveProfile (was OMPLRealVectorPlanProfile) ==========
    nb::class_<tp::OMPLRealVectorMoveProfile, tp::OMPLMoveProfile> real_vector_move_profile(
        m, "OMPLRealVectorMoveProfile");
    real_vector_move_profile
        .def(nb::init<>())
        .def_rw("solver_config", &tp::OMPLRealVectorMoveProfile::solver_config,
            "The OMPL parallel planner solver config")
        .def_rw("contact_manager_config", &tp::OMPLRealVectorMoveProfile::contact_manager_config,
            "The contact manager configuration (collision margins)")
        .def_rw("collision_check_config", &tp::OMPLRealVectorMoveProfile::collision_check_config,
            "The collision check configuration");
    bind_value_equality(real_vector_move_profile);
    bind_from_yaml(real_vector_move_profile);

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
    // getName / solve (GIL released) / terminate / clear / clone come from MotionPlanner
    nb::class_<tp::OMPLMotionPlanner, tp::MotionPlanner>(m, "OMPLMotionPlanner")
        .def(nb::init<std::string>(), "name"_a);

    // ========== Global OMPL RNG seeding ==========
    m.def("RNG_setSeed", [](std::uint_fast32_t seed) { ompl::RNG::setSeed(seed); }, "seed"_a,
          "Seed OMPL's global RNG seed generator for reproducible planning.\n\n"
          "Samplers created after this call draw deterministic seeds, so repeated solve()\n"
          "calls produce identical paths. Re-seeding after planning has started logs an\n"
          "OMPL error (harmless: only pre-existing RNG instances stay nondeterministic).\n"
          "A seed of 0 is rejected once generation has started.");
    m.def("RNG_getSeed", []() { return ompl::RNG::getSeed(); },
          "Get the first seed used by OMPL's RNG seed generator.\n\n"
          "Log this value to make a stochastic planning failure reproducible by passing\n"
          "it to RNG_setSeed in a subsequent run.");
}
