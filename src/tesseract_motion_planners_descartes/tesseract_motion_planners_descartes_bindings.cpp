/**
 * @file tesseract_motion_planners_descartes_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners Descartes
 *
 * NOTE: 0.33 API changes:
 * - DescartesPlanProfile -> DescartesMoveProfile
 * - DescartesDefaultPlanProfile -> DescartesDefaultMoveProfile
 * - Profile/ProfileDictionary moved to tesseract_common
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners Descartes
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_descartes, m) {
    m.doc() = "tesseract_motion_planners_descartes Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Import MotionPlanner base type for clone() return type
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // ========== DescartesSolverProfile<double> (base for solver profiles) ==========
    nb::class_<tp::DescartesSolverProfile<double>, tc::Profile>(m, "DescartesSolverProfileD")
        .def("getKey", &tp::DescartesSolverProfile<double>::getKey)
        .def_static("getStaticKey", &tp::DescartesSolverProfile<double>::getStaticKey);

    // ========== DescartesLadderGraphSolverProfile<double> ==========
    nb::class_<tp::DescartesLadderGraphSolverProfile<double>, tp::DescartesSolverProfile<double>>(m, "DescartesLadderGraphSolverProfileD")
        .def(nb::init<>())
        .def_rw("num_threads", &tp::DescartesLadderGraphSolverProfile<double>::num_threads,
            "Number of threads to use during planning (default: 1)");

    // Helper to cast DescartesLadderGraphSolverProfileD to Profile
    m.def("cast_DescartesSolverProfileD", [](std::shared_ptr<tp::DescartesLadderGraphSolverProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesLadderGraphSolverProfileD to Profile for use with ProfileDictionary");

    // ========== DescartesMoveProfile<double> (base, was DescartesPlanProfile) ==========
    nb::class_<tp::DescartesMoveProfile<double>, tc::Profile>(m, "DescartesMoveProfileD")
        .def("getKey", &tp::DescartesMoveProfile<double>::getKey)
        .def_static("getStaticKey", &tp::DescartesMoveProfile<double>::getStaticKey);

    // SWIG-compatible alias
    m.attr("DescartesPlanProfileD") = m.attr("DescartesMoveProfileD");

    // ========== DescartesDefaultMoveProfile<double> (was DescartesDefaultPlanProfile) ==========
    nb::class_<tp::DescartesDefaultMoveProfile<double>, tp::DescartesMoveProfile<double>>(m, "DescartesDefaultMoveProfileD")
        .def(nb::init<>())
        .def_rw("target_pose_fixed", &tp::DescartesDefaultMoveProfile<double>::target_pose_fixed)
        .def_rw("target_pose_sample_axis", &tp::DescartesDefaultMoveProfile<double>::target_pose_sample_axis)
        .def_rw("target_pose_sample_resolution", &tp::DescartesDefaultMoveProfile<double>::target_pose_sample_resolution)
        .def_rw("target_pose_sample_min", &tp::DescartesDefaultMoveProfile<double>::target_pose_sample_min)
        .def_rw("target_pose_sample_max", &tp::DescartesDefaultMoveProfile<double>::target_pose_sample_max)
        .def_rw("manipulator_ik_solver", &tp::DescartesDefaultMoveProfile<double>::manipulator_ik_solver)
        .def_rw("allow_collision", &tp::DescartesDefaultMoveProfile<double>::allow_collision)
        .def_rw("enable_collision", &tp::DescartesDefaultMoveProfile<double>::enable_collision)
        .def_rw("vertex_collision_check_config", &tp::DescartesDefaultMoveProfile<double>::vertex_collision_check_config)
        .def_rw("enable_edge_collision", &tp::DescartesDefaultMoveProfile<double>::enable_edge_collision)
        .def_rw("edge_collision_check_config", &tp::DescartesDefaultMoveProfile<double>::edge_collision_check_config)
        .def_rw("use_redundant_joint_solutions", &tp::DescartesDefaultMoveProfile<double>::use_redundant_joint_solutions)
        .def_rw("debug", &tp::DescartesDefaultMoveProfile<double>::debug);

    // SWIG-compatible alias
    m.attr("DescartesDefaultPlanProfileD") = m.attr("DescartesDefaultMoveProfileD");

    // Helper to cast DescartesDefaultMoveProfileD to Profile
    m.def("cast_DescartesMoveProfileD", [](std::shared_ptr<tp::DescartesDefaultMoveProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesDefaultMoveProfileD to Profile for use with ProfileDictionary");

    // Legacy alias
    m.def("cast_DescartesPlanProfileD", [](std::shared_ptr<tp::DescartesDefaultMoveProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesDefaultPlanProfileD to Profile (legacy alias)");

    // ========== DescartesMotionPlanner<double> ==========
    nb::class_<tp::DescartesMotionPlanner<double>>(m, "DescartesMotionPlannerD")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::DescartesMotionPlanner<double>::getName)
        // Release GIL during solve - Descartes uses OpenMP internally but that's safe
        // as long as we use a single OpenMP runtime (conda's llvm-openmp, not Homebrew's)
        .def("solve", &tp::DescartesMotionPlanner<double>::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::DescartesMotionPlanner<double>::terminate)
        .def("clear", &tp::DescartesMotionPlanner<double>::clear)
        .def("clone", [](const tp::DescartesMotionPlanner<double>& self) { return self.clone(); });
}
