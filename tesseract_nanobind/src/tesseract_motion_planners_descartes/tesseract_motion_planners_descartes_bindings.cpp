/**
 * @file tesseract_motion_planners_descartes_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners Descartes
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (for Profile base class and ProfileDictionary)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners Descartes (0.33.x: Plan -> Move profile rename)
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_descartes, m) {
    m.doc() = "tesseract_motion_planners_descartes Python bindings";

    // Import tesseract_common for Profile base class (cross-module inheritance)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== DescartesMoveProfile<double> (base) - renamed from DescartesPlanProfile in 0.33.x ==========
    nb::class_<tp::DescartesMoveProfile<double>, tc::Profile>(m, "DescartesMoveProfileD")
        .def("getKey", &tp::DescartesMoveProfile<double>::getKey)
        .def_static("getStaticKey", &tp::DescartesMoveProfile<double>::getStaticKey);

    // Alias for backwards compatibility
    m.attr("DescartesPlanProfileD") = m.attr("DescartesMoveProfileD");

    // ========== DescartesDefaultMoveProfile<double> - renamed from DescartesDefaultPlanProfile in 0.33.x ==========
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

    // Alias for backwards compatibility
    m.attr("DescartesDefaultPlanProfileD") = m.attr("DescartesDefaultMoveProfileD");

    // Helper to add Descartes move profile to ProfileDictionary
    m.def("cast_DescartesMoveProfileD", [](std::shared_ptr<tp::DescartesDefaultMoveProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesDefaultMoveProfileD to Profile for use with ProfileDictionary");

    // Alias for backwards compatibility
    m.attr("cast_DescartesPlanProfileD") = m.attr("cast_DescartesMoveProfileD");

    // ========== DescartesMotionPlanner<double> ==========
    nb::class_<tp::DescartesMotionPlanner<double>>(m, "DescartesMotionPlannerD")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::DescartesMotionPlanner<double>::getName)
        .def("solve", &tp::DescartesMotionPlanner<double>::solve, "request"_a)
        .def("terminate", &tp::DescartesMotionPlanner<double>::terminate)
        .def("clear", &tp::DescartesMotionPlanner<double>::clear);
}
