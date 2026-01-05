/**
 * @file tesseract_time_parameterization_bindings.cpp
 * @brief nanobind bindings for tesseract_time_parameterization
 *
 * NOTE: 0.33 API change - TimeParameterization now uses profiles and Environment
 */

#include "tesseract_nb.h"

// tesseract_time_parameterization
#include <tesseract_time_parameterization/core/time_parameterization.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>
// Note: ProfileDictionary moved to tesseract_common in 0.33
#include <tesseract_common/profile_dictionary.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

namespace tp = tesseract_planning;

NB_MODULE(_tesseract_time_parameterization, m) {
    m.doc() = "tesseract_time_parameterization Python bindings";

    // ========== TimeParameterization (base) ==========
    // Note: 0.33 API change - now uses profiles and Environment
    nb::class_<tp::TimeParameterization>(m, "TimeParameterization")
        .def("getName", &tp::TimeParameterization::getName)
        .def("compute", &tp::TimeParameterization::compute,
             "composite_instruction"_a, "env"_a, "profiles"_a);

    // ========== TimeOptimalTrajectoryGeneration ==========
    // Note: 0.33 changed - path_tolerance/min_angle_change now in profile
    nb::class_<tp::TimeOptimalTrajectoryGeneration, tp::TimeParameterization>(m, "TimeOptimalTrajectoryGeneration")
        .def(nb::init<std::string>(), "name"_a = "TOTG");

    // ========== IterativeSplineParameterization ==========
    // Note: 0.33 changed - add_points now in profile
    nb::class_<tp::IterativeSplineParameterization, tp::TimeParameterization>(m, "IterativeSplineParameterization")
        .def(nb::init<std::string>(), "name"_a = "ISP");
}
