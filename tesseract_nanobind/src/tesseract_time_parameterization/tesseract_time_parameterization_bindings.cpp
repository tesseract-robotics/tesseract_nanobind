/**
 * @file tesseract_time_parameterization_bindings.cpp
 * @brief nanobind bindings for tesseract_time_parameterization
 */

#include "tesseract_nb.h"

// tesseract_time_parameterization
#include <tesseract_time_parameterization/core/time_parameterization.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>

// tesseract_common for ProfileDictionary
#include <tesseract_common/profile_dictionary.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

namespace tp = tesseract_planning;
namespace te = tesseract_environment;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_time_parameterization, m) {
    m.doc() = "tesseract_time_parameterization Python bindings";

    // ========== TimeParameterization (base) ==========
    nb::class_<tp::TimeParameterization>(m, "TimeParameterization")
        .def("getName", &tp::TimeParameterization::getName)
        .def("compute", &tp::TimeParameterization::compute,
             "composite_instruction"_a, "env"_a, "profiles"_a);

    // ========== TimeOptimalTrajectoryGeneration ==========
    nb::class_<tp::TimeOptimalTrajectoryGeneration, tp::TimeParameterization>(m, "TimeOptimalTrajectoryGeneration")
        .def(nb::init<std::string>(), "name"_a = "TimeOptimalTrajectoryGeneration");

    // ========== IterativeSplineParameterization ==========
    nb::class_<tp::IterativeSplineParameterization, tp::TimeParameterization>(m, "IterativeSplineParameterization")
        .def(nb::init<std::string>(), "name"_a = "IterativeSplineParameterization");
}
