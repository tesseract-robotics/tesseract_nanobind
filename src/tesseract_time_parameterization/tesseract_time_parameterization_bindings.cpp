/**
 * @file tesseract_time_parameterization_bindings.cpp
 * @brief nanobind bindings for tesseract_time_parameterization
 *
 * NOTE: 0.33 API change - TimeParameterization now uses profiles and Environment
 */

#include "tesseract_nb.h"

// tesseract_time_parameterization
#include <tesseract_time_parameterization/core/time_parameterization.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization_profiles.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>
// Note: ProfileDictionary moved to tesseract_common in 0.33
#include <tesseract_common/profile_dictionary.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_time_parameterization, m) {
    m.doc() = "tesseract_time_parameterization Python bindings";

    // Import Profile from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // ========== InstructionsTrajectory ==========
    nb::class_<tp::InstructionsTrajectory>(m, "InstructionsTrajectory")
        .def(nb::init<tp::CompositeInstruction&>(), "program"_a)
        .def("getPosition", nb::overload_cast<Eigen::Index>(&tp::InstructionsTrajectory::getPosition, nb::const_), "i"_a)
        .def("getVelocity", nb::overload_cast<Eigen::Index>(&tp::InstructionsTrajectory::getVelocity, nb::const_), "i"_a)
        .def("getAcceleration", nb::overload_cast<Eigen::Index>(&tp::InstructionsTrajectory::getAcceleration, nb::const_), "i"_a)
        .def("getTimeFromStart", &tp::InstructionsTrajectory::getTimeFromStart, "i"_a)
        .def("setData", &tp::InstructionsTrajectory::setData, "i"_a, "velocity"_a, "acceleration"_a, "time"_a)
        .def("size", &tp::InstructionsTrajectory::size)
        .def("dof", &tp::InstructionsTrajectory::dof)
        .def("empty", &tp::InstructionsTrajectory::empty)
        .def("isTimeStrictlyIncreasing", &tp::InstructionsTrajectory::isTimeStrictlyIncreasing);

    // ========== TimeOptimalTrajectoryGenerationCompositeProfile ==========
    nb::class_<tp::TimeOptimalTrajectoryGenerationCompositeProfile, tc::Profile>(m, "TOTGCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a,
             "path_tolerance"_a, "min_angle_change"_a)
        .def_static("getStaticKey", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::getStaticKey)
        .def_rw("override_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::override_limits)
        .def_rw("velocity_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::velocity_limits)
        .def_rw("acceleration_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::acceleration_limits)
        .def_rw("max_velocity_scaling_factor", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::max_acceleration_scaling_factor)
        .def_rw("path_tolerance", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::path_tolerance)
        .def_rw("min_angle_change", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::min_angle_change);

    // ========== IterativeSplineParameterizationCompositeProfile ==========
    nb::class_<tp::IterativeSplineParameterizationCompositeProfile, tc::Profile>(m, "ISPCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a)
        .def_static("getStaticKey", &tp::IterativeSplineParameterizationCompositeProfile::getStaticKey)
        .def_rw("add_points", &tp::IterativeSplineParameterizationCompositeProfile::add_points)
        .def_rw("override_limits", &tp::IterativeSplineParameterizationCompositeProfile::override_limits)
        .def_rw("velocity_limits", &tp::IterativeSplineParameterizationCompositeProfile::velocity_limits)
        .def_rw("acceleration_limits", &tp::IterativeSplineParameterizationCompositeProfile::acceleration_limits)
        .def_rw("max_velocity_scaling_factor", &tp::IterativeSplineParameterizationCompositeProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::IterativeSplineParameterizationCompositeProfile::max_acceleration_scaling_factor);

    // ========== IterativeSplineParameterizationMoveProfile ==========
    nb::class_<tp::IterativeSplineParameterizationMoveProfile, tc::Profile>(m, "ISPMoveProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a)
        .def_static("getStaticKey", &tp::IterativeSplineParameterizationMoveProfile::getStaticKey)
        .def_rw("max_velocity_scaling_factor", &tp::IterativeSplineParameterizationMoveProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::IterativeSplineParameterizationMoveProfile::max_acceleration_scaling_factor);

    // ========== TimeParameterization (base) ==========
    // Note: 0.33 API change - now uses profiles and Environment
    nb::class_<tp::TimeParameterization>(m, "TimeParameterization")
        .def("getName", &tp::TimeParameterization::getName)
        .def("compute", &tp::TimeParameterization::compute,
             "composite_instruction"_a, "env"_a, "profiles"_a);

    // ========== TimeOptimalTrajectoryGeneration ==========
    nb::class_<tp::TimeOptimalTrajectoryGeneration, tp::TimeParameterization>(m, "TimeOptimalTrajectoryGeneration")
        .def(nb::init<std::string>(), "name"_a = "TOTG");

    // ========== IterativeSplineParameterization ==========
    nb::class_<tp::IterativeSplineParameterization, tp::TimeParameterization>(m, "IterativeSplineParameterization")
        .def(nb::init<std::string>(), "name"_a = "ISP");
}
