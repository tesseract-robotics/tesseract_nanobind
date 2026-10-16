/**
 * @file tesseract_time_parameterization_bindings.cpp
 * @brief nanobind bindings for tesseract_time_parameterization
 *
 * NOTE: 0.33 API change - TimeParameterization now uses profiles and Environment
 */

#include "tesseract_nb.h"

#include <utility>

// tesseract_time_parameterization
#include <tesseract/time_parameterization/time_parameterization.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization.h>
#include <tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>
#include <tesseract/time_parameterization/utils.h>

// tesseract_command_language
#include <tesseract/command_language/composite_instruction.h>
// Note: ProfileDictionary moved to tesseract_common in 0.33
#include <tesseract/common/profile_dictionary.h>

// tesseract_environment
#include <tesseract/environment/environment.h>

namespace tp = tesseract::time_parameterization;
namespace tcl = tesseract::command_language;
namespace tc = tesseract::common;

// rescaleTimings needs one scaling per sub-composite; upstream only assert()s it (utils.cpp),
// so a release build reads past the end of a short list.
struct ScalingsLengthError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

struct StaleTrajectoryError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// InstructionsTrajectory stores references from program.flatten() (instructions_trajectory.cpp):
// appending to `program` can reallocate its instruction vector and leave them dangling, so a
// later read returns stale values, throws, or crashes. The guard records the addresses of the
// flattened move instructions and re-flattens `program` before every call; comparing addresses
// never dereferences the stored references.
class GuardedInstructionsTrajectory {
public:
    explicit GuardedInstructionsTrajectory(tcl::CompositeInstruction& program)
        : program_(program), trajectory_(program) {
        for (const auto& instruction : program.flatten(moveFilter))
            addresses_.push_back(&instruction.get());
    }

    tp::InstructionsTrajectory& checked() {
        const auto flat = std::as_const(program_).flatten(moveFilter);
        bool same = flat.size() == addresses_.size();
        for (std::size_t k = 0; same && k < flat.size(); ++k)
            same = &flat[k].get() == addresses_[k];
        if (!same)
            throw StaleTrajectoryError("the program changed after this InstructionsTrajectory was built "
                                       "and its references into it are stale; build a new InstructionsTrajectory");
        return trajectory_;
    }

private:
    // The filter upstream's InstructionsTrajectory(program) flattens with
    static bool moveFilter(const tcl::InstructionPoly& i, const tcl::CompositeInstruction&) {
        return i.isMoveInstruction();
    }

    tcl::CompositeInstruction& program_;  // kept alive by keep_alive<1, 2>
    tp::InstructionsTrajectory trajectory_;
    std::vector<const tcl::InstructionPoly*> addresses_;
};

NB_MODULE(_tesseract_time_parameterization, m) {
    m.doc() = "tesseract_time_parameterization Python bindings";

    // Import Profile from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    // Import Environment so TimeParameterization.compute's signature names the Python type
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");

    // ========== InstructionsTrajectory ==========
    nb::exception<StaleTrajectoryError>(m, "StaleTrajectoryError", PyExc_RuntimeError)
        .attr("__doc__") = "The program changed after the InstructionsTrajectory was built; build a new one.";
    using GIT = GuardedInstructionsTrajectory;
    nb::class_<GIT>(m, "InstructionsTrajectory")
        // Holds references into `program`: keep it alive as long as the trajectory
        .def(nb::init<tcl::CompositeInstruction&>(), "program"_a, nb::keep_alive<1, 2>(),
             "Build a trajectory view over the move instructions of `program`.\n\n"
             "The trajectory keeps `program` alive and reads and writes its waypoints in place\n"
             "(`setData` changes `program`). Once instructions are appended, inserted or erased\n"
             "in `program`, every method raises `StaleTrajectoryError`: build a new\n"
             "trajectory. Each call re-flattens `program`, O(number of instructions).\n\n"
             "Raises:\n"
             "    StaleTrajectoryError: on any method call after `program` changed (a `RuntimeError`).")
        .def("getPosition", [](GIT& self, Eigen::Index i) -> const Eigen::VectorXd& {
            return std::as_const(self.checked()).getPosition(i);
        }, "i"_a)
        .def("getVelocity", [](GIT& self, Eigen::Index i) -> const Eigen::VectorXd& {
            return std::as_const(self.checked()).getVelocity(i);
        }, "i"_a)
        .def("getAcceleration", [](GIT& self, Eigen::Index i) -> const Eigen::VectorXd& {
            return std::as_const(self.checked()).getAcceleration(i);
        }, "i"_a)
        .def("getTimeFromStart", [](GIT& self, Eigen::Index i) { return self.checked().getTimeFromStart(i); }, "i"_a)
        .def("setData", [](GIT& self, Eigen::Index i, const Eigen::VectorXd& velocity,
                           const Eigen::VectorXd& acceleration, double time) {
            self.checked().setData(i, velocity, acceleration, time);
        }, "i"_a, "velocity"_a, "acceleration"_a, "time"_a)
        .def("size", [](GIT& self) { return self.checked().size(); })
        .def("dof", [](GIT& self) { return self.checked().dof(); })
        .def("empty", [](GIT& self) { return self.checked().empty(); })
        .def("isTimeStrictlyIncreasing", [](GIT& self) { return self.checked().isTimeStrictlyIncreasing(); });

    // ========== TimeOptimalTrajectoryGenerationCompositeProfile ==========
    auto totg_composite_profile = nb::class_<tp::TimeOptimalTrajectoryGenerationCompositeProfile, tc::Profile>(m, "TOTGCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a,
             "path_tolerance"_a, "min_angle_change"_a)
        .def_rw("override_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::override_limits)
        .def_rw("velocity_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::velocity_limits)
        .def_rw("acceleration_limits", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::acceleration_limits)
        .def_rw("max_velocity_scaling_factor", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::max_acceleration_scaling_factor)
        .def_rw("path_tolerance", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::path_tolerance)
        .def_rw("min_angle_change", &tp::TimeOptimalTrajectoryGenerationCompositeProfile::min_angle_change)
        .def_static("createKey", &tc::Profile::createKey<tp::TimeOptimalTrajectoryGenerationCompositeProfile>);
    bind_value_equality(totg_composite_profile);

    // ========== IterativeSplineParameterizationCompositeProfile ==========
    auto isp_composite_profile = nb::class_<tp::IterativeSplineParameterizationCompositeProfile, tc::Profile>(m, "ISPCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a)
        .def_rw("add_points", &tp::IterativeSplineParameterizationCompositeProfile::add_points)
        .def_rw("override_limits", &tp::IterativeSplineParameterizationCompositeProfile::override_limits)
        .def_rw("velocity_limits", &tp::IterativeSplineParameterizationCompositeProfile::velocity_limits)
        .def_rw("acceleration_limits", &tp::IterativeSplineParameterizationCompositeProfile::acceleration_limits)
        .def_rw("max_velocity_scaling_factor", &tp::IterativeSplineParameterizationCompositeProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::IterativeSplineParameterizationCompositeProfile::max_acceleration_scaling_factor)
        .def_rw("minimum_time_delta", &tp::IterativeSplineParameterizationCompositeProfile::minimum_time_delta)
        .def_static("createKey", &tc::Profile::createKey<tp::IterativeSplineParameterizationCompositeProfile>);
    bind_value_equality(isp_composite_profile);

    // ========== IterativeSplineParameterizationMoveProfile ==========
    auto isp_move_profile = nb::class_<tp::IterativeSplineParameterizationMoveProfile, tc::Profile>(m, "ISPMoveProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(),
             "max_velocity_scaling_factor"_a, "max_acceleration_scaling_factor"_a)
        .def_rw("max_velocity_scaling_factor", &tp::IterativeSplineParameterizationMoveProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::IterativeSplineParameterizationMoveProfile::max_acceleration_scaling_factor)
        .def_static("createKey", &tc::Profile::createKey<tp::IterativeSplineParameterizationMoveProfile>);
    bind_value_equality(isp_move_profile);

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

    // ========== ConstantTCPSpeedParameterizationCompositeProfile ==========
    auto constant_tcp_speed_composite_profile = nb::class_<tp::ConstantTCPSpeedParameterizationCompositeProfile, tc::Profile>(m, "ConstantTCPSpeedCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double, double>(),
             "max_translational_velocity"_a,
             "max_rotational_velocity"_a,
             "max_translational_acceleration"_a,
             "max_rotational_acceleration"_a,
             "max_velocity_scaling_factor"_a = 1.0,
             "max_acceleration_scaling_factor"_a = 1.0)
        .def_rw("max_translational_velocity", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_translational_velocity)
        .def_rw("max_rotational_velocity", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_rotational_velocity)
        .def_rw("max_translational_acceleration", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_translational_acceleration)
        .def_rw("max_rotational_acceleration", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_rotational_acceleration)
        .def_rw("max_velocity_scaling_factor", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_velocity_scaling_factor)
        .def_rw("max_acceleration_scaling_factor", &tp::ConstantTCPSpeedParameterizationCompositeProfile::max_acceleration_scaling_factor)
        .def_static("createKey", &tc::Profile::createKey<tp::ConstantTCPSpeedParameterizationCompositeProfile>);
    bind_value_equality(constant_tcp_speed_composite_profile);

    // ========== ConstantTCPSpeedParameterization ==========
    nb::class_<tp::ConstantTCPSpeedParameterization, tp::TimeParameterization>(m, "ConstantTCPSpeedParameterization")
        .def(nb::init<std::string>(), "name"_a = "ConstantTCPSpeed");

    // ========== RuckigTrajectorySmoothingCompositeProfile ==========
    auto ruckig_composite_profile = nb::class_<tp::RuckigTrajectorySmoothingCompositeProfile, tc::Profile>(m, "RuckigTrajectorySmoothingCompositeProfile")
        .def(nb::init<>())
        .def(nb::init<double, double>(), "duration_extension_fraction"_a, "max_duration_extension_factor"_a)
        .def_rw("duration_extension_fraction", &tp::RuckigTrajectorySmoothingCompositeProfile::duration_extension_fraction)
        .def_rw("max_duration_extension_factor", &tp::RuckigTrajectorySmoothingCompositeProfile::max_duration_extension_factor)
        .def_rw("override_limits", &tp::RuckigTrajectorySmoothingCompositeProfile::override_limits)
        .def_rw("velocity_limits", &tp::RuckigTrajectorySmoothingCompositeProfile::velocity_limits)
        .def_rw("acceleration_limits", &tp::RuckigTrajectorySmoothingCompositeProfile::acceleration_limits)
        .def_rw("jerk_limits", &tp::RuckigTrajectorySmoothingCompositeProfile::jerk_limits)
        .def_static("createKey", &tc::Profile::createKey<tp::RuckigTrajectorySmoothingCompositeProfile>);
    bind_value_equality(ruckig_composite_profile);

    // ========== RuckigTrajectorySmoothing ==========
    nb::class_<tp::RuckigTrajectorySmoothing, tp::TimeParameterization>(m, "RuckigTrajectorySmoothing")
        .def(nb::init<std::string>(), "name"_a = "RuckigTrajectorySmoothing");

    // ========== Utilities ==========
    nb::exception<ScalingsLengthError>(m, "ScalingsLengthError", PyExc_ValueError)
        .attr("__doc__") = "rescaleTimings got a scalings list that is not len(program) long.";
    m.def("rescaleTimings", [](tcl::CompositeInstruction& program, std::vector<double> scalings) {
        // Upstream checks the length only with assert(), compiled out in release builds:
        // a short list would read past its end.
        if (scalings.size() != program.size())
            throw ScalingsLengthError("rescaleTimings: scalings has " + std::to_string(scalings.size()) +
                                      " elements, program has " + std::to_string(program.size()) +
                                      " sub-composites; they must match");
        tp::rescaleTimings(program, std::move(scalings));
    }, "program"_a, "scalings"_a,
    "Rescale the timing of each sub-composite of `program` in place.\n\n"
    "Every element of `program` must be a CompositeInstruction; `scalings[i]` scales\n"
    "element i linearly (0.5 is half speed). A scaling below 1e-6 is replaced by 1.0\n"
    "with only a logged warning.\n\n"
    "Raises:\n"
    "    ScalingsLengthError: `scalings` is not `len(program)` long (a `ValueError`).");
}
