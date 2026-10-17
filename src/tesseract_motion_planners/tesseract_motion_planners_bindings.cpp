/**
 * @file tesseract_motion_planners_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners core
 */

#include "tesseract_nb.h"
#include <nanobind/stl/pair.h>
#include <nanobind/stl/unordered_map.h>

#include <tesseract/common/eigen_types.h>

// Toolpath is AlignedVector<VectorIsometry3d>; the inner vector is opaque, bound in tesseract_common.
NB_MAKE_OPAQUE(tesseract::common::VectorIsometry3d)

// tesseract_motion_planners core
#include <tesseract/motion_planners/planner.h>
#include <tesseract/motion_planners/planner_utils.h>
#include <tesseract/motion_planners/robot_config.h>
#include <tesseract/motion_planners/types.h>
#include <tesseract/motion_planners/utils.h>

// tesseract_environment
#include <tesseract/environment/environment.h>

// tesseract_collision
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/types.h>

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>

// tesseract_state_solver
#include <tesseract/state_solver/state_solver.h>

// tesseract_command_language
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/common/profile_dictionary.h>

namespace tp = tesseract::motion_planners;
namespace te = tesseract::environment;
namespace tc = tesseract::common;
namespace tcl = tesseract::command_language;
namespace tcol = tesseract::collision;
namespace tk = tesseract::kinematics;
namespace tsg = tesseract::scene_graph;

namespace {
// contactCheckProgram's `std::vector<ContactResultMap>& contacts` out-param is returned
// alongside the summary, as checkTrajectory does in tesseract_environment.
using ContactCheckResult = std::pair<tcol::ContactTrajectoryResults, std::vector<tcol::ContactResultMap>>;

// The program loop is pure C++, so the GIL is released for it. Released in-body rather than
// via call_guard so the result is cast back with the GIL held.
template <typename Manager>
ContactCheckResult contact_check_program(Manager& manager,
                                         const tsg::StateSolver& state_solver,
                                         const tcl::CompositeInstruction& program,
                                         const tcol::CollisionCheckConfig& config)
{
    ContactCheckResult out;
    nb::gil_scoped_release release;
    out.first = tp::contactCheckProgram(out.second, manager, state_solver, program, config);
    return out;
}

}  // namespace

NB_MODULE(_tesseract_motion_planners, m) {
    m.doc() = "tesseract_motion_planners Python bindings";

    // Modules defining the types this one names, imported first so the stubs resolve them.
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");
    nb::module_::import_("tesseract_robotics.tesseract_state_solver._tesseract_state_solver");
    nb::module_::import_("tesseract_robotics.tesseract_kinematics._tesseract_kinematics");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // ========== PlannerRequest ==========
    nb::class_<tp::PlannerRequest>(m, "PlannerRequest")
        .def(nb::init<>())
        .def_rw("name", &tp::PlannerRequest::name)
        .def_prop_rw("env",
            [](const tp::PlannerRequest& self) { return self.env; },
            [](tp::PlannerRequest& self, std::shared_ptr<const te::Environment> env) { self.env = env; })
        // Note: ProfileDictionary moved to tesseract_common in 0.33
        .def_prop_rw("profiles",
            [](const tp::PlannerRequest& self) { return self.profiles; },
            [](tp::PlannerRequest& self, std::shared_ptr<const tc::ProfileDictionary> profiles) { self.profiles = profiles; })
        .def_rw("instructions", &tp::PlannerRequest::instructions)
        .def_rw("verbose", &tp::PlannerRequest::verbose)
        .def_rw("format_result_as_input", &tp::PlannerRequest::format_result_as_input);

    // ========== PlannerResponse ==========
    nb::class_<tp::PlannerResponse>(m, "PlannerResponse")
        .def(nb::init<>())
        .def_rw("results", &tp::PlannerResponse::results)
        .def_rw("successful", &tp::PlannerResponse::successful)
        .def_rw("message", &tp::PlannerResponse::message)
        .def("__bool__", [](const tp::PlannerResponse& self) { return static_cast<bool>(self); });

    // ========== MotionPlanner (abstract base) ==========
    nb::class_<tp::MotionPlanner>(m, "MotionPlanner")
        .def("getName", &tp::MotionPlanner::getName)
        .def("solve", &tp::MotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::MotionPlanner::terminate)
        .def("clear", &tp::MotionPlanner::clear)
        .def_static("checkRequest", nb::overload_cast<const tp::PlannerRequest&>(&tp::MotionPlanner::checkRequest),
                    "request"_a)
        // The reason overload has the same Python arity as checkRequest, so it gets its own name.
        .def_static("checkRequestReason",
            [](const tp::PlannerRequest& request) {
                std::string reason;
                tp::MotionPlanner::checkRequest(request, reason);
                return reason;
            },
            "request"_a,
            "Why `checkRequest(request)` fails: the C++ reason, or an empty string for a valid request.")
        .def_static("assignSolution", &tp::MotionPlanner::assignSolution,
                    "mi"_a, "joint_names"_a, "joint_values"_a, "format_result_as_input"_a,
                    "Write a joint solution into `mi` in place: a state waypoint, or with "
                    "`format_result_as_input` the seed of a Cartesian / the position of a joint waypoint.")
        .def("clone", [](const tp::MotionPlanner& self) { return self.clone(); });

    // ========== Utility functions ==========
    m.def("assignCurrentStateAsSeed", &tp::assignCurrentStateAsSeed,
          "composite_instructions"_a, "env"_a,
          "Assign the current environment state as seed to all CartesianWaypoints in the program");

    m.def("formatProgram", &tp::formatProgram,
          "composite_instructions"_a, "env"_a,
          "Reorder joint and state waypoints in place to the kinematic group's joint order. "
          "Returns True if the program needed formatting.");

    // Exact-type overloads first; InstructionPoly last.
    m.def("toToolpath",
          nb::overload_cast<const tcl::CompositeInstruction&, const te::Environment&>(&tp::toToolpath),
          "ci"_a, "env"_a,
          "World-frame TCP poses of a program, one VectorIsometry3d per sub-composite.");
    m.def("toToolpath",
          nb::overload_cast<const tcl::MoveInstructionPoly&, const te::Environment&>(&tp::toToolpath),
          "mi"_a, "env"_a, "World-frame TCP pose of a move instruction.");
    m.def("toToolpath",
          nb::overload_cast<const tcl::InstructionPoly&, const te::Environment&>(&tp::toToolpath),
          "instruction"_a, "env"_a, "World-frame TCP poses of a composite or move instruction.");

    m.def("contactCheckProgram", &contact_check_program<tcol::ContinuousContactManager>,
          "manager"_a, "state_solver"_a, "program"_a, "config"_a,
          "Continuous collision check over a program. "
          "Returns (ContactTrajectoryResults, list[ContactResultMap]), one map per step.");
    m.def("contactCheckProgram", &contact_check_program<tcol::DiscreteContactManager>,
          "manager"_a, "state_solver"_a, "program"_a, "config"_a,
          "Discrete collision check over a program. "
          "Returns (ContactTrajectoryResults, list[ContactResultMap]), one map per step.");

    // ========== Robot configuration (robot_config.h, planner_utils.h; double instances) ==========
    nb::enum_<tp::RobotConfig>(m, "RobotConfig",
        "Six-axis arm configuration: wrist Flip/No-flip, elbow Up/Down, arm fronT/Back.")
        .value("NUT", tp::RobotConfig::NUT)
        .value("FUT", tp::RobotConfig::FUT)
        .value("NDT", tp::RobotConfig::NDT)
        .value("FDT", tp::RobotConfig::FDT)
        .value("NDB", tp::RobotConfig::NDB)
        .value("FDB", tp::RobotConfig::FDB)
        .value("NUB", tp::RobotConfig::NUB)
        .value("FUB", tp::RobotConfig::FUB);

    m.def("getRobotConfig",
          [](const tk::JointGroup& joint_group, const std::string& base_link, const std::string& tcp_frame,
             const Eigen::Ref<const Eigen::VectorXd>& joint_values, const Eigen::Vector2i& sign_correction) {
              return tp::getRobotConfig<double>(joint_group, base_link, tcp_frame, joint_values, sign_correction);
          },
          "joint_group"_a, "base_link"_a, "tcp_frame"_a, "joint_values"_a,
          "sign_correction"_a.sig("[1, 1]") = Eigen::Vector2i::Ones(),
          "Configuration of a six-axis arm. The last six joint values are the arm's; "
          "`sign_correction` flips the sign of joint 3 and joint 5 for the manufacturer's convention.");

    m.def("getJointTurns", &tp::getJointTurns<double>, "joint_values"_a,
          "Whole half-turns per joint, `int(q / pi)`: non-zero beyond +-pi.");

    m.def("isWithinJointLimits", &tp::isWithinJointLimits<double>,
          "joint_values"_a, "limits"_a, "coupling_limits"_a,
          "True if `joint_values` are within `limits` and joint 4 + joint 6 (third and first from "
          "the end) are within `coupling_limits`.");

    // isValidState<double> is not bound: upstream planner_utils.h:84 calls isWithinLimits without
    // its template argument, and the call does not deduce for an Eigen::Matrix limits argument,
    // so the double instance does not compile (0.35.0).
}
