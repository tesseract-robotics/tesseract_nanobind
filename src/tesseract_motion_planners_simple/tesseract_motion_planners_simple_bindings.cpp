/**
 * @file tesseract_motion_planners_simple_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners simple
 */

#include "tesseract_nb.h"
#include "tesseract_nb_yaml.h"

#include <tesseract/common/eigen_types.h>

// interpolate() returns, and getInterpolatedInstructions() takes, the opaque VectorIsometry3d
// bound in tesseract_common.
NB_MAKE_OPAQUE(tesseract::common::VectorIsometry3d)

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>

// tesseract_motion_planners simple
#include <tesseract/motion_planners/simple/interpolation.h>
#include <tesseract/motion_planners/simple/simple_motion_planner.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>

// tesseract_common (Profile/ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_environment
#include <tesseract/environment/environment.h>

// types the instruction-info helpers hold and take
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/scene_graph/scene_state.h>

// tesseract_command_language
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/waypoint_poly.h>

namespace tp = tesseract::motion_planners;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace tc = tesseract::common;
namespace tk = tesseract::kinematics;

namespace {

// SimpleMotionPlanner::solve catches the std::exception a profile's generate throws and returns
// a failed response carrying only its text (simple_motion_planner.cpp:101-112, tesseract_planning
// 0.35.0). The Python-facing solve arms this thread's slot, the trampoline stores the error (a
// Python error, or one of the named errors below) there before upstream's catch sees it, and solve
// rethrows it once upstream returns and the GIL is held again. A solve not started from Python (a
// task-composer worker) leaves the slot unarmed and keeps upstream's failed response; nothing is
// stored on such a thread. Each generate call disarms the slot while its override runs, so only
// the planner's own generate calls store; the last error stored wins, and solve raises it only
// for a failed response.
struct PendingGenerateError {
    bool armed{ false };
    std::exception_ptr error;
};
thread_local PendingGenerateError pending_generate_error;

// Arms the slot for one solve; restores the enclosing state, so a solve nested in a generate
// override keeps its own error.
class ArmedSolve {
public:
    ArmedSolve() : saved_armed_(pending_generate_error.armed), saved_error_(std::move(pending_generate_error.error)) {
        pending_generate_error.armed = true;
        pending_generate_error.error = nullptr;
    }
    ~ArmedSolve() {
        pending_generate_error.armed = saved_armed_;
        pending_generate_error.error = std::move(saved_error_);
    }
    ArmedSolve(const ArmedSolve&) = delete;
    ArmedSolve& operator=(const ArmedSolve&) = delete;

    // The last error a generate override raised during this solve, or null.
    std::exception_ptr take() { return std::exchange(pending_generate_error.error, nullptr); }

private:
    bool saved_armed_;
    std::exception_ptr saved_error_;
};

// Clears the slot's armed flag for one generate call and restores it after.
class DisarmedSlot {
public:
    DisarmedSlot() : saved_(std::exchange(pending_generate_error.armed, false)) {}
    ~DisarmedSlot() { pending_generate_error.armed = saved_; }
    DisarmedSlot(const DisarmedSlot&) = delete;
    DisarmedSlot& operator=(const DisarmedSlot&) = delete;

private:
    bool saved_;
};

// generate returned an empty list: upstream reads its back() (simple_motion_planner.cpp:261, :270).
struct EmptyGenerateError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// generate returned something other than a list of MoveInstructionPoly. Its own type, not
// nb::type_error: solve's dispatcher catches only this module's types (see the translator note in
// tesseract_nb.h), and the error is rethrown from there.
struct ProfileReturnTypeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// What is wrong with a generate result that is not a list of MoveInstructionPoly: the first element
// that is not one, or else the result's own type. Called with the GIL held.
std::string describe_wrong_result(nb::handle result) {
    if (nb::isinstance<nb::sequence>(result) && !nb::isinstance<nb::str>(result)) {
        std::size_t index = 0;
        for (nb::handle element : result) {
            if (!nb::isinstance<tcl::MoveInstructionPoly>(element))
                return "element " + std::to_string(index) + " is " +
                       std::string(nb::type_name(element.type()).c_str());
            ++index;
        }
    }
    return "not " + std::string(nb::type_name(result.type()).c_str());
}

// The last instruction generate returned differs from base_instruction in a field upstream requires
// unchanged: move type, waypoint type, profile or path profile.
struct GenerateMismatchError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// generate returned a null MoveInstructionPoly, which upstream would plan as a success.
struct NullInstructionError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// generate is pure virtual: raised for super().generate(...) from a Python override (the base
// binding has nothing to call) and for a Python subclass that does not override it.
struct PureVirtualCallError : std::logic_error {
    using std::logic_error::logic_error;
};

// Python subclasses of SimplePlannerMoveProfile. generate's instruction and manipulator-info
// arguments are const references into the planner's working data; they go to Python as copies,
// so an override that stores one holds no dangling reference. env is a shared_ptr and is passed
// as one. The ticket takes the GIL, which solve released.
class PySimplePlannerMoveProfile : public tp::SimplePlannerMoveProfile {
public:
    NB_TRAMPOLINE(tp::SimplePlannerMoveProfile, 1);

    std::vector<tcl::MoveInstructionPoly> generate(const tcl::MoveInstructionPoly& prev_instruction,
                                                   const tcl::MoveInstructionPoly& prev_seed,
                                                   const tcl::MoveInstructionPoly& base_instruction,
                                                   const tcl::InstructionPoly& next_instruction,
                                                   const std::shared_ptr<const te::Environment>& env,
                                                   const tc::ManipulatorInfo& global_manip_info) const override {
        // Everything, the ticket included, inside the try: each failure is stored in the slot
        // before upstream's catch turns it into message text. The slot is disarmed while the
        // override runs, so a solve nested in it through the MotionPlanner base (whose failure the
        // override may handle) stores nothing here; only this call's own error is kept.
        const bool armed = pending_generate_error.armed;
        DisarmedSlot disarmed;
        try {
            nb::detail::ticket t(nb_trampoline, "generate", false);
            if (!t.key.is_valid())
                throw PureVirtualCallError("SimplePlannerMoveProfile.generate is pure virtual: a Python subclass "
                                           "must override it");
            nb::object result = nb_trampoline.base().attr(t.key)(
                nb::cast(prev_instruction, nb::rv_policy::copy), nb::cast(prev_seed, nb::rv_policy::copy),
                nb::cast(base_instruction, nb::rv_policy::copy), nb::cast(next_instruction, nb::rv_policy::copy),
                env, nb::cast(global_manip_info, nb::rv_policy::copy));
            std::vector<tcl::MoveInstructionPoly> instructions;
            if (!nb::try_cast(result, instructions))
                throw ProfileReturnTypeError("SimplePlannerMoveProfile.generate must return a list of "
                                             "MoveInstructionPoly: " + describe_wrong_result(result));
            if (instructions.empty())
                throw EmptyGenerateError("SimplePlannerMoveProfile.generate returned an empty list; it must end "
                                         "with base_instruction");
            for (std::size_t i = 0; i < instructions.size(); ++i)
                if (instructions[i].isNull())
                    throw NullInstructionError("SimplePlannerMoveProfile.generate returned a null instruction at "
                                               "index " + std::to_string(i));
            // upstream asserts these in debug builds only (simple_motion_planner.cpp:250-253)
            const tcl::MoveInstructionPoly& last = instructions.back();
            const char* field = nullptr;
            if (last.getMoveType() != base_instruction.getMoveType())
                field = "move type";
            else if (last.getWaypoint().getType() != base_instruction.getWaypoint().getType())
                field = "waypoint type";
            else if (last.getProfile() != base_instruction.getProfile())
                field = "profile";
            else if (last.getPathProfile() != base_instruction.getPathProfile())
                field = "path profile";
            if (field != nullptr)
                throw GenerateMismatchError(std::string("SimplePlannerMoveProfile.generate: the last instruction's ") +
                                            field + " differs from base_instruction's; the list must end with "
                                            "base_instruction");
            return instructions;
        } catch (const std::exception&) {
            if (armed)
                pending_generate_error.error = std::current_exception();
            throw;
        }
    }
};

// The members both instruction-info structs share. `instruction` is a reference the struct
// holds, so the getter keeps the struct (and through the constructor's keep_alive, the
// instruction) alive; `manip` came from the environment's kinematics plugins, so it keeps the
// struct (and so the environment) alive.
template <typename Info, typename Group>
void bind_instruction_info(nb::class_<Info>& cls) {
    cls.def(nb::init<const tcl::MoveInstructionPoly&, const te::Environment&, const tc::ManipulatorInfo&>(),
            nb::arg("plan_instruction").noconvert(), "env"_a, "manip_info"_a,
            nb::keep_alive<1, 2>(), nb::keep_alive<1, 3>())
        .def_prop_ro("instruction", [](const Info& self) -> const tcl::MoveInstructionPoly& { return self.instruction; },
            nb::rv_policy::reference_internal, "The move instruction this info describes (not a copy)")
        .def_prop_ro("manip", [](const Info& self) -> std::shared_ptr<const Group> { return self.manip; },
            nb::for_getter(nb::keep_alive<0, 1>()))
        .def_rw("working_frame", &Info::working_frame)
        .def_rw("working_frame_transform", &Info::working_frame_transform)
        .def_rw("tcp_frame", &Info::tcp_frame)
        .def_rw("tcp_offset", &Info::tcp_offset)
        .def_rw("has_cartesian_waypoint", &Info::has_cartesian_waypoint)
        .def("calcCartesianPose", &Info::calcCartesianPose, "jp"_a, "in_world"_a = true,
             "The tool pose at joint state `jp`, in world or in the working frame.")
        .def("extractCartesianPose", &Info::extractCartesianPose, "in_world"_a = true,
             "The Cartesian waypoint's pose, in world or in the working frame.")
        .def("extractJointPosition", &Info::extractJointPosition,
             "The joint or state waypoint's position.");
}

// The longest-valid-segment interpolators, which exist for both instruction-info types.
template <typename Info>
void def_lvs_interpolators(nb::module_& m) {
    m.def("interpolateJointJointWaypoint",
          nb::overload_cast<const Info&, const Info&, double, double, double, int, int>(
              &tp::interpolateJointJointWaypoint),
          "prev"_a, "base"_a, "state_lvs_length"_a, "translation_lvs_length"_a, "rotation_lvs_length"_a,
          "min_steps"_a, "max_steps"_a);
    m.def("interpolateJointCartWaypoint",
          nb::overload_cast<const Info&, const Info&, double, double, double, int, int>(
              &tp::interpolateJointCartWaypoint),
          "prev"_a, "base"_a, "state_lvs_length"_a, "translation_lvs_length"_a, "rotation_lvs_length"_a,
          "min_steps"_a, "max_steps"_a);
    m.def("interpolateCartJointWaypoint",
          nb::overload_cast<const Info&, const Info&, double, double, double, int, int>(
              &tp::interpolateCartJointWaypoint),
          "prev"_a, "base"_a, "state_lvs_length"_a, "translation_lvs_length"_a, "rotation_lvs_length"_a,
          "min_steps"_a, "max_steps"_a);
    m.def("interpolateCartCartWaypoint",
          nb::overload_cast<const Info&, const Info&, double, double, double, int, int,
                            const tesseract::scene_graph::SceneState&>(&tp::interpolateCartCartWaypoint),
          "prev"_a, "base"_a, "state_lvs_length"_a, "translation_lvs_length"_a, "rotation_lvs_length"_a,
          "min_steps"_a, "max_steps"_a, "scene_state"_a);
}

}  // namespace

NB_MODULE(_tesseract_motion_planners_simple, m) {
    m.doc() = "tesseract_motion_planners_simple Python bindings";
    register_python_error_translator();  // SimplePlannerMoveProfile trampoline
    nb::exception<EmptyGenerateError>(m, "SimplePlannerEmptyGenerateError", PyExc_ValueError)
        .attr("__doc__") = "A Python SimplePlannerMoveProfile.generate returned an empty list.";
    nb::exception<NullInstructionError>(m, "SimplePlannerNullInstructionError", PyExc_ValueError)
        .attr("__doc__") = "A Python SimplePlannerMoveProfile.generate returned a null MoveInstructionPoly.";
    nb::exception<GenerateMismatchError>(m, "SimplePlannerGenerateMismatchError", PyExc_ValueError)
        .attr("__doc__") = "A Python generate's last instruction differs from base_instruction in a field upstream requires unchanged.";
    nb::exception<ProfileReturnTypeError>(m, "SimplePlannerProfileReturnTypeError", PyExc_TypeError)
        .attr("__doc__") = "A Python SimplePlannerMoveProfile.generate returned the wrong type.";
    nb::exception<PureVirtualCallError>(m, "SimplePlannerPureVirtualCallError", PyExc_NotImplementedError)
        .attr("__doc__") = "generate is pure virtual: super().generate(...), or a Python profile without it.";

    // Modules defining the types this one names, imported first so the stubs resolve them.
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");
    // MotionPlanner base
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");
    // Profile base and the instruction types (Profile is re-exported from tesseract_common)
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    // KinematicGroup/JointGroup (instruction info) and SceneState (Cartesian interpolation)
    nb::module_::import_("tesseract_robotics.tesseract_kinematics._tesseract_kinematics");
    nb::module_::import_("tesseract_robotics.tesseract_state_solver._tesseract_state_solver");

    // ========== SimpleMotionPlanner ==========
    // getName/terminate/clear/clone come from MotionPlanner (virtual dispatch). solve is
    // MotionPlanner's plus one thing: an error a Python generate override raised is raised from
    // it, where upstream would return a failed response (see PendingGenerateError).
    nb::class_<tp::SimpleMotionPlanner, tp::MotionPlanner>(m, "SimpleMotionPlanner")
        .def(nb::init<std::string>(), "name"_a = "SIMPLE")
        .def("solve",
            [](const tp::SimpleMotionPlanner& self, const tp::PlannerRequest& request) {
                ArmedSolve armed;
                tp::PlannerResponse response;
                {
                    nb::gil_scoped_release release;
                    response = self.solve(request);
                }
                std::exception_ptr error = armed.take();
                if (error && !response.successful)
                    std::rethrow_exception(error);
                return response;
            },
            "request"_a,
            "Plan `request`. An exception a Python profile's generate raises propagates from here.");

    // ========== generateInterpolatedProgram ==========
    m.def("generateInterpolatedProgram",
          [](const tcl::CompositeInstruction& instructions,
             const std::shared_ptr<const te::Environment>& env,
             double state_lvs,
             double translation_lvs,
             double rotation_lvs,
             int min_steps) {
              return tp::generateInterpolatedProgram(instructions, env, state_lvs, translation_lvs, rotation_lvs, min_steps);
          },
          "instructions"_a,
          "env"_a,
          "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "translation_longest_valid_segment_length"_a = 0.15,
          "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "min_steps"_a = 1,
          "Generate an interpolated program from a composite instruction");

    // ========== Interpolation (interpolation.h) ==========
    m.def("interpolate",
          nb::overload_cast<const Eigen::Isometry3d&, const Eigen::Isometry3d&, long>(&tp::interpolate),
          "start"_a, "stop"_a, "steps"_a,
          "Poses from `start` to `stop`: `steps + 1` of them, translation lerped, rotation slerped.");
    m.def("interpolate",
          nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&, const Eigen::Ref<const Eigen::VectorXd>&, long>(
              &tp::interpolate),
          "start"_a, "stop"_a, "steps"_a,
          "Joint states from `start` to `stop`, one per column: `steps + 1` columns.");
    m.def("interpolate_waypoint", &tp::interpolate_waypoint, "start"_a, "stop"_a, "steps"_a,
          "Waypoints from `start` to `stop`: `steps + 1` of them, of the start's type.");
    m.def("getInterpolatedInstructions",
          nb::overload_cast<const std::vector<std::string>&, const Eigen::MatrixXd&, const tcl::MoveInstructionPoly&>(
              &tp::getInterpolatedInstructions),
          "joint_names"_a, "states"_a, "base_instruction"_a,
          "Move instructions for the columns of `states` after the first, each a child of "
          "`base_instruction`; the last is `base_instruction` itself.");
    m.def("getInterpolatedInstructions",
          nb::overload_cast<const tc::VectorIsometry3d&, const std::vector<std::string>&, const Eigen::MatrixXd&,
                            const tcl::MoveInstructionPoly&>(&tp::getInterpolatedInstructions),
          "poses"_a, "joint_names"_a, "states"_a, "base_instruction"_a,
          "Cartesian move instructions for the columns of `states` after the first, seeded with "
          "those states; the last is `base_instruction` itself.");

    // ========== Instruction info (interpolation.h) ==========
    // What the C++ move profiles build from an instruction before interpolating. Move-only; each
    // holds a reference to its instruction (keep_alive<1, 2>, and noconvert so the reference is
    // to the caller's object, never a converted temporary) and a group from the environment's
    // kinematics plugins (keep_alive<1, 3>, as Environment.getKinematicGroup).
    nb::class_<tp::JointGroupInstructionInfo> joint_info(m, "JointGroupInstructionInfo",
        "A move instruction resolved against a joint group (no inverse kinematics)");
    // JointGroupInstructionInfo::getWorkingFrame is declared upstream but never defined
    // (tesseract_planning 0.35.0), so it stays unbound; working_frame_transform holds the pose.
    bind_instruction_info<tp::JointGroupInstructionInfo, tk::JointGroup>(joint_info);
    nb::class_<tp::KinematicGroupInstructionInfo> kin_info(m, "KinematicGroupInstructionInfo",
        "A move instruction resolved against a kinematic group (with inverse kinematics)");
    bind_instruction_info<tp::KinematicGroupInstructionInfo, tk::KinematicGroup>(kin_info);

    // ========== Waypoint interpolation (interpolation.h) ==========
    // Each returns the move instructions from `prev` to `base`, ending with `base`. The
    // fixed-size forms take step counts; the LVS forms longest-valid-segment lengths (rad, m, rad)
    // and step bounds. Cart-Cart takes the scene state the joint seeds come from.
    using KinInfo = tp::KinematicGroupInstructionInfo;
    using JointInfo = tp::JointGroupInstructionInfo;
    m.def("interpolateJointJointWaypoint",
          nb::overload_cast<const KinInfo&, const KinInfo&, int, int>(&tp::interpolateJointJointWaypoint),
          "prev"_a, "base"_a, "linear_steps"_a, "freespace_steps"_a);
    m.def("interpolateJointCartWaypoint",
          nb::overload_cast<const KinInfo&, const KinInfo&, int, int>(&tp::interpolateJointCartWaypoint),
          "prev"_a, "base"_a, "linear_steps"_a, "freespace_steps"_a);
    m.def("interpolateCartJointWaypoint",
          nb::overload_cast<const KinInfo&, const KinInfo&, int, int>(&tp::interpolateCartJointWaypoint),
          "prev"_a, "base"_a, "linear_steps"_a, "freespace_steps"_a);
    m.def("interpolateCartCartWaypoint",
          nb::overload_cast<const KinInfo&, const KinInfo&, int, int, const tesseract::scene_graph::SceneState&>(
              &tp::interpolateCartCartWaypoint),
          "prev"_a, "base"_a, "linear_steps"_a, "freespace_steps"_a, "scene_state"_a);
    def_lvs_interpolators<KinInfo>(m);
    def_lvs_interpolators<JointInfo>(m);

    m.def("getClosestJointSolution",
          nb::overload_cast<const KinInfo&, const Eigen::VectorXd&>(&tp::getClosestJointSolution),
          "info"_a, "seed"_a,
          "The inverse-kinematics solution for `info`'s Cartesian waypoint closest to `seed`.");
    m.def("getClosestJointSolution",
          [](const KinInfo& info1, const KinInfo& info2, const Eigen::VectorXd& seed) {
              std::array<Eigen::VectorXd, 2> solutions = tp::getClosestJointSolution(info1, info2, seed);
              return std::make_pair(std::move(solutions[0]), std::move(solutions[1]));
          },
          "info1"_a, "info2"_a, "seed"_a,
          "The pair of inverse-kinematics solutions, one per Cartesian waypoint, closest to each "
          "other, as `(solution1, solution2)`.");

    // ========== SimplePlannerMoveProfile (abstract base) ==========
    // Python may subclass it and override generate (PySimplePlannerMoveProfile).
    nb::class_<tp::SimplePlannerMoveProfile, tc::Profile, PySimplePlannerMoveProfile>(m, "SimplePlannerMoveProfile")
        .def(nb::init<>())
        .def_static("createKey", &tc::Profile::createKey<tp::SimplePlannerMoveProfile>,
                    "The key every simple-planner move profile registers under.")
        .def("generate",
             [](const tp::SimplePlannerMoveProfile& self, const tcl::MoveInstructionPoly& prev_instruction,
                const tcl::MoveInstructionPoly& prev_seed, const tcl::MoveInstructionPoly& base_instruction,
                const tcl::InstructionPoly& next_instruction, const std::shared_ptr<const te::Environment>& env,
                const tc::ManipulatorInfo& global_manip_info) {
                 // Only a direct Python subclass is built on the trampoline; a Python subclass of a
                 // concrete C++ profile is not, and delegates to that profile's generate.
                 if (dynamic_cast<const PySimplePlannerMoveProfile*>(&self) != nullptr)
                     throw PureVirtualCallError(
                         "SimplePlannerMoveProfile.generate is pure virtual: a Python profile cannot "
                         "delegate to it with super().generate(...)");
                 nb::gil_scoped_release release;
                 return self.generate(prev_instruction, prev_seed, base_instruction, next_instruction, env,
                                      global_manip_info);
             },
             "prev_instruction"_a, "prev_seed"_a, "base_instruction"_a, "next_instruction"_a, "env"_a,
             "global_manip_info"_a,
             "The move instructions the simple planner emits for `base_instruction`, ending with it. "
             "`next_instruction` is a null InstructionPoly for the last move.");

    // ========== SimplePlannerCompositeProfile ==========
    nb::class_<tp::SimplePlannerCompositeProfile, tc::Profile>(m, "SimplePlannerCompositeProfile")
        .def(nb::init<>())
        .def_static("createKey", &tc::Profile::createKey<tp::SimplePlannerCompositeProfile>,
                    "The key this profile type registers under in a ProfileDictionary.");

    // ========== SimplePlannerFixedSizeMoveProfile ==========
    auto py_FixedSizeMoveProfile = nb::class_<tp::SimplePlannerFixedSizeMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeMoveProfile");
    py_FixedSizeMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeMoveProfile);
    bind_from_yaml(py_FixedSizeMoveProfile);

    // ========== SimplePlannerFixedSizeAssignMoveProfile ==========
    auto py_FixedSizeAssignMoveProfile = nb::class_<tp::SimplePlannerFixedSizeAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignMoveProfile");
    py_FixedSizeAssignMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeAssignMoveProfile);
    bind_from_yaml(py_FixedSizeAssignMoveProfile);

    // ========== SimplePlannerFixedSizeAssignNoIKMoveProfile ==========
    auto py_FixedSizeAssignNoIKMoveProfile = nb::class_<tp::SimplePlannerFixedSizeAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignNoIKMoveProfile");
    py_FixedSizeAssignNoIKMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeAssignNoIKMoveProfile);
    bind_from_yaml(py_FixedSizeAssignNoIKMoveProfile);

    // ========== SimplePlannerLVSMoveProfile ==========
    auto py_LVSMoveProfile = nb::class_<tp::SimplePlannerLVSMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSMoveProfile");
    py_LVSMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSMoveProfile::max_steps);
    bind_value_equality(py_LVSMoveProfile);
    bind_from_yaml(py_LVSMoveProfile);

    // ========== SimplePlannerLVSNoIKMoveProfile ==========
    auto py_LVSNoIKMoveProfile = nb::class_<tp::SimplePlannerLVSNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSNoIKMoveProfile");
    py_LVSNoIKMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSNoIKMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSNoIKMoveProfile::max_steps);
    bind_value_equality(py_LVSNoIKMoveProfile);
    bind_from_yaml(py_LVSNoIKMoveProfile);

    // ========== SimplePlannerLVSAssignMoveProfile ==========
    auto py_LVSAssignMoveProfile = nb::class_<tp::SimplePlannerLVSAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignMoveProfile");
    py_LVSAssignMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSAssignMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSAssignMoveProfile::max_steps);
    bind_value_equality(py_LVSAssignMoveProfile);
    bind_from_yaml(py_LVSAssignMoveProfile);

    // ========== SimplePlannerLVSAssignNoIKMoveProfile ==========
    auto py_LVSAssignNoIKMoveProfile = nb::class_<tp::SimplePlannerLVSAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignNoIKMoveProfile");
    py_LVSAssignNoIKMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSAssignNoIKMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSAssignNoIKMoveProfile::max_steps);
    bind_value_equality(py_LVSAssignNoIKMoveProfile);
    bind_from_yaml(py_LVSAssignNoIKMoveProfile);
}
