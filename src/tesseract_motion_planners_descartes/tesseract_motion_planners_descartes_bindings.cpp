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
#include <tesseract/motion_planners/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners Descartes
#include <tesseract/motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract/motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>

// descartes_light core types (needed for Python-side subclassing)
#include <descartes_light/types.h>
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/core/state_evaluator.h>

// tesseract dependencies referenced by profile method signatures
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract/collision/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;
namespace dl = descartes_light;

// ------------------------------------------------------------------
// Trampolines so Python can subclass the descartes_light interfaces.
// ------------------------------------------------------------------

class PyEdgeEvaluatorD : public dl::EdgeEvaluator<double> {
public:
    NB_TRAMPOLINE(dl::EdgeEvaluator<double>, 1);

    std::pair<bool, double>
    evaluate(const dl::State<double>& start, const dl::State<double>& end) const override {
        NB_OVERRIDE_PURE(evaluate, start, end);
    }
};

class PyWaypointSamplerD : public dl::WaypointSampler<double> {
public:
    NB_TRAMPOLINE(dl::WaypointSampler<double>, 1);

    std::vector<dl::StateSample<double>> sample() const override {
        NB_OVERRIDE_PURE(sample);
    }
};

class PyStateEvaluatorD : public dl::StateEvaluator<double> {
public:
    NB_TRAMPOLINE(dl::StateEvaluator<double>, 1);

    std::pair<bool, double> evaluate(const dl::State<double>& solution) const override {
        NB_OVERRIDE(evaluate, solution);
    }
};

// Shim C++ EdgeEvaluator/WaypointSampler/StateEvaluator that hold a Python
// reference and forward calls. These exist so that when a Python-side
// DescartesMoveProfileD subclass returns a Python object from createXxx,
// we can hand back a `std::unique_ptr<...>` to C++ without trying to
// transfer ownership of a Python-allocated instance (which nanobind cannot
// do with the default deleter).
class PyHeldEdgeEvaluatorD : public dl::EdgeEvaluator<double> {
public:
    explicit PyHeldEdgeEvaluatorD(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldEdgeEvaluatorD() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::pair<bool, double>
    evaluate(const dl::State<double>& start, const dl::State<double>& end) const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::pair<bool, double>>(
            obj_.attr("evaluate")(nb::cast(&start, nb::rv_policy::reference),
                                  nb::cast(&end, nb::rv_policy::reference)));
    }

private:
    nb::object obj_;
};

class PyHeldWaypointSamplerD : public dl::WaypointSampler<double> {
public:
    explicit PyHeldWaypointSamplerD(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldWaypointSamplerD() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::vector<dl::StateSample<double>> sample() const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::vector<dl::StateSample<double>>>(obj_.attr("sample")());
    }

private:
    nb::object obj_;
};

class PyHeldStateEvaluatorD : public dl::StateEvaluator<double> {
public:
    explicit PyHeldStateEvaluatorD(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldStateEvaluatorD() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::pair<bool, double> evaluate(const dl::State<double>& solution) const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::pair<bool, double>>(
            obj_.attr("evaluate")(nb::cast(&solution, nb::rv_policy::reference)));
    }

private:
    nb::object obj_;
};

// Trampoline for the move profile itself - lets Python supply custom
// waypoint samplers / edge evaluators / state evaluators via subclassing.
// All three create* overrides wrap the Python return value in a C++ shim.
// This handles both cases uniformly:
//   - Python returns a Python subclass instance (Python owns the C++ obj)
//   - Python returns a C++-origin wrapper (e.g. delegated to a default profile)
// In the second case the shim's dispatch goes Python -> nanobind -> C++ as
// a normal virtual call, which is a one-time extra hop per shim call.
class PyDescartesMoveProfileD : public tp::DescartesMoveProfile<double> {
public:
    NB_TRAMPOLINE(tp::DescartesMoveProfile<double>, 3);

    std::unique_ptr<dl::WaypointSampler<double>>
    createWaypointSampler(const tp::MoveInstructionPoly& move_instruction,
                          const tc::ManipulatorInfo& composite_manip_info,
                          const std::shared_ptr<const tesseract_environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createWaypointSampler", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldWaypointSamplerD>(std::move(py_result));
    }

    std::unique_ptr<dl::EdgeEvaluator<double>>
    createEdgeEvaluator(const tp::MoveInstructionPoly& move_instruction,
                        const tc::ManipulatorInfo& composite_manip_info,
                        const std::shared_ptr<const tesseract_environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createEdgeEvaluator", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldEdgeEvaluatorD>(std::move(py_result));
    }

    std::unique_ptr<dl::StateEvaluator<double>>
    createStateEvaluator(const tp::MoveInstructionPoly& move_instruction,
                         const tc::ManipulatorInfo& composite_manip_info,
                         const std::shared_ptr<const tesseract_environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createStateEvaluator", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldStateEvaluatorD>(std::move(py_result));
    }
};

NB_MODULE(_tesseract_motion_planners_descartes, m) {
    m.doc() = "tesseract_motion_planners_descartes Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Import MotionPlanner base type for clone() return type
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // ========== descartes_light::State<double> ==========
    // Python-readable view of a Descartes state. Constructable from a numpy
    // vector so Python-side StateEvaluator/WaypointSampler subclasses can
    // produce one.
    nb::class_<dl::State<double>>(m, "DescartesStateD")
        .def(nb::init<>())
        .def("__init__", [](dl::State<double>* self, const Eigen::Ref<const Eigen::VectorXd>& v) {
            new (self) dl::State<double>(v);
        }, "values"_a)
        .def_rw("values", &dl::State<double>::values);

    // ========== descartes_light::StateSample<double> ==========
    nb::class_<dl::StateSample<double>>(m, "DescartesStateSampleD")
        .def(nb::init<>())
        .def(nb::init<std::shared_ptr<const dl::State<double>>, double>(), "state"_a, "cost"_a)
        .def_rw("state", &dl::StateSample<double>::state)
        .def_rw("cost", &dl::StateSample<double>::cost);

    // ========== descartes_light::EdgeEvaluator<double> ==========
    nb::class_<dl::EdgeEvaluator<double>, PyEdgeEvaluatorD>(m, "DescartesEdgeEvaluatorD")
        .def(nb::init<>())
        .def("evaluate", &dl::EdgeEvaluator<double>::evaluate, "start"_a, "end"_a,
             "Returns (is_valid, cost) for the edge between two states");

    // ========== descartes_light::WaypointSampler<double> ==========
    nb::class_<dl::WaypointSampler<double>, PyWaypointSamplerD>(m, "DescartesWaypointSamplerD")
        .def(nb::init<>())
        .def("sample", &dl::WaypointSampler<double>::sample,
             "Return the list of valid state samples for this waypoint");

    // ========== descartes_light::StateEvaluator<double> ==========
    nb::class_<dl::StateEvaluator<double>, PyStateEvaluatorD>(m, "DescartesStateEvaluatorD")
        .def(nb::init<>())
        .def("evaluate", &dl::StateEvaluator<double>::evaluate, "solution"_a,
             "Returns (is_valid, cost) for a state");

    // ========== DescartesSolverProfile<double> (base for solver profiles) ==========
    nb::class_<tp::DescartesSolverProfile<double>, tc::Profile>(m, "DescartesSolverProfileD")
        .def("getKey", &tp::DescartesSolverProfile<double>::getKey);

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
    // Python users may subclass this and override createWaypointSampler/
    // createEdgeEvaluator/createStateEvaluator to plug in custom logic.
    nb::class_<tp::DescartesMoveProfile<double>, tc::Profile, PyDescartesMoveProfileD>(m, "DescartesMoveProfileD")
        .def(nb::init<>())
        .def("getKey", &tp::DescartesMoveProfile<double>::getKey)
        .def("createWaypointSampler", &tp::DescartesMoveProfile<double>::createWaypointSampler,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a)
        .def("createEdgeEvaluator", &tp::DescartesMoveProfile<double>::createEdgeEvaluator,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a)
        .def("createStateEvaluator", &tp::DescartesMoveProfile<double>::createStateEvaluator,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a);

    // Helper to cast a Python (or C++) DescartesMoveProfileD subclass to Profile
    // for ProfileDictionary insertion.
    m.def("cast_DescartesMoveProfileD", [](std::shared_ptr<tp::DescartesMoveProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast a DescartesMoveProfileD (including Python subclasses) to Profile for use with ProfileDictionary");

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

    // Legacy alias for cast_DescartesMoveProfileD
    m.def("cast_DescartesPlanProfileD", [](std::shared_ptr<tp::DescartesMoveProfile<double>> profile) {
        return std::static_pointer_cast<tc::Profile>(profile);
    }, "profile"_a,
    "Cast a DescartesMoveProfileD to Profile (legacy alias of cast_DescartesMoveProfileD)");

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
