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

#include <sstream>

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>
#include <tesseract/motion_planners/planner.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners Descartes
#include <tesseract/motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract/motion_planners/descartes/profile/descartes_profile.h>
// createKinematicGroup's definition: the library's instantiation is a local symbol (not
// exported, tesseract_planning 0.35.0), so this TU instantiates its own.
#include <tesseract/motion_planners/descartes/impl/profile/descartes_profile.hpp>
#include <tesseract/motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>
#include <tesseract/motion_planners/descartes/descartes_utils.h>

// descartes_light core types (needed for Python-side subclassing)
#include <descartes_light/types.h>
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <descartes_light/core/state_evaluator.h>

// tesseract dependencies referenced by profile method signatures
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract/collision/types.h>

namespace tp = tesseract::motion_planners;
namespace tcl = tesseract::command_language;
namespace tc = tesseract::common;
namespace dl = descartes_light;

// tesseract_common binds VectorIsometry3d as an opaque class; without this the stl/vector caster
// would return the tool-pose samples as a plain list.
NB_MAKE_OPAQUE(tc::VectorIsometry3d)

// The library instantiates every Descartes type for both scalar types. DescartesMotionPlanner<float>
// is what the shipped task-composer config runs for every Cartesian and raster task; only
// DescartesDPipeline / DescartesDNPCPipeline run DescartesMotionPlanner<double>. A profile is found
// only under its own scalar type's key, so both are bound, with one template.

// ------------------------------------------------------------------
// Trampolines so Python can subclass the descartes_light interfaces.
// ------------------------------------------------------------------

template <typename FloatType>
class PyEdgeEvaluator : public dl::EdgeEvaluator<FloatType> {
public:
    NB_TRAMPOLINE(dl::EdgeEvaluator<FloatType>, 1);

    std::pair<bool, FloatType>
    evaluate(const dl::State<FloatType>& start, const dl::State<FloatType>& end) const override {
        NB_OVERRIDE_PURE(evaluate, start, end);
    }
};

template <typename FloatType>
class PyWaypointSampler : public dl::WaypointSampler<FloatType> {
public:
    NB_TRAMPOLINE(dl::WaypointSampler<FloatType>, 1);

    std::vector<dl::StateSample<FloatType>> sample() const override {
        NB_OVERRIDE_PURE(sample);
    }
};

template <typename FloatType>
class PyStateEvaluator : public dl::StateEvaluator<FloatType> {
public:
    NB_TRAMPOLINE(dl::StateEvaluator<FloatType>, 1);

    std::pair<bool, FloatType> evaluate(const dl::State<FloatType>& solution) const override {
        NB_OVERRIDE(evaluate, solution);
    }
};

// Shim C++ EdgeEvaluator/WaypointSampler/StateEvaluator that hold a Python
// reference and forward calls. These exist so that when a Python-side
// DescartesMoveProfile subclass returns a Python object from createXxx,
// we can hand back a `std::unique_ptr<...>` to C++ without trying to
// transfer ownership of a Python-allocated instance (which nanobind cannot
// do with the default deleter).
template <typename FloatType>
class PyHeldEdgeEvaluator : public dl::EdgeEvaluator<FloatType> {
public:
    explicit PyHeldEdgeEvaluator(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldEdgeEvaluator() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::pair<bool, FloatType>
    evaluate(const dl::State<FloatType>& start, const dl::State<FloatType>& end) const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::pair<bool, FloatType>>(
            obj_.attr("evaluate")(nb::cast(&start, nb::rv_policy::reference),
                                  nb::cast(&end, nb::rv_policy::reference)));
    }

private:
    nb::object obj_;
};

template <typename FloatType>
class PyHeldWaypointSampler : public dl::WaypointSampler<FloatType> {
public:
    explicit PyHeldWaypointSampler(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldWaypointSampler() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::vector<dl::StateSample<FloatType>> sample() const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::vector<dl::StateSample<FloatType>>>(obj_.attr("sample")());
    }

private:
    nb::object obj_;
};

template <typename FloatType>
class PyHeldStateEvaluator : public dl::StateEvaluator<FloatType> {
public:
    explicit PyHeldStateEvaluator(nb::object obj) : obj_(std::move(obj)) {}
    ~PyHeldStateEvaluator() override {
        nb::gil_scoped_acquire gil;
        obj_.reset();
    }

    std::pair<bool, FloatType> evaluate(const dl::State<FloatType>& solution) const override {
        nb::gil_scoped_acquire gil;
        return nb::cast<std::pair<bool, FloatType>>(
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
template <typename FloatType>
class PyDescartesMoveProfile : public tp::DescartesMoveProfile<FloatType> {
public:
    NB_TRAMPOLINE(tp::DescartesMoveProfile<FloatType>, 3);

    std::unique_ptr<dl::WaypointSampler<FloatType>>
    createWaypointSampler(const tcl::MoveInstructionPoly& move_instruction,
                          const tc::ManipulatorInfo& composite_manip_info,
                          const std::shared_ptr<const tesseract::environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createWaypointSampler", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldWaypointSampler<FloatType>>(std::move(py_result));
    }

    std::unique_ptr<dl::EdgeEvaluator<FloatType>>
    createEdgeEvaluator(const tcl::MoveInstructionPoly& move_instruction,
                        const tc::ManipulatorInfo& composite_manip_info,
                        const std::shared_ptr<const tesseract::environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createEdgeEvaluator", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldEdgeEvaluator<FloatType>>(std::move(py_result));
    }

    std::unique_ptr<dl::StateEvaluator<FloatType>>
    createStateEvaluator(const tcl::MoveInstructionPoly& move_instruction,
                         const tc::ManipulatorInfo& composite_manip_info,
                         const std::shared_ptr<const tesseract::environment::Environment>& env) const override {
        nb::detail::ticket t(nb_trampoline, "createStateEvaluator", true);
        nb::object py_result = nb::borrow<nb::object>(
            nb_trampoline.base().attr(t.key)(move_instruction, composite_manip_info, env));
        return std::make_unique<PyHeldStateEvaluator<FloatType>>(std::move(py_result));
    }
};

// The Python classes for one scalar type, in registration order (a base before its subclass).
// NB_MODULE constructs each as `Class(m, "Name")` with its literal Python name: the binding audit
// reads that call to map the class to its C++ template (a braced `{m, "Name"}` hides it).
// bind_descartes then defines the members, once for both scalar types.
template <typename FloatType>
struct DescartesClasses {
    using StateClass = nb::class_<dl::State<FloatType>>;
    using StateSampleClass = nb::class_<dl::StateSample<FloatType>>;
    using EdgeEvaluatorClass = nb::class_<dl::EdgeEvaluator<FloatType>, PyEdgeEvaluator<FloatType>>;
    using WaypointSamplerClass = nb::class_<dl::WaypointSampler<FloatType>, PyWaypointSampler<FloatType>>;
    using StateEvaluatorClass = nb::class_<dl::StateEvaluator<FloatType>, PyStateEvaluator<FloatType>>;
    using SolverProfileClass = nb::class_<tp::DescartesSolverProfile<FloatType>, tc::Profile>;
    using LadderGraphSolverProfileClass =
        nb::class_<tp::DescartesLadderGraphSolverProfile<FloatType>, tp::DescartesSolverProfile<FloatType>>;
    using MoveProfileClass =
        nb::class_<tp::DescartesMoveProfile<FloatType>, tc::Profile, PyDescartesMoveProfile<FloatType>>;
    using DefaultMoveProfileClass =
        nb::class_<tp::DescartesDefaultMoveProfile<FloatType>, tp::DescartesMoveProfile<FloatType>>;
    using MotionPlannerClass = nb::class_<tp::DescartesMotionPlanner<FloatType>, tp::MotionPlanner>;

    StateClass state;
    StateSampleClass state_sample;
    EdgeEvaluatorClass edge_evaluator;
    WaypointSamplerClass waypoint_sampler;
    StateEvaluatorClass state_evaluator;
    SolverProfileClass solver_profile;
    LadderGraphSolverProfileClass ladder_graph_solver_profile;
    MoveProfileClass move_profile;
    DefaultMoveProfileClass default_move_profile;
    MotionPlannerClass motion_planner;
};

template <typename FloatType>
void bind_descartes(DescartesClasses<FloatType>& c) {
    using State = dl::State<FloatType>;
    using StateSample = dl::StateSample<FloatType>;
    using EdgeEvaluator = dl::EdgeEvaluator<FloatType>;
    using WaypointSampler = dl::WaypointSampler<FloatType>;
    using StateEvaluator = dl::StateEvaluator<FloatType>;
    using SolverProfile = tp::DescartesSolverProfile<FloatType>;
    using LadderGraphSolverProfile = tp::DescartesLadderGraphSolverProfile<FloatType>;
    using MoveProfile = tp::DescartesMoveProfile<FloatType>;
    using DefaultMoveProfile = tp::DescartesDefaultMoveProfile<FloatType>;
    using Vector = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

    // ========== descartes_light::State ==========
    // Python-readable view of a Descartes state. Constructable from a numpy
    // vector so Python-side StateEvaluator/WaypointSampler subclasses can
    // produce one.
    c.state
        .def(nb::init<>())
        .def("__init__", [](State* self, const Eigen::Ref<const Vector>& v) {
            new (self) State(v);
        }, "values"_a)
        .def_rw("values", &State::values);

    // ========== descartes_light::StateSample ==========
    c.state_sample
        .def(nb::init<>())
        .def(nb::init<std::shared_ptr<const State>, FloatType>(), "state"_a, "cost"_a)
        .def(nb::init<std::shared_ptr<const State>>(), "state"_a, "A sample with cost 0")
        .def_rw("state", &StateSample::state)
        .def_rw("cost", &StateSample::cost);

    // ========== descartes_light::EdgeEvaluator ==========
    c.edge_evaluator
        .def(nb::init<>())
        .def("evaluate", &EdgeEvaluator::evaluate, "start"_a, "end"_a,
             "Returns (is_valid, cost) for the edge between two states");

    // ========== descartes_light::WaypointSampler ==========
    c.waypoint_sampler
        .def(nb::init<>())
        .def("sample", &WaypointSampler::sample,
             "Return the list of valid state samples for this waypoint")
        // upstream's print(); a Python subclass that defines __str__ keeps its own
        .def("__str__", [](const WaypointSampler& self) {
            std::stringstream ss;
            self.print(ss);
            return ss.str();
        });

    // ========== descartes_light::StateEvaluator ==========
    c.state_evaluator
        .def(nb::init<>())
        .def("evaluate", &StateEvaluator::evaluate, "solution"_a,
             "Returns (is_valid, cost) for a state");

    // ========== DescartesSolverProfile (base for solver profiles) ==========
    c.solver_profile
        .def("getKey", &SolverProfile::getKey)
        .def_static("createKey", &tc::Profile::createKey<SolverProfile>,
            "The key every Descartes solver profile carries");

    // ========== DescartesLadderGraphSolverProfile ==========
    c.ladder_graph_solver_profile
        .def(nb::init<>())
        .def_rw("num_threads", &LadderGraphSolverProfile::num_threads,
            "Number of threads to use during planning (default: 1)");
    bind_value_equality(c.ladder_graph_solver_profile);

    // ========== DescartesMoveProfile (base, was DescartesPlanProfile) ==========
    // Python users may subclass this and override createWaypointSampler/
    // createEdgeEvaluator/createStateEvaluator to plug in custom logic.
    c.move_profile
        .def(nb::init<>())
        .def("getKey", &MoveProfile::getKey)
        .def_static("createKey", &tc::Profile::createKey<MoveProfile>,
            "The key every Descartes move profile carries")
        // The group's IK solver comes from the environment's kinematics plugins, so the group keeps
        // the environment alive (as Environment.getKinematicGroup keeps its environment).
        .def("createKinematicGroup", &MoveProfile::createKinematicGroup,
             "manip_info"_a, "env"_a, nb::keep_alive<0, 3>(),
             "The kinematic group for `manip_info`'s manipulator and IK solver")
        .def("createWaypointSampler", &MoveProfile::createWaypointSampler,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a)
        .def("createEdgeEvaluator", &MoveProfile::createEdgeEvaluator,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a)
        .def("createStateEvaluator", &MoveProfile::createStateEvaluator,
             "move_instruction"_a, "composite_manip_info"_a, "env"_a);

    // ========== DescartesDefaultMoveProfile (was DescartesDefaultPlanProfile) ==========
    c.default_move_profile
        .def(nb::init<>())
        .def_rw("target_pose_fixed", &DefaultMoveProfile::target_pose_fixed)
        .def_rw("target_pose_sample_axis", &DefaultMoveProfile::target_pose_sample_axis)
        .def_rw("target_pose_sample_resolution", &DefaultMoveProfile::target_pose_sample_resolution)
        .def_rw("target_pose_sample_min", &DefaultMoveProfile::target_pose_sample_min)
        .def_rw("target_pose_sample_max", &DefaultMoveProfile::target_pose_sample_max)
        .def_rw("manipulator_ik_solver", &DefaultMoveProfile::manipulator_ik_solver)
        .def_rw("allow_collision", &DefaultMoveProfile::allow_collision)
        .def_rw("enable_collision", &DefaultMoveProfile::enable_collision)
        .def_rw("vertex_contact_manager_config", &DefaultMoveProfile::vertex_contact_manager_config)
        .def_rw("vertex_collision_check_config", &DefaultMoveProfile::vertex_collision_check_config)
        .def_rw("enable_edge_collision", &DefaultMoveProfile::enable_edge_collision)
        .def_rw("edge_contact_manager_config", &DefaultMoveProfile::edge_contact_manager_config)
        .def_rw("edge_collision_check_config", &DefaultMoveProfile::edge_collision_check_config)
        .def_rw("use_redundant_joint_solutions", &DefaultMoveProfile::use_redundant_joint_solutions)
        .def_rw("debug", &DefaultMoveProfile::debug);
    bind_value_equality(c.default_move_profile);

    // ========== DescartesMotionPlanner ==========
    // getName/solve/terminate/clear/clone are MotionPlanner's. Its solve releases the GIL:
    // Descartes uses OpenMP internally, safe as long as one OpenMP runtime is loaded (conda's
    // llvm-openmp, not Homebrew's).
    c.motion_planner.def(nb::init<std::string>(), "name"_a);
}

NB_MODULE(_tesseract_motion_planners_descartes, m) {
    m.doc() = "tesseract_motion_planners_descartes Python bindings";
    register_python_error_translator();  // evaluator, sampler and profile trampolines

    // Import Profile type from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Import MotionPlanner base type (DescartesMotionPlanner's base)
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // VectorIsometry3d (tool-pose samplers) and Environment (profile hooks)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");
    // KinematicGroup (createKinematicGroup)
    nb::module_::import_("tesseract_robotics.tesseract_kinematics._tesseract_kinematics");

    using D = DescartesClasses<double>;
    D d{
        D::StateClass(m, "DescartesStateD"),
        D::StateSampleClass(m, "DescartesStateSampleD"),
        D::EdgeEvaluatorClass(m, "DescartesEdgeEvaluatorD"),
        D::WaypointSamplerClass(m, "DescartesWaypointSamplerD"),
        D::StateEvaluatorClass(m, "DescartesStateEvaluatorD"),
        D::SolverProfileClass(m, "DescartesSolverProfileD",
            "Base of the Descartes solver profiles on `double` states, the type a "
            "DescartesDMotionPlannerTaskFactory node reads (DescartesDPipeline, DescartesDNPCPipeline)."),
        D::LadderGraphSolverProfileClass(m, "DescartesLadderGraphSolverProfileD"),
        D::MoveProfileClass(m, "DescartesMoveProfileD",
            "Base of the Descartes move profiles on `double` states, the type a "
            "DescartesDMotionPlannerTaskFactory node reads (DescartesDPipeline, DescartesDNPCPipeline)."),
        D::DefaultMoveProfileClass(m, "DescartesDefaultMoveProfileD"),
        D::MotionPlannerClass(m, "DescartesMotionPlannerD"),
    };
    bind_descartes(d);

    using F = DescartesClasses<float>;
    F f{
        F::StateClass(m, "DescartesStateF"),
        F::StateSampleClass(m, "DescartesStateSampleF"),
        F::EdgeEvaluatorClass(m, "DescartesEdgeEvaluatorF"),
        F::WaypointSamplerClass(m, "DescartesWaypointSamplerF"),
        F::StateEvaluatorClass(m, "DescartesStateEvaluatorF"),
        F::SolverProfileClass(m, "DescartesSolverProfileF",
            "Base of the Descartes solver profiles on `float` states, the type a "
            "DescartesFMotionPlannerTaskFactory node reads (every shipped Cartesian and raster task)."),
        F::LadderGraphSolverProfileClass(m, "DescartesLadderGraphSolverProfileF"),
        F::MoveProfileClass(m, "DescartesMoveProfileF",
            "Base of the Descartes move profiles on `float` states, the type a "
            "DescartesFMotionPlannerTaskFactory node reads (every shipped Cartesian and raster task)."),
        F::DefaultMoveProfileClass(m, "DescartesDefaultMoveProfileF"),
        F::MotionPlannerClass(m, "DescartesMotionPlannerF"),
    };
    bind_descartes(f);

    // ========== Tool-pose samplers (descartes_utils.h) ==========
    // What DescartesDefaultMoveProfile's target_pose_sample_* fields drive.
    m.def("sampleToolAxis", &tp::sampleToolAxis,
          "tool_pose"_a, "axis"_a, "resolution"_a, "minimum"_a, "maximum"_a,
          "Given a tool pose create samples from [minimum, maximum] around the provided axis.");
    m.def("sampleToolXAxis", &tp::sampleToolXAxis, "tool_pose"_a, "resolution"_a, "minimum"_a, "maximum"_a,
          "Given a tool pose create samples from [minimum, maximum] around the x axis.");
    m.def("sampleToolYAxis", &tp::sampleToolYAxis, "tool_pose"_a, "resolution"_a, "minimum"_a, "maximum"_a,
          "Given a tool pose create samples from [minimum, maximum] around the y axis.");
    m.def("sampleToolZAxis", &tp::sampleToolZAxis, "tool_pose"_a, "resolution"_a, "minimum"_a, "maximum"_a,
          "Given a tool pose create samples from [minimum, maximum] around the z axis.");
    m.def("sampleFixed", &tp::sampleFixed, "tool_pose"_a,
          "This will return the tool pose as the only sample.");
}
