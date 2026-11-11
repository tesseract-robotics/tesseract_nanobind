/**
 * @file motion_planners_ompl.cpp
 * @brief nanobind bindings for tesseract_motion_planners OMPL
 */

#include "tesseract_nb.h"
#include "tesseract_nb_yaml.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>

// tesseract_common (Profile, ProfileDictionary)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners OMPL
#include <tesseract/motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/ompl_solver_config.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract/collision/types.h>

// OMPL global RNG (seeding for reproducible planning)
#include <ompl/util/RandomNumbers.h>

// OMPL optimization objectives (#362): built in Python, handed to the planner through
// OMPLRealVectorMoveProfile::createOptimizationObjective
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>

namespace tp = tesseract::motion_planners;
namespace tc = tesseract::common;
namespace ob = ompl::base;

namespace {

// [dimensionless, work per unit path length] OMPL's default path-length weight of
// MechanicalWorkOptimizationObjective (MechanicalWorkOptimizationObjective.h constructor default):
// a default argument, which C++ exposes as no symbol, so it is restated here for the keyword.
constexpr double MECHANICAL_WORK_PATH_LENGTH_WEIGHT_DEFAULT = 0.00001;

// A Python override of createOptimizationObjective returned something other than an
// OptimizationObjective (or None).
struct ObjectiveReturnTypeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A Python stateCost override returned something other than a float.
struct StateCostReturnTypeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A Python stateCost override reads the state as joint values, which needs a
// RealVectorStateSpace (the space OMPLRealVectorMoveProfile plans in).
struct StateSpaceTypeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// Python subclasses of StateCostIntegralObjective override stateCost(values) -> float, where
// values holds the state's joint positions (an owned copy). The ticket takes the GIL: the
// planner calls stateCost while pricing paths, with the GIL released by solve, possibly on a
// planner thread. A Python error propagates as a C++ exception from that call. Without an
// override the C++ stateCost (Cost(1)) applies, so the integral is path length.
class PyStateCostIntegralObjective : public ob::StateCostIntegralObjective {
public:
    NB_TRAMPOLINE(ob::StateCostIntegralObjective, 1);

    PyStateCostIntegralObjective(const ob::SpaceInformationPtr& si, bool enable_motion_cost_interpolation)
        : ob::StateCostIntegralObjective(si, enable_motion_cost_interpolation) {
        if (si->getStateSpace()->getType() != ob::STATE_SPACE_REAL_VECTOR)
            throw StateSpaceTypeError(
                "StateCostIntegralObjective.stateCost needs a RealVectorStateSpace; the planning space "
                "'" + si->getStateSpace()->getName() + "' is not one");
    }

    ob::Cost stateCost(const ob::State* s) const override {
        nb::detail::ticket t(nb_trampoline, "stateCost", false);
        if (!t.key.is_valid())
            return ob::StateCostIntegralObjective::stateCost(s);
        const std::size_t dof = si_->getStateDimension();
        const double* src = s->as<ob::RealVectorStateSpace::StateType>()->values;
        auto* values = new double[dof];
        std::copy(src, src + dof, values);
        nb::capsule owner(values, [](void* p) noexcept { delete[] static_cast<double*>(p); });
        nb::ndarray<double, nb::numpy, nb::ndim<1>> array(values, { dof }, owner);
        nb::object result = nb_trampoline.base().attr(t.key)(array);
        if (!nb::isinstance<nb::float_>(result))
            throw StateCostReturnTypeError(
                "StateCostIntegralObjective.stateCost must return a float, not " +
                std::string(nb::type_name(result.type()).c_str()));
        return ob::Cost(nb::cast<double>(result));
    }
};

// The profile hook returns a unique_ptr, but an objective built in Python is shared with
// Python (and with any MultiOptimizationObjective holding it). This owns one share and
// forwards every virtual, so the planner sees the Python objective's behaviour. The share's
// deleter takes the GIL, so the planner may drop it on a thread without it.
class SharedOptimizationObjective : public ob::OptimizationObjective {
public:
    explicit SharedOptimizationObjective(std::shared_ptr<ob::OptimizationObjective> inner)
        : ob::OptimizationObjective(inner->getSpaceInformation()), inner_(std::move(inner)) {
        description_ = inner_->getDescription();
        threshold_ = inner_->getCostThreshold();
        if (inner_->hasCostToGoHeuristic()) {
            setCostToGoHeuristic([inner = inner_](const ob::State* state, const ob::Goal* goal) {
                return inner->costToGo(state, goal);
            });
        }
    }

    bool isSatisfied(ob::Cost c) const override { return inner_->isSatisfied(c); }
    bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const override { return inner_->isCostBetterThan(c1, c2); }
    bool isCostEquivalentTo(ob::Cost c1, ob::Cost c2) const override { return inner_->isCostEquivalentTo(c1, c2); }
    bool isFinite(ob::Cost cost) const override { return inner_->isFinite(cost); }
    ob::Cost betterCost(ob::Cost c1, ob::Cost c2) const override { return inner_->betterCost(c1, c2); }
    ob::Cost stateCost(const ob::State* s) const override { return inner_->stateCost(s); }
    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override { return inner_->motionCost(s1, s2); }
    ob::Cost controlCost(const ompl::control::Control* c, unsigned int steps) const override {
        return inner_->controlCost(c, steps);
    }
    ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const override { return inner_->combineCosts(c1, c2); }
    ob::Cost subtractCosts(ob::Cost c1, ob::Cost c2) const override { return inner_->subtractCosts(c1, c2); }
    ob::Cost identityCost() const override { return inner_->identityCost(); }
    ob::Cost infiniteCost() const override { return inner_->infiniteCost(); }
    ob::Cost initialCost(const ob::State* s) const override { return inner_->initialCost(s); }
    ob::Cost terminalCost(const ob::State* s) const override { return inner_->terminalCost(s); }
    bool isSymmetric() const override { return inner_->isSymmetric(); }
    ob::Cost averageStateCost(unsigned int numStates) const override { return inner_->averageStateCost(numStates); }
    ob::Cost motionCostHeuristic(const ob::State* s1, const ob::State* s2) const override {
        return inner_->motionCostHeuristic(s1, s2);
    }
    ob::Cost motionCostBestEstimate(const ob::State* s1, const ob::State* s2) const override {
        return inner_->motionCostBestEstimate(s1, s2);
    }
    ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr& probDefn,
                                                     unsigned int maxNumberCalls) const override {
        return inner_->allocInformedStateSampler(probDefn, maxNumberCalls);
    }
    void print(std::ostream& out) const override { inner_->print(out); }

private:
    std::shared_ptr<ob::OptimizationObjective> inner_;
};

// Python subclasses of OMPLRealVectorMoveProfile override
// createOptimizationObjective(si, env, manip) -> OptimizationObjective | None. The ticket takes
// the GIL, which solve released; a Python error propagates from solve (the planner does not
// catch it). Without an override the C++ default (path length) applies.
class PyOMPLRealVectorMoveProfile : public tp::OMPLRealVectorMoveProfile {
public:
    NB_TRAMPOLINE(tp::OMPLRealVectorMoveProfile, 1);

    std::unique_ptr<ob::OptimizationObjective>
    createOptimizationObjective(const ompl::geometric::SimpleSetup& simple_setup,
                                const std::shared_ptr<const tesseract::environment::Environment>& env,
                                const std::shared_ptr<const tesseract::kinematics::JointGroup>& manip,
                                const tp::OMPLStateExtractor& state_extractor) const override {
        nb::detail::ticket t(nb_trampoline, "createOptimizationObjective", false);
        if (!t.key.is_valid())
            return tp::OMPLRealVectorMoveProfile::createOptimizationObjective(simple_setup, env, manip,
                                                                             state_extractor);
        nb::object result = nb_trampoline.base().attr(t.key)(simple_setup.getSpaceInformation(), env, manip);
        // None means no objective, as a nullptr from the C++ hook does.
        if (result.is_none())
            return nullptr;
        if (!nb::isinstance<ob::OptimizationObjective>(result))
            throw ObjectiveReturnTypeError(
                "OMPLRealVectorMoveProfile.createOptimizationObjective must return an OptimizationObjective "
                "or None, not " + std::string(nb::type_name(result.type()).c_str()));
        return std::make_unique<SharedOptimizationObjective>(nb::cast<std::shared_ptr<ob::OptimizationObjective>>(result));
    }
};

}  // namespace

NB_MODULE(motion_planners_ompl, m) {
    m.doc() = "tesseract_motion_planners_ompl Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.command_language");
    auto profile_type = cl_module.attr("Profile");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.collision");

    // Import MotionPlanner: base class of OMPLMotionPlanner and clone() return type
    nb::module_::import_("tesseract_robotics.motion_planners._motion_planners");

    // Environment and JointGroup: arguments of the createOptimizationObjective override
    nb::module_::import_("tesseract_robotics.environment");
    nb::module_::import_("tesseract_robotics.kinematics");

    nb::exception<ObjectiveReturnTypeError>(m, "OMPLObjectiveReturnTypeError", PyExc_TypeError);
    nb::exception<StateCostReturnTypeError>(m, "OMPLStateCostError", PyExc_TypeError);
    nb::exception<StateSpaceTypeError>(m, "OMPLStateSpaceTypeError", PyExc_TypeError);
    register_python_error_translator();  // createOptimizationObjective and stateCost trampolines

    // ========== ompl::base optimization objectives (#362) ==========
    // The types a createOptimizationObjective override builds an objective from. Python
    // cannot subclass the objectives; the C++ ones cover length, clearance and work.
    nb::class_<ob::SpaceInformation>(m, "SpaceInformation",
        "The planning problem's OMPL space information (opaque). The objectives are built on it.");

    nb::class_<ob::Cost>(m, "Cost", "An OMPL cost value")
        .def(nb::init<double>(), "value"_a = 0.0)
        .def("value", &ob::Cost::value)
        .def("__float__", &ob::Cost::value)
        .def("__repr__", [](const ob::Cost& c) { return "Cost(" + std::to_string(c.value()) + ")"; });

    nb::class_<ob::OptimizationObjective>(m, "OptimizationObjective",
        "Base of the OMPL optimization objectives. The planner's optimize loop ends when "
        "isSatisfied(cost of the best path) is true, or at planning_time.")
        .def("isSatisfied", &ob::OptimizationObjective::isSatisfied, "cost"_a,
            "True if cost is better than the cost threshold")
        .def("getCostThreshold", &ob::OptimizationObjective::getCostThreshold)
        .def("setCostThreshold", &ob::OptimizationObjective::setCostThreshold, "cost"_a)
        .def("getDescription", &ob::OptimizationObjective::getDescription);

    nb::class_<ob::PathLengthOptimizationObjective, ob::OptimizationObjective>(m, "PathLengthOptimizationObjective",
        "Joint-space path length (the planner's default objective)")
        .def(nb::init<const ob::SpaceInformationPtr&>(), "si"_a);

    nb::class_<ob::MaximizeMinClearanceObjective, ob::OptimizationObjective>(m, "MaximizeMinClearanceObjective",
        "Maximise the smallest clearance the state validity checker reports along the path. "
        "tesseract's OMPL state validators do not report clearance (OMPL's default, 0, for every "
        "state), so on this stack the term is constant.")
        .def(nb::init<const ob::SpaceInformationPtr&>(), "si"_a);

    nb::class_<ob::MechanicalWorkOptimizationObjective, ob::OptimizationObjective>(
        m, "MechanicalWorkOptimizationObjective", "Mechanical work along the path, plus a small path-length term")
        .def(nb::init<const ob::SpaceInformationPtr&, double>(), "si"_a, "path_length_weight"_a = MECHANICAL_WORK_PATH_LENGTH_WEIGHT_DEFAULT);

    nb::class_<ob::StateCostIntegralObjective, ob::OptimizationObjective, PyStateCostIntegralObjective>(
        m, "StateCostIntegralObjective",
        "Integral of a per-state cost along the path. Subclass it and override "
        "stateCost(self, values) -> float, where values holds the state's joint positions "
        "(rad or m per joint); the planner's thread takes the GIL for every call. Without "
        "an override the cost of every state is 1, so the integral is path length.")
        .def(nb::init<const ob::SpaceInformationPtr&, bool>(), "si"_a, "enable_motion_cost_interpolation"_a = false)
        .def("isMotionCostInterpolationEnabled", &ob::StateCostIntegralObjective::isMotionCostInterpolationEnabled);

    nb::class_<ob::MultiOptimizationObjective, ob::OptimizationObjective>(m, "MultiOptimizationObjective",
        "Weighted sum of objectives")
        .def(nb::init<const ob::SpaceInformationPtr&>(), "si"_a)
        .def("addObjective", &ob::MultiOptimizationObjective::addObjective, "objective"_a, "weight"_a)
        .def("getObjectiveCount", &ob::MultiOptimizationObjective::getObjectiveCount)
        .def("getObjectiveWeight", &ob::MultiOptimizationObjective::getObjectiveWeight, "index"_a)
        .def("setObjectiveWeight", &ob::MultiOptimizationObjective::setObjectiveWeight, "index"_a, "weight"_a);

    // ========== OMPLSolverConfig ==========
    nb::class_<tp::OMPLSolverConfig> solver_config(m, "OMPLSolverConfig");
    solver_config
        .def(nb::init<>())
        .def_rw("planning_time", &tp::OMPLSolverConfig::planning_time,
            "Max planning time allowed in seconds (default: 5.0)")
        .def_rw("max_solutions", &tp::OMPLSolverConfig::max_solutions,
            "Max number of solutions to find before exiting (default: 10)")
        .def_rw("simplify", &tp::OMPLSolverConfig::simplify,
            "Simplify trajectory (default: false). Ignores n_output_states if true.")
        .def_rw("simplify_time", &tp::OMPLSolverConfig::simplify_time,
            "Max time in seconds to spend simplifying when simplify is true (default: 0.0). "
            "<= 0 runs simplification to completion (unbounded); > 0 time-bounds it.")
        .def_rw("optimize", &tp::OMPLSolverConfig::optimize,
            "Use all planning time to optimize trajectory (default: true)")
        .def("addPlanner", [](tp::OMPLSolverConfig& self, std::shared_ptr<tp::OMPLPlannerConfigurator> planner) {
            self.planners.push_back(planner);
        }, "planner"_a, "Add a planner configurator (each runs in parallel)")
        .def("clearPlanners", [](tp::OMPLSolverConfig& self) {
            self.planners.clear();
        }, "Clear all planners");
    // Compares planning_time, max_solutions, simplify, optimize and the planner *count*
    // (upstream ompl_solver_config.cpp), not the configurators themselves.
    bind_value_equality(solver_config);

    // ========== OMPLPlannerType enum ==========
    nb::enum_<tp::OMPLPlannerType>(m, "OMPLPlannerType")
        .value("SBL", tp::OMPLPlannerType::SBL)
        .value("EST", tp::OMPLPlannerType::EST)
        .value("LBKPIECE1", tp::OMPLPlannerType::LBKPIECE1)
        .value("BKPIECE1", tp::OMPLPlannerType::BKPIECE1)
        .value("KPIECE1", tp::OMPLPlannerType::KPIECE1)
        .value("BiTRRT", tp::OMPLPlannerType::BiTRRT)
        .value("RRT", tp::OMPLPlannerType::RRT)
        .value("RRTConnect", tp::OMPLPlannerType::RRTConnect)
        .value("RRTstar", tp::OMPLPlannerType::RRTstar)
        .value("TRRT", tp::OMPLPlannerType::TRRT)
        .value("PRM", tp::OMPLPlannerType::PRM)
        .value("PRMstar", tp::OMPLPlannerType::PRMstar)
        .value("LazyPRMstar", tp::OMPLPlannerType::LazyPRMstar)
        .value("SPARS", tp::OMPLPlannerType::SPARS);

    // ========== OMPLPlannerConfigurator (base) ==========
    // create(ompl::base::SpaceInformationPtr) stays unbound: it takes and returns OMPL types.
    nb::class_<tp::OMPLPlannerConfigurator>(m, "OMPLPlannerConfigurator")
        .def("getType", &tp::OMPLPlannerConfigurator::getType);

    // ========== Configurators, one per OMPLPlannerType ==========
    nb::class_<tp::SBLConfigurator, tp::OMPLPlannerConfigurator>(m, "SBLConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::SBLConfigurator::range);

    nb::class_<tp::ESTConfigurator, tp::OMPLPlannerConfigurator>(m, "ESTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::ESTConfigurator::range)
        .def_rw("goal_bias", &tp::ESTConfigurator::goal_bias);

    nb::class_<tp::LBKPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "LBKPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::LBKPIECE1Configurator::range)
        .def_rw("border_fraction", &tp::LBKPIECE1Configurator::border_fraction)
        .def_rw("min_valid_path_fraction", &tp::LBKPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::BKPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "BKPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::BKPIECE1Configurator::range)
        .def_rw("border_fraction", &tp::BKPIECE1Configurator::border_fraction)
        .def_rw("failed_expansion_score_factor", &tp::BKPIECE1Configurator::failed_expansion_score_factor)
        .def_rw("min_valid_path_fraction", &tp::BKPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::KPIECE1Configurator, tp::OMPLPlannerConfigurator>(m, "KPIECE1Configurator")
        .def(nb::init<>())
        .def_rw("range", &tp::KPIECE1Configurator::range)
        .def_rw("goal_bias", &tp::KPIECE1Configurator::goal_bias)
        .def_rw("border_fraction", &tp::KPIECE1Configurator::border_fraction)
        .def_rw("failed_expansion_score_factor", &tp::KPIECE1Configurator::failed_expansion_score_factor)
        .def_rw("min_valid_path_fraction", &tp::KPIECE1Configurator::min_valid_path_fraction);

    nb::class_<tp::BiTRRTConfigurator, tp::OMPLPlannerConfigurator>(m, "BiTRRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::BiTRRTConfigurator::range)
        .def_rw("temp_change_factor", &tp::BiTRRTConfigurator::temp_change_factor)
        .def_rw("cost_threshold", &tp::BiTRRTConfigurator::cost_threshold)
        .def_rw("init_temperature", &tp::BiTRRTConfigurator::init_temperature)
        .def_rw("frontier_threshold", &tp::BiTRRTConfigurator::frontier_threshold)
        .def_rw("frontier_node_ratio", &tp::BiTRRTConfigurator::frontier_node_ratio);

    nb::class_<tp::RRTConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConfigurator::range)
        .def_rw("goal_bias", &tp::RRTConfigurator::goal_bias);

    nb::class_<tp::RRTConnectConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTConnectConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTConnectConfigurator::range);

    nb::class_<tp::RRTstarConfigurator, tp::OMPLPlannerConfigurator>(m, "RRTstarConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::RRTstarConfigurator::range)
        .def_rw("goal_bias", &tp::RRTstarConfigurator::goal_bias)
        .def_rw("delay_collision_checking", &tp::RRTstarConfigurator::delay_collision_checking);

    nb::class_<tp::TRRTConfigurator, tp::OMPLPlannerConfigurator>(m, "TRRTConfigurator")
        .def(nb::init<>())
        .def_rw("range", &tp::TRRTConfigurator::range)
        .def_rw("goal_bias", &tp::TRRTConfigurator::goal_bias)
        .def_rw("temp_change_factor", &tp::TRRTConfigurator::temp_change_factor)
        .def_rw("init_temperature", &tp::TRRTConfigurator::init_temperature)
        .def_rw("frontier_threshold", &tp::TRRTConfigurator::frontier_threshold)
        .def_rw("frontier_node_ratio", &tp::TRRTConfigurator::frontier_node_ratio);

    nb::class_<tp::PRMConfigurator, tp::OMPLPlannerConfigurator>(m, "PRMConfigurator")
        .def(nb::init<>())
        .def_rw("max_nearest_neighbors", &tp::PRMConfigurator::max_nearest_neighbors);

    nb::class_<tp::PRMstarConfigurator, tp::OMPLPlannerConfigurator>(m, "PRMstarConfigurator")
        .def(nb::init<>());

    nb::class_<tp::LazyPRMstarConfigurator, tp::OMPLPlannerConfigurator>(m, "LazyPRMstarConfigurator")
        .def(nb::init<>());

    nb::class_<tp::SPARSConfigurator, tp::OMPLPlannerConfigurator>(m, "SPARSConfigurator")
        .def(nb::init<>())
        .def_rw("max_failures", &tp::SPARSConfigurator::max_failures)
        .def_rw("dense_delta_fraction", &tp::SPARSConfigurator::dense_delta_fraction)
        .def_rw("sparse_delta_fraction", &tp::SPARSConfigurator::sparse_delta_fraction)
        .def_rw("stretch_factor", &tp::SPARSConfigurator::stretch_factor);

    // Bound after the configurators so the stub names them; the element type is
    // shared_ptr<const OMPLPlannerConfigurator>, cast to the bound derived class.
    solver_config.def_rw("planners", &tp::OMPLSolverConfig::planners,
        "Planner configurators, one parallel planner each. The list is copied on access; its "
        "configurators are shared with the config, so editing one edits the config. Assign a "
        "new list (or use addPlanner/clearPlanners) to change which planners it holds.");

    // ========== OMPLMoveProfile (base, was OMPLPlanProfile) ==========
    // createSimpleSetup / createStateExtractor stay unbound: they take or return OMPL types.
    nb::class_<tp::OMPLMoveProfile, tc::Profile>(m, "OMPLMoveProfile")
        .def("getKey", &tp::OMPLMoveProfile::getKey)
        // Every OMPL move profile shares the key of this base (OMPLMoveProfile()'s constructor).
        .def_static("createKey", &tc::Profile::createKey<tp::OMPLMoveProfile>)
        .def("createSolverConfig", &tp::OMPLMoveProfile::createSolverConfig,
            "The solver config the planner uses for this profile");

    // ========== OMPLRealVectorMoveProfile (was OMPLRealVectorPlanProfile) ==========
    // Subclassable: a Python subclass may override
    // createOptimizationObjective(si, env, manip) -> OptimizationObjective | None (#362).
    nb::class_<tp::OMPLRealVectorMoveProfile, tp::OMPLMoveProfile, PyOMPLRealVectorMoveProfile> real_vector_move_profile(
        m, "OMPLRealVectorMoveProfile");
    real_vector_move_profile
        .def(nb::init<>())
        .def_rw("solver_config", &tp::OMPLRealVectorMoveProfile::solver_config,
            "The OMPL parallel planner solver config")
        .def_rw("contact_manager_config", &tp::OMPLRealVectorMoveProfile::contact_manager_config,
            "The contact manager configuration (collision margins)")
        .def_rw("collision_check_config", &tp::OMPLRealVectorMoveProfile::collision_check_config,
            "The collision check configuration");
    bind_value_equality(real_vector_move_profile);
    bind_from_yaml(real_vector_move_profile,
        "Nested sub-configs are replaced, not merged: a key naming one rebuilds it from that "
        "struct's own defaults, so fields left out lose the profile's values. This is per "
        "struct; here `solver_config:` starts with no planners (OMPLSolverConfig() holds "
        "none).");

    // ========== OMPLMotionPlanner ==========
    // getName / solve (GIL released) / terminate / clear / clone come from MotionPlanner
    nb::class_<tp::OMPLMotionPlanner, tp::MotionPlanner>(m, "OMPLMotionPlanner")
        .def(nb::init<std::string>(), "name"_a);

    // ========== Global OMPL RNG seeding ==========
    m.def("RNG_setSeed", [](std::uint_fast32_t seed) { ompl::RNG::setSeed(seed); }, "seed"_a,
          "Seed OMPL's global RNG seed generator for reproducible planning.\n\n"
          "Samplers created after this call draw deterministic seeds, so repeated solve()\n"
          "calls produce identical paths. Re-seeding after planning has started logs an\n"
          "OMPL error (harmless: only pre-existing RNG instances stay nondeterministic).\n"
          "A seed of 0 is rejected once generation has started.");
    m.def("RNG_getSeed", []() { return ompl::RNG::getSeed(); },
          "Get the first seed used by OMPL's RNG seed generator.\n\n"
          "Log this value to make a stochastic planning failure reproducible by passing\n"
          "it to RNG_setSeed in a subsequent run.");
}
