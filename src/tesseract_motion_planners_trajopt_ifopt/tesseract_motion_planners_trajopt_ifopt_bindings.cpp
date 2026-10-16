/**
 * @file tesseract_motion_planners_trajopt_ifopt_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners TrajOpt IFOPT
 *
 * NOTE: 0.33 API changes:
 * - TrajOptIfoptPlanProfile -> TrajOptIfoptMoveProfile
 * - TrajOptIfoptDefaultPlanProfile -> TrajOptIfoptDefaultMoveProfile
 * - Profile/ProfileDictionary moved to tesseract_common
 */

#include "tesseract_nb.h"
#include "tesseract_nb_yaml.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>
#include <tesseract/motion_planners/planner.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners TrajOpt IFOPT
#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>
#include <tesseract/motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

// types named by the profile hooks and the term factories
#include <tesseract/command_language/poly/joint_waypoint_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/environment/environment.h>
#include <trajopt_ifopt/core/constraint_set.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>

// trajopt_common for collision config
#include <trajopt_common/collision_types.h>

// trajopt_sqp for SQPParameters (TrajOptIfoptSolverProfile::opt_params)
#include <trajopt_sqp/types.h>

// OsqpEigen settings (forwarded setters on TrajOptIfoptOSQPSolverProfile)
#include <OsqpEigen/Settings.hpp>

namespace tp = tesseract::motion_planners;
namespace tc = tesseract::common;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace ti = trajopt_ifopt;

namespace {
using ConstraintSets = std::vector<std::shared_ptr<ti::ConstraintSet>>;
using ConstVars = std::vector<std::shared_ptr<const ti::Var>>;

// upstream asserts !vars.empty() and then reads vars.front(); release builds have no assert.
const ConstVars& require_vars(const ConstVars& vars, const char* caller) {
    require_non_null(vars, caller, "vars", "Var");
    if (vars.empty())
        throw std::invalid_argument(std::string(caller) + ": vars is empty");
    return vars;
}

// One read-write list property per TrajOptIfoptTermInfos field. The list is copied on access
// (assign the whole list); None elements are rejected, upstream dereferences every set.
template <ConstraintSets tp::TrajOptIfoptTermInfos::*Field>
void def_term_list(nb::class_<tp::TrajOptIfoptTermInfos>& cls, const char* name) {
    cls.def_prop_rw(
        name,
        [](const tp::TrajOptIfoptTermInfos& self) { return self.*Field; },
        [name](tp::TrajOptIfoptTermInfos& self, const ConstraintSets& sets) {
            self.*Field = require_non_null(sets, "TrajOptIfoptTermInfos", name, "ConstraintSet");
        });
}

// A Python profile's create returned something other than the struct the planner takes. Its own
// type, not nb::type_error: solve's dispatcher lives in another module, whose catch does not match
// this module's nanobind exception types, so only a translator registered here maps it.
struct ProfileReturnTypeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A Python move profile's create returned a waypoint info without a node; the planner moves
// every node into its NodesVariables, which dereferences it.
struct WaypointInfoWithoutNodeError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A Python move profile's create returned a waypoint info something else still refers to (the
// override kept it, or a Node taken from its `node`). The planner moves the node out of the
// info and frees it with the problem, so that Node would dangle after solve.
struct WaypointInfoInUseError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// Python subclasses of the move and composite profiles. The instruction and manipulator-info
// arguments are const references into the planner's data and go to Python as copies; env and the
// nodes are shared_ptrs and are passed as such. The ticket takes the GIL, which solve released.
// A Python error propagates from solve: the planner does not catch it.
class PyTrajOptIfoptMoveProfile : public tp::TrajOptIfoptMoveProfile {
public:
    NB_TRAMPOLINE(tp::TrajOptIfoptMoveProfile, 1);

    tp::TrajOptIfoptWaypointInfo create(const tcl::MoveInstructionPoly& move_instruction,
                                        const tc::ManipulatorInfo& composite_manip_info,
                                        const std::shared_ptr<const te::Environment>& env,
                                        int index) const override {
        nb::detail::ticket t(nb_trampoline, "create", true);
        nb::object result = nb_trampoline.base().attr(t.key)(
            nb::cast(move_instruction, nb::rv_policy::copy), nb::cast(composite_manip_info, nb::rv_policy::copy),
            env, index);
        if (!nb::isinstance<tp::TrajOptIfoptWaypointInfo>(result))
            throw ProfileReturnTypeError("TrajOptIfoptMoveProfile.create must return a TrajOptIfoptWaypointInfo");
        auto& info = nb::cast<tp::TrajOptIfoptWaypointInfo&>(result);
        if (!info.node)
            throw WaypointInfoWithoutNodeError(
                "TrajOptIfoptMoveProfile.create returned a TrajOptIfoptWaypointInfo without a node; return "
                "the info a C++ move profile's create builds");
        // `result` is the one reference the call returned; any other is a holder the planner's
        // move would leave with a dangling node.
        if (Py_REFCNT(result.ptr()) != 1)
            throw WaypointInfoInUseError(
                "TrajOptIfoptMoveProfile.create returned a TrajOptIfoptWaypointInfo that is still referenced "
                "(kept by the profile, or through a Node from its `node`); the planner takes its node, so "
                "return an info nothing else holds");
        return std::move(info);
    }
};

class PyTrajOptIfoptCompositeProfile : public tp::TrajOptIfoptCompositeProfile {
public:
    NB_TRAMPOLINE(tp::TrajOptIfoptCompositeProfile, 1);

    tp::TrajOptIfoptTermInfos create(const tc::ManipulatorInfo& composite_manip_info,
                                     const std::shared_ptr<const te::Environment>& env,
                                     const std::vector<std::shared_ptr<const ti::Node>>& nodes,
                                     const std::vector<int>& fixed_indices) const override {
        nb::detail::ticket t(nb_trampoline, "create", true);
        nb::object result = nb_trampoline.base().attr(t.key)(
            nb::cast(composite_manip_info, nb::rv_policy::copy), env, nodes, fixed_indices);
        if (!nb::isinstance<tp::TrajOptIfoptTermInfos>(result))
            throw ProfileReturnTypeError("TrajOptIfoptCompositeProfile.create must return a TrajOptIfoptTermInfos");
        // A copy: the sets are shared_ptrs, and a Python set's deleter takes the GIL to release it.
        return nb::cast<tp::TrajOptIfoptTermInfos>(result);
    }
};
}  // namespace

NB_MODULE(_tesseract_motion_planners_trajopt_ifopt, m) {
    m.doc() = "tesseract_motion_planners_trajopt_ifopt Python bindings";
    register_python_error_translator();  // move and composite profile trampolines

    nb::exception<ProfileReturnTypeError>(m, "TrajOptIfoptProfileReturnTypeError", PyExc_TypeError)
        .attr("__doc__") = "A Python TrajOpt-Ifopt profile's create returned the wrong type.";
    nb::exception<WaypointInfoWithoutNodeError>(m, "TrajOptIfoptWaypointInfoWithoutNodeError", PyExc_ValueError)
        .attr("__doc__") = "A Python TrajOptIfoptMoveProfile.create returned a waypoint info without a node.";
    nb::exception<WaypointInfoInUseError>(m, "TrajOptIfoptWaypointInfoInUseError", PyExc_ValueError)
        .attr("__doc__") = "A Python TrajOptIfoptMoveProfile.create returned a waypoint info still referenced elsewhere.";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import MotionPlanner base type (TrajOptIfoptMotionPlanner's base)
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // Environment, ManipulatorInfo and JointWaypointPoly in the hook and factory signatures
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");

    // ========== trajopt_common::TrajOptCollisionConfig ==========
    // Owned by the trajopt_ifopt module; import so the composite profile's
    // collision_cost_config / collision_constraint_config members resolve
    // (same pattern as tesseract_motion_planners_trajopt).
    nb::module_::import_("tesseract_robotics.trajopt_ifopt._trajopt_ifopt");

    // ========== trajopt_sqp::SQPParameters ==========
    // Owned by the trajopt_sqp module; import so TrajOptIfoptSolverProfile's
    // opt_params member resolves.
    nb::module_::import_("tesseract_robotics.trajopt_sqp._trajopt_sqp");

    // ========== TrajOptIfoptCartesianWaypointConfig ==========
    nb::class_<tp::TrajOptIfoptCartesianWaypointConfig> cart_wp_config(m, "TrajOptIfoptCartesianWaypointConfig");
    cart_wp_config
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptIfoptCartesianWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptIfoptCartesianWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptIfoptCartesianWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptIfoptCartesianWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptIfoptCartesianWaypointConfig::coeff);
    bind_value_equality(cart_wp_config);

    // ========== TrajOptIfoptJointWaypointConfig ==========
    nb::class_<tp::TrajOptIfoptJointWaypointConfig> joint_wp_config(m, "TrajOptIfoptJointWaypointConfig");
    joint_wp_config
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptIfoptJointWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptIfoptJointWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptIfoptJointWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptIfoptJointWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptIfoptJointWaypointConfig::coeff);
    bind_value_equality(joint_wp_config);

    // ========== TrajOptIfoptTermInfos ==========
    // The constraint and cost sets a profile's create() hands the planner, which adds each
    // list to its TrajOptQPProblem as constraints / squared / absolute / hinge costs.
    nb::class_<tp::TrajOptIfoptTermInfos> term_infos(m, "TrajOptIfoptTermInfos",
        "Constraint and cost sets a TrajOpt-Ifopt profile adds to the planner's problem");
    term_infos.def(nb::init<>());
    def_term_list<&tp::TrajOptIfoptTermInfos::constraints>(term_infos, "constraints");
    def_term_list<&tp::TrajOptIfoptTermInfos::squared_costs>(term_infos, "squared_costs");
    def_term_list<&tp::TrajOptIfoptTermInfos::absolute_costs>(term_infos, "absolute_costs");
    def_term_list<&tp::TrajOptIfoptTermInfos::hinge_costs>(term_infos, "hinge_costs");

    // ========== TrajOptIfoptWaypointInfo ==========
    // Move-only: it owns its Node through a unique_ptr. `node` is read-only and refers into
    // this object (reference_internal), so a Python Node keeps its waypoint info alive.
    nb::class_<tp::TrajOptIfoptWaypointInfo>(m, "TrajOptIfoptWaypointInfo",
        "What a TrajOpt-Ifopt move profile builds for one waypoint: its variables and terms")
        .def(nb::init<>())
        .def_rw("term_infos", &tp::TrajOptIfoptWaypointInfo::term_infos)
        .def_prop_ro("node", [](tp::TrajOptIfoptWaypointInfo& self) { return self.node.get(); },
            nb::rv_policy::reference_internal,
            nb::for_getter(nb::sig("def node(self, /) -> tesseract_robotics.trajopt_ifopt._trajopt_ifopt.Node | None")),
            "The waypoint's variable node (owned by this object), or None")
        .def_rw("fixed", &tp::TrajOptIfoptWaypointInfo::fixed,
            "Whether the waypoint is fixed (no collision terms at this step)");

    // ========== TrajOptIfoptMoveProfile (base, was TrajOptIfoptPlanProfile) ==========
    // Python may subclass it and override create (PyTrajOptIfoptMoveProfile).
    nb::class_<tp::TrajOptIfoptMoveProfile, tc::Profile, PyTrajOptIfoptMoveProfile>(m, "TrajOptIfoptMoveProfile")
        .def(nb::init<>())
        .def("getKey", &tp::TrajOptIfoptMoveProfile::getKey)
        .def_static("createKey", &tc::Profile::createKey<tp::TrajOptIfoptMoveProfile>,
            "The key every TrajOpt-Ifopt move profile carries")
        .def("create", &tp::TrajOptIfoptMoveProfile::create,
            "move_instruction"_a, "composite_manip_info"_a, "env"_a, "index"_a,
            "Build the waypoint info the planner adds for this move instruction");

    // SWIG-compatible alias
    m.attr("TrajOptIfoptPlanProfile") = m.attr("TrajOptIfoptMoveProfile");

    // ========== TrajOptIfoptCompositeProfile (base) ==========
    // Python may subclass it and override create (PyTrajOptIfoptCompositeProfile).
    nb::class_<tp::TrajOptIfoptCompositeProfile, tc::Profile, PyTrajOptIfoptCompositeProfile>(
        m, "TrajOptIfoptCompositeProfile")
        .def(nb::init<>())
        .def("getKey", &tp::TrajOptIfoptCompositeProfile::getKey)
        .def_static("createKey", &tc::Profile::createKey<tp::TrajOptIfoptCompositeProfile>,
            "The key every TrajOpt-Ifopt composite profile carries")
        .def("create",
            [](const tp::TrajOptIfoptCompositeProfile& self, const tc::ManipulatorInfo& composite_manip_info,
               const std::shared_ptr<const tesseract::environment::Environment>& env,
               const std::vector<std::shared_ptr<const ti::Node>>& nodes, const std::vector<int>& fixed_indices) {
                return self.create(composite_manip_info, env,
                                   require_non_null(nodes, "TrajOptIfoptCompositeProfile.create", "nodes", "Node"),
                                   fixed_indices);
            },
            "composite_manip_info"_a, "env"_a, "nodes"_a, "fixed_indices"_a,
            "Build the trajectory-wide terms the planner adds over all waypoint nodes");

    // ========== TrajOptIfoptSolverProfile (base) ==========
    // No trampoline: create returns a unique_ptr to the concrete TrustRegionSQPSolver, which a
    // Python-allocated solver cannot become. Configure the solver through the fields (#340).
    // opt_params is the SQP loop itself (max_iterations, max_qp_solver_failures,
    // trust-region ratios, merit-coeff inflation) -- create() assigns it straight
    // onto the TrustRegionSQPSolver, so it is the only route to those knobs. The
    // sco back-end exposes them via TrajOptOSQPSolverProfile; unbound here, an
    // IFOPT migration silently loses them and runs on C++ defaults (max_iterations
    // 50, max_qp_solver_failures 3).
    nb::class_<tp::TrajOptIfoptSolverProfile, tc::Profile>(m, "TrajOptIfoptSolverProfile")
        .def_rw("opt_params", &tp::TrajOptIfoptSolverProfile::opt_params)
        // Copied on access: assign the whole list. Each callback is registered on the solver
        // create() builds; the planner's solve() calls them with the GIL taken per call.
        .def_prop_rw("callbacks",
            [](const tp::TrajOptIfoptSolverProfile& self) { return self.callbacks; },
            [](tp::TrajOptIfoptSolverProfile& self,
               const std::vector<std::shared_ptr<trajopt_sqp::SQPCallback>>& callbacks) {
                self.callbacks = require_non_null(callbacks, "TrajOptIfoptSolverProfile", "callbacks", "SQPCallback");
            },
            "SQP callbacks registered on the solver, called once per SQP iteration")
        .def("createOptimizationCallbacks", &tp::TrajOptIfoptSolverProfile::createOptimizationCallbacks)
        .def("getKey", &tp::TrajOptIfoptSolverProfile::getKey)
        .def_static("createKey", &tc::Profile::createKey<tp::TrajOptIfoptSolverProfile>,
            "The key every TrajOpt-Ifopt solver profile carries")
        .def("create", &tp::TrajOptIfoptSolverProfile::create, "verbose"_a = false,
            "Build the TrustRegionSQPSolver the planner solves with");

    // ========== TrajOptIfoptDefaultMoveProfile (was TrajOptIfoptDefaultPlanProfile) ==========
    nb::class_<tp::TrajOptIfoptDefaultMoveProfile, tp::TrajOptIfoptMoveProfile> default_move_profile(
        m, "TrajOptIfoptDefaultMoveProfile");
    default_move_profile
        .def(nb::init<>())
        .def_rw("cartesian_cost_config", &tp::TrajOptIfoptDefaultMoveProfile::cartesian_cost_config)
        .def_rw("cartesian_constraint_config", &tp::TrajOptIfoptDefaultMoveProfile::cartesian_constraint_config)
        .def_rw("joint_cost_config", &tp::TrajOptIfoptDefaultMoveProfile::joint_cost_config)
        .def_rw("joint_constraint_config", &tp::TrajOptIfoptDefaultMoveProfile::joint_constraint_config);
    bind_value_equality(default_move_profile);
    bind_from_yaml(default_move_profile);

    // SWIG-compatible alias
    m.attr("TrajOptIfoptDefaultPlanProfile") = m.attr("TrajOptIfoptDefaultMoveProfile");

    // ========== TrajOptIfoptDefaultCompositeProfile ==========
    // Note: longest_valid_segment_fraction/length removed in 0.33, collision config via TrajOptCollisionConfig.
    // The two collision configs carry max_num_cnt, which ONLY trajopt_ifopt honours
    // (trajopt_common/collision_types.h: "only used by trajopt_ifopt because the
    // constraints size must be fixed") -- it caps the collision constraint at that
    // many rows per timestep, one per worst link pair, so QP size stops tracking
    // contact count. Unbound they are unreachable and the planner silently runs on
    // C++ defaults: no margins, no buffers, no modify_collision_objects scoping.
    nb::class_<tp::TrajOptIfoptDefaultCompositeProfile, tp::TrajOptIfoptCompositeProfile> default_composite_profile(
        m, "TrajOptIfoptDefaultCompositeProfile");
    default_composite_profile
        .def(nb::init<>())
        .def_rw("collision_cost_config", &tp::TrajOptIfoptDefaultCompositeProfile::collision_cost_config)
        .def_rw("collision_constraint_config", &tp::TrajOptIfoptDefaultCompositeProfile::collision_constraint_config)
        .def_rw("smooth_velocities", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_velocities)
        .def_rw("velocity_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::velocity_coeff)
        .def_rw("smooth_accelerations", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_accelerations)
        .def_rw("acceleration_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::acceleration_coeff)
        .def_rw("smooth_jerks", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_jerks)
        .def_rw("jerk_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::jerk_coeff);
    bind_value_equality(default_composite_profile);
    bind_from_yaml(default_composite_profile);

    // ========== TrajOptIfoptOSQPSolverProfile ==========
    // qp_settings is a unique_ptr<OsqpEigen::Settings>; expose setters that forward into it
    // (same pattern as trajopt_sqp.OSQPEigenSolver). Defaults via setDefaultOSQPSettings:
    // polish=true, warmStart=true, adaptiveRho=true, maxIter=8192, absTol=1e-4, relTol=1e-6
    nb::class_<tp::TrajOptIfoptOSQPSolverProfile, tp::TrajOptIfoptSolverProfile> osqp_solver_profile(
        m, "TrajOptIfoptOSQPSolverProfile");
    osqp_solver_profile
        .def(nb::init<>())
        .def("setPolish", [](tp::TrajOptIfoptOSQPSolverProfile& self, bool v) {
            self.qp_settings->setPolish(v); }, "polish"_a,
            "Enable solution polishing (default: true)")
        .def("setWarmStart", [](tp::TrajOptIfoptOSQPSolverProfile& self, bool v) {
            self.qp_settings->setWarmStart(v); }, "warm_start"_a,
            "Enable warm-starting (default: true)")
        .def("setAdaptiveRho", [](tp::TrajOptIfoptOSQPSolverProfile& self, bool v) {
            self.qp_settings->setAdaptiveRho(v); }, "adaptive_rho"_a,
            "Enable adaptive step size (default: true)")
        .def("setAdaptiveRhoInterval", [](tp::TrajOptIfoptOSQPSolverProfile& self, int v) {
            self.qp_settings->setAdaptiveRhoInterval(v); }, "interval"_a,
            "Adapt rho every N iterations. OSQP's default 0 adapts on wall-clock timing,\n"
            "making solutions nondeterministic run-to-run; set a fixed interval (e.g. 25)\n"
            "for reproducible optimization.")
        .def("setMaxIteration", [](tp::TrajOptIfoptOSQPSolverProfile& self, int v) {
            self.qp_settings->setMaxIteration(v); }, "max_iter"_a,
            "Max OSQP iterations per QP solve (default: 8192)")
        .def("setAbsoluteTolerance", [](tp::TrajOptIfoptOSQPSolverProfile& self, double v) {
            self.qp_settings->setAbsoluteTolerance(v); }, "abs_tol"_a,
            "Absolute convergence tolerance (default: 1e-4)")
        .def("setRelativeTolerance", [](tp::TrajOptIfoptOSQPSolverProfile& self, double v) {
            self.qp_settings->setRelativeTolerance(v); }, "rel_tol"_a,
            "Relative convergence tolerance (default: 1e-6)")
        .def("setVerbosity", [](tp::TrajOptIfoptOSQPSolverProfile& self, bool v) {
            self.qp_settings->setVerbosity(v); }, "verbose"_a,
            "Enable OSQP console output (default: false)");
    // operator== also compares qp_settings, through upstream's free OsqpEigen::Settings operator==.
    bind_value_equality(osqp_solver_profile);
    bind_from_yaml(osqp_solver_profile);

    // Helper to add TrajOptIfopt move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptMoveProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptIfoptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt move profile to ProfileDictionary");

    // Legacy alias
    m.def("ProfileDictionary_addTrajOptIfoptPlanProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptIfoptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt plan profile to ProfileDictionary (legacy alias)");

    // Helper to add TrajOptIfopt composite profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptCompositeProfile", [](tc::ProfileDictionary& dict,
                                                                   const std::string& ns,
                                                                   const std::string& profile_name,
                                                                   std::shared_ptr<tp::TrajOptIfoptCompositeProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt composite profile to ProfileDictionary");

    // Helper to add TrajOptIfopt solver profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptSolverProfile", [](tc::ProfileDictionary& dict,
                                                                const std::string& ns,
                                                                const std::string& profile_name,
                                                                std::shared_ptr<tp::TrajOptIfoptSolverProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt solver profile to ProfileDictionary");

    // ========== Term factories (trajopt_ifopt_utils.h) ==========
    // The planner's own term set-up; a hand-built TrajOptQPProblem gets the same sets.
    // copyOSQPEigenSettings takes the unbound OsqpEigen::Settings and stays unbound.
    m.def("createJointPositionConstraint", &tp::createJointPositionConstraint,
          "joint_waypoint"_a, "var"_a, "coeffs"_a,
          "Joint position constraint at a waypoint: equality to its position, or its tolerance band");
    m.def("createCollisionConstraints",
          [](const ConstVars& vars, const std::shared_ptr<const tesseract::environment::Environment>& env,
             const tc::ManipulatorInfo& manip_info, const trajopt_common::TrajOptCollisionConfig& config,
             const std::vector<int>& fixed_indices, bool fixed_sparsity) {
              return tp::createCollisionConstraints(require_vars(vars, "createCollisionConstraints"), env,
                                                    manip_info, config, fixed_indices, fixed_sparsity);
          },
          "vars"_a, "env"_a, "manip_info"_a, "config"_a, "fixed_indices"_a, "fixed_sparsity"_a,
          "Collision constraint sets of config's collision check type: one per unfixed step (DISCRETE)\n"
          "or one per segment (LVS_DISCRETE, LVS_CONTINUOUS, CONTINUOUS); none for NONE");
    m.def("createJointVelocityConstraint",
          [](const Eigen::Ref<const Eigen::VectorXd>& target, const ConstVars& vars, const Eigen::VectorXd& coeffs) {
              return tp::createJointVelocityConstraint(target, require_vars(vars, "createJointVelocityConstraint"),
                                                       coeffs);
          },
          "target"_a, "vars"_a, "coeffs"_a, "Joint velocity constraint over vars toward target");
    m.def("createJointAccelerationConstraint",
          [](const Eigen::Ref<const Eigen::VectorXd>& target, const ConstVars& vars, const Eigen::VectorXd& coeffs) {
              return tp::createJointAccelerationConstraint(
                  target, require_vars(vars, "createJointAccelerationConstraint"), coeffs);
          },
          "target"_a, "vars"_a, "coeffs"_a, "Joint acceleration constraint over vars toward target");
    m.def("createJointJerkConstraint",
          [](const Eigen::Ref<const Eigen::VectorXd>& target, const ConstVars& vars, const Eigen::VectorXd& coeffs) {
              return tp::createJointJerkConstraint(target, require_vars(vars, "createJointJerkConstraint"), coeffs);
          },
          "target"_a, "vars"_a, "coeffs"_a, "Joint jerk constraint over vars toward target");

    // ========== TrajOptIfoptMotionPlanner ==========
    // getName/solve/terminate/clear/clone are MotionPlanner's (solve releases the GIL there)
    nb::class_<tp::TrajOptIfoptMotionPlanner, tp::MotionPlanner>(m, "TrajOptIfoptMotionPlanner")
        .def(nb::init<std::string>(), "name"_a);
}
