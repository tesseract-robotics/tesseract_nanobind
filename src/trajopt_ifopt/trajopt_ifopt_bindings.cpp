/**
 * @file trajopt_ifopt_bindings.cpp
 * @brief nanobind bindings for trajopt_ifopt variables, constraints, and costs
 *
 * NOTE: Default arguments for Eigen types (Isometry3d, Matrix) cause std::bad_cast
 * at module load time. All Eigen defaults are avoided - use default constructor
 * and set members instead.
 */

#include "tesseract_nb.h"

// trajopt_ifopt core (replaces ifopt base classes)
#include <trajopt_ifopt/core/bounds.h>
#include <trajopt_ifopt/core/component.h>
#include <trajopt_ifopt/core/constraint_set.h>

// trajopt_ifopt variable sets (new variable system)
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

// trajopt_ifopt constraints
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/joint_acceleration_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h>
#include <trajopt_ifopt/constraints/joint_jerk_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>

// trajopt_common
#include <trajopt_common/collision_types.h>

// tesseract_collision for CollisionCheckConfig (used by TrajOptCollisionConfig)
#include <tesseract_collision/core/types.h>

// tesseract dependencies
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_environment/environment.h>

namespace ti = trajopt_ifopt;
namespace tc = trajopt_common;

NB_MODULE(_trajopt_ifopt, m) {
    m.doc() = "trajopt_ifopt Python bindings - variables, constraints, and costs for IFOPT-based trajectory optimization";

    // Import tesseract_collision for CollisionCheckConfig type
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== BoundsType enum ==========
    nb::enum_<ti::BoundsType>(m, "BoundsType")
        .value("UNBOUNDED", ti::BoundsType::kUnbounded)
        .value("EQUALITY", ti::BoundsType::kEquality)
        .value("LOWER_BOUND", ti::BoundsType::kLowerBound)
        .value("UPPER_BOUND", ti::BoundsType::kUpperBound)
        .value("RANGE_BOUND", ti::BoundsType::kRangeBound);

    // ========== RangeBoundHandling enum ==========
    nb::enum_<ti::RangeBoundHandling>(m, "RangeBoundHandling")
        .value("KEEP_AS_IS", ti::RangeBoundHandling::kKeepAsIs)
        .value("SPLIT_TO_TWO_INEQUALITIES", ti::RangeBoundHandling::kSplitToTwoInequalities);

    // ========== Bounds ==========
    nb::class_<ti::Bounds>(m, "Bounds")
        .def(nb::init<double, double>(), "lower"_a = 0.0, "upper"_a = 0.0,
             "Create bounds with lower and upper limits")
        .def("set", &ti::Bounds::set, "lower"_a, "upper"_a,
             "Set both lower and upper bounds")
        .def("setLower", &ti::Bounds::setLower, "lower"_a,
             "Set the lower bound")
        .def("getLower", &ti::Bounds::getLower,
             "Get the lower bound value")
        .def("setUpper", &ti::Bounds::setUpper, "upper"_a,
             "Set the upper bound")
        .def("getUpper", &ti::Bounds::getUpper,
             "Get the upper bound value")
        .def("getType", &ti::Bounds::getType,
             "Get the cached bound classification");

    // ========== Component (base class) ==========
    nb::class_<ti::Component>(m, "Component")
        .def("getName", &ti::Component::getName,
             "Get the component name")
        .def("getRows", &ti::Component::getRows,
             "Get the number of rows");

    // ========== Variables (inherits Component) ==========
    nb::class_<ti::Variables, ti::Component>(m, "Variables")
        .def("getValues", &ti::Variables::getValues,
             "Get the current variable values")
        .def("setVariables", &ti::Variables::setVariables, "x"_a,
             "Set the variable values from a flat decision vector")
        .def("getBounds", &ti::Variables::getBounds,
             "Get the variable bounds")
        .def("getHash", &ti::Variables::getHash,
             "Get the variable hash for caching");

    // ========== Differentiable (inherits Component) ==========
    nb::class_<ti::Differentiable, ti::Component>(m, "Differentiable")
        .def("isDynamic", &ti::Differentiable::isDynamic,
             "Whether this component can change its number of rows")
        .def("isScalar", &ti::Differentiable::isScalar,
             "Whether this component is scalar or stacked")
        .def("getNonZeros", &ti::Differentiable::getNonZeros,
             "Estimated number of non-zero entries in the Jacobian")
        .def("update", &ti::Differentiable::update,
             "Recompute sizing/state for dynamic-sized components")
        .def("getCoefficients", &ti::Differentiable::getCoefficients,
             "Get the coefficient vector");

    // ========== ConstraintSet (inherits Differentiable) ==========
    nb::class_<ti::ConstraintSet, ti::Differentiable>(m, "ConstraintSet")
        .def("linkWithVariables", &ti::ConstraintSet::linkWithVariables, "x"_a,
             "Connect the constraint with the optimization variables");

    // ========== Var (variable segment) ==========
    nb::class_<ti::Var>(m, "Var")
        .def("getIdentifier", &ti::Var::getIdentifier,
             "Get the identifier for the variable segment")
        .def("getIndex", &ti::Var::getIndex,
             "Get the starting index of this variable segment")
        .def("size", &ti::Var::size,
             "Get the number of elements in this variable segment")
        .def("value", &ti::Var::value,
             "Get the current variable values")
        .def("name", &ti::Var::name,
             "Get the variable names")
        .def("bounds", &ti::Var::bounds,
             "Get the variable bounds")
        .def("setVariables", &ti::Var::setVariables, "x"_a,
             "Set the variable values from the full decision vector");

    // ========== Node (group of related variables) ==========
    nb::class_<ti::Node>(m, "Node")
        .def(nb::init<std::string>(), "node_name"_a = "Node",
             "Create a node with an optional name")
        .def("getName", &ti::Node::getName,
             "Get the name of this node")
        .def("addVar",
             nb::overload_cast<const std::string&, double, ti::Bounds>(&ti::Node::addVar),
             "name"_a, "value"_a, "bounds"_a = ti::NoBound,
             "Add a scalar variable to this node")
        .def("addVar",
             nb::overload_cast<const std::string&, const std::vector<std::string>&,
                               const Eigen::VectorXd&, const std::vector<ti::Bounds>&>(&ti::Node::addVar),
             "name"_a, "child_names"_a, "values"_a, "bounds"_a,
             "Add a vector-valued variable to this node")
        .def("hasVar", &ti::Node::hasVar, "name"_a,
             "Check whether this node has a variable by name")
        .def("getVar", &ti::Node::getVar, "name"_a,
             "Get a variable by name")
        .def("getValues", &ti::Node::getValues,
             "Get all variable values as a single vector")
        .def("getBounds", &ti::Node::getBounds,
             "Get all variable bounds")
        .def("size", &ti::Node::size,
             "Get total number of scalar decision variables")
        .def("setVariables", &ti::Node::setVariables, "x"_a,
             "Update all variable values from the full decision vector");

    // ========== NodesVariables (main variable container, replaces JointPosition) ==========
    nb::class_<ti::NodesVariables, ti::Variables>(m, "NodesVariables")
        .def("getNode", &ti::NodesVariables::getNode, "opt_idx"_a,
             "Get node based on index")
        .def("getNodes", &ti::NodesVariables::getNodes,
             "Get all nodes")
        .def("getDim", &ti::NodesVariables::getDim,
             "Get the dimensions of every node");

    // ========== Factory for NodesVariables ==========
    // Node is non-copyable and uses unique_ptr ownership, which doesn't play well
    // with Python object ownership. This factory creates everything on the C++ side.
    m.def("createNodesVariables",
          [](const std::string& variable_name,
             const std::vector<std::string>& joint_names,
             const std::vector<Eigen::VectorXd>& initial_values,
             const std::vector<ti::Bounds>& bounds) -> std::shared_ptr<ti::NodesVariables> {
              std::vector<std::unique_ptr<ti::Node>> nodes;
              nodes.reserve(initial_values.size());
              for (size_t i = 0; i < initial_values.size(); ++i) {
                  auto node = std::make_unique<ti::Node>(variable_name + "_" + std::to_string(i));
                  node->addVar("joints", joint_names, initial_values[i], bounds);
                  nodes.push_back(std::move(node));
              }
              return std::make_shared<ti::NodesVariables>(variable_name, std::move(nodes));
          },
          "variable_name"_a, "joint_names"_a, "initial_values"_a, "bounds"_a,
          "Create NodesVariables with one vector Var per node.\n\n"
          "Args:\n"
          "    variable_name: Name for the variable set\n"
          "    joint_names: Joint name for each DOF\n"
          "    initial_values: List of initial joint value vectors (one per waypoint)\n"
          "    bounds: Joint bounds (shared across all waypoints)");

    // ========== Utility functions ==========
    m.def("interpolate", &ti::interpolate, "start"_a, "end"_a, "steps"_a,
          "Interpolate between two joint vectors, returning 'steps' waypoints");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::MatrixX2d>&>(&ti::toBounds),
          "limits"_a, "Convert Nx2 matrix [lower, upper] to vector of Bounds");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                         const Eigen::Ref<const Eigen::VectorXd>&>(&ti::toBounds),
          "lower_limits"_a, "upper_limits"_a, "Convert lower/upper limit vectors to vector of Bounds");

    // ========== CartPosConstraint ==========
    // Constructor takes individual parameters (CartPosInfo struct removed)
    nb::class_<ti::CartPosConstraint, ti::ConstraintSet>(m, "CartPosConstraint")
        .def("__init__", [](ti::CartPosConstraint* self,
                            std::shared_ptr<const ti::Var> position_var,
                            std::shared_ptr<const tesseract_kinematics::JointGroup> manip,
                            std::string source_frame,
                            std::string target_frame,
                            const Eigen::Isometry3d& source_frame_offset,
                            const Eigen::Isometry3d& target_frame_offset,
                            const std::string& name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) ti::CartPosConstraint(position_var, manip,
                     std::move(source_frame), std::move(target_frame),
                     source_frame_offset, target_frame_offset,
                     name, range_bound_handling);
             },
             "position_var"_a, "manip"_a,
             "source_frame"_a, "target_frame"_a,
             "source_frame_offset"_a, "target_frame_offset"_a,
             "name"_a = "CartPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create Cartesian position constraint")
        .def("calcValues", &ti::CartPosConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("getValues", &ti::CartPosConstraint::getValues,
             "Get current constraint values")
        .def("setTargetPose", &ti::CartPosConstraint::setTargetPose, "target_frame_offset"_a,
             "Set the target pose - critical for online planning!")
        .def("getTargetPose", &ti::CartPosConstraint::getTargetPose,
             "Get the target pose for the constraint")
        .def("getCurrentPose", &ti::CartPosConstraint::getCurrentPose,
             "Get current TCP pose in world frame")
        .def_rw("use_numeric_differentiation", &ti::CartPosConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== JointPosConstraint ==========
    nb::class_<ti::JointPosConstraint, ti::ConstraintSet>(m, "JointPosConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::shared_ptr<const ti::Var>&,
                      const Eigen::VectorXd&, std::string, ti::RangeBoundHandling>(),
             "target"_a, "position_var"_a, "coeffs"_a, "name"_a = "JointPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create joint position constraint with target values")
        .def("getValues", &ti::JointPosConstraint::getValues,
             "Get current constraint values");

    // ========== JointVelConstraint ==========
    nb::class_<ti::JointVelConstraint, ti::ConstraintSet>(m, "JointVelConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::Var>>&,
                      const Eigen::VectorXd&, std::string>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointVel",
             "Create joint velocity constraint with target values")
        .def("getValues", &ti::JointVelConstraint::getValues,
             "Get current constraint values");

    // ========== JointAccelConstraint ==========
    nb::class_<ti::JointAccelConstraint, ti::ConstraintSet>(m, "JointAccelConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::Var>>&,
                      const Eigen::VectorXd&, std::string>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointAccel",
             "Create joint acceleration constraint with target values")
        .def("getValues", &ti::JointAccelConstraint::getValues,
             "Get current constraint values");

    // ========== trajopt_common::CollisionCoeffData ==========
    nb::class_<tc::CollisionCoeffData>(m, "CollisionCoeffData")
        .def(nb::init<>())
        .def(nb::init<double>(), "default_collision_coeff"_a)
        .def("setDefaultCollisionCoeff", &tc::CollisionCoeffData::setDefaultCollisionCoeff)
        .def("getDefaultCollisionCoeff", &tc::CollisionCoeffData::getDefaultCollisionCoeff)
        .def("setCollisionCoeff", &tc::CollisionCoeffData::setCollisionCoeff,
             "obj1"_a, "obj2"_a, "collision_coeff"_a)
        .def("getCollisionCoeff", &tc::CollisionCoeffData::getCollisionCoeff,
             "obj1"_a, "obj2"_a)
        // Backwards compatibility aliases
        .def("setPairCollisionCoeff", &tc::CollisionCoeffData::setCollisionCoeff,
             "obj1"_a, "obj2"_a, "collision_coeff"_a)
        .def("getPairCollisionCoeff", &tc::CollisionCoeffData::getCollisionCoeff,
             "obj1"_a, "obj2"_a);

    // ========== trajopt_common::TrajOptCollisionConfig ==========
    nb::class_<tc::TrajOptCollisionConfig>(m, "TrajOptCollisionConfig")
        .def(nb::init<>())
        .def(nb::init<double, double>(), "margin"_a, "coeff"_a)
        .def_rw("enabled", &tc::TrajOptCollisionConfig::enabled)
        .def_rw("contact_manager_config", &tc::TrajOptCollisionConfig::contact_manager_config)
        .def_rw("collision_check_config", &tc::TrajOptCollisionConfig::collision_check_config)
        .def_rw("collision_coeff_data", &tc::TrajOptCollisionConfig::collision_coeff_data)
        .def_rw("collision_margin_buffer", &tc::TrajOptCollisionConfig::collision_margin_buffer)
        .def_rw("max_num_cnt", &tc::TrajOptCollisionConfig::max_num_cnt);

    // ========== DiscreteCollisionEvaluator (base class) ==========
    nb::class_<ti::DiscreteCollisionEvaluator>(m, "DiscreteCollisionEvaluator")
        .def("getCollisionMarginBuffer", &ti::DiscreteCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::DiscreteCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::DiscreteCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== SingleTimestepCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::SingleTimestepCollisionEvaluator, ti::DiscreteCollisionEvaluator>(m, "SingleTimestepCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== DiscreteCollisionConstraint ==========
    nb::class_<ti::DiscreteCollisionConstraint, ti::ConstraintSet>(m, "DiscreteCollisionConstraint")
        .def(nb::init<std::shared_ptr<ti::DiscreteCollisionEvaluator>,
                      std::shared_ptr<const ti::Var>,
                      int, bool, std::string>(),
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollision")
        .def("getValues", &ti::DiscreteCollisionConstraint::getValues)
        .def("getCollisionEvaluator", &ti::DiscreteCollisionConstraint::getCollisionEvaluator);

    // ========== ContinuousCollisionEvaluator (base class) ==========
    nb::class_<ti::ContinuousCollisionEvaluator>(m, "ContinuousCollisionEvaluator")
        .def("getCollisionMarginBuffer", &ti::ContinuousCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::ContinuousCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::ContinuousCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== LVSDiscreteCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSDiscreteCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSDiscreteCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== LVSContinuousCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSContinuousCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSContinuousCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== ContinuousCollisionConstraint ==========
    nb::class_<ti::ContinuousCollisionConstraint, ti::ConstraintSet>(m, "ContinuousCollisionConstraint")
        .def("__init__", [](ti::ContinuousCollisionConstraint* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {var0, var1};
                 new (self) ti::ContinuousCollisionConstraint(eval, vars, fixed0, fixed1, max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision")
        .def("getValues", &ti::ContinuousCollisionConstraint::getValues)
        .def("getCollisionEvaluator", &ti::ContinuousCollisionConstraint::getCollisionEvaluator);

    // ========== JointJerkConstraint ==========
    nb::class_<ti::JointJerkConstraint, ti::ConstraintSet>(m, "JointJerkConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::Var>>&,
                      const Eigen::VectorXd&, std::string>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointJerk",
             "Create joint jerk constraint (requires 4+ consecutive waypoints)")
        .def("getValues", &ti::JointJerkConstraint::getValues,
             "Get current constraint values");

    // ========== CartLineInfo (struct) ==========
    nb::class_<ti::CartLineInfo>(m, "CartLineInfo")
        .def(nb::init<>())
        .def_rw("manip", &ti::CartLineInfo::manip, "The joint group")
        .def_rw("source_frame", &ti::CartLineInfo::source_frame, "TCP frame")
        .def_rw("target_frame", &ti::CartLineInfo::target_frame, "Reference frame")
        .def_rw("source_frame_offset", &ti::CartLineInfo::source_frame_offset, "TCP offset")
        .def_rw("target_frame_offset1", &ti::CartLineInfo::target_frame_offset1, "Line start pose")
        .def_rw("target_frame_offset2", &ti::CartLineInfo::target_frame_offset2, "Line end pose")
        .def_rw("indices", &ti::CartLineInfo::indices,
                "DOF indices to constrain: default {0,1,2,3,4,5}");

    // ========== CartLineConstraint ==========
    nb::class_<ti::CartLineConstraint, ti::ConstraintSet>(m, "CartLineConstraint")
        .def(nb::init<ti::CartLineInfo, std::shared_ptr<const ti::Var>,
                      const Eigen::VectorXd&, std::string>(),
             "info"_a, "position_var"_a, "coeffs"_a, "name"_a = "CartLine",
             "Create Cartesian line constraint")
        .def("getValues", &ti::CartLineConstraint::getValues,
             "Get current constraint values")
        .def("calcValues", &ti::CartLineConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("getLinePoint", &ti::CartLineConstraint::getLinePoint,
             "source_tf"_a, "target_tf1"_a, "target_tf2"_a,
             "Find nearest point on line (uses SLERP for orientation)")
        .def_rw("use_numeric_differentiation", &ti::CartLineConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== DiscreteCollisionNumericalConstraint ==========
    nb::class_<ti::DiscreteCollisionNumericalConstraint, ti::ConstraintSet>(m, "DiscreteCollisionNumericalConstraint")
        .def(nb::init<std::shared_ptr<ti::DiscreteCollisionEvaluator>,
                      std::shared_ptr<const ti::Var>,
                      int, bool, std::string>(),
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollisionNumerical",
             "Create discrete collision constraint with numerical jacobians")
        .def("getValues", &ti::DiscreteCollisionNumericalConstraint::getValues)
        .def("calcValues", &ti::DiscreteCollisionNumericalConstraint::calcValues, "joint_vals"_a)
        .def("getCollisionEvaluator", &ti::DiscreteCollisionNumericalConstraint::getCollisionEvaluator);

    // ========== InverseKinematicsInfo (struct) ==========
    nb::class_<ti::InverseKinematicsInfo>(m, "InverseKinematicsInfo")
        .def(nb::init<>())
        .def_rw("manip", &ti::InverseKinematicsInfo::manip, "The kinematic group (with IK solver)")
        .def_rw("working_frame", &ti::InverseKinematicsInfo::working_frame, "Working frame (not currently used)")
        .def_rw("tcp_frame", &ti::InverseKinematicsInfo::tcp_frame, "TCP frame (not currently used)")
        .def_rw("tcp_offset", &ti::InverseKinematicsInfo::tcp_offset, "TCP offset (not currently used)");

    // ========== InverseKinematicsConstraint ==========
    nb::class_<ti::InverseKinematicsConstraint, ti::ConstraintSet>(m, "InverseKinematicsConstraint")
        .def(nb::init<const Eigen::Isometry3d&, ti::InverseKinematicsInfo::ConstPtr,
                      std::shared_ptr<const ti::Var>,
                      std::shared_ptr<const ti::Var>,
                      std::string>(),
             "target_pose"_a, "kinematic_info"_a, "constraint_var"_a, "seed_var"_a,
             "name"_a = "InverseKinematics",
             "Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)")
        .def("getValues", &ti::InverseKinematicsConstraint::getValues,
             "Get current constraint values");
}
