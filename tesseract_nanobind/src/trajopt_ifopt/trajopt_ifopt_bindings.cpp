/**
 * @file trajopt_ifopt_bindings.cpp
 * @brief nanobind bindings for trajopt_ifopt variables, constraints, and costs
 *
 * NOTE: Default arguments for Eigen types (Isometry3d, Matrix) cause std::bad_cast
 * at module load time. All Eigen defaults are avoided - use default constructor
 * and set members instead.
 */

#include "tesseract_nb.h"

// ifopt base classes (for cross-module inheritance)
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>

// trajopt_ifopt variable sets
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
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

    // Import ifopt module to enable cross-module inheritance
    nb::module_::import_("tesseract_robotics.ifopt._ifopt");

    // Import tesseract_collision for CollisionCheckConfig type
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== JointPosition (variable set) ==========
    // Inherits from ifopt::VariableSet for cross-module compatibility
    nb::class_<ti::JointPosition, ifopt::VariableSet>(m, "JointPosition")
        .def(nb::init<const Eigen::Ref<const Eigen::VectorXd>&, std::vector<std::string>, const std::string&>(),
             "init_value"_a, "joint_names"_a, "name"_a = "Joint_Position",
             "Create joint position variable set with initial values and joint names")
        .def("SetVariables", &ti::JointPosition::SetVariables, "x"_a,
             "Set the joint position values")
        .def("GetValues", &ti::JointPosition::GetValues,
             "Get the current joint position values")
        .def("GetJointNames", &ti::JointPosition::GetJointNames,
             "Get the joint names associated with this variable set")
        .def("GetRows", &ti::JointPosition::GetRows,
             "Get number of variables")
        .def("GetName", &ti::JointPosition::GetName,
             "Get the name of this variable set")
        .def("SetBounds", nb::overload_cast<const Eigen::Ref<const Eigen::MatrixX2d>&>(&ti::JointPosition::SetBounds),
             "bounds"_a, "Set joint bounds from Nx2 matrix [lower, upper]")
        .def("GetBounds", &ti::JointPosition::GetBounds,
             "Get the variable bounds");

    // ========== Utility functions ==========
    m.def("interpolate", &ti::interpolate, "start"_a, "end"_a, "steps"_a,
          "Interpolate between two joint vectors, returning 'steps' waypoints");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::MatrixX2d>&>(&ti::toBounds),
          "limits"_a, "Convert Nx2 matrix [lower, upper] to vector of ifopt::Bounds");

    // ========== CartPosInfo::Type enum ==========
    nb::enum_<ti::CartPosInfo::Type>(m, "CartPosInfoType")
        .value("TARGET_ACTIVE", ti::CartPosInfo::Type::TARGET_ACTIVE)
        .value("SOURCE_ACTIVE", ti::CartPosInfo::Type::SOURCE_ACTIVE)
        .value("BOTH_ACTIVE", ti::CartPosInfo::Type::BOTH_ACTIVE);

    // ========== CartPosInfo (struct) ==========
    // Use default constructor + member assignment instead of parametrized constructor
    // (Eigen default args cause std::bad_cast)
    nb::class_<ti::CartPosInfo>(m, "CartPosInfo")
        .def(nb::init<>())
        .def_rw("manip", &ti::CartPosInfo::manip, "The joint group")
        .def_rw("source_frame", &ti::CartPosInfo::source_frame, "Link which should reach desired pos")
        .def_rw("target_frame", &ti::CartPosInfo::target_frame, "Target frame to be reached")
        .def_rw("source_frame_offset", &ti::CartPosInfo::source_frame_offset, "Static transform applied to source_frame")
        .def_rw("target_frame_offset", &ti::CartPosInfo::target_frame_offset, "Static transform applied to target_frame")
        .def_rw("type", &ti::CartPosInfo::type, "Indicates which link is active")
        .def_rw("indices", &ti::CartPosInfo::indices,
                "Indices to return: default {0,1,2,3,4,5}. Position: {0,1,2}, Rotation: {3,4,5}");

    // ========== CartPosConstraint ==========
    nb::class_<ti::CartPosConstraint, ifopt::ConstraintSet>(m, "CartPosConstraint")
        .def(nb::init<const ti::CartPosInfo&, std::shared_ptr<const ti::JointPosition>, const std::string&>(),
             "info"_a, "position_var"_a, "name"_a = "CartPos",
             "Create Cartesian position constraint")
        .def("CalcValues", &ti::CartPosConstraint::CalcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("GetValues", &ti::CartPosConstraint::GetValues,
             "Get current constraint values")
        .def("SetTargetPose", &ti::CartPosConstraint::SetTargetPose, "target_frame_offset"_a,
             "Set the target pose - critical for online planning!")
        .def("GetTargetPose", &ti::CartPosConstraint::GetTargetPose,
             "Get the target pose for the constraint")
        .def("GetCurrentPose", &ti::CartPosConstraint::GetCurrentPose,
             "Get current TCP pose in world frame")
        .def_rw("use_numeric_differentiation", &ti::CartPosConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== JointPosConstraint ==========
    nb::class_<ti::JointPosConstraint, ifopt::ConstraintSet>(m, "JointPosConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::JointPosition>>&,
                      const Eigen::VectorXd&, const std::string&>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointPos",
             "Create joint position constraint with target values")
        .def("GetValues", &ti::JointPosConstraint::GetValues,
             "Get current constraint values");

    // ========== JointVelConstraint ==========
    nb::class_<ti::JointVelConstraint, ifopt::ConstraintSet>(m, "JointVelConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::JointPosition>>&,
                      const Eigen::VectorXd&, const std::string&>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointVel",
             "Create joint velocity constraint with target values")
        .def("GetValues", &ti::JointVelConstraint::GetValues,
             "Get current constraint values");

    // ========== JointAccelConstraint ==========
    nb::class_<ti::JointAccelConstraint, ifopt::ConstraintSet>(m, "JointAccelConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::JointPosition>>&,
                      const Eigen::VectorXd&, const std::string&>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointAccel",
             "Create joint acceleration constraint with target values")
        .def("GetValues", &ti::JointAccelConstraint::GetValues,
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
        .def_rw("collision_check_config", &tc::TrajOptCollisionConfig::collision_check_config)
        .def_rw("collision_coeff_data", &tc::TrajOptCollisionConfig::collision_coeff_data)
        .def_rw("collision_margin_buffer", &tc::TrajOptCollisionConfig::collision_margin_buffer)
        .def_rw("max_num_cnt", &tc::TrajOptCollisionConfig::max_num_cnt);

    // ========== DiscreteCollisionEvaluator (base class) ==========
    // Note: GetCollisionConfig was removed in 0.33, replaced with individual methods
    nb::class_<ti::DiscreteCollisionEvaluator>(m, "DiscreteCollisionEvaluator")
        .def("GetCollisionMarginBuffer", &ti::DiscreteCollisionEvaluator::GetCollisionMarginBuffer)
        .def("GetCollisionMarginData", &ti::DiscreteCollisionEvaluator::GetCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("GetCollisionCoeffData", &ti::DiscreteCollisionEvaluator::GetCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== SingleTimestepCollisionEvaluator ==========
    // Note: 0.33 changed from shared_ptr<TrajOptCollisionConfig> to const TrajOptCollisionConfig&
    nb::class_<ti::SingleTimestepCollisionEvaluator, ti::DiscreteCollisionEvaluator>(m, "SingleTimestepCollisionEvaluator")
        .def(nb::init<std::shared_ptr<ti::CollisionCache>,
                      std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "collision_cache"_a, "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== CollisionCache ==========
    nb::class_<ti::CollisionCache>(m, "CollisionCache")
        .def(nb::init<std::size_t>(), "size"_a = 10);

    // ========== DiscreteCollisionConstraint ==========
    nb::class_<ti::DiscreteCollisionConstraint, ifopt::ConstraintSet>(m, "DiscreteCollisionConstraint")
        .def(nb::init<std::shared_ptr<ti::DiscreteCollisionEvaluator>,
                      std::shared_ptr<const ti::JointPosition>,
                      int, bool, const std::string&>(),
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollisionV3")
        .def("GetValues", &ti::DiscreteCollisionConstraint::GetValues)
        .def("CalcValues", &ti::DiscreteCollisionConstraint::CalcValues, "joint_vals"_a)
        .def("GetCollisionEvaluator", &ti::DiscreteCollisionConstraint::GetCollisionEvaluator);

    // ========== ContinuousCollisionEvaluator (base class) ==========
    // Note: GetCollisionConfig was removed in 0.33, replaced with individual methods
    nb::class_<ti::ContinuousCollisionEvaluator>(m, "ContinuousCollisionEvaluator")
        .def("GetCollisionMarginBuffer", &ti::ContinuousCollisionEvaluator::GetCollisionMarginBuffer)
        .def("GetCollisionMarginData", &ti::ContinuousCollisionEvaluator::GetCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("GetCollisionCoeffData", &ti::ContinuousCollisionEvaluator::GetCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== LVSDiscreteCollisionEvaluator ==========
    // Uses discrete collision checking at interpolated points between two states
    // Note: 0.33 changed from shared_ptr<TrajOptCollisionConfig> to const TrajOptCollisionConfig&
    nb::class_<ti::LVSDiscreteCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSDiscreteCollisionEvaluator")
        .def(nb::init<std::shared_ptr<ti::CollisionCache>,
                      std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "collision_cache"_a, "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== LVSContinuousCollisionEvaluator ==========
    // Uses continuous (swept) collision checking between two states
    // Note: 0.33 changed from shared_ptr<TrajOptCollisionConfig> to const TrajOptCollisionConfig&
    nb::class_<ti::LVSContinuousCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSContinuousCollisionEvaluator")
        .def(nb::init<std::shared_ptr<ti::CollisionCache>,
                      std::shared_ptr<const tesseract_kinematics::JointGroup>,
                      std::shared_ptr<const tesseract_environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "collision_cache"_a, "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false);

    // ========== ContinuousCollisionConstraint ==========
    // Constraint for collision between two trajectory states
    // Note: 0.33 changed from array<bool,2> to two separate bool args
    nb::class_<ti::ContinuousCollisionConstraint, ifopt::ConstraintSet>(m, "ContinuousCollisionConstraint")
        .def("__init__", [](ti::ContinuousCollisionConstraint* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::JointPosition> var0,
                            std::shared_ptr<const ti::JointPosition> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::JointPosition>, 2> vars = {var0, var1};
                 new (self) ti::ContinuousCollisionConstraint(eval, vars, fixed0, fixed1, max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision")
        .def("GetValues", &ti::ContinuousCollisionConstraint::GetValues)
        .def("GetCollisionEvaluator", &ti::ContinuousCollisionConstraint::GetCollisionEvaluator);

    // ========== JointJerkConstraint ==========
    // Bounds on joint jerk (3rd derivative of position) for smoother trajectories
    nb::class_<ti::JointJerkConstraint, ifopt::ConstraintSet>(m, "JointJerkConstraint")
        .def(nb::init<const Eigen::VectorXd&, const std::vector<std::shared_ptr<const ti::JointPosition>>&,
                      const Eigen::VectorXd&, const std::string&>(),
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointJerk",
             "Create joint jerk constraint (requires 4+ consecutive waypoints)")
        .def("GetValues", &ti::JointJerkConstraint::GetValues,
             "Get current constraint values");

    // ========== CartLineInfo (struct) ==========
    // Info for constraining TCP to lie on a line segment between two poses
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
    // Constrain TCP to lie on a line segment (for approach/retract, linear paths)
    nb::class_<ti::CartLineConstraint, ifopt::ConstraintSet>(m, "CartLineConstraint")
        .def(nb::init<ti::CartLineInfo, std::shared_ptr<const ti::JointPosition>,
                      const Eigen::VectorXd&, const std::string&>(),
             "info"_a, "position_var"_a, "coeffs"_a, "name"_a = "CartLine",
             "Create Cartesian line constraint")
        .def("GetValues", &ti::CartLineConstraint::GetValues,
             "Get current constraint values")
        .def("CalcValues", &ti::CartLineConstraint::CalcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("GetLinePoint", &ti::CartLineConstraint::GetLinePoint,
             "source_tf"_a, "target_tf1"_a, "target_tf2"_a,
             "Find nearest point on line (uses SLERP for orientation)")
        .def_rw("use_numeric_differentiation", &ti::CartLineConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== DiscreteCollisionNumericalConstraint ==========
    // Same as DiscreteCollisionConstraint but uses numerical differentiation for jacobians
    nb::class_<ti::DiscreteCollisionNumericalConstraint, ifopt::ConstraintSet>(m, "DiscreteCollisionNumericalConstraint")
        .def(nb::init<std::shared_ptr<ti::DiscreteCollisionEvaluator>,
                      std::shared_ptr<const ti::JointPosition>,
                      int, bool, const std::string&>(),
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollisionNumerical",
             "Create discrete collision constraint with numerical jacobians")
        .def("GetValues", &ti::DiscreteCollisionNumericalConstraint::GetValues)
        .def("CalcValues", &ti::DiscreteCollisionNumericalConstraint::CalcValues, "joint_vals"_a)
        .def("GetCollisionEvaluator", &ti::DiscreteCollisionNumericalConstraint::GetCollisionEvaluator);

    // ========== InverseKinematicsInfo (struct) ==========
    // Info for IK-based optimization constraint
    nb::class_<ti::InverseKinematicsInfo>(m, "InverseKinematicsInfo")
        .def(nb::init<>())
        .def_rw("manip", &ti::InverseKinematicsInfo::manip, "The kinematic group (with IK solver)")
        .def_rw("working_frame", &ti::InverseKinematicsInfo::working_frame, "Working frame (not currently used)")
        .def_rw("tcp_frame", &ti::InverseKinematicsInfo::tcp_frame, "TCP frame (not currently used)")
        .def_rw("tcp_offset", &ti::InverseKinematicsInfo::tcp_offset, "TCP offset (not currently used)");

    // ========== InverseKinematicsConstraint ==========
    // Constrain joints to stay within bounds of an IK solution
    nb::class_<ti::InverseKinematicsConstraint, ifopt::ConstraintSet>(m, "InverseKinematicsConstraint")
        .def(nb::init<const Eigen::Isometry3d&, ti::InverseKinematicsInfo::ConstPtr,
                      std::shared_ptr<const ti::JointPosition>,
                      std::shared_ptr<const ti::JointPosition>,
                      const std::string&>(),
             "target_pose"_a, "kinematic_info"_a, "constraint_var"_a, "seed_var"_a,
             "name"_a = "InverseKinematics",
             "Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)")
        .def("GetValues", &ti::InverseKinematicsConstraint::GetValues,
             "Get current constraint values");
}
