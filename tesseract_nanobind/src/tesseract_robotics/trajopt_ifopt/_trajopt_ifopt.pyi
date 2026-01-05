from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.ifopt._ifopt


class JointPosition(tesseract_robotics.ifopt._ifopt.VariableSet):
    def __init__(self, init_value: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], joint_names: Sequence[str], name: str = 'Joint_Position') -> None:
        """Create joint position variable set with initial values and joint names"""

    def SetVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the joint position values"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current joint position values"""

    def GetJointNames(self) -> list[str]:
        """Get the joint names associated with this variable set"""

    def GetRows(self) -> int:
        """Get number of variables"""

    def GetName(self) -> str:
        """Get the name of this variable set"""

    def SetBounds(self, bounds: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)]) -> None:
        """Set joint bounds from Nx2 matrix [lower, upper]"""

    def GetBounds(self) -> list[tesseract_robotics.ifopt._ifopt.Bounds]:
        """Get the variable bounds"""

def interpolate(start: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], end: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], steps: int) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
    """Interpolate between two joint vectors, returning 'steps' waypoints"""

def toBounds(limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)]) -> list[tesseract_robotics.ifopt._ifopt.Bounds]:
    """Convert Nx2 matrix [lower, upper] to vector of ifopt::Bounds"""

class CartPosInfoType(enum.Enum):
    TARGET_ACTIVE = 0

    SOURCE_ACTIVE = 1

    BOTH_ACTIVE = 2

class CartPosInfo:
    def __init__(self) -> None: ...

    @property
    def manip(self) -> "tesseract_kinematics::JointGroup":
        """The joint group"""

    @manip.setter
    def manip(self, arg: "tesseract_kinematics::JointGroup", /) -> None: ...

    @property
    def source_frame(self) -> str:
        """Link which should reach desired pos"""

    @source_frame.setter
    def source_frame(self, arg: str, /) -> None: ...

    @property
    def target_frame(self) -> str:
        """Target frame to be reached"""

    @target_frame.setter
    def target_frame(self, arg: str, /) -> None: ...

    @property
    def source_frame_offset(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Static transform applied to source_frame"""

    @source_frame_offset.setter
    def source_frame_offset(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

    @property
    def target_frame_offset(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Static transform applied to target_frame"""

    @target_frame_offset.setter
    def target_frame_offset(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

    @property
    def type(self) -> CartPosInfoType:
        """Indicates which link is active"""

    @type.setter
    def type(self, arg: CartPosInfoType, /) -> None: ...

    @property
    def indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """
        Indices to return: default {0,1,2,3,4,5}. Position: {0,1,2}, Rotation: {3,4,5}
        """

    @indices.setter
    def indices(self, arg: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], /) -> None: ...

class CartPosConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, info: CartPosInfo, position_var: JointPosition, name: str = 'CartPos') -> None:
        """Create Cartesian position constraint"""

    def CalcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def SetTargetPose(self, target_frame_offset: "Eigen::Transform<double, 3, 1, 0>") -> None:
        """Set the target pose - critical for online planning!"""

    def GetTargetPose(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Get the target pose for the constraint"""

    def GetCurrentPose(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Get current TCP pose in world frame"""

    @property
    def use_numeric_differentiation(self) -> bool:
        """If true, use numeric differentiation (default: true)"""

    @use_numeric_differentiation.setter
    def use_numeric_differentiation(self, arg: bool, /) -> None: ...

class JointPosConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[JointPosition], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointPos') -> None:
        """Create joint position constraint with target values"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointVelConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[JointPosition], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointVel') -> None:
        """Create joint velocity constraint with target values"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointAccelConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[JointPosition], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointAccel') -> None:
        """Create joint acceleration constraint with target values"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class CollisionCoeffData:
    def __init__(self, default_collision_coeff: float = 1.0) -> None: ...

    def setPairCollisionCoeff(self, obj1: str, obj2: str, collision_coeff: float) -> None: ...

    def getPairCollisionCoeff(self, obj1: str, obj2: str) -> float: ...

class TrajOptCollisionConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, margin: float, coeff: float) -> None: ...

    @property
    def collision_coeff_data(self) -> CollisionCoeffData: ...

    @collision_coeff_data.setter
    def collision_coeff_data(self, arg: CollisionCoeffData, /) -> None: ...

    @property
    def collision_margin_buffer(self) -> float: ...

    @collision_margin_buffer.setter
    def collision_margin_buffer(self, arg: float, /) -> None: ...

    @property
    def max_num_cnt(self) -> int: ...

    @max_num_cnt.setter
    def max_num_cnt(self, arg: int, /) -> None: ...

class DiscreteCollisionEvaluator:
    def GetCollisionConfig(self) -> TrajOptCollisionConfig: ...

class SingleTimestepCollisionEvaluator(DiscreteCollisionEvaluator):
    def __init__(self, collision_cache: CollisionCache, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class CollisionCache:
    def __init__(self, size: int = 10) -> None: ...

class DiscreteCollisionConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: JointPosition, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'DiscreteCollisionV3') -> None: ...

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def CalcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def GetCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class ContinuousCollisionEvaluator:
    def GetCollisionConfig(self) -> TrajOptCollisionConfig: ...

class LVSDiscreteCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, collision_cache: CollisionCache, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class LVSContinuousCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, collision_cache: CollisionCache, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class ContinuousCollisionConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, collision_evaluator: ContinuousCollisionEvaluator, position_var0: JointPosition, position_var1: JointPosition, fixed0: bool = False, fixed1: bool = False, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'LVSCollision') -> None: ...

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def GetCollisionEvaluator(self) -> ContinuousCollisionEvaluator: ...

class JointJerkConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[JointPosition], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointJerk') -> None:
        """Create joint jerk constraint (requires 4+ consecutive waypoints)"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class CartLineInfo:
    def __init__(self) -> None: ...

    @property
    def manip(self) -> "tesseract_kinematics::JointGroup":
        """The joint group"""

    @manip.setter
    def manip(self, arg: "tesseract_kinematics::JointGroup", /) -> None: ...

    @property
    def source_frame(self) -> str:
        """TCP frame"""

    @source_frame.setter
    def source_frame(self, arg: str, /) -> None: ...

    @property
    def target_frame(self) -> str:
        """Reference frame"""

    @target_frame.setter
    def target_frame(self, arg: str, /) -> None: ...

    @property
    def source_frame_offset(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """TCP offset"""

    @source_frame_offset.setter
    def source_frame_offset(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

    @property
    def target_frame_offset1(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Line start pose"""

    @target_frame_offset1.setter
    def target_frame_offset1(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

    @property
    def target_frame_offset2(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Line end pose"""

    @target_frame_offset2.setter
    def target_frame_offset2(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

    @property
    def indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """DOF indices to constrain: default {0,1,2,3,4,5}"""

    @indices.setter
    def indices(self, arg: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], /) -> None: ...

class CartLineConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, info: CartLineInfo, position_var: JointPosition, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'CartLine') -> None:
        """Create Cartesian line constraint"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def CalcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def GetLinePoint(self, source_tf: "Eigen::Transform<double, 3, 1, 0>", target_tf1: "Eigen::Transform<double, 3, 1, 0>", target_tf2: "Eigen::Transform<double, 3, 1, 0>") -> "Eigen::Transform<double, 3, 1, 0>":
        """Find nearest point on line (uses SLERP for orientation)"""

    @property
    def use_numeric_differentiation(self) -> bool:
        """If true, use numeric differentiation (default: true)"""

    @use_numeric_differentiation.setter
    def use_numeric_differentiation(self, arg: bool, /) -> None: ...

class DiscreteCollisionNumericalConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: JointPosition, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'DiscreteCollisionNumerical') -> None:
        """Create discrete collision constraint with numerical jacobians"""

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def CalcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def GetCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class InverseKinematicsInfo:
    def __init__(self) -> None: ...

    @property
    def manip(self) -> "tesseract_kinematics::KinematicGroup":
        """The kinematic group (with IK solver)"""

    @manip.setter
    def manip(self, arg: "tesseract_kinematics::KinematicGroup", /) -> None: ...

    @property
    def working_frame(self) -> str:
        """Working frame (not currently used)"""

    @working_frame.setter
    def working_frame(self, arg: str, /) -> None: ...

    @property
    def tcp_frame(self) -> str:
        """TCP frame (not currently used)"""

    @tcp_frame.setter
    def tcp_frame(self, arg: str, /) -> None: ...

    @property
    def tcp_offset(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """TCP offset (not currently used)"""

    @tcp_offset.setter
    def tcp_offset(self, arg: "Eigen::Transform<double, 3, 1, 0>", /) -> None: ...

class InverseKinematicsConstraint(tesseract_robotics.ifopt._ifopt.ConstraintSet):
    def __init__(self, target_pose: "Eigen::Transform<double, 3, 1, 0>", kinematic_info: InverseKinematicsInfo, constraint_var: JointPosition, seed_var: JointPosition, name: str = 'InverseKinematics') -> None:
        """
        Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)
        """

    def GetValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""
