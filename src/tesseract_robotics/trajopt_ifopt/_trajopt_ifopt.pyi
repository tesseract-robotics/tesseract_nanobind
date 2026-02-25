from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_collision._tesseract_collision


class BoundsType(enum.Enum):
    UNBOUNDED = 0

    EQUALITY = 1

    LOWER_BOUND = 2

    UPPER_BOUND = 3

    RANGE_BOUND = 4

class RangeBoundHandling(enum.Enum):
    KEEP_AS_IS = 0

    SPLIT_TO_TWO_INEQUALITIES = 1

class Bounds:
    def __init__(self, lower: float = 0.0, upper: float = 0.0) -> None:
        """Create bounds with lower and upper limits"""

    def set(self, lower: float, upper: float) -> None:
        """Set both lower and upper bounds"""

    def setLower(self, lower: float) -> None:
        """Set the lower bound"""

    def getLower(self) -> float:
        """Get the lower bound value"""

    def setUpper(self, upper: float) -> None:
        """Set the upper bound"""

    def getUpper(self) -> float:
        """Get the upper bound value"""

    def getType(self) -> BoundsType:
        """Get the cached bound classification"""

class Component:
    def getName(self) -> str:
        """Get the component name"""

    def getRows(self) -> int:
        """Get the number of rows"""

class Variables(Component):
    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current variable values"""

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the variable values from a flat decision vector"""

    def getBounds(self) -> list[Bounds]:
        """Get the variable bounds"""

    def getHash(self) -> int:
        """Get the variable hash for caching"""

class Differentiable(Component):
    def isDynamic(self) -> bool:
        """Whether this component can change its number of rows"""

    def isScalar(self) -> bool:
        """Whether this component is scalar or stacked"""

    def getNonZeros(self) -> int:
        """Estimated number of non-zero entries in the Jacobian"""

    def update(self) -> int:
        """Recompute sizing/state for dynamic-sized components"""

    def getCoefficients(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the coefficient vector"""

class ConstraintSet(Differentiable):
    def __init__(self, name: str, n_constraints: int) -> None:
        """Create a constraint set with name and number of constraints"""

    def linkWithVariables(self, x: Variables) -> None:
        """Connect the constraint with the optimization variables"""

    def getVariables(self) -> Variables:
        """Get the linked optimization variables (protected in C++, exposed via trampoline)"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current constraint values"""

    def getBounds(self) -> list[Bounds]:
        """Get the constraint bounds"""

    def getJacobian(self) -> "scipy.sparse.csr_matrix":
        """Get the constraint Jacobian (sparse matrix)"""

class Var:
    def getIdentifier(self) -> str:
        """Get the identifier for the variable segment"""

    def getIndex(self) -> int:
        """Get the starting index of this variable segment"""

    def size(self) -> int:
        """Get the number of elements in this variable segment"""

    def value(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current variable values"""

    def name(self) -> list[str]:
        """Get the variable names"""

    def bounds(self) -> list[Bounds]:
        """Get the variable bounds"""

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> None:
        """Set the variable values from the full decision vector"""

class Node:
    def __init__(self, node_name: str = 'Node') -> None:
        """Create a node with an optional name"""

    def getName(self) -> str:
        """Get the name of this node"""

    @overload
    def addVar(self, name: str, value: float, bounds: Bounds = ...) -> Var:
        """Add a scalar variable to this node"""

    @overload
    def addVar(self, name: str, child_names: Sequence[str], values: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], bounds: Sequence[Bounds]) -> Var:
        """Add a vector-valued variable to this node"""

    def hasVar(self, name: str) -> bool:
        """Check whether this node has a variable by name"""

    def getVar(self, name: str) -> Var:
        """Get a variable by name"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get all variable values as a single vector"""

    def getBounds(self) -> list[Bounds]:
        """Get all variable bounds"""

    def size(self) -> int:
        """Get total number of scalar decision variables"""

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> None:
        """Update all variable values from the full decision vector"""

class NodesVariables(Variables):
    def getNode(self, opt_idx: int) -> Node:
        """Get node based on index"""

    def getNodes(self) -> list[Node]:
        """Get all nodes"""

    def getDim(self) -> int:
        """Get the dimensions of every node"""

def createNodesVariables(variable_name: str, joint_names: Sequence[str], initial_values: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], bounds: Sequence[Bounds]) -> NodesVariables:
    """
    Create NodesVariables with one vector Var per node.

    Args:
        variable_name: Name for the variable set
        joint_names: Joint name for each DOF
        initial_values: List of initial joint value vectors (one per waypoint)
        bounds: Joint bounds (shared across all waypoints)
    """

def interpolate(start: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], end: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], steps: int) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
    """Interpolate between two joint vectors, returning 'steps' waypoints"""

@overload
def toBounds(limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)]) -> list[Bounds]:
    """Convert Nx2 matrix [lower, upper] to vector of Bounds"""

@overload
def toBounds(lower_limits: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)], upper_limits: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> list[Bounds]:
    """Convert lower/upper limit vectors to vector of Bounds"""

class CartPosConstraint(ConstraintSet):
    def __init__(self, position_var: Var, manip: "tesseract_kinematics::JointGroup", source_frame: str, target_frame: str, source_frame_offset: "Eigen::Transform<double, 3, 1, 0>", target_frame_offset: "Eigen::Transform<double, 3, 1, 0>", name: str = 'CartPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """Create Cartesian position constraint"""

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def setTargetPose(self, target_frame_offset: "Eigen::Transform<double, 3, 1, 0>") -> None:
        """Set the target pose - critical for online planning!"""

    def getTargetPose(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Get the target pose for the constraint"""

    def getCurrentPose(self) -> "Eigen::Transform<double, 3, 1, 0>":
        """Get current TCP pose in world frame"""

    @property
    def use_numeric_differentiation(self) -> bool:
        """If true, use numeric differentiation (default: true)"""

    @use_numeric_differentiation.setter
    def use_numeric_differentiation(self, arg: bool, /) -> None: ...

class JointPosConstraint(ConstraintSet):
    def __init__(self, target: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """Create joint position constraint with target values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointVelConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointVel') -> None:
        """Create joint velocity constraint with target values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointAccelConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointAccel') -> None:
        """Create joint acceleration constraint with target values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class CollisionCoeffData:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, default_collision_coeff: float) -> None: ...

    def setDefaultCollisionCoeff(self, arg: float, /) -> None: ...

    def getDefaultCollisionCoeff(self) -> float: ...

    def setCollisionCoeff(self, obj1: str, obj2: str, collision_coeff: float) -> None: ...

    def getCollisionCoeff(self, obj1: str, obj2: str) -> float: ...

    def setPairCollisionCoeff(self, obj1: str, obj2: str, collision_coeff: float) -> None: ...

    def getPairCollisionCoeff(self, obj1: str, obj2: str) -> float: ...

class TrajOptCollisionConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, margin: float, coeff: float) -> None: ...

    @property
    def enabled(self) -> bool: ...

    @enabled.setter
    def enabled(self, arg: bool, /) -> None: ...

    @property
    def contact_manager_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig: ...

    @contact_manager_config.setter
    def contact_manager_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig, /) -> None: ...

    @property
    def collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig: ...

    @collision_check_config.setter
    def collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

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
    def getCollisionMarginBuffer(self) -> float: ...

    def getCollisionMarginData(self) -> "tesseract_common::CollisionMarginData": ...

    def getCollisionCoeffData(self) -> CollisionCoeffData: ...

class SingleTimestepCollisionEvaluator(DiscreteCollisionEvaluator):
    def __init__(self, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class DiscreteCollisionConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: Var, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'DiscreteCollision') -> None: ...

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def getCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class ContinuousCollisionEvaluator:
    def getCollisionMarginBuffer(self) -> float: ...

    def getCollisionMarginData(self) -> "tesseract_common::CollisionMarginData": ...

    def getCollisionCoeffData(self) -> CollisionCoeffData: ...

class LVSDiscreteCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class LVSContinuousCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, manip: "tesseract_kinematics::JointGroup", env: "tesseract_environment::Environment", collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class ContinuousCollisionConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: ContinuousCollisionEvaluator, position_var0: Var, position_var1: Var, fixed0: bool = False, fixed1: bool = False, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'LVSCollision') -> None: ...

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def getCollisionEvaluator(self) -> ContinuousCollisionEvaluator: ...

class JointJerkConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'JointJerk') -> None:
        """Create joint jerk constraint (requires 4+ consecutive waypoints)"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
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

class CartLineConstraint(ConstraintSet):
    def __init__(self, info: CartLineInfo, position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], name: str = 'CartLine') -> None:
        """Create Cartesian line constraint"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def getLinePoint(self, source_tf: "Eigen::Transform<double, 3, 1, 0>", target_tf1: "Eigen::Transform<double, 3, 1, 0>", target_tf2: "Eigen::Transform<double, 3, 1, 0>") -> "Eigen::Transform<double, 3, 1, 0>":
        """Find nearest point on line (uses SLERP for orientation)"""

    @property
    def use_numeric_differentiation(self) -> bool:
        """If true, use numeric differentiation (default: true)"""

    @use_numeric_differentiation.setter
    def use_numeric_differentiation(self, arg: bool, /) -> None: ...

class DiscreteCollisionNumericalConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: Var, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'DiscreteCollisionNumerical') -> None:
        """Create discrete collision constraint with numerical jacobians"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def getCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

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

class InverseKinematicsConstraint(ConstraintSet):
    def __init__(self, target_pose: "Eigen::Transform<double, 3, 1, 0>", kinematic_info: InverseKinematicsInfo, constraint_var: Var, seed_var: Var, name: str = 'InverseKinematics') -> None:
        """
        Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""
