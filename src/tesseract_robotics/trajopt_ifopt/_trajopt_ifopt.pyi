"""
trajopt_ifopt Python bindings - variables, constraints, and costs for IFOPT-based trajectory optimization
"""

from collections.abc import Callable, Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray
import scipy.sparse

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment
import tesseract_robotics.tesseract_kinematics._tesseract_kinematics


class ConstraintNotLinkedError(RuntimeError):
    """getJacobian() on a constraint set that is not linked to variables yet."""

class CartLineIndicesUnsupportedError(ValueError):
    """CartLineConstraint.getJacobian() with fewer than 6 indices (#237)."""

class BoundsSizeError(ValueError):
    """
    A bounds list whose length is not the number of rows or values it bounds.
    """

class JointValuesSizeError(ValueError):
    """joint_vals whose length is not the constraint's number of joints."""

class VarSizeMismatchError(ValueError):
    """toTrajArray over Vars of different sizes."""

class VarNotInVariableSetError(ValueError):
    """
    A constraint linked to a variable set that does not hold its Vars, or a collision constraint built on a Var that is not in a NodesVariables.
    """

class NodeOwnershipError(ValueError):
    """
    A Node handed to NodesVariables that is in a variable set already, or twice.
    """

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

    def __iadd__(self, scalar: float) -> Bounds:
        """Shift both limits up by scalar and reclassify the bounds type"""

    def __isub__(self, scalar: float) -> Bounds:
        """Shift both limits down by scalar and reclassify the bounds type"""

class Component:
    def getName(self) -> str:
        """Get the component name"""

    def getRows(self) -> int:
        """Get the number of rows"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current values (variables, or constraint / cost values)"""

    def getBounds(self) -> list[Bounds]:
        """Get the bounds, one per row"""

class Variables(Component):
    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> None:
        """Set the variable values from a flat decision vector"""

    def getHash(self) -> int:
        """Get the variable hash for caching"""

class Differentiable(Component):
    class Mode(enum.Enum):
        STACK_ROWS = 0
        """One row per value"""

        SUM_TO_SCALAR = 1
        """All values summed into one row"""

    def getJacobian(self) -> scipy.sparse.csr_matrix[float]:
        """
        Get the Jacobian (sparse matrix); a constraint set must be linked to variables first
        """

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
        """
        Get the linked optimization variables (protected in C++, exposed via trampoline)
        """

    def getJacobian(self) -> scipy.sparse.csr_matrix[float]:
        """Get the constraint Jacobian (sparse matrix)"""

    def update(self) -> int:
        """Recompute sizing/state for dynamic-sized components"""

    def getCoefficients(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the coefficient vector"""

class Var:
    @overload
    def __init__(self, index: int, name: str, value: float, bounds: Bounds = ...) -> None:
        """
        Create a standalone scalar variable at index of the decision vector; a value outside bounds is clamped into them
        """

    @overload
    def __init__(self, index: int, identifier: str, names: Sequence[str], values: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], bounds: Sequence[Bounds]) -> None:
        """
        Create a standalone vector variable starting at index of the decision vector; values outside bounds are clamped into them
        """

    def getIdentifier(self) -> str:
        """Get the identifier for the variable segment"""

    def getParent(self) -> Node | None:
        """The Node holding this variable, or None for a standalone Var"""

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

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None:
        """Set the variable values from the full decision vector"""

class Node:
    def __init__(self, node_name: str = 'Node') -> None:
        """Create a node with an optional name"""

    def getName(self) -> str:
        """Get the name of this node"""

    def getParent(self) -> NodesVariables | None:
        """
        The NodesVariables holding this node, or None before it is handed to one
        """

    @overload
    def addVar(self, name: str, value: float, bounds: Bounds = ...) -> Var:
        """Add a scalar variable to this node"""

    @overload
    def addVar(self, name: str, child_names: Sequence[str], values: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], bounds: Sequence[Bounds]) -> Var:
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

    def setVariables(self, x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> None:
        """Update all variable values from the full decision vector"""

class NodesVariables(Variables):
    def __init__(self, variable_name: str, nodes: Sequence[Node]) -> None:
        """
        Create the variable set from its nodes, in order; takes ownership of the nodes, which become unusable through the caller's references (read them back with getNode / getNodes). Raises NodeOwnershipError for a Node given twice or already in a set
        """

    def getNode(self, opt_idx: int) -> Node:
        """Get node based on index"""

    def getNodes(self) -> list[Node]:
        """Get all nodes"""

    def getDim(self) -> int:
        """Get the dimensions of every node"""

def createNodesVariables(variable_name: str, joint_names: Sequence[str], initial_values: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]], bounds: Sequence[Bounds]) -> NodesVariables:
    """
    Create NodesVariables with one vector Var per node.

    Args:
        variable_name: Name for the variable set
        joint_names: Joint name for each DOF
        initial_values: List of initial joint value vectors (one per waypoint)
        bounds: Joint bounds (shared across all waypoints)
    """

def interpolate(start: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], end: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], steps: int) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
    """Interpolate between two joint vectors, returning 'steps' waypoints"""

@overload
def toBounds(limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)] | Sequence[Sequence[float]]) -> list[Bounds]:
    """Convert Nx2 matrix [lower, upper] to vector of Bounds"""

@overload
def toBounds(lower_limits: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], upper_limits: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> list[Bounds]:
    """Convert lower/upper limit vectors to vector of Bounds"""

def getClosestValidPoint(input: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], bounds: Sequence[Bounds]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """input clamped into bounds, entry by entry"""

def calcBoundsErrors(input: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], bounds: Sequence[Bounds]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    Signed bound error per entry: input - lower below the lower bound (negative), input - upper above the upper bound (positive), else 0
    """

def calcBoundsViolations(input: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], bounds: Sequence[Bounds]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """Bound violation per entry: the magnitude of calcBoundsErrors"""

def calcNumericalConstraintGradient(variables: Variables, constraint_set: ConstraintSet, epsilon: float = 1e-08) -> scipy.sparse.csr_matrix[float]:
    """
    Forward-difference Jacobian of constraint_set's values over variables (sparse matrix). Perturbs each variable in turn through variables.setVariables, calling update() and getValues(), and restores the variables afterwards.
    """

def calcForwardNumJac(f: Callable[[Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]], x: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], epsilon: float) -> scipy.sparse.csr_matrix[float]:
    """
    Forward-difference Jacobian of f at x (sparse matrix, len(f(x)) x len(x))
    """

def toTrajArray(joint_positions: Sequence[Var]) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='C')]:
    """The Vars' values as a waypoints x joints matrix, one row per Var"""

def toJointTrajectory(joint_positions: Sequence[Var]) -> tesseract_robotics.tesseract_common._tesseract_common.JointTrajectory:
    """
    A JointTrajectory with one JointState (the Var's names and values) per Var
    """

class CartPosConstraint(ConstraintSet):
    @overload
    def __init__(self, position_var: Var, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, source_frame: str, target_frame: str, source_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, name: str = 'CartPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """Create Cartesian position constraint"""

    @overload
    def __init__(self, position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], bounds: Sequence[Bounds], manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, source_frame: str, target_frame: str, source_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, name: str = 'CartPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """
        Create Cartesian position constraint with per-axis coefficients and bounds. coeffs and bounds are both length 6, ordered [x, y, z, rx, ry, rz]; a zero coefficient frees that axis.
        """

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def calcJacobianBlock(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> scipy.sparse.csr_matrix[float]:
        """
        The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); needs linked variables
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def setTargetPose(self, target_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> None:
        """Set the target pose - critical for online planning!"""

    def getTargetPose(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """Get the target pose for the constraint"""

    def getCurrentPose(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """Get current TCP pose in world frame"""

    @property
    def use_numeric_differentiation(self) -> bool:
        """If true, use numeric differentiation (default: true)"""

    @use_numeric_differentiation.setter
    def use_numeric_differentiation(self, arg: bool, /) -> None: ...

class JointPosConstraint(ConstraintSet):
    @overload
    def __init__(self, target: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'JointPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """Create joint position constraint with target values"""

    @overload
    def __init__(self, bounds: Sequence[Bounds], position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'JointPos', range_bound_handling: RangeBoundHandling = RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES) -> None:
        """
        Create joint position constraint with one bound per joint: an equality, a one-sided limit or a range. coeffs has length n_dof, 1 (one weight for every joint) or 0 (weight 1 for every joint). A range becomes two one-sided rows unless range_bound_handling is KEEP_AS_IS.
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointVelConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'JointVel') -> None:
        """Create joint velocity constraint with target values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class JointAccelConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'JointAccel') -> None:
        """Create joint acceleration constraint with target values"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class CollisionCoeffData:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, default_collision_coeff: float) -> None: ...

    def __eq__(self, arg: CollisionCoeffData, /) -> bool: ...

    def __ne__(self, arg: CollisionCoeffData, /) -> bool: ...

    __hash__: None = None

    def setDefaultCollisionCoeff(self, arg: float, /) -> None: ...

    def getDefaultCollisionCoeff(self) -> float: ...

    def setCollisionCoeff(self, obj1: str, obj2: str, collision_coeff: float) -> None: ...

    def getCollisionCoeff(self, obj1: str, obj2: str) -> float: ...

    def getCollisionCoeffPairData(self) -> dict[tuple[str, str], float]:
        """Every pair with a coefficient: {(link_a, link_b): coeff}"""

    def getPairsWithZeroCoeff(self) -> set[tuple[str, str]]:
        """The pairs whose coefficient is 0 (collision checking switched off)"""

class TrajOptCollisionConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, margin: float, coeff: float) -> None: ...

    def __eq__(self, arg: TrajOptCollisionConfig, /) -> bool: ...

    def __ne__(self, arg: TrajOptCollisionConfig, /) -> bool: ...

    __hash__: None = None

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

class LinkGradientResults:
    def __init__(self) -> None: ...

    @property
    def has_gradient(self) -> bool: ...

    @has_gradient.setter
    def has_gradient(self, arg: bool, /) -> None: ...

    @property
    def gradient(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @gradient.setter
    def gradient(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def translation_vector(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @translation_vector.setter
    def translation_vector(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def jacobian(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]: ...

    @jacobian.setter
    def jacobian(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')] | Sequence[Sequence[float]], /) -> None: ...

    @property
    def scale(self) -> float: ...

    @scale.setter
    def scale(self, arg: float, /) -> None: ...

    @property
    def cc_type(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContinuousCollisionType: ...

    @cc_type.setter
    def cc_type(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContinuousCollisionType, /) -> None: ...

    def clear(self) -> None: ...

class GradientResults:
    def __init__(self) -> None: ...

    @property
    def gradients(self) -> list[LinkGradientResults]: ...

    @gradients.setter
    def gradients(self, arg: Sequence[LinkGradientResults], /) -> None: ...

    @property
    def cc_gradients(self) -> list[LinkGradientResults]: ...

    @cc_gradients.setter
    def cc_gradients(self, arg: Sequence[LinkGradientResults], /) -> None: ...

    @property
    def error(self) -> float: ...

    @error.setter
    def error(self, arg: float, /) -> None: ...

    @property
    def error_with_buffer(self) -> float: ...

    @error_with_buffer.setter
    def error_with_buffer(self, arg: float, /) -> None: ...

    def clear(self) -> None: ...

class LinkMaxError:
    def __init__(self) -> None: ...

    @property
    def has_error(self) -> list[bool]: ...

    @has_error.setter
    def has_error(self, arg: Sequence[bool], /) -> None: ...

    @property
    def error(self) -> list[float]: ...

    @error.setter
    def error(self, arg: Sequence[float], /) -> None: ...

    @property
    def error_with_buffer(self) -> list[float]: ...

    @error_with_buffer.setter
    def error_with_buffer(self, arg: Sequence[float], /) -> None: ...

    def getMaxError(self) -> float: ...

    def getMaxErrorWithBuffer(self) -> float: ...

class GradientResultsSet:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, reserve: int) -> None: ...

    @property
    def key(self) -> tuple[str, str]: ...

    @key.setter
    def key(self, arg: tuple[str, str], /) -> None: ...

    @property
    def shape_key(self) -> tuple[int, int]: ...

    @shape_key.setter
    def shape_key(self, arg: tuple[int, int], /) -> None: ...

    @property
    def coeff(self) -> float: ...

    @coeff.setter
    def coeff(self, arg: float, /) -> None: ...

    @property
    def is_continuous(self) -> bool: ...

    @is_continuous.setter
    def is_continuous(self, arg: bool, /) -> None: ...

    @property
    def max_error(self) -> list[LinkMaxError]: ...

    @max_error.setter
    def max_error(self, arg: Sequence[LinkMaxError], /) -> None: ...

    @property
    def results(self) -> list[GradientResults]: ...

    @results.setter
    def results(self, arg: Sequence[GradientResults], /) -> None: ...

    def add(self, gradient_result: GradientResults) -> None: ...

    def getMaxError(self) -> float: ...

    def getMaxErrorT0(self) -> float: ...

    def getMaxErrorT1(self) -> float: ...

    def getMaxErrorWithBuffer(self) -> float: ...

    def getMaxErrorWithBufferT0(self) -> float: ...

    def getMaxErrorWithBufferT1(self) -> float: ...

class CollisionCacheData:
    def __init__(self) -> None: ...

    @property
    def contact_results_map(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContactResultMap: ...

    @contact_results_map.setter
    def contact_results_map(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContactResultMap, /) -> None: ...

    @property
    def gradient_results_sets(self) -> list[GradientResultsSet]: ...

    @gradient_results_sets.setter
    def gradient_results_sets(self, arg: Sequence[GradientResultsSet], /) -> None: ...

class DiscreteCollisionEvaluator:
    def calcCollisions(self, dof_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], max_allowed: int) -> CollisionCacheData:
        """
        Check collisions at dof_vals, keeping at most max_allowed contacts per link pair; returns the contacts and their gradient result sets
        """

    def getGradient(self, dofvals: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], contact_result: tesseract_robotics.tesseract_collision._tesseract_collision.ContactResult) -> GradientResults:
        """Gradient of one contact at dofvals"""

    def getJointGroup(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup: ...

    def getCollisionMarginBuffer(self) -> float: ...

    def getCollisionMarginData(self) -> tesseract_robotics.tesseract_common._tesseract_common.CollisionMarginData: ...

    def getCollisionCoeffData(self) -> CollisionCoeffData: ...

class SingleTimestepCollisionEvaluator(DiscreteCollisionEvaluator):
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class DiscreteCollisionConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: Var, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'DiscreteCollision') -> None: ...

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class ContinuousCollisionEvaluator:
    def calcCollisionData(self, dof_vals0: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], dof_vals1: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], vars0_fixed: bool, vars1_fixed: bool, max_allowed: int) -> CollisionCacheData:
        """
        Check collisions along the segment dof_vals0 -> dof_vals1, keeping at most max_allowed contacts per link pair; returns the contacts and their gradient result sets
        """

    def calcGradientData(self, dof_vals0: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], dof_vals1: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float], contact_results: tesseract_robotics.tesseract_collision._tesseract_collision.ContactResult) -> GradientResults:
        """Gradient of one contact along the segment dof_vals0 -> dof_vals1"""

    def getJointGroup(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup: ...

    def getCollisionMarginBuffer(self) -> float: ...

    def getCollisionMarginData(self) -> tesseract_robotics.tesseract_common._tesseract_common.CollisionMarginData: ...

    def getCollisionCoeffData(self) -> CollisionCoeffData: ...

class LVSDiscreteCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class LVSContinuousCollisionEvaluator(ContinuousCollisionEvaluator):
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, env: tesseract_robotics.tesseract_environment._tesseract_environment.Environment, collision_config: TrajOptCollisionConfig, dynamic_environment: bool = False) -> None: ...

class ContinuousCollisionConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: ContinuousCollisionEvaluator, position_var0: Var, position_var1: Var, fixed0: bool = False, fixed1: bool = False, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'LVSCollision') -> None: ...

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getCollisionEvaluator(self) -> ContinuousCollisionEvaluator: ...

class DiscreteCollisionConstraintD(ConstraintSet):
    def __init__(self, collision_evaluator: DiscreteCollisionEvaluator, position_var: Var, name: str = 'DiscreteCollision') -> None:
        """
        Create a discrete collision constraint with one row per contact (rows change on update())
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class ContinuousCollisionConstraintD(ConstraintSet):
    def __init__(self, collision_evaluator: ContinuousCollisionEvaluator, position_var0: Var, position_var1: Var, fixed0: bool = False, fixed1: bool = False, name: str = 'LVSCollision') -> None:
        """
        Create a continuous collision constraint with one row per contact (rows change on update())
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getCollisionEvaluator(self) -> ContinuousCollisionEvaluator: ...

class ContinuousCollisionNumericalConstraint(ConstraintSet):
    def __init__(self, collision_evaluator: ContinuousCollisionEvaluator, position_var0: Var, position_var1: Var, fixed0: bool = False, fixed1: bool = False, max_num_cnt: int = 1, fixed_sparsity: bool = False, name: str = 'LVSCollision') -> None:
        """Create continuous collision constraint with numerical jacobians"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getCollisionEvaluator(self) -> ContinuousCollisionEvaluator: ...

class JointJerkConstraint(ConstraintSet):
    def __init__(self, targets: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], position_vars: Sequence[Var], coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'JointJerk') -> None:
        """Create joint jerk constraint (requires 4+ consecutive waypoints)"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

class CartLineInfo:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, source_frame: str, target_frame: str, target_frame_offset1: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_frame_offset2: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, source_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, indices: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')] | Sequence[int]) -> None:
        """
        Create the info of a line from target_frame_offset1 to target_frame_offset2
        """

    @overload
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, source_frame: str, target_frame: str, target_frame_offset1: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_frame_offset2: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, source_frame_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> None:
        """Create the info of a line, constraining all 6 indices"""

    @overload
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, source_frame: str, target_frame: str, target_frame_offset1: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_frame_offset2: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> None:
        """
        Create the info of a line, with an identity source_frame_offset and all 6 indices
        """

    @property
    def manip(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup:
        """The joint group"""

    @manip.setter
    def manip(self, arg: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.JointGroup, /) -> None: ...

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
    def source_frame_offset(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """TCP offset"""

    @source_frame_offset.setter
    def source_frame_offset(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def target_frame_offset1(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """Line start pose"""

    @target_frame_offset1.setter
    def target_frame_offset1(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def target_frame_offset2(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """Line end pose"""

    @target_frame_offset2.setter
    def target_frame_offset2(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

    @property
    def indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """DOF indices to constrain: default {0,1,2,3,4,5}"""

    @indices.setter
    def indices(self, arg: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')] | Sequence[int], /) -> None: ...

class CartLineConstraint(ConstraintSet):
    def __init__(self, info: CartLineInfo, position_var: Var, coeffs: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], name: str = 'CartLine') -> None:
        """Create Cartesian line constraint"""

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def getLine(self) -> tuple[tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, tesseract_robotics.tesseract_common._tesseract_common.Isometry3d]:
        """The line's two end poses (target_frame_offset1, target_frame_offset2)"""

    def getInfo(self) -> CartLineInfo:
        """The constraint's CartLineInfo"""

    def getJacobian(self) -> scipy.sparse.csr_matrix[float]:
        """
        Get the constraint Jacobian (sparse matrix); raises CartLineIndicesUnsupportedError unless all 6 indices are set (#237)
        """

    def calcJacobianBlock(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> scipy.sparse.csr_matrix[float]:
        """
        The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); needs linked variables and all 6 indices (#237)
        """

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Calculate error values for given joint values"""

    def getLinePoint(self, source_tf: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_tf1: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, target_tf2: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
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

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def calcValues(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    def calcJacobianBlock(self, joint_vals: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)] | Sequence[float]) -> scipy.sparse.csr_matrix[float]:
        """
        The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); needs linked variables and an update() since the last variable change
        """

    def getCollisionEvaluator(self) -> DiscreteCollisionEvaluator: ...

class InverseKinematicsInfo:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.KinematicGroup, working_frame: str, tcp_frame: str, tcp_offset: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> None:
        """Create IK info; raises if manip has no link tcp_frame"""

    @overload
    def __init__(self, manip: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.KinematicGroup, working_frame: str, tcp_frame: str) -> None:
        """
        Create IK info with an identity tcp_offset; raises if manip has no link tcp_frame
        """

    @property
    def manip(self) -> tesseract_robotics.tesseract_kinematics._tesseract_kinematics.KinematicGroup:
        """The kinematic group (with IK solver)"""

    @manip.setter
    def manip(self, arg: tesseract_robotics.tesseract_kinematics._tesseract_kinematics.KinematicGroup, /) -> None: ...

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
    def tcp_offset(self) -> tesseract_robotics.tesseract_common._tesseract_common.Isometry3d:
        """TCP offset (not currently used)"""

    @tcp_offset.setter
    def tcp_offset(self, arg: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, /) -> None: ...

class InverseKinematicsConstraint(ConstraintSet):
    def __init__(self, target_pose: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d, kinematic_info: InverseKinematicsInfo, constraint_var: Var, seed_var: Var, name: str = 'InverseKinematics') -> None:
        """
        Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)
        """

    def getValues(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get current constraint values"""

    def setBounds(self, bounds: Sequence[Bounds]) -> None:
        """Set the bounds, one per row; raises BoundsSizeError otherwise"""

    def setTargetPose(self, target_pose: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d) -> None:
        """Set the target pose the IK solution is computed for"""

    def getKinematicInfo(self) -> InverseKinematicsInfo:
        """The constraint's InverseKinematicsInfo"""
