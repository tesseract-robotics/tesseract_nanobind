"""Type stubs for OMPL base state space bindings."""

from enum import IntEnum
from typing import List

class RealVectorBounds:
    """Bounds for real vector state spaces (used for SE2 x,y bounds)."""

    low: List[float]
    high: List[float]

    def __init__(self, dim: int) -> None:
        """Create bounds for n dimensions."""
        ...

    def setLow(self, value: float) -> None:
        """Set lower bound for all dimensions."""
        ...

    def setLow(self, index: int, value: float) -> None:
        """Set lower bound for specific dimension."""
        ...

    def setHigh(self, value: float) -> None:
        """Set upper bound for all dimensions."""
        ...

    def setHigh(self, index: int, value: float) -> None:
        """Set upper bound for specific dimension."""
        ...

    def getVolume(self) -> float:
        """Compute volume of bounded region."""
        ...

    def getDifference(self) -> List[float]:
        """Get high[i] - low[i] for each dimension."""
        ...

class SE2State:
    """SE(2) state: (x, y, yaw)."""

    def getX(self) -> float:
        """Get X coordinate."""
        ...

    def getY(self) -> float:
        """Get Y coordinate."""
        ...

    def getYaw(self) -> float:
        """Get yaw angle (rotation about Z)."""
        ...

    def setX(self, x: float) -> None:
        """Set X coordinate."""
        ...

    def setY(self, y: float) -> None:
        """Set Y coordinate."""
        ...

    def setXY(self, x: float, y: float) -> None:
        """Set X and Y coordinates."""
        ...

    def setYaw(self, yaw: float) -> None:
        """Set yaw angle."""
        ...

class _State:
    """Opaque OMPL state pointer (use getStateAs to access)."""
    ...

class SE2StateSpace:
    """SE(2) state space: position (x,y) and orientation (yaw)."""

    def __init__(self) -> None: ...

    def setBounds(self, bounds: RealVectorBounds) -> None:
        """Set bounds for x,y (2D RealVectorBounds)."""
        ...

    def getBounds(self) -> RealVectorBounds:
        """Get current x,y bounds."""
        ...

    def allocState(self) -> _State:
        """Allocate a new state (must call freeState when done)."""
        ...

    def freeState(self, state: _State) -> None:
        """Free a previously allocated state."""
        ...

    def distance(self, state1: _State, state2: _State) -> float:
        """Compute distance between two states."""
        ...

    def interpolate(
        self, from_state: _State, to_state: _State, t: float, state: _State
    ) -> None:
        """Interpolate: state = from + t*(to-from), t in [0,1]."""
        ...

    def getStateAs(self, state: _State) -> SE2State:
        """Cast state to SE2State for accessing x,y,yaw."""
        ...

class ReedsSheppPathSegmentType(IntEnum):
    """Segment types in a Reeds-Shepp path."""

    RS_NOP = 0
    RS_LEFT = 1
    RS_STRAIGHT = 2
    RS_RIGHT = 3

class ReedsSheppPath:
    """Complete description of a Reeds-Shepp path (up to 5 segments)."""

    @property
    def totalLength(self) -> float:
        """Total path length."""
        ...

    @property
    def lengths(self) -> List[float]:
        """Length of each segment (5 values, unused segments are 0)."""
        ...

    @property
    def types(self) -> List[int]:
        """Type of each segment as int (0=NOP, 1=LEFT, 2=STRAIGHT, 3=RIGHT)."""
        ...

    def length(self) -> float:
        """Total path length."""
        ...

class ReedsSheppStateSpace(SE2StateSpace):
    """SE(2) state space with Reeds-Shepp distance metric.

    For car-like vehicles that can move forward AND backward.
    """

    def __init__(self, turningRadius: float = 1.0) -> None:
        """Create with specified minimum turning radius."""
        ...

    def reedsShepp(self, state1: _State, state2: _State) -> ReedsSheppPath:
        """Compute optimal Reeds-Shepp path between two states."""
        ...

class DubinsPathSegmentType(IntEnum):
    """Segment types in a Dubins path."""

    DUBINS_LEFT = 0
    DUBINS_STRAIGHT = 1
    DUBINS_RIGHT = 2

class DubinsPath:
    """Complete description of a Dubins path (exactly 3 segments)."""

    @property
    def lengths(self) -> List[float]:
        """Length of each of the 3 segments."""
        ...

    @property
    def reverse(self) -> bool:
        """Whether path should be followed in reverse (for symmetric distance)."""
        ...

    def length(self) -> float:
        """Total path length."""
        ...

class DubinsStateSpace(SE2StateSpace):
    """SE(2) state space with Dubins distance metric.

    For car-like vehicles that can only move FORWARD (no reverse).
    Note: Dubins distance is NOT a proper metric (triangle inequality can fail).
    """

    def __init__(self, turningRadius: float = 1.0, isSymmetric: bool = False) -> None:
        """Create with turning radius.

        isSymmetric=true makes d(a,b)=d(b,a) but then triangle inequality may not hold.
        """
        ...

    def dubins(self, state1: _State, state2: _State) -> DubinsPath:
        """Compute optimal Dubins path between two states."""
        ...

    def isMetricSpace(self) -> bool:
        """Returns False (Dubins distance is not a proper metric)."""
        ...
