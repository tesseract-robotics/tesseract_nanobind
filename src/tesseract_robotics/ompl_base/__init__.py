"""OMPL base state space bindings for car-like vehicle planning.

Exposes SE(2) state spaces with specialized distance metrics:
- SE2StateSpace: Basic (x, y, yaw) state space
- ReedsSheppStateSpace: For vehicles that can go forward AND backward
- DubinsStateSpace: For vehicles that can only go forward

Example:
    >>> from tesseract_robotics.ompl_base import ReedsSheppStateSpace, RealVectorBounds
    >>> import math
    >>>
    >>> # Create state space with 1.5m turning radius
    >>> rs = ReedsSheppStateSpace(turningRadius=1.5)
    >>>
    >>> # Set workspace bounds
    >>> bounds = RealVectorBounds(2)
    >>> bounds.setLow(-10.0)
    >>> bounds.setHigh(10.0)
    >>> rs.setBounds(bounds)
    >>>
    >>> # Create start and goal states
    >>> start = rs.allocState()
    >>> goal = rs.allocState()
    >>> rs.getStateAs(start).setXY(0, 0)
    >>> rs.getStateAs(start).setYaw(0)
    >>> rs.getStateAs(goal).setXY(5, 3)
    >>> rs.getStateAs(goal).setYaw(math.pi / 2)
    >>>
    >>> # Compute optimal Reeds-Shepp path
    >>> path = rs.reedsShepp(start, goal)
    >>> print(f"Path length: {path.length():.3f}")
    >>>
    >>> # Cleanup
    >>> rs.freeState(start)
    >>> rs.freeState(goal)
"""

from ._ompl_base import (
    DubinsPath,
    DubinsPathSegmentType,
    # Dubins (forward only)
    DubinsStateSpace,
    # Bounds
    RealVectorBounds,
    ReedsSheppPath,
    ReedsSheppPathSegmentType,
    # Reeds-Shepp (forward + backward)
    ReedsSheppStateSpace,
    SE2State,
    # SE2 base
    SE2StateSpace,
    # Opaque state handle
    State,
)

__all__ = [
    "State",
    "RealVectorBounds",
    "SE2StateSpace",
    "SE2State",
    "ReedsSheppStateSpace",
    "ReedsSheppPath",
    "ReedsSheppPathSegmentType",
    "DubinsStateSpace",
    "DubinsPath",
    "DubinsPathSegmentType",
]
