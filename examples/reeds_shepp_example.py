"""
Reeds-Shepp Path Planning Example

Demonstrates car-like vehicle path planning using OMPL's Reeds-Shepp state space:
- Compute optimal paths for vehicles that can go forward AND backward
- Visualize path segments (Left turns, Right turns, Straight)
- Compare with Dubins paths (forward-only)

Reeds-Shepp paths are optimal for car-like robots with a minimum turning radius
that can drive both forward and backward (like parallel parking).

Pipeline Overview:
1. Create Reeds-Shepp state space with minimum turning radius
2. Set workspace bounds
3. Allocate start and goal states
4. Compute optimal path and interpolate waypoints
5. Compare with Dubins (forward-only) path

Key Concepts:
- SE(2): Position (x,y) + orientation (yaw) - the configuration space
- Turning Radius: Minimum radius the vehicle can turn (constraint)
- Reeds-Shepp: Optimal paths using arcs and straights, forward/backward
- Dubins: Similar but forward-only (longer when reverse would help)

Reference:
    J.A. Reeds and L.A. Shepp, "Optimal paths for a car that goes both
    forwards and backwards," Pacific Journal of Mathematics, 145(2):367-393, 1990.
"""

import math

from tesseract_robotics.ompl_base import (
    DubinsStateSpace,
    RealVectorBounds,
    ReedsSheppPathSegmentType,
    ReedsSheppStateSpace,
)

# Segment type names for pretty printing
RS_SEGMENT_NAMES = {
    ReedsSheppPathSegmentType.RS_NOP: "NOP",
    ReedsSheppPathSegmentType.RS_LEFT: "LEFT",
    ReedsSheppPathSegmentType.RS_STRAIGHT: "STRAIGHT",
    ReedsSheppPathSegmentType.RS_RIGHT: "RIGHT",
}


def format_path(path):
    """Format a Reeds-Shepp path as human-readable string."""
    segments = []
    for i, (seg_type, length) in enumerate(zip(path.types, path.lengths)):
        if seg_type != 0:  # Skip NOP
            name = RS_SEGMENT_NAMES.get(seg_type, f"TYPE_{seg_type}")
            # Negative length means reverse
            direction = "REV" if length < 0 else "FWD"
            segments.append(f"{name}({abs(length):.2f},{direction})")
    return " -> ".join(segments) if segments else "NONE"


def run(pipeline=None, num_planners=None):
    """Run Reeds-Shepp path planning example.

    Args:
        pipeline: Unused (for API compatibility)
        num_planners: Unused (for API compatibility)

    Returns:
        True on success
    """
    print("=" * 60)
    print("Reeds-Shepp Path Planning Example")
    print("=" * 60)

    # Create Reeds-Shepp state space with 1.5 meter turning radius
    turning_radius = 1.5
    rs = ReedsSheppStateSpace(turningRadius=turning_radius)
    print(f"\nTurning radius: {turning_radius} m")

    # Set workspace bounds (x: -20 to 20, y: -20 to 20)
    bounds = RealVectorBounds(2)
    bounds.setLow(-20.0)
    bounds.setHigh(20.0)
    rs.setBounds(bounds)

    # Allocate states
    start = rs.allocState()
    goal = rs.allocState()
    interp = rs.allocState()

    # ========== Example 1: Simple parallel parking ==========
    print("\n--- Example 1: Parallel Parking ---")
    print("Start: (0, 0) facing +X")
    print("Goal:  (-3, 0) facing +X (back into spot)")

    rs.getStateAs(start).setXY(0, 0)
    rs.getStateAs(start).setYaw(0)
    rs.getStateAs(goal).setXY(-3, 0)
    rs.getStateAs(goal).setYaw(0)

    path = rs.reedsShepp(start, goal)
    print(f"Path length: {path.length():.3f} m")
    print(f"Segments: {format_path(path)}")

    # Reeds-Shepp can just reverse straight back
    print("(Optimal: straight reverse = 3.0 m)")

    # ========== Example 2: 90 degree turn ==========
    print("\n--- Example 2: 90 Degree Turn ---")
    print("Start: (0, 0) facing +X (East)")
    print("Goal:  (2, 2) facing +Y (North)")

    rs.getStateAs(start).setXY(0, 0)
    rs.getStateAs(start).setYaw(0)
    rs.getStateAs(goal).setXY(2, 2)
    rs.getStateAs(goal).setYaw(math.pi / 2)

    path = rs.reedsShepp(start, goal)
    print(f"Path length: {path.length():.3f} m")
    print(f"Segments: {format_path(path)}")

    # Interpolate and show waypoints
    print("\nWaypoints along path:")
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        rs.interpolate(start, goal, t, interp)
        st = rs.getStateAs(interp)
        yaw_deg = math.degrees(st.getYaw())
        print(f"  t={t:.2f}: ({st.getX():6.2f}, {st.getY():6.2f}) yaw={yaw_deg:6.1f}°")

    # ========== Example 3: U-turn ==========
    print("\n--- Example 3: U-Turn ---")
    print("Start: (0, 0) facing +X")
    print("Goal:  (0, 3) facing -X (opposite direction)")

    rs.getStateAs(start).setXY(0, 0)
    rs.getStateAs(start).setYaw(0)
    rs.getStateAs(goal).setXY(0, 3)
    rs.getStateAs(goal).setYaw(math.pi)

    path = rs.reedsShepp(start, goal)
    print(f"Path length: {path.length():.3f} m")
    print(f"Segments: {format_path(path)}")

    # ========== Compare with Dubins ==========
    print("\n" + "=" * 60)
    print("Comparing Reeds-Shepp vs Dubins (forward-only)")
    print("=" * 60)

    dubins = DubinsStateSpace(turningRadius=turning_radius)
    dubins.setBounds(bounds)

    start_d = dubins.allocState()
    goal_d = dubins.allocState()

    # Case: backing up (Reeds-Shepp wins)
    print("\n--- Case: Backing Up ---")
    print("Start: (0, 0) facing +X")
    print("Goal:  (-3, 0) facing +X")

    rs.getStateAs(start).setXY(0, 0)
    rs.getStateAs(start).setYaw(0)
    rs.getStateAs(goal).setXY(-3, 0)
    rs.getStateAs(goal).setYaw(0)

    dubins.getStateAs(start_d).setXY(0, 0)
    dubins.getStateAs(start_d).setYaw(0)
    dubins.getStateAs(goal_d).setXY(-3, 0)
    dubins.getStateAs(goal_d).setYaw(0)

    rs_dist = rs.distance(start, goal)
    dubins_dist = dubins.distance(start_d, goal_d)

    print(f"Reeds-Shepp: {rs_dist:.3f} m (can reverse)")
    print(f"Dubins:      {dubins_dist:.3f} m (must loop around)")
    print(
        f"Savings:     {dubins_dist - rs_dist:.3f} m ({100 * (1 - rs_dist / dubins_dist):.1f}%)"
    )

    # Case: forward travel (similar)
    print("\n--- Case: Forward Travel ---")
    print("Start: (0, 0) facing +X")
    print("Goal:  (5, 0) facing +X")

    rs.getStateAs(start).setXY(0, 0)
    rs.getStateAs(start).setYaw(0)
    rs.getStateAs(goal).setXY(5, 0)
    rs.getStateAs(goal).setYaw(0)

    dubins.getStateAs(start_d).setXY(0, 0)
    dubins.getStateAs(start_d).setYaw(0)
    dubins.getStateAs(goal_d).setXY(5, 0)
    dubins.getStateAs(goal_d).setYaw(0)

    rs_dist = rs.distance(start, goal)
    dubins_dist = dubins.distance(start_d, goal_d)

    print(f"Reeds-Shepp: {rs_dist:.3f} m")
    print(f"Dubins:      {dubins_dist:.3f} m")
    print("(Same - no benefit from reverse)")

    # Cleanup
    rs.freeState(start)
    rs.freeState(goal)
    rs.freeState(interp)
    dubins.freeState(start_d)
    dubins.freeState(goal_d)

    print("\n" + "=" * 60)
    print("Example completed successfully!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    run()
