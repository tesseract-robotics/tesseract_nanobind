# Quickstart

This guide walks you through your first motion planning task.

## Load a Robot

```python
from tesseract_robotics.planning import Robot
import numpy as np

# Load a bundled example robot (ABB IRB2400)
robot = Robot.from_tesseract_support("abb_irb2400")

# Or load from your own URDF/SRDF files
robot = Robot.from_urdf(
    urdf_path="/path/to/robot.urdf",
    srdf_path="/path/to/robot.srdf"
)
```

## Forward Kinematics

Compute the TCP (Tool Center Point) pose from joint values:

```python
# Home position (6 joints for ABB IRB2400)
joints = np.zeros(6)

# Get TCP pose as Isometry3d (4x4 transform)
tcp_pose = robot.fk(joints, group="manipulator")

print(f"Position: {tcp_pose.translation()}")
print(f"Rotation matrix:\n{tcp_pose.rotation()}")
```

## Inverse Kinematics

Find joint values that achieve a target TCP pose:

```python
# Create target pose (translate TCP by 10cm in X)
target = robot.fk(joints)
target.translate([0.1, 0, 0])

# Solve IK (may return multiple solutions)
solutions = robot.ik(target, group="manipulator")

if solutions:
    print(f"Found {len(solutions)} IK solutions")
    print(f"First solution: {solutions[0]}")
else:
    print("No IK solution found - target may be unreachable")
```

## Collision Checking

Check if a configuration is collision-free:

```python
# Check current configuration
is_safe = robot.check_collision(joints)
print(f"Collision-free: {is_safe}")

# Get detailed collision results
contacts = robot.get_contacts(joints)
for contact in contacts:
    print(f"Contact between {contact.link_names}: {contact.distance:.3f}m")
```

## Motion Planning

Plan a collision-free path between configurations:

```python
from tesseract_robotics.planning import Planner

# Define start and goal
start = np.zeros(6)
goal = np.array([0.5, -0.5, 0.5, 0.0, 0.5, 0.0])

# Create planner and plan
planner = Planner(robot)
trajectory = planner.plan(
    start=start,
    goal=goal,
    planner="ompl",  # Options: ompl, trajopt, trajopt_ifopt, simple
    profile="DEFAULT"
)

if trajectory:
    print(f"Planned trajectory with {len(trajectory)} waypoints")

    # Access waypoints
    for i, waypoint in enumerate(trajectory):
        print(f"  Waypoint {i}: {waypoint.positions}")
```

## Using the Composer (Advanced)

For complex multi-step tasks, use the TaskComposer:

```python
from tesseract_robotics.planning import Composer

# Create composer with robot
composer = Composer(robot)

# Add waypoints
composer.add_freespace(goal_joints=np.array([0.5, 0, 0, 0, 0, 0]))
composer.add_cartesian(goal_pose=target_pose)
composer.add_freespace(goal_joints=start)

# Execute planning
result = composer.plan()

if result.success:
    trajectories = result.get_trajectories()
    print(f"Planned {len(trajectories)} trajectory segments")
```

## Visualization

Use the TesseractViewer to visualize robots and trajectories:

```python
from tesseract_robotics.viewer import TesseractViewer

viewer = TesseractViewer()
viewer.update_environment(robot.env, [0, 0, 0])

# Animate a trajectory
if trajectory:
    viewer.update_trajectory(trajectory.raw_results)

viewer.start_serve_background()
print("Open http://localhost:8000 in your browser")
input("Press Enter to exit...")
```

## Next Steps

- [Core Concepts](concepts.md) - Understand the architecture
- [Motion Planning Guide](../user-guide/planning.md) - Deep dive into planners
- [Examples](../examples/index.md) - Learn from working examples
