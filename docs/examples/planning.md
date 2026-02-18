# Planning Examples

Motion planning examples from simple freespace to complex industrial tasks.

## Freespace OMPL

Basic OMPL planning between joint configurations:

```python title="freespace_ompl_example.py"
from tesseract_robotics.planning import Robot, Planner
import numpy as np

# Load robot
robot = Robot.from_tesseract_support("abb_irb2400")

# Define start and goal
start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
goal = np.array([1.0, -0.5, 0.5, 0.0, 0.5, 0.0])

# Plan
planner = Planner(robot)
trajectory = planner.plan(start=start, goal=goal, planner="ompl")

if trajectory:
    print(f"Planned trajectory with {len(trajectory)} waypoints")
    for i, wp in enumerate(trajectory[:3]):
        print(f"  [{i}] {wp.positions}")
    print("  ...")
else:
    print("Planning failed!")
```

## Basic Cartesian

Cartesian straight-line motion with TrajOpt:

```python title="basic_cartesian_example.py"
from tesseract_robotics.planning import Robot, Planner
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
planner = Planner(robot)

# Start configuration
start = np.zeros(6)

# Cartesian goal (tool pose)
goal_pose = Isometry3d.Identity()
goal_pose.translate([0.8, 0.3, 0.6])

# Rotation: point tool down
from scipy.spatial.transform import Rotation
R = Rotation.from_euler('y', 90, degrees=True)
goal_pose.rotate(R.as_matrix())

# Plan Cartesian path
trajectory = planner.plan(
    start=start,
    goal=goal_pose,  # Cartesian goal
    planner="trajopt"
)

if trajectory:
    print(f"Cartesian path with {len(trajectory)} waypoints")

    # Verify final pose
    final_joints = trajectory[-1].positions
    final_pose = robot.fk(final_joints)
    error = np.linalg.norm(final_pose.translation() - goal_pose.translation())
    print(f"Position error: {error*1000:.2f} mm")
```

## Glass Upright (Orientation Constraint)

Keep end-effector orientation constrained (e.g., carrying a glass of water):

```python title="glass_upright_example.py"
from tesseract_robotics.planning import Robot, Composer
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
composer = Composer(robot)

# Define waypoints (keep tool pointing up)
waypoint_1 = Isometry3d.Identity()
waypoint_1.translate([0.6, 0.3, 0.8])

waypoint_2 = Isometry3d.Identity()
waypoint_2.translate([0.6, -0.3, 0.8])

waypoint_3 = Isometry3d.Identity()
waypoint_3.translate([0.8, 0.0, 0.6])

# All waypoints with upright orientation constraint
for wp in [waypoint_1, waypoint_2, waypoint_3]:
    composer.add_cartesian(
        goal_pose=wp,
        orientation_tolerance=0.1  # ±0.1 rad tolerance on tilt
    )

# Plan with orientation constraint
result = composer.plan()

if result.success:
    print("Glass upright path planned successfully")

    # Verify orientation constraint
    for i, traj in enumerate(result.get_trajectories()):
        for wp in traj:
            pose = robot.fk(wp.positions)
            z_axis = pose.rotation()[:, 2]  # Tool Z-axis
            tilt = np.arccos(np.dot(z_axis, [0, 0, 1]))
            if tilt > 0.15:
                print(f"Warning: tilt = {np.degrees(tilt):.1f}°")
```

## Pick and Place

Complete pick and place workflow with object attachment:

```python title="pick_and_place_example.py"
from tesseract_robotics.planning import Robot, Composer
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType, Visual, Collision
from tesseract_robotics.tesseract_environment import AddLinkCommand, RemoveLinkCommand
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env

# Add workpiece to scene
def add_workpiece(env, name, position):
    link = Link(name)

    visual = Visual()
    visual.geometry = Box(0.05, 0.05, 0.1)
    link.addVisual(visual)

    collision = Collision()
    collision.geometry = Box(0.05, 0.05, 0.1)
    link.addCollision(collision)

    joint = Joint(f"{name}_joint")
    joint.type = JointType.FIXED
    joint.parent_link_name = "base_link"
    joint.child_link_name = name
    joint.parent_to_joint_origin_transform = Isometry3d.Identity()
    joint.parent_to_joint_origin_transform.translate(position)

    env.applyCommand(AddLinkCommand(link, joint))

# Add workpiece at pick location
add_workpiece(env, "workpiece", [0.6, 0.3, 0.05])

# Define poses
pick_approach = Isometry3d.Identity()
pick_approach.translate([0.6, 0.3, 0.25])

pick_pose = Isometry3d.Identity()
pick_pose.translate([0.6, 0.3, 0.12])

place_approach = Isometry3d.Identity()
place_approach.translate([0.6, -0.3, 0.25])

place_pose = Isometry3d.Identity()
place_pose.translate([0.6, -0.3, 0.12])

# Build pick sequence
composer = Composer(robot)

# 1. Approach pick
composer.add_freespace(goal_pose=pick_approach)

# 2. Linear down to pick
composer.add_linear(goal_pose=pick_pose)

# Plan pick approach
pick_result = composer.plan()

if pick_result.success:
    print("Pick approach planned")

    # 3. Attach workpiece to gripper
    # (In practice, move joint parent from world to tool0)

    # 4. Linear retreat
    composer.clear()
    composer.add_linear(goal_pose=pick_approach)

    # 5. Transit to place
    composer.add_freespace(goal_pose=place_approach)

    # 6. Linear down to place
    composer.add_linear(goal_pose=place_pose)

    place_result = composer.plan()

    if place_result.success:
        print("Place sequence planned")
```

## Raster Pattern

Industrial raster pattern for welding/painting:

```python title="raster_example.py"
from tesseract_robotics.planning import Robot, Composer
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
composer = Composer(robot)

# Define raster pattern (5 passes, 10cm apart)
start_x, start_y, z = 0.5, -0.2, 0.5
pass_length = 0.4
pass_spacing = 0.1
num_passes = 5

raster_poses = []
for i in range(num_passes):
    y = start_y + i * pass_spacing

    # Direction alternates each pass
    if i % 2 == 0:
        x_start, x_end = start_x, start_x + pass_length
    else:
        x_start, x_end = start_x + pass_length, start_x

    # Start of pass
    pose_start = Isometry3d.Identity()
    pose_start.translate([x_start, y, z])
    raster_poses.append(pose_start)

    # End of pass
    pose_end = Isometry3d.Identity()
    pose_end.translate([x_end, y, z])
    raster_poses.append(pose_end)

# Add approach
approach = Isometry3d.Identity()
approach.translate([start_x, start_y, z + 0.1])
composer.add_freespace(goal_pose=approach)

# Add raster passes (linear motion)
for pose in raster_poses:
    composer.add_linear(goal_pose=pose)

# Plan
result = composer.plan()

if result.success:
    print(f"Raster pattern with {num_passes} passes planned")
    total_waypoints = sum(len(t) for t in result.get_trajectories())
    print(f"Total waypoints: {total_waypoints}")
```

## Hybrid Planning (OMPL + TrajOpt)

Use OMPL for exploration, TrajOpt for smoothing:

```python title="freespace_hybrid_example.py"
from tesseract_robotics.planning import Robot, Planner
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
planner = Planner(robot)

start = np.zeros(6)
goal = np.array([1.5, -0.8, 0.6, 0.5, 0.8, -0.5])

# Step 1: OMPL finds feasible path
ompl_trajectory = planner.plan(start=start, goal=goal, planner="ompl")

if ompl_trajectory:
    print(f"OMPL found path with {len(ompl_trajectory)} waypoints")

    # Step 2: TrajOpt smooths the path
    smooth_trajectory = planner.refine(
        trajectory=ompl_trajectory,
        planner="trajopt"
    )

    if smooth_trajectory:
        print(f"TrajOpt smoothed to {len(smooth_trajectory)} waypoints")

        # Compare path lengths
        def path_length(traj):
            length = 0
            for i in range(1, len(traj)):
                length += np.linalg.norm(
                    traj[i].positions - traj[i-1].positions
                )
            return length

        print(f"OMPL path length: {path_length(ompl_trajectory):.3f} rad")
        print(f"TrajOpt path length: {path_length(smooth_trajectory):.3f} rad")
```

## Multi-Step with Task Composer

Complex sequence using full Task Composer pipeline:

```python title="car_seat_example.py"
from tesseract_robotics.planning import Robot, Composer
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
composer = Composer(robot)

# Define joint targets for car seat welding simulation
home = np.zeros(6)
positions = [
    np.array([0.3, -0.3, 0.3, 0.0, 0.5, 0.0]),   # Position 1
    np.array([0.5, -0.2, 0.4, 0.2, 0.4, 0.1]),   # Position 2
    np.array([0.4, -0.4, 0.2, -0.1, 0.6, -0.1]), # Position 3
    np.array([0.6, -0.1, 0.3, 0.1, 0.5, 0.2]),   # Position 4
]

# Build program: home → pos1 → pos2 → ... → home
for pos in positions:
    composer.add_freespace(goal_joints=pos)

# Return home
composer.add_freespace(goal_joints=home)

# Plan entire sequence
result = composer.plan()

if result.success:
    print(f"Multi-step sequence planned:")
    for i, traj in enumerate(result.get_trajectories()):
        print(f"  Segment {i+1}: {len(traj)} waypoints")
```

## Visualization

All examples can be visualized:

```python
from tesseract_robotics.viewer import TesseractViewer

viewer = TesseractViewer()
viewer.update_environment(robot.env, [0, 0, 0])

# Show trajectory
if result.success:
    viewer.update_trajectory(result.raw_results)

# Start server
viewer.start_serve_background()
print("Open http://localhost:8000 in browser")
input("Press Enter to exit...")
```

## Next Steps

- [Online Planning](online-planning.md) - Real-time replanning examples
- [Low-Level SQP Guide](../user-guide/low-level-sqp.md) - Direct solver access
