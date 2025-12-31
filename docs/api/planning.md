# tesseract_robotics.planning

High-level planning API for robot motion planning.

::: tesseract_robotics.planning
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 2
      docstring_style: google
      show_signature_annotations: true
      separate_signature: true

## Classes

### Robot

The main entry point for working with a robot.

```python
from tesseract_robotics.planning import Robot

# Load from bundled models
robot = Robot.from_tesseract_support("abb_irb2400")

# Load from URDF/SRDF
robot = Robot.from_urdf("robot.urdf", "robot.srdf")

# Access environment
env = robot.env

# Forward kinematics
pose = robot.fk(joints, group="manipulator")

# Inverse kinematics
solutions = robot.ik(target_pose, group="manipulator")

# Collision checking
is_safe = robot.check_collision(joints)
contacts = robot.get_contacts(joints)
```

#### Methods

| Method | Description |
|--------|-------------|
| `from_tesseract_support(name)` | Load bundled robot model |
| `from_urdf(urdf, srdf)` | Load from URDF/SRDF files |
| `fk(joints, group)` | Forward kinematics |
| `ik(pose, group, seed)` | Inverse kinematics |
| `check_collision(joints)` | Collision check (bool) |
| `get_contacts(joints)` | Detailed collision info |
| `get_joint_names(group)` | Joint names for group |
| `get_joint_limits(group)` | Position/velocity limits |

### Planner

Motion planner wrapper.

```python
from tesseract_robotics.planning import Planner

planner = Planner(robot)

# Joint-space planning
trajectory = planner.plan(
    start=start_joints,
    goal=goal_joints,
    planner="ompl"
)

# Cartesian planning
trajectory = planner.plan(
    start=start_joints,
    goal=goal_pose,  # Isometry3d
    planner="trajopt"
)

# Refine existing trajectory
smooth = planner.refine(trajectory, planner="trajopt")
```

#### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `start` | `np.ndarray` | Start joint configuration |
| `goal` | `np.ndarray` or `Isometry3d` | Goal (joint or Cartesian) |
| `planner` | `str` | Planner type: ompl, trajopt, descartes, simple |
| `profile` | `str` | Profile name (default: "DEFAULT") |

### Composer

Task composition for complex sequences.

```python
from tesseract_robotics.planning import Composer

composer = Composer(robot)

# Add motion segments
composer.add_freespace(goal_joints=via_point)
composer.add_cartesian(goal_pose=target_pose)
composer.add_linear(goal_joints=final_joints)

# Plan all segments
result = composer.plan()

if result.success:
    trajectories = result.get_trajectories()
```

#### Methods

| Method | Description |
|--------|-------------|
| `add_freespace(goal_joints)` | Add freespace motion |
| `add_cartesian(goal_pose)` | Add Cartesian motion |
| `add_linear(goal_pose)` | Add linear Cartesian motion |
| `clear()` | Clear all segments |
| `plan()` | Execute planning pipeline |

### Trajectory

Planned trajectory result.

```python
if trajectory:
    for waypoint in trajectory:
        print(f"Positions: {waypoint.positions}")
        print(f"Velocities: {waypoint.velocities}")
        print(f"Time: {waypoint.time}")
```

#### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `positions` | `np.ndarray` | Joint positions |
| `velocities` | `np.ndarray` | Joint velocities |
| `accelerations` | `np.ndarray` | Joint accelerations |
| `time` | `float` | Time from start (seconds) |

### ComposerResult

Result from Composer.plan().

```python
result = composer.plan()

if result.success:
    print(f"Planned {len(result.get_trajectories())} segments")
else:
    print(f"Failed: {result.message}")
```

#### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `success` | `bool` | Planning succeeded |
| `message` | `str` | Error message if failed |
| `raw_results` | object | Raw TaskComposer output |

#### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `get_trajectories()` | `list[Trajectory]` | Extract trajectories |

## Enums

### PlannerType

```python
from tesseract_robotics.planning import PlannerType

PlannerType.OMPL       # Sampling-based (OMPL)
PlannerType.TRAJOPT    # Optimization (TrajOpt)
PlannerType.DESCARTES  # Graph search (Descartes)
PlannerType.SIMPLE     # Interpolation
```

### MotionType

```python
from tesseract_robotics.planning import MotionType

MotionType.FREESPACE  # Any collision-free path
MotionType.LINEAR     # Straight Cartesian line
MotionType.CIRCULAR   # Circular arc
```
