# tesseract_robotics.tesseract_kinematics

Forward and inverse kinematics solvers.

## Kinematic Groups

### KinematicGroup

Full kinematics with FK and IK.

```python
from tesseract_robotics.tesseract_kinematics import KinematicGroup

# Get from environment (defined in SRDF)
manip = env.getKinematicGroup("manipulator")

# Joint information
joint_names = manip.getJointNames()
n_joints = len(joint_names)

# Limits
limits = manip.getLimits()
pos_min = limits.joint_limits[:, 0]
pos_max = limits.joint_limits[:, 1]
vel_limits = limits.velocity_limits
```

### JointGroup

Forward kinematics only (no IK solver).

```python
from tesseract_robotics.tesseract_kinematics import JointGroup

group = env.getJointGroup("manipulator")

# Same interface as KinematicGroup for FK
```

## Forward Kinematics

Compute link transforms from joint values.

```python
import numpy as np

manip = env.getKinematicGroup("manipulator")
joint_values = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

# Compute all link transforms
transforms = manip.calcFwdKin(joint_values)

# Get specific link
tcp_pose = transforms["tool0"]
print(f"TCP position: {tcp_pose.translation()}")
print(f"TCP rotation:\n{tcp_pose.rotation()}")
```

## Inverse Kinematics

Compute joint values for target pose.

```python
from tesseract_robotics.tesseract_common import Isometry3d

manip = env.getKinematicGroup("manipulator")

# Target pose
target = Isometry3d.Identity()
target.translate([0.5, 0.2, 0.3])

# Seed (initial guess)
seed = np.zeros(6)

# Solve IK
solutions = manip.calcInvKin(target, seed)

if solutions:
    print(f"Found {len(solutions)} solutions")
    best = solutions[0]
    print(f"Solution: {best}")
else:
    print("No IK solution found")
```

### KinGroupIKInput

Batch IK for multiple targets.

```python
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs

# Single IK input
ik_input = KinGroupIKInput()
ik_input.pose = target_pose
ik_input.tip_link_name = "tool0"
ik_input.working_frame = "base_link"

# Multiple inputs
inputs = KinGroupIKInputs()
inputs.append(ik_input)

# Solve
solutions = manip.calcInvKin(inputs, seed)
```

## Redundant Solutions

Get all IK solutions including joint wraparound.

```python
from tesseract_robotics.tesseract_kinematics import getRedundantSolutions

# Initial solution
solution = solutions[0]

# Get redundant solutions (considering joint limits)
redundant = getRedundantSolutions(
    solution,
    limits.joint_limits[:, 0],  # lower
    limits.joint_limits[:, 1],  # upper
    np.array([2*np.pi] * 6)     # resolution
)
```

## UR Robot Parameters

Analytical IK parameters for Universal Robots.

```python
from tesseract_robotics.tesseract_kinematics import (
    URParameters, UR10Parameters, UR10eParameters,
    UR5Parameters, UR5eParameters, UR3Parameters, UR3eParameters
)

# Get default parameters
params = UR10Parameters()
print(f"d1: {params.d1}")
print(f"a2: {params.a2}")
print(f"a3: {params.a3}")
print(f"d4: {params.d4}")
print(f"d5: {params.d5}")
print(f"d6: {params.d6}")
```

## Kinematics Plugin Factory

Load kinematics solvers from plugins.

```python
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory

# Usually handled by Environment initialization
# Plugins: KDLFwdKin, OPWInvKin, etc.
```

## Usage Example

```python
from tesseract_robotics.planning import Robot
import numpy as np

# Load robot
robot = Robot.from_tesseract_support("abb_irb2400")
manip = robot.env.getKinematicGroup("manipulator")

# Current pose
joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
fk = manip.calcFwdKin(joints)
current_pose = fk["tool0"]

# Move 10cm in X
target = Isometry3d(current_pose.matrix())
target.translate([0.1, 0, 0])

# Solve IK
solutions = manip.calcInvKin(target, joints)

if solutions:
    # Verify solution
    fk_check = manip.calcFwdKin(solutions[0])
    check_pose = fk_check["tool0"]

    error = np.linalg.norm(
        target.translation() - check_pose.translation()
    )
    print(f"Position error: {error:.6f} m")
```

## Tips

1. **IK Seeds**: Better seeds = faster convergence
2. **Multiple Solutions**: Most 6-DOF robots have up to 8 IK solutions
3. **Joint Limits**: IK solvers respect joint limits
4. **Analytical vs Numerical**: OPW (UR, ABB) is analytical and fast; KDL is numerical

## Auto-generated API Reference

::: tesseract_robotics.tesseract_kinematics._tesseract_kinematics
    options:
      show_root_heading: false
      show_source: false
      members_order: source
