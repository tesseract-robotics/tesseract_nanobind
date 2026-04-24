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

Compute joint values for a target pose. `calcInvKin` takes a `KinGroupIKInput`
(not a raw `Isometry3d`) — it bundles the target pose with the tip link and
working frame.

```python
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
import numpy as np

manip = env.getKinematicGroup("manipulator")

# Build target
mat = np.eye(4)
mat[:3, 3] = [0.5, 0.2, 0.3]
target_pose = Isometry3d(mat)

ik_input = KinGroupIKInput(target_pose, "base_link", "tool0")
seed = np.zeros(6)

solutions = manip.calcInvKin(ik_input, seed)

if solutions:
    print(f"Found {len(solutions)} solutions")
    best = solutions[0]
else:
    print("No IK solution found")
```

### Batch IK — KinGroupIKInputs

```python
from tesseract_robotics.tesseract_kinematics import KinGroupIKInputs

inputs = KinGroupIKInputs()
inputs.append(ik_input)
inputs.append(another_input)

solutions = manip.calcInvKin(inputs, seed)
```

## Redundant Solutions

Get all IK solutions including joint wraparound.

```python
from tesseract_robotics.tesseract_kinematics import getRedundantSolutions

# Initial solution
solution = solutions[0]

# Get redundant solutions by adding ±2π to redundancy-capable joints
# Signature: getRedundantSolutions(sol, limits[:, 0:2], redundancy_capable_joint_indices)
redundant = getRedundantSolutions(
    solution,
    limits.joint_limits,       # N×2 matrix (lower, upper per joint)
    [5],                       # redundant joint indices (e.g., last wrist joint)
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
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
import numpy as np

# Load robot
robot = Robot.from_tesseract_support("abb_irb2400")
manip = robot.env.getKinematicGroup("manipulator")

# Current pose
joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
fk = manip.calcFwdKin(joints)
current_pose = fk["tool0"]

# Move 10cm in X — build a new 4x4 matrix
mat = current_pose.matrix().copy()
mat[0, 3] += 0.1
target_pose = Isometry3d(mat)

# Solve IK
solutions = manip.calcInvKin(KinGroupIKInput(target_pose, "base_link", "tool0"), joints)

if solutions:
    # Verify solution
    fk_check = manip.calcFwdKin(solutions[0])
    check_pose = fk_check["tool0"]

    error = np.linalg.norm(
        target_pose.translation() - check_pose.translation()
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
