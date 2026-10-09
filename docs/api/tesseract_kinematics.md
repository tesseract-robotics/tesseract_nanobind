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

A `KinematicGroup` can also be built without an `Environment`, from an
inverse kinematics solver the plugin factory created. The group takes a copy
(`clone()`) of `inv_kin`, so the solver passed in stays usable; the group keeps
it, and so its factory, alive.

```python
inv_kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", scene_graph, scene_state)
manip = KinematicGroup("manipulator", joint_names, inv_kin, scene_graph, scene_state)
```

Upstream's checks raise `RuntimeError`: `joint_names` of the wrong size or with
other names than the solver's, or a working frame or tip link that is not a
link in `scene_state`.

### JointGroup

Forward kinematics only (no IK solver).

```python
from tesseract_robotics.tesseract_kinematics import JointGroup

group = env.getJointGroup("manipulator")

# Same interface as KinematicGroup for FK
```

### Jacobians

`calcJacobian` has the four C++ overloads. `base_link_name` is the frame the
Jacobian is expressed in; `link_point` is a point in the link frame (the
Jacobian's reference point moves there), a NumPy array of 3 floats.

```python
p = np.array([0.0, 0.0, 0.1])
J = group.calcJacobian(q, "tool0")                  # in the group base frame
J = group.calcJacobian(q, "tool0", p)               # at a point on tool0
J = group.calcJacobian(q, "link_2", "tool0")        # expressed in link_2
J = group.calcJacobian(q, "link_2", "tool0", p)
```

An unknown link name raises `KeyError`. `calcJacobianWithPoint(q, link, p)` is
an alias of `calcJacobian(q, link, p)`.

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

### Batch IK

`calcInvKin` also takes a list of `KinGroupIKInput` (or a `KinGroupIKInputs`),
one per tip link; `calcInvKinMultiple` is an alias of the list form.

```python
solutions = manip.calcInvKin([ik_input, another_input], seed)
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

Load kinematics solvers from plugins. `Environment` usually does this from the
SRDF's kinematics plugin config; a factory can also be built from a config file
(`pathlib.Path`) or YAML content (`str`).

```python
from pathlib import Path

from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory

factory = KinematicsPluginFactory(Path("robot_plugins.yaml"), locator)

# Solvers per group: {group_name: PluginInfoContainer}
fwd = factory.getFwdKinPlugins()
info = fwd["manipulator"].plugins["KDLFwdKinChain"]

# Register a second solver for the group and make it the default
factory.addFwdKinPlugin("manipulator", "MyChain", info)
factory.setDefaultFwdKinPlugin("manipulator", "MyChain")

# Create by group and solver name, or from an explicit PluginInfo
fwd_kin = factory.createFwdKin("manipulator", "MyChain", scene_graph, scene_state)
fwd_kin = factory.createFwdKin("my_chain", info, scene_graph, scene_state)

factory.removeFwdKinPlugin("manipulator", "MyChain")
factory.saveConfig(Path("saved_plugins.yaml"))  # getConfig() returns the same YAML as str
```

The same methods exist for inverse kinematics (`addInvKinPlugin`,
`getInvKinPlugins`, …, `createInvKin`). A solver keeps its factory alive: its
code lives in a plugin library the factory loaded.

`setDefault…KinPlugin` and `remove…KinPlugin` raise `KeyError` for an unknown
group or solver. `saveConfig` raises `OSError` when the file cannot be written.

!!! warning "A group's last solver cannot be removed"
    `remove…KinPlugin` raises `KinematicsPluginRemovalError` (a `RuntimeError`)
    when the solver is the last one of its group, and leaves the factory
    unchanged. tesseract 0.35.0 erases the group in that case and then reads,
    and for the default solver writes, through the erased map iterator
    (kinematics_plugin_factory.cpp:150–154; reported upstream as
    [tesseract#1381](https://github.com/tesseract-robotics/tesseract/issues/1381)).
    The guard goes once a fixed tesseract is the minimum version.

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
