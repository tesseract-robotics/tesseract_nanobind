# tesseract_robotics.kinematics

<!-- snippet: ambient
from pathlib import Path

from tesseract_robotics import get_tesseract_support_path
from tesseract_robotics.kinematics import KinGroupIKInput, KinematicsPluginFactory
from tests.docs.doc_assets import enter_scratch_dir

enter_scratch_dir()
factory = KinematicsPluginFactory(
    get_tesseract_support_path() / "urdf" / "abb_irb2400_plugins.yaml", locator
)
scene_graph = env.getSceneGraph()
scene_state = env.getState()
joint_names = [str(j) for j in env.getGroupJointIds("manipulator")]
q = np.zeros(6)
group = env.getKinematicGroup("manipulator")
manip = group
-->

Forward and inverse kinematics solvers.

## Kinematic Groups

### KinematicGroup

Full kinematics with FK and IK.

```python
from tesseract_robotics.kinematics import KinematicGroup

# Get from environment (defined in SRDF)
manip = env.getKinematicGroup("manipulator")

# Joint information
joint_names = manip.getJointIds()
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
inv_kin = factory.createInvKin("manipulator", "OPWInvKin", scene_graph, scene_state)
manip = KinematicGroup("manipulator", joint_names, inv_kin, scene_graph, scene_state)
```

Upstream's checks raise `RuntimeError`: `joint_names` of the wrong size or with
other names than the solver's, or a working frame or tip link that is not a
link in `scene_state`.

### JointGroup

Forward kinematics only (no IK solver).

```python
from tesseract_robotics.kinematics import JointGroup

group = env.getJointGroup("manipulator")

# Same interface as KinematicGroup for FK
```

### Jacobians

`calcJacobian` has the four C++ overloads. `base_link_id` is the frame the
Jacobian is expressed in; `link_point` is a point in the link frame (the
Jacobian's reference point moves there), a NumPy array of 3 floats.

```python
p = np.array([0.0, 0.0, 0.1])
J = group.calcJacobian(q, "tool0")                  # in the group base frame
J = group.calcJacobian(q, "tool0", p)               # at a point on tool0
J = group.calcJacobian(q, "link_2", "tool0")        # expressed in link_2
J = group.calcJacobian(q, "link_2", "tool0", p)
```

An unknown link name raises `KeyError`. `calcJacobianWithPoint` is removed:
pass the point to `calcJacobian(q, link, p)`.

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
print(f"TCP position: {tcp_pose.translation}")
print(f"TCP rotation:\n{tcp_pose.rotation}")
```

## Inverse Kinematics

Compute joint values for a target pose. `calcInvKin` takes a `KinGroupIKInput`
(not a raw `Isometry3d`) — it bundles the target pose with the tip link and
working frame.

```python
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.kinematics import KinGroupIKInput
import numpy as np

manip = env.getKinematicGroup("manipulator")

# Build a reachable target: the tool0 pose at known joint values
target_pose = Isometry3d(manip.calcFwdKin(np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0]))["tool0"])

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
one per tip link. `calcInvKinMultiple` is removed.

```python
another_input = KinGroupIKInput(target_pose, "base_link", "tool0")
solutions = manip.calcInvKin([ik_input, another_input], seed)
```

## Redundant Solutions

Get all IK solutions including joint wraparound.

```python
from tesseract_robotics.kinematics import getRedundantSolutions

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

## Kinematics Utilities

From `tesseract/kinematics/utils.h` and `validate.h`. Each one checks its
inputs first and raises a Python exception where upstream would index or read
out of bounds.

```python
import numpy as np
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.kinematics import (
    calcManipulability, calcManipulabilityVolume, calcManipulabilityVolume_each,
    checkKinematics, harmonizeTowardMedian, harmonizeTowardZero,
    isNearSingularity, isValid, numericalJacobian,
)

# Finite-difference Jacobian (forward difference, step 1e-8): an oracle for calcJacobian
J_num = numericalJacobian(Isometry3d.Identity(), group, q, "tool0", np.zeros(3))
J_rel = numericalJacobian(group, q, "link_2", Isometry3d.Identity(), "tool0", Isometry3d.Identity())

J = group.calcJacobian(q, "tool0")
manip = calcManipulability(J)        # 6-row jacobian; manip.m, .m_linear, .f, ...
manip.m.eigen_values, manip.m.measure, manip.m.condition, manip.m.volume
w = calcManipulabilityVolume(J)      # bitwise manip.m.volume, from one SVD instead of three
ws = calcManipulabilityVolume_each(np.stack([J, J]))  # (M, 6, n) -> (M,), GIL released
isNearSingularity(J)                 # smallest singular value < 0.01 (threshold=...)

qs = np.array([3 * np.pi / 2, -3 * np.pi / 2, 5.0])
harmonizeTowardZero(qs, [0, 1])      # in place: [-pi/2, pi/2, 5.0]
harmonizeTowardMedian(qs, [0, 1], limits.joint_limits)  # in place, around each joint's limit midpoint

isValid([0.0] * 6)                   # all six finite
checkKinematics(env.getKinematicGroup("manipulator"))  # FK -> IK round trip, tol=1e-3 (m and rad)
```

| function | raises |
| --- | --- |
| `numericalJacobian` | `KeyError` for a link the solver or group does not have; `ValueError` when `joint_values` is not `numJoints()` long |
| `calcManipulability`, `calcManipulabilityVolume` | `TypeError` for a jacobian without 6 rows; `EmptyJacobianError` for one without columns; `NonFiniteJacobianError` for a NaN or infinite entry |
| `calcManipulabilityVolume_each` | `TypeError` for anything but an (M, 6, n) stack; `EmptyJacobianError` for n = 0; `NonFiniteJacobianError` naming the index of the jacobian with a NaN or infinite entry |
| `isNearSingularity` | `EmptyJacobianError` for a matrix without rows or columns; `NonFiniteJacobianError` for a NaN or infinite entry |
| `harmonizeTowardZero`, `harmonizeTowardMedian` | `IndexError` for an index outside `qs` (or `position_limits`), before any write; `TypeError` for a list, a tuple or an array that would need converting (`float32`, read-only) |
| `isValid` | `TypeError` for anything but 6 values |

!!! note "Upstream semantics"
    `calcManipulability` sets `measure` and `condition` to the largest double
    (`sys.float_info.max`) when an ellipsoid's smallest eigenvalue is about zero.
    `checkKinematics` returns `True` when IK finds no solution at all: only a
    solution farther than `tol` from the FK pose makes it `False`.
    `checkKinematics` and `calcManipulabilityVolume_each` release the GIL while they run.

### Manipulability

Yoshikawa's manipulability of a 6 × n jacobian $J$ is the volume of its velocity ellipsoid, the
product of its singular values $\sigma_i$:

$$
w(J) = \sqrt{\det\left(J J^{\mathsf T}\right)} = \prod_{i=1}^{6} \sigma_i
$$

It is 0 at a singularity, and for every jacobian with fewer than 6 columns. The rows of $J$ mix
m/rad (linear, rows 0–2) and rad/rad (angular, rows 3–5), so $w$ depends on the length unit:
compare it between configurations of one robot, not between robots.

| call | returns | cost |
| --- | --- | --- |
| `calcManipulability(J)` | `Manipulability`: the velocity ellipsoids `m`, `m_linear`, `m_angular` and the force ellipsoids `f`, `f_linear`, `f_angular`, each with `eigen_values`, `measure`, `condition` and `volume` | three SVDs: $J$, its linear rows and its angular rows |
| `calcManipulabilityVolume(J)` | `float`, the same bits as `calcManipulability(J).m.volume` | one SVD |
| `calcManipulabilityVolume_each(Js)` | `(M,)` array for an `(M, 6, n)` stack; row `i` is the same bits as `calcManipulabilityVolume(Js[i])` | one SVD per jacobian, with the GIL released |

All three follow tesseract's rule (tesseract#1396). $w$ is the product of the singular values,
multiplied in increasing order, when $J$ has full numerical row rank, and exactly `0.0` below
it. A singular value counts as zero when $\sigma_i \le \min(6, n)\,\varepsilon\,\sigma_{\max}$
(Eigen's `JacobiSVD` rank threshold, $\varepsilon$ = 2.2e-16). The values come from the SVD of
$J$, not from $J J^{\mathsf T}$, which would square the condition number and lose every singular
value below $\sqrt{\varepsilon}\,\sigma_{\max}$.

```python
from tesseract_robotics.kinematics import NonFiniteJacobianError

Js = np.stack([J, 2.0 * J])
ws = calcManipulabilityVolume_each(Js)               # C-contiguous float64 is read in place
assert ws[0] == calcManipulabilityVolume(J) == calcManipulability(J).m.volume

bad = J.copy()
bad[2, 3] = np.nan
try:
    calcManipulabilityVolume_each(np.stack([J, bad]))
except NonFiniteJacobianError as error:              # a ValueError
    assert "jacobian 1 is not finite (entry [2, 3] = nan)" in str(error)
```

`EmptyJacobianError` and `NonFiniteJacobianError` are `ValueError`s. A jacobian with a NaN or
infinite entry is refused because its SVD reads rank 0, which is the answer for a singularity:
volume `0.0`, and `isNearSingularity` true. A blown-up forward-kinematics chain would
otherwise read as a wrist singularity.

`calcManipulabilityVolume` is tesseract's function (tesseract#1408, carried on the build's fork
until it merges). `calcManipulabilityVolume_each` is the bindings' batch form of it and keeps its
spelling ([binding conventions](../developer/binding-conventions.md#names), rule 4). Every
decomposition runs inside tesseract, in the routine `calcManipulability` uses, which is why the
values are the same bits on every platform and whatever was imported first: a second SVD compiled
into the binding module differed in the last bits on linux-arm64.

| claim | checked by |
| --- | --- |
| same bits as `calcManipulability(J).m.volume`, for n = 5, 6, 7 | `test_calc_manipulability_volume_is_the_full_calls_volume_bitwise`, `test_calc_manipulability_volume_each_row_is_the_per_jacobian_call_bitwise` |
| same bits across the rank threshold, $\sigma_{\min}/\sigma_{\max}$ from 1e-1 to 1e-20 | `test_calc_manipulability_volume_is_the_full_calls_volume_across_the_rank_threshold` |
| a near-singular $J$ ($\sigma_{\min}$ = 1e-6) reads $\prod \sigma_i$, not 0 | `test_calc_manipulability_volume_near_singular_is_singular_value_product` |
| same bits with `environment` imported before `kinematics` | `test_calc_manipulability_volume_is_the_full_calls_volume_with_environment_imported_first` |
| the GIL is released | `test_calc_manipulability_volume_each_releases_gil` |

The tests are in `tests/tesseract_kinematics/test_kinematics_utils.py` and
`tests/test_gil_entry_points.py`. No timing is claimed: none has been measured on this build.

`solvePInv` and `dampedPInv` are not bound (use `np.linalg.pinv` / `lstsq`),
nor is `getRedundantSolutionsHelper` (the recursion behind
`getRedundantSolutions`).

## UR Robot Parameters

Analytical IK parameters for Universal Robots.

```python
from tesseract_robotics.kinematics import (
    URParameters, UR10Parameters, UR10eParameters,
    UR5Parameters, UR5eParameters, UR3Parameters, UR3eParameters
)

# Get default parameters
params = UR10Parameters  # a ready-made URParameters, not a class
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

from tesseract_robotics import get_tesseract_support_path
from tesseract_robotics.kinematics import KinematicsPluginFactory

plugins_path = get_tesseract_support_path() / "urdf" / "abb_irb2400_plugins.yaml"
factory = KinematicsPluginFactory(plugins_path, locator)

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
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.kinematics import KinGroupIKInput
import numpy as np

# Load robot
locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
robot = Robot.from_tesseract_support("abb_irb2400", locator)
manip = robot.env.getKinematicGroup("manipulator")

# Current pose
joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
fk = manip.calcFwdKin(joints)
current_pose = fk["tool0"]

# Move 10cm in X — build a new 4x4 matrix
mat = current_pose.matrix.copy()
mat[0, 3] += 0.1
target_pose = Isometry3d(mat)

# Solve IK
solutions = manip.calcInvKin(KinGroupIKInput(target_pose, "base_link", "tool0"), joints)

if solutions:
    # Verify solution
    fk_check = manip.calcFwdKin(solutions[0])
    check_pose = fk_check["tool0"]

    error = np.linalg.norm(
        target_pose.translation - check_pose.translation
    )
    print(f"Position error: {error:.6f} m")
```

## Tips

1. **IK Seeds**: Better seeds = faster convergence
2. **Multiple Solutions**: Most 6-DOF robots have up to 8 IK solutions
3. **Joint Limits**: IK solvers respect joint limits
4. **Analytical vs Numerical**: OPW (UR, ABB) is analytical and fast; KDL is numerical

## Reference

::: tesseract_robotics.kinematics
