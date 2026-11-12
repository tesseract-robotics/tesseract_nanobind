# Kinematics

<!-- snippet: ambient
trajectory = [np.zeros(6), np.full(6, 0.1)]
-->

Forward kinematics maps joint values to link poses; inverse kinematics finds the joint values
that put a link at a pose. Both run on a **kinematic group**, a set of joints named in the SRDF
(`"manipulator"`), with the solvers the SRDF's kinematics plugin configuration selects: KDL
(numerical), OPW and UR (closed form). `robot.fk(...)` and `robot.ik(...)` are the short form;
the kinematic group gives limits, Jacobians and every IK branch. Angles are rad, lengths m;
[Frames and poses](../concepts/frames.md) fixes the frame of each pose.

## Kinematic groups

A kinematic group defines a chain of joints for FK/IK calculations:

```xml
<!-- In robot.srdf -->
<group name="manipulator">
  <chain base_link="base_link" tip_link="tool0"/>
</group>
```

### Getting a kinematic group

```python
from tesseract_robotics.planning import Robot

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
robot = Robot.from_tesseract_support("abb_irb2400", locator)
manip = robot.env.getKinematicGroup("manipulator")

# Group info
print(f"Joint names:     {list(manip.getJointIds())}")
print(f"Link names:      {list(manip.getLinkIds())}")
print(f"Base link:       {manip.getBaseLinkId()}")
print(f"Tip link(s):     {list(manip.getAllPossibleTipLinkIds())}")
```

## Forward kinematics

Compute a link's pose from joint values.

=== "High-Level API"

    ```python
    import numpy as np

    joints = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

    # group_name, joint positions, then the link whose pose you want
    tcp_pose = robot.fk("manipulator", joints, "tool0")

    print(f"Position:   ({tcp_pose.x}, {tcp_pose.y}, {tcp_pose.z})")
    print(f"Quaternion: ({tcp_pose.qx}, {tcp_pose.qy}, {tcp_pose.qz}, {tcp_pose.qw})")
    ```

    !!! info "Quaternion convention — scalar-last project-wide"
        Project canonical quaternion order is `[qx, qy, qz, qw]` (scalar-last).
        `Pose.qx/qy/qz/qw`, `Pose.quaternion`, `Quaterniond.from_xyzw`,
        `q.coeffs()`, and every factory in `common` follow this.

        There is no scalar-first constructor: build one with
        `Quaterniond.from_xyzw(qx, qy, qz, qw)` or `Quaterniond([qx, qy, qz, qw])`.

=== "Low-Level API"

    ```python
    # Returns dict[str, Isometry3d] — one transform per link in the chain
    transforms = manip.calcFwdKin(joints)

    tcp_isometry = transforms["tool0"]
    print(tcp_isometry.translation)   # np.ndarray(3,), a read-only property
    print(tcp_isometry.rotation)      # np.ndarray(3, 3)
    ```

### Pose vs Isometry3d

`Pose` is a subclass of `Isometry3d` (see `planning/transforms.py`). Every `Pose` IS an `Isometry3d`, so any C++ binding that accepts `Isometry3d` accepts `Pose` directly — no conversion step.

```python
from tesseract_robotics.planning import Pose
from tesseract_robotics.common import Isometry3d
import numpy as np

pose = Pose.from_xyz_rpy(0.5, 0.0, 0.3, 0, 0, np.pi / 4)
assert isinstance(pose, Isometry3d)   # subclass relationship

# Need a plain Isometry3d? Pass the Pose through the Isometry3d ctor.
iso = Isometry3d(pose)

# Wrap an existing Isometry3d as a Pose (gains the ergonomic factories +
# @-operator composition).
pose2 = Pose(iso)

# Composition (Pose uses Python's @ operator)
combined = pose @ pose2

# Low-level composition (Isometry3d uses *)
iso_combined = iso * iso
```

## Inverse kinematics

Find joint values that achieve a target pose.

=== "High-Level API"

    ```python
    from tesseract_robotics.planning import Pose

    target = Pose.from_xyz_rpy(0.6, 0.0, 0.5, 0.0, np.pi, 0.0)

    from tesseract_robotics.planning import IKNoSolutionError

    # Returns the first solution as an np.ndarray; raises IKNoSolutionError when there is none
    try:
        solution = robot.ik("manipulator", target, "tool0", seed=joints)
        print(f"Solution: {solution}")
    except IKNoSolutionError:
        print("No IK solution found")

    # Enumerate all solutions (e.g. redundant 7-DOF arms)
    solutions = robot.ik("manipulator", target, "tool0", seed=joints, all_solutions=True)
    print(f"Found {len(solutions)} solution(s)")
    ```

    The full signature is `robot.ik(group_name, target_pose, tip_link, seed=None, all_solutions=False)`; `tip_link` is required — see `planning/core.py`.

=== "Low-Level API"

    ```python
    from tesseract_robotics.kinematics import KinGroupIKInput

    # Pose subclasses Isometry3d, so KinGroupIKInput accepts it directly.
    working_frame = manip.getBaseLinkId()
    tip_link = list(manip.getActiveLinkIds())[-1]

    ik_input = KinGroupIKInput(target, working_frame, tip_link)
    seed = np.zeros(6)
    solutions = manip.calcInvKin(ik_input, seed)  # list[np.ndarray]
    ```

!!! warning "IK may raise `IKNoSolutionError`"
    `robot.ik(...)` raises `IKNoSolutionError` (a `RuntimeError`) when:

    - the target is outside the workspace,
    - no IK solver is configured in the SRDF,
    - the solver hit its iteration limit.

A closed-form solver returns every branch:

<figure markdown>
![Four IRB2400 poses side by side, all with the flange at the same RGB-axis target: two reaching forward (wrist + and −) and two with the base turned around and the upper arm leaning back over the base. ---](../assets/figures/ik_branches.svg#only-light)
![Four IRB2400 poses side by side, all with the flange at the same RGB-axis target: two reaching forward (wrist + and −) and two with the base turned around and the upper arm leaning back over the base. ---](../assets/figures/ik_branches-dark.svg#only-dark)
<figcaption markdown="span">`Robot.ik(..., all_solutions=True)` returns every branch within the joint limits: four for this tool0 pose on the IRB2400 (0.7, 0.2, 1.4 m, tool z along +x), facing the target or turned half a revolution and leaning back, each with the wrist flipped or not. Joint values in rad; forward kinematics of each lands on the pose. Reproduce: `pixi run docs-figures ik_branches`.</figcaption>
</figure>

### IK solvers

| Solver | Type | Robots |
|--------|------|--------|
| **KDL** | Numerical | Any serial chain |
| **OPW** | Analytical | 6-DOF industrial arms (ABB, KUKA, Fanuc) |
| **UR** | Analytical | Universal Robots |

Configure the solver in the SRDF:

```xml
<kinematics_plugin_config>
  <group name="manipulator">
    <plugin name="KDLInvKin"/>
  </group>
</kinematics_plugin_config>
```

### Filtering IK solutions by configuration

An analytical solver returns one solution per arm configuration. `getRobotConfig`
classifies a six-axis solution by three letters: wrist **F**lipped or **N**ot (sign of
joint 5), elbow **U**p or **D**own (joint 3 against π/2), and arm in fron**T** of or
**B**ehind joint 1. Keep one family to get a consistent path, e.g. no wrist flip on an
ABB IRB2400:

```python
import numpy as np
from tesseract_robotics.planning import Pose, Robot
from tesseract_robotics.kinematics import KinGroupIKInput
from tesseract_robotics.motion_planners import (
    RobotConfig,
    getJointTurns,
    getRobotConfig,
    isWithinJointLimits,
)

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
robot = Robot.from_tesseract_support("abb_irb2400", locator)
manip = robot.env.getKinematicGroup("manipulator")

# tool0 pointing along +x of base_link
target = Pose.from_xyz_quat(0.8, 0.0, 1.0, 0.0, 0.70710678, 0.0, 0.70710678)
solutions = manip.calcInvKin(KinGroupIKInput(target, "base_link", "tool0"), np.zeros(6))

for q in solutions:
    config = getRobotConfig(manip, "base_link", "tool0", q)
    print(config.name, getJointTurns(q))   # FUT [ 0  0  0 -1  0 -1], NUT [0 0 0 0 0 0]

no_flip = [q for q in solutions if getRobotConfig(manip, "base_link", "tool0", q) == RobotConfig.NUT]

# Joint limits plus a joint-4 + joint-6 coupling window (for a dress pack, say).
limits = manip.getLimits().joint_limits
in_window = [q for q in no_flip if isWithinJointLimits(q, limits, np.array([-np.pi, np.pi]))]
```

`getRobotConfig` reads the last six joint values, so it works for an arm on a rail or
positioner too. Its `sign_correction` argument (default `[1, 1]`) flips the sign of
joint 3 and joint 5 for manufacturers whose joint directions differ from the convention.
`getJointTurns` gives `int(q / π)` per joint: non-zero where a wrist joint is more than
half a turn from zero.

## Joint limits

```python
limits = manip.getLimits()

# Position limits: 2-column matrix [lower, upper] per joint
pos_limits = limits.joint_limits
print(f"Joint 1: [{pos_limits[0, 0]}, {pos_limits[0, 1]}]")

# Velocity / acceleration limits (same shape)
vel_limits = limits.velocity_limits
acc_limits = limits.acceleration_limits
```

`Robot.get_joint_limits("manipulator")` returns the same information as a `{joint_name: {lower, upper, velocity, acceleration}}` dict.

### Checking limits

```python
def is_within_limits(joints: np.ndarray, manip) -> bool:
    limits = manip.getLimits()
    lower = limits.joint_limits[:, 0]
    upper = limits.joint_limits[:, 1]
    return bool(np.all(joints >= lower) and np.all(joints <= upper))
```

### Sample grids and reach

The utilities the positioner solvers sample with are callable directly:

```python
from tesseract_robotics import kinematics as tk

graph = env.getSceneGraph()
limits = tk.gatherJointLimits(graph, manip.getJointIds())  # (N, 2) [lower, upper]
grid = tk.buildSampleGrid(limits, np.full(len(limits), 0.1))
# grid[i]: ceil((hi - lo) / 0.1) + 1 evenly spaced values of joint i (not their product)

reach = tk.computeChainReachUpperBound(graph, "base_link", "tool0")  # metres
```

`computeChainReachUpperBound` sums the joint offsets along the chain (plus each prismatic
joint's largest |limit|), so it bounds the base-to-tip distance over every configuration.
`PositionerSampleResolution(name, value, min=None, max=None)` is the per-joint resolution record
of the REP / ROP solver configs; `None` bounds default to the joint's limits.

## Jacobian

The Jacobian relates joint velocities to end-effector velocities:

```python
# 6xN Jacobian matrix at the named link
jacobian = manip.calcJacobian(joints, "tool0")

joint_velocities = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
tcp_velocity = jacobian @ joint_velocities  # 6D twist [vx, vy, vz, wx, wy, wz]
```

### Singularity detection

Two measures, both from the singular values σ of the Jacobian:

```python
from tesseract_robotics.kinematics import (
    calcManipulability, calcManipulabilityVolume, calcManipulabilityVolume_each,
    isNearSingularity,
)

near = isNearSingularity(jacobian)             # smallest σ < 0.01 (threshold=...)
w = calcManipulabilityVolume(jacobian)         # Yoshikawa manipulability: the product of all six σ
ellipsoids = calcManipulability(jacobian)      # the velocity and force ellipsoids, whole, linear and angular
assert w == ellipsoids.m.volume

# Along a path: one call for the (M, 6, n) stack, with the GIL released
waypoints = np.linspace(joints, joints + 0.2, 5)
volumes = calcManipulabilityVolume_each(
    np.stack([manip.calcJacobian(q, "tool0") for q in waypoints])
)
```

`isNearSingularity` answers "is one direction of motion nearly lost?"; the manipulability
falls smoothly towards 0 as a singularity approaches, which makes it the one to plot along a
path or to bound in an optimization. Its value depends on the robot and the length unit, so
compare it between configurations of one robot.

Use these instead of `np.sqrt(np.linalg.det(J @ J.T))`: forming J·Jᵀ squares the condition
number, so a singular value below about 1.5e-8 of the largest (√ε) is lost in rounding and a
near-singular configuration cannot be told from a singular one. A Jacobian with a NaN or infinite entry raises
`NonFiniteJacobianError` instead of reading as a singularity. The definitions, the rank rule and
the errors are in the [API reference](../api/kinematics.md#manipulability).

## Tool frames

Define Tool Center Point (TCP) offsets relative to the flange:

```python
from tesseract_robotics.planning import Pose

# Gripper extends 15 cm from the flange
tool_offset = Pose.from_xyz(0.0, 0.0, 0.15)

# FK to flange, then apply the tool offset
flange_pose = robot.fk("manipulator", joints, "tool0")
tcp_pose = flange_pose @ tool_offset

# For IK, transform the target back into the flange frame
target_tcp = tcp_pose
target_flange = target_tcp @ tool_offset.inverse()
solution = robot.ik("manipulator", target_flange, "tool0")
```

`fk` and `ik` work on links only. For planning, pass the offset to the `MotionProgram`
instead (`tcp_offset=tool_offset`, or the name of an SRDF group TCP) and the planner applies
it: see [Frames and the TCP](planning.md#frames-and-the-tcp).

## Working with multiple groups

Some robots have multiple kinematic groups:

<!-- snippet: skip (needs a robot whose SRDF defines dual-arm and positioner groups; the IRB2400 has only "manipulator") -->
```python
# Dual-arm robot (the IRB2400 has one group, "manipulator")
left_arm  = robot.env.getKinematicGroup("left_arm")
right_arm = robot.env.getKinematicGroup("right_arm")

# Manipulator + external positioner
full_chain = robot.env.getKinematicGroup("manipulator_with_positioner")
```

## Performance tips

!!! tip "Cache kinematic groups"
    Creating a kinematic group walks the SRDF. Cache the group if you're
    looping over many configurations:

    ```python
    manip = robot.env.getKinematicGroup("manipulator")
    for joints in trajectory:
        transforms = manip.calcFwdKin(joints)
    ```

!!! tip "Prefer analytical IK when available"
    OPW is dramatically faster than KDL for supported 6-DOF arms. Configure
    it in your SRDF for production workloads.

!!! note "IK runs without the GIL"
    `calcInvKin` (on `KinematicGroup` and `InverseKinematics`) releases the GIL,
    so IK on worker threads runs in parallel. `calcFwdKin` and
    `getKinematicGroup` hold it. See
    [Threading and the GIL](../developer/threading.md).

## Next steps

- [Collision Detection](collision.md) — Check for collisions against the world
- [Motion Planning](planning.md) — Plan collision-free trajectories
