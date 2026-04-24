# Kinematics

tesseract_robotics provides forward and inverse kinematics through **kinematic groups** defined in the SRDF. The high-level `Robot` wrapper exposes them as `robot.fk(...)` / `robot.ik(...)`.

## Kinematic Groups

A kinematic group defines a chain of joints for FK/IK calculations:

```xml
<!-- In robot.srdf -->
<group name="manipulator">
  <chain base_link="base_link" tip_link="tool0"/>
</group>
```

### Getting a Kinematic Group

```python
from tesseract_robotics.planning import Robot

robot = Robot.from_tesseract_support("abb_irb2400")
manip = robot.env.getKinematicGroup("manipulator")

# Group info
print(f"Joint names:     {list(manip.getJointNames())}")
print(f"Link names:      {list(manip.getLinkNames())}")
print(f"Base link:       {manip.getBaseLinkName()}")
print(f"Tip link(s):     {list(manip.getTipLinkNames())}")
```

## Forward Kinematics

Compute a link's pose from joint values.

=== "High-Level API"

    ```python
    import numpy as np

    joints = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

    # group_name is the FIRST positional argument;
    # tip_link defaults to the last active link in the chain.
    tcp_pose = robot.fk("manipulator", joints)

    print(f"Position:   {tcp_pose.position}")      # np.ndarray([x, y, z])
    print(f"Quaternion: {tcp_pose.quaternion}")    # [qx, qy, qz, qw] scalar-last
    ```

    !!! info "Quaternion convention"
        `Pose.quaternion` returns `[qx, qy, qz, qw]` (scalar-last) — the same
        convention used by `scipy.spatial.transform.Rotation.as_quat()`. This
        differs from C++ Eigen, which uses scalar-first `[qw, qx, qy, qz]`.

=== "Low-Level API"

    ```python
    # Returns dict[str, Isometry3d] — one transform per link in the chain
    transforms = manip.calcFwdKin(joints)

    tcp_isometry = transforms["tool0"]
    print(tcp_isometry.translation())   # method, not property
    print(tcp_isometry.rotation())
    ```

### Pose vs Isometry3d

`Pose` is the Python-side wrapper around a 4x4 matrix (see `planning/transforms.py`). `Isometry3d` is the low-level C++/Eigen type.

```python
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Pose -> Isometry3d
pose = Pose.from_xyz_rpy(0.5, 0.0, 0.3, 0, 0, np.pi / 4)
iso = pose.to_isometry()

# Isometry3d -> Pose
pose2 = Pose.from_isometry(iso)

# Composition (Pose uses Python's @ operator)
combined = pose @ pose2

# Low-level composition (Isometry3d uses *)
iso_combined = iso * iso
```

## Inverse Kinematics

Find joint values that achieve a target pose.

=== "High-Level API"

    ```python
    from tesseract_robotics.planning import Pose

    target = Pose.from_xyz_rpy(0.6, 0.0, 0.5, 0.0, np.pi, 0.0)

    # Returns a single np.ndarray solution, None, or a list if all_solutions=True
    solution = robot.ik("manipulator", target, seed=joints)
    if solution is not None:
        print(f"Solution: {solution}")
    else:
        print("No IK solution found")

    # Enumerate all solutions (e.g. redundant 7-DOF arms)
    solutions = robot.ik("manipulator", target, seed=joints, all_solutions=True)
    if solutions:
        print(f"Found {len(solutions)} solution(s)")
    ```

    The full signature is `robot.ik(group_name, target_pose, seed=None, tip_link=None, all_solutions=False)` — see `planning/core.py`.

=== "Low-Level API"

    ```python
    from tesseract_robotics.tesseract_kinematics import KinGroupIKInput

    target_iso = target.to_isometry()
    working_frame = manip.getBaseLinkName()
    tip_link = list(manip.getActiveLinkNames())[-1]

    ik_input = KinGroupIKInput(target_iso, working_frame, tip_link)
    seed = np.zeros(6)
    solutions = manip.calcInvKin(ik_input, seed)  # list[np.ndarray]
    ```

!!! warning "IK may return `None`"
    `robot.ik(...)` returns `None` when:

    - the target is outside the workspace,
    - no IK solver is configured in the SRDF,
    - the solver hit its iteration limit.

### IK Solvers

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

## Joint Limits

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

### Checking Limits

```python
def is_within_limits(joints: np.ndarray, manip) -> bool:
    limits = manip.getLimits()
    lower = limits.joint_limits[:, 0]
    upper = limits.joint_limits[:, 1]
    return bool(np.all(joints >= lower) and np.all(joints <= upper))
```

## Jacobian

The Jacobian relates joint velocities to end-effector velocities:

```python
# 6xN Jacobian matrix at the named link
jacobian = manip.calcJacobian(joints, "tool0")

joint_velocities = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
tcp_velocity = jacobian @ joint_velocities  # 6D twist [vx, vy, vz, wx, wy, wz]
```

### Singularity Detection

```python
def manipulability(jacobian: np.ndarray) -> float:
    """Yoshikawa manipulability index."""
    return float(np.sqrt(np.linalg.det(jacobian @ jacobian.T)))

def is_near_singularity(jacobian: np.ndarray, threshold: float = 0.01) -> bool:
    return manipulability(jacobian) < threshold
```

## Tool Frames

Define Tool Center Point (TCP) offsets relative to the flange:

```python
from tesseract_robotics.planning import Pose

# Gripper extends 15 cm from the flange
tool_offset = Pose.from_xyz(0.0, 0.0, 0.15)

# FK to flange, then apply the tool offset
flange_pose = robot.fk("manipulator", joints)
tcp_pose = flange_pose @ tool_offset

# For IK, transform the target back into the flange frame
target_flange = target_tcp @ tool_offset.inverse()
solution = robot.ik("manipulator", target_flange)
```

## Working with Multiple Groups

Some robots have multiple kinematic groups:

```python
# Dual-arm robot
left_arm  = robot.env.getKinematicGroup("left_arm")
right_arm = robot.env.getKinematicGroup("right_arm")

# Manipulator + external positioner
full_chain = robot.env.getKinematicGroup("manipulator_with_positioner")
```

## Performance Tips

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

## Next Steps

- [Collision Detection](collision.md) — Check for collisions against the world
- [Motion Planning](planning.md) — Plan collision-free trajectories
