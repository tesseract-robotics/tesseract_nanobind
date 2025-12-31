# Kinematics

tesseract_robotics provides forward and inverse kinematics through kinematic groups defined in the SRDF.

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
print(f"Joint names: {manip.getJointNames()}")
print(f"Link names: {manip.getLinkNames()}")
print(f"Base link: {manip.getBaseLinkName()}")
print(f"Tip link: {manip.getTipLinkNames()}")
```

## Forward Kinematics

Compute end-effector pose from joint values:

=== "High-Level API"

    ```python
    import numpy as np

    joints = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

    # Returns Isometry3d (4x4 transform)
    tcp_pose = robot.fk(joints, group="manipulator")

    print(f"Position: {tcp_pose.translation()}")
    print(f"Rotation:\n{tcp_pose.rotation()}")
    ```

=== "Low-Level API"

    ```python
    manip = robot.env.getKinematicGroup("manipulator")

    # Returns dict of link_name -> Isometry3d
    transforms = manip.calcFwdKin(joints)

    tcp_pose = transforms["tool0"]
    ```

### Transform Operations

```python
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Create identity transform
pose = Isometry3d.Identity()

# Translation
pose.translate([0.1, 0.2, 0.3])
position = pose.translation()  # np.array([0.1, 0.2, 0.3])

# Rotation (3x3 matrix)
rotation = pose.rotation()

# Full 4x4 matrix
matrix = pose.matrix()

# Compose transforms
pose2 = Isometry3d.Identity()
pose2.rotate(Eigen.AngleAxisd(np.pi/4, [0, 0, 1]))
combined = pose * pose2
```

## Inverse Kinematics

Find joint values that achieve a target pose:

=== "High-Level API"

    ```python
    # Target pose
    target = robot.fk(np.zeros(6))
    target.translate([0.1, 0, 0])  # Move 10cm in X

    # Solve IK (returns list of solutions)
    solutions = robot.ik(target, group="manipulator")

    if solutions:
        print(f"Found {len(solutions)} solutions")
        for i, sol in enumerate(solutions):
            print(f"  Solution {i}: {sol}")
    else:
        print("No IK solution found")
    ```

=== "Low-Level API"

    ```python
    manip = robot.env.getKinematicGroup("manipulator")

    # With seed (initial guess)
    seed = np.zeros(6)
    solutions = manip.calcInvKin(target, seed)
    ```

!!! warning "IK May Return Empty"
    IK returns an empty list if:

    - Target is outside workspace
    - No IK solver is configured in SRDF
    - Solver iterations exceeded

### IK Solvers

| Solver | Type | Robots |
|--------|------|--------|
| **KDL** | Numerical | Any serial chain |
| **OPW** | Analytical | 6-DOF industrial arms (ABB, KUKA, Fanuc) |
| **UR** | Analytical | Universal Robots |

Configure in SRDF:

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

# Position limits (rad for revolute, m for prismatic)
pos_limits = limits.joint_limits
print(f"Joint 1: [{pos_limits.col(0)[0]}, {pos_limits.col(1)[0]}]")

# Velocity limits (rad/s or m/s)
vel_limits = limits.velocity_limits
print(f"Max velocity: {vel_limits}")

# Acceleration limits
acc_limits = limits.acceleration_limits
```

### Checking Limits

```python
def is_within_limits(joints, manip):
    limits = manip.getLimits()
    lower = limits.joint_limits[:, 0]
    upper = limits.joint_limits[:, 1]
    return np.all(joints >= lower) and np.all(joints <= upper)
```

## Jacobian

The Jacobian relates joint velocities to end-effector velocities:

```python
# 6xN Jacobian matrix
jacobian = manip.calcJacobian(joints, "tool0")

# Use for velocity kinematics
joint_velocities = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
tcp_velocity = jacobian @ joint_velocities  # 6D twist [vx, vy, vz, wx, wy, wz]
```

### Singularity Detection

```python
def manipulability(jacobian):
    """Yoshikawa manipulability index."""
    return np.sqrt(np.linalg.det(jacobian @ jacobian.T))

def is_near_singularity(jacobian, threshold=0.01):
    """Check if near a singularity."""
    return manipulability(jacobian) < threshold
```

## Tool Frames

Define tool center point (TCP) offsets:

```python
from tesseract_robotics.tesseract_common import Isometry3d

# Tool offset (e.g., gripper extends 15cm from flange)
tool_offset = Isometry3d.Identity()
tool_offset.translate([0, 0, 0.15])

# Apply to FK result
flange_pose = robot.fk(joints)
tcp_pose = flange_pose * tool_offset

# For IK, transform target to flange frame
target_flange = target_tcp * tool_offset.inverse()
solutions = robot.ik(target_flange)
```

## Working with Multiple Groups

Some robots have multiple kinematic groups:

```python
# Dual-arm robot
left_arm = env.getKinematicGroup("left_arm")
right_arm = env.getKinematicGroup("right_arm")

# External axis + manipulator
full_chain = env.getKinematicGroup("manipulator_with_positioner")
```

## Performance Tips

!!! tip "Cache Kinematic Groups"
    Creating a kinematic group involves parsing. Cache the group object:

    ```python
    # Do this once
    manip = env.getKinematicGroup("manipulator")

    # Reuse for all FK/IK calls
    for joints in trajectory:
        pose = manip.calcFwdKin(joints)
    ```

!!! tip "Use Analytical IK When Available"
    OPW solver is ~100x faster than KDL for supported robots.
    Configure it in your SRDF for production use.

## Next Steps

- [Collision Detection](collision.md) - Check for collisions
- [Motion Planning](planning.md) - Plan collision-free paths
