# tesseract_robotics.trajopt_ifopt

Robotics-specific constraints and variables for trajectory optimization.

## Overview

```python
from tesseract_robotics.trajopt_ifopt import (
    # Variables
    JointPosition,
    # Joint constraints
    JointPosConstraint, JointVelConstraint, JointAccelConstraint, JointJerkConstraint,
    # Cartesian constraints
    CartPosInfo, CartPosConstraint, CartLineInfo, CartLineConstraint,
    # Collision constraints
    TrajOptCollisionConfig, DiscreteCollisionConstraint, ContinuousCollisionConstraint,
    # IK constraint
    InverseKinematicsInfo, InverseKinematicsConstraint,
    # Utilities
    interpolate, toBounds,
)
```

## Variables

### JointPosition

Optimization variable representing joint configuration at a timestep.

```python
from tesseract_robotics.trajopt_ifopt import JointPosition
import numpy as np

joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
initial_values = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])

# Create variable
var = JointPosition(initial_values, joint_names, "position_0")

# Access values
print(f"Values: {var.GetValues()}")
print(f"Names: {var.GetJointNames()}")

# Set bounds
from tesseract_robotics.ifopt import Bounds
bounds = [Bounds(-3.14, 3.14) for _ in range(6)]
var.SetBounds(bounds)
```

| Method | Returns | Description |
|--------|---------|-------------|
| `GetValues()` | `np.ndarray` | Current joint values |
| `SetVariables(x)` | `None` | Set joint values |
| `GetJointNames()` | `list[str]` | Joint names |
| `GetBounds()` | `list[Bounds]` | Joint limits |
| `SetBounds(bounds)` | `None` | Set joint limits |

## Joint Constraints

### JointPosConstraint

Constrain joint positions to target values.

```python
from tesseract_robotics.trajopt_ifopt import JointPosConstraint
import numpy as np

targets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # home position
coeffs = np.ones(6)  # weight per joint

constraint = JointPosConstraint(targets, position_var, coeffs, "joint_pos")
```

### JointVelConstraint

Constrain joint velocities between consecutive timesteps.

```python
from tesseract_robotics.trajopt_ifopt import JointVelConstraint

targets = np.zeros(6)  # zero velocity
coeffs = np.ones(6)

# Requires two consecutive position variables
constraint = JointVelConstraint(targets, [pos_var_0, pos_var_1], coeffs, "joint_vel")
```

### JointAccelConstraint

Constrain joint accelerations (requires 3 consecutive positions).

```python
from tesseract_robotics.trajopt_ifopt import JointAccelConstraint

targets = np.zeros(6)
coeffs = np.ones(6)

constraint = JointAccelConstraint(
    targets, [pos_var_0, pos_var_1, pos_var_2], coeffs, "joint_accel"
)
```

### JointJerkConstraint

Constrain joint jerk (requires 4 consecutive positions).

```python
from tesseract_robotics.trajopt_ifopt import JointJerkConstraint

targets = np.zeros(6)
coeffs = np.ones(6)

constraint = JointJerkConstraint(
    targets, [pos_var_0, pos_var_1, pos_var_2, pos_var_3], coeffs, "joint_jerk"
)
```

## Cartesian Constraints

### CartPosInfo / CartPosConstraint

Constrain TCP to a Cartesian pose.

```python
from tesseract_robotics.trajopt_ifopt import CartPosInfo, CartPosInfoType, CartPosConstraint
from tesseract_robotics.tesseract_common import Isometry3d

# Create info struct
info = CartPosInfo()
info.manip = kin_group  # KinematicGroup from env.getKinematicGroup()
info.source_frame = "tool0"
info.target_frame = "base_link"
info.target_frame_offset = target_pose  # Isometry3d

# Constraint indices (which DoFs to constrain)
# CartPosInfoType: FULL (all 6), POSITION_ONLY (xyz), ORIENTATION_ONLY (rpy)
info.indices = [0, 1, 2, 3, 4, 5]  # all 6 DoF

# Create constraint
coeffs = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  # xyz, rpy weights
constraint = CartPosConstraint(info, position_var, coeffs, "cart_pos")

# Get current error
values = constraint.GetValues()
```

### CartLineInfo / CartLineConstraint

Constrain TCP to lie on a line between two points.

```python
from tesseract_robotics.trajopt_ifopt import CartLineInfo, CartLineConstraint

info = CartLineInfo()
info.manip = kin_group
info.source_frame = "tool0"
info.target_frame = "base_link"
info.target_frame_offset1 = line_start  # Isometry3d
info.target_frame_offset2 = line_end    # Isometry3d

constraint = CartLineConstraint(info, position_var, coeffs, "cart_line")
```

## Collision Constraints

### TrajOptCollisionConfig

Configuration for collision checking.

```python
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

config = TrajOptCollisionConfig()
config.contact_manager_config.margin_data_override_type = 0
config.contact_manager_config.margin_data.setDefaultCollisionMargin(0.025)
config.collision_margin_buffer = 0.01
config.collision_coeff_data.default_collision_coeff = 20.0
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `contact_manager_config` | object | Contact manager settings |
| `collision_margin_buffer` | `float` | Safety margin buffer |
| `collision_coeff_data` | object | Collision cost coefficients |

### DiscreteCollisionConstraint

Collision avoidance at a single configuration.

```python
from tesseract_robotics.trajopt_ifopt import (
    DiscreteCollisionConstraint, SingleTimestepCollisionEvaluator, CollisionCache
)

# Create evaluator
cache = CollisionCache(100)
evaluator = SingleTimestepCollisionEvaluator(
    cache, kin_group, env, config, True  # True = dynamic environment
)

# Create constraint
constraint = DiscreteCollisionConstraint(
    evaluator,
    position_var,
    max_num_cnt=3,       # max collision pairs
    fixed_sparsity=False,
    name="collision"
)
```

### ContinuousCollisionConstraint

Collision avoidance between consecutive configurations (swept volume).

```python
from tesseract_robotics.trajopt_ifopt import (
    ContinuousCollisionConstraint, LVSContinuousCollisionEvaluator
)

# LVS = Longest Valid Segment (interpolates and checks)
evaluator = LVSContinuousCollisionEvaluator(
    cache, kin_group, env, config, True
)

# Constraint between two consecutive position variables
constraint = ContinuousCollisionConstraint(
    evaluator,
    position_var_0,
    position_var_1,
    max_num_cnt=3,
    fixed_sparsity=False,
    name="continuous_collision"
)
```

### Collision Evaluators

| Class | Description |
|-------|-------------|
| `SingleTimestepCollisionEvaluator` | Discrete check at single config |
| `LVSDiscreteCollisionEvaluator` | Discrete checks at interpolated points |
| `LVSContinuousCollisionEvaluator` | Swept volume collision check |

## IK Constraint

### InverseKinematicsInfo / InverseKinematicsConstraint

Constrain variables to match IK solution.

```python
from tesseract_robotics.trajopt_ifopt import InverseKinematicsInfo, InverseKinematicsConstraint

info = InverseKinematicsInfo()
info.manip = kin_group
info.working_frame = "base_link"
info.tcp_frame = "tool0"
info.tcp_offset = Isometry3d.Identity()

constraint = InverseKinematicsConstraint(
    target_pose,      # Isometry3d
    info,
    constraint_var,   # variable to constrain
    seed_var,         # seed for IK
    "ik_constraint"
)
```

## Utilities

### interpolate

Linear interpolation between joint configurations.

```python
from tesseract_robotics.trajopt_ifopt import interpolate
import numpy as np

start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
end = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
steps = 10

trajectory = interpolate(start, end, steps)  # (10, 6) array
```

### toBounds

Convert numpy arrays to ifopt Bounds.

```python
from tesseract_robotics.trajopt_ifopt import toBounds
import numpy as np

lower = np.array([-3.14, -3.14, -3.14])
upper = np.array([3.14, 3.14, 3.14])

bounds = toBounds(lower, upper)  # list[Bounds]
```

## Auto-generated API Reference

::: tesseract_robotics.trajopt_ifopt._trajopt_ifopt
    options:
      show_root_heading: false
      show_source: false
      members_order: source
