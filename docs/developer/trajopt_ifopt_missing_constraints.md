# TrajOpt IFOPT: Missing Python Bindings Analysis

## Summary

4 C++ constraint classes in `trajopt_ifopt` lack Python bindings.

| Constraint | Priority | Complexity | Use Case |
|-----------|----------|------------|----------|
| `JointJerkConstraint` | High | Low | Smoother trajectories |
| `CartLineConstraint` | Medium | Medium | Linear tool paths |
| `DiscreteCollisionNumericalConstraint` | Low | Low | Alternative collision jacobian |
| `InverseKinematicsConstraint` | Low | High | IK-based optimization |

---

## 1. JointJerkConstraint

**File:** `trajopt_ifopt/constraints/joint_jerk_constraint.h`

**Purpose:** Bounds on joint jerk (3rd derivative of position). Enables smoother trajectories by limiting rate of acceleration change.

**Formula:** `jerk = pos[i+2] - 3*pos[i+1] + 3*pos[i] - pos[i-1]` (finite difference)

**Constructor:**
```cpp
JointJerkConstraint(
    const Eigen::VectorXd& targets,                              // jerk targets per DOF
    const std::vector<std::shared_ptr<const JointPosition>>& position_vars,  // 4+ consecutive waypoints
    const Eigen::VectorXd& coeffs,                               // weights per DOF
    const std::string& name = "JointJerk"
);
```

**Binding complexity:** LOW - follows exact same pattern as `JointAccelConstraint` (already wrapped).

**Use cases:**
- Industrial robots where servo drives have jerk limits
- Smooth motion for camera/sensor payload
- Reducing mechanical wear

---

## 2. CartLineConstraint

**File:** `trajopt_ifopt/constraints/cartesian_line_constraint.h`

**Purpose:** Constrain TCP to lie on a line segment between two poses. The constraint finds the nearest point on the line and measures error from that point.

**Info struct (`CartLineInfo`):**
```cpp
struct CartLineInfo {
    std::shared_ptr<const tesseract_kinematics::JointGroup> manip;
    std::string source_frame;           // TCP frame
    std::string target_frame;           // reference frame
    Eigen::Isometry3d source_frame_offset;   // TCP offset
    Eigen::Isometry3d target_frame_offset1;  // line start pose
    Eigen::Isometry3d target_frame_offset2;  // line end pose
    Eigen::VectorXi indices;            // which DOF to constrain (default: all 6)
};
```

**Constructor:**
```cpp
CartLineConstraint(
    CartLineInfo info,
    std::shared_ptr<const JointPosition> position_var,
    const Eigen::VectorXd& coeffs,
    const std::string& name = "CartLine"
);
```

**Key method:**
```cpp
// Finds nearest point on line (uses SLERP for orientation)
Eigen::Isometry3d GetLinePoint(
    const Eigen::Isometry3d& source_tf,
    const Eigen::Isometry3d& target_tf1,
    const Eigen::Isometry3d& target_tf2
) const;
```

**Binding complexity:** MEDIUM - requires new `CartLineInfo` struct binding.

**Use cases:**
- Approach/retract motions along a line
- Welding/milling along linear segments
- Assembly insertion with linear compliance

**Note:** Uses numeric differentiation by default (`use_numeric_differentiation = true`).

---

## 3. DiscreteCollisionNumericalConstraint

**File:** `trajopt_ifopt/constraints/collision/discrete_collision_numerical_constraint.h`

**Purpose:** Same as `DiscreteCollisionConstraint` but uses numerical differentiation for jacobians instead of analytical.

**Constructor:**
```cpp
DiscreteCollisionNumericalConstraint(
    std::shared_ptr<DiscreteCollisionEvaluator> collision_evaluator,
    std::shared_ptr<const JointPosition> position_var,
    int max_num_cnt = 1,
    bool fixed_sparsity = false,
    const std::string& name = "DiscreteCollisionNumerical"
);
```

**Binding complexity:** LOW - identical signature to `DiscreteCollisionConstraint`.

**Use cases:**
- Debugging when analytical jacobians produce issues
- Validating analytical collision jacobians
- Fallback when analytical fails

**Note:** The analytical version (`DiscreteCollisionConstraint`) is generally preferred for performance.

---

## 4. InverseKinematicsConstraint

**File:** `trajopt_ifopt/constraints/inverse_kinematics_constraint.h`

**Purpose:** Constrains joints to stay within bounds of an IK solution. Uses a seed variable to compute IK, then constrains deviation from that solution.

**Info struct (`InverseKinematicsInfo`):**
```cpp
struct InverseKinematicsInfo {
    std::shared_ptr<const tesseract_kinematics::KinematicGroup> manip;  // NOTE: KinematicGroup, not JointGroup
    std::string working_frame;   // (not currently respected)
    std::string tcp_frame;       // (not currently respected)
    Eigen::Isometry3d tcp_offset;  // (not currently respected)
};
```

**Constructor:**
```cpp
InverseKinematicsConstraint(
    const Eigen::Isometry3d& target_pose,
    InverseKinematicsInfo::ConstPtr kinematic_info,
    std::shared_ptr<const JointPosition> constraint_var,  // variable being constrained
    std::shared_ptr<const JointPosition> seed_var,        // seed for IK (usually adjacent waypoint)
    const std::string& name = "InverseKinematics"
);
```

**Binding complexity:** HIGH
- Requires `KinematicGroup` (vs `JointGroup` used elsewhere) - may need additional bindings
- Takes two `JointPosition` variables with different roles
- Several members marked "not currently respected" - API may be unstable

**Use cases:**
- Maintaining IK consistency across trajectory
- Preventing large joint jumps between waypoints
- Descartes-style redundancy resolution (see TODO in header)

**Note:** Header comments suggest integration with descartes_light samplers for z-free constraints.

---

## Existing Bindings Reference

Already wrapped in `trajopt_ifopt_bindings.cpp`:

| Constraint | Lines | Notes |
|-----------|-------|-------|
| `JointPosition` (variable) | 48-65 | Variable set, not constraint |
| `CartPosInfo` | 83-92 | Info struct |
| `CartPosConstraint` | 95-110 | Cartesian pose |
| `JointPosConstraint` | 113-119 | Joint position |
| `JointVelConstraint` | 122-128 | Joint velocity |
| `JointAccelConstraint` | 131-137 | Joint acceleration |
| `DiscreteCollisionConstraint` | 175-184 | Single-state collision |
| `ContinuousCollisionConstraint` | 216-232 | Two-state swept collision |

---

## Recommendations

1. **JointJerkConstraint** - Wrap first. Simple pattern match with `JointAccelConstraint`. High value for industrial users.

2. **CartLineConstraint** - Wrap second. Useful for linear tool paths. Requires `CartLineInfo` struct.

3. **DiscreteCollisionNumericalConstraint** - Optional. Only useful for debugging/validation.

4. **InverseKinematicsConstraint** - Defer. Complex dependencies, unclear API stability.

---

## Implementation Notes

From existing bindings:

1. **Eigen default args cause `std::bad_cast`** - Use default constructor + member assignment (see `CartPosInfo` pattern at line 83).

2. **`std::array` args need lambda wrapper** - See `ContinuousCollisionConstraint` at line 217.

3. **Cross-module inheritance** - Import parent module first (line 44).
