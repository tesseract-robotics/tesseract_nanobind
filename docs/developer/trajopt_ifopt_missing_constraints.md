# TrajOpt-Ifopt constraint bindings

All ten constraint classes of trajopt 0.35.0's `trajopt_ifopt` are bound, in
[`src/trajopt_ifopt/trajopt_ifopt_bindings.cpp`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/trajopt_ifopt/trajopt_ifopt_bindings.cpp).
The [`trajopt_ifopt` reference](../api/trajopt_ifopt.md#constraints) has how each one enters a
`TrajOptQPProblem`: as a constraint, or as a cost with the penalty types its row bounds allow.
This page records what the bindings do beyond a one-to-one wrap, and why. That covers the
workarounds for trajopt 0.35.0, tracked for removal in #146, and the patterns to follow when
binding the next class.

## Bound classes

| Class | Python constructors | Binding notes |
|---|---|---|
| `JointPosConstraint` | `(target, position_var, coeffs, name, range_bound_handling)`; `(bounds, position_var, coeffs, name, range_bound_handling)` | The bounds form broadcasts `coeffs` before calling trajopt. In 0.35.0 its range split reads the caller's `coeffs` instead of the broadcast member, so a length-1 `coeffs` is read past its end (fix in the open tesseract-robotics/trajopt#592). |
| `JointVelConstraint`, `JointAccelConstraint`, `JointJerkConstraint` | `(targets, position_vars, coeffs, name)` | Acceleration needs at least four waypoints and jerk six; fewer raise `RuntimeError`. |
| `CartPosConstraint` | `(position_var, manip, source_frame, target_frame, source_frame_offset, target_frame_offset, name, range_bound_handling)`; `(position_var, coeffs, bounds, manip, …)` | The per-axis form: a zero coefficient drops that axis' row. |
| `CartLineConstraint` | `(info: CartLineInfo, position_var, coeffs, name)` | `use_numeric_differentiation` is exposed and defaults to `True`. |
| `DiscreteCollisionConstraint`, `DiscreteCollisionNumericalConstraint` | `(collision_evaluator, position_var, max_num_cnt=1, fixed_sparsity=False, name)` | `max_num_cnt` defaults to trajopt's 1 contact row; tesseract_planning's planner and example pass the collision config's `max_num_cnt` (3 by default). |
| `ContinuousCollisionConstraint` | `(collision_evaluator, position_var0, position_var1, fixed0=False, fixed1=False, max_num_cnt=1, fixed_sparsity=False, name)` | trajopt takes the two variables as a `std::array`; a custom `__init__` builds it. |
| `InverseKinematicsConstraint` | `(target_pose, kinematic_info: InverseKinematicsInfo, constraint_var, seed_var, name)` | `InverseKinematicsInfo` takes a `KinematicGroup`, not a `JointGroup`. trajopt 0.35.0 marks its `working_frame`, `tcp_frame` and `tcp_offset` "Not currently respected". |

The collision evaluators each accept one collision check type, and their constructors raise
otherwise: `SingleTimestepCollisionEvaluator` takes `DISCRETE`, `LVSDiscreteCollisionEvaluator`
`LVS_DISCRETE`, and `LVSContinuousCollisionEvaluator` `LVS_CONTINUOUS` or `CONTINUOUS`.

## Behaviour the bindings add

**`getJacobian()` returns a compressed copy.** The collision constraints (analytic and numerical,
discrete and continuous) assemble their Jacobian with `coeffRef` while a contact is active. That
leaves Eigen's sparse matrix uncompressed, and nanobind's Eigen caster returns only compressed
matrices, so `getJacobian()` raised for exactly the rows a caller needs. The binding compresses a
copy. `tests/trajopt_ifopt/test_constraint_set_jacobian.py` covers a collision set with an active
contact.

**Python `ConstraintSet` subclasses report `getNonZeros() == 0`.** trajopt's unset default is −1,
which a Python subclass could not change. `TrajOptQPProblem` reserves storage from the sum of its
cost sets' hints, so a Python cost set on its own raised `ValueError: vector`. 0 is a valid hint:
storage grows as needed (#146).

**`TrajOptQPProblem` is bound as a non-movable subclass.** This one lives in
`src/trajopt_sqp/trajopt_sqp_bindings.cpp`. trajopt 0.35.0 defaults the move constructor in the
header, where the PIMPL type is incomplete, and nanobind instantiates the move constructor of
every move-constructible type it binds. `TrajOptQPProblemBinding` adds no state and deletes copy
and move; bind `TrajOptQPProblem` directly once trajopt defines the move out of line (#146).

## Binding patterns

1. **No Eigen default arguments.** An Eigen default argument raises `std::bad_cast` in this module,
   so the Python signatures leave Eigen parameters without defaults and callers pass them, for
   example `Isometry3d.Identity()`.
2. **`std::array` parameters get a custom `__init__`.** Take the elements as separate arguments and
   build the array in a lambda, as `ContinuousCollisionConstraint` does.
3. **Import the base module first.** `ConstraintSet` is bound in `_trajopt_ifopt` and used by
   `_trajopt_sqp`, so `tesseract_robotics.trajopt_sqp` imports `trajopt_ifopt` before its own
   extension.
4. **Python subclasses go through the trampoline.** A `ConstraintSet` subclass overrides
   `getValues`, `getBounds`, `getJacobian`, `update` and `getCoefficients`; `getJacobian` returns a
   `scipy.sparse` matrix, which makes scipy a hard runtime dependency.

## History

The 0.34 upgrade bound the four classes the 0.33 bindings lacked:
- `JointJerkConstraint`: smoother trajectories, servo jerk limits.
- `CartLineConstraint`: the tool on a line, for approach and retract or welding along a segment.
- `DiscreteCollisionNumericalConstraint`: checking the analytic collision Jacobian.
- `InverseKinematicsConstraint`: keeping the joints near an IK solution.

This page was that plan's record; the table above replaces its per-class notes.
