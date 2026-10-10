# Kinematics Examples

Kinematics below the high-level `Robot` API: solver plugins, Jacobians and
manipulability, and state solvers. Each example is a port of an upstream
tesseract test or takes its parameters from upstream source, cited in the
module docstring. Every value printed is also asserted in `tests/examples`.

```bash
# Installed console script
tesseract_kinematics_plugins_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.kinematics_plugins_example
```

## Kinematics Plugins

KUKA LBR IIWA 14 R820 forward and inverse kinematics built from the plugin
YAML the SRDF names (`lbr_iiwa_14_r820_plugins.yaml`): one forward solver,
`KDLFwdKinChain`, and two inverse solvers, `KDLInvKinChainLMA` (the default)
and `KDLInvKinChainNR`.

Load the factory from the YAML file and list its solvers:

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:factory"
```

Switch the default inverse solver and solve upstream's `runInvKinIIWATest`
problem with each: the target is `tool0` at q = 0, (0, 0, 1.306) m with the
identity rotation, seeded at ±0.785398 rad.

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:switch_solver"
```

Each solution is checked with the solver's own stopping rule rather than a
guessed tolerance, evaluated on the error twist KDL uses (`diff(FK(q), target)`
in the base frame):

| solver | stopping rule (orocos_kdl 1.5.3) | bound |
|---|---|---|
| `KDLInvKinChainLMA` | task-weighted twist norm, weights (1, 1, 1, 0.1, 0.1, 0.1) | `eps` = 1e-5 |
| `KDLInvKinChainNR` | every twist component | `pos_eps` = 1e-6 |

NR tests the iterate before its final Newton update, so the returned q is one
step further on; the example checks the returned q against the same bound.

Solvers can also be created from a `PluginInfo` directly, without a group
entry in the factory:

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:plugin_info"
```

Add a solver, make it the default and remove it again, as upstream's
`PluginFactorAPIUnit` does. When the default is removed, the first remaining
solver by name becomes the default. Removing a group's last solver raises
`KinematicsPluginRemovalError`.

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:edit_plugins"
```

`saveConfig` writes the factory's current configuration, and a factory built
from that file reports the same `getConfig()`:

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:save_config"
```

A `KinematicGroup` needs no `Environment`, only an inverse solver, a scene
graph and a scene state. `calcInvKin` takes a Python list of `KinGroupIKInput`;
a list never converts to the opaque `KinGroupIKInputs`, so it always selects
the list overload.

```python title="kinematics_plugins_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_plugins_example.py:kinematic_group"
```

## Kinematics Analysis

```bash
tesseract_kinematics_analysis_example
```

A port of upstream's `runJacobianTest` on the IIWA, followed by the analysis
functions of `tesseract/kinematics/utils.h` and the pose-error helpers of
`tesseract/common/utils.h`, each on upstream's own unit-test values.

### Jacobians against finite differences

The `ForwardKinematics` Jacobian, re-based with `jacobianChangeBase` and
`jacobianChangeRefPoint`, against `numericalJacobian`, at upstream's
q = (−0.785398, 0.785398, …) with link points e_k and a `change_base` of
Rz(90°) plus a unit translation:

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:jacobian_fwd_kin"
```

Every `JointGroup.calcJacobian` overload on a `KinematicGroup`, for every link,
against the matching `numericalJacobian` overload: a static base link becomes a
`change_base`, an active one the base-link form of `numericalJacobian`.

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:jacobian_group"
```

`numericalJacobian` is a forward difference with δ = 1e-8, so its error is
truncation plus rounding, and the tolerance adds the two:

| term | value | source |
|---|---|---|
| truncation, δ/2 · max point norm | 1.7e-8 | δ: `kinematics/core/src/utils.cpp:44, 80`; norm ≤ 1.306 + 1 + 1 m |
| rounding, 2 · (8 transforms · 3ε) · 3.306 m / δ | 3.5e-6 | FK chain `joint_a1`…`joint_a7-tool0` |
| `JACOBIAN_TOL` | 3.5e-6 | measured maximum 6.6e-8; upstream's ceiling is 1e-3 |

The form relative to an active base link differences two numerical Jacobians,
so its bound is twice that.

### Twists

Changing the base and reference point of a twist J·q̇ equals changing them on J
and then multiplying by q̇; the residual is rounding only.

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:twist"
```

### Manipulability and singularity

At q = 0 the IIWA is singular: `joint_a1`, `joint_a5` and `joint_a7` all turn
about the base z-axis (every axis `0 0 1`, and the x offsets ∓0.00043624 of
`joint_a2` and `joint_a4` cancel), so their Jacobian columns are identical and
the rank drops to 5. The example checks the columns, the rank and σ_min against
`isNearSingularity`'s default threshold 0.01 (`utils.h:129`), not only the flag.
At a singular configuration the ellipsoid's `measure` and `condition` are
`sys.float_info.max` and its `volume` is 0, as upstream.

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:singularity"
```

### Harmonizing joint angles

Shifting every redundancy-capable joint by 2π and harmonizing gives q back to
within 2.7e-15 rad, not bit-exactly: `q + 2π` and the harmonizer's `+ π`
each round once (`fmod` itself is exact).

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:harmonize"
```

### Validity and limits

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:validity"
```

`KinematicLimits.resize` sizes all four limit arrays; `isWithinLimits` has no
tolerance, and `enforceLimits` returns a clamped copy and leaves its input
alone.

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:limits"
```

### Pose errors and tolerance bands

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:transform_error"
```

`applyTolerances` zeroes the part of an error inside a band and shifts the
rest by the band edge. `calcJacobianTransformErrorDiff` applies it to both the
error and the perturbed error, so a band around both gives a zero row, a band
below both leaves the raw difference, and a band edge between them gives the
clamped difference 0.15 (upstream's `calcJacobianTransformErrorDiff_Toleranced`):

```python title="kinematics_analysis_example.py"
--8<-- "src/tesseract_robotics/examples/kinematics_analysis_example.py:tolerances"
```

## State Solver

```bash
tesseract_state_solver_example
```

A port of upstream's floating-joint and insert-scene-graph state-solver tests
(`state_solver_test_suite.h`, `runSetFloatingJointStateTest` and
`runAddSceneGraphTest`). After every change, a `KDLStateSolver` rebuilt from the
same scene graph is the oracle: KDL treats a FLOATING joint as fixed at its
origin (`kdl_parser.cpp:133–160`), so moving the origin with
`changeJointOrigin` and rebuilding gives an independent computation of the
same link transforms. Link transforms are compared with upstream's
`isApprox(1e-6)`.

`replaceJoint` turns `joint_a1` into a FLOATING joint at x = 1.25 m. It is no
longer an active joint, so `joint_a2`…`joint_a7` remain:

```python title="state_solver_example.py"
--8<-- "src/tesseract_robotics/examples/state_solver_example.py:floating_joint"
```

Move the floating joint with floating values only, then with joints and
floating values together (upstream's y = 1.5 m and z = 1.5 m steps):

```python title="state_solver_example.py"
--8<-- "src/tesseract_robotics/examples/state_solver_example.py:move_floating"
```

!!! note "Reading floating values back"
    Upstream's last step reads `state.floating_joints` from the `SceneState`,
    edits it and passes it back. `SceneState.floating_joints` is not bound in
    Python, so the example builds the `floating_joint_values` dict itself.

Every native `setState`, `getState` and `getLinkTransforms` overload, each with
`floating_joint_values`. Each `setState` starts from a different state, so a
call that changed nothing would disagree with the oracle:

```python title="state_solver_example.py"
--8<-- "src/tesseract_robotics/examples/state_solver_example.py:overloads"
```

The three `getJacobian` overloads give the same matrix, which agrees with
KDL's; the floating joint adds no column.

```python title="state_solver_example.py"
--8<-- "src/tesseract_robotics/examples/state_solver_example.py:jacobian"
```

`insertSceneGraph` attaches upstream's two-link sub-graph (a unit box link and
a link 1.25 m along x, joined by a FIXED joint), first through a FIXED joint,
then again under the prefix `prefix_` through a FLOATING joint. The inserted
root sits at parent ∘ joint origin, its child 1.25 m along x from it, and a
floating value for the attach joint moves the whole sub-graph.

```python title="state_solver_example.py"
--8<-- "src/tesseract_robotics/examples/state_solver_example.py:insert_scene_graph"
```
