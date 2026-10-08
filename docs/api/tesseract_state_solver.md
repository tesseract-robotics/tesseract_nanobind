# tesseract_robotics.tesseract_state_solver

Forward kinematics of a whole scene graph: joint values in, `SceneState` (joint values and
every link and joint transform) out. `OFKTStateSolver` is the mutable solver an `Environment`
uses; `KDLStateSolver` is built on a KDL tree. Both take the `SceneGraph` in their constructor.

```python
import numpy as np
from tesseract_robotics.tesseract_state_solver import OFKTStateSolver

solver = OFKTStateSolver(scene_graph)
names = solver.getActiveJointNames()

solver.setState(np.zeros(len(names)))           # every active joint, in getActiveJointNames() order
solver.setState({names[0]: 0.3})                # by name
solver.setState(names[:1], np.array([0.3]))     # names and values
state = solver.getState()                       # current SceneState
probe = solver.getState({names[0]: 0.5})        # a state for other values; the solver is unchanged
jac = solver.getJacobian({names[0]: 0.5}, "tool0")
transforms = solver.getLinkTransforms(names, np.zeros(len(names)))  # dict[str, Isometry3d]
```

## Overloads and validation

`setState`, `getState` and `getJacobian` take the joint values as a vector over the active
joints, a `dict` by name, or names plus values, each with an optional
`floating_joint_values` (`dict[str, Isometry3d]`). `setState(floating_joint_values)` and
`getState(floating_joint_values)` set only floating joints. A `dict` of floats selects the
joint-value form and a `dict` of `Isometry3d` the floating form; an empty `{}` is the
joint-value form.

Every overload checks its input before calling the solver and raises `ValueError` for a joint
name that is not an active joint, a floating name that is not a floating joint, names and
values of different lengths, or a vector whose length is not the number of active joints. An
unknown link in `getJacobian` raises `KeyError`. Upstream `OFKTStateSolver` dereferences a null
node for an unknown joint name (the GH #43 crash), so this validation is what keeps the process
alive.

`setStateByMap` and `setStateByNamesAndValues` are Python-only aliases of `setState`, bound to
the same functions, so they validate the same way.

!!! note
    `KDLStateSolver` ignores `floating_joint_values`, and `setState(floating_joint_values)` /
    `getState(floating_joint_values)` raise `RuntimeError` (upstream: "not supported").

## Inserting a scene graph

`MutableStateSolver.insertSceneGraph(scene_graph, joint, prefix="")` attaches another graph
with `joint`, whose parent is a link of the solver and whose child is the inserted graph's
root (with `prefix` already applied). It returns `False`, as upstream, when a link is missing
or the joint name exists.

::: tesseract_robotics.tesseract_state_solver
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3
