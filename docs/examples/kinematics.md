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
