# Environment Examples

The `Environment` below the high-level `Robot` API: how it is built from
commands, what state it keeps, and the plugin information it loads. Each example
takes its scenario and parameters from upstream source, cited in the module
docstring. Every value printed is also asserted in `tests/examples`.

```bash
# Installed console scripts
tesseract_environment_commands_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.environment_commands_example
```

## Environment Commands

Every change to an `Environment` is a command, and `getCommandHistory()`
returns them in order. The example builds ABB IRB2400 environments from files,
from an SRDF with a calibration, and from another environment's history.

### Resource locators

`GeneralResourceLocator` resolves `package://name/...` through a directory named
`name` that holds a `package.xml`. The default constructor reads the
`TESSERACT_RESOURCE_PATH`, `ROS_PACKAGE_PATH` and `AMENT_PREFIX_PATH` variables;
`paths` and `environment_variables` (keyword-only) choose the sources
explicitly. A locator built with no variables resolves nothing until
`loadEnvironmentVariable` adds one, which returns `False` for an unset variable.
`addPath` registers the packages under a directory: here a temporary package
holding the calibration file used below.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:locators"
```

### The init history

`init(urdf, srdf, locator)` runs upstream's `getInitCommands`
(`environment.cpp:95–137`): `AddSceneGraph`, then, when there is an SRDF,
`AddContactManagersPluginInfo`, `AddKinematicsInformation`, one
`ChangeJointOrigin` per calibrated joint and `ChangeCollisionMargins` if the SRDF
sets margins. `abb_irb2400.srdf` sets neither, so its history is three commands;
without an SRDF it is one.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:init_history"
```

| `init` call | `getCommandHistory()` by `CommandType` |
|---|---|
| `init(Path(urdf), Path(srdf), locator)` | `ADD_SCENE_GRAPH`, `ADD_CONTACT_MANAGERS_PLUGIN_INFO`, `ADD_KINEMATICS_INFORMATION` |
| `init(Path(urdf), locator)` | `ADD_SCENE_GRAPH` |
| `init(scene_graph, srdf)`, SRDF with calibration | the first three, then `CHANGE_JOINT_ORIGIN` ×2 |

### Calibration

The SRDF's `calibration_config` element names a YAML file of joint origins. The
example adds one to the shipped `abb_irb2400.srdf`, pointing at upstream's
unit-test calibration (`tesseract_srdf_unit.cpp:672–690`): `joint_1` at
(1, 2, 3) and `joint_2` at (4, 5, 6), both unrotated. It is a fixture for the
mechanism, not a calibration of a real IRB2400. Each calibrated joint becomes a
`ChangeJointOrigin` command, and the environment's joint origins are exactly the
calibrated ones. The calibration is a hash map, so the order of those commands
is unspecified.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:calibration"
```

`CalibrationInfo.insert` adds the other's joints and overwrites the ones both
have:

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:calibration_info"
```

The same SRDF's `contact_managers_plugin_info` is read from
`contact_manager_plugins.yaml`: the Bullet and FCL factory libraries, three
discrete managers and two continuous ones, and it equals the environment's
`getContactManagersPluginInfo()`.

### Applying commands

A URDF-only environment has no contact-manager plugin information, so it has no
discrete manager (`getDiscreteContactManager()` returns `None`).
`AddContactManagersPluginInfoCommand` adds the information, after which the
default manager is `BulletDiscreteBVHManager`. `applyCommands` then applies the
calibration as one batch. An event callback receives one `CommandAppliedEvent`
per `applyCommand` or `applyCommands` call; its `commands` is the whole history,
and its `revision` the history's length.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:commands"
```

| call | event `revision` | event `commands` |
|---|---|---|
| `applyCommand(AddContactManagersPluginInfoCommand(...))` | 2 | `ADD_SCENE_GRAPH`, `ADD_CONTACT_MANAGERS_PLUGIN_INFO` |
| `applyCommands([ChangeJointOriginCommand, ChangeJointOriginCommand])` | 4 | the above, then `CHANGE_JOINT_ORIGIN` ×2 |

### Replay, clone and equality

`init(commands)` rebuilds an environment from another one's history. The
histories are equal command by command, and so are the revisions and the state,
but the two environments are not equal: `Environment ==` also compares the
time of the last change and of the last state change (`environment.cpp:393–413`),
and the replay was built later. `clone()` copies both timestamps
(`environment.cpp:424–444`), so a clone is equal to its original until a command
is applied to it.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:replay"
```

!!! note "Comparing states"
    `SceneState` has no `==` binding yet ([#232](https://github.com/tesseract-robotics/tesseract_nanobind/issues/232)),
    so `state_a == state_b` compares identity. The example compares joints and
    transforms itself, with the precision of upstream's `SceneState ==`
    (`scene_state.cpp:53–67`).

### Bytes resources

A `BytesResource` is a resource held in memory, built from `bytes` or from a
list of byte values. Both constructors give equal resources:

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:bytes_resource"
```
