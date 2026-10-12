# Environment Examples

The `Environment` below the high-level `Robot` API: how it is built from
commands, what state it keeps, and the plugin information it loads. Each example
takes its scenario and parameters from upstream source, cited in the module
docstring. Every value printed is also asserted in `tests/examples`.

```bash
# Installed console scripts
tesseract_environment_commands_example
tesseract_environment_state_example
tesseract_plugin_configuration_example

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
list of byte values; both constructors give equal resources. Its optional
`parent` locator resolves a relative url next to the resource's own url, here
`package.xml` beside the calibration file. Without a parent nothing resolves,
and `BytesResource ==` counts whether a parent is set, so the two are unequal.

```python title="environment_commands_example.py"
--8<-- "src/tesseract_robotics/examples/environment_commands_example.py:bytes_resource"
```

## Environment State

The IIWA environment's state, with its first joint made FLOATING: every native
`setState` / `getState` / `getLinkTransforms` overload, the joint and link
queries, timestamps, and TCP lookup.

### Floating joints

A `ReplaceJointCommand` turns `joint_a1` into a FLOATING joint at x = 1.25, the
first step of upstream's floating-joint test (`state_solver_test_suite.h:766–820`).
A floating joint has no joint value, so it leaves `getActiveJointNames()`; its
pose is read with `getCurrentFloatingJointValues()`, for all floating joints or
for the names given. The command moves the revision from 3 to 4; the init
revision stays 3.

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:floating_joint"
```

Each `setState` overload then moves the floating joint to the next origin of
upstream's test (y = 1.5, then z = 1.5, then back to the first), starting from
zero joint values and the previous origin, so a call that
changed nothing would be caught. After each one, the joint values and the
floating value read back, and `link_1` sits at `base_link` ∘ origin: a floating
joint adds no motion of its own.

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:set_state"
```

| overload | `joint_a1` origin | joint values afterwards |
|---|---|---|
| `setState(floating)` | (1.25, 1.5, 0) | unchanged (zero) |
| `setState(dict, floating)` | (1.25, 1.5, 1.5) | upstream's `jvals` |
| `setState(names, values, floating)` | (1.25, 0, 0) | upstream's `jvals` |

### Computing a state without setting it

`getState` with arguments computes a state for other joint values and floating
values and leaves the environment's own state as it was. `getLinkTransforms()`
lists the current state in `getLinkNames()` order; with joint names and values
(and optionally floating values) it computes, like `getState`. Each agrees with
`getState` within upstream's precision of 1e-6 (`tesseract_environment_unit.cpp:155–158`).

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:get_state"
```

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:link_transforms"
```

### Queries

`getCurrentJointValues(names)` reads joint values in the order given.
`getActiveLinkNames(joints)` lists the links those joints move, and
`getStaticLinkNames(joints)` the rest. `getJointGroup(name, joints)` builds a
`JointGroup` over any joints, whose forward kinematics match the environment's.
`getJointLimits` returns the URDF limits; the URDF parser sets an unspecified
acceleration to half the velocity and an unspecified jerk to 1000
(`limits.cpp:59, 65`). `getLinkVisibility` and `getLinkCollisionEnabled` follow
their commands.

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:queries"
```

| query | result |
|---|---|
| `getCurrentJointValues(["joint_a3", "joint_a2"])` | [−0.785398, 0.785398] |
| `getActiveLinkNames(["joint_a5"])` | `link_5`, `link_6`, `link_7`, `tool0` |
| `getStaticLinkNames(["joint_a5"])` | `base`, `base_link`, `link_1` … `link_4` |
| `getJointLimits("joint_a2")` | lower −2.0942, upper 2.0942, velocity 1.4834, acceleration 0.7417, jerk 1000 |

!!! note "Timestamps"
    `setState` sets `getTimestamp()` and `getCurrentStateTimestamp()` from one
    `system_clock::now()` (`environment.cpp:651–654`), so the two are equal right
    after it; a command sets only `getTimestamp()`. `system_clock` can step
    backwards, so compare timestamps for equality, not order.

### Tool center points

`findTCPOffset(manip_info)` resolves `manip_info.tcp_offset`
(`environment.cpp:832–862`), as upstream's `EnvFindTCPUnit` test runs it:

1. an `Isometry3d` is returned as is;
2. a name must not be a link (`tool0` raises `RuntimeError`);
3. a name is looked up in the manipulator's `group_tcps` in the SRDF
   (`laser`, `welder`);
4. otherwise each callback registered with `addFindTCPOffsetCallback` is asked
   in turn, and one that raises means "not found here";
5. if none finds it, `RuntimeError`.

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:find_tcp"
```

| `tcp_offset` | result |
|---|---|
| `Isometry3d`, z = 0.1 | that transform |
| `"laser"` (`xyz="1 .1 1" rpy="0 1.57 0"`) | translation (1, 0.1, 1), rotation 1.57 rad about y |
| `"welder"` (`xyz=".1 1 .2" wxyz="1 0 0 0"`) | translation (0.1, 1, 0.2), no rotation |
| `"laser_callback"`, before the callback | `RuntimeError: Could not find tcp by name laser_callback'!` |
| `"laser_callback"`, after the callback | z = 0.1 |
| `"tool0"` (a link) | `RuntimeError: The tcp offset name 'tool0' should not be an existing link …` |
| `"unknown"` | `RuntimeError: Could not find tcp by name unknown'!` |

### Manipulator info

A `ManipulatorInfo` is empty while its manipulator, working frame or TCP frame
is unset. `getCombined(override)` takes every field the override sets, and its
TCP offset with its TCP frame, as upstream's `ManipulatorInfo` unit test shows:

```python title="environment_state_example.py"
--8<-- "src/tesseract_robotics/examples/environment_state_example.py:manipulator_info"
```

## Plugin Configuration

The plugin information tesseract reads from YAML, built in Python: contact
managers, profiles and the task composer. Each info type has `search_paths`,
`search_libraries` and its plugins; `insert` merges one into another, `empty`
and `clear` do what they say.

!!! note "`insert` merges, it does not replace"
    `insert(other)` appends `other`'s search paths and libraries, adds or
    overwrites its plugins by name, and takes its default plugin only when
    `other` names one (`plugin_info.cpp:71–81, 162–184, 214–230`). Inserting the
    same information twice lists its libraries twice.

### Contact managers

The example builds `contact_manager_plugins.yaml` as a Bullet part, with the
defaults, and an FCL part without one, and merges them. The result equals the
information the IIWA SRDF loads from the shipped file. (Wheels strip
`search_paths` from the shipped support YAMLs, so the example compares the
other fields.)

```python title="plugin_configuration_example.py"
--8<-- "src/tesseract_robotics/examples/plugin_configuration_example.py:contact_managers"
```

Applied to a URDF-only environment with `AddContactManagersPluginInfoCommand`,
it makes every manager it names available, each created by name:

```python title="plugin_configuration_example.py"
--8<-- "src/tesseract_robotics/examples/plugin_configuration_example.py:apply"
```

### Profiles and the task composer

There is no shipped profile-plugin YAML; the example uses the values of
upstream's `ProfilePluginInfoUnit` test (`tesseract_common_unit.cpp:1268–1298`).
`plugin_infos` maps a group to its plugins, and `insert` merges into a group
rather than replacing it:

```python title="plugin_configuration_example.py"
--8<-- "src/tesseract_robotics/examples/plugin_configuration_example.py:profiles"
```

`TaskComposerPluginInfo` mirrors `task_composer_plugins.yaml`'s search
libraries, its executor block (`TaskflowExecutor`, 5 threads) and two of its
36 task entries, each a `PipelineTaskFactory`:

```python title="plugin_configuration_example.py"
--8<-- "src/tesseract_robotics/examples/plugin_configuration_example.py:task_composer"
```

!!! warning "Data only, for now"
    Upstream builds a `ProfilePluginFactory` from a `ProfilesPluginInfo` and a
    `TaskComposerPluginFactory` from a `TaskComposerPluginInfo`. Neither
    constructor is bound, so in Python these two types hold data but nothing
    consumes them yet. `TaskComposerPluginFactory` is built from a config file.

### Configured and available task composer plugins

`getTaskComposerNodePlugins()` returns the configured nodes, name → `PluginInfo`:
the names `createTaskComposerNode` accepts. `getAvailableTaskComposerNodePlugins()`
returns the factory classes the search libraries export: the valid values of a
`PluginInfo.class_name`. Every configured class is available, and
`TaskComposer.get_available_pipelines()` lists only configured names, the ones
whose node is created without error.

```python title="plugin_configuration_example.py"
--8<-- "src/tesseract_robotics/examples/plugin_configuration_example.py:factory"
```

| | count (macOS arm64, conda env) |
|---|---|
| configured node plugins | 36, all `PipelineTaskFactory` |
| available node classes | 33 |
| loadable pipelines | 36 |

`get_available_pipelines()` creates every configured pipeline once, so it loads
the planner plugins: the first call in a process took 12 s here.
