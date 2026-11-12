# API changes: 0.35 → 0.36

<!-- snippet: ambient
from pathlib import Path

import tesseract_robotics
from tesseract_robotics.collision import ContactResult, ContactResultMap
from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.common import Isometry3d, JointState
from tesseract_robotics.geometry import Box
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link
from tesseract_robotics.task_composer import (
    AnyPoly_wrap_CompositeInstruction, AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary, TaskComposerContext, TaskComposerDataStorage,
    TaskComposerGraph, TaskComposerLog, TaskComposerPluginFactory, TaskComposerPortMap,
)
from tests.docs.doc_assets import (
    emitter_scene, enter_scratch_dir, ifopt_contact_scene, joint_program,
)

enter_scratch_dir()
globals().update(ifopt_contact_scene())
globals().update(emitter_scene(robot))

# the state solver, a joint state, a contact map and the scene graph of the IRB2400
solver = env.getStateSolver()
values = np.zeros(6)
state = JointState(["joint_1"], np.zeros(1))
contacts = ContactResultMap()
result = ContactResult()
graph = env.getSceneGraph()

# the task composer: two pipeline nodes, an executor and storage holding a program to plan
factory = TaskComposerPluginFactory(tesseract_robotics.get_task_composer_config_path(), locator)
node = factory.createTaskComposerNode("FreespacePipeline")
first = factory.createTaskComposerNode("FreespacePipeline")
second = factory.createTaskComposerNode("FreespacePipeline")
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
storage = TaskComposerDataStorage()
for _port in ("planning_input", "k"):
    storage.setData(_port, AnyPoly_wrap_CompositeInstruction(joint_program(robot)))
storage.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(ProfileDictionary()))
overrides = TaskComposerPortMap()
overrides.set("planning_input", "k")

# a log whose data storage holds the planners' default profiles
profiles = create_trajopt_default_profiles()
log = TaskComposerLog("failing plan")
log.initial_data = TaskComposerDataStorage("profiles only")
log.context = TaskComposerContext("ctx")

records = []
events = []
on_event = events.append
x, y, z, w = 0.0, 0.0, 0.0, 1.0
pose = Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.0, -1.0, 0.0, 0.0)
link = Link("obstacle")
_collision = Collision()
_collision.geometry = Box(0.1, 0.1, 0.1)
link.addCollision(_collision)
joint = Joint("obstacle_joint")
joint.type = JointType.FIXED
joint.parent_link_id = "base_link"
joint.child_link_id = "obstacle"

# the planning example's entry point, failing as an example does when planning fails
from tesseract_robotics.examples.failures import PlanningFailedError


def main():
    raise PlanningFailedError("planning failed")


manager = env.getDiscreteContactManager()
objects = [("box", [Box(0.1, 0.1, 0.1)], [Isometry3d.Identity()])]
-->

!!! info "0.36.0.0rc1"
    `0.36.0.0rc1` is a pre-release (`pip install --pre tesseract-robotics-nanobind`). Its wheels
    build tesseract, trajopt and tesseract_planning from `nanobind-0.36` branches on the
    `jf---` forks: upstream `master` plus open upstream pull requests, so the release does not
    wait on upstream review.

    | repository | upstream master | carried PRs |
    | --- | --- | --- |
    | tesseract | `a9c22066` | #1396 |
    | tesseract_planning | `a6483823` | #789, #792, #794 |
    | trajopt | `963baf7d` | #628, #629 |

    `tesseract_robotics.build_identity.UPSTREAM_SOURCES` lists the exact commits. A PR that
    merges upstream drops out of the table; the final 0.36.0.x builds from the upstream tag.
    #792 makes `isValidState` bindable (#333); #629 and #794 use tesseract's
    `PerThreadContactManager` (tesseract#1394).

Each section below names what was removed or renamed and shows the 0.35 call next to
its 0.36 replacement.

!!! note "Module names"
    The Python modules lost their `tesseract_` prefix: `tesseract_robotics.tesseract_collision` is
    now `tesseract_robotics.collision`. The old names are gone (no aliases). The 0.35 snippets below keep the names 0.35 used. Full table:
    [0.36 module names](breaking-changes.md#036-module-names).

## Links and joints: names → ids

Upstream now addresses links and joints by a typed id derived from the name
(upstream `IDENTITY_DESIGN.md`, full rename table in `IDENTITY_MIGRATION.md`). The
bindings mirror it: `LinkId`, `JointId` and `LinkIdPair` in `common`. A
`str` converts to an id implicitly, so most call sites that pass names keep working;
what changes is what getters return and what they are called.

```python
env.getLinkIds()            # was getLinkNames(); returns list[LinkId]
str(env.getRootLinkId())    # the name; LinkId("tool0") == "tool0" is True
transforms = env.getState().link_transforms   # dict[str, Isometry3d], as in 0.35
transforms["tool0"]         # maps stay keyed by name
```

| 0.35 | 0.36 |
|---|---|
| `get*Names()` (links, joints, active, static, tip) | `get*Ids()` |
| `getBaseLinkName()` / `getRootLinkName()` | `getBaseLinkId()` / `getRootLinkId()` |
| `getGroupJointNames(group)` | `getGroupJointIds(group)` |
| `ContactResult.link_names` | `ContactResult.link_ids` |
| `Joint.parent_link_name` / `child_link_name` | `Joint.parent_link_id` / `child_link_id` |
| `JointMimic.joint_name` | `JointMimic.joint_id` |
| `JointState.joint_names` | `JointState.joint_ids` |
| `KinGroupIKInput.tip_link_name` | `KinGroupIKInput.tip_link_id` |
| command `getLinkName()` / `getJointName()` | `getLinkId()` / `getJointId()` |
| waypoint `getNames()` / `setNames()` | `getJointIds()` / `setJointIds()` |
| `acm.isCollisionAllowed("a", "b")` | `acm.isCollisionAllowed(("a", "b"))` (one `LinkIdPair` or tuple) |
| `CollisionCoeffData.getCollisionCoeff("a", "b")` | `getCollisionCoeff(("a", "b"))` |

`LinkId` and `JointId` do not convert into each other. To get a plain name back, call
`.name()` or `str()`. An id is not a `str`: `"link_" + lid`, `json.dumps` and format
specs need `.name()` first.

A `LinkIdPair` is unordered, so it never compares equal to a tuple (a tuple's hash
depends on order). Methods taking a pair accept `("a", "b")`; index a pair-keyed dict,
such as `acm.getAllAllowedCollisions()`, by `LinkIdPair("a", "b")`.

The high-level `tesseract_robotics.planning` API keeps `str` in its signatures:
`Robot.get_joint_names()`, `get_link_names()`, `RobotState.joint_names` and
`TrajectoryPoint.joint_names` still return names.

### Ids hash and compare like their name

`LinkId("tool0") == "tool0"` and `hash(LinkId("tool0")) == hash("tool0")`, so an id
and its name find the same `dict` or `set` entry in either direction. Code that only
compares, looks up or prints keeps working; code that concatenates, serialises or
formats needs `.name()`.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
names = env.getLinkNames()                     # list[str]
parent = env.getJoint("joint_1").parent_link_name
label = "link_" + names[0]
```

```python
# 0.36
ids = env.getLinkIds()                         # list[LinkId]
parent = env.getJoint("joint_1").parent_link_id
label = "link_" + ids[0].name()                # a LinkId is not a str
assert "base_link" in ids                      # a str equals its id
```

### Maps keyed by a link or joint stay keyed by name

Every map keyed by a link or joint comes back as a plain `dict[str, V]`, as in 0.35:
`calcFwdKin`, `SceneState.joints` / `floating_joints` / `link_transforms` /
`joint_transforms`, `getLinkTransforms(ids, values)`, `getCurrentFloatingJointValues`,
`getFloatingJointValues`, the joint-limit commands' `getLimits`,
`ContactManagerConfig.modify_object_enabled`, `KinematicsInformation.group_states` /
`group_tcps`, `CalibrationInfo.joints` and the keys of `SceneGraph.getAdjacencyMap`.
`transforms["tool0"]` is exactly typed. A map you pass in may be keyed by names or ids.
Maps keyed by a `LinkIdPair` (`getAllAllowedCollisions`, `ContactResultMap`) keep the pair:
a tuple key would be ordered.

```python
# 0.36
transforms = env.getState().link_transforms    # dict[str, Isometry3d]
tool0 = transforms["tool0"]
assert all(type(name) is str for name in transforms)
state_copy = env.getState()
state_copy.link_transforms = transforms        # names or ids both convert back
```

### State solver: ids, and no `{name: value}` Jacobian

`StateSolver` follows the same rename: `getJointIds()`, `getActiveJointIds()`,
`getFloatingJointIds()`, `getBaseLinkId()`, `getLinkIds()`, `getActiveLinkIds()`,
`getStaticLinkIds()`, `isActiveLinkId(link_id)`, `hasLinkId(link_id)`. Keyword arguments
follow: `joint_names=` → `joint_ids=`, `link_name=` → `link_id=`. `SceneState.joints`,
`link_transforms` and `joint_transforms` stay keyed by name (see below).

`getJacobian` lost its `{name: value}` overload; upstream 0.36 has none. Pass the
values in solver order, or the ids next to the values.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
jac = solver.getJacobian({"joint_1": 0.0, "joint_2": 0.5}, "tool0")
```

```python
# 0.36
jac = solver.getJacobian(["joint_1", "joint_2"], [0.0, 0.5], "tool0")
jac = solver.getJacobian(values, "tool0")      # values in getJointIds() order
```

### Link pairs: `LinkIdPair`

`makeOrderedLinkPair` and `UnorderedLinkPairError` are removed. A `LinkIdPair` is
unordered by construction, so `LinkIdPair("b", "a") == LinkIdPair("a", "b")` and no
unordered key is left to reject. `orderedNameView()` returns the two names sorted, as
a `tuple[str, str]`, for output or for a tuple-keyed structure of your own.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
from tesseract_robotics.tesseract_common import makeOrderedLinkPair
key = makeOrderedLinkPair("b", "a")            # ("a", "b")
contacts.addContactResult(key, result)         # ("b", "a") raised UnorderedLinkPairError
```

```python
# 0.36
from tesseract_robotics.common import LinkIdPair
key = LinkIdPair("b", "a")                     # the same pair as LinkIdPair("a", "b")
contacts.addContactResult(key, result)         # ("b", "a") works too
key.orderedNameView()                          # ("a", "b")
```

Readers that took two names take one pair, as a `LinkIdPair` or a `(str, str)` tuple;
writers keep their two-id form.

| 0.35 | 0.36 |
|---|---|
| `acm.isCollisionAllowed("a", "b")` | `acm.isCollisionAllowed(("a", "b"))` |
| `CollisionMarginPairData.getCollisionMargin("a", "b")` | `getCollisionMargin(("a", "b"))` |
| `CollisionCoeffData.getCollisionCoeff("a", "b")` | `getCollisionCoeff(("a", "b"))` |
| `acm.addAllowedCollision("a", "b", reason)` | unchanged (also takes a pair) |
| `CollisionMarginPairData.setCollisionMargin("a", "b", m)` | unchanged |

`CollisionMarginData.getCollisionMargin` takes either form.
`CollisionCoeffData.getPairsWithZeroCoeff()` returns a `set[LinkIdPair]`.

### Environment commands take ids

The command constructors take a `JointId` / `LinkId` (or a `str`), as the 0.36
headers do, and their keyword arguments are renamed to match: `joint_name=` →
`joint_id=`, `link_name=` → `link_id=`. `MoveJointCommand` keeps `parent_link=`,
`ChangeLinkVisibilityCommand` keeps `visible=`.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
RemoveLinkCommand(link_name="tool0")
MoveJointCommand(joint_name="joint_6", parent_link="base_link")
```

```python
# 0.36
from tesseract_robotics.common import LinkId
from tesseract_robotics.environment import MoveJointCommand, RemoveLinkCommand

RemoveLinkCommand(link_id="tool0")             # or RemoveLinkCommand(LinkId("tool0"))
MoveJointCommand(joint_id="joint_6", parent_link="base_link")
```

The getters were renamed with them: `getLinkName()` → `getLinkId()`,
`getJointName()` → `getJointId()`.

### Id containers read as copies

A list or dict of ids read from a data member is a copy: `JointState.joint_ids`,
`SceneState.joints` / `floating_joints`, `CalibrationInfo.joints`,
`ContactTrajectoryResults.joint_ids`, `ContactManagerConfig.modify_object_enabled`,
`ShortestPath.links` / `joints` / `active_joints`, and
`KinematicsInformation.chain_groups` / `joint_groups` / `link_groups` / `group_states`.
Editing the copy in place does not change the owner; assign the field instead.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
state.joint_names.append("joint_7")
```

```python
# 0.36
ids = state.joint_ids                          # a copy
ids.append("joint_7")
state.joint_ids = ids
```

Before the copy, an id read from such a field pointed into C++ storage and dangled
once the owner replaced or cleared the field.

### Unordered results

Some 0.36 results come from upstream `unordered_set` / `unordered_map` containers and
have no defined order: `SceneGraph.getJointChildrenIds(list)` (the single-id and list
overloads can return the same links in a different order), iteration over a
`ContactResultMap` (yields `(LinkIdPair, ContactResultVector)`), its `getContainer()`,
and `getPairsWithZeroCoeff()`. Compare them as sets, or sort them first.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
assert graph.getJointChildrenNames(["joint_1"]) == ["link_1", "link_2", "link_3"]
```

```python
# 0.36
assert sorted(graph.getJointChildrenIds(["joint_1"])) == [
    "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0",
]
pairs = {pair.orderedNameView() for pair, _ in contacts}
```

## Geometry: `SDFMesh` → `SignedDistanceField`

`SDFMesh`, `createSDFMeshFromPath`, `createSDFMeshFromResource` and
`createSDFMeshFromBytes` are gone upstream. `SignedDistanceField` is a different thing,
not `SDFMesh` renamed: `SDFMesh` was a triangle mesh flagged for signed-distance
collision, `SignedDistanceField` a volumetric grid of signed distances (see
[Collision](user-guide/collision.md)). Build one from a distance function, a
precomputed grid or OpenVDB / NanoVDB bytes.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
from tesseract_robotics.tesseract_geometry import createSDFMeshFromPath
geom = createSDFMeshFromPath("part.stl")
```

```python
# 0.36: a field, sampled from a distance function (negative inside)
from tesseract_robotics.geometry import createDiscreteSignedDistanceField
geom = createDiscreteSignedDistanceField(
    lambda p: float(np.linalg.norm(p)) - 0.1,  # a 10 cm sphere
    [-0.2, -0.2, -0.2], [0.2, 0.2, 0.2], [32, 32, 32],
)
# or: readSignedDistanceFieldVDB(Path("part.vdb").read_bytes())
```

A mesh that needs no distance field stays a `createMeshFromPath` /
`createConvexMeshFromPath` geometry.

## Task composer: `TaskComposerKeys` → `TaskComposerPortMap`

Node ports map to storage keys through `TaskComposerPortMap` (upstream #760):

| 0.35 | 0.36 |
|---|---|
| `node.getInputKeys()` / `getOutputKeys()` | `node.getInputPortMappings()` / `getOutputPortMappings()` |
| `keys.get("program")` | `ports.single("program")` (or `.multiple(...)`, `.at(...)`) |
| `keys.has("program")` | `ports.contains("program")` |
| `node.setInputKeys(...)` / `setOutputKeys(...)` | `node.setPortMappings(inputs, outputs)` |
| `node.getPorts()` | `node.getPortContract()` (a copy; `None` for a graph or pipeline) |
| `node.validatePorts()` | `node.getPortContract().validateOrThrow(inputs, outputs, name)` |
| `TaskComposerNodePorts.input_required` / `input_optional` / … | `inputs()` / `outputs()` (port → `PortDefinition`), `addRequiredInput(...)` / `addOptionalInput(...)` / … |
| `graph.setOverrideInputKeys(keys)` / `setOverrideOutputKeys(keys)` | `graph.setOverrideInputPortMappings(ports)` / `setOverrideOutputPortMappings(ports)` |
| `TaskComposerNodeInfo.input_keys` / `output_keys` | `input_port_mappings` / `output_port_mappings` |

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
keys = node.getInputKeys()
program_key = keys.get("program")
node.validatePorts()
```

```python
# 0.36
ports = node.getInputPortMappings()
program_key = ports.single("planning_input")
contract = node.getPortContract()              # None for a graph or pipeline
if contract is not None:
    contract.validateOrThrow(
        node.getInputPortMappings(), node.getOutputPortMappings(), node.getName()
    )
```

`validateOrThrow` raises `RuntimeError` listing every violation; `validate` returns
them as a list of `TaskComposerNodePorts.ValidationError`.

### Graphs need boundary port mappings

A 0.36 graph has no fixed port contract: its ports are whatever its boundary mappings
declare. A graph built in Python is valid, and can run, only once `setPortMappings`
supplies its children's inputs and outputs, and an override is accepted only for a
port those mappings declare. Call `setPortMappings` first.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
graph = TaskComposerGraph("g")
a, b = graph.addNode(first), graph.addNode(second)
graph.addEdges(a, [b])
graph.setTerminals([b])
executor.run(graph, storage)
```

```python
# 0.36
graph = TaskComposerGraph("g")
a, b = graph.addNode(first), graph.addNode(second)
graph.addEdges(a, [b])
graph.setTerminals([b])
graph.setPortMappings(first.getInputPortMappings(), second.getOutputPortMappings())
graph.setOverrideInputPortMappings(overrides)  # only after setPortMappings
executor.run(graph, storage)
```

## Logging: console_bridge → spdlog

Upstream logs through spdlog (#1367). The console_bridge API no longer reached
tesseract's output and is removed: `setLogLevel`, `getLogLevel`, `log`,
`useOutputHandler`, `restorePreviousOutputHandler`, `OutputHandler` and the
`CONSOLE_BRIDGE_LOG_*` constants. `LogLevel` is now spdlog's level enum (`trace`,
`debug`, `info`, `warn`, `err`, `critical`, `off`).

| 0.35 | 0.36 |
|---|---|
| `setLogLevel(LogLevel.CONSOLE_BRIDGE_LOG_ERROR)` | `getLogger().set_level(LogLevel.err)` |
| `getLogLevel()` | `getLogger().level()` |
| `useOutputHandler(handler)` / `restorePreviousOutputHandler()` | `addLogRecordHandler(fn)` / `removeLogRecordHandler(id)` |
| `flushOutputHandlers()`, `OutputHandlerFlushForbiddenError` | none needed: handlers run synchronously |

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
from tesseract_robotics.tesseract_common import (
    LogLevel, OutputHandler, setLogLevel, useOutputHandler,
)

class Collect(OutputHandler):
    def log(self, text, level, filename, line):
        records.append(text)

setLogLevel(LogLevel.CONSOLE_BRIDGE_LOG_ERROR)
useOutputHandler(Collect())
```

```python
# 0.36
from tesseract_robotics.common import (
    LogLevel, addLogRecordHandler, getLogger, removeLogRecordHandler,
)

getLogger().set_level(LogLevel.err)
handler_id = addLogRecordHandler(lambda record: records.append(record.message))
...
removeLogRecordHandler(handler_id)
```

A handler runs on the thread that logged the record, often a task-composer or
OpenMP worker, not the thread that registered it. Keep it short and thread-safe
(append to a list or a `queue.Queue`), and don't block in it on work that thread is
supposed to finish.

Record handlers run synchronously on the thread that logs: upstream calls them after the
logger returns, with no tesseract lock held (`logging.cpp` `dispatchHandlers`), so the
console_bridge deadlock (a handler waiting for the GIL under console_bridge's global mutex) has
no 0.36 equivalent, and main's delivery thread, `flushOutputHandlers()` and flush barrier are
gone with it. A handler on an executor worker calls a `TaskComposerServer` like any other
thread and waits for a reload; one on the reloading thread itself raises
`TaskComposerServerBusyError`.

descartes_light 0.36 still logs through console_bridge (`ladder_graph_solver.hpp`,
`bgl_solver.hpp`): its output goes to stderr and never reaches `addLogRecordHandler` until
[swri-robotics/descartes_light#130](https://github.com/swri-robotics/descartes_light/issues/130).

See [common logging](api/common.md#logging).

## trajopt_sqp: constraint violations carry raw and weighted forms

`evaluateConvexConstraintViolations()`, `getExactConstraintViolations()` and
`SQPResults.*_constraint_violations` return a `ConstraintViolations` with `raw`
(unweighted, compared against `cnt_tolerance`) and `weighted` (charged by the merit)
arrays instead of one array.

## trajopt_sqp: `ABSOLUTE` and `HINGE` costs are modelled as charged

The 0.35.0.x wheels (trajopt 0.35.0) still read an `ABSOLUTE` or `HINGE` cost as 0 at every QP solution, leave its coefficient out of the exact cost, and ignore a callback's `False`; the pinned trajopt `a504b22` fixes all three, the first two by trajopt#592 (#151).

## trajopt_ifopt: contact gradients are a free function

The collision evaluators lost `getGradient` (`DiscreteCollisionEvaluator`) and
`calcGradientData` (`ContinuousCollisionEvaluator`): upstream 0.36 moved the gradient
to `trajopt_common::getGradient`, bound as `trajopt_ifopt.getGradient`. It takes the
margin, margin buffer and joint group the evaluator used to supply; the two-timestep
overload replaces `calcGradientData`. `calcCollisions` / `calcCollisionData` also fill
`CollisionCacheData.gradient_results_sets`.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
gradient = evaluator.getGradient(dof_vals, contact)
gradient = lvs_evaluator.calcGradientData(dof_vals0, dof_vals1, contact)
```

```python
# 0.36
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.common import LinkIdPair

pair = LinkIdPair(*contact.link_ids)
margin = evaluator.getCollisionMarginData().getCollisionMargin(pair)
buffer = evaluator.getCollisionMarginBuffer()
group = evaluator.getJointGroup()
gradient = ti.getGradient(dof_vals, contact, margin, buffer, group)
gradient = ti.getGradient(dof_vals0, dof_vals1, contact, margin, buffer, group)
```

## Serialization: a `TaskComposerLog` holds planner profiles

Saving a log whose data storage holds planner profiles raised `RuntimeError` ("Trying to save an
unregistered polymorphic type") on macOS and Windows. On Linux it saved, but a process without that
planner's library in it could not load the result. Now it round-trips on every platform, in any
process: `serialization` links the planner libraries and registers their types, so
importing it loads OMPL, Descartes and TrajOpt too.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35: store an empty dictionary and rebuild the profiles on replay
storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(ProfileDictionary()))
```

```python
# 0.36: the profiles are in the archive
from tesseract_robotics.serialization import task_composer_log_to_file
from tesseract_robotics.task_composer import AnyPoly_wrap_ProfileDictionary

storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))
task_composer_log_to_file(log, "failing_plan.tclx")
```

Building the bindings now requires the Descartes, TrajOpt and TrajOptIfopt components; CMake stops
if one is missing instead of skipping its modules.

## Python surface: constructors, callbacks and failures that raise

The audit of the Python surface removed duplicate spellings and made failures raise a named
exception instead of returning a flag, `None` or a wrong value. No aliases remain; the full
list is in [breaking changes](breaking-changes.md).

### Quaternions are xyzw

The four-scalar constructor was scalar-first, unlike every other quaternion surface.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
q = Quaterniond(w, x, y, z)
```

```python
# 0.36
from tesseract_robotics.common import Quaterniond

q = Quaterniond.from_xyzw(x, y, z, w)          # or Quaterniond([x, y, z, w])
```

### Event callbacks return their key

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
env.addEventCallback(7, EventCallbackFn(on_event))
env.removeEventCallback(hash=7)
```

```python
# 0.36
key = env.addEventCallback(on_event)           # a plain callable
env.removeEventCallback(key)
```

The event a callback receives owns its data, so a callback may keep it.

### Cartesian targets take a pose

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
target = CartesianTarget(position=[0.6, 0.1, 0.8], quaternion=[0, 0, 0, 1])
```

```python
# 0.36
from tesseract_robotics.planning import CartesianTarget, Pose

target = CartesianTarget(Pose.from_xyz_quat(0.6, 0.1, 0.8, 0, 0, 0, 1))
target = CartesianTarget.from_xyz_quat([0.6, 0.1, 0.8], [0, 0, 0, 1])   # same target
```

### `Robot.ik` and the `Robot` mutators raise

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
q = robot.ik("tool0", pose)
if q is None:
    return
if not robot.add_link(link, joint):
    return
```

```python
# 0.36
from tesseract_robotics.planning.errors import IKNoSolutionError

try:
    q = robot.ik("manipulator", pose, "tool0")  # ndarray
except IKNoSolutionError:
    q = None                                   # handle the unreachable pose here
robot.add_link(link, joint)                    # None; EnvironmentCommandError if refused
```

### Serialization and URDF writes

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
if not serialization.environment_to_file(env, "env.xml"):
    ...
blob = serialization.environment_to_binary(env)    # list[int]
```

```python
# 0.36
from pathlib import Path

from tesseract_robotics import serialization

try:
    serialization.environment_to_file(env, Path("env.xml"))
except OSError:
    ...
blob = serialization.environment_to_binary(env)    # bytes
```

### Examples raise `PlanningFailedError`

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
ok = main()          # printed "planning failed", returned a failed result
```

```python
# 0.36
from tesseract_robotics.examples.failures import PlanningFailedError

try:
    main()
except PlanningFailedError as error:
    print(error)
```

### Emitters: frames, postures and pulses

Each move carries its working frame, TCP frame and TCP offset from its `ManipulatorInfo`. The
emitter takes a binding from those names to the controller's data and raises `UnboundFrameError`
for an unbound name. RAPID confdata needs an arm-posture solver and the seed joints; JBI needs
the pulse calibration, since its positions are encoder pulses.

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35
module = emit_rapid(composite, {"weld": RapidProfile(tool="tool0", wobj="wobj0")})
jbi = emit_jbi(composite, {"weld": JbiProfile(tool=1)})
```

```python
# 0.36
from tesseract_robotics.emitters import OpwPostureSolver
from tesseract_robotics.emitters.jbi import JbiPulseCalibration, JbiProfile, emit_jbi, jbi_frames
from tesseract_robotics.emitters.rapid import RapidProfile, emit_rapid, rapid_frames

result = emit_rapid(
    composite,
    {"weld": RapidProfile.build(speed="v100", zone="fine")},
    frames=rapid_frames(work_objects={"base_link": "wobj0"}, tools={"tool0": "tool0"}),
    posture=OpwPostureSolver(opw_parameters),
)
module = result.text                           # an EmittedProgram, with its identity header
result = emit_jbi(
    joint_composite,                           # JBI emits joint moves only
    {"weld": JbiProfile.build()},
    frames=jbi_frames(tools={"tool0": 1}),
    program_name="WELD_01",
    calibration=JbiPulseCalibration.build(pulses_per_rad=pulses_per_rad, zero_pulses=zero_pulses),
)
```

[Program emitters](user-guide/emitters.md) has a complete, tested example per dialect.

## New in 0.36: batch collision updates, sample grids, config schemas

Additions, not changes: 0.35 code keeps working. Each was a 0.36 API with no binding (#373).

| Module | New |
|---|---|
| `collision` | `CollisionObjectSpec`; `addCollisionObjects`, `removeCollisionObjects`, `setCollisionObjectsEnabled`, `getCollisionObjectsTransform`, `setCollisionObjectsTransform(ids, state)` on both managers; `setCollisionObjectsTransform(ids, state0, state1)` on `ContinuousContactManager`; `collideImplicitSDF`, `makeImplicitSDFShape` and their types |
| `kinematics` | `PositionerSampleResolution`, `gatherJointLimits`, `buildSampleGrid`, `computeChainReachUpperBound` |
| `task_composer` | `schema()` on graphs, pipelines, tasks and the Taskflow executor; `TaskComposerNodePorts.inputSchema` / `outputSchema`; `ValidationError.toString` |
| `common` | `PropertyTree` (YAML as text), `PluginDiscoveryInfo` |
| `trajopt_ifopt` | `CartAxisKinematics`; `calcJacobianBlock` on the CartAxis constraints; `CollisionCoeffData.hasZeroCoeff` |
| planners | `==`, `!=` and `from_yaml` on the PIQP solver profiles; `checkPIQPSettings`; `==` / `!=` on `trajopt_sqp.PIQPSettings` (#393) |

<!-- snippet: skip (0.35 API, removed in 0.36) -->
```python
# 0.35: one call, one broadphase update, per object
for name, shapes, poses in objects:
    manager.addCollisionObject(name, 0, shapes, poses)
```

```python
# 0.36: one batch
from tesseract_robotics.collision import CollisionObjectSpec

manager.addCollisionObjects([CollisionObjectSpec(n, 0, s, p) for n, s, p in objects])
```

The CartAxis constraints now check that their `Var` is in the set they are linked to, as the
other `Var`-based constraints do: linking one to a set that does not hold its `Var` raises
`VarNotInVariableSetError`.

Left unbound, with the reason in `scripts/audit_ledger.json`: the REP / ROP / RTP solver plugin
internals (`factory_utils.h`, `rtp_factory.h`, `rtp_inv_kin.h`), reached through
`KinematicsPluginFactory`; `PIQPSolver.solver()`, which returns a `piqp::SparseSolver` no module
binds; and `PropertyTreeBuilder` / `PropertyTree.addValidator`, since schemas are built by the C++
type that owns them.
