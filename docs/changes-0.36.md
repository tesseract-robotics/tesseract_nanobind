# API Changes: 0.35 → 0.36 (upstream main)

!!! warning "Unreleased"
    This page tracks the `upstream-main` branch (issue #141, PR #161), which builds
    against the 0.36 dev packages on the `tesseract-robotics/label/dev` channel:
    `tesseract-robotics` and `tesseract-robotics-planning` `0.36.0.dev20260929`,
    `trajopt` `0.36.0.dev20260930`, built from tesseract `f4cc080`, trajopt `de9e941`
    and tesseract_planning `4efed5f` ahead of the 0.36 release. The `upstream/`
    submodules are ahead of those packages, at tesseract `2b79e91`, trajopt `a504b22`,
    tesseract_planning `a648382`. Released wheels (0.35.0.x) still expose the 0.35 API.

Each section below names what was removed or renamed and shows the 0.35 call next to
its 0.36 replacement.

## Links and joints: names → ids

Upstream now addresses links and joints by a typed id derived from the name
(upstream `IDENTITY_DESIGN.md`, full rename table in `IDENTITY_MIGRATION.md`). The
bindings mirror it: `LinkId`, `JointId` and `LinkIdPair` in `tesseract_common`. A
`str` converts to an id implicitly, so most call sites that pass names keep working;
what changes is what getters return and what they are called.

```python
env.getLinkIds()            # was getLinkNames(); returns list[LinkId]
str(env.getRootLinkId())    # the name; LinkId("tool0") == "tool0" is True
transforms = env.getState().link_transforms   # dict[LinkId, Isometry3d]
transforms["tool0"]         # str keys still find id-keyed entries
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

```python
# 0.35
names = env.getLinkNames()                     # list[str]
parent = env.getJoint("joint_a1").parent_link_name
label = "link_" + names[0]

# 0.36
ids = env.getLinkIds()                         # list[LinkId]
parent = env.getJoint("joint_a1").parent_link_id
label = "link_" + ids[0].name()                # a LinkId is not a str
assert "base_link" in ids                      # a str equals its id
```

### State solver: ids, and no `{name: value}` Jacobian

`StateSolver` follows the same rename: `getJointIds()`, `getActiveJointIds()`,
`getFloatingJointIds()`, `getBaseLinkId()`, `getLinkIds()`, `getActiveLinkIds()`,
`getStaticLinkIds()`, `isActiveLinkId(link_id)`, `hasLinkId(link_id)`. Keyword arguments
follow: `joint_names=` → `joint_ids=`, `link_name=` → `link_id=`. `SceneState.joints`,
`link_transforms` and `joint_transforms` are keyed by id.

`getJacobian` lost its `{name: value}` overload; upstream 0.36 has none. Pass the
values in solver order, or the ids next to the values.

```python
# 0.35
jac = solver.getJacobian({"joint_a1": 0.0, "joint_a2": 0.5}, "tool0")

# 0.36
jac = solver.getJacobian(["joint_a1", "joint_a2"], [0.0, 0.5], "tool0")
jac = solver.getJacobian(values, "tool0")      # values in getJointIds() order
```

### Link pairs: `LinkIdPair`

`makeOrderedLinkPair` and `UnorderedLinkPairError` are removed. A `LinkIdPair` is
unordered by construction, so `LinkIdPair("b", "a") == LinkIdPair("a", "b")` and no
unordered key is left to reject. `orderedNameView()` returns the two names sorted, as
a `tuple[str, str]`, for output or for a tuple-keyed structure of your own.

```python
# 0.35
from tesseract_robotics.tesseract_common import makeOrderedLinkPair
key = makeOrderedLinkPair("b", "a")            # ("a", "b")
contacts.addContactResult(key, result)         # ("b", "a") raised UnorderedLinkPairError

# 0.36
from tesseract_robotics.tesseract_common import LinkIdPair
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

```python
# 0.35
RemoveLinkCommand(link_name="tool0")
MoveJointCommand(joint_name="joint_a6", parent_link="base_link")

# 0.36
RemoveLinkCommand(link_id="tool0")             # or RemoveLinkCommand(LinkId("tool0"))
MoveJointCommand(joint_id="joint_a6", parent_link="base_link")
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

```python
# 0.35
state.joint_names.append("joint_a7")

# 0.36
ids = state.joint_ids                          # a copy
ids.append("joint_a7")
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

```python
# 0.35
assert graph.getJointChildrenNames(["joint_1"]) == ["link_2", "link_3"]

# 0.36
assert sorted(graph.getJointChildrenIds(["joint_1"])) == ["link_2", "link_3"]
pairs = {pair.orderedNameView() for pair, _ in contacts}
```

## Geometry: `SDFMesh` → `SignedDistanceField`

`SDFMesh`, `createSDFMeshFromPath`, `createSDFMeshFromResource` and
`createSDFMeshFromBytes` are gone upstream. `SignedDistanceField` is a different thing,
not `SDFMesh` renamed: `SDFMesh` was a triangle mesh flagged for signed-distance
collision, `SignedDistanceField` a volumetric grid of signed distances (see
[Collision](user-guide/collision.md)). Build one from a distance function, a
precomputed grid or OpenVDB / NanoVDB bytes.

```python
# 0.35
from tesseract_robotics.tesseract_geometry import createSDFMeshFromPath
geom = createSDFMeshFromPath("part.stl")

# 0.36: a field, sampled from a distance function (negative inside)
from tesseract_robotics.tesseract_geometry import createDiscreteSignedDistanceField
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

```python
# 0.35
keys = node.getInputKeys()
program_key = keys.get("program")
node.validatePorts()

# 0.36
ports = node.getInputPortMappings()
program_key = ports.single("program")
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

```python
# 0.35
graph = TaskComposerGraph("g")
a, b = graph.addNode(first), graph.addNode(second)
graph.addEdges(a, [b])
graph.setTerminals([b])
executor.run(graph, storage)

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

# 0.36
from tesseract_robotics.tesseract_common import (
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

See [tesseract_common logging](api/tesseract_common.md#logging).

## trajopt_sqp: constraint violations carry raw and weighted forms

`evaluateConvexConstraintViolations()`, `getExactConstraintViolations()` and
`SQPResults.*_constraint_violations` return a `ConstraintViolations` with `raw`
(unweighted, compared against `cnt_tolerance`) and `weighted` (charged by the merit)
arrays instead of one array.

## trajopt_ifopt: contact gradients are a free function

The collision evaluators lost `getGradient` (`DiscreteCollisionEvaluator`) and
`calcGradientData` (`ContinuousCollisionEvaluator`): upstream 0.36 moved the gradient
to `trajopt_common::getGradient`, bound as `trajopt_ifopt.getGradient`. It takes the
margin, margin buffer and joint group the evaluator used to supply; the two-timestep
overload replaces `calcGradientData`. `calcCollisions` / `calcCollisionData` also fill
`CollisionCacheData.gradient_results_sets`.

```python
# 0.35
gradient = evaluator.getGradient(dof_vals, contact)
gradient = lvs_evaluator.calcGradientData(dof_vals0, dof_vals1, contact)

# 0.36
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import LinkIdPair

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
process: `tesseract_serialization` links the planner libraries and registers their types, so
importing it loads OMPL, Descartes and TrajOpt too.

```python
# 0.35: store an empty dictionary and rebuild the profiles on replay
storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(ProfileDictionary()))

# 0.36: the profiles are in the archive
storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))
task_composer_log_to_file(log, "failing_plan.tclx")
```

Building the bindings now requires the Descartes, TrajOpt and TrajOptIfopt components; CMake stops
if one is missing instead of skipping its modules.
