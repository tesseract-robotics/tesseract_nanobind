# API Changes: 0.35 → 0.36 (upstream main)

!!! warning "Unreleased"
    This page tracks the `upstream-main` branch, which builds against tesseract,
    trajopt and tesseract_planning `master` ahead of 0.36 (issue #141). Pinned at
    tesseract `f4cc080`, trajopt `de9e941`, tesseract_planning `4efed5f`. Released
    wheels (0.35.0.x) still expose the 0.35 API.

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
`.name()` or `str()`.

The high-level `tesseract_robotics.planning` API keeps `str` in its signatures:
`Robot.get_joint_names()`, `get_link_names()`, `RobotState.joint_names` and
`TrajectoryPoint.joint_names` still return names.

## Geometry: `SDFMesh` → `SignedDistanceField`

`SDFMesh` is gone upstream; `SignedDistanceField` is a real signed distance field
(see [Collision](user-guide/collision.md)).

## Task composer: `TaskComposerKeys` → `TaskComposerPortMap`

Node ports map to storage keys through `TaskComposerPortMap` (upstream #760):

| 0.35 | 0.36 |
|---|---|
| `node.getInputKeys()` / `getOutputKeys()` | `node.getInputPortMappings()` / `getOutputPortMappings()` |
| `keys.get("program")` | `ports.single("program")` (or `.multiple(...)`, `.at(...)`) |
| `keys.has("program")` | `ports.contains("program")` |
| `node.setInputKeys(...)` / `setOutputKeys(...)` | `node.setPortMappings(inputs, outputs)` |

## Logging: console_bridge → spdlog

Upstream logs through spdlog (#1367). `setLogLevel(CONSOLE_BRIDGE_LOG_*)` and
`useOutputHandler` still import but no longer affect tesseract's output. Use:

```python
from tesseract_robotics.tesseract_common import LoggerLevel, getLogger, addLogRecordHandler

getLogger().set_level(LoggerLevel.err)
handler_id = addLogRecordHandler(lambda record: print(record.level, record.message))
```

See [tesseract_common logging](api/tesseract_common.md#logging).

## trajopt_sqp: constraint violations carry raw and weighted forms

`evaluateConvexConstraintViolations()`, `getExactConstraintViolations()` and
`SQPResults.*_constraint_violations` return a `ConstraintViolations` with `raw`
(unweighted, compared against `cnt_tolerance`) and `weighted` (charged by the merit)
arrays instead of one array.
