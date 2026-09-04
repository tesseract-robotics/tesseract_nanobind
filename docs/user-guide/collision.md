# Collision Detection

tesseract_robotics provides discrete and continuous collision checking using FCL and Bullet backends.

## Quick Collision Check

The collision API is driven by the environment's contact manager — there is no one-call shortcut on `Robot`. See [`src/tesseract_robotics/examples/tesseract_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_collision_example.py) for the canonical pattern.

```python
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_ALL,
)
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
joint_names = [f"joint_{i + 1}" for i in range(6)]
joints = np.array([0.5, -0.5, 0.5, 0.0, 0.5, 0.0])

# Update state, then sync the manager with the new link transforms
robot.set_joints(joints, joint_names=joint_names)
state = robot.env.getState()

manager = robot.env.getDiscreteContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
manager.setCollisionObjectsTransform(state.link_transforms)

# contactTest() populates the ContactResultMap passed in (returns None)
contacts = ContactResultMap()
manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))

print(f"Collision-free: {contacts.size() == 0}")

# Flatten to a vector for iteration
results = ContactResultVector()
contacts.flattenMoveResults(results)
for i in range(len(results)):
    r = results[i]
    print(f"{r.link_names[0]} <-> {r.link_names[1]}: distance = {r.distance:.4f} m")
```

## Collision Managers

tesseract supports multiple collision backends:

| Manager | Strengths | Use Case |
|---------|-----------|----------|
| **FCL** | Fast broad-phase | General purpose |
| **Bullet** | Continuous collision | Trajectory validation |
| **BulletCast** | Swept volumes | Time-parameterized paths |

### Discrete vs Continuous

```mermaid
graph LR
    subgraph Discrete
        A[Config A] --> B{Collision?}
    end

    subgraph Continuous
        C[Config A] --> D[Config B]
        D --> E{Swept Volume<br/>Collision?}
    end
```

!!! info "When to Use Continuous"
    - **Discrete**: Fast point checks, obstacle avoidance in planning
    - **Continuous**: Trajectory validation, fast-moving robots, thin obstacles

## Discrete Collision Checking

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactTestType_ALL,
)

# Get discrete manager
manager = robot.env.getDiscreteContactManager()

# Configure request
request = ContactRequest(ContactTestType_ALL)  # or _FIRST, _CLOSEST, _LIMITED
request.calculate_distance = True
request.calculate_penetration = True

# Sync manager with current state
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
manager.setCollisionObjectsTransform(robot.env.getState().link_transforms)
manager.setDefaultCollisionMargin(0.05)  # 5cm margin

# contactTest populates `contacts` in-place
contacts = ContactResultMap()
manager.contactTest(contacts, request)

# Iterate pairs via flattened vector (simplest idiom)
from tesseract_robotics.tesseract_collision import ContactResultVector
results = ContactResultVector()
contacts.flattenMoveResults(results)
for i in range(len(results)):
    r = results[i]
    print(f"{r.link_names[0]} <-> {r.link_names[1]}: {r.distance:.4f}m")
```

## Continuous Collision Checking

Check for collisions along a motion segment. Each active link needs a start and end transform via `setCollisionObjectsTransformCast`:

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactTestType_ALL,
)

# Resolve link transforms at the two endpoints
joint_names = robot.get_joint_names("manipulator")
start_transforms = robot.env.getState(joint_names, start_joints).link_transforms
end_transforms = robot.env.getState(joint_names, end_joints).link_transforms

# Get continuous manager
manager = robot.env.getContinuousContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())

# Set start and end poses per link
for link_name, pose_start in start_transforms.items():
    pose_end = end_transforms[link_name]
    manager.setCollisionObjectsTransformCast(link_name, pose_start, pose_end)

# Check swept volume
contacts = ContactResultMap()
manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))
```

## LVS (Longest Valid Segment)

LVS interpolates between waypoints and checks at discrete points:

```mermaid
graph LR
    A[Start] --> B[...]
    B --> C[...]
    C --> D[...]
    D --> E[End]

    B --> F{Check}
    C --> G{Check}
    D --> H{Check}
```

Used in TrajOpt for efficient continuous collision approximation:

```python
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

# TrajOptCollisionConfig(margin, coeff)
config = TrajOptCollisionConfig(0.025, 20.0)  # 2.5cm margin, coeff=20
config.collision_margin_buffer = 0.005  # Additional buffer beyond margin
config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
config.collision_check_config.longest_valid_segment_length = 0.05  # 5cm interpolation
```

`CollisionEvaluatorType` members (from `tesseract_collision`): `DISCRETE`, `CONTINUOUS`, `LVS_DISCRETE`, `LVS_CONTINUOUS`.

## Contact Margins

Contact margins define the safety buffer around objects. Use `CollisionMarginData` from `tesseract_common`:

```python
from tesseract_robotics.tesseract_common import CollisionMarginData

# Default margin for all pairs (applied to the contact manager)
manager.setCollisionMarginData(CollisionMarginData(0.02))  # 2cm

# Or set the default margin directly
manager.setDefaultCollisionMargin(0.02)

# Per-pair override (link-a, link-b, margin)
manager.setCollisionMarginPair("link_6", "obstacle", 0.05)
```

### Margin Visualization

```
┌──────────────────────────────────────┐
│                                      │
│   ┌─────────┐      ┌─────────┐       │
│   │  Link   │      │Obstacle │       │
│   │         │ 2cm  │         │       │
│   │  ┌───┐  │<---->│  ┌───┐  │       │
│   │  │   │  │margin│  │   │  │       │
│   │  └───┘  │      │  └───┘  │       │
│   └─────────┘      └─────────┘       │
│                                      │
└──────────────────────────────────────┘
```

## Allowed Collision Matrix

Skip collision checks for adjacent or always-safe pairs. The ACM lives on the scene graph and is owned by the environment:

```python
# Fetch the ACM (mutable via scene graph commands on the environment)
acm = robot.env.getAllowedCollisionMatrix()

# Query whether a pair is allowed
if acm.isCollisionAllowed("link_1", "link_2"):
    print("link_1 and link_2 are whitelisted")
```

To add entries, issue a scene graph command through the environment (see `car_seat_example.py` for the canonical pattern):

```python
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix
from tesseract_robotics.tesseract_environment import (
    ModifyAllowedCollisionsCommand, ModifyAllowedCollisionsType,
)

additions = AllowedCollisionMatrix()
additions.addAllowedCollision("gripper", "workpiece", "Attached")
robot.env.applyCommand(
    ModifyAllowedCollisionsCommand(additions, ModifyAllowedCollisionsType.ADD)
)
```

Adjacent links already defined in the SRDF are added automatically when the environment is loaded.

## Performance Optimization

!!! tip "Reduce Active Objects"
    Only check links that can actually collide:

    ```python
    # Skip static links that can't reach each other
    active_links = ["link_4", "link_5", "link_6", "tool0"]
    manager.setActiveCollisionObjects(active_links)
    ```

!!! tip "Use Appropriate Margin"
    Larger margins = slower checks. Use the minimum safe margin:

    - Motion planning: 2-5cm
    - Final validation: 0-1cm
    - Real-time: As small as safe

!!! tip "Choose the Right Test Type"
    ```python
    from tesseract_robotics.tesseract_collision import (
        ContactTestType_FIRST,
        ContactTestType_ALL,
    )

    # Stop at first collision (fastest)
    request = ContactRequest(ContactTestType_FIRST)

    # Get all collisions (for debugging)
    request = ContactRequest(ContactTestType_ALL)
    ```

## Integration with Planning

Planners automatically use collision checking — configure collision margins through planner profiles rather than calling the contact manager yourself:

```python
from tesseract_robotics.planning import (
    MotionProgram, JointTarget, plan_freespace,
)
import numpy as np

# Build a program (start/goal joint states)
program = (
    MotionProgram("manipulator")
    .move_to(JointTarget(np.zeros(6)))
    .move_to(JointTarget(np.array([0.5, -0.5, 0.5, 0.0, 0.5, 0.0])))
)

# OMPL (sampling) — checks collisions at sampled states
# TrajOpt — would use collision cost/constraint via TrajOptCollisionConfig
# Descartes (plan_cartesian) — expects Cartesian targets, not joint targets
result = plan_freespace(robot, program)
```

Collision margins for TrajOpt pipelines live in `TrajOptCollisionConfig` (see LVS section above) and are applied via composite profiles — see [`src/tesseract_robotics/planning/profiles.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/planning/profiles.py) for the stock profile builder.

## Collision Geometry Types

| Type | Performance | Accuracy |
|------|-------------|----------|
| **Sphere** | Fastest | Low |
| **Box** | Fast | Medium |
| **Cylinder** | Fast | Medium |
| **Capsule** | Fast | Medium |
| **Mesh** | Slow | High |
| **ConvexMesh** | Medium | High |
| **Octree** | Medium | Voxel-discrete |
| **SignedDistanceField** | Medium | Grid-discrete (concave OK) — discrete managers only |

!!! tip "Use Convex Decomposition"
    For complex meshes, use `makeConvexMesh` to build a convex hull:

    ```python
    from tesseract_robotics.tesseract_collision import makeConvexMesh

    convex = makeConvexMesh(mesh)
    ```

    To load a convex mesh directly from a file, use the planning helper:

    ```python
    from tesseract_robotics.planning import convex_mesh_from_file

    convex = convex_mesh_from_file("/path/to/mesh.stl")
    ```

## Point Clouds as Collision Obstacles

Sensor-derived obstacles — depth-camera scans, LiDAR, fused mapping output —
go through a `tesseract_geometry.Octree`. Each occupied voxel becomes a
sub-shape the discrete contact manager checks against the robot, so
planners and collision queries treat the cloud just like any other
geometry.

**1. Build the octree and attach it as a link.** 

Load your points into a `PointCloud`, pass it through `createOctree`, and wrap
the result in `tesseract_geometry.Octree`. Then hand the geometry to
`create_obstacle()`.

```python
import numpy as np
from tesseract_robotics.planning import Pose, create_obstacle
from tesseract_robotics.tesseract_geometry import (
    Octree, OctreeSubType, PointCloud, createOctree,
)

# verts: (N, 3) float64 ndarray — pulled from your loader of choice
pc = PointCloud()
for x, y, z in verts:
    pc.addPoint(float(x), float(y), float(z))

# resolution = leaf edge length in metres; smaller = finer = more sub-shapes
octree = Octree(createOctree(pc, resolution=0.02, prune=True, binary=True),
                OctreeSubType.BOX, pruned=True, binary_octree=True)

create_obstacle(
    robot,
    name="scan",
    geometry=octree,
    transform=Pose.from_xyz(1.3, 0.3, 0.0),
)
```

`OctreeSubType` picks the per-leaf primitive — `BOX` is the cheap default,
`SPHERE_INSIDE`/`SPHERE_OUTSIDE` use inscribed/circumscribed spheres if you
need rounder swept volumes.

**2. Margins.** The discrete contact manager applies a margin to every pair
(see [Contact Margins](#contact-margins)). For an octree obstacle the
defaults work fine — set a single value on the manager:

```python
manager = robot.env.getDiscreteContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
manager.setCollisionMarginData(CollisionMarginData(0.02))  # 2 cm safety buffer
```

TrajOpt-based planners route around the cloud out of the box —
`create_trajopt_default_profiles()` enables both the collision cost
(`coeff=50`) and the collision constraint (`coeff=10`) against every
geometry in the env, including the octree. No octree-specific tuning
needed.

!!! tip "Choosing a resolution"
    The leaf size is the dominant cost knob — halving it roughly 8×s the
    sub-shape count. A 1–2 cm leaf is a typical starting point for
    depth-camera scans of a tabletop workspace; coarsen aggressively for
    full-room scans. At sub-centimetre resolutions TrajOpt can saturate
    its iteration budget; coarsen until the planner converges.

For a full collision-aware vs. collision-disabled comparison with a viewer
walkthrough, see the
[Point cloud → Octree](../examples/basic.md#point-cloud-octree) example.

## Signed Distance Fields as Collision Obstacles

A `tesseract_geometry.SignedDistanceField` stores the distance to a surface —
negative inside, positive outside — sampled on a regular grid, rather than
storing the surface itself. Two reasons to reach for one over a mesh or an
octree:

- **Concave shapes work.** A `ConvexMesh` cannot represent a pocket or a bore,
  and the full `Mesh` path that can is the slowest option. An SDF handles
  concavity natively.
- **It is already your native format.** Fused mapping output (nvblox, VDB
  ESDFs, TSDF integration) *is* a distance field. Feeding it in directly avoids
  meshing it first and then throwing the distance information away.

!!! warning "Requires a discrete-only environment"
    An SDF is concave, so continuous (cast) managers reject it — and an
    `Environment` populates *every* manager its contact-manager plugin config
    declares. Against the stock config, which declares a cast manager, merely
    adding an SDF link fails with `RuntimeError: I can only collision check
    convex shapes and compound shapes made of convex shapes`.

    The fix is configuration, not code: point the SRDF at a
    `contact_manager_plugins.yaml` with no `continuous_plugins` block (step 2
    below). Such an environment then behaves normally for SDFs —
    `create_obstacle()`, contact tests, margins — at the cost of having no
    continuous/cast collision checking at all.

The snippets below are loaded directly from
[`sdf_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/sdf_collision_example.py),
so they are the code that actually runs — see [Runnable example](#runnable-example)
at the end of the section.

**1. Build the field.**

Write the distance function and sample it onto a grid with
`createDiscreteSignedDistanceField`. Pass `batched=True` to receive all `(N, 3)`
sample points in one call and return `N` distances — one Python call instead of
`dimensions.prod()`, and the natural shape for a numpy or GPU evaluator. The
obstacle here is a ball with a cylindrical bore — concave, so no convex hull
applies:

```python title="sdf_collision_example.py"
--8<-- "src/tesseract_robotics/examples/sdf_collision_example.py:build_field"
```

`dimensions` is the sample count per axis, so the voxel size is
`(domain_max - domain_min) / (dimensions - 1)`. The sampling happens once,
during this call: the returned geometry holds no reference back to your
callable, so nothing takes the GIL during collision checking.

A field baked offline loads the same way — `readSignedDistanceFieldVDB`
(OpenVDB) and `readSignedDistanceFieldNVDB` (NanoVDB) take the raw file bytes:

```python
from pathlib import Path
from tesseract_robotics.tesseract_geometry import readSignedDistanceFieldVDB

field = readSignedDistanceFieldVDB(Path("scan.vdb").read_bytes())
```

**2. Configure the environment with discrete plugins only.**

Write a `contact_manager_plugins.yaml` that omits the `continuous_plugins`
block. `hasContinuousContactManagerPlugins()` is then false, the `Environment`
never builds a cast manager, and concave geometry stops being rejected:

```yaml title="contact_manager_plugins.yaml"
--8<-- "src/tesseract_robotics/examples/sdf_collision_example.py:contact_manager_yaml"
```

Point the SRDF at it, exactly as the stock scenes do:

```xml
<contact_managers_plugin_config filename="package://my_cell/config/contact_manager_plugins.yaml"/>
```

**3. Attach the field.**

Load the robot with that SRDF and add the field. `create_obstacle()` works here
too, but it always attaches a `Visual` as well — and a field has no surface, so
the viewer logs an "unsupported visual geometry" warning for it. Splitting the
components keeps the field collision-only, which is what it actually is:

```python title="sdf_collision_example.py"
--8<-- "src/tesseract_robotics/examples/sdf_collision_example.py:attach"
```

**4. Check contacts.**

From here nothing is SDF-specific — same manager, same margin, same
`contactTest`:

```python title="sdf_collision_example.py"
--8<-- "src/tesseract_robotics/examples/sdf_collision_example.py:sweep"
```

SDFs honour the margin band like any other geometry: contacts are reported at
positive distance up to the margin, which is what TrajOpt's collision cost needs
to push links apart *before* they touch. Sweeping a 0.05 m probe along `+x`
against the field above gives `+0.09`, `+0.05`, `+0.02`, `0.0000`, then negative
once it penetrates, and no contact at all once the gap exceeds the 0.1 m margin —
matching the analytic surface-to-surface gap to four decimals.
`BulletDiscreteSimpleManager` and `FCLDiscreteBVHManager` report the same values.

!!! warning "Deep penetration depth is approximate"
    Reported distance is exact in the margin band and at shallow penetration —
    the values above match the analytic gap to four decimals — but deep inside
    the solid it is bounded by the probe sampling. Against a plain sphere field
    of radius 0.25, a 0.05 probe centred at the origin reports `-0.13` where the
    true depth is `-0.30`. That is fine for planning, which only needs the
    gradient near the surface, but do not read deep penetration values as exact.

!!! tip "Choosing a resolution"
    Same trade as an octree leaf size, and the same 8× scaling: `dimensions` of
    65 per axis is 275k samples, 129 is 2.1M. Between samples the field is
    trilinearly interpolated, so resolution bounds the geometric error — size it
    against the smallest feature you need represented (the bore above needs
    several samples across its 0.16 m diameter), not against your margin.

!!! note "Not the Gazebo SDF"
    Unrelated to the "SDF" Simulation Description Format. This geometry also
    replaces the old `SDFMesh`, which despite the name was a mesh, not a field.

### Runnable example

The snippets above come from
[`sdf_collision_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/sdf_collision_example.py),
which runs the whole thing end to end — the concave field, the VDB round trip,
the discrete-only environment, the swept distance profile, the rejection you get
without the YAML, and a `TesseractViewer` animation (a field has no surface, so
it draws the zero level set as a voxel shell):

```bash
tesseract_sdf_collision_example
# or
pixi run python -m tesseract_robotics.examples.sdf_collision_example
```

## Debugging Collisions

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_ALL,
)

def debug_collision(robot, joints, joint_names):
    """Print detailed collision info."""
    robot.set_joints(joints, joint_names=joint_names)
    state = robot.env.getState()

    manager = robot.env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    manager.setCollisionObjectsTransform(state.link_transforms)

    contacts = ContactResultMap()
    manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))

    if contacts.size() == 0:
        print("No collisions detected")
        return

    results = ContactResultVector()
    contacts.flattenMoveResults(results)
    print(f"Found {len(results)} contact(s):")
    for i in range(len(results)):
        r = results[i]
        print(f"\n  Contact {i + 1}:")
        print(f"    Links: {r.link_names[0]} <-> {r.link_names[1]}")
        print(f"    Distance: {r.distance:.4f} m")
        print(f"    Normal: {r.normal}")
        if r.distance < 0:
            print(f"    PENETRATION: {-r.distance:.4f} m")
```

## Next Steps

- [Motion Planning](planning.md) - Collision-aware planning
- [Low-Level SQP](low-level-sqp.md) - Real-time collision avoidance
