# tesseract_robotics.tesseract_collision

Collision detection managers and contact queries.

## Contact Managers

### DiscreteContactManager

Checks collision at a single configuration.

```python
from tesseract_robotics.tesseract_collision import DiscreteContactManager

# Get from environment
manager = env.getDiscreteContactManager()

# Clone for thread safety
my_manager = manager.clone()

# Set collision objects state
my_manager.setCollisionObjectsTransform(link_transforms)

# Run collision check — fills ContactResultMap
result_map = ContactResultMap()
my_manager.contactTest(result_map, request)
```

### ContinuousContactManager

Checks collision along swept motion.

```python
from tesseract_robotics.tesseract_collision import ContinuousContactManager

# Get from environment
manager = env.getContinuousContactManager()

# Clone for thread safety
my_manager = manager.clone()

# Moving objects: start and end transforms (dict/dict, name + 2 poses, or names + 2 pose lists).
# Static objects take the single-pose overloads.
my_manager.setCollisionObjectsTransform(
    link_transforms_start,
    link_transforms_end
)

# Run continuous collision check
result_map = ContactResultMap()
my_manager.contactTest(result_map, request)
```

### ContactManagersPluginFactory

Loads contact manager plugins (Bullet, FCL) from a YAML config and creates managers.

```python
from pathlib import Path

from tesseract_robotics.tesseract_collision import ContactManagersPluginFactory

factory = ContactManagersPluginFactory(Path("contact_manager_plugins.yaml"), locator)

plugins = factory.getDiscreteContactManagerPlugins()  # dict[str, PluginInfo]
factory.addDiscreteContactManagerPlugin("MyBullet", plugins["BulletDiscreteBVHManager"])
factory.setDefaultDiscreteContactManagerPlugin("MyBullet")
manager = factory.createDiscreteContactManager("MyBullet")   # by registered name
manager = factory.createDiscreteContactManager("adhoc", plugins["BulletDiscreteBVHManager"])

print(factory.getConfig())                 # the config as a YAML string
factory.saveConfig(Path("cm.yaml"))        # OSError if the file cannot be written
```

The continuous methods mirror the discrete ones. `remove…Plugin` and
`setDefault…Plugin` raise `KeyError` for an unknown name; `create…ContactManager(name)`
returns `None` for one. A manager keeps its factory alive: its code lives in a
plugin library the factory loaded, so dropping the factory first is safe.

## Contact Request

Configure what contacts to find.

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest, ContactTestType
)

request = ContactRequest()
request.type = ContactTestType.ALL  # find all contacts

# Alternatively
request.type = ContactTestType.FIRST    # stop at first contact
request.type = ContactTestType.CLOSEST  # only closest contact
request.type = ContactTestType.LIMITED  # up to max contacts
```

| ContactTestType | Description |
|-----------------|-------------|
| `FIRST` | Stop at first contact found |
| `CLOSEST` | Only return closest contact |
| `ALL` | Return all contacts |
| `LIMITED` | Return up to N contacts |

### Validating contacts

`ContactRequest.is_valid` takes a `ContactResultValidator`: each contact the
manager finds is passed to it, and a `False` return drops that contact. Subclass
it and implement `__call__`. `None` (the default) disables validation.

```python
from tesseract_robotics.tesseract_collision import ContactResultValidator


class IgnoreGripper(ContactResultValidator):
    def __call__(self, result):
        return "gripper" not in result.link_names


request.is_valid = IgnoreGripper()   # the request keeps the validator alive
manager.contactTest(result_map, request)
```

`contactTest` releases the GIL; the validator takes it again for each call, so a
contact test with a Python validator also runs from a worker thread. An exception
raised in `__call__` propagates out of `contactTest`. The `result` argument is a
copy, so keeping it beyond the call is safe.

## Contact Results

### ContactResult

Single contact between two objects.

```python
from tesseract_robotics.tesseract_collision import ContactResult, ContactResultVector

# ContactResultMap maps a link pair to its contacts.
# Flatten it to iterate individual contacts.
result_vector = ContactResultVector()
result_map.flattenMoveResults(result_vector)   # or flattenCopyResults()

for contact in result_vector:
    print(f"Contact: {contact.link_names[0]} <-> {contact.link_names[1]}")
    print(f"  Distance: {contact.distance}")
    print(f"  Point A: {contact.nearest_points[0]}")
    print(f"  Point B: {contact.nearest_points[1]}")
    print(f"  Normal: {contact.normal}")
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `distance` | `float` | Signed distance (negative = penetration) |
| `link_names` | `list[str]` (2) | Colliding link names |
| `type_id` | `list[int]` (2) | Collision object type ids |
| `shape_id` | `list[int]` (2) | Shape index within each link (`-1` if unset) |
| `subshape_id` | `list[int]` (2) | Sub-shape index within each shape (`-1` if unset) |
| `nearest_points` | `list[np.ndarray]` (2) | Nearest points, world frame |
| `nearest_points_local` | `list[np.ndarray]` (2) | Nearest points, each link's frame |
| `transform` | `list[Isometry3d]` (2) | Link transforms at the contact |
| `normal` | `np.ndarray` | Contact normal (A to B) |
| `cc_time` | `list[float]` (2) | Continuous collision times (`-1` if not continuous) |
| `cc_type` | `list[ContinuousCollisionType]` (2) | Continuous collision type per link |
| `cc_transform` | `list[Isometry3d]` (2) | Cast object's end transform, per link |

The pair fields hold exactly two items, one per link. Assigning a list of any
other length raises `TypeError`. Every read returns a new list, so
`contact.link_names[0] = "x"` changes only that copy; assign the whole list instead.

### ContactResultMap

Map from an ordered link pair `(link_a, link_b)` (`link_a` does not sort after
`link_b`) to a `ContactResultVector`. `count()` is the number of contacts,
`size()` / `len()` the number of pairs that have at least one.

```python
from tesseract_robotics.tesseract_collision import ContactResultMap

result_map = ContactResultMap()
manager.contactTest(result_map, request)

print(f"{result_map.size()} contact pairs, {result_map.count()} contacts")
print(result_map.getSummary())

# Read: every value is a copy, never a view into the map
for (link_a, link_b), contacts in result_map:      # (key, ContactResultVector) pairs
    print(link_a, link_b, len(contacts))
contacts = result_map.at(("base_link", "link_1"))  # KeyError if absent
as_dict = result_map.getContainer()                # dict[tuple[str, str], ContactResultVector]

# Drop every contact that involves the gripper (the vector is live only during the call)
def drop_gripper(key, contacts):
    if "gripper" in key:
        contacts.clear()

result_map.filter(drop_gripper)
```

Building a map: `addContactResult(key, result)` appends, `setContactResult(key, result)`
replaces the pair's results; both also take a `ContactResultVector`, and both return
a copy of the last result stored. A key whose names are out of order raises
`UnorderedLinkPairError`, and an empty vector raises `EmptyContactResultsError`
(both subclass `ValueError`; upstream only asserts these).
`clear()` empties every vector but keeps the keys, so iteration still yields them
with empty vectors until `shrinkToFit()` removes them. Iteration walks a snapshot,
so changing the map inside the loop is safe. `addInterpolatedCollisionResults`
merges a sub-segment's results, setting `cc_time` / `cc_type` for the active
links, with an optional `filter` callback.

### ContactResultVector

Flat list of `ContactResult`. Use this for iteration.

```python
from tesseract_robotics.tesseract_collision import ContactResultVector

all_contacts = ContactResultVector()
result_map.flattenMoveResults(all_contacts)   # or flattenCopyResults to keep map data

for contact in all_contacts:
    print(contact.distance, contact.link_names)
```

## Configuration

### CollisionCheckConfig

Configure collision checking behavior.

```python
from tesseract_robotics.tesseract_collision import CollisionCheckConfig

config = CollisionCheckConfig()
config.contact_request = request
config.longest_valid_segment_length = 0.01  # for continuous
```

### ContactManagerConfig

Configure contact manager settings.

```python
from tesseract_robotics.tesseract_collision import ContactManagerConfig

config = ContactManagerConfig()
config.margin_data.setDefaultCollisionMargin(0.025)
config.margin_data.setPairCollisionMargin("link_a", "link_b", 0.05)
```

## Convex Mesh Generation

Generate convex hulls for collision. `makeConvexMesh` turns a `Mesh` into a `ConvexMesh`;
`createConvexHull` works on a raw point set (Bullet's convex hull computer).

```python
import numpy as np
from tesseract_robotics.tesseract_collision import ConvexHullError, createConvexHull, makeConvexMesh

convex = makeConvexMesh(mesh)  # mesh: tesseract_geometry.Mesh

points = [np.array([x, y, z]) for x in (0.0, 1.0) for y in (0.0, 1.0) for z in (0.0, 1.0)]
n_faces, vertices, faces = createConvexHull(points)
# faces is flat: [count, i0, ..., i{count-1}] per face, indexing into vertices.

# shrink > 0 moves every face inwards by that many metres; shrink_clamp > 0 caps it at
# shrink_clamp * (smallest face distance from the hull centre).
n_faces, vertices, faces = createConvexHull(points, shrink=0.1)
```

A shrink that cannot be applied (larger than the hull, unclamped) raises `ConvexHullError`
(a `RuntimeError`). An empty point set is not an error: it gives 0 faces.

## Collision Evaluator Types

Used with trajectory optimization.

```python
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType

CollisionEvaluatorType.DISCRETE           # single config
CollisionEvaluatorType.LVS_DISCRETE       # interpolated discrete
CollisionEvaluatorType.LVS_CONTINUOUS     # swept volume
CollisionEvaluatorType.CONTINUOUS         # continuous
```

## Usage Example

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest, ContactTestType, ContactResultMap, ContactResultVector
)

# Setup
manager = env.getDiscreteContactManager().clone()
request = ContactRequest()
request.type = ContactTestType.ALL

# Set state
env.setState({"joint_1": 0.0, "joint_2": 0.0})
state = env.getState()
manager.setCollisionObjectsTransform(state.link_transforms)

# Check collision
results = ContactResultMap()
manager.contactTest(results, request)

# Process results
if not results.empty():
    contacts = ContactResultVector()
    results.flattenMoveResults(contacts)
    for c in contacts:
        if c.distance < 0:
            print(f"Penetration: {c.link_names} depth={-c.distance:.4f}")
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_collision._tesseract_collision
    options:
      show_root_heading: false
      show_source: false
      members_order: source
