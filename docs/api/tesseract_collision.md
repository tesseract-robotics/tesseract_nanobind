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

# Set start and end transforms
my_manager.setCollisionObjectsTransform(
    link_transforms_start,
    link_transforms_end
)

# Run continuous collision check
result_map = ContactResultMap()
my_manager.contactTest(result_map, request)
```

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

## Contact Results

### ContactResult

Single contact between two objects.

```python
from tesseract_robotics.tesseract_collision import ContactResult, ContactResultVector

# ContactResultMap is a nested map (link_a → link_b → list[ContactResult]).
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

Nested map: `link_a → link_b → list[ContactResult]`. Exposes `size()`,
`empty()`, `count()`, `clear()`, `getSummary()`, and
`flattenCopyResults()` / `flattenMoveResults()` for iteration.

```python
from tesseract_robotics.tesseract_collision import ContactResultMap

result_map = ContactResultMap()
manager.contactTest(result_map, request)

print(f"{result_map.size()} contact pairs")
print(result_map.getSummary())
```

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

Generate convex hulls for collision.

```python
from tesseract_robotics.tesseract_collision import makeConvexMesh

# From vertices
vertices = [np.array([x, y, z]) for ...]
convex = makeConvexMesh(vertices)
```

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
