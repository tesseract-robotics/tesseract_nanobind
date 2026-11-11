# tesseract_robotics.scene_graph

<!-- snippet: ambient
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.geometry import Box
from tesseract_robotics.scene_graph import Joint, Link

link = Link("part")
joint = Joint("joint_1")
geometry = Box(0.1, 0.1, 0.1)
transform = Isometry3d.Identity()
-->

Scene graph elements: links, joints, and kinematic tree.

## SceneGraph

The kinematic tree structure.

```python
from tesseract_robotics.scene_graph import Joint, JointType, Link, SceneGraph

graph = SceneGraph()
graph.setName("my_robot")

# Add elements: a chain base_link -> link_1 -> ... -> link_4 -> tool0
names = ["base_link", "link_1", "link_2", "link_3", "link_4", "tool0"]
for name in names:
    graph.addLink(Link(name))
for parent, child in zip(names, names[1:]):
    joint = Joint(f"{parent}_to_{child}")
    joint.type = JointType.FIXED
    joint.parent_link_id = parent
    joint.child_link_id = child
    graph.addJoint(joint)

# Query structure
links = graph.getLinks()
joints = graph.getJoints()
root = graph.getRoot()

# Find elements
link = graph.getLink("base_link")
joint = graph.getJoint("base_link_to_link_1")

# Kinematic queries
child_links = graph.getLinkChildrenIds("base_link")
moved_links = graph.getJointChildrenIds("base_link_to_link_1")   # links below the joint
moved_links = graph.getJointChildrenIds(["base_link_to_link_1", "link_1_to_link_2"])  # union, no duplicates, unordered
path = graph.getShortestPath("base_link", "tool0")

# Which listed link each link moves with: a listed link maps itself and every link
# below it, stopping at another listed link; an unknown name raises KeyError
graph.getAdjacencyMap(["link_2", "link_4"])  # {"link_2": "link_2", "link_3": "link_2", "link_4": "link_4", ...}
```

### Allowed collision matrix

`getAllowedCollisionMatrix()` returns the graph's own matrix, and `setAllowedCollisionMatrix(acm)`
makes the graph keep `acm` itself, so later edits to `acm` show up in `isCollisionAllowed`.
`setAllowedCollisionMatrix(None)` raises `TypeError` and leaves the matrix in place: upstream would
store a null pointer that the next ACM call dereferences.

```python
from tesseract_robotics.common import AllowedCollisionMatrix

acm = AllowedCollisionMatrix()
acm.addAllowedCollision("link_1", "link_2", "adjacent")
graph.setAllowedCollisionMatrix(acm)
graph.isCollisionAllowed(("link_1", "link_2"))  # True
```

## Link

A rigid body in the kinematic tree.

```python
from tesseract_robotics.scene_graph import Link, Visual, Collision
from tesseract_robotics.geometry import Box
from tesseract_robotics.common import Isometry3d

# Create link
link = Link("my_link")

# Add visual geometry
visual = Visual()
visual.geometry = Box(0.1, 0.1, 0.1)
visual.origin = Isometry3d.Identity()
link.addVisual(visual)  # Note: use addVisual(), not visual.append()

# Add collision geometry
collision = Collision()
collision.geometry = Box(0.1, 0.1, 0.1)
collision.origin = Isometry3d.Identity()
link.addCollision(collision)

# Access geometries
visuals = link.visual       # list[Visual]
collisions = link.collision # list[Collision]

# Clear geometries
link.clearVisual()
link.clearCollision()

# Hide the link, or take it out of collision checking (both True by default)
link.visible = False
link.collision_enabled = False
```

`clone()` copies `visible` and `collision_enabled`, and `==` compares them. `Link` stays
unhashable: `getHash()` hashes only the name, while `==` compares more.

## Visual

Visual representation of a link.

```python
from tesseract_robotics.scene_graph import Visual, Material

visual = Visual()
visual.name = "visual_0"
visual.geometry = geometry
visual.origin = transform

# Material (optional)
material = Material("red")  # the name is fixed at construction; getName() reads it
material.color = np.array([1.0, 0.0, 0.0, 1.0])  # RGBA
visual.material = material
```

## Collision

Collision geometry of a link.

```python
from tesseract_robotics.scene_graph import Collision

collision = Collision()
collision.name = "collision_0"
collision.geometry = geometry
collision.origin = transform
```

## Joint

Connection between two links.

```python
from tesseract_robotics.scene_graph import Joint, JointType

joint = Joint("joint_1")
joint.type = JointType.REVOLUTE
joint.parent_link_id = "link_0"
joint.child_link_id = "link_1"
joint.parent_to_joint_origin_transform = Isometry3d.Identity()
joint.axis = np.array([0.0, 0.0, 1.0])  # rotation axis
```

### JointType

| Type | Description |
|------|-------------|
| `REVOLUTE` | Rotational with limits |
| `CONTINUOUS` | Rotational, no limits |
| `PRISMATIC` | Linear sliding |
| `FIXED` | No motion |
| `FLOATING` | 6-DOF free motion |
| `PLANAR` | 2D motion in a plane |

`str()` of a joint type is upstream's `operator<<`, so `str(JointType.REVOLUTE)` is `"Revolute"`
(`"Unknown"` for `UNKNOWN`). `.name` (`"REVOLUTE"`) and `repr()` (`"JointType.REVOLUTE"`) are
unchanged.

### Printing joint values and paths

`JointDynamics`, `JointLimits`, `JointSafety`, `JointCalibration`, `JointMimic` and `ShortestPath`
print with upstream's `operator<<`; the shorter `repr()` of `JointDynamics` and `JointLimits` stays.

```python
from tesseract_robotics.scene_graph import JointLimits

print(JointLimits(-1, 1, 10, 2, 3, 4))
# lower=-1 upper=1 effort=10 velocity=2 acceleration=3 jerk=4

print(graph.getShortestPath("base_link", "tool0"))
# Links:
#   base_link
#   ...
# Joints:
#   ...
# Active Joints:
#   ...
```

### JointLimits

```python
from tesseract_robotics.scene_graph import JointLimits

limits = JointLimits()
limits.lower = -3.14
limits.upper = 3.14
limits.velocity = 2.0
limits.acceleration = 5.0
limits.effort = 100.0

joint.limits = limits
```

### JointDynamics

```python
from tesseract_robotics.scene_graph import JointDynamics

dynamics = JointDynamics()
dynamics.damping = 0.1
dynamics.friction = 0.05

joint.dynamics = dynamics
```

### JointMimic

```python
from tesseract_robotics.scene_graph import JointMimic

mimic = JointMimic()
mimic.joint_id = "joint_1"  # joint to mimic
mimic.multiplier = 1.0
mimic.offset = 0.0

joint.mimic = mimic
```

## Inertial

Mass and inertia properties.

```python
from tesseract_robotics.scene_graph import Inertial

inertial = Inertial()
inertial.origin = Isometry3d.Identity()
inertial.mass = 1.0
inertial.ixx = 0.01
inertial.iyy = 0.01
inertial.izz = 0.01
inertial.ixy = 0.0
inertial.ixz = 0.0
inertial.iyz = 0.0

link.inertial = inertial
```

## SceneState

Current state of the scene graph. The class is bound in `state_solver`;
`scene_graph.SceneState` is the same object, imported on first access (the
state-solver extension imports this module's extension, so an eager re-export would be an
import cycle).

```python
from tesseract_robotics.scene_graph import SceneState

state = env.getState()

# Joint positions
positions = state.joints  # dict[str, float]

# Link transforms
transforms = state.link_transforms  # dict[str, Isometry3d]
tcp_pose = transforms["tool0"]
```

`floating_joints`, `getFloatingJointValues` and `==` are described with the state solver:
[SceneState](state_solver.md#scenestate).

## Reference

::: tesseract_robotics.scene_graph._scene_graph
