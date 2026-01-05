# tesseract_robotics.tesseract_scene_graph

Scene graph elements: links, joints, and kinematic tree.

## SceneGraph

The kinematic tree structure.

```python
from tesseract_robotics.tesseract_scene_graph import SceneGraph

graph = SceneGraph()
graph.setName("my_robot")

# Add elements
graph.addLink(link)
graph.addJoint(joint)

# Query structure
links = graph.getLinks()
joints = graph.getJoints()
root = graph.getRoot()

# Find elements
link = graph.getLink("base_link")
joint = graph.getJoint("joint_1")

# Kinematic queries
children = graph.getChildren("base_link")
parent = graph.getParent("tool0")
path = graph.getShortestPath("base_link", "tool0")
```

## Link

A rigid body in the kinematic tree.

```python
from tesseract_robotics.tesseract_scene_graph import Link, Visual, Collision
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_common import Isometry3d

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
```

## Visual

Visual representation of a link.

```python
from tesseract_robotics.tesseract_scene_graph import Visual, Material

visual = Visual()
visual.name = "visual_0"
visual.geometry = geometry
visual.origin = transform

# Material (optional)
material = Material()
material.name = "red"
material.color = np.array([1.0, 0.0, 0.0, 1.0])  # RGBA
visual.material = material
```

## Collision

Collision geometry of a link.

```python
from tesseract_robotics.tesseract_scene_graph import Collision

collision = Collision()
collision.name = "collision_0"
collision.geometry = geometry
collision.origin = transform
```

## Joint

Connection between two links.

```python
from tesseract_robotics.tesseract_scene_graph import Joint, JointType

joint = Joint("joint_1")
joint.type = JointType.REVOLUTE
joint.parent_link_name = "link_0"
joint.child_link_name = "link_1"
joint.parent_to_joint_origin_transform = Isometry3d.Identity()
joint.axis = np.array([0, 0, 1])  # rotation axis
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

### JointLimits

```python
from tesseract_robotics.tesseract_scene_graph import JointLimits

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
from tesseract_robotics.tesseract_scene_graph import JointDynamics

dynamics = JointDynamics()
dynamics.damping = 0.1
dynamics.friction = 0.05

joint.dynamics = dynamics
```

### JointMimic

```python
from tesseract_robotics.tesseract_scene_graph import JointMimic

mimic = JointMimic()
mimic.joint_name = "joint_1"  # joint to mimic
mimic.multiplier = 1.0
mimic.offset = 0.0

joint.mimic = mimic
```

## Inertial

Mass and inertia properties.

```python
from tesseract_robotics.tesseract_scene_graph import Inertial

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

Current state of the scene graph.

```python
from tesseract_robotics.tesseract_scene_graph import SceneState

state = env.getState()

# Joint positions
positions = state.joints  # dict[str, float]

# Link transforms
transforms = state.link_transforms  # dict[str, Isometry3d]
tcp_pose = transforms["tool0"]
```
