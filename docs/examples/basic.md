# Basic Examples

Fundamental examples demonstrating kinematics, collision detection, and scene graph operations.

## Kinematics Example

Forward and inverse kinematics with the ABB IRB2400:

```python title="tesseract_kinematics_example.py"
from tesseract_robotics.planning import Robot
import numpy as np

# Load robot
robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env

# Get kinematic group
manip = env.getKinematicGroup("manipulator")
joint_names = list(manip.getJointNames())
print(f"Joints: {joint_names}")

# Forward kinematics
joints = np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0])
transforms = manip.calcFwdKin(joints)

tcp_pose = transforms["tool0"]
print(f"\nTCP Position: {tcp_pose.translation()}")
print(f"TCP Rotation:\n{tcp_pose.rotation()}")

# Inverse kinematics
target = tcp_pose
target.translate([0.1, 0, 0])  # Move 10cm in X

solutions = manip.calcInvKin(target, joints)
if solutions:
    print(f"\nFound {len(solutions)} IK solutions")
    print(f"First solution: {solutions[0]}")
else:
    print("No IK solution found")

# Joint limits
limits = manip.getLimits()
print(f"\nJoint limits:")
for i, name in enumerate(joint_names):
    lower = limits.joint_limits[i, 0]
    upper = limits.joint_limits[i, 1]
    print(f"  {name}: [{lower:.2f}, {upper:.2f}] rad")
```

??? example "Expected Output"
    ```
    Joints: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    TCP Position: [1.142  0.     1.131]
    TCP Rotation:
    [[ 0.878 -0.     0.479]
     [ 0.     1.     0.   ]
     [-0.479  0.     0.878]]

    Found 2 IK solutions
    First solution: [ 0.098 -0.435  0.464  0.     0.471 -0.098]

    Joint limits:
      joint_1: [-3.14, 3.14] rad
      joint_2: [-1.75, 1.92] rad
      ...
    ```

## Collision Detection Example

Check for collisions in the environment:

```python title="tesseract_collision_example.py"
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_collision import (
    ContactRequest, ContactTestType
)
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env

# Safe configuration
safe_joints = np.zeros(6)
print(f"Safe config collision-free: {robot.check_collision(safe_joints)}")

# Self-collision configuration (extreme joint angles)
collision_joints = np.array([0, 1.5, -2.0, 0, 1.5, 0])
print(f"Collision config safe: {robot.check_collision(collision_joints)}")

# Detailed collision info
if not robot.check_collision(collision_joints):
    contacts = robot.get_contacts(collision_joints)
    print(f"\nCollisions detected: {len(contacts)}")
    for contact in contacts:
        print(f"  {contact.link_names[0]} <-> {contact.link_names[1]}")
        print(f"    Distance: {contact.distance:.4f} m")
```

## Scene Graph Example

Manipulate the scene graph (add/remove objects):

```python title="scene_graph_example.py"
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_geometry import Box, Sphere
from tesseract_robotics.tesseract_scene_graph import (
    Link, Joint, JointType, Visual, Collision, Material
)
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand, RemoveLinkCommand
)
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env
scene = env.getSceneGraph()

# Print existing links
print("Existing links:")
for link in scene.getLinks():
    print(f"  {link.getName()}")

# Create obstacle link
obstacle_link = Link("obstacle_box")

# Add visual
visual = Visual()
visual.geometry = Box(0.3, 0.3, 0.3)  # 30cm cube
visual.origin = Isometry3d.Identity()
material = Material("red")
material.color = np.array([0.8, 0.2, 0.2, 1.0])
visual.material = material
obstacle_link.addVisual(visual)

# Add collision
collision = Collision()
collision.geometry = Box(0.3, 0.3, 0.3)
collision.origin = Isometry3d.Identity()
obstacle_link.addCollision(collision)

# Create joint (fixed to world)
obstacle_joint = Joint("obstacle_joint")
obstacle_joint.type = JointType.FIXED
obstacle_joint.parent_link_name = "base_link"
obstacle_joint.child_link_name = "obstacle_box"
obstacle_joint.parent_to_joint_origin_transform = Isometry3d.Identity()
obstacle_joint.parent_to_joint_origin_transform.translate([0.8, 0.0, 0.3])

# Add to environment
cmd = AddLinkCommand(obstacle_link, obstacle_joint)
env.applyCommand(cmd)
print("\nAdded obstacle_box to scene")

# Verify obstacle affects collision
joints = np.array([0.5, 0, 0, 0, 0, 0])  # Reach toward obstacle
is_safe = robot.check_collision(joints)
print(f"Collision with obstacle: {not is_safe}")

# Remove obstacle
env.applyCommand(RemoveLinkCommand("obstacle_box"))
print("Removed obstacle_box from scene")

# Verify no collision after removal
is_safe = robot.check_collision(joints)
print(f"Collision after removal: {not is_safe}")
```

## Geometry Showcase

All available geometry types:

```python title="geometry_showcase_example.py"
from tesseract_robotics.tesseract_geometry import (
    Box, Sphere, Cylinder, Capsule, Cone,
    Mesh, ConvexMesh, Plane, Octree
)
from tesseract_robotics.tesseract_common import VectorVector3d
import numpy as np

# Primitive shapes
box = Box(0.5, 0.3, 0.2)  # length, width, height
sphere = Sphere(0.15)      # radius
cylinder = Cylinder(0.1, 0.4)  # radius, length
capsule = Capsule(0.1, 0.3)    # radius, length
cone = Cone(0.15, 0.3)         # radius, length

print("Primitive shapes:")
print(f"  Box: {box.getX()} x {box.getY()} x {box.getZ()}")
print(f"  Sphere radius: {sphere.getRadius()}")
print(f"  Cylinder: r={cylinder.getRadius()}, l={cylinder.getLength()}")
print(f"  Capsule: r={capsule.getRadius()}, l={capsule.getLength()}")
print(f"  Cone: r={cone.getRadius()}, l={cone.getLength()}")

# Mesh from vertices (simple tetrahedron)
vertices = VectorVector3d()
vertices.append(np.array([0, 0, 0]))
vertices.append(np.array([1, 0, 0]))
vertices.append(np.array([0.5, 1, 0]))
vertices.append(np.array([0.5, 0.5, 1]))

triangles = np.array([
    0, 1, 2,  # Base
    0, 1, 3,  # Side 1
    1, 2, 3,  # Side 2
    0, 2, 3   # Side 3
], dtype=np.int32)

mesh = Mesh(vertices, triangles)
print(f"\nMesh: {mesh.getVertexCount()} vertices, {mesh.getTriangleCount()} triangles")

# Plane (infinite, used for ground)
plane = Plane(0, 0, 1, 0)  # Normal (0,0,1), distance 0
print(f"Plane normal: ({plane.getA()}, {plane.getB()}, {plane.getC()})")
```

??? tip "Loading Mesh Files"
    ```python
    from tesseract_robotics.tesseract_geometry import Mesh

    # From file (STL, OBJ, DAE)
    mesh = Mesh.fromFile("/path/to/model.stl")

    # With scale
    mesh = Mesh.fromFile("/path/to/model.stl", scale=[0.001, 0.001, 0.001])  # mm to m
    ```

## Transforms and Poses

Working with Isometry3d transforms:

```python title="transforms_example.py"
from tesseract_robotics.tesseract_common import Isometry3d
import numpy as np

# Identity transform
pose = Isometry3d.Identity()

# Translation
pose.translate([0.5, 0.2, 0.3])
print(f"Position: {pose.translation()}")

# Rotation from axis-angle
from scipy.spatial.transform import Rotation
R = Rotation.from_euler('z', 45, degrees=True)
pose.rotate(R.as_matrix())

# Get components
position = pose.translation()  # np.array([x, y, z])
rotation = pose.rotation()     # 3x3 rotation matrix
matrix = pose.matrix()         # 4x4 homogeneous matrix

# Compose transforms
pose2 = Isometry3d.Identity()
pose2.translate([0.1, 0, 0])
combined = pose * pose2  # pose2 in pose's frame

# Inverse
inverse = pose.inverse()

# From 4x4 matrix
matrix = np.eye(4)
matrix[:3, 3] = [1, 2, 3]  # Translation
pose_from_matrix = Isometry3d(matrix)
```

## State Management

Working with environment state:

```python title="state_example.py"
from tesseract_robotics.planning import Robot
import numpy as np

robot = Robot.from_tesseract_support("abb_irb2400")
env = robot.env

# Get current state
state = env.getState()
print(f"Joint names: {state.joint_names}")
print(f"Positions: {state.position}")

# Set state (dict interface)
env.setState({
    "joint_1": 0.5,
    "joint_2": -0.3,
    "joint_3": 0.2
})

# Set state (array interface)
joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
joint_values = np.array([0.5, -0.3, 0.2, 0.0, 0.5, 0.0])
env.setState(joint_names, joint_values)

# Get link transform in current state
tcp = env.getLinkTransform("tool0")
print(f"TCP position: {tcp.translation()}")
```

## Next Steps

- [Planning Examples](planning.md) - Motion planning
- [Online Planning](online-planning.md) - Real-time replanning
