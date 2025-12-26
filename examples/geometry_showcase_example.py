"""
Geometry Showcase Example
=========================

Demonstrates ALL geometry types available in tesseract_robotics bindings.

C++ Reference:
    tesseract_geometry/examples/create_geometries_example.cpp

Overview
--------
This example creates a visual gallery of every geometry type supported by
Tesseract. Each geometry is positioned in a grid next to a KUKA IIWA robot
for visual comparison.

Geometry Types
--------------
**Primitives** (mathematical definitions, fast collision):
    - Box: Axis-aligned rectangular solid (length x width x height)
    - Sphere: Ball defined by radius
    - Cylinder: Circular prism (radius x length)
    - Capsule: Cylinder with hemispherical end caps - ideal for swept volumes
    - Cone: Circular cone (radius x length)
    - Plane: Infinite half-space (ax + by + cz + d = 0) - for collision only

**Mesh Types** (from vertices/faces or files):
    - Mesh: General triangulated surface - for complex visual geometry
    - ConvexMesh: Convex hull - efficient collision for convex objects
    - SDFMesh: Signed Distance Field mesh - for smooth distance queries

**File Loaders**:
    - createMeshFromPath(): Load Mesh from STL/OBJ/DAE files
    - createConvexMeshFromPath(): Load as ConvexMesh for efficient collision

Key Concepts
------------
1. **Visual vs Collision Geometry**: Links have separate visual (rendering) and
   collision (physics) geometry. They can differ - e.g., use detailed mesh for
   visual but convex hull for fast collision.

2. **Geometry Types for Collision**: Primitives are fastest (analytical).
   ConvexMesh uses GJK algorithm. General Mesh uses BVH tree (slowest).

3. **Face Format**: Mesh faces use [n, v0, v1, ..., vn-1] format where n is
   the vertex count per face (usually 3 for triangles).

4. **Scale Parameter**: File loaders accept scale vector [sx, sy, sz] to
   resize meshes on import.

Related Examples
----------------
- scene_graph_example.py: Building kinematic chains with links/joints
- tesseract_collision_example.py: Collision checking with geometries
"""

import sys
from pathlib import Path

import numpy as np

from tesseract_robotics.planning import Robot, Pose
from tesseract_robotics.tesseract_geometry import (
    Box,
    Sphere,
    Cylinder,
    Capsule,
    Cone,
    Plane,
    Mesh,
    ConvexMesh,
    SDFMesh,
    createMeshFromPath,
    createConvexMeshFromPath,
    GeometryType,
)
from tesseract_robotics.tesseract_scene_graph import (
    Joint,
    JointType,
    Link,
    Visual,
    Collision,
    Material,
)
from tesseract_robotics.tesseract_common import (
    VectorVector3d,
)

# Viewer (skip import during testing)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def create_geometry_link(
    name: str,
    geometry,
    transform: Pose,
    color: tuple = (0.5, 0.5, 0.5, 1.0),
) -> tuple:
    """
    Create a link with visual/collision geometry and fixed joint.

    This helper demonstrates the standard pattern for adding geometry to a
    Tesseract environment:
    1. Create Link with name
    2. Attach Visual (rendering) and Collision (physics) components
    3. Create Joint connecting to parent link with transform

    Args:
        name: Link name (must be unique in scene graph)
        geometry: Geometry object (Box, Sphere, Mesh, etc.)
        transform: Link position/orientation relative to parent
        color: RGBA color tuple (0-1 range) for visualization

    Returns:
        Tuple of (Link, Joint) ready to add via robot.add_link()
    """
    link = Link(name)

    # Visual component: used for rendering only, can be high-detail mesh
    visual = Visual()
    visual.geometry = geometry
    material = Material(f"{name}_material")
    material.color = np.array(color, dtype=np.float64)
    visual.material = material
    link.visual.append(visual)

    # Collision component: used for physics queries, often simplified geometry
    # Here we use same geometry for both - in practice you might use
    # detailed mesh for visual but convex hull for collision
    collision = Collision()
    collision.geometry = geometry
    link.collision.append(collision)

    # Fixed joint attaches this link rigidly to the robot's base_link
    # The transform positions the geometry in world coordinates
    joint = Joint(f"joint_{name}")
    joint.parent_link_name = "base_link"
    joint.child_link_name = name
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = transform.to_isometry()

    return link, joint


def create_tetrahedron_vertices_faces():
    """
    Create vertices and faces for a tetrahedron (simplest 3D mesh).

    A tetrahedron is the minimal closed 3D mesh: 4 vertices, 4 triangular faces.
    This demonstrates the vertex/face data format required by Tesseract meshes.

    Returns:
        Tuple of (VectorVector3d vertices, numpy faces array)
    """
    # VectorVector3d is the required container for mesh vertices
    # Each vertex is a 3D point in local coordinates
    vertices = VectorVector3d()
    vertices.append(np.array([0.0, 0.0, 0.1], dtype=np.float64))  # apex
    vertices.append(np.array([0.1, 0.0, -0.05], dtype=np.float64))
    vertices.append(np.array([-0.05, 0.087, -0.05], dtype=np.float64))
    vertices.append(np.array([-0.05, -0.087, -0.05], dtype=np.float64))

    # Faces use indexed format: [vertex_count, idx0, idx1, idx2, ...]
    # For triangles: [3, v0, v1, v2] per face
    # Vertex winding determines face normal (counter-clockwise = outward)
    faces = np.array(
        [
            3,
            0,
            1,
            2,  # front face
            3,
            0,
            2,
            3,  # left face
            3,
            0,
            3,
            1,  # right face
            3,
            1,
            3,
            2,  # bottom face (base triangle)
        ],
        dtype=np.int32,
    )

    return vertices, faces


def create_pyramid_vertices_faces():
    """
    Create vertices and faces for a square pyramid (for ConvexMesh demo).

    A pyramid is a good test case for ConvexMesh because it's inherently convex.
    ConvexMesh uses the GJK algorithm for collision which is much faster than
    general mesh collision (BVH tree).

    Returns:
        Tuple of (VectorVector3d vertices, numpy faces array)
    """
    # Square pyramid: 5 vertices (apex + 4 base corners)
    vertices = VectorVector3d()
    vertices.append(np.array([0.0, 0.0, 0.15], dtype=np.float64))  # apex at top
    vertices.append(np.array([0.1, 0.1, 0.0], dtype=np.float64))  # base corners
    vertices.append(np.array([0.1, -0.1, 0.0], dtype=np.float64))
    vertices.append(np.array([-0.1, -0.1, 0.0], dtype=np.float64))
    vertices.append(np.array([-0.1, 0.1, 0.0], dtype=np.float64))

    # Faces: 4 triangular sides + square base (triangulated as 2 triangles)
    # Note: All non-triangular polygons must be triangulated
    faces = np.array(
        [
            3,
            0,
            1,
            2,  # front side
            3,
            0,
            2,
            3,  # right side
            3,
            0,
            3,
            4,  # back side
            3,
            0,
            4,
            1,  # left side
            3,
            1,
            4,
            3,  # base triangle 1
            3,
            1,
            3,
            2,  # base triangle 2
        ],
        dtype=np.int32,
    )

    return vertices, faces


def run(**kwargs):
    """Demonstrate all geometry types in tesseract_robotics.

    Returns:
        dict with robot (for viewer compatibility)
    """
    print("=" * 60)
    print("Tesseract Geometry Showcase")
    print("=" * 60)

    # Load KUKA IIWA robot as base
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"\nLoaded robot: {robot.env.getSceneGraph().getName()}")
    print(f"Links: {len(robot.get_link_names())}")

    # Grid layout parameters
    x_offset = 0.8  # Distance from robot
    y_spacing = 0.35  # Horizontal spacing
    z_base = 0.5  # Base height

    # =========================================================================
    # 1. PRIMITIVE SHAPES
    # =========================================================================
    # Primitives use analytical formulas for collision detection - fastest option.
    # Use these when your geometry can be approximated by simple shapes.
    print("\n--- Primitive Shapes ---")

    # Box: defined by length (x), width (y), height (z) centered at origin
    box = Box(0.15, 0.15, 0.15)
    print(f"Box: {box.getX():.2f} x {box.getY():.2f} x {box.getZ():.2f}")
    assert box.getType() == GeometryType.BOX
    link, joint = create_geometry_link(
        "box_link",
        box,
        Pose.from_xyz(x_offset, -2 * y_spacing, z_base),
        color=(0.9, 0.2, 0.2, 1.0),  # red
    )
    robot.add_link(link, joint)

    # Sphere
    sphere = Sphere(0.08)
    print(f"Sphere: radius={sphere.getRadius():.2f}")
    assert sphere.getType() == GeometryType.SPHERE
    link, joint = create_geometry_link(
        "sphere_link",
        sphere,
        Pose.from_xyz(x_offset, -1 * y_spacing, z_base),
        color=(0.2, 0.9, 0.2, 1.0),  # green
    )
    robot.add_link(link, joint)

    # Cylinder
    cylinder = Cylinder(0.06, 0.2)
    print(f"Cylinder: r={cylinder.getRadius():.2f}, l={cylinder.getLength():.2f}")
    assert cylinder.getType() == GeometryType.CYLINDER
    link, joint = create_geometry_link(
        "cylinder_link",
        cylinder,
        Pose.from_xyz(x_offset, 0, z_base),
        color=(0.2, 0.2, 0.9, 1.0),  # blue
    )
    robot.add_link(link, joint)

    # Capsule: cylinder with hemispherical end caps
    # Ideal for representing swept volumes or robot links - smooth collision
    capsule = Capsule(0.05, 0.15)
    print(f"Capsule: r={capsule.getRadius():.2f}, l={capsule.getLength():.2f}")
    assert capsule.getType() == GeometryType.CAPSULE
    link, joint = create_geometry_link(
        "capsule_link",
        capsule,
        Pose.from_xyz(x_offset, 1 * y_spacing, z_base),
        color=(0.9, 0.9, 0.2, 1.0),  # yellow
    )
    robot.add_link(link, joint)

    # Cone
    cone = Cone(0.08, 0.2)
    print(f"Cone: r={cone.getRadius():.2f}, l={cone.getLength():.2f}")
    assert cone.getType() == GeometryType.CONE
    link, joint = create_geometry_link(
        "cone_link",
        cone,
        Pose.from_xyz(x_offset, 2 * y_spacing, z_base),
        color=(0.9, 0.5, 0.2, 1.0),  # orange
    )
    robot.add_link(link, joint)

    # Plane: infinite half-space defined by equation ax + by + cz + d = 0
    # Here: 0x + 0y + 1z + 0 = 0, i.e., the z=0 floor plane
    # Used for ground/wall collision - everything below/behind plane collides
    plane = Plane(0, 0, 1, 0)
    print(
        f"Plane: {plane.getA()}x + {plane.getB()}y + {plane.getC()}z + {plane.getD()} = 0"
    )
    assert plane.getType() == GeometryType.PLANE
    # Note: Plane is collision-only geometry - cannot be visualized as a shape
    # It represents the half-space where ax + by + cz + d <= 0

    # =========================================================================
    # 2. MESH TYPES (from vertices/faces)
    # =========================================================================
    # Meshes allow arbitrary geometry but have different collision performance:
    # - Mesh: BVH tree, handles concave shapes, slowest
    # - ConvexMesh: GJK algorithm, requires convex shape, faster
    # - SDFMesh: Signed distance field, smooth gradients for optimization
    print("\n--- Mesh Types (programmatic) ---")

    z_row2 = z_base + 0.35

    # Mesh: general triangulated surface - supports any topology including concave
    # Use for complex visual geometry or when collision accuracy matters more than speed
    vertices, faces = create_tetrahedron_vertices_faces()
    mesh = Mesh(vertices, faces)
    print(f"Mesh: {mesh.getVertexCount()} vertices, {mesh.getFaceCount()} faces")
    assert mesh.getType() == GeometryType.MESH
    link, joint = create_geometry_link(
        "mesh_link",
        mesh,
        Pose.from_xyz(x_offset, -1 * y_spacing, z_row2),
        color=(0.6, 0.2, 0.8, 1.0),  # purple
    )
    robot.add_link(link, joint)

    # ConvexMesh: efficient collision using GJK/EPA algorithms
    # Requires geometry to be convex (no internal angles > 180 degrees)
    # ~10x faster than general Mesh for collision queries
    vertices, faces = create_pyramid_vertices_faces()
    convex_mesh = ConvexMesh(vertices, faces)
    print(
        f"ConvexMesh: {convex_mesh.getVertexCount()} vertices, {convex_mesh.getFaceCount()} faces"
    )
    assert convex_mesh.getType() == GeometryType.CONVEX_MESH
    link, joint = create_geometry_link(
        "convex_mesh_link",
        convex_mesh,
        Pose.from_xyz(x_offset, 0, z_row2),
        color=(0.2, 0.8, 0.8, 1.0),  # cyan
    )
    robot.add_link(link, joint)

    # SDFMesh: provides signed distance field for smooth gradient queries
    # Essential for trajectory optimization (TrajOpt) which needs distance gradients
    # Positive distance = outside, negative = inside, smooth transition at surface
    vertices, faces = create_tetrahedron_vertices_faces()
    sdf_mesh = SDFMesh(vertices, faces)
    print(
        f"SDFMesh: {sdf_mesh.getVertexCount()} vertices, {sdf_mesh.getFaceCount()} faces"
    )
    assert sdf_mesh.getType() == GeometryType.SDF_MESH
    link, joint = create_geometry_link(
        "sdf_mesh_link",
        sdf_mesh,
        Pose.from_xyz(x_offset, 1 * y_spacing, z_row2),
        color=(0.8, 0.2, 0.6, 1.0),  # magenta
    )
    robot.add_link(link, joint)

    # =========================================================================
    # 3. MESH FROM FILE
    # =========================================================================
    # Load mesh geometry from CAD files (STL, OBJ, DAE, PLY supported)
    # Scale parameter allows resizing on import: [sx, sy, sz]
    print("\n--- Mesh Types (from file) ---")

    z_row3 = z_base + 0.7

    # TESSERACT_SUPPORT_DIR contains example meshes from tesseract_support package
    import os

    tesseract_support = os.environ.get("TESSERACT_SUPPORT_DIR")
    if tesseract_support:
        # createMeshFromPath returns a list because some formats (DAE) contain
        # multiple meshes. For single-mesh files like STL, use meshes[0].
        sphere_mesh_path = Path(tesseract_support) / "meshes" / "sphere_p25m.stl"
        if sphere_mesh_path.exists():
            meshes = createMeshFromPath(
                str(sphere_mesh_path),
                np.array([0.3, 0.3, 0.3]),  # uniform scale to 30%
            )
            if meshes and len(meshes) > 0:
                loaded_mesh = meshes[0]
                print(
                    f"Loaded mesh: {loaded_mesh.getVertexCount()} vertices, {loaded_mesh.getFaceCount()} faces"
                )
                link, joint = create_geometry_link(
                    "loaded_mesh_link",
                    loaded_mesh,
                    Pose.from_xyz(x_offset, -1 * y_spacing, z_row3),
                    color=(0.4, 0.7, 0.4, 1.0),  # olive
                )
                robot.add_link(link, joint)

        # createConvexMeshFromPath: same as createMeshFromPath but computes
        # convex hull automatically. Use for collision geometry of complex CAD.
        box_mesh_path = Path(tesseract_support) / "meshes" / "box_2m.stl"
        if box_mesh_path.exists():
            convex_meshes = createConvexMeshFromPath(
                str(box_mesh_path),
                np.array([0.05, 0.05, 0.05]),  # scale to 5% (2m -> 0.1m)
            )
            if convex_meshes and len(convex_meshes) > 0:
                loaded_convex = convex_meshes[0]
                print(f"Loaded ConvexMesh: {loaded_convex.getVertexCount()} vertices")
                link, joint = create_geometry_link(
                    "loaded_convex_link",
                    loaded_convex,
                    Pose.from_xyz(x_offset, 0, z_row3),
                    color=(0.7, 0.7, 0.4, 1.0),  # tan
                )
                robot.add_link(link, joint)
    else:
        print("TESSERACT_SUPPORT_DIR not set, skipping file-based meshes")

    # =========================================================================
    # SUMMARY
    # =========================================================================
    print("\n" + "=" * 60)
    print("Geometry Summary")
    print("=" * 60)
    print("""
Primitive Shapes (front row, left to right):
  - Box (red):     3D rectangular solid
  - Sphere (green): Ball defined by radius
  - Cylinder (blue): Circular prism
  - Capsule (yellow): Cylinder with hemispherical caps
  - Cone (orange): Circular cone

Programmatic Meshes (middle row):
  - Mesh (purple): Triangulated surface from vertices/faces
  - ConvexMesh (cyan): Convex hull for efficient collision
  - SDFMesh (magenta): Signed distance field mesh

File-based Meshes (top row, if available):
  - Loaded Mesh (olive): From STL file
  - Loaded ConvexMesh (tan): ConvexMesh from STL

Note: Plane geometry is a half-space for collision detection,
not typically visualized as a shape.
    """)

    print(f"\nTotal scene links: {len(robot.get_link_names())}")

    return {"robot": robot}


def main():
    results = run()

    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.start_serve_background()
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
