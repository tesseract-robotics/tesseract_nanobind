"""
Geometry Showcase Example

Demonstrates ALL geometry types available in tesseract_robotics bindings.

C++ Reference:
    tesseract_geometry/examples/create_geometries_example.cpp

**Overview**
This example creates a visual gallery of every geometry type supported by
Tesseract. Each geometry is positioned in a grid next to a KUKA IIWA robot
for visual comparison.

**Geometry Types**
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

**Volumetric Types**:
    - SignedDistanceField: Distance to the surface sampled on a grid, negative inside -
      gives distance and gradient everywhere, including for concave shapes

**File Loaders**:
    - createMeshFromPath(): Load Mesh from STL/OBJ/DAE files
    - createConvexMeshFromPath(): Load as ConvexMesh for efficient collision

**Key Concepts**
1. **Visual vs Collision Geometry**: Links have separate visual (rendering) and
   collision (physics) geometry. They can differ - e.g., use detailed mesh for
   visual but convex hull for fast collision.

2. **Geometry Types for Collision**: Primitives are fastest (analytical).
   ConvexMesh uses GJK algorithm. General Mesh uses BVH tree (slowest).

3. **Face Format**: Mesh faces use [n, v0, v1, ..., vn-1] format where n is
   the vertex count per face (usually 3 for triangles).

4. **Scale Parameter**: File loaders accept scale vector [sx, sy, sz] to
   resize meshes on import.

**Related Examples**
- scene_graph_example.py: Building kinematic chains with links/joints
- tesseract_collision_example.py: Collision checking with geometries
"""

import sys

import numpy as np

from tesseract_robotics import (
    get_tesseract_resource_path,
    get_tesseract_support_path,
)
from tesseract_robotics.common import (
    GeneralResourceLocator,
)
from tesseract_robotics.geometry import (
    Box,
    Capsule,
    Cone,
    ConvexMesh,
    Cylinder,
    GeometryType,
    Mesh,
    Plane,
    Sphere,
    createConvexMeshFromPath,
    createDiscreteSignedDistanceField,
    createMeshFromPath,
    readSignedDistanceFieldVDB,
    writeSignedDistanceFieldVDB,
)
from tesseract_robotics.planning import Pose, Robot
from tesseract_robotics.scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Material,
    Visual,
)

# Viewer (skip import during testing)

# Colours [r, g, b, a], each in 0..1.
GREY = (0.5, 0.5, 0.5, 1.0)
RED = (0.9, 0.2, 0.2, 1.0)
GREEN = (0.2, 0.9, 0.2, 1.0)
BLUE = (0.2, 0.2, 0.9, 1.0)
YELLOW = (0.9, 0.9, 0.2, 1.0)
ORANGE = (0.9, 0.5, 0.2, 1.0)
PURPLE = (0.6, 0.2, 0.8, 1.0)
CYAN = (0.2, 0.8, 0.8, 1.0)
OLIVE = (0.4, 0.7, 0.4, 1.0)
TAN = (0.7, 0.7, 0.4, 1.0)

# Primitive dimensions [m].
BOX_EDGE_M = 0.15
SPHERE_RADIUS_M = 0.08
CYLINDER_RADIUS_M = 0.06
CYLINDER_LENGTH_M = 0.2
CAPSULE_RADIUS_M = 0.05
CAPSULE_LENGTH_M = 0.15
CONE_RADIUS_M = 0.08
CONE_LENGTH_M = 0.2
# The z = 0 plane, as a x + b y + c z + d = 0.
FLOOR_PLANE = (0, 0, 1, 0)

# Grid layout [m]: distance in front of the robot, spacing along y, height of the first row
# and the rise to the second and third rows.
GRID_X_M = 0.8
GRID_Y_PITCH_M = 0.35
GRID_Z_M = 0.5
GRID_ROW2_RISE_M = 0.35
GRID_ROW3_RISE_M = 0.7

# A tetrahedron [m]: an apex above three base corners, and its four triangles, wound
# counter-clockwise seen from outside.
TETRAHEDRON_VERTICES_M = (
    (0.0, 0.0, 0.1),
    (0.1, 0.0, -0.05),
    (-0.05, 0.087, -0.05),
    (-0.05, -0.087, -0.05),
)
TETRAHEDRON_TRIANGLES = ((0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 3, 2))
# A square pyramid [m]: apex, then the base corners; six triangles (the square base is two).
PYRAMID_VERTICES_M = (
    (0.0, 0.0, 0.15),
    (0.1, 0.1, 0.0),
    (0.1, -0.1, 0.0),
    (-0.1, -0.1, 0.0),
    (-0.1, 0.1, 0.0),
)
PYRAMID_TRIANGLES = ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1), (1, 4, 3), (1, 3, 2))
# Vertices of a face in the faces array, which each face starts with [-].
TRIANGLE_VERTICES = 3

# Sphere distance field: default radius [m], samples per axis [-], and how far the grid
# reaches past the sphere, as a multiple of its radius [-].
SDF_RADIUS_M = 0.08
SDF_SAMPLES = 17
SDF_DOMAIN_MARGIN = 1.5
# A probe at the grid's corner, outside the sphere [m].
SDF_PROBE_M = 0.15
# Scale applied to the meshes loaded from file [-]: 30 % on the sphere, 5 % on the 2 m box.
SPHERE_FILE_SCALE = 0.3
BOX_FILE_SCALE = 0.05
# Width of the console banner [characters].
BANNER_WIDTH = 60


def create_geometry_link(
    name: str,
    geometry,
    transform: Pose,
    color: tuple = GREY,
    add_collision: bool = True,
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
        add_collision: Whether to add collision geometry (disable for unsupported types)

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
    link.addVisual(visual)

    # Collision component: used for physics queries, often simplified geometry
    # Note: Some geometry types (PolygonMesh) aren't supported by Bullet
    if add_collision:
        collision = Collision()
        collision.geometry = geometry
        link.addCollision(collision)

    # Fixed joint attaches this link rigidly to the robot's base_link
    # The transform positions the geometry in world coordinates
    joint = Joint(f"joint_{name}")
    joint.parent_link_id = "base_link"
    joint.child_link_id = name
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = transform

    return link, joint


def _faces(triangles) -> np.ndarray:
    """Triangles of vertex indices as the faces array: `[3, v0, v1, v2]` per face."""
    return np.array(
        [index for triangle in triangles for index in (TRIANGLE_VERTICES, *triangle)],
        dtype=np.int32,
    )


def create_tetrahedron_vertices_faces():
    """
    Create vertices and faces for a tetrahedron (simplest 3D mesh).

    A tetrahedron is the minimal closed 3D mesh: 4 vertices, 4 triangular faces.
    This demonstrates the vertex/face data format required by Tesseract meshes.

    Returns:
        Tuple of (list of (3,) vertex arrays, numpy faces array)
    """
    # Each vertex is a 3D point in local coordinates
    vertices = [np.array(vertex, dtype=np.float64) for vertex in TETRAHEDRON_VERTICES_M]

    # Faces use indexed format: [vertex_count, idx0, idx1, idx2, ...]
    # For triangles: [3, v0, v1, v2] per face
    # Vertex winding determines face normal (counter-clockwise = outward)
    faces = _faces(TETRAHEDRON_TRIANGLES)

    return vertices, faces


def create_pyramid_vertices_faces():
    """
    Create vertices and faces for a square pyramid (for ConvexMesh demo).

    A pyramid is a good test case for ConvexMesh because it's inherently convex.
    ConvexMesh uses the GJK algorithm for collision which is much faster than
    general mesh collision (BVH tree).

    Returns:
        Tuple of (list of (3,) vertex arrays, numpy faces array)
    """
    # Square pyramid: 5 vertices (apex + 4 base corners)
    vertices = [np.array(vertex, dtype=np.float64) for vertex in PYRAMID_VERTICES_M]

    # Faces: 4 triangular sides + square base (triangulated as 2 triangles)
    # Note: All non-triangular polygons must be triangulated
    faces = _faces(PYRAMID_TRIANGLES)

    return vertices, faces


def create_sphere_sdf(radius: float = SDF_RADIUS_M, samples: int = SDF_SAMPLES):
    """
    Bake an analytic sphere into a SignedDistanceField.

    An SDF is defined by a distance function rather than by vertices: negative inside the
    surface, zero on it, positive outside. `createDiscreteSignedDistanceField` evaluates that
    function once per grid point and keeps only the samples, so the returned geometry holds no
    reference back into Python - nothing takes the GIL during collision checking.

    The batched form hands the callable every sample point as one (N, 3) array instead of
    calling it N times, which is how you would drive a vectorised or GPU evaluator.

    Args:
        radius: Sphere radius in metres
        samples: Number of grid samples along each axis (>= 2)

    Returns:
        SignedDistanceField spanning a cube that comfortably contains the sphere
    """
    half_extent = radius * SDF_DOMAIN_MARGIN
    domain_min = np.full(3, -half_extent)
    domain_max = np.full(3, half_extent)
    dimensions = np.full(3, samples, dtype=np.int32)

    def sphere_distance(points):
        return np.linalg.norm(points, axis=1) - radius

    return createDiscreteSignedDistanceField(
        sphere_distance, domain_min, domain_max, dimensions, batched=True
    )


def run(**kwargs):
    """Demonstrate all geometry types in tesseract_robotics.

    Returns:
        dict with robot (for viewer compatibility)
    """
    print("=" * BANNER_WIDTH)
    print("Tesseract Geometry Showcase")
    print("=" * BANNER_WIDTH)

    # Load KUKA IIWA robot as base
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    print(f"\nLoaded robot: {robot.env.getSceneGraph().getName()}")
    print(f"Links: {len(robot.get_link_names())}")

    # Grid layout parameters
    x_offset = GRID_X_M  # Distance from robot
    y_spacing = GRID_Y_PITCH_M  # Horizontal spacing
    z_base = GRID_Z_M  # Base height

    # =========================================================================
    # 1. PRIMITIVE SHAPES
    # =========================================================================
    # Primitives use analytical formulas for collision detection - fastest option.
    # Use these when your geometry can be approximated by simple shapes.
    print("\n--- Primitive Shapes ---")

    # Box: defined by length (x), width (y), height (z) centered at origin
    box = Box(BOX_EDGE_M, BOX_EDGE_M, BOX_EDGE_M)
    print(f"Box: {box.getX():.2f} x {box.getY():.2f} x {box.getZ():.2f}")
    assert box.getType() == GeometryType.BOX
    link, joint = create_geometry_link(
        "box_link",
        box,
        Pose.from_xyz(x_offset, -2 * y_spacing, z_base),
        color=RED,
    )
    robot.add_link(link, joint)

    # Sphere
    sphere = Sphere(SPHERE_RADIUS_M)
    print(f"Sphere: radius={sphere.getRadius():.2f}")
    assert sphere.getType() == GeometryType.SPHERE
    link, joint = create_geometry_link(
        "sphere_link",
        sphere,
        Pose.from_xyz(x_offset, -1 * y_spacing, z_base),
        color=GREEN,
    )
    robot.add_link(link, joint)

    # Cylinder
    cylinder = Cylinder(CYLINDER_RADIUS_M, CYLINDER_LENGTH_M)
    print(f"Cylinder: r={cylinder.getRadius():.2f}, l={cylinder.getLength():.2f}")
    assert cylinder.getType() == GeometryType.CYLINDER
    link, joint = create_geometry_link(
        "cylinder_link",
        cylinder,
        Pose.from_xyz(x_offset, 0, z_base),
        color=BLUE,
    )
    robot.add_link(link, joint)

    # Capsule: cylinder with hemispherical end caps
    # Ideal for representing swept volumes or robot links - smooth collision
    capsule = Capsule(CAPSULE_RADIUS_M, CAPSULE_LENGTH_M)
    print(f"Capsule: r={capsule.getRadius():.2f}, l={capsule.getLength():.2f}")
    assert capsule.getType() == GeometryType.CAPSULE
    link, joint = create_geometry_link(
        "capsule_link",
        capsule,
        Pose.from_xyz(x_offset, 1 * y_spacing, z_base),
        color=YELLOW,
    )
    robot.add_link(link, joint)

    # Cone
    cone = Cone(CONE_RADIUS_M, CONE_LENGTH_M)
    print(f"Cone: r={cone.getRadius():.2f}, l={cone.getLength():.2f}")
    assert cone.getType() == GeometryType.CONE
    link, joint = create_geometry_link(
        "cone_link",
        cone,
        Pose.from_xyz(x_offset, 2 * y_spacing, z_base),
        color=ORANGE,
    )
    robot.add_link(link, joint)

    # Plane: infinite half-space defined by equation ax + by + cz + d = 0
    # Here: 0x + 0y + 1z + 0 = 0, i.e., the z=0 floor plane
    # Used for ground/wall collision - everything below/behind plane collides
    plane = Plane(*FLOOR_PLANE)
    print(f"Plane: {plane.getA()}x + {plane.getB()}y + {plane.getC()}z + {plane.getD()} = 0")
    assert plane.getType() == GeometryType.PLANE
    # Note: Plane is collision-only geometry - cannot be visualized as a shape
    # It represents the half-space where ax + by + cz + d <= 0

    # =========================================================================
    # 2. MESH TYPES (from vertices/faces)
    # =========================================================================
    # Meshes allow arbitrary geometry but have different collision performance:
    # - Mesh: BVH tree, handles concave shapes, slowest
    # - ConvexMesh: GJK algorithm, requires convex shape, faster
    print("\n--- Mesh Types (programmatic) ---")

    z_row2 = z_base + GRID_ROW2_RISE_M

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
        color=PURPLE,
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
        color=CYAN,
    )
    robot.add_link(link, joint)

    # =========================================================================
    # 3. SIGNED DISTANCE FIELD (volumetric)
    # =========================================================================
    # Unlike the mesh types, an SDF stores no surface at all: it samples the signed
    # distance to the surface on a regular grid (negative inside, positive outside).
    # That gives distance and gradient everywhere in the volume, which is what
    # trajectory optimization (TrajOpt) consumes, and it handles concave shapes that
    # a convex hull cannot.
    print("\n--- Signed Distance Field ---")

    sdf = create_sphere_sdf(radius=SDF_RADIUS_M)
    dims = sdf.getDimensions()
    print(f"SignedDistanceField: {dims[0]}x{dims[1]}x{dims[2]} samples")
    print(f"  distance at centre:  {sdf.getDistance(np.zeros(3)):+.4f} m (inside)")
    print(f"  distance at corner:  {sdf.getDistance(np.full(3, SDF_PROBE_M)):+.4f} m (outside)")
    assert sdf.getType() == GeometryType.SIGNED_DISTANCE_FIELD

    # Fields round-trip through the standard OpenVDB and NanoVDB grid formats, so a
    # field baked offline (or fused from sensor data) can be loaded straight in.
    blob = writeSignedDistanceFieldVDB(sdf)
    print(f"  OpenVDB round trip:  {len(blob)} bytes")
    assert readSignedDistanceFieldVDB(blob).getType() == GeometryType.SIGNED_DISTANCE_FIELD

    # Not added to this scene. An SDF is concave, so continuous (cast) contact managers
    # reject it, and this robot loads the stock contact-manager config which declares one -
    # adding the link would raise. Fields need an environment configured with discrete
    # plugins only; see sdf_collision_example.py for that setup end to end.
    print("  (collision-only, and needs a discrete-only environment - see")
    print("   sdf_collision_example.py; nothing to render, so not added here)")

    # =========================================================================
    # 4. MESH FROM FILE
    # =========================================================================
    # Load mesh geometry from CAD files (STL, OBJ, DAE, PLY supported)
    # Scale parameter allows resizing on import: [sx, sy, sz]
    print("\n--- Mesh Types (from file) ---")

    z_row3 = z_base + GRID_ROW3_RISE_M

    # tesseract's support data (the `[support]` extra) has example meshes; without it
    # get_tesseract_support_path() raises SupportDataNotFoundError, naming the fix.
    tesseract_support = get_tesseract_support_path()

    # createMeshFromPath returns a list because some formats (DAE) contain
    # multiple meshes; an STL holds one.
    (loaded_mesh,) = createMeshFromPath(
        str(tesseract_support / "meshes" / "sphere_p25m.stl"),
        np.full(3, SPHERE_FILE_SCALE),  # uniform scale to 30%
    )
    print(
        f"Loaded mesh: {loaded_mesh.getVertexCount()} vertices, {loaded_mesh.getFaceCount()} faces"
    )
    link, joint = create_geometry_link(
        "loaded_mesh_link",
        loaded_mesh,
        Pose.from_xyz(x_offset, -1 * y_spacing, z_row3),
        color=OLIVE,
    )
    robot.add_link(link, joint)

    # createConvexMeshFromPath: same as createMeshFromPath but computes
    # convex hull automatically. Use for collision geometry of complex CAD.
    (loaded_convex,) = createConvexMeshFromPath(
        str(tesseract_support / "meshes" / "box_2m.stl"),
        np.full(3, BOX_FILE_SCALE),  # scale to 5% (2m -> 0.1m)
    )
    print(f"Loaded ConvexMesh: {loaded_convex.getVertexCount()} vertices")
    link, joint = create_geometry_link(
        "loaded_convex_link",
        loaded_convex,
        Pose.from_xyz(x_offset, 0, z_row3),
        color=TAN,
    )
    robot.add_link(link, joint)

    # =========================================================================
    # SUMMARY
    # =========================================================================
    print("\n" + "=" * BANNER_WIDTH)
    print("Geometry Summary")
    print("=" * BANNER_WIDTH)
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

Volumetric (constructed above, not added to this scene):
  - SignedDistanceField: collision-only, and needs a discrete-only environment
    (see sdf_collision_example.py)

File-based Meshes (top row, if available):
  - Loaded Mesh (olive): From STL file
  - Loaded ConvexMesh (tan): ConvexMesh from STL

Note: Plane geometry is a half-space for collision detection,
not typically visualized as a shape.
    """)

    print(f"\nTotal scene links: {len(robot.get_link_names())}")

    return {"robot": robot}


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["robot"].env)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
