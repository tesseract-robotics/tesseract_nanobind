"""
Signed Distance Field Collision Example
=======================================

Builds a concave obstacle as a ``tesseract_geometry.SignedDistanceField``, puts it in an
``Environment``, and sweeps a probe past it to show the distances a contact manager reports.

An SDF stores no surface. It samples the signed distance *to* a surface — negative inside,
zero on it, positive outside — on a regular grid, which gives distance and gradient everywhere
in the volume. Two reasons to reach for one:

- **Concave shapes work.** A ``ConvexMesh`` cannot represent a pocket or a bore, and the
  general ``Mesh`` path that can is the slowest option. The obstacle here is a ball with a
  cylindrical bore through it — no convex hull applies.
- **It may already be your native format.** Fused mapping output (nvblox, VDB ESDFs, TSDF
  integration) *is* a distance field; feeding it in directly avoids meshing it and throwing
  the distance information away.

The one constraint: an SDF is concave, so **continuous (cast) contact managers reject it**.
An ``Environment`` populates
every manager its contact-manager plugin config declares, so with the stock config — which
declares a cast manager — merely adding an SDF link throws.

The fix is configuration, not code: point the SRDF at a ``contact_manager_plugins.yaml`` that
declares only discrete plugins. This example writes exactly that (see
``DISCRETE_ONLY_CONTACT_MANAGER_PLUGINS``) and then everything behaves normally —
``create_obstacle``, contact tests, margins. The trade is that such an environment has no
continuous/cast collision checking at all, so use it for cells where discrete checking is
enough. Step 5 below demonstrates the rejection you get without it.

Pipeline
--------
1. Sample a concave distance function onto a grid with ``createDiscreteSignedDistanceField``.
2. Round-trip the field through the standard OpenVDB and NanoVDB grid formats.
3. Build a discrete-only ``Environment`` from a generated URDF/SRDF/YAML trio and attach the
   field to it, alongside a display-only proxy of its zero level set.
4. Sweep the probe joint through the margin band and compare reported vs analytic distance.
5. Show the same field being rejected by an environment that declares a cast manager.
6. Animate the probe diving through the obstacle in ``TesseractViewer``.

Notes
-----
- The scene files are generated into a temporary directory at run time so the YAML that makes
  this work is visible in this file rather than hidden in a data directory.
- A field has no surface, so there is nothing for the viewer to tessellate. To make it visible
  the example adds a *display-only* companion link: the lattice points straddling the zero
  level set, voxelized into an ``Octree``, which the viewer already draws as instanced cubes.
  That shell is a picture of the sampled data the solver queries - it is never a collision
  shape, and the field itself remains the only collision geometry.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

from tesseract_robotics.planning import Pose, Robot, create_obstacle
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_CLOSEST,
)
from tesseract_robotics.tesseract_common import CollisionMarginData, GeneralResourceLocator
from tesseract_robotics.tesseract_geometry import (
    GeometryType,
    Octree,
    OctreeSubType,
    PointCloud,
    createDiscreteSignedDistanceField,
    createOctree,
    readSignedDistanceFieldNVDB,
    readSignedDistanceFieldVDB,
    writeSignedDistanceFieldNVDB,
    writeSignedDistanceFieldVDB,
)
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Material,
    Visual,
)

# Viewer (skip import during testing)
TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer

# Obstacle: a ball of this radius with a bore of this radius drilled along z.
BALL_RADIUS = 0.25
BORE_RADIUS = 0.08
PROBE_RADIUS = 0.05
MARGIN = 0.1

# Grid resolution. Voxel size is (domain_max - domain_min) / (dimensions - 1), so 65 samples
# across 0.7 m is ~11 mm - about 15 samples across the bore, enough to resolve it.
DOMAIN_HALF_EXTENT = 0.35
SAMPLES_PER_AXIS = 65

# Probe travel for the animated sweep: clear of the margin band at one end, penetrating at
# the other, so the crossing is visible.
SWEEP_START = 0.55
SWEEP_END = 0.20

# The whole point of this example: no `continuous_plugins` block. hasContinuousContactManager-
# Plugins() is then false, the Environment never builds a cast manager, and concave geometry
# stops being rejected on addLink. Everything else is the stock config.
DISCRETE_ONLY_CONTACT_MANAGER_PLUGINS = """\
# --8<-- [start:contact_manager_yaml]
contact_manager_plugins:
  search_libraries:
    - tesseract_collision_bullet_factories
    - tesseract_collision_fcl_factories
  discrete_plugins:
    default: BulletDiscreteBVHManager
    plugins:
      BulletDiscreteBVHManager:
        class: BulletDiscreteBVHManagerFactory
      BulletDiscreteSimpleManager:
        class: BulletDiscreteSimpleManagerFactory
      FCLDiscreteBVHManager:
        class: FCLDiscreteBVHManagerFactory
  # no continuous_plugins - that is what makes SDFs loadable here
# --8<-- [end:contact_manager_yaml]
"""

# For step 5 - the stock arrangement, which does declare a cast manager.
CAST_ENABLED_CONTACT_MANAGER_PLUGINS = DISCRETE_ONLY_CONTACT_MANAGER_PLUGINS + """\
  continuous_plugins:
    default: BulletCastBVHManager
    plugins:
      BulletCastBVHManager:
        class: BulletCastBVHManagerFactory
"""

# A prismatic "probe" sphere on a rail, so a joint value slides it toward the obstacle.
# tesseract:make_convex is required on the robot tag by tesseract 0.35's URDF parser.
PROBE_CELL_URDF = f"""\
<?xml version="1.0" encoding="utf-8"?>
<robot name="sdf_cell"
       xmlns:tesseract="https://github.com/tesseract-robotics/tesseract"
       tesseract:make_convex="false">
  <link name="base_link"/>
  <link name="probe">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><sphere radius="{PROBE_RADIUS}"/></geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><sphere radius="{PROBE_RADIUS}"/></geometry>
    </collision>
  </link>
  <joint name="probe_x" type="prismatic">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="probe"/>
    <axis xyz="1 0 0"/>
    <limit effort="0" lower="-1.0" upper="1.0" velocity="1.0"/>
  </joint>
</robot>
"""

PROBE_CELL_SRDF = """\
<?xml version="1.0" ?>
<robot name="sdf_cell">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="probe"/>
  </group>
  <contact_managers_plugin_config filename="file://{yaml_path}"/>
</robot>
"""


# --8<-- [start:build_field]
def bored_ball(points: np.ndarray) -> np.ndarray:
    """
    Signed distance to a ball with a cylindrical bore, for an ``(N, 3)`` array of points.

    This is the *batched* form: it receives every sample point in one call and returns one
    distance per point. Passing ``batched=True`` alongside it means one Python call instead of
    ``dimensions.prod()`` of them - the shape a vectorised or GPU evaluator wants.

    CSG subtraction is ``max(solid, -cut)``: ``ball`` is negative inside the sphere, and
    ``bore`` is positive inside the cylinder, so taking the max carves the cylinder out.
    """
    ball = np.linalg.norm(points, axis=1) - BALL_RADIUS
    bore = BORE_RADIUS - np.linalg.norm(points[:, :2], axis=1)
    return np.maximum(ball, bore)


def build_field():
    """Sample :func:`bored_ball` onto a dense grid and return the geometry."""
    return createDiscreteSignedDistanceField(
        bored_ball,
        domain_min=np.full(3, -DOMAIN_HALF_EXTENT),
        domain_max=np.full(3, DOMAIN_HALF_EXTENT),
        dimensions=np.full(3, SAMPLES_PER_AXIS, dtype=np.int32),
        batched=True,
    )
# --8<-- [end:build_field]


def write_scene(directory: Path, contact_manager_plugins: str) -> tuple[str, str]:
    """Write a URDF/SRDF/YAML trio into ``directory`` and return the URDF and SRDF paths."""
    directory.mkdir(parents=True, exist_ok=True)

    yaml_path = directory / "contact_manager_plugins.yaml"
    yaml_path.write_text(contact_manager_plugins, encoding="utf-8")

    urdf_path = directory / "sdf_cell.urdf"
    urdf_path.write_text(PROBE_CELL_URDF, encoding="utf-8")

    srdf_path = directory / "sdf_cell.srdf"
    srdf_path.write_text(PROBE_CELL_SRDF.format(yaml_path=yaml_path), encoding="utf-8")

    return f"file://{urdf_path}", f"file://{srdf_path}"


def build_robot(directory: Path, contact_manager_plugins: str) -> Robot:
    """Load the probe cell with the given contact-manager plugin configuration."""
    urdf_url, srdf_url = write_scene(directory, contact_manager_plugins)
    return Robot.from_urdf(urdf_url, srdf_url, GeneralResourceLocator())


def level_set_octree(field, resolution: float = 0.025) -> Octree:
    """
    Voxelize the field's zero level set into an ``Octree``, for display only.

    A field has no surface, so there is nothing for the viewer to tessellate. What *can* be
    drawn is the sampled grid itself: keep the lattice points that straddle the surface
    (``|d|`` within one voxel of zero) and hand them to ``createOctree``, which the viewer
    already renders as instanced cubes. The result is a voxel shell of the true zero level set
    - a faithful picture of the data the collision solver actually queries, not a remesh.

    Args:
        field: the SignedDistanceField to visualize
        resolution: octree leaf size in metres; coarser means fewer cubes to draw

    Returns:
        Octree geometry suitable for a Visual (never a Collision - see the module docstring)
    """
    dims = field.getDimensions()
    # Flat and x-fastest, so Fortran order restores the (nx, ny, nz) lattice.
    grid = field.getDistances().reshape(dims, order="F")

    lo, hi = field.getDomainMin(), field.getDomainMax()
    axes = [np.linspace(lo[i], hi[i], dims[i]) for i in range(3)]
    voxel = max((hi[i] - lo[i]) / (dims[i] - 1) for i in range(3))

    # A one-voxel band around zero: thin enough to read as a surface, thick enough to be
    # watertight (a thinner band leaves holes where the surface cuts between samples).
    i, j, k = np.nonzero(np.abs(grid) <= voxel)

    cloud = PointCloud()
    for x, y, z in zip(axes[0][i], axes[1][j], axes[2][k]):
        cloud.addPoint(float(x), float(y), float(z))

    return Octree(
        createOctree(cloud, resolution, True, True),
        OctreeSubType.BOX,
        pruned=True,
        binary_octree=True,
    )


def attach_link(
    robot: Robot,
    name: str,
    geometry,
    *,
    collision: bool = False,
    visual_color: tuple | None = None,
) -> bool:
    """
    Attach ``geometry`` on a fixed joint, choosing which components it gets.

    ``create_obstacle()`` is the normal way to do this and works fine for a field, but it always
    attaches a Visual as well - and a field has no surface, so the viewer logs an
    "unsupported visual geometry" warning for it. Splitting the two components lets the field be
    collision-only and the level-set proxy be visual-only, which is what each actually is.

    Args:
        collision: add a Collision component (what a contact manager sees)
        visual_color: RGBA to render with; ``None`` means no Visual component at all
    """
    link = Link(name)

    if visual_color is not None:
        visual = Visual()
        visual.geometry = geometry
        material = Material(f"{name}_material")
        material.color = np.array(visual_color, dtype=np.float64)
        visual.material = material
        link.addVisual(visual)

    if collision:
        collision_component = Collision()
        collision_component.geometry = geometry
        link.addCollision(collision_component)

    joint = Joint(f"joint_{name}")
    joint.parent_link_name = "base_link"
    joint.child_link_name = name
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = Pose.from_xyz(0, 0, 0)

    return robot.add_link(link, joint)


# --8<-- [start:sweep]
def sweep(robot: Robot, positions) -> list[tuple[float, float | None, float]]:
    """
    Slide the probe along +x and report the closest distance to the field at each stop.

    Returns ``(joint_value, reported_distance_or_None, analytic_gap)`` per position. The
    analytic gap is the exact surface-to-surface distance along the sweep axis, which the
    reported value should match inside the margin band.
    """
    manager = robot.env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(["probe"])
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))

    rows = []
    for x in positions:
        robot.set_joints({"probe_x": float(x)})
        manager.setCollisionObjectsTransform(robot.env.getState().link_transforms)

        results = ContactResultMap()
        manager.contactTest(results, ContactRequest(ContactTestType_CLOSEST))
        contacts = ContactResultVector()
        results.flattenMoveResults(contacts)

        reported = contacts[0].distance if len(contacts) else None
        rows.append((float(x), reported, float(x) - BALL_RADIUS - PROBE_RADIUS))
    return rows
# --8<-- [end:sweep]


def run(**kwargs):
    """
    Demonstrate SignedDistanceField construction, serialization, and collision checking.

    Returns:
        dict with the field and the discrete-only robot
    """
    print("=" * 68)
    print("Signed Distance Field Collision")
    print("=" * 68)

    # =====================================================================
    # 1. Build the field from a concave distance function
    # =====================================================================
    print("\n--- 1. Sample the distance function onto a grid ---")
    field = build_field()
    dims = field.getDimensions()
    voxel = 2 * DOMAIN_HALF_EXTENT / (SAMPLES_PER_AXIS - 1)
    print(f"SignedDistanceField: {dims[0]}x{dims[1]}x{dims[2]} samples, voxel {voxel * 1000:.1f} mm")
    assert field.getType() == GeometryType.SIGNED_DISTANCE_FIELD
    assert field.isDiscretized()

    # Spot-check the concavity: the bore is a hole, so its axis is *outside* the solid.
    for label, point in (
        ("bore axis (in the hole)", np.zeros(3)),
        ("inside the shell", np.array([0.20, 0.0, 0.0])),
        ("outside the ball", np.array([0.30, 0.0, 0.0])),
    ):
        print(f"  {label:24}  d = {field.getDistance(point):+.4f} m")

    # =====================================================================
    # 2. Round-trip through the standard VDB grid formats
    # =====================================================================
    # A field baked offline loads the same way: readSignedDistanceFieldVDB(path.read_bytes()).
    # The local scale is not stored in the grid and is supplied on read.
    print("\n--- 2. VDB / NanoVDB round trip ---")
    for name, write, read in (
        ("OpenVDB", writeSignedDistanceFieldVDB, readSignedDistanceFieldVDB),
        ("NanoVDB", writeSignedDistanceFieldNVDB, readSignedDistanceFieldNVDB),
    ):
        blob = write(field)
        restored = read(blob)
        assert restored.getType() == GeometryType.SIGNED_DISTANCE_FIELD
        np.testing.assert_array_equal(restored.getDimensions(), dims)
        print(f"  {name}: {len(blob):>7} bytes -> {restored!r}")

    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)

        # =================================================================
        # 3. Put it in a discrete-only Environment
        # =================================================================
        print("\n--- 3. Attach to a discrete-only Environment ---")
        # --8<-- [start:attach]
        robot = build_robot(tmpdir / "discrete", DISCRETE_ONLY_CONTACT_MANAGER_PLUGINS)

        # Collision-only: the field is what the contact manager queries, and it has no surface
        # to render. create_obstacle() would work here too - it just also attaches a Visual the
        # viewer cannot draw, so we split the components instead.
        added = attach_link(robot, "bored_ball", field, collision=True)
        # --8<-- [end:attach]
        print(f"  field attached (collision-only) -> {added}")

        # Display-only companion so the viewer has something to draw where the field is.
        proxy = level_set_octree(field)
        attach_link(
            robot, "bored_ball_surface", proxy, visual_color=(0.85, 0.35, 0.15, 1.0)
        )
        print(f"  level-set proxy: {proxy.calcNumSubShapes()} cubes (visual only)")
        print(f"  links: {robot.get_link_names()}")

        # =================================================================
        # 4. Sweep the probe through the margin band
        # =================================================================
        # Contacts are reported at positive distance out to the margin. That band is the
        # load-bearing case for trajectory optimization: it is the gradient that pushes links
        # apart *before* they touch.
        print(f"\n--- 4. Sweep the probe (margin {MARGIN} m) ---")
        print(f"  {'probe_x':>8}  {'reported':>12}  {'analytic':>10}")
        rows = sweep(robot, (0.45, 0.39, 0.35, 0.32, 0.30, 0.25))
        for x, reported, analytic in rows:
            shown = f"{reported:+.4f}" if reported is not None else "no contact"
            print(f"  {x:8.2f}  {shown:>12}  {analytic:+10.4f}")

        # Inside the band the field is exact; outside it, nothing is reported.
        for x, reported, analytic in rows:
            if analytic > MARGIN:
                assert reported is None, f"expected no contact at x={x}"
            else:
                assert reported is not None, f"expected a contact at x={x}"
                assert abs(reported - analytic) < 1e-3, f"x={x}: {reported} vs {analytic}"

        # =================================================================
        # 5. Why the YAML matters
        # =================================================================
        print("\n--- 5. The same field against a cast-enabled environment ---")
        cast_robot = build_robot(tmpdir / "cast", CAST_ENABLED_CONTACT_MANAGER_PLUGINS)
        try:
            create_obstacle(
                cast_robot, name="bored_ball", geometry=field, transform=Pose.from_xyz(0, 0, 0)
            )
            print("  unexpectedly accepted - this backend now supports concave cast shapes")
        except RuntimeError as e:
            print(f"  rejected, as expected: {e}")

    # Park the probe clear of the obstacle so the viewer opens on a non-colliding pose.
    robot.set_joints({"probe_x": SWEEP_START})

    print("\nDone.")
    return {"field": field, "robot": robot, "proxy": proxy}


def sweep_trajectory(period: float = 4.0, steps: int = 60):
    """
    A probe_x trajectory that dives into the obstacle and backs out again, for the viewer.

    ``update_trajectory_list`` wants one row per waypoint, joint values first and the timestamp
    last, so this is a plain list of ``[probe_x, t]``. Built by hand rather than planned: the
    point is to watch the probe cross the margin band, not to solve anything.
    """
    half = steps // 2
    inward = np.linspace(SWEEP_START, SWEEP_END, half)
    positions = np.concatenate([inward, inward[::-1]])
    times = np.linspace(0.0, period, positions.size)
    return [[float(x), float(t)] for x, t in zip(positions, times)]


def main() -> int:
    results = run()
    robot = results["robot"]

    if TesseractViewer is None or "pytest" in sys.modules:
        return 0

    viewer = TesseractViewer()
    # Bundle the trajectory into the same update_environment call rather than pushing the
    # scene and the trajectory separately - the two race on the browser side, and a
    # refresh_scene that lands after the trajectory resets the animation to rest.
    viewer.update_environment(robot.env, [0, 0, 0])
    viewer.update_trajectory_list(["probe_x"], sweep_trajectory())
    viewer.start_serve_background()

    print("Viewer at http://localhost:8000")
    print(
        "The orange voxel shell is the field's zero level set (display only); the grey sphere\n"
        "is the probe. The field itself is what the contact manager queries - the shell is\n"
        "just the sampled grid made visible."
    )
    print("Press Enter to exit.")
    try:
        input("> ")
    except (EOFError, KeyboardInterrupt):
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
