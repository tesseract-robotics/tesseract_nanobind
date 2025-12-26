"""
Geometry helpers for creating collision objects and obstacles.

Provides factory functions for common geometry types and a high-level
interface for adding obstacles to the environment.

Example:
    from tesseract_robotics.planning import Robot, box, sphere, create_obstacle, Transform

    robot = Robot.from_tesseract_support("abb_irb2400")

    # Add a box obstacle
    create_obstacle(
        robot,
        name="table",
        geometry=box(1.0, 0.5, 0.05),
        transform=Transform.from_xyz(0.5, 0, 0.4),
    )

    # Add a sphere obstacle
    create_obstacle(
        robot,
        name="ball",
        geometry=sphere(0.1),
        transform=Transform.from_xyz(0.3, 0.2, 0.6),
    )
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from tesseract_robotics.planning.core import Robot

from tesseract_robotics.planning.transforms import Transform
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_geometry import (
    Box,
    Cone,
    ConvexMesh,
    Cylinder,
    Geometry,
    Mesh,
    Sphere,
)
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Material,
    Visual,
)


def box(x: float, y: float, z: float) -> Box:
    """
    Create a box geometry.

    Args:
        x: Size in X direction (meters)
        y: Size in Y direction (meters)
        z: Size in Z direction (meters)

    Returns:
        Box geometry
    """
    return Box(x, y, z)


def sphere(radius: float) -> Sphere:
    """
    Create a sphere geometry.

    Args:
        radius: Sphere radius (meters)

    Returns:
        Sphere geometry
    """
    return Sphere(radius)


def cylinder(radius: float, length: float) -> Cylinder:
    """
    Create a cylinder geometry.

    Args:
        radius: Cylinder radius (meters)
        length: Cylinder length/height (meters)

    Returns:
        Cylinder geometry
    """
    return Cylinder(radius, length)


def cone(radius: float, length: float) -> Cone:
    """
    Create a cone geometry.

    Args:
        radius: Base radius (meters)
        length: Cone height (meters)

    Returns:
        Cone geometry
    """
    return Cone(radius, length)


def mesh_from_file(
    filepath: str | Path,
    scale: np.ndarray | None = None,
) -> Mesh:
    """
    Load mesh geometry from file.

    Supports common mesh formats (STL, DAE, OBJ, etc.)

    Args:
        filepath: Path to mesh file
        scale: Optional scale factors [sx, sy, sz]

    Returns:
        Mesh geometry
    """
    from tesseract_robotics.tesseract_common import GeneralResourceLocator
    from tesseract_robotics.tesseract_geometry import createMeshFromResource

    locator = GeneralResourceLocator()
    filepath = str(filepath)

    if scale is None:
        scale = np.array([1.0, 1.0, 1.0])
    else:
        scale = np.asarray(scale, dtype=np.float64)

    # Use file:// URL for local files
    if not filepath.startswith("package://") and not filepath.startswith("file://"):
        filepath = f"file://{filepath}"

    meshes = createMeshFromResource(
        locator.locateResource(filepath),
        scale,
        True,  # triangulate
        True,  # flatten
    )

    if meshes is None or len(meshes) == 0:
        raise RuntimeError(
            f"Failed to load mesh from {filepath}. "
            f"Check file exists and format is supported (STL, DAE, OBJ)."
        )

    return meshes[0]


def convex_mesh_from_file(
    filepath: str | Path,
    scale: np.ndarray | None = None,
) -> ConvexMesh:
    """
    Load convex mesh geometry from file.

    Creates a convex hull from the loaded mesh, suitable for
    efficient collision checking.

    Args:
        filepath: Path to mesh file
        scale: Optional scale factors [sx, sy, sz]

    Returns:
        ConvexMesh geometry
    """
    from tesseract_robotics.tesseract_geometry import makeConvexMesh

    mesh = mesh_from_file(filepath, scale)
    return makeConvexMesh(mesh)


def _add_visual_and_collision(
    link: Link,
    geometry: Geometry,
    origin: Isometry3d,
    name: str,
    color: tuple | None = None,
) -> None:
    """Add matching visual and collision geometry to a link."""
    visual = Visual()
    visual.origin = origin
    visual.geometry = geometry

    if color is not None:
        material = Material(f"{name}_material")
        material.color = np.array(color, dtype=np.float64)
        visual.material = material

    link.visual.append(visual)

    collision = Collision()
    collision.origin = origin
    collision.geometry = geometry
    link.collision.append(collision)


def create_obstacle(
    robot: Robot,
    name: str,
    geometry: Geometry,
    transform: Transform | None = None,
    parent_link: str = "base_link",
    color: tuple | None = None,
) -> bool:
    """
    Add a collision obstacle to the robot environment.

    Creates a new link with visual and collision geometry, attached
    to the parent link with a fixed joint.

    Args:
        robot: Robot instance to add obstacle to
        name: Name for the obstacle link
        geometry: Geometry object (box, sphere, etc.)
        transform: Pose of the obstacle (identity if None)
        parent_link: Link to attach obstacle to
        color: Optional RGBA color tuple (0-1 range)

    Returns:
        True if obstacle was added successfully

    Example:
        create_obstacle(
            robot,
            name="box_obstacle",
            geometry=box(0.5, 0.5, 0.5),
            transform=Transform.from_xyz(0.5, 0, 0.3),
            color=(0.8, 0.2, 0.2, 1.0),
        )
    """
    if transform is None:
        transform = Transform.identity()

    link = Link(name)
    _add_visual_and_collision(link, geometry, transform.to_isometry(), name, color)

    # Create fixed joint
    joint = Joint(f"joint_{name}")
    joint.parent_link_name = parent_link
    joint.child_link_name = name
    joint.type = JointType.FIXED

    return robot.add_link(link, joint)


def create_link_with_geometry(
    name: str,
    geometry: Geometry,
    origin: Transform | None = None,
    color: tuple | None = None,
) -> Link:
    """
    Create a link with visual and collision geometry.

    Lower-level function for creating links without adding them
    to an environment. Useful for building custom scene graphs.

    Args:
        name: Link name
        geometry: Geometry object
        origin: Transform from link frame to geometry (identity if None)
        color: Optional RGBA color

    Returns:
        Configured Link object
    """
    isometry = origin.to_isometry() if origin else Isometry3d.Identity()

    link = Link(name)
    _add_visual_and_collision(link, geometry, isometry, name, color)
    return link


def create_fixed_joint(
    name: str,
    parent_link: str,
    child_link: str,
    origin: Transform | None = None,
) -> Joint:
    """
    Create a fixed joint between two links.

    Args:
        name: Joint name
        parent_link: Parent link name
        child_link: Child link name
        origin: Transform from parent to child (identity if None)

    Returns:
        Configured Joint object
    """
    joint = Joint(name)
    joint.parent_link_name = parent_link
    joint.child_link_name = child_link
    joint.type = JointType.FIXED

    if origin is not None:
        joint.parent_to_joint_origin_transform = origin.to_isometry()

    return joint
