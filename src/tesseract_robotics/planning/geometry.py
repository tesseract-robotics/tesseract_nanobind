"""
Geometry helpers for creating collision objects and obstacles.

Provides factory functions for common geometry types and a high-level
interface for adding obstacles to the environment.

Example:
    ```python
    from tesseract_robotics import get_tesseract_resource_path
    from tesseract_robotics.common import GeneralResourceLocator
    from tesseract_robotics.planning import Robot, box, sphere, create_obstacle, Pose

    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400", locator)

    # Add a box obstacle
    create_obstacle(
        robot,
        name="table",
        geometry=box(1.0, 0.5, 0.05),
        transform=Pose.from_xyz(0.5, 0, 0.4),
    )

    # Add a sphere obstacle
    create_obstacle(
        robot,
        name="ball",
        geometry=sphere(0.1),
        transform=Pose.from_xyz(0.3, 0.2, 0.6),
    )
    ```
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Final
from urllib.parse import urlparse
from urllib.request import url2pathname

import numpy as np
from numpy.typing import ArrayLike

if TYPE_CHECKING:
    from tesseract_robotics.planning.core import Robot

from tesseract_robotics.common import (
    GeneralResourceLocator,
    Isometry3d,
    Resource,
    SimpleLocatedResource,
)
from tesseract_robotics.geometry import (
    Box,
    Cone,
    ConvexMesh,
    Cylinder,
    Geometry,
    Mesh,
    Sphere,
    createConvexMeshFromPath,
    createMeshFromResource,
)
from tesseract_robotics.planning.errors import (
    AmbiguousMeshError,
    InvalidArrayShapeError,
    MeshLoadError,
    ResourceNotFoundError,
)
from tesseract_robotics.planning.transforms import Pose
from tesseract_robotics.scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
    Material,
    Visual,
)

# URL schemes a mesh path may carry; anything else is a local path. A file:// URL is parsed here,
# a package:// URL is resolved by the locator.
FILE_SCHEME: Final = "file://"
URL_SCHEMES: Final = ("package://", FILE_SCHEME)

# Triangulate faces; right for visual and plain meshes, wrong for convex hull input.
TRIANGULATE: Final = True

# Condense the file's meshes into one (upstream: for visual meshes only). A file that still yields
# several meshes is reported, not truncated.
FLATTEN: Final = True

# Collision meshes are never flattened: each submesh keeps its own hull.
FLATTEN_CONVEX: Final = False


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


def _locate(filepath: str | Path, locator: GeneralResourceLocator) -> Resource:
    """Resolve a `package://` / `file://` URL or a filesystem path to a located resource.

    Args:
        filepath: A URL with a `package://` or `file://` scheme, or a local path.
        locator: Resolves the URL.

    Returns:
        The located resource.

    Raises:
        ResourceNotFoundError: a local path is not an existing file, or the locator cannot
            resolve the URL to one.
    """
    text = str(filepath)
    if text.startswith(FILE_SCHEME):
        # Parsed here, not by the locator: GeneralResourceLocator strips the scheme and turns an
        # RFC 8089 Windows URI (file:///C:/...) into /C:/..., which it cannot resolve.
        path = Path(url2pathname(urlparse(text).path))
    elif text.startswith(URL_SCHEMES):
        resource = locator.locateResource(text)
        if resource is None:
            raise ResourceNotFoundError(f"Locator could not resolve mesh resource: {text}")
        # A package:// URL resolves to a path whether or not the file is there.
        if not Path(resource.getFilePath()).is_file():
            raise ResourceNotFoundError(f"Mesh resource {text} resolves to a missing file")
        return resource
    else:
        path = Path(filepath)
    path = path.resolve()
    if not path.is_file():
        raise ResourceNotFoundError(f"Mesh file does not exist: {path}")
    # A local file is its own resource; the locator still resolves the URLs the mesh refers to.
    return SimpleLocatedResource(path.as_uri(), str(path), locator)


def _scale_vector(scale: ArrayLike | None) -> np.ndarray:
    """`scale` as a 3-vector; None is the unit scale."""
    if scale is None:
        return np.ones(3)
    vector = np.asarray(scale, dtype=np.float64)
    if vector.shape != (3,):
        raise InvalidArrayShapeError(f"Scale must have 3 elements, got {vector.shape}")
    return vector


def meshes_from_file(
    filepath: str | Path,
    scale: ArrayLike | None = None,
    locator: GeneralResourceLocator | None = None,
) -> list[Mesh]:
    """Load every mesh a file holds (STL, DAE, OBJ, ...).

    A scene file such as a DAE can hold several meshes; this returns all of them, in file order.

    Args:
        filepath: Local path, `file://` URL or `package://` URL of the mesh file.
        scale: Optional scale factors `[sx, sy, sz]`.
        locator: Resolves `package://` URLs (uses `GeneralResourceLocator` if not provided).

    Returns:
        One or more meshes.

    Raises:
        ResourceNotFoundError: the file does not exist or the URL does not resolve.
        MeshLoadError: the file loaded no mesh.
        InvalidArrayShapeError: `scale` does not have 3 elements.
    """
    locator = locator or GeneralResourceLocator()
    resource = _locate(filepath, locator)
    meshes = createMeshFromResource(resource, _scale_vector(scale), TRIANGULATE, FLATTEN)
    if meshes is None or len(meshes) == 0:
        raise MeshLoadError(
            f"Failed to load a mesh from {filepath}; check the format is supported (STL, DAE, OBJ)."
        )
    return list(meshes)


def mesh_from_file(
    filepath: str | Path,
    scale: ArrayLike | None = None,
    locator: GeneralResourceLocator | None = None,
) -> Mesh:
    """Load the single mesh of a file (STL, DAE, OBJ, ...).

    Args:
        filepath: Local path, `file://` URL or `package://` URL of the mesh file.
        scale: Optional scale factors `[sx, sy, sz]`.
        locator: Resolves `package://` URLs (uses `GeneralResourceLocator` if not provided).

    Returns:
        The mesh.

    Raises:
        ResourceNotFoundError: the file does not exist or the URL does not resolve.
        MeshLoadError: the file loaded no mesh.
        AmbiguousMeshError: the file holds several meshes; use `meshes_from_file`.
        InvalidArrayShapeError: `scale` does not have 3 elements.
    """
    meshes = meshes_from_file(filepath, scale, locator)
    if len(meshes) > 1:
        raise AmbiguousMeshError(
            f"{filepath} holds {len(meshes)} meshes; mesh_from_file returns one. "
            f"Use meshes_from_file to get all of them."
        )
    return meshes[0]


def convex_mesh_from_file(
    filepath: str | Path,
    scale: ArrayLike | None = None,
    locator: GeneralResourceLocator | None = None,
) -> ConvexMesh:
    """Load the single convex mesh of a file.

    Creates a convex hull from the loaded mesh, suitable for
    efficient collision checking.

    Args:
        filepath: Local path, `file://` URL or `package://` URL of the mesh file.
        scale: Optional scale factors `[sx, sy, sz]`.
        locator: Resolves `package://` URLs (uses `GeneralResourceLocator` if not provided).

    Returns:
        The convex mesh.

    Raises:
        ResourceNotFoundError: the file does not exist or the URL does not resolve.
        MeshLoadError: the file loaded no mesh.
        AmbiguousMeshError: the file holds several meshes.
        InvalidArrayShapeError: `scale` does not have 3 elements.
    """
    locator = locator or GeneralResourceLocator()
    path = Path(_locate(filepath, locator).getFilePath())
    meshes = createConvexMeshFromPath(str(path), _scale_vector(scale), TRIANGULATE, FLATTEN_CONVEX)
    if meshes is None or len(meshes) == 0:
        raise MeshLoadError(
            f"Failed to load a convex mesh from {filepath}; "
            f"check the format is supported (STL, DAE, OBJ)."
        )
    if len(meshes) > 1:
        raise AmbiguousMeshError(
            f"{filepath} holds {len(meshes)} meshes; convex_mesh_from_file returns one."
        )
    return meshes[0]


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

    link.addVisual(visual)

    collision = Collision()
    collision.origin = origin
    collision.geometry = geometry
    link.addCollision(collision)


def create_obstacle(
    robot: Robot,
    name: str,
    geometry: Geometry,
    transform: Pose | None = None,
    parent_link: str = "base_link",
    color: tuple | None = None,
) -> None:
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

    Raises:
        EnvironmentCommandError: the environment refused the link (duplicate name, unknown parent).

    Example:
        create_obstacle(
            robot,
            name="box_obstacle",
            geometry=box(0.5, 0.5, 0.5),
            transform=Pose.from_xyz(0.5, 0, 0.3),
            color=(0.8, 0.2, 0.2, 1.0),
        )
    """
    if transform is None:
        transform = Pose()

    # Create link with geometry at origin of link frame
    link = Link(name)
    _add_visual_and_collision(link, geometry, Isometry3d.Identity(), name, color)

    # Create fixed joint with transform from parent to child
    joint = Joint(f"joint_{name}")
    joint.parent_link_id = parent_link
    joint.child_link_id = name
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = transform

    robot.add_link(link, joint)


def create_link_with_geometry(
    name: str,
    geometry: Geometry,
    origin: Pose | None = None,
    color: tuple | None = None,
) -> Link:
    """
    Create a link with visual and collision geometry.

    Lower-level function for creating links without adding them
    to an environment. Useful for building custom scene graphs.

    Args:
        name: Link name
        geometry: Geometry object
        origin: Pose from link frame to geometry (identity if None)
        color: Optional RGBA color

    Returns:
        Configured Link object
    """
    isometry = origin if origin is not None else Isometry3d.Identity()

    link = Link(name)
    _add_visual_and_collision(link, geometry, isometry, name, color)
    return link


def create_fixed_joint(
    name: str,
    parent_link: str,
    child_link: str,
    origin: Pose | None = None,
) -> Joint:
    """
    Create a fixed joint between two links.

    Args:
        name: Joint name
        parent_link: Parent link name
        child_link: Child link name
        origin: Pose from parent to child (identity if None)

    Returns:
        Configured Joint object
    """
    joint = Joint(name)
    joint.parent_link_id = parent_link
    joint.child_link_id = child_link
    joint.type = JointType.FIXED

    if origin is not None:
        joint.parent_to_joint_origin_transform = origin

    return joint
