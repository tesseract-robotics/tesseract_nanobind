"""
Mesh Round-Trip Example
=======================

Load meshes from bytes, paths and resources; rebuild one through the full
mesh constructors; write meshes to PLY and load them back.

C++ Reference:
    geometry/examples/create_geometries_example.cpp  (createMeshFromPath on a support mesh)
    geometry/test/tesseract_geometry_unit.cpp:281-370  (PolygonMesh / ConvexMesh, UUID, CreationMethod)
    geometry/test/tesseract_geometry_unit.cpp:794-889  (box_2m.ply counts, LoadMeshWithMaterialGltf2Unit)
    geometry/test/tesseract_geometry_unit.cpp:104-120, 905-920  (point cloud octree, checkTranslationInvariant)
    urdf/src/mesh.cpp:84-88  (URDF meshes load with triangulate=true)
    urdf/src/utils.cpp:140-145  (writeMeshToFile = writeSimplePlyFile)
    support/meshes/box_2m.ply, support/meshes/tesseract_material_mesh.glb

Overview
--------
1. `createMeshFromBytes` / `createSDFMeshFromBytes` / `createConvexMeshFromBytes`
   on the bytes of `box_2m.ply`.
2. `tesseract_material_mesh.glb` through `create*MeshFromPath` and
   `create*MeshFromResource` with `normals`, `vertex_colors` and
   `material_and_texture` on: four meshes, the last one textured.
3. That textured quad rebuilt through the full `Mesh`, `SDFMesh`, `PolygonMesh`
   and `ConvexMesh` constructors, with a `MeshTexture` built from its texture
   image and UVs; `getUUID` / `setUUID` and `ConvexMesh.CreationMethod`.
4. `writeMeshToFile` to a temporary directory, reloaded with `createMeshFromPath`.
5. `Isometry3d.apply_points` / `apply_directions` on the glb mesh's vertices and normals.
6. `PointCloud.Point` into a `PointCloud`, then `createOctree`.

Key Concepts
------------
**triangulate defaults to True**:
    Upstream's C++ `createMeshFromBytes` defaults `triangulate` to false
    (`mesh_parser.h:541`); the binding defaults it to true, as the URDF parser
    passes for every mesh (`urdf/src/mesh.cpp:84-88`). Step 1 calls the default.

**Equality includes the UUID**:
    `Geometry ==` compares type and UUID (`geometry.cpp:40`); `PolygonMesh ==`
    adds the vertex count, face count and scale (`polygon_mesh.cpp:111-120`);
    `ConvexMesh ==` adds the creation method (`convex_mesh.cpp:80-86`). Every
    constructed mesh gets a fresh UUID, so two meshes built from the same parts
    compare unequal until `setUUID` copies one UUID onto the other.

**The PLY round trip keeps positions, not vertices**:
    `writeMeshToFile` writes vertex positions and faces only. The reader joins
    identical vertices (`aiProcess_JoinIdenticalVertices`, `mesh_parser.h:372`),
    so a mesh that repeats a position, one vertex per normal as the glb mesh
    does, comes back with one vertex per distinct position. Face counts of
    triangle meshes are kept.
"""

import math
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import (
    AngleAxisd,
    GeneralResourceLocator,
    Isometry3d,
    Resource,
)
from tesseract_robotics.tesseract_geometry import (
    ConvexMesh,
    GeometryType,
    Mesh,
    MeshTexture,
    PointCloud,
    PolygonMesh,
    SDFMesh,
    createConvexMeshFromBytes,
    createConvexMeshFromPath,
    createConvexMeshFromResource,
    createMeshFromBytes,
    createMeshFromPath,
    createMeshFromResource,
    createOctree,
    createSDFMeshFromBytes,
    createSDFMeshFromPath,
    createSDFMeshFromResource,
)
from tesseract_robotics.tesseract_urdf import writeMeshToFile

BOX_URL = "package://tesseract/support/meshes/box_2m.ply"
GLB_URL = "package://tesseract/support/meshes/tesseract_material_mesh.glb"
UNIT_SCALE = np.ones(3, dtype=np.float64)

# tesseract_geometry_unit.cpp:914-915: the transform of checkTranslationInvariant
TRANSLATION = np.array([1.5, -2.0, 0.75], dtype=np.float64)
ROTATION_AXIS = np.array([0.0, 0.0, 1.0], dtype=np.float64)
ROTATION_ANGLE = math.pi / 2
# tesseract_geometry_unit.cpp:119: createOctree(pc, 0.01, false), m
OCTREE_RESOLUTION = 0.01

# writeSimplePlyFile writes `float` coordinates with std::fixed and
# setprecision(digits10 + 1) = 7 decimals (ply_io.cpp:102): half a unit of the
# 7th decimal, m; the reader then rounds to float32, half a float32 ulp more
PLY_DECIMAL_ROUNDING = 0.5e-7


def locate(locator: GeneralResourceLocator, url: str) -> Resource:
    """The resource behind a `package://` URL; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return resource


def counts(meshes) -> list[tuple[int, int]]:
    """(faces, vertices) per mesh."""
    return [(mesh.getFaceCount(), mesh.getVertexCount()) for mesh in meshes]


def within_ply_precision(source, reloaded) -> bool:
    """Every vertex of each mesh lies within the PLY write precision of a vertex of the other."""
    a = np.array(source.getVertices(), dtype=np.float64)
    b = np.array(reloaded.getVertices(), dtype=np.float64)
    largest = np.abs(a).max() + PLY_DECIMAL_ROUNDING
    bound = PLY_DECIMAL_ROUNDING + 0.5 * float(np.spacing(np.float32(largest)))

    def covered(points, by) -> bool:
        return all((np.abs(by - point) <= bound).all(axis=1).any() for point in points)

    return covered(a, b) and covered(b, a)


def load_bytes(box_path: Path) -> dict[str, Any]:
    """Step 1: the three mesh types from the bytes of box_2m.ply."""
    # --8<-- [start:from_bytes]
    data = box_path.read_bytes()
    # default triangulate=True, as the URDF parser loads meshes (urdf/src/mesh.cpp:84-88)
    meshes = createMeshFromBytes("box_2m.ply", data)
    sdf_meshes = createSDFMeshFromBytes(
        "box_2m.ply", data, UNIT_SCALE, True, False, False, False, False
    )
    # triangulate=False keeps the six quads (tesseract_geometry_unit.cpp:806-811)
    convex_meshes = createConvexMeshFromBytes(
        "box_2m.ply", data, UNIT_SCALE, False, False, False, False, False
    )
    # --8<-- [end:from_bytes]
    return {
        "url": meshes[0].getResource().getUrl(),
        "Mesh": counts(meshes),
        "SDFMesh": counts(sdf_meshes),
        "ConvexMesh": counts(convex_meshes),
        "meshes": meshes,
        "convex_meshes": convex_meshes,
    }


def load_flags(locator: GeneralResourceLocator) -> dict[str, Any]:
    """Step 2: the glb through each path and resource loader, every flag on."""
    resource = locate(locator, GLB_URL)
    glb_path = resource.getFilePath()
    # --8<-- [start:flags]
    # scale, triangulate, flatten, normals, vertex_colors, material_and_texture
    # (LoadMeshWithMaterialGltf2Unit, tesseract_geometry_unit.cpp:824)
    flags = (UNIT_SCALE, True, True, True, True, True)
    by_path = {
        "Mesh": createMeshFromPath(glb_path, *flags),
        "SDFMesh": createSDFMeshFromPath(glb_path, *flags),
        "ConvexMesh": createConvexMeshFromPath(glb_path, *flags),
    }
    by_resource = {
        "Mesh": createMeshFromResource(resource, *flags),
        "SDFMesh": createSDFMeshFromResource(resource, *flags),
        "ConvexMesh": createConvexMeshFromResource(resource, *flags),
    }
    # --8<-- [end:flags]
    meshes = by_path["Mesh"]
    textures = [mesh.getTextures() or [] for mesh in meshes]
    texture = textures[3][0]
    unflagged = createMeshFromPath(glb_path, UNIT_SCALE, True, True, False, False, False)
    return {
        "path": {name: counts(found) for name, found in by_path.items()},
        "resource": {name: counts(found) for name, found in by_resource.items()},
        "normals": [len(mesh.getNormals() or []) for mesh in meshes],
        "textures": [len(found) for found in textures],
        "texture_bytes": len(texture.getTextureImage().getResourceContents()),
        "uvs": len(texture.getUVs()),
        "unflagged": {
            "normals": any(mesh.getNormals() is not None for mesh in unflagged),
            "textures": any(mesh.getTextures() is not None for mesh in unflagged),
        },
        "meshes": meshes,
    }


def rebuild(quad: Mesh) -> dict[str, Any]:
    """Step 3: the textured quad through each full constructor, with and without face_count."""
    vertices = quad.getVertices()
    faces = quad.getFaces()
    normals = quad.getNormals()
    material = quad.getMaterial()
    loaded_texture = (quad.getTextures() or [])[0]
    # --8<-- [start:constructors]
    texture = MeshTexture(loaded_texture.getTextureImage(), loaded_texture.getUVs())
    # the glb has no vertex colors; one RGBA per vertex from the quad's base color
    colors = [material.getBaseColorFactor()] * len(vertices)
    common = (quad.getScale(), normals, colors, material, [texture])
    pairs = {
        "Mesh": (
            Mesh(vertices, faces, quad.getResource(), *common),
            Mesh(vertices, faces, quad.getFaceCount(), quad.getResource(), *common),
        ),
        "SDFMesh": (
            SDFMesh(vertices, faces, quad.getResource(), *common),
            SDFMesh(vertices, faces, quad.getFaceCount(), quad.getResource(), *common),
        ),
        "PolygonMesh": (
            PolygonMesh(vertices, faces, quad.getResource(), *common, GeometryType.POLYGON_MESH),
            PolygonMesh(
                vertices,
                faces,
                quad.getFaceCount(),
                quad.getResource(),
                *common,
                GeometryType.POLYGON_MESH,
            ),
        ),
        "ConvexMesh": (
            ConvexMesh(vertices, faces, quad.getResource(), *common),
            ConvexMesh(vertices, faces, quad.getFaceCount(), quad.getResource(), *common),
        ),
    }
    # --8<-- [end:constructors]

    def parts_kept(mesh) -> bool:
        textures = mesh.getTextures() or []
        return (
            np.array_equal(np.array(mesh.getVertexColors()), np.array(colors))
            and np.array_equal(np.array(mesh.getNormals()), np.array(normals))
            and len(textures) == 1
            and len(textures[0].getUVs()) == len(loaded_texture.getUVs())
        )

    result = {}
    for name, (first, second) in pairs.items():
        # --8<-- [start:uuid]
        before = first == second  # False: each constructor draws a fresh UUID
        second.setUUID(first.getUUID())
        # --8<-- [end:uuid]
        result[name] = {
            "counts": counts((first, second)),
            "parts_kept": [parts_kept(first), parts_kept(second)],
            "equal_before_uuid": before,
            "equal_after_uuid": first == second,
            "type": first.getType().name,
        }
    return result


def creation_method(convex: ConvexMesh) -> dict[str, Any]:
    """Step 3: ConvexMesh.CreationMethod (tesseract_geometry_unit.cpp:349-351)."""
    # --8<-- [start:creation_method]
    loaded = convex.getCreationMethod()
    copy = ConvexMesh(convex.getVertices(), convex.getFaces(), convex.getFaceCount())
    copy.setUUID(convex.getUUID())
    equal_before_set = copy == convex
    convex.setCreationMethod(ConvexMesh.CreationMethod.CONVERTED)
    # --8<-- [end:creation_method]
    return {
        "loaded": loaded.name,
        "set": convex.getCreationMethod().name,
        "equal_before_set": equal_before_set,
        "equal_after_set": copy == convex,
    }


def roundtrip(meshes: dict[str, Mesh], directory: Path) -> dict[str, Any]:
    """Step 4: writeMeshToFile, then createMeshFromPath on the written file."""
    result = {}
    for name, mesh in meshes.items():
        # --8<-- [start:roundtrip]
        path = directory / f"{Path(name).stem}.ply"
        writeMeshToFile(mesh, str(path))
        reloaded = createMeshFromPath(str(path))
        # --8<-- [end:roundtrip]
        distinct = np.unique(np.array(mesh.getVertices(), dtype=np.float64), axis=0)
        result[name] = {
            "source": counts([mesh])[0],
            "reloaded": counts(reloaded)[0],
            "distinct": len(distinct),
            "within_ply_precision": within_ply_precision(mesh, reloaded[0]),
        }
    return result


def transform(mesh: Mesh) -> dict[str, bool]:
    """Step 5: the batched applies equal Isometry3d * p one point at a time."""
    # --8<-- [start:transform]
    tf = Isometry3d.Identity()
    tf.translate(TRANSLATION)
    tf.rotate(AngleAxisd(ROTATION_ANGLE, ROTATION_AXIS))
    rotation = Isometry3d.Identity()
    rotation.rotate(AngleAxisd(ROTATION_ANGLE, ROTATION_AXIS))
    vertices = np.array(mesh.getVertices(), dtype=np.float64)
    normals = np.array(mesh.getNormals(), dtype=np.float64)
    points = tf.apply_points(vertices)
    directions = tf.apply_directions(normals)
    # --8<-- [end:transform]
    return {
        "points": np.array_equal(points, np.array([tf * v for v in vertices])),
        "directions": np.array_equal(directions, np.array([rotation * n for n in normals])),
    }


def point_cloud(box: Mesh) -> dict[str, Any]:
    """Step 6: PointCloud.Point both ways, then createOctree."""
    # --8<-- [start:point_cloud]
    origin = PointCloud.Point()
    cloud = PointCloud()
    cloud.points = [PointCloud.Point(*vertex) for vertex in box.getVertices()] + [origin]
    octree = createOctree(cloud, OCTREE_RESOLUTION, False)
    # --8<-- [end:point_cloud]
    return {
        "points": len(cloud.points),
        "default_point": (origin.x, origin.y, origin.z),
        "leaves": octree.getNumLeafNodes(),
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator()
    box_path = Path(locate(locator, BOX_URL).getFilePath())

    from_bytes = load_bytes(box_path)
    flagged = load_flags(locator)
    glb_mesh = flagged["meshes"][0]
    with tempfile.TemporaryDirectory() as directory:
        reloaded = roundtrip(
            {"box_2m.ply": from_bytes["meshes"][0], "tesseract_material_mesh.glb[0]": glb_mesh},
            Path(directory),
        )
    return {
        "bytes": {key: from_bytes[key] for key in ("url", "Mesh", "SDFMesh", "ConvexMesh")},
        "flags": {key: value for key, value in flagged.items() if key != "meshes"},
        "constructors": rebuild(flagged["meshes"][3]),
        "creation_method": creation_method(from_bytes["convex_meshes"][0]),
        "roundtrip": reloaded,
        "transform": transform(glb_mesh),
        "point_cloud": point_cloud(from_bytes["meshes"][0]),
    }


def main() -> None:
    """Console entry point (`tesseract_mesh_roundtrip_example`): run() and print the results."""
    result = run()
    loaded = result["bytes"]
    print(
        f"From bytes ({loaded['url']}): Mesh {loaded['Mesh']}, SDFMesh {loaded['SDFMesh']}, "
        f"ConvexMesh (triangulate=False) {loaded['ConvexMesh']}  [(faces, vertices)]"
    )
    flags = result["flags"]
    print(f"glb by path: {flags['path']['Mesh']}; by resource: {flags['resource']['Mesh']}")
    print(
        f"normals {flags['normals']}, textures {flags['textures']}, "
        f"texture {flags['texture_bytes']} bytes with {flags['uvs']} UVs"
    )
    for name, parts in result["constructors"].items():
        print(
            f"{name}: {parts['counts']}, equal before setUUID {parts['equal_before_uuid']}, "
            f"after {parts['equal_after_uuid']}"
        )
    method = result["creation_method"]
    print(
        f"CreationMethod {method['loaded']} -> {method['set']}; equal before {method['equal_before_set']}, "
        f"after {method['equal_after_set']}"
    )
    for name, trip in result["roundtrip"].items():
        print(
            f"{name}: {trip['source']} -> PLY -> {trip['reloaded']} "
            f"({trip['distinct']} distinct positions, within PLY precision: {trip['within_ply_precision']})"
        )
    print(f"apply_points / apply_directions match tf * p: {result['transform']}")
    cloud = result["point_cloud"]
    print(f"PointCloud: {cloud['points']} points -> {cloud['leaves']} octree leaves")


if __name__ == "__main__":
    main()
