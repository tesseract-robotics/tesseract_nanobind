"""
Convex Decomposition Example

Port of upstream's `create_convex_decomposition` command-line tool: split a
mesh into convex hulls with V-HACD and write each hull to a PLY file.

C++ Reference:
    collision/vhacd/src/create_convex_decomposition.cpp
    collision/vhacd/include/tesseract/collision/vhacd/convex_decomposition_vhacd.h:41-59
    collision/vhacd/src/convex_decomposition_vhacd.cpp  (compute)
    support/meshes/car_seat/visual/seat.dae, support/meshes/car_seat/collision/seat_*.stl

**Overview**
1. The command-line options of upstream's tool, with its short flags and value
   ranges. Defaults are `VHACDParameters()`'s.
2. Decompose the car-seat visual mesh `seat.dae` at those defaults, check that
   every hull is convex and within `max_convex_hulls`, and write each hull as
   `<i>_<output>`, as the tool does.
3. Decompose again with each `FillMode`, and with the default-constructed
   `ConvexDecompositionVHACD()`.
4. Report the hull count and bounding box against the decomposition shipped
   with tesseract support (`car_seat/collision/seat_*.stl`), which was made
   with other, unrecorded parameters.

**Key Concepts**
**Deviations from the C++ tool**:
    `--input` defaults to `seat.dae` and `--output` to `seat.ply`, so the example
    runs without arguments; upstream requires both. Upstream reads only PLY
    (`loadSimplePlyFile`, not bound); `createMeshFromPath` reads any format
    Assimp does. `--fill_mode` is added: upstream's tool cannot set the fill
    mode, and the mode changes the result (step 3). `async_ACD` stays at its
    default, as in upstream's tool. As there, a range is checked only for a
    value given on the command line: the default `max_convex_hulls` of 64 lies
    outside `-n`'s range [1, 32].

**Convexity check**:
    Each hull is upstream's `createConvexHull` of the V-HACD hull points: Bullet
    computes it exactly on an integer grid and returns `int * scaling + center`
    (`btConvexHullComputer.cpp:2097`, Bullet 3.25), an affine image of a convex
    set, so only that last product and sum round. Every face must be planar and
    every vertex must lie on the inner side of every plane of the hull's own
    faces, within the plane shift that rounding allows. V-HACD can return a flat
    hull, all its vertices in one plane (two faces, front and back); such a hull
    is convex when each face is a convex polygon in that plane. A second `createConvexHull` of the vertices is convex
    by the same test, and each of its vertices is one of the hull's, moved by up
    to one step of Bullet's grid, (max - min) / 10216 per axis, truncated
    (`btConvexHullComputer.cpp:1992, 2025`). It is not idempotent: it can drop a
    vertex that lies within a grid step of a face (one RAYCAST_FILL hull of the
    seat loses a vertex 4 micrometres above a face), so the counts are not compared.
"""

import argparse
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    ConvexDecompositionVHACD,
    FillMode,
    VHACDParameters,
    createConvexHull,
)
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.geometry import createMeshFromPath
from tesseract_robotics.urdf import writeMeshToFile

SEAT_URL = "package://tesseract/support/meshes/car_seat/visual/seat.dae"
SHIPPED_HULLS_GLOB = "seat_*.stl"
DEFAULT_OUTPUT = "seat.ply"

# btConvexHullComputer.cpp:1992 (Bullet 3.25): hull coordinates are quantised to
# (max - min) / 10216 per axis, then truncated to an integer
BULLET_HULL_GRID = 10216
# Rounding of the grid round trip, (p - center) * s and int * scaling + center:
# four roundings of at most half an ulp of the coordinate each, in ulp
HULL_ROUND_TRIP_ULPS = 2
# Hull vertices are int * scaling + center (btConvexHullComputer.cpp:2097, Bullet 3.25):
# two roundings of at most half an ulp of the largest coordinate each, per axis, in ulp
HULL_OUTPUT_ULPS = 1
# Rounding of orient3d evaluated in doubles, relative to its permanent: Shewchuk's
# o3derrboundA = (7 + 56 eps) eps, eps = 2**-53 (Shewchuk 1997, predicates.c exactinit)
EPS = 2.0**-53
ORIENT3D_ERRBOUND = (7.0 + 56.0 * EPS) * EPS
# The same for orient2d: ccwerrboundA = (3 + 16 eps) eps (predicates.c exactinit)
ORIENT2D_ERRBOUND = (3.0 + 16.0 * EPS) * EPS
# writeSimplePlyFile writes `float` coordinates with std::fixed and
# setprecision(digits10 + 1) = 7 decimals (ply_io.cpp:84-101): half a unit of the
# 7th decimal, m; the reader then rounds to float32, half a float32 ulp more
PLY_DECIMAL_ROUNDING = 0.5e-7

# Parsed only, never decomposed: one value per option, each away from its default
EXAMPLE_ARGV = [
    "-n", "8", "-r", "1000", "-e", "2", "-d", "4", "-s", "false",
    "-v", "16", "-l", "3", "-p", "true", "--fill_mode", "RAYCAST_FILL",
]  # fmt: skip

PARAMETER_FIELDS = [
    "max_convex_hulls",
    "resolution",
    "minimum_volume_percent_error_allowed",
    "max_recursion_depth",
    "shrinkwrap",
    "fill_mode",
    "max_num_vertices_per_ch",
    "async_ACD",
    "min_edge_length",
    "find_best_plane",
]


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def in_range(convert, low, high):
    """An argparse type: `convert`, then upstream's `check_range(value, low, high)`."""

    def parse(text: str):
        value = convert(text)
        if value < low or value > high:
            raise argparse.ArgumentTypeError(f"Value {value} is not in valid range [{low}, {high}]")
        return value

    return parse


def boolean(text: str) -> bool:
    """A boost::program_options bool: true/false, yes/no, on/off, 1/0."""
    lowered = text.lower()
    if lowered in ("true", "yes", "on", "1"):
        return True
    if lowered in ("false", "no", "off", "0"):
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {text!r}")


# --8<-- [start:options]
def parse_args(argv: list[str]) -> argparse.Namespace:
    """create_convex_decomposition.cpp's options; defaults from VHACDParameters()."""
    defaults = VHACDParameters()
    unsigned_max = 2**32 - 1
    parser = argparse.ArgumentParser(description="Create a convex decomposition of a mesh")
    parser.add_argument("-i", "--input", help="File path to mesh used to create a convex hull.")
    parser.add_argument(
        "-o",
        "--output",
        default=DEFAULT_OUTPUT,
        help="File path to save the generated convex hull as a ply.",
    )
    parser.add_argument(
        "-n",
        "--max_convex_hulls",
        type=in_range(int, 1, 32),
        default=defaults.max_convex_hulls,
        help="Maximum number of convex hulls",
    )
    parser.add_argument(
        "-r",
        "--resolution",
        type=in_range(int, 1, unsigned_max),
        default=defaults.resolution,
        help="Number of voxels to use to represent the shape",
    )
    parser.add_argument(
        "-e",
        "--min_volume_percent_error",
        type=in_range(float, 0.001, 10.0),
        default=defaults.minimum_volume_percent_error_allowed,
        help="If the voxels are within this threshold percentage of the volume of the hull, "
        "we consider this a close enough approximation",
    )
    parser.add_argument(
        "-d",
        "--max_recursion_depth",
        type=in_range(int, 0, unsigned_max),
        default=defaults.max_recursion_depth,
        help="Maximum recursion depth for convex decomposition improvement",
    )
    parser.add_argument(
        "-s",
        "--shrinkwrap",
        type=boolean,
        default=defaults.shrinkwrap,
        help="Shrinkwrap the voxel positions to the source mesh on output",
    )
    parser.add_argument(
        "-v",
        "--max_num_vertices",
        type=in_range(int, 4, unsigned_max),
        default=defaults.max_num_vertices_per_ch,
        help="Maximum number of vertices per convex hull",
    )
    parser.add_argument(
        "-l",
        "--min_edge_length",
        type=in_range(int, 1, unsigned_max),
        default=defaults.min_edge_length,
        help="Once a voxel patch has an edge length of less than this value in all 3 "
        "dimensions, stop recursing",
    )
    parser.add_argument(
        "-p",
        "--find_best_plane",
        type=boolean,
        default=defaults.find_best_plane,
        help="Flag for attempting to split planes along best location (experimental)",
    )
    # Not in upstream's tool: how the voxels inside the mesh are filled
    parser.add_argument(
        "--fill_mode",
        choices=[mode.name for mode in FillMode],
        default=defaults.fill_mode.name,
        help="FLOOD_FILL for closed meshes, RAYCAST_FILL for open ones, SURFACE_ONLY for hollow",
    )
    return parser.parse_args(argv)


def parameters(args: argparse.Namespace) -> VHACDParameters:
    """VHACDParameters from the parsed options."""
    params = VHACDParameters()
    params.max_convex_hulls = args.max_convex_hulls
    params.resolution = args.resolution
    params.minimum_volume_percent_error_allowed = args.min_volume_percent_error
    params.max_recursion_depth = args.max_recursion_depth
    params.shrinkwrap = args.shrinkwrap
    params.fill_mode = FillMode[args.fill_mode]
    params.max_num_vertices_per_ch = args.max_num_vertices
    params.min_edge_length = args.min_edge_length
    params.find_best_plane = args.find_best_plane
    return params


# --8<-- [end:options]


def parameter_values(params: VHACDParameters) -> dict[str, Any]:
    """Every VHACDParameters field, the fill mode by name."""
    values = {name: getattr(params, name) for name in PARAMETER_FIELDS}
    values["fill_mode"] = params.fill_mode.name
    return values


def orient3d(a, b, c, d, eta: float):
    """Shewchuk's orient3d of points `d` against the plane (a, b, c), and its error bound.

    The bound covers the evaluation's rounding and a shift of up to `eta` in each
    entry of the differences: the determinant is multilinear with +-1 terms, so
    the shift changes it by at most perm(|M| + eta) - perm(|M|).
    """
    ad, bd, cd = a - d, b - d, c - d
    det = (
        ad[:, 0] * (bd[:, 1] * cd[:, 2] - bd[:, 2] * cd[:, 1])
        + bd[:, 0] * (cd[:, 1] * ad[:, 2] - cd[:, 2] * ad[:, 1])
        + cd[:, 0] * (ad[:, 1] * bd[:, 2] - ad[:, 2] * bd[:, 1])
    )

    def permanent(x, y, z):
        return (
            x[:, 0] * (y[:, 1] * z[:, 2] + y[:, 2] * z[:, 1])
            + y[:, 0] * (z[:, 1] * x[:, 2] + z[:, 2] * x[:, 1])
            + z[:, 0] * (x[:, 1] * y[:, 2] + x[:, 2] * y[:, 1])
        )

    perm = permanent(np.abs(ad), np.abs(bd), np.abs(cd))
    shifted = permanent(np.abs(ad) + eta, np.abs(bd) + eta, np.abs(cd) + eta)
    return det, shifted - perm + ORIENT3D_ERRBOUND * perm


def face_plane(corners):
    """Three corners spanning the face's plane: the first, and the pair widest from it.

    Bullet can list a corner on a straight edge of a polygon face, so the first
    three corners may be collinear.
    """
    a = corners[0]
    pairs = [(j, k) for j in range(1, len(corners)) for k in range(j + 1, len(corners))]
    areas = [np.linalg.norm(np.cross(corners[j] - a, corners[k] - a)) for j, k in pairs]
    j, k = pairs[int(np.argmax(areas))]
    return a, corners[j], corners[k]


def orient2d(a, b, c, eta: float):
    """Shewchuk's orient2d of points `c` against the line (a, b), and its error bound.

    The bound is built as in `orient3d`, from the 2x2 permanent.
    """
    ac, bc = a - c, b - c
    det = ac[:, 0] * bc[:, 1] - ac[:, 1] * bc[:, 0]
    perm = np.abs(ac[:, 0] * bc[:, 1]) + np.abs(ac[:, 1] * bc[:, 0])
    shifted = (np.abs(ac[:, 0]) + eta) * (np.abs(bc[:, 1]) + eta) + (np.abs(ac[:, 1]) + eta) * (
        np.abs(bc[:, 0]) + eta
    )
    return det, shifted - perm + ORIENT2D_ERRBOUND * perm


def flat_faces_convex(vertices, faces, plane, eta: float) -> bool:
    """A zero-volume hull: every vertex on `plane`, and every face a convex polygon in it.

    The polygons are tested in the plane's projection that drops the coordinate
    along which the plane's normal is largest; dropping a coordinate is exact.
    """
    a, b, c = plane
    det, bound = orient3d(a, b, c, vertices, eta)
    if (np.abs(det) > bound).any():
        return False
    keep = [axis for axis in range(3) if axis != int(np.argmax(np.abs(np.cross(b - a, c - a))))]
    flat = vertices[:, keep]
    i = 0
    while i < len(faces):
        corners = flat[faces[i + 1 : i + 1 + faces[i]]]
        i += 1 + faces[i]
        inside = corners.mean(axis=0, keepdims=True)
        for start, end in zip(corners, np.roll(corners, -1, axis=0)):
            side, side_bound = orient2d(start[None], end[None], inside, eta)
            if abs(side[0]) <= side_bound[0]:
                return False  # the polygon is a segment
            det, bound = orient2d(start[None], end[None], flat, eta)
            if (np.sign(side[0]) * det < -bound).any():
                return False
    return True


def faces_bound(vertices, faces) -> bool:
    """Every face is planar and every vertex lies on its inner side, within rounding."""
    inside = vertices.mean(axis=0, keepdims=True)
    # each entry of a difference of two vertices, each off by HULL_OUTPUT_ULPS per axis
    eta = 2 * HULL_OUTPUT_ULPS * float(np.spacing(np.abs(vertices).max()))
    i = 0
    while i < len(faces):
        corners = faces[i + 1 : i + 1 + faces[i]]
        i += 1 + faces[i]
        a, b, c = face_plane(vertices[corners])
        side, side_bound = orient3d(a, b, c, inside, eta)
        if abs(side[0]) <= side_bound[0]:
            # the face plane passes through the centroid: a flat hull
            return flat_faces_convex(vertices, faces, (a, b, c), eta)
        det, bound = orient3d(a, b, c, vertices, eta)
        if (np.abs(det[corners]) > bound[corners]).any():
            return False
        if (np.sign(side[0]) * det < -bound).any():
            return False
    return True


def hull_is_convex(hull) -> bool:
    """The hull's faces bound its vertices, and `createConvexHull` of them keeps only them.

    The second hull is convex by the same test, and each of its vertices is one of
    the hull's, moved by at most its own grid step. It may drop a vertex that
    lies within a grid step of a face, so the counts can differ.
    """
    vertices = np.array(hull.getVertices())
    if not faces_bound(vertices, np.array(hull.getFaces())):
        return False
    _, rehulled, rehulled_faces = createConvexHull(hull.getVertices())
    rehulled = np.array(rehulled)
    if not faces_bound(rehulled, rehulled_faces):
        return False
    grid_step = (vertices.max(axis=0) - vertices.min(axis=0)) / BULLET_HULL_GRID
    bound = grid_step + HULL_ROUND_TRIP_ULPS * np.spacing(np.abs(vertices).max(axis=0))
    return all((np.abs(vertices - vertex) <= bound).all(axis=1).any() for vertex in rehulled)


def reload_matches(hull, path: Path) -> bool:
    """The PLY at `path` holds the hull's vertices, to the file's write precision.

    Vertices closer than that precision may merge on reload, so the sets are
    compared both ways rather than by count.
    """
    vertices = np.array(hull.getVertices())
    reloaded = np.array(createMeshFromPath(str(path))[0].getVertices())
    largest = np.abs(vertices).max() + PLY_DECIMAL_ROUNDING
    bound = PLY_DECIMAL_ROUNDING + 0.5 * float(np.spacing(np.float32(largest)))

    def covered(points, by) -> bool:
        return all((np.abs(by - point) <= bound).all(axis=1).any() for point in points)

    return covered(reloaded, vertices) and covered(vertices, reloaded)


def signature(hulls) -> list[tuple[float, ...]]:
    """The hulls' vertex coordinates, order-independent, for comparing decompositions."""
    return sorted(tuple(np.array(hull.getVertices()).ravel()) for hull in hulls)


def bounding_box(meshes) -> list[list[float]]:
    """[min, max] corners over every vertex of `meshes`."""
    vertices = np.vstack([np.array(mesh.getVertices()) for mesh in meshes])
    return [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]


def run(argv: list[str] | None = None, output_dir: Path | None = None) -> dict[str, Any]:
    """Run the example; return every result the test asserts on.

    Args:
        argv: Command-line options; none gives upstream's defaults.
        output_dir: Where the hull files go; a temporary directory when None.
    """
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    args = parse_args(argv or [])
    seat_path = Path(args.input) if args.input else locate(locator, SEAT_URL)

    # --8<-- [start:decompose]
    params = parameters(args)
    mesh = createMeshFromPath(str(seat_path))[0]
    hulls = ConvexDecompositionVHACD(params).compute(mesh.getVertices(), mesh.getFaces(), False)
    # --8<-- [end:decompose]

    with tempfile.TemporaryDirectory() as tmp:
        directory = output_dir if output_dir is not None else Path(tmp)
        # --8<-- [start:write]
        written = []
        for i, hull in enumerate(hulls):
            path = directory / f"{i}_{Path(args.output).name}"
            writeMeshToFile(hull, str(path))
            written.append(path)
        # --8<-- [end:write]
        reloaded_matches = all(reload_matches(hull, path) for path, hull in zip(written, hulls))
        files_written = sum(path.is_file() for path in written)

    # --8<-- [start:fill_modes]
    fill_modes = {}
    for mode in FillMode:
        mode_params = parameters(args)
        mode_params.fill_mode = mode
        fill_modes[mode.name] = ConvexDecompositionVHACD(mode_params).compute(
            mesh.getVertices(), mesh.getFaces(), False
        )
    default_hulls = ConvexDecompositionVHACD().compute(mesh.getVertices(), mesh.getFaces(), False)
    # --8<-- [end:fill_modes]

    signatures = [signature(mode_hulls) for mode_hulls in fill_modes.values()]
    shipped = [
        createMeshFromPath(str(path))[0]
        for path in sorted(seat_path.parent.parent.joinpath("collision").glob(SHIPPED_HULLS_GLOB))
    ]
    return {
        "parameters": parameter_values(params),
        "parameters_from_argv": parameter_values(parameters(parse_args(EXAMPLE_ARGV))),
        "seat": {
            "num_vertices": mesh.getVertexCount(),
            "num_faces": mesh.getFaceCount(),
            "num_hulls": len(hulls),
            "max_convex_hulls": params.max_convex_hulls,
            "all_convex": all(hull_is_convex(hull) for hull in hulls),
            "files_written": files_written,
            "reloaded_matches": reloaded_matches,
            "bounding_box": bounding_box(hulls),
            "mesh_bounding_box": bounding_box([mesh]),
        },
        "default_constructor_equal": signature(default_hulls) == signature(hulls),
        "fill_modes": {
            name: {
                "num_hulls": len(mode_hulls),
                "all_convex": all(hull_is_convex(hull) for hull in mode_hulls),
                "bounding_box": bounding_box(mode_hulls),
            }
            for name, mode_hulls in fill_modes.items()
        },
        "fill_modes_differ": all(
            signatures[i] != signatures[j]
            for i in range(len(signatures))
            for j in range(i + 1, len(signatures))
        ),
        "shipped": {"num_hulls": len(shipped), "bounding_box": bounding_box(shipped)},
    }


def main() -> None:
    """Console entry point (`tesseract_convex_decomposition_example`): run() and print the results."""
    args = sys.argv[1:]
    output = Path(parse_args(args).output)
    result = run(args, output_dir=output.parent.resolve())
    seat = result["seat"]
    print(f"Parameters: {result['parameters']}")
    print(f"Mesh: {seat['num_vertices']} vertices, {seat['num_faces']} faces")
    print(
        f"{seat['num_hulls']} hulls (max {seat['max_convex_hulls']}), all convex: "
        f"{seat['all_convex']}; wrote {seat['files_written']} files as <i>_{output.name}"
    )
    print(f"Default-constructed decomposition identical: {result['default_constructor_equal']}")
    for name, mode in result["fill_modes"].items():
        print(
            f"{name}: {mode['num_hulls']} hulls, box {np.round(mode['bounding_box'], 4).tolist()}"
        )
    print(f"Fill modes give different decompositions: {result['fill_modes_differ']}")
    print(
        f"Mesh box {np.round(seat['mesh_bounding_box'], 4).tolist()}; "
        f"shipped seat_*.stl: {result['shipped']['num_hulls']} hulls, "
        f"box {np.round(result['shipped']['bounding_box'], 4).tolist()}"
    )


if __name__ == "__main__":
    main()
