"""gh-179: ConvexDecomposition / ConvexDecompositionVHACD, VHACDParameters, FillMode.

Upstream convex_decomposition_vhacd.cpp (0.35.0) reads `faces` without bounds checks
and throws a bare std::runtime_error for non-triangle faces, so the binding validates
`faces` first: MalformedFacesError for a count running past the end or an index
outside `vertices`, NonTriangleFaceError for a face that is not a triangle.
"""

import itertools
import re
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    ConvexDecomposition,
    ConvexDecompositionVHACD,
    FillMode,
    MalformedFacesError,
    NonTriangleFaceError,
    VHACDParameters,
)
from tesseract_robotics.tesseract_geometry import ConvexMesh

# m: second cube's offset along x; leaves a 2 m gap, so V-HACD must split the mesh.
CUBE_GAP_OFFSET = 3.0
# Voxel count for the tests: a coarse grid keeps compute fast on two cubes.
TEST_RESOLUTION = 10000
# s: GIL switch interval during the release probe; far longer than compute, so only an
# explicit release lets the main thread run while compute is in progress.
LONG_SWITCH_INTERVAL = 100.0

# nm line of a strong (non-weak) defined global symbol: address, then T/D/B/S/R.
STRONG_DEFINITION = re.compile(r"^[0-9a-fA-F]+ [TDBSR] ")
# The VHACD namespace itself, not names ending in VHACD (ConvexDecompositionVHACD::).
VHACD_NAMESPACE = re.compile(r"(?<![\w:])VHACD::")

# Header defaults, tesseract/collision/vhacd/convex_decomposition_vhacd.h:38-78.
HEADER_DEFAULTS = {
    "max_convex_hulls": 64,
    "resolution": 400000,
    "minimum_volume_percent_error_allowed": 1.0,
    "max_recursion_depth": 10,
    "shrinkwrap": True,
    "fill_mode": FillMode.FLOOD_FILL,
    "max_num_vertices_per_ch": 64,
    "async_ACD": True,
    "min_edge_length": 2,
    "find_best_plane": False,
}
NON_DEFAULTS = {
    "max_convex_hulls": 8,
    "resolution": 20000,
    "minimum_volume_percent_error_allowed": 2.5,
    "max_recursion_depth": 4,
    "shrinkwrap": False,
    "fill_mode": FillMode.RAYCAST_FILL,
    "max_num_vertices_per_ch": 32,
    "async_ACD": False,
    "min_edge_length": 3,
    "find_best_plane": True,
}

# Triangles of the unit cube [0, 1]^3 by corner index (corner = itertools.product order).
_CUBE_TRIANGLES = [
    (0, 1, 3), (0, 3, 2),  # x = 0
    (4, 6, 7), (4, 7, 5),  # x = 1
    (0, 4, 5), (0, 5, 1),  # y = 0
    (2, 3, 7), (2, 7, 6),  # y = 1
    (0, 2, 6), (0, 6, 4),  # z = 0
    (1, 5, 7), (1, 7, 3),  # z = 1
]  # fmt: skip


def _two_cubes():
    corners = [np.array(c, dtype=float) for c in itertools.product((0.0, 1.0), repeat=3)]
    vertices = corners + [c + np.array([CUBE_GAP_OFFSET, 0.0, 0.0]) for c in corners]
    faces = []
    for offset in (0, len(corners)):
        for tri in _CUBE_TRIANGLES:
            faces += [3, *(i + offset for i in tri)]
    return vertices, np.array(faces, dtype=np.int32)


def _params():
    params = VHACDParameters()
    params.resolution = TEST_RESOLUTION
    return params


def test_two_cubes_decompose_into_at_least_two_hulls():
    vertices, faces = _two_cubes()
    hulls = ConvexDecompositionVHACD(_params()).compute(vertices, faces, verbose=False)
    assert len(hulls) >= 2
    assert all(isinstance(h, ConvexMesh) for h in hulls)

    points = np.vstack([np.asarray(v) for h in hulls for v in h.getVertices()])
    expected = np.vstack(vertices)
    extent = expected.max(axis=0) - expected.min(axis=0)
    voxel = float(extent.max()) / TEST_RESOLUTION ** (1.0 / 3.0)
    np.testing.assert_allclose(points.min(axis=0), expected.min(axis=0), atol=voxel)
    np.testing.assert_allclose(points.max(axis=0), expected.max(axis=0), atol=voxel)


def test_parameters_header_defaults():
    params = VHACDParameters()
    for name, value in HEADER_DEFAULTS.items():
        assert getattr(params, name) == value, name


@pytest.mark.parametrize("name", sorted(NON_DEFAULTS))
def test_parameters_round_trip(name):
    params = VHACDParameters()
    setattr(params, name, NON_DEFAULTS[name])
    assert getattr(params, name) == NON_DEFAULTS[name]


def test_fill_mode_native_names():
    assert {m.name for m in FillMode} == {"FLOOD_FILL", "SURFACE_ONLY", "RAYCAST_FILL"}


def test_base_has_no_constructor():
    with pytest.raises(TypeError):
        ConvexDecomposition()


def test_compute_dispatches_through_base():
    decomposition = ConvexDecompositionVHACD(_params())
    assert isinstance(decomposition, ConvexDecomposition)
    vertices, faces = _two_cubes()
    assert len(ConvexDecomposition.compute(decomposition, vertices, faces, verbose=False)) >= 2


def test_default_constructor():
    vertices, faces = _two_cubes()
    assert ConvexDecompositionVHACD().compute(vertices, faces, verbose=False)


def test_compute_releases_gil():
    """With a switch interval far above compute's run time, the main thread can only see the
    worker inside compute if compute drops the GIL. A compute too fast to observe fails, never
    passes vacuously."""
    vertices, faces = _two_cubes()
    decomposition = ConvexDecompositionVHACD(_params())
    started = threading.Event()
    state = {"in_compute": False, "hulls": None}

    def worker():
        state["in_compute"] = True
        started.set()
        state["hulls"] = decomposition.compute(vertices, faces, verbose=False)
        state["in_compute"] = False

    previous = sys.getswitchinterval()
    sys.setswitchinterval(LONG_SWITCH_INTERVAL)
    try:
        thread = threading.Thread(target=worker)
        thread.start()
        started.wait()
        seen_inside = state["in_compute"]
        thread.join()
    finally:
        sys.setswitchinterval(previous)

    assert seen_inside, "main thread never ran while compute was in progress: GIL held"
    assert len(state["hulls"]) >= 2


def test_face_count_past_end_raises():
    vertices, faces = _two_cubes()
    with pytest.raises(MalformedFacesError, match="past the end"):
        ConvexDecompositionVHACD(_params()).compute(vertices, faces[:-1], verbose=False)


def test_face_index_out_of_range_raises():
    vertices, faces = _two_cubes()
    faces = faces.copy()
    faces[1] = len(vertices)
    with pytest.raises(MalformedFacesError, match="vertex index"):
        ConvexDecompositionVHACD(_params()).compute(vertices, faces, verbose=False)


def test_non_triangle_face_raises():
    vertices, _ = _two_cubes()
    quad = np.array([4, 0, 1, 3, 2], dtype=np.int32)
    with pytest.raises(NonTriangleFaceError):
        ConvexDecompositionVHACD(_params()).compute(vertices, quad, verbose=False)


def test_face_errors_are_value_errors():
    assert issubclass(MalformedFacesError, ValueError)
    assert issubclass(NonTriangleFaceError, ValueError)


@pytest.mark.skipif(
    sys.platform == "win32",
    reason="no nm, and a linked .pyd has no symbol table; the binding's #error on "
    "VHACD_GOOGOL_SIZE guards the include order at build time on every platform",
)
def test_extension_has_single_vhacd_copy():
    """The extension must not compile its own V-HACD (convex_decomposition_vhacd.h defines
    ENABLE_VHACD_IMPLEMENTATION): no strong defined VHACD:: symbols, only weak inline ones.

    Oracle checked against a TU that includes convex_decomposition_vhacd.h alone: nm finds
    2894 strong VHACD:: definitions there (macOS, 0.35.0)."""
    import tesseract_robotics.tesseract_collision._tesseract_collision as ext

    proc = subprocess.run(
        ["nm", "-C", str(Path(ext.__file__))], capture_output=True, text=True, check=True
    )
    strong = [
        line
        for line in proc.stdout.splitlines()
        if STRONG_DEFINITION.match(line) and VHACD_NAMESPACE.search(line)
    ]
    assert not strong, f"{len(strong)} strong VHACD:: definitions, e.g. {strong[:5]}"
