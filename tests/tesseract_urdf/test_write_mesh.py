"""#220: tesseract_urdf.writeMeshToFile (tesseract/urdf/utils.h:30)."""

import os
from pathlib import Path

import pytest

from tesseract_robotics import tesseract_geometry, tesseract_urdf

SPHERE_STL = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "meshes/sphere_p25m.stl"


def _sphere():
    (mesh,) = tesseract_geometry.createMeshFromPath(str(SPHERE_STL))
    return mesh


def test_write_mesh_round_trips(tmp_path):
    mesh = _sphere()
    out = tmp_path / "m.ply"
    tesseract_urdf.writeMeshToFile(mesh, str(out))
    (back,) = tesseract_geometry.createMeshFromPath(str(out))
    assert (back.getVertexCount(), back.getFaceCount()) == (
        mesh.getVertexCount(),
        mesh.getFaceCount(),
    )


def test_write_mesh_is_always_ply(tmp_path):
    """urdf/src/utils.cpp:140-155 writes a PLY file whatever the extension."""
    out = tmp_path / "m.stl"
    tesseract_urdf.writeMeshToFile(_sphere(), str(out))
    assert out.read_bytes().startswith(b"ply")


def test_write_mesh_refuses_none(tmp_path):
    with pytest.raises(TypeError):
        tesseract_urdf.writeMeshToFile(None, str(tmp_path / "m.ply"))


def test_write_mesh_missing_directory_raises_runtime_error(tmp_path):
    with pytest.raises(RuntimeError, match="Could not export file"):
        tesseract_urdf.writeMeshToFile(_sphere(), str(tmp_path / "missing" / "m.ply"))
