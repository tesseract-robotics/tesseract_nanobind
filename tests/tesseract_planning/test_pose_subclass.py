"""A subclass of `Pose` survives compose, inverse and the factories.

A consumer that tags poses (a frame-typed subclass) must get its own type back from every operation
that builds a pose, or each compose would re-wrap the result in Python.
"""

import numpy as np
import pytest

from tesseract_robotics.common import AngleAxisd, Isometry3d, Quaterniond
from tesseract_robotics.planning import Pose


class _Sub(Pose):
    """A subclass with no behaviour of its own."""


def _rigid(angle: float, axis: list[float], translation: list[float]) -> np.ndarray:
    matrix = np.array(Isometry3d(AngleAxisd(angle, np.asarray(axis) / np.linalg.norm(axis))).matrix)
    matrix[:3, 3] = translation
    return matrix


M = _rigid(0.7, [1.0, 2.0, 3.0], [0.1, -0.2, 0.3])
N = _rigid(-1.3, [0.0, 1.0, -1.0], [1.5, 0.25, -0.75])


def test_inverse_keeps_subclass():
    inverse = _Sub(M).inverse()
    assert type(inverse) is _Sub
    assert np.array_equal(inverse.matrix, Isometry3d(M).inverse().matrix)


def test_mul_keeps_subclass():
    product = _Sub(M) * Isometry3d(N)
    assert type(product) is _Sub
    assert np.array_equal(product.matrix, (Isometry3d(M) * Isometry3d(N)).matrix)


def test_matmul_keeps_subclass():
    product = _Sub(M) @ Isometry3d(N)
    assert type(product) is _Sub
    assert np.array_equal(product.matrix, (Isometry3d(M) * Isometry3d(N)).matrix)
    with pytest.raises(TypeError):
        _Sub(M) @ 3  # noqa: B018


def test_mul_non_isometry_unchanged():
    point = _Sub(M) * np.array([1.0, 2.0, 3.0])
    assert isinstance(point, np.ndarray)
    assert point.shape == (3,)
    assert np.array_equal(point, Isometry3d(M) * np.array([1.0, 2.0, 3.0]))
    rotated = _Sub(M) * Quaterniond(AngleAxisd(0.4, np.array([0.0, 0.0, 1.0])))
    expected = Isometry3d(M) * Quaterniond(AngleAxisd(0.4, np.array([0.0, 0.0, 1.0])))
    assert np.array_equal(rotated.matrix, expected.matrix)


@pytest.mark.parametrize(
    "build",
    [
        lambda: _Sub.from_xyz(1.0, 2.0, 3.0),
        lambda: _Sub.from_xyz_quat([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
        lambda: _Sub.from_xyz_quat(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        lambda: _Sub.from_xyz_rpy([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]),
        lambda: _Sub.from_xyz_rpy(1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
        lambda: _Sub.from_matrix_position(np.eye(3), [1.0, 2.0, 3.0]),
    ],
)
def test_factories_keep_subclass(build):
    assert type(build()) is _Sub
