"""The implicit-SDF collision solver (implicit_sdf_collision_solver.h, #373).

`makeImplicitSDFShape` wraps a primitive in world coordinates; `collideImplicitSDF` finds the
contacts between two implicit shapes, also ones whose distance and gradient are Python callables.
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    ImplicitSDFCollisionConfig,
    ImplicitSDFContact,
    ImplicitSDFShape,
    collideImplicitSDF,
    makeImplicitSDFShape,
)
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_geometry import Mesh, Sphere

# m: sphere radii and the distance between their centres
R0, R1 = 0.5, 0.3
CENTRE_DISTANCE = 1.0
# m: the solver iterates to a step of `minimum_step` (1e-4 by default); a tenth of that
DISTANCE_ATOL = 1e-5
# m: contact margin, wide enough to report the separated spheres
MARGIN = 0.5


def _pose(x: float) -> Isometry3d:
    m = np.eye(4)
    m[0, 3] = x
    return Isometry3d(m)


def _config() -> ImplicitSDFCollisionConfig:
    config = ImplicitSDFCollisionConfig()
    config.contact_margin = MARGIN
    return config


def _python_sphere(centre: np.ndarray, radius: float) -> ImplicitSDFShape:
    shape = ImplicitSDFShape()
    shape.distance = lambda p: float(np.linalg.norm(p - centre) - radius)
    shape.gradient = lambda p: (p - centre) / np.linalg.norm(p - centre)
    shape.aabb = (centre - radius, centre + radius)
    return shape


def test_config_defaults():
    config = ImplicitSDFCollisionConfig()
    assert config.initial_sample_count == 16
    assert config.max_iterations == 20
    assert config.max_contacts == 4
    assert config.contact_margin == 0.0
    assert config.minimum_step > 0.0
    assert config.duplicate_tolerance > 0.0


def test_two_spheres_distance():
    s0 = makeImplicitSDFShape(Sphere(R0), _pose(0.0))
    s1 = makeImplicitSDFShape(Sphere(R1), _pose(CENTRE_DISTANCE))
    assert s0.isValid() and s1.isValid()
    lo, hi = s0.aabb
    np.testing.assert_allclose(lo, [-R0] * 3)
    np.testing.assert_allclose(hi, [R0] * 3)
    assert s0.distance(np.zeros(3)) == pytest.approx(-R0)
    contacts = collideImplicitSDF(s0, s1, _config())
    assert contacts
    assert all(isinstance(c, ImplicitSDFContact) for c in contacts)
    expected = CENTRE_DISTANCE - R0 - R1
    assert min(c.distance for c in contacts) == pytest.approx(expected, abs=DISTANCE_ATOL)
    a, b = contacts[0].nearest_points
    assert np.linalg.norm(b - a) == pytest.approx(expected, abs=DISTANCE_ATOL)


def test_python_callables():
    """A shape whose distance and gradient are Python callables gives the same distance."""
    s0 = _python_sphere(np.zeros(3), R0)
    s1 = makeImplicitSDFShape(Sphere(R1), _pose(CENTRE_DISTANCE))
    assert s0.isValid()
    contacts = collideImplicitSDF(s0, s1, _config())
    expected = CENTRE_DISTANCE - R0 - R1
    assert min(c.distance for c in contacts) == pytest.approx(expected, abs=DISTANCE_ATOL)


def test_callable_error_propagates():
    s0 = _python_sphere(np.zeros(3), R0)

    def boom(p):
        raise ZeroDivisionError("boom")

    s0.distance = boom
    s1 = makeImplicitSDFShape(Sphere(R1), _pose(CENTRE_DISTANCE))
    with pytest.raises(ZeroDivisionError):
        collideImplicitSDF(s0, s1, _config())


def test_unsupported_geometry_is_invalid():
    vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    mesh = Mesh(list(vertices), np.array([3, 0, 1, 2], dtype=np.int32))
    assert not makeImplicitSDFShape(mesh, _pose(0.0)).isValid()
    assert not ImplicitSDFShape().isValid()
