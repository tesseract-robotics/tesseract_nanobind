"""The implicit-SDF collision solver (implicit_sdf_collision_solver.h, #373).

`makeImplicitSDFShape` wraps a primitive in world coordinates; `collideImplicitSDF` finds the
contacts between two implicit shapes, also ones whose distance and gradient are Python callables.
"""

import gc
import sys
import weakref

import numpy as np
import pytest

from tesseract_robotics.collision import (
    ImplicitSDFCollisionConfig,
    ImplicitSDFContact,
    ImplicitSDFShape,
    collideImplicitSDF,
    makeImplicitSDFShape,
)
from tesseract_robotics.common import Isometry3d
from tesseract_robotics.geometry import (
    Mesh,
    SignedDistanceField,
    Sphere,
    createSignedDistanceField,
)

# m: sphere radii and the distance between their centres
R0, R1 = 0.5, 0.3
CENTRE_DISTANCE = 1.0
# m: the solver iterates to a step of `minimum_step` (1e-4 by default); a tenth of that
DISTANCE_ATOL = 1e-5
# m: contact margin, wide enough to report the separated spheres
MARGIN = 0.5
# The field of the lifetime tests: a sphere of radius R0 sampled on a cube of half-edge
# FIELD_HALF_EDGE (m), FIELD_SAMPLES per axis.
FIELD_HALF_EDGE = 1.0
FIELD_SAMPLES = 20
# m: on the x axis, outside the field's sphere and inside its domain
FIELD_PROBE_X = 0.8
# m: the other sphere's x position, so that it overlaps the field's sphere
FIELD_OVERLAP_X = 0.8
# Fields allocated after a drop, enough to reuse a freed field's block; their value (m) is one no
# sphere of this size has anywhere in the domain.
REUSE_FIELDS = 200
REUSE_DISTANCE = 123.0
# m: what a shape reassigned during a call reports, far outside every search box and margin
FAR_DISTANCE = 1000.0


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


def _field_bounds():
    low, high = [-FIELD_HALF_EDGE] * 3, [FIELD_HALF_EDGE] * 3
    return low, high, [FIELD_SAMPLES] * 3


def _sphere_field() -> SignedDistanceField:
    axis = np.linspace(-FIELD_HALF_EDGE, FIELD_HALF_EDGE, FIELD_SAMPLES)
    x, y, z = np.meshgrid(axis, axis, axis, indexing="ij")
    grid = np.sqrt(x**2 + y**2 + z**2) - R0
    return SignedDistanceField(*_field_bounds(), grid.ravel(order="F"))


def _drop_and_reuse() -> list[SignedDistanceField]:
    """Collect dropped objects, then fill freed blocks with fields of another value.

    A shape that only references a freed field can still read its old values from the block, so a
    drop alone can pass. Reusing the block makes such a read visibly wrong, or crashes. The caller
    keeps the result, so the fields stay allocated.
    """
    gc.collect()
    values = np.full(FIELD_SAMPLES**3, REUSE_DISTANCE)
    return [SignedDistanceField(*_field_bounds(), values) for _ in range(REUSE_FIELDS)]


def test_sdf_shape_keeps_its_field_alive():
    """A shape stays correct after its field's last Python reference is gone."""
    point = np.array([FIELD_PROBE_X, 0.0, 0.0])
    other = makeImplicitSDFShape(Sphere(R0), _pose(FIELD_OVERLAP_X))
    field = _sphere_field()
    references = sys.getrefcount(field)
    shape = makeImplicitSDFShape(field, _pose(0.0))
    assert sys.getrefcount(field) > references, "the shape must hold the field"
    distance = shape.distance(point)
    gradient = shape.gradient(point)
    contacts = len(collideImplicitSDF(shape, other))
    assert contacts > 0, "the overlapping spheres must touch while the field is alive"

    del field
    reused = _drop_and_reuse()

    assert shape.distance(point) == distance
    np.testing.assert_array_equal(shape.gradient(point), gradient)
    assert len(collideImplicitSDF(shape, other)) == contacts
    assert reused


def test_sdf_shape_callables_keep_the_field_alive_without_the_shape():
    """`distance` and `gradient` taken from a shape outlive it and the field."""
    point = np.array([FIELD_PROBE_X, 0.0, 0.0])
    field = _sphere_field()
    references = sys.getrefcount(field)
    shape = makeImplicitSDFShape(field, _pose(0.0))
    assert sys.getrefcount(field) > references, "the shape must hold the field"
    distance_of, gradient_of = shape.distance, shape.gradient
    distance, gradient = distance_of(point), gradient_of(point)

    del field, shape
    reused = _drop_and_reuse()

    assert distance_of(point) == distance
    np.testing.assert_array_equal(gradient_of(point), gradient)
    assert reused


class _Sampler:
    """A sampler `weakref.finalize` can watch; a lazy field keeps it for as long as it lives."""

    def __call__(self, point):
        return float(np.linalg.norm(point) - R0)


def test_lazy_field_lives_until_the_last_callable_dies():
    """Liveness read from a finalizer, so no allocator state and no read of freed memory."""
    point = np.array([FIELD_PROBE_X, 0.0, 0.0])
    sampler = _Sampler()
    finalized = []
    weakref.finalize(sampler, finalized.append, True)
    field = createSignedDistanceField(sampler, *_field_bounds())
    del sampler
    shape = makeImplicitSDFShape(field, _pose(0.0))
    distance_of, gradient_of = shape.distance, shape.gradient

    del field, shape
    gc.collect()
    assert not finalized, "the callables must hold the field"
    assert distance_of(point) == pytest.approx(FIELD_PROBE_X - R0)

    del gradient_of
    gc.collect()
    assert not finalized, "`distance` alone must hold the field"

    del distance_of
    gc.collect()
    assert finalized, "the last callable must release the field"


def _replace_members(shape: ImplicitSDFShape) -> None:
    shape.distance = lambda _point: FAR_DISTANCE
    shape.gradient = lambda _point: np.array([1.0, 0.0, 0.0])


def _contact_distances(s0: ImplicitSDFShape, s1: ImplicitSDFShape) -> list[float]:
    return [contact.distance for contact in collideImplicitSDF(s0, s1, _config())]


def test_reassigning_a_shape_during_a_call_does_not_change_that_call():
    """A shape's own `distance` replaces its members mid-call; the call keeps the ones it started with.

    The replacements report no contact anywhere, so a solver that reads them returns another
    answer. The same assignment from a second thread would free what such a solver is calling.
    """
    centre = np.zeros(3)
    other = makeImplicitSDFShape(Sphere(R1), _pose(CENTRE_DISTANCE))
    expected = _contact_distances(_python_sphere(centre, R0), other)
    assert expected, "the spheres must report contacts within the margin"

    shape = _python_sphere(centre, R0)
    original = shape.distance

    def distance(point):
        _replace_members(shape)
        return original(point)

    shape.distance = distance
    assert _contact_distances(shape, other) == pytest.approx(expected, abs=DISTANCE_ATOL)
    assert shape.distance(centre) == FAR_DISTANCE, "the assignment itself must still take effect"


def test_dropping_a_field_during_a_call_does_not_change_that_call():
    """A callback removes the last holders of an SDF shape's field while the solver is using it.

    On a solver that reads the shape in place this replaces the field's closures and frees them
    and the field under it; the replacements make that visible as another answer, not a crash.
    """
    field_shape = makeImplicitSDFShape(_sphere_field(), _pose(0.0))
    other = _python_sphere(np.array([FIELD_OVERLAP_X, 0.0, 0.0]), R0)
    expected = _contact_distances(field_shape, other)
    assert expected, "the overlapping spheres must touch"

    original = other.distance

    def distance(point):
        _replace_members(field_shape)
        return original(point)

    other.distance = distance
    assert _contact_distances(field_shape, other) == pytest.approx(expected, abs=DISTANCE_ATOL)


def test_shape_refuses_none_geometry():
    with pytest.raises(TypeError):
        makeImplicitSDFShape(None, _pose(0.0))  # ty: ignore[invalid-argument-type]


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
