"""Tests for the Eigen geometry primitives exposed by `tesseract_common`."""

import math

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics.tesseract_common import (
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    AngleAxisd,
    Hyperplane3d,
    Isometry3d,
    ParametrizedLine3d,
    Quaterniond,
    Translation3d,
)

# Canonical 90° rotations used across several tests — built from
# `AngleAxisd` so the test is self-documenting and never grows
# half-precision quaternion literals like `0.70710678`.
_ROT_90_Z = Quaterniond(AngleAxisd(math.pi / 2, Z_AXIS))
_ROT_90_X = Quaterniond(AngleAxisd(math.pi / 2, X_AXIS))


def test_repr_isometry3d():
    iso = Isometry3d.Identity()
    r = repr(iso)
    assert r.startswith("Isometry3d(")
    assert "translation=[0, 0, 0]" in r
    # Project canonical: scalar-last [qx, qy, qz, qw] in repr.
    assert "quaternion=[x=0, y=0, z=0, w=1]" in r


def test_repr_translation3d():
    assert repr(Translation3d(1.0, 2.0, 3.0)) == "Translation3d(1, 2, 3)"


def test_repr_quaterniond():
    # Project canonical: scalar-last [qx, qy, qz, qw] order.
    assert repr(Quaterniond.from_xyzw(0.0, 0.0, 0.0, 1.0)) == "Quaterniond(x=0, y=0, z=0, w=1)"


def test_repr_angleaxisd():
    aa = AngleAxisd(math.pi / 2, Z_AXIS)
    r = repr(aa)
    assert r.startswith("AngleAxisd(")
    assert "axis=[0, 0, 1]" in r


def test_isapprox_isometry3d():
    a = Isometry3d.Identity()
    b = Isometry3d.Identity()
    assert a.isApprox(b)

    # Perturbation below default precision is "approx"; above is not.
    sub_precision = np.array([1e-15, 0.0, 0.0])
    a.translate(sub_precision)
    assert a.isApprox(b)
    assert not a.isApprox(b, prec=1e-20)


def test_isapprox_quaterniond_component_wise():
    """isApprox is component-wise; q and -q (same rotation) are NOT approx."""
    q = _ROT_90_Z
    q_neg = Quaterniond.from_xyzw(-q.x, -q.y, -q.z, -q.w)
    assert not q.isApprox(q_neg)
    # ...but they describe the same rotation, so geodesic distance is zero.
    assert q.angularDistance(q_neg) == pytest.approx(0.0, abs=1e-6)


def test_isapprox_angleaxisd_translation3d():
    aa1 = AngleAxisd(1.0, Z_AXIS)
    aa2 = AngleAxisd(1.0 + 1e-15, Z_AXIS)
    assert aa1.isApprox(aa2)

    t1 = Translation3d(1.0, 2.0, 3.0)
    t2 = Translation3d(1.0, 2.0, 3.0 + 1e-15)
    assert t1.isApprox(t2)


def test_isometry3d_canonical_ctor_vector_quat():
    iso = Isometry3d(np.array([1.0, 2.0, 3.0]), Quaterniond.Identity())
    nptest.assert_allclose(iso.translation(), [1.0, 2.0, 3.0])
    nptest.assert_allclose(iso.rotation(), np.eye(3))


def test_isometry3d_canonical_ctor_translation_quat():
    iso = Isometry3d(Translation3d(4.0, 5.0, 6.0), _ROT_90_X)
    nptest.assert_allclose(iso.translation(), [4.0, 5.0, 6.0])
    # 90° around X sends +Y to +Z.
    nptest.assert_allclose(iso.linear() @ Y_AXIS, Z_AXIS, atol=1e-12)


def test_isometry3d_fluent_mutators():
    iso = Isometry3d.Identity().translate(X_AXIS).rotate(_ROT_90_Z)
    # `translate` runs before `rotate`, and rotation happens about the
    # already-translated origin, leaving the translation vector unchanged.
    nptest.assert_allclose(iso.translation(), X_AXIS, atol=1e-12)


def test_isometry3d_pretranslate_vs_translate():
    """`translate` post-multiplies (local frame); `pretranslate` pre-multiplies (global)."""
    a = Isometry3d.Identity().rotate(_ROT_90_Z).translate(X_AXIS)
    # +X in the rotated local frame is +Y in the global frame.
    nptest.assert_allclose(a.translation(), Y_AXIS, atol=1e-12)

    b = Isometry3d.Identity().rotate(_ROT_90_Z).pretranslate(X_AXIS)
    nptest.assert_allclose(b.translation(), X_AXIS, atol=1e-12)


def test_isometry3d_rotate_overloads():
    aa = AngleAxisd(math.pi / 2, Z_AXIS)
    q = Quaterniond(aa)
    R = aa.toRotationMatrix()

    iso_q = Isometry3d.Identity().rotate(q)
    iso_aa = Isometry3d.Identity().rotate(aa)
    iso_R = Isometry3d.Identity().rotate(R)
    assert iso_q.isApprox(iso_aa)
    assert iso_q.isApprox(iso_R)


def test_quaterniond_from_coeffs_vector4d():
    """Coeffs ctor reads (x, y, z, w) — Eigen-internal layout. Round-trip via coeffs()."""
    source = _ROT_90_Z
    coeffs = source.coeffs()  # (x, y, z, w)
    recovered = Quaterniond(coeffs)
    assert recovered.isApprox(source)
    nptest.assert_allclose(recovered.coeffs(), coeffs)


def test_quaterniond_vec_and_squarednorm():
    # Project canonical scalar-last [qx, qy, qz, qw].
    q = Quaterniond.from_xyzw(0.1, 0.2, 0.3, 0.5)
    nptest.assert_allclose(q.vec(), [0.1, 0.2, 0.3])
    expected_sq = 0.1**2 + 0.2**2 + 0.3**2 + 0.5**2
    assert q.squaredNorm() == pytest.approx(expected_sq)


def test_angleaxisd_from_matrix3d():
    aa_original = AngleAxisd(math.pi / 3, Z_AXIS)
    aa_recovered = AngleAxisd(aa_original.toRotationMatrix())
    assert aa_recovered.angle() == pytest.approx(math.pi / 3)
    nptest.assert_allclose(aa_recovered.axis(), Z_AXIS, atol=1e-12)


@pytest.mark.parametrize("axis", [X_AXIS, Y_AXIS, Z_AXIS])
def test_axis_constants_are_readonly(axis):
    """Frozen constants — the same arrays are reused everywhere, so they must not be mutable."""
    with pytest.raises(ValueError):
        axis[0] = 99.0


def test_quaterniond_euler_angles_zyx_roundtrip():
    """Build R = Rz(yaw)·Ry(pitch)·Rx(roll), extract via Eigen's eulerAngles('ZYX')."""
    roll, pitch, yaw = 0.5, -0.4, 0.3  # negative pitch — non-trivial range
    q = (
        Quaterniond(AngleAxisd(yaw, Z_AXIS))
        * Quaterniond(AngleAxisd(pitch, Y_AXIS))
        * Quaterniond(AngleAxisd(roll, X_AXIS))
    )
    extracted = q.eulerAngles("ZYX")
    nptest.assert_allclose(extracted, [yaw, pitch, roll], atol=1e-12)


def test_quaterniond_euler_angles_case_insensitive():
    q = Quaterniond(AngleAxisd(math.pi / 4, Z_AXIS))
    nptest.assert_allclose(q.eulerAngles("zyx"), q.eulerAngles("ZYX"))


def test_quaterniond_euler_angles_rejects_bad_length():
    q = Quaterniond.Identity()
    with pytest.raises(ValueError):
        q.eulerAngles("ZY")
    with pytest.raises(ValueError):
        q.eulerAngles("ZYXX")


def test_quaterniond_euler_angles_rejects_invalid_axis():
    q = Quaterniond.Identity()
    with pytest.raises(ValueError, match="invalid axis 'W'"):
        q.eulerAngles("ZYW")


@pytest.mark.parametrize("order", ["XXY", "XYY", "ZZZ", "YXX"])
def test_quaterniond_euler_angles_rejects_repeated_adjacent_axes(order):
    """Tait-Bryan and proper-Euler both require adjacent axes to differ."""
    q = Quaterniond.Identity()
    with pytest.raises(ValueError, match="adjacent axes to differ"):
        q.eulerAngles(order)


def test_quaterniond_euler_angles_accepts_proper_euler():
    """Proper Euler (first == last, adjacent differ) is well-defined; must not be rejected."""
    q = Quaterniond(AngleAxisd(math.pi / 4, Z_AXIS))
    q.eulerAngles("ZYZ")  # no raise expected


# ---------- Defensive design: no aliasing, no uninit memory ----------


def test_isometry3d_default_ctor_is_identity():
    """Bare `Isometry3d()` must produce identity, not Eigen's uninit memory."""
    iso = Isometry3d()
    assert iso.isApprox(Isometry3d.Identity())


def test_isometry3d_copy_ctor_does_not_alias():
    """`Isometry3d(other)` must deep-copy; mutating `other` after must not affect the copy."""
    original = Isometry3d.Identity()
    original.translate(np.array([1.0, 0.0, 0.0]))
    copy = Isometry3d(original)
    original.translate(np.array([10.0, 0.0, 0.0]))
    nptest.assert_allclose(copy.translation(), [1.0, 0.0, 0.0], atol=1e-12)
    nptest.assert_allclose(original.translation(), [11.0, 0.0, 0.0], atol=1e-12)


def test_quaterniond_from_non_orthonormal_matrix_rejected():
    """Constructing a quaternion from a non-rotation matrix must fail loudly."""
    with pytest.raises(ValueError, match="not orthonormal"):
        Quaterniond(np.eye(3) * 2.0)  # scaling masquerading as rotation
    with pytest.raises(ValueError, match="not orthonormal"):
        Quaterniond(np.array([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))  # shear


def test_quaterniond_from_valid_rotation_matrix_accepted():
    """Round-trip through rotation matrix must not trip the orthonormality check."""
    q_in = Quaterniond(AngleAxisd(math.pi / 3, X_AXIS))
    R = q_in.toRotationMatrix()
    q_out = Quaterniond(R)
    assert q_out.angularDistance(q_in) == pytest.approx(0.0, abs=1e-12)


# ---------- Hyperplane3d ----------


def test_hyperplane_normal_offset_ctor():
    plane = Hyperplane3d(Z_AXIS, -5.0)  # z = 5
    nptest.assert_allclose(plane.normal(), Z_AXIS)
    assert plane.offset() == -5.0
    # Signed distance of origin to z = 5: −5 (origin is below the plane along +Z).
    assert plane.signedDistance(np.zeros(3)) == pytest.approx(-5.0)


def test_hyperplane_normal_point_ctor():
    """Hyperplane(normal, point) places the plane *through* the point."""
    plane = Hyperplane3d(Z_AXIS, np.array([0.0, 0.0, 5.0]))
    assert plane.signedDistance(np.array([0.0, 0.0, 5.0])) == pytest.approx(0.0, abs=1e-12)
    assert plane.signedDistance(np.array([0.0, 0.0, 5.0]) + Z_AXIS) == pytest.approx(1.0)


def test_hyperplane_through_three_points_right_hand_rule():
    """Through(p0, p1, p2) MUST follow the right-hand rule: (p1 − p0) × (p2 − p0).

    The binding swaps Eigen's internal operand order to enforce this
    convention. Locking it down so a future binding refactor can't quietly
    re-expose the upstream quirk.
    """
    # XY plane traversed counter-clockwise from origin → normal = +Z.
    plane = Hyperplane3d.Through(np.zeros(3), X_AXIS, Y_AXIS)
    nptest.assert_allclose(plane.normal(), Z_AXIS, atol=1e-12)
    # Reversing the last two points flips the normal.
    plane_flipped = Hyperplane3d.Through(np.zeros(3), Y_AXIS, X_AXIS)
    nptest.assert_allclose(plane_flipped.normal(), -Z_AXIS, atol=1e-12)
    # General case: normal direction matches (p1 − p0) × (p2 − p0).
    p0 = np.array([1.0, 1.0, 1.0])
    p1 = np.array([2.0, 1.0, 1.0])
    p2 = np.array([1.0, 2.0, 1.0])
    expected_normal = np.cross(p1 - p0, p2 - p0)
    expected_normal /= np.linalg.norm(expected_normal)
    nptest.assert_allclose(Hyperplane3d.Through(p0, p1, p2).normal(), expected_normal, atol=1e-12)


def test_hyperplane_signed_vs_abs_distance():
    plane = Hyperplane3d(Z_AXIS, 0.0)  # z = 0
    above = np.array([1.0, 2.0, 3.0])
    below = np.array([1.0, 2.0, -3.0])
    assert plane.signedDistance(above) == pytest.approx(3.0)
    assert plane.signedDistance(below) == pytest.approx(-3.0)
    assert plane.absDistance(below) == pytest.approx(3.0)


def test_hyperplane_projection_drops_normal_component():
    plane = Hyperplane3d(Z_AXIS, 0.0)
    nptest.assert_allclose(
        plane.projection(np.array([1.0, 2.0, 5.0])),
        [1.0, 2.0, 0.0],
        atol=1e-12,
    )


def test_hyperplane_normalize_makes_distance_methods_euclidean():
    """`normalize` rescales BOTH normal and offset by ||normal||.

    Geometry (the set of points on the plane) is preserved, but Eigen's
    `signedDistance` / `absDistance` / `projection` evaluate `normal · p + offset`
    directly — so they only return TRUE Euclidean distances after normalisation.
    """
    plane_raw = Hyperplane3d(2.0 * Z_AXIS, -10.0)  # same plane geometrically as (Z, −5)
    point = np.array([0.0, 0.0, 7.0])
    # Pre-normalisation: signedDistance = (2·7) + (−10) = 4, which is 2× the truth.
    assert plane_raw.signedDistance(point) == pytest.approx(4.0)

    plane_raw.normalize()
    nptest.assert_allclose(np.linalg.norm(plane_raw.normal()), 1.0, atol=1e-12)
    # Post-normalisation: signedDistance is the actual Euclidean distance.
    assert plane_raw.signedDistance(point) == pytest.approx(2.0)
    # And the same plane is unchanged from one built normalised in the first place.
    plane_ref = Hyperplane3d(Z_AXIS, -5.0)
    assert plane_raw.isApprox(plane_ref)


def test_hyperplane_coeffs_layout():
    plane = Hyperplane3d(Z_AXIS, -5.0)
    nptest.assert_allclose(plane.coeffs(), [0.0, 0.0, 1.0, -5.0])


def test_hyperplane_isapprox():
    a = Hyperplane3d(Z_AXIS, 0.0)
    b = Hyperplane3d(Z_AXIS, 1e-15)
    assert a.isApprox(b)
    assert not a.isApprox(b, prec=1e-20)


def test_hyperplane_repr():
    r = repr(Hyperplane3d(Z_AXIS, -5.0))
    assert "Hyperplane3d(" in r
    assert "normal=[0, 0, 1]" in r
    assert "offset=-5" in r


# ---------- ParametrizedLine3d ----------


def test_parametrized_line_origin_direction_ctor():
    line = ParametrizedLine3d(np.array([1.0, 2.0, 3.0]), Z_AXIS)
    nptest.assert_allclose(line.origin(), [1.0, 2.0, 3.0])
    nptest.assert_allclose(line.direction(), Z_AXIS)


def test_parametrized_line_through_normalises_direction():
    """`Through(p0, p1)` stores `(p1 − p0).normalized()`, NOT the raw difference.

    Distinct from the direct ctor `ParametrizedLine3d(origin, direction)` which
    keeps whatever direction you pass. Locking the asymmetry into a test so
    the convention can't drift silently.
    """
    line = ParametrizedLine3d.Through(np.zeros(3), np.array([3.0, 0.0, 0.0]))
    nptest.assert_allclose(line.origin(), np.zeros(3))
    nptest.assert_allclose(line.direction(), X_AXIS)  # magnitude 1, not 3
    # Direct ctor with the same direction does NOT normalise:
    raw = ParametrizedLine3d(np.zeros(3), np.array([3.0, 0.0, 0.0]))
    nptest.assert_allclose(raw.direction(), [3.0, 0.0, 0.0])


def test_parametrized_line_pointAt():
    line = ParametrizedLine3d(np.array([1.0, 2.0, 3.0]), Z_AXIS)
    nptest.assert_allclose(line.pointAt(0.0), [1.0, 2.0, 3.0])
    nptest.assert_allclose(line.pointAt(4.0), [1.0, 2.0, 7.0])


def test_parametrized_line_distance_requires_unit_direction():
    """Eigen's `distance()` assumes unit direction.

    For a point with a component ALONG the direction, a non-unit direction
    produces a wildly wrong "distance" — the formula `||diff − (dir·diff)·dir||`
    scales the parallel-projection term by ||dir||² instead of dividing it
    out. We assert the exact wrong value so a future Eigen rewrite to the
    division form would fail this test rather than silently start agreeing.

    Hand calculation for line `(origin=0, direction=2·Ẑ)`, point `(1, 0, 5)`:
        diff           = (1, 0, 5)
        direction·diff = 2 · 5 = 10
        diff − 10·dir  = (1, 0, 5) − (0, 0, 20) = (1, 0, −15)
        ||·||          = √(1 + 0 + 225) = √226
    """
    line_unit = ParametrizedLine3d(np.zeros(3), Z_AXIS)
    line_2x = ParametrizedLine3d(np.zeros(3), 2.0 * Z_AXIS)
    p = np.array([1.0, 0.0, 5.0])  # true Euclidean distance to Z-axis is 1.0
    assert line_unit.distance(p) == pytest.approx(1.0, abs=1e-12)
    assert line_2x.distance(p) == pytest.approx(math.sqrt(226.0), abs=1e-12)


def test_parametrized_line_projection_on_unit_line():
    line = ParametrizedLine3d(np.zeros(3), Z_AXIS)
    nptest.assert_allclose(line.projection(np.array([1.0, 2.0, 5.0])), [0.0, 0.0, 5.0], atol=1e-12)


def test_parametrized_line_intersects_plane():
    line = ParametrizedLine3d(np.array([1.0, 2.0, -3.0]), Z_AXIS)
    plane_xy = Hyperplane3d(Z_AXIS, 0.0)
    assert line.intersectionParameter(plane_xy) == pytest.approx(3.0)
    nptest.assert_allclose(line.intersectionPoint(plane_xy), [1.0, 2.0, 0.0], atol=1e-12)


def test_parametrized_line_parallel_to_plane_returns_non_finite():
    """Line parallel to plane → divide-by-zero in t; Eigen returns ±inf or NaN, not an exception."""
    line = ParametrizedLine3d(np.array([0.0, 0.0, 5.0]), X_AXIS)  # along +X, z=5
    plane_xy = Hyperplane3d(Z_AXIS, 0.0)
    t = line.intersectionParameter(plane_xy)
    assert not math.isfinite(t)


def test_parametrized_line_isapprox():
    """Eigen's `isApprox` is RELATIVE — comparing against a zero origin breaks down
    because the relative-tolerance denominator vanishes. Use a non-zero origin to
    exercise the well-defined case.
    """
    base = np.array([1.0, 2.0, 3.0])
    a = ParametrizedLine3d(base, Z_AXIS)
    b = ParametrizedLine3d(base + np.array([1e-15, 0.0, 0.0]), Z_AXIS)
    assert a.isApprox(b)
    assert not a.isApprox(b, prec=1e-20)


def test_parametrized_line_repr():
    r = repr(ParametrizedLine3d(np.array([1.0, 2.0, 3.0]), Z_AXIS))
    assert "ParametrizedLine3d(" in r
    assert "origin=[1, 2, 3]" in r
    assert "direction=[0, 0, 1]" in r


# ---------- Degenerate-input rejection (same defensive principle as Quaterniond) ----------


def test_hyperplane_rejects_zero_normal():
    with pytest.raises(ValueError, match="zero-magnitude"):
        Hyperplane3d(np.zeros(3), 1.0)
    with pytest.raises(ValueError, match="zero-magnitude"):
        Hyperplane3d(np.zeros(3), np.array([1.0, 2.0, 3.0]))


@pytest.mark.parametrize(
    "p0, p1, p2",
    [
        # Three coincident points.
        (np.zeros(3), np.zeros(3), np.zeros(3)),
        # Two coincident points, third elsewhere.
        (np.zeros(3), np.zeros(3), X_AXIS),
        # Three points strictly along the X axis (collinear, distinct).
        (np.zeros(3), X_AXIS, 2.0 * X_AXIS),
    ],
)
def test_hyperplane_through_rejects_collinear_points(p0, p1, p2):
    with pytest.raises(ValueError, match="collinear"):
        Hyperplane3d.Through(p0, p1, p2)


def test_parametrized_line_rejects_zero_direction():
    with pytest.raises(ValueError, match="zero-magnitude"):
        ParametrizedLine3d(np.zeros(3), np.zeros(3))


def test_parametrized_line_through_rejects_coincident_points():
    with pytest.raises(ValueError, match="coincident"):
        ParametrizedLine3d.Through(np.array([1.0, 2.0, 3.0]), np.array([1.0, 2.0, 3.0]))


# ---------- transform() under an isometry ----------


def test_hyperplane_transform_translates_and_rotates():
    """Translating the XY plane up by 5 along Z gives the plane z = 5; rotating it
    90° about X (so +Z normal swings to +Y) gives the plane y = 5 (after translate).
    """
    plane = Hyperplane3d(Z_AXIS, 0.0)  # z = 0
    tf = Isometry3d(np.array([0.0, 0.0, 5.0]), Quaterniond.Identity())  # translate +5Z
    plane.transform(tf)
    nptest.assert_allclose(plane.normal(), Z_AXIS, atol=1e-12)
    assert plane.signedDistance(np.zeros(3)) == pytest.approx(-5.0)
    assert plane.signedDistance(np.array([0.0, 0.0, 5.0])) == pytest.approx(0.0, abs=1e-12)


def test_hyperplane_transform_preserves_membership():
    """A point on the plane stays on the plane after any rigid transform applied to both."""
    plane = Hyperplane3d.Through(np.zeros(3), X_AXIS, Y_AXIS)  # z = 0
    point_on = np.array([3.0, 4.0, 0.0])
    assert plane.signedDistance(point_on) == pytest.approx(0.0, abs=1e-12)

    tf = Isometry3d(
        np.array([1.0, 2.0, 3.0]),
        Quaterniond(AngleAxisd(math.pi / 5, X_AXIS)),
    )
    transformed_point = tf.matrix()[:3, :3] @ point_on + tf.translation()
    plane.transform(tf)
    assert plane.signedDistance(transformed_point) == pytest.approx(0.0, abs=1e-12)


def test_parametrized_line_transform_round_trip():
    """Applying a transform then its inverse returns the line to its starting state."""
    line = ParametrizedLine3d(np.array([1.0, 2.0, 3.0]), Z_AXIS)
    origin_before = line.origin().copy()
    direction_before = line.direction().copy()

    tf = Isometry3d(
        np.array([5.0, -1.0, 2.0]),
        Quaterniond(AngleAxisd(math.pi / 3, Y_AXIS)),
    )
    line.transform(tf)
    line.transform(tf.inverse())

    nptest.assert_allclose(line.origin(), origin_before, atol=1e-12)
    nptest.assert_allclose(line.direction(), direction_before, atol=1e-12)


def test_parametrized_line_transform_translates_origin_only():
    """A pure translation moves the origin but leaves the direction unchanged."""
    line = ParametrizedLine3d(np.zeros(3), Z_AXIS)
    tf = Isometry3d(np.array([1.0, 2.0, 3.0]), Quaterniond.Identity())
    line.transform(tf)
    nptest.assert_allclose(line.origin(), [1.0, 2.0, 3.0], atol=1e-12)
    nptest.assert_allclose(line.direction(), Z_AXIS, atol=1e-12)
