"""Dense Eigen arguments accept Python sequences as well as numpy arrays.

nanobind's Eigen caster imports arguments through DLPack or the buffer protocol only, so a
`list` or `tuple` raised `TypeError` at every `Eigen::Vector3d` / `VectorXd` / `Matrix3d`
parameter. `tesseract_nb.h` runs a sequence through `numpy.asarray` when implicit
conversion is allowed; shape and dtype checks stay nanobind's.
"""

import os
from pathlib import Path

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import (
    tesseract_command_language,
    tesseract_common,
    tesseract_environment,
    tesseract_geometry,
    tesseract_kinematics,
)
from tesseract_robotics.tesseract_common import (
    AngleAxisd,
    GeneralResourceLocator,
    Isometry3d,
    JointState,
    Quaterniond,
)

AS_LIST = list
AS_TUPLE = tuple


def as_nested_tuple(rows):
    return tuple(tuple(row) for row in rows)


def as_nested_list(rows):
    return [list(row) for row in rows]


VECTOR_FORMS = pytest.mark.parametrize(
    "form", [AS_LIST, AS_TUPLE, np.array], ids=["list", "tuple", "ndarray"]
)
MATRIX_FORMS = pytest.mark.parametrize(
    "form", [as_nested_list, as_nested_tuple, np.array], ids=["list", "tuple", "ndarray"]
)

XYZ = [1.0, 2.0, 3.0]
JOINTS = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6]
# A 90 degree turn about z, exact in float64.
ROT_Z_90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]


@pytest.fixture(scope="module")
def abb_env():
    env = tesseract_environment.Environment()
    support = Path(os.environ["TESSERACT_SUPPORT_DIR"])
    assert env.init(
        support / "urdf" / "abb_irb2400.urdf",
        support / "urdf" / "abb_irb2400.srdf",
        GeneralResourceLocator(),
    )
    return env


# --- Vector3d (`const Eigen::Vector3d&`) ---


@VECTOR_FORMS
def test_translate_accepts(form):
    iso = Isometry3d.Identity().translate(form(XYZ))
    nptest.assert_array_equal(iso.translation, XYZ)


@VECTOR_FORMS
def test_pretranslate_accepts(form):
    iso = Isometry3d.Identity().pretranslate(form(XYZ))
    nptest.assert_array_equal(iso.translation, XYZ)


@VECTOR_FORMS
def test_isometry_translation_rotation_ctor_accepts(form):
    iso = Isometry3d(form(XYZ), Quaterniond.Identity())
    nptest.assert_array_equal(iso.translation, XYZ)
    nptest.assert_array_equal(iso.linear, np.eye(3))


@VECTOR_FORMS
def test_angle_axis_axis_accepts(form):
    aa = AngleAxisd(0.5, form([0.0, 0.0, 1.0]))
    nptest.assert_array_equal(aa.axis, [0.0, 0.0, 1.0])


# --- Vector4d ---


@VECTOR_FORMS
def test_quaternion_from_vector4_accepts(form):
    q = Quaterniond(form([0.0, 0.0, 0.0, 1.0]))
    assert q.isApprox(Quaterniond.Identity())


# --- VectorXd (`const Eigen::VectorXd&` and `Eigen::Ref<const Eigen::VectorXd>`) ---


@VECTOR_FORMS
def test_joint_state_ctor_accepts(form):
    js = JointState(["a", "b", "c"], form(XYZ))
    nptest.assert_array_equal(js.position, XYZ)


@VECTOR_FORMS
def test_joint_state_position_setter_accepts(form):
    js = JointState()
    js.position = form(XYZ)
    nptest.assert_array_equal(js.position, XYZ)


@VECTOR_FORMS
def test_joint_waypoint_ctor_accepts(form):
    wp = tesseract_command_language.JointWaypoint(["a", "b", "c"], form(XYZ))
    nptest.assert_array_equal(wp.getPosition(), XYZ)


@VECTOR_FORMS
def test_calc_fwd_kin_ref_accepts(abb_env, form):
    jg = abb_env.getJointGroup("manipulator")
    expected = jg.calcFwdKin(np.array(JOINTS))["tool0"].matrix
    nptest.assert_array_equal(jg.calcFwdKin(form(JOINTS))["tool0"].matrix, expected)


@VECTOR_FORMS
def test_set_state_ref_accepts(form):
    env = tesseract_environment.Environment()
    support = Path(os.environ["TESSERACT_SUPPORT_DIR"])
    assert env.init(
        support / "urdf" / "abb_irb2400.urdf",
        support / "urdf" / "abb_irb2400.srdf",
        GeneralResourceLocator(),
    )
    names = list(env.getJointGroup("manipulator").getJointNames())
    env.setState(names, form(JOINTS))
    nptest.assert_array_equal(env.getCurrentJointValues(names), JOINTS)


# --- Matrix3d / Matrix4d ---


@MATRIX_FORMS
def test_quaternion_from_matrix3_accepts(form):
    q = Quaterniond(form(ROT_Z_90))
    nptest.assert_allclose(q.toRotationMatrix(), ROT_Z_90, atol=1e-15)


@MATRIX_FORMS
def test_rotate_matrix3_accepts(form):
    iso = Isometry3d.Identity().rotate(form(ROT_Z_90))
    nptest.assert_array_equal(iso.linear, ROT_Z_90)


@MATRIX_FORMS
def test_isometry_from_matrix4_accepts(form):
    m = np.eye(4)
    m[:3, :3] = ROT_Z_90
    m[:3, 3] = XYZ
    iso = Isometry3d(form(m.tolist()))
    nptest.assert_array_equal(iso.matrix, m)


# --- conversion rules ---


def test_integer_list_converts_to_float64():
    iso = Isometry3d.Identity().translate([1, 2, 3])
    assert iso.translation.dtype == np.float64
    nptest.assert_array_equal(iso.translation, XYZ)


@pytest.mark.parametrize(
    "bad",
    [[1.0, 2.0], (1.0, 2.0, 3.0, 4.0), [[1.0, 2.0, 3.0]], [], [1.0, [2.0], 3.0]],
    ids=["short", "long", "nested", "empty", "ragged"],
)
def test_wrong_shape_raises_type_error(bad):
    with pytest.raises(TypeError):
        Isometry3d.Identity().translate(bad)


@pytest.mark.parametrize(
    "bad",
    [["a", "b", "c"], [None, None, None], [1.0, "2", 3.0], "abc", [1 + 2j, 0, 0]],
    ids=["str-items", "none-items", "mixed", "str", "complex"],
)
def test_non_numeric_raises_type_error(bad):
    with pytest.raises(TypeError):
        Isometry3d.Identity().translate(bad)


def test_wrong_matrix_shape_raises_type_error():
    with pytest.raises(TypeError):
        Quaterniond([[1.0, 0.0], [0.0, 1.0]])
    with pytest.raises(TypeError):
        Isometry3d.Identity().rotate([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def test_wrong_joint_count_still_reaches_upstream_check(abb_env):
    """A 3-element list converts; the binding's own length check then rejects it."""
    names = list(abb_env.getJointGroup("manipulator").getJointNames())
    with pytest.raises(ValueError, match="joint_values length"):
        abb_env.setState(names, [0.0, 0.0, 0.0])


def test_vector_overloads_beside_scalar_overloads():
    """Scalar and `Vector3d` overloads resolve side by side; the latter now takes a list."""
    assert Quaterniond.from_rpy(0.0, 0.0, 0.0).isApprox(Quaterniond.from_rpy([0.0, 0.0, 0.0]))
    nptest.assert_array_equal(tesseract_common.Translation3d(1.0, 2.0, 3.0).translation(), XYZ)
    nptest.assert_array_equal(tesseract_common.Translation3d(XYZ).translation(), XYZ)


# --- integer targets (`Eigen::VectorXi`) and in-place buffers (mutable `Eigen::Ref`) ---

TRIANGLE = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]


def test_integer_list_accepted_for_vectorxi():
    mesh = tesseract_geometry.Mesh(TRIANGLE, [3, 0, 1, 2])
    nptest.assert_array_equal(mesh.getFaces(), [3, 0, 1, 2])


def test_float_list_refused_for_vectorxi():
    """Floats are not truncated into integer face indices."""
    with pytest.raises(TypeError):
        tesseract_geometry.Mesh(TRIANGLE, [3.0, 0.0, 1.5, 2.0])


def test_in_place_ref_refuses_list():
    """`harmonizeTowardZero` writes into its argument; a list could not see the result."""
    qs = [4.0, 0.0]
    with pytest.raises(TypeError):
        tesseract_kinematics.harmonizeTowardZero(qs, [0])
    arr = np.array(qs)
    tesseract_kinematics.harmonizeTowardZero(arr, [0])
    assert arr[0] != qs[0]
