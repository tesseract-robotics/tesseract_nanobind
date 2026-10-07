"""Value equality on the tesseract_common value types (#169).

Each class binds its C++ operator==/operator!= as __eq__/__ne__ and sets __hash__ = None.
ResourceLocator and Resource keep identity equality: their C++ operator== returns true
unconditionally (resource_locator.cpp:46, :206, 0.35.0).
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    BytesResource,
    CollisionMarginData,
    CollisionMarginPairData,
    GeneralResourceLocator,
    Isometry3d,
    JointState,
    KinematicLimits,
    KinematicsPluginInfo,
    ManipulatorInfo,
    PluginInfo,
    PluginInfoContainer,
    Resource,
    ResourceLocator,
    SimpleLocatedResource,
)


def _changed(make, mutate):
    obj = make()
    mutate(obj)
    return obj


def _package_dir(root):
    """Upstream package rule: a directory holding `package.xml` is a package named after it."""
    pkg = root / "my_pkg"
    pkg.mkdir(exist_ok=True)  # make_other runs twice per test
    (pkg / "package.xml").write_text("<package/>")
    return root


def _limits(rows, joint_upper=1.0):
    limits = KinematicLimits()
    limits.resize(rows)  # resize leaves values unset: assign every matrix
    limits.joint_limits = np.tile([-1.0, joint_upper], (rows, 1))
    limits.velocity_limits = np.tile([-2.0, 2.0], (rows, 1))
    limits.acceleration_limits = np.tile([-3.0, 3.0], (rows, 1))
    limits.jerk_limits = np.tile([-4.0, 4.0], (rows, 1))
    return limits


# (make, make_other), both taking a scratch directory: two make() calls give equal objects;
# make_other() differs in one field.
CASES = {
    "AllowedCollisionMatrix": (
        lambda _: AllowedCollisionMatrix(),
        lambda _: _changed(
            AllowedCollisionMatrix, lambda m: m.addAllowedCollision("a", "b", "adjacent")
        ),
    ),
    "BytesResource": (
        lambda _: BytesResource("package://pkg/a.bin", b"abc"),
        lambda _: BytesResource("package://pkg/a.bin", b"abd"),
    ),
    "CollisionMarginData": (lambda _: CollisionMarginData(0.1), lambda _: CollisionMarginData(0.2)),
    "CollisionMarginPairData": (
        lambda _: CollisionMarginPairData(),
        lambda _: _changed(CollisionMarginPairData, lambda d: d.setCollisionMargin("a", "b", 0.1)),
    ),
    "GeneralResourceLocator": (
        lambda _: GeneralResourceLocator(environment_variables=[]),
        lambda tmp: GeneralResourceLocator(paths=[_package_dir(tmp)], environment_variables=[]),
    ),
    "JointState": (
        lambda _: JointState(["j1"], np.array([0.0])),
        lambda _: JointState(["j1"], np.array([1.0])),
    ),
    "KinematicLimits": (lambda _: _limits(2), lambda _: _limits(2, joint_upper=1.5)),
    "KinematicsPluginInfo": (
        lambda _: KinematicsPluginInfo(),
        lambda _: _changed(
            KinematicsPluginInfo, lambda i: setattr(i, "search_paths", ["/opt/plugins"])
        ),
    ),
    "ManipulatorInfo": (
        lambda _: ManipulatorInfo("manipulator", "base_link", "tool0"),
        lambda _: ManipulatorInfo("manipulator", "base_link", "tool1"),
    ),
    "PluginInfo": (
        lambda _: PluginInfo(),
        lambda _: _changed(PluginInfo, lambda p: setattr(p, "class_name", "SomeFactory")),
    ),
    "PluginInfoContainer": (
        lambda _: PluginInfoContainer(),
        lambda _: _changed(
            PluginInfoContainer, lambda c: setattr(c, "default_plugin", "some_plugin")
        ),
    ),
    "SimpleLocatedResource": (
        lambda _: SimpleLocatedResource("package://pkg/a.urdf", "/x/a.urdf"),
        lambda _: SimpleLocatedResource("package://pkg/a.urdf", "/y/a.urdf"),
    ),
}


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_equal_when_built_alike(tmp_path, make, make_other):
    a, b = make(tmp_path), make(tmp_path)
    assert a is not b
    assert a == b
    assert not (a != b)


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_unequal_when_one_field_differs(tmp_path, make, make_other):
    assert make(tmp_path) != make_other(tmp_path)
    assert not (make(tmp_path) == make_other(tmp_path))


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_unhashable(tmp_path, make, make_other):
    obj = make(tmp_path)
    assert type(obj).__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(obj)
    with pytest.raises(TypeError, match="unhashable"):
        {}.setdefault(obj)


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_foreign_operand_is_unequal_without_raising(tmp_path, make, make_other):
    """nanobind returns NotImplemented for a foreign operand; Python falls back to identity."""
    obj = make(tmp_path)
    assert (obj == object()) is False
    assert (obj != object()) is True
    assert obj != 5


def test_sibling_resources_never_compare_equal():
    """Neither operand is the other's class: both __eq__ return NotImplemented, identity decides."""
    br = BytesResource("package://pkg/a.urdf", b"<robot/>")
    slr = SimpleLocatedResource("package://pkg/a.urdf", "/x/a.urdf")
    assert br != slr
    assert slr != br


def test_joint_state_tolerance_comes_from_cpp():
    """JointState::operator== (joint_state.cpp, 0.35.0): sizes exactly, then isApprox(1e-5)."""
    a = JointState(["j1"], np.array([1.0]))
    assert a == JointState(["j1"], np.array([1.0 + 1e-9]))
    assert a != JointState(["j1"], np.array([1.0, 0.0]))


def test_manipulator_info_tcp_offset_kind_matters():
    """A link-name tcp_offset never equals a pose tcp_offset (manipulator_info.cpp compares the variant index)."""
    by_name = ManipulatorInfo("manipulator", "base_link", "tool0", "tcp_link")
    by_pose = ManipulatorInfo("manipulator", "base_link", "tool0", Isometry3d.Identity())
    assert by_name != by_pose


def test_kinematic_limits_of_different_shapes_are_unequal():
    """Upstream isApprox never compares shapes; the binding checks them first instead of reading out of bounds."""
    assert KinematicLimits() != _limits(2)
    assert _limits(2) != _limits(3)
    assert not (_limits(1) == KinematicLimits())


class _LocatorA(ResourceLocator):
    def locateResource(self, url):
        return None


class _LocatorB(ResourceLocator):
    def locateResource(self, url):
        return BytesResource(url, b"b")


@pytest.mark.parametrize("cls", [ResourceLocator, Resource])
def test_trivially_true_operator_not_bound(cls):
    """ResourceLocator/Resource::operator== return true (resource_locator.cpp:46, :206, 0.35.0)."""
    assert "__eq__" not in vars(cls)
    assert "__ne__" not in vars(cls)


def test_python_locators_keep_identity_equality():
    """A bound base operator== would make every pair of Python locators equal."""
    a, b = _LocatorA(), _LocatorB()
    assert a != b
    assert _LocatorA() != _LocatorA()
    assert a == a
    assert {a: 1}[a] == 1  # still hashable, by identity
    assert GeneralResourceLocator(environment_variables=[]) != a
    assert a != GeneralResourceLocator(environment_variables=[])


def test_located_resource_parent_compared_by_presence_only():
    """Upstream compares parents through ResourceLocator::operator==, which is always true."""
    with_a = BytesResource("package://pkg/a.bin", b"x", parent=_LocatorA())
    with_b = BytesResource("package://pkg/a.bin", b"x", parent=_LocatorB())
    without = BytesResource("package://pkg/a.bin", b"x")
    assert with_a == with_b
    assert with_a != without
