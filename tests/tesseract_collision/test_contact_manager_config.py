"""Tests for ContactManagerConfig (acm / modify_object_enabled), CollisionCheckConfig
(exit_condition), ContactAllowedValidator bindings, and the manager methods that apply
them (applyContactManagerConfig, set/getContactAllowedValidator, incrementCollisionMargin,
setCollisionMarginPairData) plus the continuous-manager methods that were previously
missing (addCollisionObject, cast transforms, etc.).
"""

import gc
import os

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    ACMOverrideType,
    CollisionCheckConfig,
    CollisionCheckExitType,
    CollisionCheckProgramType,
    CollisionEvaluatorType,
    ContactManagerConfig,
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResultMap,
    ContactTestType_ALL,
)
from tesseract_robotics.tesseract_common import (
    ACMContactAllowedValidator,
    AllowedCollisionMatrix,
    CollisionMarginData,
    CollisionMarginPairData,
    CollisionMarginPairOverrideType,
    CombinedContactAllowedValidator,
    CombinedContactAllowedValidatorType,
    Isometry3d,
    VectorIsometry3d,
    _FilesystemPath,
)
from tesseract_robotics.tesseract_geometry import Box, GeometriesConst

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


# --------------------------------------------------------------------------- #
# Plain-config / type bindings (no manager required)
# --------------------------------------------------------------------------- #
def test_collision_check_exit_type_enum():
    assert CollisionCheckExitType.FIRST is not None
    assert CollisionCheckExitType.ONE_PER_STEP is not None
    assert CollisionCheckExitType.ALL is not None


def test_collision_check_config_exit_condition():
    cfg = CollisionCheckConfig()
    # Default per C++ header is FIRST
    assert cfg.exit_condition == CollisionCheckExitType.FIRST
    cfg.exit_condition = CollisionCheckExitType.ALL
    assert cfg.exit_condition == CollisionCheckExitType.ALL


def test_collision_check_config_full_ctor():
    cfg = CollisionCheckConfig(
        ContactRequest(),
        CollisionEvaluatorType.CONTINUOUS,
        0.01,
        CollisionCheckProgramType.ALL_EXCEPT_START,
        CollisionCheckExitType.ONE_PER_STEP,
    )
    assert cfg.type == CollisionEvaluatorType.CONTINUOUS
    assert cfg.longest_valid_segment_length == pytest.approx(0.01)
    assert cfg.check_program_mode == CollisionCheckProgramType.ALL_EXCEPT_START
    assert cfg.exit_condition == CollisionCheckExitType.ONE_PER_STEP


def test_contact_manager_config_acm_and_modify_object_enabled():
    cfg = ContactManagerConfig()

    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("box_link", "cone_link", "unit_test")
    cfg.acm = acm
    cfg.acm_override_type = ACMOverrideType.OR
    assert cfg.acm.isCollisionAllowed("box_link", "cone_link")
    assert cfg.acm_override_type == ACMOverrideType.OR

    cfg.modify_object_enabled = {"thin_box_link": True, "cone_link": False}
    assert dict(cfg.modify_object_enabled) == {"thin_box_link": True, "cone_link": False}


def test_contact_manager_config_margin_helpers():
    cfg = ContactManagerConfig(0.05)
    cfg.pair_margin_data = CollisionMarginPairData()
    cfg.pair_margin_override_type = CollisionMarginPairOverrideType.MODIFY
    # Should not raise
    cfg.incrementMargins(0.01)
    cfg.scaleMargins(2.0)
    cfg.validate()


def test_contact_allowed_validator_types():
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("a", "b", "unit_test")

    validator = ACMContactAllowedValidator(acm)
    assert validator("a", "b") is True
    assert validator("a", "c") is False

    combined = CombinedContactAllowedValidator([validator], CombinedContactAllowedValidatorType.OR)
    assert combined("a", "b") is True
    assert combined("a", "c") is False


# --------------------------------------------------------------------------- #
# Manager-backed tests
# --------------------------------------------------------------------------- #
def _box(size=1.0):
    box = Box(size, size, size)
    shapes = GeometriesConst()
    shapes.append(box)
    poses = VectorIsometry3d()
    poses.append(Isometry3d(np.eye(4)))
    return shapes, poses


def _two_overlapping_boxes(checker):
    """Add two unit boxes that overlap at the origin."""
    shapes_a, poses_a = _box()
    shapes_b, poses_b = _box()
    checker.addCollisionObject("box_a", 0, shapes_a, poses_a)
    checker.addCollisionObject("box_b", 0, shapes_b, poses_b)
    checker.setActiveCollisionObjects(["box_a", "box_b"])
    checker.setCollisionMarginData(CollisionMarginData(0.1))


def _get_discrete_factory():
    support_dir = os.environ["TESSERACT_SUPPORT_DIR"]
    cfg = _FilesystemPath(support_dir + "/urdf/contact_manager_plugins.yaml")
    locator = TesseractSupportResourceLocator()
    return ContactManagersPluginFactory(cfg, locator), locator


def _num_contacts(checker):
    result = ContactResultMap()
    checker.contactTest(result, ContactRequest(ContactTestType_ALL))
    return result.count()


def test_apply_contact_manager_config_acm_suppresses_contact():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        _two_overlapping_boxes(checker)
        # Without an ACM the two overlapping boxes are in contact
        assert _num_contacts(checker) > 0

        # Apply a config whose ACM allows box_a <-> box_b collision; ASSIGN replaces
        # the manager's validator entirely with one generated from the ACM.
        cfg = ContactManagerConfig()
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("box_a", "box_b", "unit_test")
        cfg.acm = acm
        cfg.acm_override_type = ACMOverrideType.ASSIGN
        checker.applyContactManagerConfig(cfg)

        assert _num_contacts(checker) == 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_set_contact_allowed_validator_suppresses_contact():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        _two_overlapping_boxes(checker)
        assert _num_contacts(checker) > 0

        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("box_a", "box_b", "unit_test")
        validator = ACMContactAllowedValidator(acm)
        checker.setContactAllowedValidator(validator)

        assert checker.getContactAllowedValidator() is not None
        assert _num_contacts(checker) == 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_modify_object_enabled_disables_object():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        _two_overlapping_boxes(checker)
        assert _num_contacts(checker) > 0

        cfg = ContactManagerConfig()
        cfg.modify_object_enabled = {"box_b": False}
        checker.applyContactManagerConfig(cfg)

        assert not checker.isCollisionObjectEnabled("box_b")
        assert _num_contacts(checker) == 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_discrete_increment_and_pair_margin():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        shapes_a, poses_a = _box()
        checker.addCollisionObject("box_a", 0, shapes_a, poses_a)
        checker.setCollisionMarginData(CollisionMarginData(0.0))

        checker.incrementCollisionMargin(0.25)
        assert checker.getCollisionMarginData().getMaxCollisionMargin() == pytest.approx(0.25)

        pair_data = CollisionMarginPairData()
        # Should not raise; REPLACE is the binding's default override type.
        checker.setCollisionMarginPairData(pair_data)
        checker.setCollisionMarginPairData(pair_data, CollisionMarginPairOverrideType.MODIFY)
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_discrete_set_transform_vector_overload():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        shapes_a, poses_a = _box()
        shapes_b, poses_b = _box()
        checker.addCollisionObject("box_a", 0, shapes_a, poses_a)
        checker.addCollisionObject("box_b", 0, shapes_b, poses_b)
        checker.setActiveCollisionObjects(["box_a", "box_b"])
        checker.setCollisionMarginData(CollisionMarginData(0.1))

        # Move box_b far away using the (names, poses) vector overload
        far = np.eye(4)
        far[0][3] = 10.0
        poses = [Isometry3d(np.eye(4)), Isometry3d(far)]
        checker.setCollisionObjectsTransform(["box_a", "box_b"], poses)

        assert _num_contacts(checker) == 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_continuous_manager_add_object_and_cast_transform():
    factory, locator = _get_discrete_factory()
    checker = factory.createContinuousContactManager("BulletCastBVHManager")
    try:
        # addCollisionObject / getCollisionObjectGeometries were previously unbound
        shapes_a, poses_a = _box()
        shapes_b, poses_b = _box()
        assert checker.addCollisionObject("static_box", 0, shapes_a, poses_a)
        assert checker.addCollisionObject("moving_box", 0, shapes_b, poses_b)
        assert len(checker.getCollisionObjectGeometries("static_box")) == 1
        assert len(checker.getCollisionObjectGeometriesTransforms("moving_box")) == 1

        checker.setActiveCollisionObjects(["moving_box"])
        checker.setCollisionMarginData(CollisionMarginData(0.1))
        assert checker.getCollisionMarginData().getMaxCollisionMargin() == pytest.approx(0.1)

        # static object stays at origin
        checker.setCollisionObjectsTransform("static_box", Isometry3d(np.eye(4)))

        # cast (moving) object sweeps from far away through the static box
        start = np.eye(4)
        start[0][3] = -5.0
        end = np.eye(4)
        end[0][3] = 0.0  # ends overlapping the static box
        checker.setCollisionObjectsTransformCast(
            "moving_box",
            Isometry3d(start),
            Isometry3d(end),
        )

        result = ContactResultMap()
        checker.contactTest(result, ContactRequest(ContactTestType_ALL))
        assert result.count() > 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


def test_continuous_manager_apply_config_acm():
    factory, locator = _get_discrete_factory()
    checker = factory.createContinuousContactManager("BulletCastBVHManager")
    try:
        shapes_a, poses_a = _box()
        shapes_b, poses_b = _box()
        checker.addCollisionObject("static_box", 0, shapes_a, poses_a)
        checker.addCollisionObject("moving_box", 0, shapes_b, poses_b)
        checker.setActiveCollisionObjects(["moving_box"])
        checker.setCollisionMarginData(CollisionMarginData(0.1))
        checker.setCollisionObjectsTransform("static_box", Isometry3d(np.eye(4)))

        start = np.eye(4)
        start[0][3] = -5.0
        end = np.eye(4)
        checker.setCollisionObjectsTransformCast("moving_box", Isometry3d(start), Isometry3d(end))

        result = ContactResultMap()
        checker.contactTest(result, ContactRequest(ContactTestType_ALL))
        assert result.count() > 0

        cfg = ContactManagerConfig()
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("static_box", "moving_box", "unit_test")
        cfg.acm = acm
        cfg.acm_override_type = ACMOverrideType.ASSIGN
        checker.applyContactManagerConfig(cfg)

        result2 = ContactResultMap()
        checker.contactTest(result2, ContactRequest(ContactTestType_ALL))
        assert result2.count() == 0
    finally:
        del checker
        del factory
        del locator
        gc.collect()
