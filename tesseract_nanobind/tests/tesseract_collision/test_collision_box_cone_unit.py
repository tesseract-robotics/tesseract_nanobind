"""Test collision checking with box and cone primitives."""
import numpy as np
import numpy.testing as nptest
import os
import pytest

from tesseract_robotics import tesseract_geometry
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_collision


def addCollisionObjects(checker):
    """Add collision objects to checker for testing."""
    initial_count = len(checker.getCollisionObjects())

    # Add static box
    box = tesseract_geometry.Box(1, 1, 1)
    box_pose = np.eye(4)
    obj1_shapes = tesseract_geometry.GeometriesConst()
    obj1_shapes.append(box)
    obj1_poses = tesseract_common.VectorIsometry3d()
    obj1_poses.append(tesseract_common.Isometry3d(box_pose))
    checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, False)
    checker.enableCollisionObject("box_link")

    # Add thin box (disabled)
    thin_box = tesseract_geometry.Box(0.1, 1, 1)
    obj2_shapes = tesseract_geometry.GeometriesConst()
    obj2_shapes.append(thin_box)
    obj2_poses = tesseract_common.VectorIsometry3d()
    obj2_poses.append(tesseract_common.Isometry3d(np.eye(4)))
    checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses)
    checker.disableCollisionObject("thin_box_link")

    # Add cone
    cone = tesseract_geometry.Cone(0.25, 0.25)
    obj3_shapes = tesseract_geometry.GeometriesConst()
    obj3_shapes.append(cone)
    obj3_poses = tesseract_common.VectorIsometry3d()
    obj3_poses.append(tesseract_common.Isometry3d(np.eye(4)))
    checker.addCollisionObject("cone_link", 0, obj3_shapes, obj3_poses)

    # Add box and remove
    remove_box = tesseract_geometry.Box(0.1, 1, 1)
    obj4_shapes = tesseract_geometry.GeometriesConst()
    obj4_shapes.append(remove_box)
    obj4_poses = tesseract_common.VectorIsometry3d()
    obj4_poses.append(tesseract_common.Isometry3d(np.eye(4)))
    checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses)
    assert len(checker.getCollisionObjects()) == initial_count + 4
    assert checker.hasCollisionObject("remove_box_link")
    checker.removeCollisionObject("remove_box_link")
    assert not checker.hasCollisionObject("remove_box_link")

    # Try functions on non-existent link
    assert not checker.removeCollisionObject("link_does_not_exist")
    assert not checker.enableCollisionObject("link_does_not_exist")
    assert not checker.disableCollisionObject("link_does_not_exist")

    # Try to add empty collision object
    assert not checker.addCollisionObject(
        "empty_link", 0, tesseract_geometry.GeometriesConst(), tesseract_common.VectorIsometry3d()
    )

    # Verify counts and geometry
    assert len(checker.getCollisionObjects()) == initial_count + 3
    for link_name in ["box_link", "thin_box_link", "cone_link"]:
        assert checker.hasCollisionObject(link_name)
        assert len(checker.getCollisionObjectGeometries(link_name)) == 1
        assert len(checker.getCollisionObjectGeometriesTransforms(link_name)) == 1
        tfs = checker.getCollisionObjectGeometriesTransforms(link_name)
        for i in range(len(tfs)):
            nptest.assert_almost_equal(tfs[i].matrix(), np.eye(4))


def run_test(checker):
    """Run collision test with the given checker."""
    addCollisionObjects(checker)

    # Test collision
    checker.setActiveCollisionObjects(["box_link", "cone_link"])
    checker.setCollisionMarginData(tesseract_common.CollisionMarginData(0.1))
    nptest.assert_almost_equal(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1)

    # Set transforms
    location = tesseract_common.TransformMap()
    location["box_link"] = tesseract_common.Isometry3d(np.eye(4))
    cone_transform = np.eye(4)
    cone_transform[0][3] = 0.2
    location["cone_link"] = tesseract_common.Isometry3d(cone_transform)
    checker.setCollisionObjectsTransform(location)

    # Perform collision check
    result = tesseract_collision.ContactResultMap()
    checker.contactTest(result, tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_CLOSEST))
    result_vector = tesseract_collision.ContactResultVector()
    result.flattenMoveResults(result_vector)

    assert len(result_vector) > 0
    contact = result_vector[0]
    assert contact.distance < 0, f"Expected penetration, got distance={contact.distance}"
    assert len(contact.link_names) == 2
    assert "box_link" in contact.link_names or "cone_link" in contact.link_names
    assert len(contact.nearest_points) == 2
    assert len(contact.nearest_points[0]) == 3
    assert len(contact.normal) == 3


def test_bullet_discrete_simple(iiwa_env):
    """Test discrete collision checking via Environment's contact manager."""
    env, _, _ = iiwa_env
    checker = env.getDiscreteContactManager()
    run_test(checker)


def test_bullet_discrete_bvh(iiwa_env):
    """Test BVH discrete collision checking via Environment's contact manager."""
    env, _, _ = iiwa_env
    checker = env.getDiscreteContactManager()
    run_test(checker)
