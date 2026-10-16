"""CartLineInfo and InverseKinematicsInfo field constructors (#290).

The C++ defaults of the trailing Eigen parameters are spelled as separate overloads: Eigen
default values fail at module load. The `std::vector<Bounds>` constructors of JointVel /
JointAccel / JointJerkConstraint, the other half of #290, are declared but not defined in
trajopt 0.35.0, so they are not bound.
"""

from __future__ import annotations

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import Isometry3d

from .scenes import Iiwa

# m: offsets that make each pose argument distinguishable
LINE_LENGTH = 0.1
TCP_LENGTH = 0.05


def _translated(x: float = 0.0, z: float = 0.0) -> Isometry3d:
    matrix = np.eye(4)
    matrix[0, 3], matrix[2, 3] = x, z
    return Isometry3d(matrix)


def test_cart_line_info_all_fields(iiwa: Iiwa):
    info = ti.CartLineInfo(
        iiwa.manip,
        "tool0",
        "base_link",
        _translated(),
        _translated(x=LINE_LENGTH),
        _translated(z=TCP_LENGTH),
        np.array([0, 1, 2]),
    )
    assert list(info.manip.getJointNames()) == iiwa.joint_names
    assert (info.source_frame, info.target_frame) == ("tool0", "base_link")
    np.testing.assert_allclose(info.target_frame_offset1.matrix, np.eye(4))
    np.testing.assert_allclose(info.target_frame_offset2.matrix, _translated(x=LINE_LENGTH).matrix)
    np.testing.assert_allclose(info.source_frame_offset.matrix, _translated(z=TCP_LENGTH).matrix)
    np.testing.assert_array_equal(info.indices, [0, 1, 2])


def test_cart_line_info_defaults(iiwa: Iiwa):
    info = ti.CartLineInfo(
        iiwa.manip, "tool0", "base_link", _translated(), _translated(x=LINE_LENGTH)
    )
    np.testing.assert_allclose(info.source_frame_offset.matrix, np.eye(4))
    np.testing.assert_array_equal(info.indices, np.arange(6))

    with_offset = ti.CartLineInfo(
        iiwa.manip,
        "tool0",
        "base_link",
        _translated(),
        _translated(x=LINE_LENGTH),
        _translated(z=TCP_LENGTH),
    )
    np.testing.assert_allclose(
        with_offset.source_frame_offset.matrix, _translated(z=TCP_LENGTH).matrix
    )
    np.testing.assert_array_equal(with_offset.indices, np.arange(6))


def test_inverse_kinematics_info_all_fields(iiwa: Iiwa):
    info = ti.InverseKinematicsInfo(iiwa.manip, "base_link", "tool0", _translated(z=TCP_LENGTH))
    assert list(info.manip.getJointNames()) == iiwa.joint_names
    assert (info.working_frame, info.tcp_frame) == ("base_link", "tool0")
    np.testing.assert_allclose(info.tcp_offset.matrix, _translated(z=TCP_LENGTH).matrix)


def test_inverse_kinematics_info_default_offset(iiwa: Iiwa):
    info = ti.InverseKinematicsInfo(iiwa.manip, "base_link", "tool0")
    np.testing.assert_allclose(info.tcp_offset.matrix, np.eye(4))
