"""Id-typed data members read as copies, like the str fields they replaced (#141).

A reference into C++ storage would let a later assignment rewrite an id that is
already a dict key, so the dict could no longer find it.
"""

import pytest

from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
from tesseract_robotics.tesseract_scene_graph import Joint, JointMimic
from tesseract_robotics.trajopt_ifopt import CartLineInfo, InverseKinematicsInfo

ID_FIELDS = [
    (lambda: Joint("j"), "child_link_id"),
    (lambda: Joint("j"), "parent_link_id"),
    (JointMimic, "joint_id"),
    (ManipulatorInfo, "working_frame"),
    (ManipulatorInfo, "tcp_frame"),
    (KinGroupIKInput, "working_frame"),
    (KinGroupIKInput, "tip_link_id"),
    (CartLineInfo, "source_frame"),
    (CartLineInfo, "target_frame"),
    (InverseKinematicsInfo, "working_frame"),
    (InverseKinematicsInfo, "tcp_frame"),
]


@pytest.mark.parametrize(
    ("make", "field"), ID_FIELDS, ids=lambda v: v if isinstance(v, str) else ""
)
def test_id_field_read_is_a_copy(make, field):
    obj = make()
    setattr(obj, field, "a")
    before = getattr(obj, field)
    index = {before: 1}

    setattr(obj, field, "b")

    assert before == "a"
    assert index["a"] == 1
    assert getattr(obj, field) == "b"
