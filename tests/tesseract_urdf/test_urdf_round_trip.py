"""URDF round-trip contract: parse a support robot, write it, parse the result, compare.

Links, joints, joint types, parents, children, origins, axes and limits survive the trip.
A failed write raises URDFWriteError with upstream's whole nested cause in the message.
"""

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path, get_tesseract_support_path, urdf
from tesseract_robotics.common import GeneralResourceLocator

# [m and rad] the writer prints origins and limits as decimal text; a round trip through it
# reproduces them to the printed precision, far below any physical meaning.
ROUND_TRIP_ATOL = 1e-9

# Support robots whose geometry the upstream writer handles. Robots with
# `tesseract:make_convex` collision meshes (abb_irb2400, lbr_iiwa_14_r820, ...) do not round
# trip: upstream's writeGeometry has no CONVEX_MESH branch (tesseract/urdf/src/geometry.cpp).
ROBOTS = ["boxbot.urdf"]


def _locator() -> GeneralResourceLocator:
    return GeneralResourceLocator(paths=[get_tesseract_resource_path()])


def _joints(scene_graph):
    return {joint.getName(): joint for joint in scene_graph.getJoints()}


@pytest.mark.parametrize("robot", ROBOTS)
def test_parse_write_reparse_preserves_the_kinematics(robot, tmp_path):
    original = urdf.parseURDFFile(str(get_tesseract_support_path() / "urdf" / robot), _locator())
    written = urdf.writeURDFFile(original, str(tmp_path), "robot")
    assert written == tmp_path / "urdf" / "robot.urdf"
    assert written.is_file()
    reparsed = urdf.parseURDFFile(str(written), _locator())

    assert {link.getName() for link in reparsed.getLinks()} == {
        link.getName() for link in original.getLinks()
    }
    before, after = _joints(original), _joints(reparsed)
    assert set(after) == set(before)
    for name, joint in before.items():
        other = after[name]
        assert other.type == joint.type, name
        assert other.parent_link_id == joint.parent_link_id, name
        assert other.child_link_id == joint.child_link_id, name
        np.testing.assert_allclose(
            other.parent_to_joint_origin_transform.matrix,
            joint.parent_to_joint_origin_transform.matrix,
            atol=ROUND_TRIP_ATOL,
            err_msg=name,
        )
        np.testing.assert_allclose(other.axis, joint.axis, atol=ROUND_TRIP_ATOL, err_msg=name)
        if joint.limits is None:
            assert other.limits is None, name
            continue
        for field in ("lower", "upper", "velocity", "effort"):
            assert getattr(other.limits, field) == pytest.approx(
                getattr(joint.limits, field), abs=ROUND_TRIP_ATOL
            ), (name, field)


def test_write_failure_names_the_nested_cause(tmp_path):
    graph = urdf.parseURDFFile(
        str(get_tesseract_support_path() / "urdf" / "boxbot.urdf"), _locator()
    )
    blocker = tmp_path / "not_a_directory"
    blocker.write_text("")
    with pytest.raises(urdf.URDFWriteError, match=r"writeURDFFile: .+"):
        urdf.writeURDFFile(graph, str(blocker / "pkg"), "robot")
    assert issubclass(urdf.URDFWriteError, RuntimeError)
