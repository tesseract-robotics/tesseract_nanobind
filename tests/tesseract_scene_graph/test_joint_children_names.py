"""`SceneGraph.getJointChildrenIds` binds both C++ overloads: one joint name and a list.

The graph is upstream's `createTestSceneGraph` (tesseract_scene_graph_unit.cpp:44-92); the
expectations are its unit test (:137-151) and the list overload's union of each joint's
children. 0.36 collects it in a `std::unordered_set` (graph.cpp:922-932): no duplicates, no
order, so the list results are compared sorted (expected lists are sorted and duplicate-free,
so equality still rules out duplicates).
"""

from __future__ import annotations

import numpy as np
import pytest

import tesseract_robotics.scene_graph as sg
from tesseract_robotics.common import Isometry3d, Quaterniond

# createTestSceneGraph offsets every joint 1.25 along x or y (tesseract_scene_graph_unit.cpp:57-80)
JOINT_OFFSET = 1.25


def _joint(name, parent, child, joint_type, axis, limits=None):
    offset = np.zeros(3)
    offset[axis] = JOINT_OFFSET
    joint = sg.Joint(name)
    joint.parent_to_joint_origin_transform = Isometry3d(offset, Quaterniond.Identity())
    joint.parent_link_id = parent
    joint.child_link_id = child
    joint.type = joint_type
    if limits is not None:
        joint.limits = limits
    return joint


@pytest.fixture(scope="module")
def graph() -> sg.SceneGraph:
    g = sg.SceneGraph()
    for i in range(1, 6):
        assert g.addLink(sg.Link(f"link_{i}"))
    for joint in [
        _joint("joint_1", "link_1", "link_2", sg.JointType.FIXED, 0),
        _joint(
            "joint_2", "link_2", "link_3", sg.JointType.PLANAR, 0, sg.JointLimits(-1, 1, 0, 2, 3, 6)
        ),
        _joint("joint_3", "link_3", "link_4", sg.JointType.FLOATING, 0),
        _joint(
            "joint_4",
            "link_2",
            "link_5",
            sg.JointType.REVOLUTE,
            1,
            sg.JointLimits(-1, 1, 0, 2, 3, 4),
        ),
    ]:
        assert g.addJoint(joint)
    return g


@pytest.mark.parametrize(
    ("names", "children"),
    [
        (["joint_4"], ["link_5"]),  # tesseract_scene_graph_unit.cpp:138-140
        (["joint_3"], ["link_4"]),  # :142-144
        (["joint_1"], ["link_2", "link_3", "link_4", "link_5"]),  # :146-151
        # the active joints, as Environment passes them (environment.cpp:577)
        (["joint_2", "joint_4"], ["link_3", "link_4", "link_5"]),
        # link_5 is below both joints and listed once
        (["joint_1", "joint_4"], ["link_2", "link_3", "link_4", "link_5"]),
        ([], []),
    ],
)
def test_joint_children_names_of_a_list(graph, names, children):
    assert sorted(map(str, graph.getJointChildrenIds(names))) == children


def test_joint_children_names_of_a_tuple(graph):
    assert sorted(map(str, graph.getJointChildrenIds(("joint_3", "joint_4")))) == [
        "link_4",
        "link_5",
    ]


def test_a_str_still_selects_the_one_name_overload(graph):
    """A `str` is a sequence of `str`; it must not be read as a list of one-letter names."""
    assert graph.getJointChildrenIds("joint_4") == ["link_5"]


@pytest.mark.parametrize("names", ["nope", ["joint_4", "nope"]])
def test_unknown_joint_name_raises(graph, names):
    """Upstream's getEdge throws std::runtime_error (graph.cpp:1092-1099) for either overload."""
    with pytest.raises(RuntimeError, match="'nope' does not exist"):
        graph.getJointChildrenIds(names)
