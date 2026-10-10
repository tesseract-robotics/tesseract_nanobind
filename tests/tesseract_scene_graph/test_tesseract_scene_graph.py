import os

import numpy as np
import pytest

import tesseract_robotics.tesseract_scene_graph as sg
from tesseract_robotics import tesseract_common, tesseract_srdf

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


def _translation(p):
    H = np.eye(4)
    H[0:3, 3] = p
    return tesseract_common.Isometry3d(H)


def test_tesseract_scene_graph():
    g = sg.SceneGraph()
    assert g.addLink(sg.Link("base_link"))
    assert g.addLink(sg.Link("link_1"))
    assert g.addLink(sg.Link("link_2"))
    assert g.addLink(sg.Link("link_3"))
    assert g.addLink(sg.Link("link_4"))
    assert g.addLink(sg.Link("link_5"))

    base_joint = sg.Joint("base_joint")
    base_joint.parent_link_name = "base_link"
    base_joint.child_link_name = "link_1"
    base_joint.type = sg.JointType_FIXED
    assert g.addJoint(base_joint)

    joint_1 = sg.Joint("joint_1")
    joint_1.parent_link_name = "link_1"
    joint_1.child_link_name = "link_2"
    joint_1.type = sg.JointType_FIXED
    assert g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_2")
    joint_2.parent_to_joint_origin_transform = _translation([1.25, 0, 0])
    joint_2.parent_link_name = "link_2"
    joint_2.child_link_name = "link_3"
    joint_2.type = sg.JointType_PLANAR
    joint_2.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25, 0, 0])
    joint_3.parent_link_name = "link_3"
    joint_3.child_link_name = "link_4"
    joint_3.type = sg.JointType_FLOATING
    assert g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_4")
    joint_4.parent_to_joint_origin_transform = _translation([0, 1.25, 0])
    joint_4.parent_link_name = "link_2"
    joint_4.child_link_name = "link_5"
    joint_4.type = sg.JointType_REVOLUTE
    joint_4.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_4)

    adjacent_links = g.getAdjacentLinkNames("link_3")
    assert len(adjacent_links) == 1
    assert adjacent_links[0] == "link_4"

    inv_adjacent_links = g.getInvAdjacentLinkNames("link_3")
    assert len(inv_adjacent_links) == 1
    assert inv_adjacent_links[0] == "link_2"

    child_link_names = g.getLinkChildrenNames("link_5")
    assert len(child_link_names) == 0

    child_link_names = g.getLinkChildrenNames("link_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getLinkChildrenNames("link_2")
    assert len(child_link_names) == 3
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    child_link_names = g.getJointChildrenNames("joint_4")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_5"

    child_link_names = g.getJointChildrenNames("joint_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getJointChildrenNames("joint_1")
    assert len(child_link_names) == 4
    assert "link_2" in child_link_names
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    assert g.isAcyclic()
    assert g.isTree()

    g.addLink(sg.Link("link_6"))
    assert not g.isTree()

    g.removeLink("link_6")
    assert g.isTree()

    joint_5 = sg.Joint("joint_5")
    joint_5.parent_to_joint_origin_transform = _translation([0, 1.5, 0])
    joint_5.parent_link_name = "link_5"
    joint_5.child_link_name = "link_4"
    joint_5.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_5)

    assert g.isAcyclic()
    assert not g.isTree()

    joint_6 = sg.Joint("joint_6")
    joint_6.parent_to_joint_origin_transform = _translation([0, 1.25, 0])
    joint_6.parent_link_name = "link_5"
    joint_6.child_link_name = "link_1"
    joint_6.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_6)

    assert not g.isAcyclic()
    assert not g.isTree()

    path = g.getShortestPath("link_1", "link_4")

    assert len(path.links) == 4
    assert "link_1" in path.links
    assert "link_2" in path.links
    assert "link_3" in path.links
    assert "link_4" in path.links
    assert len(path.joints) == 3
    assert "joint_1" in path.joints
    assert "joint_2" in path.joints
    assert "joint_3" in path.joints

    print(g.getName())


def test_load_srdf_unit():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    srdf_file = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")

    locator = TesseractSupportResourceLocator()

    g = sg.SceneGraph()

    g.setName("kuka_lbr_iiwa_14_r820")

    assert g.addLink(sg.Link("base_link"))
    assert g.addLink(sg.Link("link_1"))
    assert g.addLink(sg.Link("link_2"))
    assert g.addLink(sg.Link("link_3"))
    assert g.addLink(sg.Link("link_4"))
    assert g.addLink(sg.Link("link_5"))
    assert g.addLink(sg.Link("link_6"))
    assert g.addLink(sg.Link("link_7"))
    assert g.addLink(sg.Link("tool0"))

    joint_1 = sg.Joint("joint_a1")
    joint_1.parent_link_name = "base_link"
    joint_1.child_link_name = "link_1"
    joint_1.type = sg.JointType_FIXED
    assert g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_a2")
    joint_2.parent_link_name = "link_1"
    joint_2.child_link_name = "link_2"
    joint_2.type = sg.JointType_REVOLUTE
    joint_2.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_a3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25, 0, 0])
    joint_3.parent_link_name = "link_2"
    joint_3.child_link_name = "link_3"
    joint_3.type = sg.JointType_REVOLUTE
    joint_3.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_a4")
    joint_4.parent_to_joint_origin_transform = _translation([1.25, 0, 0])
    joint_4.parent_link_name = "link_3"
    joint_4.child_link_name = "link_4"
    joint_4.type = sg.JointType_REVOLUTE
    joint_4.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_4)

    joint_5 = sg.Joint("joint_a5")
    joint_5.parent_to_joint_origin_transform = _translation([0, 1.25, 0])
    joint_5.parent_link_name = "link_4"
    joint_5.child_link_name = "link_5"
    joint_5.type = sg.JointType_REVOLUTE
    joint_5.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_5)

    joint_6 = sg.Joint("joint_a6")
    joint_6.parent_to_joint_origin_transform = _translation([0, 1.25, 0])
    joint_6.parent_link_name = "link_5"
    joint_6.child_link_name = "link_6"
    joint_6.type = sg.JointType_REVOLUTE
    joint_6.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_6)

    joint_7 = sg.Joint("joint_a7")
    joint_7.parent_to_joint_origin_transform = _translation([0, 1.25, 0])
    joint_7.parent_link_name = "link_6"
    joint_7.child_link_name = "link_7"
    joint_7.type = sg.JointType_REVOLUTE
    joint_7.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
    assert g.addJoint(joint_7)

    joint_tool0 = sg.Joint("base_joint")
    joint_tool0.parent_link_name = "link_7"
    joint_tool0.child_link_name = "tool0"
    joint_tool0.type = sg.JointType_FIXED
    assert g.addJoint(joint_tool0)

    srdf = tesseract_srdf.SRDFModel()
    srdf.initFile(g, srdf_file, locator)

    tesseract_srdf.processSRDFAllowedCollisions(g, srdf)

    acm = g.getAllowedCollisionMatrix()

    assert acm.isCollisionAllowed("link_1", "link_2")
    assert not acm.isCollisionAllowed("base_link", "link_5")

    g.removeAllowedCollision("link_1", "link_2")

    assert not acm.isCollisionAllowed("link_1", "link_2")

    g.clearAllowedCollisions()
    assert len(acm.getAllAllowedCollisions()) == 0


def _tree_graph():
    """base_link -fixed- link_1 -fixed- link_2 -planar- link_3 -floating- link_4; link_2 -revolute- link_5."""
    g = sg.SceneGraph()
    for name in ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5"):
        assert g.addLink(sg.Link(name))
    for name, parent, child, joint_type in (
        ("base_joint", "base_link", "link_1", sg.JointType.FIXED),
        ("joint_1", "link_1", "link_2", sg.JointType.FIXED),
        ("joint_2", "link_2", "link_3", sg.JointType.PLANAR),
        ("joint_3", "link_3", "link_4", sg.JointType.FLOATING),
        ("joint_4", "link_2", "link_5", sg.JointType.REVOLUTE),
    ):
        joint = sg.Joint(name)
        joint.parent_link_name = parent
        joint.child_link_name = child
        joint.type = joint_type
        if joint_type in (sg.JointType.PLANAR, sg.JointType.REVOLUTE):
            joint.limits = sg.JointLimits(-1, 1, 1, 1, 1, 1)
        assert g.addJoint(joint)
    return g


def test_joint_value_types_str_is_upstream_operator():
    """#215: the upstream operator<< formats (scene_graph/src/joint.cpp @ 0.35.0)."""
    assert str(sg.JointDynamics(0.5, 2)) == "damping=0.5 friction=2"
    assert (
        str(sg.JointLimits(-1, 1, 10, 2, 3, 4))
        == "lower=-1 upper=1 effort=10 velocity=2 acceleration=3 jerk=4"
    )
    assert (
        str(sg.JointSafety(1.5, -1.5, 3, 4))
        == "soft_upper_limit=1.5 soft_lower_limit=-1.5 k_position=3 k_velocity=4"
    )
    assert str(sg.JointCalibration(0.25, 1, 2)) == "reference_position=0.25 rising=1 falling=2"
    assert str(sg.JointMimic(0.5, 2, "joint_a")) == "joint_name=joint_a offset=0.5 multiplier=2"


def test_joint_value_types_repr_unchanged():
    assert repr(sg.JointDynamics(0.5, 2)) == "JointDynamics(damping=0.500000, friction=2.000000)"
    assert (
        repr(sg.JointLimits(-1, 1, 10, 2, 3, 4)) == "JointLimits(lower=-1.000000, upper=1.000000)"
    )


def test_joint_type_str_is_upstream_operator():
    """#215: str() is upstream's operator<< (joint.cpp:272-312); repr() and .name are Python's."""
    expected = {
        sg.JointType.FIXED: "Fixed",
        sg.JointType.PLANAR: "Planar",
        sg.JointType.FLOATING: "Floating",
        sg.JointType.REVOLUTE: "Revolute",
        sg.JointType.PRISMATIC: "Prismatic",
        sg.JointType.CONTINUOUS: "Continuous",
        sg.JointType.UNKNOWN: "Unknown",
    }
    assert {t: str(t) for t in sg.JointType} == expected
    assert sg.JointType.REVOLUTE.name == "REVOLUTE"
    # nanobind makes every enum's __repr__ Enum.__str__ (nb_enum.cpp:75-76); binding __str__
    # replaces only __str__, so repr keeps the "JointType.REVOLUTE" form it always had.
    assert repr(sg.JointType.REVOLUTE) == "JointType.REVOLUTE"


def test_shortest_path_str_is_upstream_operator():
    """#215: graph.cpp:1295-1309, a section per list, one indented name per line."""
    path = _tree_graph().getShortestPath("link_1", "link_4")
    expected = (
        "Links:\n"
        + "".join(f"  {n}\n" for n in path.links)
        + "Joints:\n"
        + "".join(f"  {n}\n" for n in path.joints)
        + "Active Joints:\n"
        + "".join(f"  {n}\n" for n in path.active_joints)
    )
    assert str(path) == expected
    assert path.links == ["link_1", "link_2", "link_3", "link_4"]


def test_link_visible_and_collision_enabled():
    """#216: link.h:210, :213; clone() copies both (link.cpp:154-158), == compares both (:190-193)."""
    link = sg.Link("link_a")
    assert link.visible is True
    assert link.collision_enabled is True
    link.visible = False
    link.collision_enabled = False
    clone = link.clone()
    assert clone.visible is False
    assert clone.collision_enabled is False
    assert clone == link
    clone.visible = True
    assert clone != link


def test_set_allowed_collision_matrix_shares_the_matrix():
    g = _tree_graph()
    acm = tesseract_common.AllowedCollisionMatrix()
    acm.addAllowedCollision("link_1", "link_2", "adjacent")
    g.setAllowedCollisionMatrix(acm)
    assert g.isCollisionAllowed("link_1", "link_2")
    # the graph keeps the Python object's shared_ptr: later edits to acm are edits to the graph's
    acm.addAllowedCollision("link_2", "link_3", "test")
    assert g.isCollisionAllowed("link_2", "link_3")


def test_set_allowed_collision_matrix_refuses_none():
    """#216: upstream stores a null and the next ACM call dereferences it (graph.cpp:746-770)."""
    g = _tree_graph()
    g.addAllowedCollision("link_1", "link_2", "adjacent")
    with pytest.raises(TypeError):
        g.setAllowedCollisionMatrix(None)
    assert g.isCollisionAllowed("link_1", "link_2")
    assert not g.isCollisionAllowed("link_1", "link_3")


def test_get_adjacency_map():
    """#216: each listed link maps itself and the links below it, stopping at another listed link.

    _tree_graph: base_link - link_1 - link_2 - {link_3 - link_4, link_5} (graph.cpp:118-151, :921-949).
    """
    g = _tree_graph()
    assert g.getAdjacencyMap(["link_3"]) == {"link_3": "link_3", "link_4": "link_3"}
    expected = {"link_2": "link_2", "link_5": "link_2", "link_3": "link_3", "link_4": "link_3"}
    assert g.getAdjacencyMap(["link_2", "link_3"]) == expected
    assert g.getAdjacencyMap(["link_3", "link_2"]) == expected
    assert g.getAdjacencyMap(["link_4"]) == {"link_4": "link_4"}


def test_get_adjacency_map_unknown_link_raises_key_error():
    with pytest.raises(KeyError, match="no_such_link"):
        _tree_graph().getAdjacencyMap(["link_3", "no_such_link"])
