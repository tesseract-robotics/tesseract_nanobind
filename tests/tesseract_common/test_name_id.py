"""LinkId / JointId / LinkIdPair: upstream's id types as seen from Python.

Upstream (tesseract 5e8ad73b) addresses links and joints by a hash-of-name id that
retains its name. The bindings mirror that, with implicit ``str -> id`` conversion,
and keep ids interchangeable with their names as dict keys.
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    CollisionMarginData,
    Isometry3d,
    JointId,
    JointState,
    LinkId,
    LinkIdPair,
    ManipulatorInfo,
)


def test_link_id_keeps_name_and_equals_str():
    lid = LinkId("base_link")
    assert lid.name() == "base_link"
    assert lid.isValid()
    assert lid == "base_link"
    assert hash(lid) == hash("base_link")
    assert str(lid) == "base_link"
    assert repr(lid) == "LinkId('base_link')"


def test_invalid_id():
    assert not LinkId().isValid()
    assert LinkId() == LinkId("")


def test_id_keyed_dict_indexed_by_str():
    d = {LinkId("tool0"): 1}
    assert d["tool0"] == 1


def test_link_and_joint_ids_are_distinct_types():
    assert LinkId("a") != JointId("a")
    with pytest.raises(TypeError):
        JointState([LinkId("j1")], np.array([0.0]))  # a LinkId must not convert to a JointId


def test_str_list_converts_to_joint_ids():
    js = JointState(["j1", "j2"], np.array([0.1, 0.2]))
    assert [j.name() for j in js.joint_ids] == ["j1", "j2"]
    assert all(isinstance(j, JointId) for j in js.joint_ids)


def test_link_id_pair_is_unordered():
    assert LinkIdPair("a", "b") == LinkIdPair("b", "a")
    assert hash(LinkIdPair("a", "b")) == hash(LinkIdPair("b", "a"))
    assert LinkIdPair("b", "a").orderedNameView() == ("a", "b")


@pytest.mark.parametrize("pair", [("a", "b"), ("b", "a")])
def test_link_id_pair_never_equals_a_tuple(pair):
    # Equal objects must hash equal; a tuple's hash is order-dependent, an
    # unordered pair's is not, so no tuple may compare equal to a LinkIdPair.
    p = LinkIdPair("a", "b")
    assert p != pair
    assert pair != p
    assert pair not in {p}
    assert p not in {pair}


def test_ids_pickle_by_name():
    import pickle

    assert pickle.loads(pickle.dumps(LinkId("tool0"))) == LinkId("tool0")
    assert pickle.loads(pickle.dumps(JointId("j1"))) == JointId("j1")


def test_acm_takes_pairs_and_str_tuples():
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("link_a", "link_b", "Adjacent")
    assert acm.isCollisionAllowed(LinkIdPair("link_b", "link_a"))
    assert acm.isCollisionAllowed(("link_a", "link_b"))
    assert not acm.isCollisionAllowed(("link_a", "link_c"))
    entries = acm.getAllAllowedCollisions()
    assert entries[LinkIdPair("link_a", "link_b")] == "Adjacent"


def test_manipulator_info_frames_are_link_ids():
    info = ManipulatorInfo("manipulator", "base_link", "tool0")
    assert info.working_frame == "base_link"
    assert isinstance(info.tcp_frame, LinkId)
    info.tcp_offset = "tcp_link"
    assert info.tcp_offset == LinkId("tcp_link")
    info.tcp_offset = Isometry3d()
    assert isinstance(info.tcp_offset, Isometry3d)


def test_collision_margin_pair_by_names():
    data = CollisionMarginData(0.01)
    data.setCollisionMargin("a", "b", 0.05)
    assert data.getCollisionMargin("b", "a") == pytest.approx(0.05)
