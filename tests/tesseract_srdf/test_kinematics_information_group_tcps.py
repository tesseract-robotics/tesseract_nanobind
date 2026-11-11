"""Regression tests for KinematicsInformation.group_tcps Python accessor.

The underlying C++ field is a name-keyed map of name-keyed maps with an Eigen aligned
allocator on the inner map; the name-keyed map caster handles both allocators, so the
field is a plain copy-policy read/write attribute.
"""

from __future__ import annotations

import numpy as np

from tesseract_robotics.common import Isometry3d, LinkId, Quaterniond
from tesseract_robotics.srdf import KinematicsInformation


def test_group_tcps_empty_on_default_construction() -> None:
    ki = KinematicsInformation()
    assert ki.group_tcps == {}


def test_group_tcps_read_after_add_group_tcp_method() -> None:
    ki = KinematicsInformation()
    tcp = Isometry3d(np.array([0.1, 0.2, 0.3]), Quaterniond.Identity())
    ki.addGroupTCP("arm", "tool0", tcp)

    tcps = ki.group_tcps
    assert set(tcps.keys()) == {"arm"}
    assert set(tcps["arm"].keys()) == {"tool0"}
    np.testing.assert_allclose(tcps["arm"]["tool0"].translation, [0.1, 0.2, 0.3])


def test_group_tcps_setter_round_trip() -> None:
    ki = KinematicsInformation()
    tcp_a = Isometry3d(np.array([1.0, 0.0, 0.0]), Quaterniond.Identity())
    tcp_b = Isometry3d(np.array([0.0, 2.0, 0.0]), Quaterniond.Identity())

    ki.group_tcps = {"arm": {"tool0": tcp_a, "weld_tip": tcp_b}}

    assert ki.hasGroupTCP("arm", "tool0")
    assert ki.hasGroupTCP("arm", "weld_tip")
    np.testing.assert_allclose(ki.group_tcps["arm"]["tool0"].translation, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(ki.group_tcps["arm"]["weld_tip"].translation, [0.0, 2.0, 0.0])


def test_group_tcps_setter_replaces_prior_content() -> None:
    ki = KinematicsInformation()
    ki.addGroupTCP("arm", "tool0", Isometry3d())
    ki.addGroupTCP("positioner", "centerpiece", Isometry3d())
    assert set(ki.group_tcps.keys()) == {"arm", "positioner"}

    replacement_tcp = Isometry3d(np.array([7.0, 8.0, 9.0]), Quaterniond.Identity())
    ki.group_tcps = {"new_group": {"new_tcp": replacement_tcp}}

    assert set(ki.group_tcps.keys()) == {"new_group"}
    np.testing.assert_allclose(ki.group_tcps["new_group"]["new_tcp"].translation, [7.0, 8.0, 9.0])


def test_group_tcps_multiple_groups() -> None:
    ki = KinematicsInformation()
    ki.addGroupTCP("arm", "tool0", Isometry3d(np.array([1.0, 0.0, 0.0]), Quaterniond.Identity()))
    ki.addGroupTCP("arm", "flange", Isometry3d(np.array([0.0, 1.0, 0.0]), Quaterniond.Identity()))
    ki.addGroupTCP(
        "positioner", "fixture", Isometry3d(np.array([0.0, 0.0, 1.0]), Quaterniond.Identity())
    )

    tcps = ki.group_tcps
    assert set(tcps.keys()) == {"arm", "positioner"}
    assert set(tcps["arm"].keys()) == {"tool0", "flange"}
    assert set(tcps["positioner"].keys()) == {"fixture"}


def test_group_tcps_setter_accepts_link_id_keys() -> None:
    ki = KinematicsInformation()
    tcp = Isometry3d(np.array([4.0, 5.0, 6.0]), Quaterniond.Identity())
    ki.group_tcps = {"arm": {LinkId("tool0"): tcp}}
    assert ki.hasGroupTCP("arm", "tool0")
    np.testing.assert_allclose(ki.group_tcps["arm"]["tool0"].translation, [4.0, 5.0, 6.0])


def test_group_tcps_values_are_copies_and_survive_replacement() -> None:
    ki = KinematicsInformation()
    ki.group_tcps = {
        "arm": {"tool0": Isometry3d(np.array([1.0, 2.0, 3.0]), Quaterniond.Identity())}
    }
    held = ki.group_tcps["arm"]["tool0"]
    ki.group_tcps = {"other": {"t": Isometry3d()}}
    np.testing.assert_allclose(held.translation, [1.0, 2.0, 3.0])
    assert set(ki.group_tcps.keys()) == {"other"}


def test_group_states_round_trip() -> None:
    ki = KinematicsInformation()
    ki.group_states = {"arm": {"home": {"j1": 0.5, "j2": -0.5}}}
    states = ki.group_states
    assert set(states["arm"]["home"].keys()) == {"j1", "j2"}
    assert states["arm"]["home"]["j1"] == 0.5
    held = states["arm"]["home"]
    ki.group_states = {}
    assert held["j2"] == -0.5
    assert ki.group_states == {}
