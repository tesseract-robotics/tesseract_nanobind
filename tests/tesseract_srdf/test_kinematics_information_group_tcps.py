"""Regression tests for KinematicsInformation.group_tcps Python accessor.

The underlying C++ field is unordered_map<string, unordered_map<string, Isometry3d,
Eigen::aligned_allocator<...>>>. A raw def_rw exposed a Python attribute whose getter
raised TypeError on access because nanobind's default stl/unordered_map caster does
not pattern-match against the aligned_allocator template arg. The binding now exposes
it via def_prop_rw with explicit lambdas that copy through default-allocator nested
maps at the boundary.
"""

from __future__ import annotations

import numpy as np

from tesseract_robotics.tesseract_common import Isometry3d, Quaterniond
from tesseract_robotics.tesseract_srdf import KinematicsInformation


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
