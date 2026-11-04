"""Id-container fields read as copies: the ids outlive a later edit of the owner.

0.36 made the keys and elements of these fields bound `JointId` / `LinkId` objects
(they were `str`). A `def_rw` getter casts with `rv_policy::reference_internal`,
which nanobind forwards to every element (`nanobind/stl/detail/nb_dict.h:77`), so
without `rv_policy::copy` each id wrapped a node of the C++ container and dangled
once the owner cleared or replaced it.
"""

import sys

import pytest

from tesseract_robotics.tesseract_collision import ContactManagerConfig, ContactTrajectoryResults
from tesseract_robotics.tesseract_common import (
    CalibrationInfo,
    Isometry3d,
    JointId,
    JointState,
    LinkId,
)
from tesseract_robotics.tesseract_scene_graph import ShortestPath
from tesseract_robotics.tesseract_srdf import KinematicsInformation
from tesseract_robotics.tesseract_state_solver import SceneState

# Enough entries that the freed nodes are reused by the refill below
NUM_IDS = 64


def _ids(value) -> list[str]:
    """Every JointId / LinkId in a nested read of a field, as names, in a stable order."""
    if isinstance(value, (JointId, LinkId)):
        return [str(value)]
    if isinstance(value, dict):
        items = sorted(value.items(), key=lambda item: str(item[0]))
        return [name for key, val in items for name in _ids(key) + _ids(val)]
    if isinstance(value, (list, tuple)):
        return [name for item in value for name in _ids(item)]
    return []


NAMES = [f"id_{i}" for i in range(NUM_IDS)]
FILLER = [f"{'x' * 40}_{i}" for i in range(4 * NUM_IDS)]
CASES = {
    "CalibrationInfo.joints": (
        CalibrationInfo,
        "joints",
        lambda names: {name: Isometry3d.Identity() for name in names},
    ),
    "JointState.joint_ids": (JointState, "joint_ids", list),
    "SceneState.joints": (SceneState, "joints", lambda names: dict.fromkeys(names, 0.0)),
    "SceneState.floating_joints": (
        SceneState,
        "floating_joints",
        lambda names: {name: Isometry3d.Identity() for name in names},
    ),
    "ShortestPath.links": (ShortestPath, "links", list),
    "ShortestPath.joints": (ShortestPath, "joints", list),
    "ShortestPath.active_joints": (ShortestPath, "active_joints", list),
    "ContactTrajectoryResults.joint_ids": (ContactTrajectoryResults, "joint_ids", list),
    "ContactManagerConfig.modify_object_enabled": (
        ContactManagerConfig,
        "modify_object_enabled",
        lambda names: dict.fromkeys(names, True),
    ),
    "KinematicsInformation.chain_groups": (
        KinematicsInformation,
        "chain_groups",
        lambda names: {"group": list(zip(names[::2], names[1::2]))},
    ),
    "KinematicsInformation.joint_groups": (
        KinematicsInformation,
        "joint_groups",
        lambda names: {"group": list(names)},
    ),
    "KinematicsInformation.link_groups": (
        KinematicsInformation,
        "link_groups",
        lambda names: {"group": list(names)},
    ),
    "KinematicsInformation.group_states": (
        KinematicsInformation,
        "group_states",
        lambda names: {"group": {"state": dict.fromkeys(names, 0.0)}},
    ),
}


@pytest.mark.parametrize("case", CASES)
def test_ids_read_from_a_field_survive_replacing_it(case):
    cls, field, build = CASES[case]
    owner = cls()
    setattr(owner, field, build(NAMES))
    owner_refs = sys.getrefcount(owner)
    read = getattr(owner, field)
    # a reference into the owner keeps it alive, once per element; a copy does not
    assert sys.getrefcount(owner) == owner_refs
    expected = _ids(getattr(owner, field))
    assert len(expected) >= NUM_IDS

    setattr(owner, field, build(FILLER))

    assert _ids(read) == expected
