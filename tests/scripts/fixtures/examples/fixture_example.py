"""Fixture example for the coverage oracle's tests: one call per recorder mechanism.

This directory stands in for `src/tesseract_robotics/examples/` (the recorder's
example root). Each line below is one call the oracle tests expect to be
recorded; the ablation test deletes one of them and expects exactly that entry
to turn uncovered. Keep one API call per line.
"""

import numpy as np
from fixture_helper import clear_from_helper

from tesseract_robotics.tesseract_common import (
    ACMContactAllowedValidator,
    AllowedCollisionMatrix,
    Isometry3d,
    JointState,
    JointTrajectory,
    makeOrderedLinkPair,
)
from tesseract_robotics.tesseract_scene_graph import JointType


def run():
    acm = AllowedCollisionMatrix()
    acm_from_entries = AllowedCollisionMatrix({("a", "b"): "Adjacent"})
    acm_from_entries.removeAllowedCollision("a", "b")
    acm_from_entries.removeAllowedCollision("a")
    state = JointState(["j1"], np.array([0.1]))
    trajectory = JointTrajectory([state], "fixture")
    first = trajectory[0]
    trajectory[0] = first
    same = acm == acm_from_entries
    differ = acm != acm_from_entries
    allowed = ACMContactAllowedValidator(acm)("a", "b")
    names = first.joint_names
    identity = Isometry3d.Identity()
    pair = makeOrderedLinkPair("b", "a")
    label = str(JointType.FIXED)
    clear_from_helper(acm)
    return same, differ, allowed, names, identity, pair, label
