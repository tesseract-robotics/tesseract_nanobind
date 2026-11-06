"""SWIG-era compatibility names that C++ does not have are gone from the command language."""

import copy

import numpy as np
import pytest

from tesseract_robotics import tesseract_command_language as tcl
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d


@pytest.mark.parametrize(
    "name",
    ["MoveInstructionType_LINEAR", "MoveInstructionType_FREESPACE", "MoveInstructionType_CIRCULAR"],
)
def test_move_instruction_type_constants_removed(name):
    """#275: the enum members are the only spelling."""
    assert not hasattr(tcl, name)
    assert not hasattr(tcl._tesseract_command_language, name)


def test_package_has_no_all():
    """#275: `import *` exports every public name of the extension, not a hand-kept list."""
    assert not hasattr(tcl, "__all__")
    namespace: dict[str, object] = {}
    exec("from tesseract_robotics.tesseract_command_language import *", namespace)
    assert "toJointTrajectory" in namespace
    assert "moveFilter" in namespace


@pytest.mark.parametrize("cls", ["MoveInstruction", "MoveInstructionPoly"])
@pytest.mark.parametrize(
    "name", ["assignCartesianWaypoint", "assignJointWaypoint", "assignStateWaypoint"]
)
def test_assign_waypoint_aliases_removed(cls, name):
    """#276: `setWaypoint(wp)` is the one spelling; C++ 0.33 removed assign*Waypoint."""
    assert not hasattr(getattr(tcl, cls), name)
    assert hasattr(getattr(tcl, cls), "setWaypoint")


def test_profile_dictionary_add_profile_helper_removed():
    """#276: `ProfileDictionary.addProfile` takes every module's profiles directly."""
    assert not hasattr(tcl, "ProfileDictionary_addProfile")


RETIRED_POLY_HELPERS = [
    "CartesianWaypointPoly_wrap_CartesianWaypoint",
    "JointWaypointPoly_wrap_JointWaypoint",
    "StateWaypointPoly_wrap_StateWaypoint",
    "MoveInstructionPoly_wrap_MoveInstruction",
    "InstructionPoly_as_MoveInstructionPoly",
    "InstructionPoly_as_CompositeInstruction",
    "InstructionPoly_as_WaitInstruction",
    "InstructionPoly_as_TimerInstruction",
    "InstructionPoly_as_SetAnalogInstruction",
    "InstructionPoly_as_SetDigitalInstruction",
    "InstructionPoly_as_SetToolInstruction",
    "WaypointPoly_as_CartesianWaypointPoly",
    "WaypointPoly_as_JointWaypointPoly",
    "WaypointPoly_as_StateWaypointPoly",
]


@pytest.mark.parametrize("name", RETIRED_POLY_HELPERS)
def test_poly_helpers_removed(name):
    """#328: the Poly constructors and `as…()` methods are the native forms."""
    assert not hasattr(tcl, name)


def test_append_move_instruction_removed():
    """#328: `push_back(mi)` is the native form."""
    assert not hasattr(tcl.CompositeInstruction, "appendMoveInstruction")


def test_waypoint_as_is_a_reference_and_copy_copy_detaches():
    """#328: `as…Waypoint()` edits reach the move; `copy.copy` is the copy the old helper gave."""
    target = Isometry3d.Identity() * Translation3d(0.1, 0.2, 0.3)
    mi = tcl.MoveInstruction(
        tcl.CartesianWaypointPoly(tcl.CartesianWaypoint(Isometry3d.Identity())),
        tcl.MoveInstructionType.LINEAR,
        "DEFAULT",
    )
    detached = copy.copy(mi.getWaypoint().asCartesianWaypoint())
    mi.getWaypoint().asCartesianWaypoint().setTransform(target)
    np.testing.assert_allclose(
        mi.getWaypoint().asCartesianWaypoint().getTransform().matrix, target.matrix
    )
    np.testing.assert_allclose(detached.getTransform().matrix, np.eye(4))
