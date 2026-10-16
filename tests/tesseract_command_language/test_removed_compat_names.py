"""SWIG-era compatibility names that C++ does not have are gone from the command language."""

import pytest

from tesseract_robotics import tesseract_command_language as tcl


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
