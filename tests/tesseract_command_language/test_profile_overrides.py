"""Per-namespace profile overrides on instructions (#265)."""

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
)


def _move():
    return MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [1.0])), MoveInstructionType.LINEAR, "P", "PATH"
    )


def test_move_profile_overrides():
    mi = _move()
    mi.setProfileOverrides({"ns": "P2"})
    assert mi.getProfileOverrides() == {"ns": "P2"}
    assert mi.getProfile("ns") == "P2"
    assert mi.getProfile("other") == "P"


def test_move_path_profile_overrides():
    mi = _move()
    mi.setPathProfileOverrides({"ns": "PATH2"})
    assert mi.getPathProfileOverrides() == {"ns": "PATH2"}
    assert mi.getPathProfile("ns") == "PATH2"
    assert mi.getPathProfile("other") == "PATH"


def test_overrides_getter_returns_a_copy():
    mi = _move()
    mi.setProfileOverrides({"ns": "P2"})
    mi.getProfileOverrides()["ns"] = "X"
    assert mi.getProfile("ns") == "P2"


def test_composite_profile_overrides():
    ci = CompositeInstruction("C")
    ci.setProfileOverrides({"ns": "C2"})
    assert ci.getProfileOverrides() == {"ns": "C2"}
    assert ci.getProfile("ns") == "C2"
    assert ci.getProfile("other") == "C"


def test_move_instruction_poly_reports_overrides():
    mi = _move()
    mi.setProfileOverrides({"ns": "P2"})
    mi.setPathProfileOverrides({"ns": "PATH2"})
    mip = MoveInstructionPoly(mi)
    assert mip.getProfileOverrides() == {"ns": "P2"}
    assert mip.getPathProfileOverrides() == {"ns": "PATH2"}
    mip.setProfileOverrides({"ns2": "P3"})
    assert mip.getProfile("ns2") == "P3"
    mip.setPathProfileOverrides({"ns2": "PATH3"})
    assert mip.getPathProfile("ns2") == "PATH3"
