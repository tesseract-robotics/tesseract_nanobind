"""Acceptance test — ABB IRB 2400 draws a 1 m × 1 m square outline + zigzag fill
at 10 cm spacing, emits RAPID, and golden-locks the output.

Workflow: `MotionProgram → CompositeInstruction → emit_rapid → string` compared
against the checked-in golden at ``golden/abb_irb2400_zigzag_square.mod``. No
planner runs — the test exercises the sparse-RAPID path (controller does
linear interpolation between Cartesian waypoints).

Golden regeneration: if the emitter output legitimately changes (precision
tweak, syntax adjustment), regenerate by running this test once, copying the
``actual`` output from the unified diff into the golden ``.mod`` file, and
re-running. Do NOT hand-edit the golden to fit a divergent run.
"""

from __future__ import annotations

import difflib
from pathlib import Path
from typing import Final

import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.emitters.rapid import RapidProfile, emit_rapid
from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
)
from tesseract_robotics.planning.toolpaths import OUTLINE_POSE_COUNT, ZIGZAG_LINE_COUNT

#: TCP frame name — the robot link the planner / emitter treats as the tool tip.
TCP_FRAME: Final[str] = "tool0"

#: Manipulator joint group name in the URDF / SRDF.
MANIPULATOR: Final[str] = "manipulator"

#: Expected leaf-instruction count: the outline moves plus 2 per zigzag line.
EXPECTED_INSTRUCTION_COUNT: Final[int] = OUTLINE_POSE_COUNT + 2 * ZIGZAG_LINE_COUNT  # = 27

GOLDEN: Final[Path] = Path(__file__).parent / "golden" / "abb_irb2400_zigzag_square.mod"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def abb_irb2400() -> Robot:
    """ABB IRB 2400 robot loaded from tesseract_support.

    Session-scoped: loading the URDF + meshes is expensive and the robot
    isn't mutated by any test in this module.
    """
    return Robot.from_tesseract_support(
        "abb_irb2400", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )


@pytest.fixture
def zigzag_square_program(zigzag_square_poses: list[Pose]) -> MotionProgram:
    """The standard zigzag square (`planning.toolpaths.zigzag_square`) as `MoveL`s.

    The outline poses get the ``OUTLINE`` profile, the zigzag endpoints ``RASTER``.
    """
    program = MotionProgram(
        MANIPULATOR, tcp_frame=TCP_FRAME, working_frame="base_link", profile="cartesian_program"
    )
    for i, pose in enumerate(zigzag_square_poses):
        profile = "OUTLINE" if i < OUTLINE_POSE_COUNT else "RASTER"
        program.linear_to(CartesianTarget(pose, profile=profile))
    return program


@pytest.fixture
def rapid_profiles() -> dict[str, RapidProfile]:
    """RAPID variable bindings keyed by tesseract profile name.

    Every profile name appearing on a Move in the program must have an entry,
    else ``emit_rapid`` raises ``MissingProfileError``. The three planning-
    internal names (``cartesian_program`` / ``freespace_profile`` / ``DEFAULT``)
    ride along on instructions even when no planner has run; they all map to
    the same default profile here.
    """
    default = RapidProfile()  # ABB built-in v200 / z10 / tool0 / wobj0
    profiles: dict[str, RapidProfile] = {
        "OUTLINE": RapidProfile(speed="v100", zone="fine"),
        "RASTER": RapidProfile(),  # same as default — explicit for readability
    }
    profiles.update(dict.fromkeys(["cartesian_program", "freespace_profile", "DEFAULT"], default))
    return profiles


@pytest.fixture
def zigzag_composite(zigzag_square_program: MotionProgram, abb_irb2400: Robot):
    """``MotionProgram`` → ``CompositeInstruction`` without running a planner.

    Sparse-RAPID workflow: the controller does linear interpolation between
    Cartesian waypoints. Running an offline planner first would convert the
    program to dense joint trajectories (``MoveAbsJ`` per timestep), defeating
    the point of ``MoveL``.
    """
    joint_names = abb_irb2400.get_joint_names(MANIPULATOR)
    zigzag_square_program.set_joint_names(joint_names)
    return zigzag_square_program.to_composite_instruction(joint_names=joint_names)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def _diff(expected: str, actual: str) -> str:
    """Return a unified diff between two strings; empty if identical."""
    return "\n".join(
        difflib.unified_diff(
            expected.splitlines(),
            actual.splitlines(),
            fromfile="golden",
            tofile="emit_rapid",
            lineterm="",
        )
    )


def test_rapid_emit_abb_irb2400_zigzag_square(
    zigzag_composite, rapid_profiles: dict[str, RapidProfile]
) -> None:
    """End-to-end: emit the zigzag-square program as RAPID and golden-compare.

    Locks the full ``MODULE`` output — any change in formatter precision,
    statement ordering, or default profile values trips this test.
    """
    actual = emit_rapid(zigzag_composite, rapid_profiles)
    expected = GOLDEN.read_text()
    assert actual == expected, f"Output diverged from golden:\n{_diff(expected, actual)}"


def test_zigzag_square_program_instruction_count(
    zigzag_square_program: MotionProgram, abb_irb2400: Robot
) -> None:
    """Verifies the program contains exactly ``EXPECTED_INSTRUCTION_COUNT`` leaf moves.

    Locks the determined math: 4 outline edges + 1 closure + 2 × N zigzag
    endpoints. A regression that drops a corner or doubles a line surfaces here.
    """
    joint_names = abb_irb2400.get_joint_names(MANIPULATOR)
    zigzag_square_program.set_joint_names(joint_names)
    composite = zigzag_square_program.to_composite_instruction(joint_names=joint_names)
    instructions = composite.getInstructions()
    assert len(instructions) == EXPECTED_INSTRUCTION_COUNT, (
        f"expected exactly {EXPECTED_INSTRUCTION_COUNT} leaf instructions, got {len(instructions)}"
    )
