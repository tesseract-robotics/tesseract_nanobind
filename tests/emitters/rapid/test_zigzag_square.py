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

import numpy as np
import pytest

from tesseract_robotics.emitters.rapid import RapidProfile, emit_rapid
from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
)

# ---------------------------------------------------------------------------
# Geometry — the determined shape of the test program. All coordinates in
# metres; all angles are encoded in the quaternion below.
# ---------------------------------------------------------------------------

#: Workpiece-plane height above the robot base. Chosen so the entire 1 m × 1 m
#: square sits within the IRB 2400's ~1.55 m reach envelope.
Z_HEIGHT_M: Final[float] = 0.5

#: X-axis extent of the square outline (metres).
X_RANGE_M: Final[tuple[float, float]] = (0.3, 1.3)

#: Y-axis extent of the square outline (metres).
Y_RANGE_M: Final[tuple[float, float]] = (-0.5, 0.5)

#: Spacing between consecutive zigzag scan lines (metres).
ZIGZAG_SPACING_M: Final[float] = 0.1

#: Number of zigzag scan lines: ((Y_RANGE width) / SPACING) + 1.
ZIGZAG_LINE_COUNT: Final[int] = 11

#: TCP frame name — the robot link the planner / emitter treats as the tool tip.
TCP_FRAME: Final[str] = "tool0"

#: Manipulator joint group name in the URDF / SRDF.
MANIPULATOR: Final[str] = "manipulator"

#: Tool-pointing-down orientation: 180° rotation about world Z. Encoded as
#: scalar-last [qx, qy, qz, qw] per project CLAUDE.md convention. After this
#: rotation, the TCP Z-axis points along world -Z (into the workpiece).
TOOL_DOWN_QUAT_XYZW: Final[tuple[float, float, float, float]] = (0.0, 0.0, 1.0, 0.0)

#: Expected leaf-instruction count: 5 outline moves (4 corners + 1 closure)
#: plus 2 × ZIGZAG_LINE_COUNT zigzag endpoints.
EXPECTED_INSTRUCTION_COUNT: Final[int] = 5 + 2 * ZIGZAG_LINE_COUNT  # = 27

GOLDEN: Final[Path] = Path(__file__).parent / "golden" / "abb_irb2400_zigzag_square.mod"


def _pose_at(x: float, y: float) -> Pose:
    """Pose at ``(x, y, Z_HEIGHT_M)`` with the tool pointing down."""
    return Pose.from_xyz_quat([x, y, Z_HEIGHT_M], TOOL_DOWN_QUAT_XYZW)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def abb_irb2400() -> Robot:
    """ABB IRB 2400 robot loaded from tesseract_support.

    Session-scoped: loading the URDF + meshes is expensive and the robot
    isn't mutated by any test in this module.
    """
    return Robot.from_tesseract_support("abb_irb2400")


@pytest.fixture
def zigzag_square_program() -> MotionProgram:
    """1 m × 1 m square outline + zigzag fill at ``ZIGZAG_SPACING_M`` spacing.

    Coordinate frame: tool pointing down (``TOOL_DOWN_QUAT_XYZW``); plane at
    ``Z_HEIGHT_M`` above the robot base. X spans ``X_RANGE_M``; Y spans
    ``Y_RANGE_M``. Square corners + closure produce 5 ``MoveL`` instructions;
    zigzag produces 2 endpoints per line × ``ZIGZAG_LINE_COUNT`` lines.
    """
    program = MotionProgram(MANIPULATOR, tcp_frame=TCP_FRAME, profile="cartesian_program")

    x_lo, x_hi = X_RANGE_M
    y_lo, y_hi = Y_RANGE_M

    # Square outline: 4 corners + explicit closure back to the first corner.
    open_corners: list[tuple[float, float]] = [
        (x_lo, y_lo),
        (x_hi, y_lo),
        (x_hi, y_hi),
        (x_lo, y_hi),
    ]
    for x, y in open_corners + [open_corners[0]]:
        program.linear_to(CartesianTarget(_pose_at(x, y), profile="OUTLINE"))

    # Zigzag fill: alternate sweep direction on each line for shortest transit.
    y_values = np.linspace(y_lo, y_hi, ZIGZAG_LINE_COUNT)
    for i, y in enumerate(y_values):
        x_start, x_end = (x_lo, x_hi) if i % 2 == 0 else (x_hi, x_lo)
        program.linear_to(CartesianTarget(_pose_at(x_start, float(y)), profile="RASTER"))
        program.linear_to(CartesianTarget(_pose_at(x_end, float(y)), profile="RASTER"))
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
    return zigzag_square_program.to_composite_instruction(
        joint_names=joint_names, tcp_frame=TCP_FRAME
    )


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
    composite = zigzag_square_program.to_composite_instruction(
        joint_names=joint_names, tcp_frame=TCP_FRAME
    )
    instructions = composite.getInstructions()
    assert len(instructions) == EXPECTED_INSTRUCTION_COUNT, (
        f"expected exactly {EXPECTED_INSTRUCTION_COUNT} leaf instructions, got {len(instructions)}"
    )
