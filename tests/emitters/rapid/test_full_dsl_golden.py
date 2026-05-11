"""Full-DSL golden parity test.

Exercises every DSL class — `Module`, `While`, `Proc`, `For`, `RapidBool`,
`Tooldata`, `Workobject`, `MoveL`, `MoveJ`, `MoveC`, `MoveAbsJ`, `Robtarget`,
`Comment`, `AssignVariable`, `TPWrite`, `Stop`, `Zone`, `Speed`, `Pos` —
plus a direct `RapidWriter.write()` call for raw-line emission — and
golden-locks the output against a checked-in `.mod` file.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tesseract_robotics.emitters.rapid import RapidTarget
from tesseract_robotics.emitters.rapid.rapid_writer import (
    AssignVariable,
    Comment,
    Config,
    ExternalAxis,
    For,
    Module,
    MoveAbsJ,
    MoveC,
    MoveJ,
    MoveL,
    Pos,
    Proc,
    RapidBool,
    RapidWriter,
    Robtarget,
    Speed,
    Stop,
    Tooldata,
    TPWrite,
    While,
    Workobject,
    Zone,
)
from tesseract_robotics.planning import Pose

GOLDEN = Path(__file__).parent / "golden" / "full_dsl.mod"


@pytest.fixture(autouse=True)
def _clear_singleton():
    """Reset RapidWriter between tests so each starts with an empty buffer."""
    RapidWriter().clear()
    yield
    RapidWriter().clear()


def test_full_dsl_golden() -> None:
    """Full DSL coverage — output byte-for-byte against checked-in golden."""
    fr = Pose.from_xyz_quat(0.6, -0.1, 0.8, 0.0, 0.707170, 0.0, 0.707170)
    ext_axe = [1.57, 1.57]

    target_default = RapidTarget(fr, config=Config(0, 0, 0, 0))
    target_with_ext = RapidTarget(
        fr, config=Config(2, 0, 1, 0), external_axis=ExternalAxis(ext_axe)
    )

    rapid = RapidWriter()

    with Module("EXAMPLE"):
        with While("running"):
            with Proc("example_proc"):
                with For("i", 1, 10):
                    bA = RapidBool("flag_a", False)
                    assert bA == "flag_a"
                    bB = RapidBool("flag_b", True)
                    assert bB == "flag_b"
                    Tooldata("tool_demo", 0, 0, 0, [1, 0, 0, 0], [0, 0, 0], 25)
                    Workobject("wobj_demo", 1, 1, 1, [1, 0, 0, 0])
                    MoveL(target_default)
                    MoveJ(target_with_ext)
                    MoveC(target_default, target_default)
                    MoveAbsJ([0, 0, 0, 0, 0, 0], external_axis=ExternalAxis(ext_axe))
                    rapid.write(f"{bB} := False;")
                    Robtarget("target01", target_default)
                    Robtarget("target02", target_with_ext)
                    Comment("raster pass complete")
                    assert AssignVariable("flag_b", False) == "flag_b"
                    TPWrite("message written to teach pendant")
                    Stop()
                    assert Zone("zone_demo") == "zone_demo"
                    assert Speed("v_demo") == "v_demo"
                    assert Pos("pos_demo", 0, 0, 100) == "pos_demo"

    assert rapid.getvalue() == GOLDEN.read_text(), (
        "Output diverged from full-DSL golden — regenerate the .mod file if the "
        "change is intentional."
    )
