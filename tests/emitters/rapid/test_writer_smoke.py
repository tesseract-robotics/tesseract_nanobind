"""Smoke tests for the RAPID DSL: output shape and target/profile bundles."""

from __future__ import annotations

import math

from tesseract_robotics.emitters.core.external_axes import ExternalAxisKind, ExternalAxisValue
from tesseract_robotics.emitters.rapid import Config, ExternalAxis, JointTarget, RapidTarget
from tesseract_robotics.emitters.rapid import rapid_writer as rpdw
from tesseract_robotics.emitters.rapid.profile import RapidProfile, tool0, wobj0
from tesseract_robotics.planning import Pose

_FAST = RapidProfile.build(speed="v200", zone="z10")
_CONF = Config(0, 0, 0, 0)


def test_module_proc_moveabsj_emits_expected_lines() -> None:
    """MoveAbsJ takes radians and prints degrees."""
    with rpdw.RapidWriter() as rapid, rpdw.Module("YO"), rpdw.Proc("main"):
        rpdw.MoveAbsJ(JointTarget.build([0, 0, 0, 0, 0, 0]))
        rpdw.MoveAbsJ(JointTarget.build([math.pi / 2, 0, 0, 0, 0, 0]))
        rpdw.Comment("done")
        rpdw.Stop()
    lines = [line for line in rapid.getvalue().splitlines() if line.strip()]
    assert lines[0] == "MODULE YO"
    assert lines[1] == "    PROC main()"
    assert "[0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]" in lines[2]
    assert "[90.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]" in lines[3]
    assert lines[2].endswith(", v200, z10, tool0;")
    assert lines[4:] == ["        ! done", "        Stop;", "    ENDPROC", "ENDMODULE"]


def test_movej_converts_units_and_quaternion_order() -> None:
    target = RapidTarget(Pose.from_xyz_quat([0.6, -0.1, 0.8], [0.0, 0.0, 0.0, 1.0]), _CONF)
    with rpdw.RapidWriter() as rapid, rpdw.Module("M"), rpdw.Proc("p"):
        rpdw.MoveJ(target)
    out = rapid.getvalue()
    assert "[[600.000, -100.000, 800.000]" in out
    assert "[1.000000000, 0.000000000, 0.000000000, 0.000000000]" in out


def test_movel_uses_profile_tool_and_wobj() -> None:
    target = RapidTarget(Pose.from_xyz(0.5, 0.0, 0.5), _CONF)
    with rpdw.RapidWriter() as rapid, rpdw.Module("M"), rpdw.Proc("p"):
        rpdw.MoveL(target, _FAST, tool0, wobj0)
    [line] = [ln for ln in rapid.getvalue().splitlines() if "MoveL " in ln]
    assert line.endswith(", v200, z10, tool0 \\WObj:=wobj0;")


def test_movel_references_declared_robtarget() -> None:
    target = RapidTarget(Pose.from_xyz(0.5, 0.0, 0.5), _CONF)
    with rpdw.RapidWriter() as rapid, rpdw.Module("M"):
        p1 = rpdw.Robtarget("p1", target)
        with rpdw.Proc("p"):
            rpdw.MoveL(p1, _FAST)
    out = rapid.getvalue()
    assert "CONST robtarget p1 := [[500.000, 0.000, 500.000]," in out
    [line] = [ln.strip() for ln in out.splitlines() if "MoveL " in ln]
    assert line.startswith("MoveL p1,")


def test_movec_accepts_mixed_inline_and_declared_targets() -> None:
    via = RapidTarget(Pose.from_xyz(0.1, 0.0, 0.0), _CONF)
    end = RapidTarget(Pose.from_xyz(0.2, 0.0, 0.0), _CONF)
    with rpdw.RapidWriter() as rapid, rpdw.Module("M"):
        via_var = rpdw.Robtarget("via1", via)
        with rpdw.Proc("p"):
            rpdw.MoveC(via_var, end)
    [line] = [ln.strip() for ln in rapid.getvalue().splitlines() if "MoveC " in ln]
    assert line.startswith("MoveC via1, [[200.000,")


def test_target_carries_config_and_external_axis() -> None:
    eax = ExternalAxis.build([ExternalAxisValue(1.57, ExternalAxisKind.LINEAR)])
    target = RapidTarget(Pose.from_xyz(0.5, 0.0, 0.5), Config(2, 0, 1, 0), eax)
    assert "[2, 0, 1, 0]" in target.literal()
    assert "[1570.000,9E+09,9E+09,9E+09,9E+09,9E+09]" in target.literal()
