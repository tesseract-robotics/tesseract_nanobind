"""Smoke tests for the RAPID DSL — verifies the with-block DSL
still produces the expected output shape after modernization, and that the
DSL classes accept `(RapidTarget, RapidProfile)` bundles."""

from __future__ import annotations

import math

import pytest

from tesseract_robotics.emitters.rapid import RapidProfile, RapidTarget
from tesseract_robotics.emitters.rapid import rapid_writer as rpdw
from tesseract_robotics.planning import Pose

# Sample profile fixture reused across move-emission tests.
_FAST = RapidProfile(speed="v200", zone="z10", tool="tool0", wobj="wobj0")


@pytest.fixture(autouse=True)
def _clear_singleton():
    """Reset the RapidWriter singleton's buffer + indent between tests.

    Note: must call .clear() on the SAME instance (not .reset() which would
    create a new one and orphan RapidCommand.rapid which captured the old
    reference at class-definition time)."""
    rpdw.RapidWriter().clear()
    yield
    rpdw.RapidWriter().clear()


def test_module_proc_moveabsj_emits_expected_lines():
    """MoveAbsJ takes radians (tesseract convention) and converts to degrees."""
    rapid = rpdw.RapidWriter()
    with rpdw.Module("YO"):
        with rpdw.Proc("main"):
            rpdw.MoveAbsJ(rpdw.JointTarget([0, 0, 0, 0, 0, 0]))
            rpdw.MoveAbsJ(rpdw.JointTarget([math.pi / 2, 0, 0, 0, 0, 0]))
            rpdw.Comment("done")
            rpdw.Stop()

    output = rapid.getvalue()
    lines = [line for line in output.splitlines() if line.strip()]

    assert lines[0] == "MODULE YO"
    assert lines[1] == "    PROC main()"
    assert lines[2].startswith("        MoveAbsJ ")
    assert "[0.00, 0.00, 0.00, 0.00, 0.00, 0.00]" in lines[2]
    assert "[90.00, 0.00, 0.00, 0.00, 0.00, 0.00]" in lines[3]
    assert lines[4] == "        ! done"
    assert lines[5] == "        Stop;"
    assert lines[6] == "    ENDPROC"
    assert lines[7] == "ENDMODULE"


def test_movej_takes_target_bundle():
    """MoveJ takes a `RapidTarget` bundle (m → mm, scalar-last → scalar-first
    conversion inside `robtarget_from_pose`)."""
    target = RapidTarget(Pose.from_xyz_quat(0.6, -0.1, 0.8, 0.0, 0.0, 0.0, 1.0))

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            rpdw.MoveJ(target)

    output = rapid.getvalue()
    assert "MoveJ " in output
    # Position rendered in mm with 4 decimals
    assert "[[600.0000, -100.0000, 800.0000]" in output
    # Quaternion rendered scalar-first w,x,y,z = [1, 0, 0, 0]
    assert "[1.00000000, 0.00000000, 0.00000000, 0.00000000]" in output


def test_movec_takes_via_end_target_bundles():
    """`MoveC` takes (via_target, end_target) — two independent `RapidTarget` bundles."""
    via = RapidTarget(Pose.from_xyz_quat(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    end = RapidTarget(Pose.from_xyz_quat(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            rpdw.MoveC(via, end)

    output = rapid.getvalue()
    assert output.count("MoveC ") == 1
    assert "[[100.0000," in output
    assert "[[200.0000," in output


def test_movel_uses_profile_bundle():
    """`MoveL(target, profile)` substitutes profile vars in the emitted line."""
    target = RapidTarget(Pose.from_xyz_quat(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0))

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            rpdw.MoveL(target, _FAST)

    movel_lines = [line for line in rapid.getvalue().splitlines() if "MoveL " in line]
    assert len(movel_lines) == 1
    assert ", v200, z10, tool0 \\Wobj:=wobj0;" in movel_lines[0]


def test_movel_accepts_robtarget_varname_string():
    """`MoveL("p1", profile)` references a previously-declared robtarget by
    name, instead of inlining a literal — the idiomatic style for long
    toolpaths and operator-tunable programs.

    The `Robtarget` declaration class returns the varname, so the round-trip
    is `varname = Robtarget(name, target); MoveL(varname, profile)`.
    """
    from tesseract_robotics.emitters.rapid.rapid_writer import Robtarget

    target = RapidTarget(Pose.from_xyz_quat(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0))

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            p1 = Robtarget("p1", target)
            assert p1 == "p1"
            rpdw.MoveL(p1, _FAST)

    out = rapid.getvalue()
    assert "PERS p1 := " in out  # declaration emitted first
    # Reference, not literal — line starts with `MoveL p1,` not `MoveL [[...`
    movel_lines = [line for line in out.splitlines() if "MoveL " in line]
    assert len(movel_lines) == 1
    assert movel_lines[0].lstrip().startswith("MoveL p1,")
    assert movel_lines[0].rstrip().endswith(", v200, z10, tool0 \\Wobj:=wobj0;")


def test_movec_accepts_mixed_inline_and_varname_targets():
    """`MoveC` accepts each of via/end independently as RapidTarget or str."""
    from tesseract_robotics.emitters.rapid.rapid_writer import Robtarget

    via_target = RapidTarget(Pose.from_xyz_quat(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    end_target = RapidTarget(Pose.from_xyz_quat(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            via_var = Robtarget("via1", via_target)
            rpdw.MoveC(via_var, end_target)  # via=str, end=RapidTarget (inline)

    movec_lines = [line for line in rapid.getvalue().splitlines() if "MoveC " in line]
    assert len(movec_lines) == 1
    line = movec_lines[0].lstrip()
    assert line.startswith("MoveC via1,")
    assert "[[200.0000," in line  # end is inline robtarget literal


def test_target_carries_config_and_external_axis():
    """`RapidTarget.config` and `.external_axis` flow into the emitted robtarget."""
    from tesseract_robotics.emitters.rapid.rapid_writer import Config, ExternalAxis

    target = RapidTarget(
        Pose.from_xyz_quat(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
        config=Config(2, 0, 1, 0),
        external_axis=ExternalAxis([1.57, 1.57]),
    )

    rapid = rpdw.RapidWriter()
    with rpdw.Module("M"):
        with rpdw.Proc("p"):
            rpdw.MoveL(target, _FAST)

    out = rapid.getvalue()
    assert "[2, 0, 1, 0]" in out
    assert "[1.57,1.57,9E+09,9E+09,9E+09,9E+09]" in out


def test_config_neutral_for_known_robot():
    """`Config.neutral_for("abb_irb2400")` returns the registered known-good
    neutral axis-quadrant config for that robot."""
    from tesseract_robotics.emitters.rapid.rapid_writer import Config

    cfg = Config.neutral_for("abb_irb2400")
    assert str(cfg) == "[1, 0, 0, 0]"


def test_config_neutral_for_unknown_robot_raises():
    """Unregistered robot names fail loud with the known-robots list."""
    from tesseract_robotics.emitters.rapid.rapid_writer import Config

    with pytest.raises(ValueError, match="no neutral Config registered"):
        Config.neutral_for("unregistered_robot")
