"""Full-DSL golden parity and grammar test.

Exercises every DSL class (`Module`, `Proc`, `While`, `For`, every declaration,
every statement) in a structurally valid module, golden-locks the output, and
checks it against the independent RAPID grammar checker.
"""

from __future__ import annotations

import math
from pathlib import Path

from tesseract_robotics.emitters.core.external_axes import ExternalAxisKind, ExternalAxisValue
from tesseract_robotics.emitters.rapid import Config, ExternalAxis, JointTarget, RapidTarget
from tesseract_robotics.emitters.rapid.profile import RapidProfile
from tesseract_robotics.emitters.rapid.rapid_writer import (
    AssignVariable,
    Comment,
    Conf,
    For,
    Jointtarget,
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
    SetAO,
    SetDO,
    Speed,
    Stop,
    Tooldata,
    TPWrite,
    WaitDI,
    WaitTime,
    While,
    Workobject,
    Zone,
)
from tesseract_robotics.emitters.rapid.syntax import MotionType
from tesseract_robotics.planning import Pose
from tests.emitters.rapid.rapid_grammar import check_module

GOLDEN = Path(__file__).parent / "golden" / "full_dsl.mod"


def _full_dsl() -> str:
    pose = Pose.from_xyz_quat([0.6, -0.1, 0.8], [0.0, 0.707170, 0.0, 0.707170])
    eax = ExternalAxis.build(
        [
            ExternalAxisValue(0.25, ExternalAxisKind.LINEAR),
            ExternalAxisValue(math.pi / 2, ExternalAxisKind.ROTARY),
        ]
    )
    home = RapidTarget(pose, Config(0, 0, 0, 0))
    turned = RapidTarget(pose, Config(-1, 0, 1, 1), eax)
    with RapidWriter() as rapid:
        with Module("EXAMPLE"):
            Comment("module data")
            tool = Tooldata("gun", Pose.from_xyz(0.0, 0.0, 0.2), 2.5, [0.0, 0.0, 0.1])
            wobj = Workobject("table", Pose.from_xyz(1.0, 0.0, 0.4))
            speed = Speed(
                "v_weld",
                tcp_m_per_s=0.05,
                ori_rad_per_s=math.radians(100.0),
                leax_m_per_s=1.0,
                reax_rad_per_s=math.radians(100.0),
            )
            zone = Zone(
                "z_weld",
                finepoint=False,
                pzone_tcp_m=0.001,
                pzone_ori_m=0.0015,
                pzone_eax_m=0.0015,
                zone_ori_rad=math.radians(0.15),
                zone_leax_m=0.0015,
                zone_reax_rad=math.radians(0.15),
            )
            p_home = Robtarget("p_home", home)
            j_park = Jointtarget("j_park", JointTarget.build([0, 0, 0, 0, math.pi / 6, 0]))
            Pos("offset", [0.0, 0.0, 0.1])
            profile = RapidProfile.build(speed=speed.name, zone=zone.name)
            with Proc("main"):
                flag = RapidBool("done", False)
                Conf(MotionType.LINEAR, True)
                MoveAbsJ(j_park, profile, tool.name)
                MoveJ(p_home, profile, tool.name, wobj.name)
                MoveL(turned, profile, tool.name, wobj.name)
                MoveC(home, p_home, profile, tool.name, wobj.name)
                with While("NOT done"):
                    with For("i", 1, 3):
                        SetDO("do_torch", True)
                        WaitTime(0.5)
                        SetAO("ao_voltage", 24.5)
                    WaitDI("di_part", True)
                    AssignVariable(flag, "TRUE")
                TPWrite('done: "pass" \\ ok')
                Stop()
    return rapid.getvalue()


def test_full_dsl_golden() -> None:
    assert _full_dsl() == GOLDEN.read_text(), (
        "Output diverged from full-DSL golden; regenerate the .mod file if the change is intentional."
    )


def test_full_dsl_is_valid_rapid() -> None:
    check_module(_full_dsl())
