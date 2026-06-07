"""RapidBackend: IR events → RAPID via the existing DSL, byte-identical."""

from __future__ import annotations

from tesseract_robotics.emitters.core.backend import _dispatch
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    MoveKind,
    SetDigital,
)
from tesseract_robotics.emitters.rapid.backend import RapidBackend
from tesseract_robotics.emitters.rapid.profile import RapidProfile
from tesseract_robotics.planning import Pose

_PROFILE = RapidProfile(speed="v200", zone="z10", tool="tool0", wobj="wobj0")


def _run(events, profiles) -> str:
    backend = RapidBackend(profiles=profiles, module_name="M", proc_name="main")
    backend.prog_start()
    for e in events:
        _dispatch(e, backend)
    return next(iter(backend.prog_finish().values()))


def test_timer_originated_setdigital_renders_plain_doN() -> None:
    # key=None, index=7 → "do7" (NOT "do{7}")
    src = _run([SetDigital(key=None, index=7, value=True)], {})
    assert "SetDO do7, 1;" in src


def test_named_setdigital_with_index_renders_subscript() -> None:
    src = _run([SetDigital(key="do_array", index=3, value=False)], {})
    assert "SetDO do_array{3}, 0;" in src


def test_named_setdigital_index0_renders_bare() -> None:
    src = _run([SetDigital(key="do_torch", index=0, value=True)], {})
    assert "SetDO do_torch, 1;" in src


def test_linear_move_emits_movel() -> None:
    src = _run(
        [
            CartesianMove(
                pose=Pose.from_xyz_quat([0.5, -0.2, 0.62], [0, 0, 0, 1]),
                kind=MoveKind.LINEAR,
                profile="P",
                seed_joints=None,
            )
        ],
        {"P": _PROFILE},
    )
    assert "MoveL [[500.0000, -200.0000, 620.0000]," in src
    assert src.strip().startswith("MODULE M") and src.strip().endswith("ENDMODULE")
