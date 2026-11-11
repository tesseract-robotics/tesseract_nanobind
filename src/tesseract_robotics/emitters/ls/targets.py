"""Pure Fanuc LS (TP ASCII) literal formatters: no state, no I/O, property-testable.

Fanuc orientation W/P/R is fixed-angle XYZ (W about X, P about Y, R about Z):
`R = Rz(R) Ry(P) Rx(W)`. The extraction is the canonical atan2 form. At the
gimbal lock (P = ±90 deg) only `W ∓ R` is determined; with `R = 0`,
`Rz(0) Ry(±90) Rx(W)` has `r12 = -sin(W)` and `r11 = cos(W)` on both branches,
so `W = atan2(-r12, r11)` is exact there.

Two `/POS` entry shapes are produced, both multi-line literal strings the
backend splices into the `/POS` section verbatim:

- joint targets (from `JointMove`): `J1 = ... deg, ..., J6 = ... deg`;
- Cartesian targets (from `CartesianMove`): `CONFIG` + `X/Y/Z` (mm) + `W/P/R` (deg).

The `/MN` instruction-line formatters live here too, so every string Fanuc
parses is built in one pure, testable place.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Final

import numpy as np

from tesseract_robotics.planning.transforms import Pose

from ..core.errors import ExternalAxisError
from ..core.precision import DEG_DECIMALS, MM_DECIMALS, fixed
from ..core.units import Metres, Radians, to_degrees, to_millimetres
from .syntax import BRAND, FanucArmConfig, comment_text, turn_number

#: Robot axes Fanuc names J1..J6 in a joint `/POS` entry.
_ROBOT_AXES: Final[int] = 6
#: `cos(P)` floor below which the extraction switches to the gimbal branch;
#: sqrt(machine epsilon), see `emitters.krl.targets._GIMBAL_FLOOR`.
_GIMBAL_FLOOR: Final[float] = float(np.sqrt(np.finfo(np.float64).eps))


def wpr_from_pose(pose: Pose) -> tuple[float, float, float]:
    """Return Fanuc `(W, P, R)` in degrees for `pose` (fixed-angle XYZ)."""
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    cos_p = math.hypot(r[0, 0], r[1, 0])
    p = math.atan2(-r[2, 0], cos_p)
    if cos_p > _GIMBAL_FLOOR:
        w = math.atan2(r[2, 1], r[2, 2])
        rr = math.atan2(r[1, 0], r[0, 0])
    else:
        w = math.atan2(-r[1, 2], r[1, 1])
        rr = 0.0
    return (to_degrees(Radians(w)), to_degrees(Radians(p)), to_degrees(Radians(rr)))


def _six(joints: Sequence[float]) -> Sequence[float]:
    if len(joints) != _ROBOT_AXES:
        raise ExternalAxisError(len(joints), _ROBOT_AXES, BRAND)
    return joints


def config_string(config: FanucArmConfig, joints_rad: Sequence[float]) -> str:
    """`'N U T, t1, t4, t6'`: placement letters plus J1/J4/J6 turn numbers."""
    q = _six(joints_rad)
    turns = (turn_number(Radians(q[0])), turn_number(Radians(q[3])), turn_number(Radians(q[5])))
    return f"{config.letters()}, {turns[0]}, {turns[1]}, {turns[2]}"


def joint_pos_block(index: int, joints_rad: Sequence[float], *, uf: int, ut: int) -> str:
    """Joint-style `/POS` entry `P[index]{ ... J1..J6 = ... deg };`.

    Raises:
        ExternalAxisError: not exactly six joint values.
    """
    deg = [fixed(to_degrees(Radians(j)), DEG_DECIMALS) for j in _six(joints_rad)]
    return (
        f"P[{index}]{{\n"
        f"   GP1:\n"
        f"\tUF : {uf}, UT : {ut},\n"
        f"\tJ1 = {deg[0]} deg, J2 = {deg[1]} deg, J3 = {deg[2]} deg,\n"
        f"\tJ4 = {deg[3]} deg, J5 = {deg[4]} deg, J6 = {deg[5]} deg\n"
        f"}};"
    )


def cartesian_pos_block(index: int, pose: Pose, *, uf: int, ut: int, config: str) -> str:
    """Cartesian `/POS` entry `P[index]{ ... CONFIG, X/Y/Z mm, W/P/R deg };`."""
    x, y, z = (fixed(to_millimetres(Metres(float(v))), MM_DECIMALS) for v in pose.translation)
    w, p, r = (fixed(v, DEG_DECIMALS) for v in wpr_from_pose(pose))
    return (
        f"P[{index}]{{\n"
        f"   GP1:\n"
        f"\tUF : {uf}, UT : {ut},\t\tCONFIG : '{config}',\n"
        f"\tX = {x} mm, Y = {y} mm, Z = {z} mm,\n"
        f"\tW = {w} deg, P = {p} deg, R = {r} deg\n"
        f"}};"
    )


def joint_move_line(line_no: int, index: int, speed_percent: float, termination: str) -> str:
    """`/MN` joint move `<n>:J P[index] <pct>% <term> ;`."""
    return f"{line_no:>4d}:J P[{index}] {speed_percent:g}% {termination} ;"


def linear_move_line(line_no: int, index: int, speed_mms: float, termination: str) -> str:
    """`/MN` linear move `<n>:L P[index] <mm>mm/sec <term> ;`."""
    return f"{line_no:>4d}:L P[{index}] {speed_mms:g}mm/sec {termination} ;"


def comment_line(line_no: int, text: str) -> str:
    """`/MN` remark line `<n>:! <text> ;`; `text` must be one line without `;`."""
    return f"{line_no:>4d}:! {comment_text(text)} ;"


def instruction_line(line_no: int, body: str) -> str:
    """Numbered non-motion `/MN` line `<n>:<body> ;`."""
    return f"{line_no:>4d}:{body} ;"
