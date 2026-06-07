"""Pure Fanuc LS (TP ASCII) literal formatters — no state, no I/O, property-testable.

Fanuc orientation W/P/R is **fixed-angle XYZ** (W about X, P about Y, R about Z):
``R = Rz(R)·Ry(P)·Rx(W)``. The extraction below is the canonical atan2 form with a
gimbal-lock branch at |P| ≈ 90° (the twin of the KRL ZYX extractor in
``emitters.krl.targets`` — different axis order, same numerical discipline).

Two ``/POS`` entry shapes are produced here, both as multi-line literal strings the
backend splices into the ``/POS`` section verbatim:

- joint targets (from ``JointMove``) → ``J1 = … deg, … J6 = … deg``;
- Cartesian targets (from ``CartesianMove``) → ``CONFIG`` + ``X/Y/Z`` (mm) + ``W/P/R`` (deg).

The ``/MN`` instruction-line formatters live here too so every string Fanuc parses
is built in one pure, testable place.
"""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.planning import Pose

from ..core.units import Metres, Radians, to_degrees, to_millimetres

#: Joint/orientation angle fields print with 3 decimals (deg).
_ANGLE_DECIMALS = 3
#: Linear position fields print with 3 decimals (mm).
_POS_DECIMALS = 3
#: |cos(P)| floor for the XYZ gimbal singularity (|P| ≈ 90°). Pure numeric guard.
_GIMBAL_FLOOR = 1e-9
#: Robot axis count Fanuc names J1..J6 in a joint ``/POS`` entry.
_ROBOT_AXES = 6
#: Fixed user/tool frame indices for v1 (no frame management yet).
_USER_FRAME = 0
_TOOL_FRAME = 1
#: Default arm-config string. ``N U T`` = non-flip / up / no-turn; the three
#: trailing zeros are turn numbers for J1/J4/J6. Fixed in v1 (no config solving).
_DEFAULT_CONFIG = "N U T, 0, 0, 0"


def wpr_from_pose(pose: Pose) -> tuple[float, float, float]:
    """Return Fanuc ``(W, P, R)`` in degrees for ``pose`` (fixed-angle XYZ).

    Args:
        pose: Cartesian target pose (rotation read from ``pose.rotation_matrix``).

    Returns:
        ``(W, P, R)`` Euler angles in degrees, negative-zero normalized.
    """
    r = np.asarray(pose.rotation_matrix, dtype=np.float64)
    cp = math.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
    if cp > _GIMBAL_FLOOR:
        w = math.atan2(r[2, 1], r[2, 2])
        p = math.atan2(-r[2, 0], cp)
        rr = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: P = ±90°, fold into W, set R = 0
        w = math.atan2(-r[1, 2], r[1, 1])
        p = math.atan2(-r[2, 0], cp)
        rr = 0.0
    return (
        to_degrees(Radians(w)),
        to_degrees(Radians(p)),
        to_degrees(Radians(rr)),
    )


def _fmt(value: float, decimals: int) -> str:
    """Fixed-decimal format with negative-zero normalized to ``0``.

    ``round`` to the display precision then ``+ 0.0`` collapses ``-0.0`` and tiny
    negatives (``-1e-16``) to a clean ``0.000`` so output is deterministic.
    """
    return f"{round(value, decimals) + 0.0:.{decimals}f}"


def joint_pos_block(index: int, joints_rad: tuple[float, ...]) -> str:
    """Format a joint-style ``/POS`` entry ``P[index]{ … J1..J6 = … deg };``.

    Args:
        index: 1-based position index (``P[index]``).
        joints_rad: joint angles in radians; needs at least 6 (robot) axes.

    Returns:
        The full multi-line ``/POS`` entry, newline-terminated by ``};``.

    Raises:
        ValueError: fewer than 6 joint values supplied.
    """
    if len(joints_rad) < _ROBOT_AXES:
        raise ValueError(f"Fanuc joint target needs >={_ROBOT_AXES} joints; got {len(joints_rad)}")
    deg = [_fmt(to_degrees(Radians(j)), _ANGLE_DECIMALS) for j in joints_rad[:_ROBOT_AXES]]
    return (
        f"P[{index}]{{\n"
        f"   GP1:\n"
        f"\tUF : {_USER_FRAME}, UT : {_TOOL_FRAME},\n"
        f"\tJ1 = {deg[0]} deg, J2 = {deg[1]} deg, J3 = {deg[2]} deg,\n"
        f"\tJ4 = {deg[3]} deg, J5 = {deg[4]} deg, J6 = {deg[5]} deg\n"
        f"}};"
    )


def cartesian_pos_block(index: int, pose: Pose) -> str:
    """Format a Cartesian ``/POS`` entry ``P[index]{ … X/Y/Z mm, W/P/R deg };``.

    Args:
        index: 1-based position index (``P[index]``).
        pose: Cartesian target pose (translation in metres → emitted as mm).

    Returns:
        The full multi-line ``/POS`` entry, newline-terminated by ``};``.
    """
    t = pose.translation
    x = _fmt(to_millimetres(Metres(float(t[0]))), _POS_DECIMALS)
    y = _fmt(to_millimetres(Metres(float(t[1]))), _POS_DECIMALS)
    z = _fmt(to_millimetres(Metres(float(t[2]))), _POS_DECIMALS)
    w, p, r = wpr_from_pose(pose)
    return (
        f"P[{index}]{{\n"
        f"   GP1:\n"
        f"\tUF : {_USER_FRAME}, UT : {_TOOL_FRAME},\t\tCONFIG : '{_DEFAULT_CONFIG}',\n"
        f"\tX = {x} mm, Y = {y} mm, Z = {z} mm,\n"
        f"\tW = {_fmt(w, _ANGLE_DECIMALS)} deg, P = {_fmt(p, _ANGLE_DECIMALS)} deg, "
        f"R = {_fmt(r, _ANGLE_DECIMALS)} deg\n"
        f"}};"
    )


def joint_move_line(line_no: int, index: int, speed_percent: float, termination: str) -> str:
    """Format a ``/MN`` joint move ``<n>:J P[index] <pct>% <term> ;``.

    Args:
        line_no: 1-based instruction line number.
        index: position-table index moved to.
        speed_percent: joint speed as a percentage.
        termination: termination type (``FINE`` or ``CNT<n>``).

    Returns:
        The instruction line (no trailing newline).
    """
    return f"{line_no:>4d}:J P[{index}] {speed_percent:g}% {termination} ;"


def linear_move_line(line_no: int, index: int, speed_mms: float, termination: str) -> str:
    """Format a ``/MN`` linear move ``<n>:L P[index] <mm>mm/sec <term> ;``.

    Args:
        line_no: 1-based instruction line number.
        index: position-table index moved to.
        speed_mms: linear speed in millimetres/second.
        termination: termination type (``FINE`` or ``CNT<n>``).

    Returns:
        The instruction line (no trailing newline).
    """
    return f"{line_no:>4d}:L P[{index}] {speed_mms:g}mm/sec {termination} ;"


def comment_line(line_no: int, text: str) -> str:
    """Format a ``/MN`` remark line ``<n>:! <text> ;`` (Fanuc ``!`` comment)."""
    return f"{line_no:>4d}:! {text} ;"


def instruction_line(line_no: int, body: str) -> str:
    """Format a numbered non-motion ``/MN`` line ``<n>:<body> ;``.

    ``body`` is the bare controller statement (e.g. ``WAIT 2.5(sec)``,
    ``DO[3]=ON``); this owns the line-number prefix and trailing `` ;``.
    """
    return f"{line_no:>4d}:{body} ;"
