"""RAPID target bundles + literal formatters — pure, no writer state.

The twin of the other brands' ``targets.py``. Holds the robtarget/jointtarget
data bundles (``RapidTarget`` / ``JointTarget``), the axis-quadrant ``Config``
and ``ExternalAxis`` value types, and the private ``/MN``-line and literal
formatters consumed by the DSL move/declaration classes. All conversions
(m → mm, scalar-last → scalar-first quaternion) live here, inline.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_common import Quaterniond

from .profile import RapidProfile
from .utils import format_ext_axis


@dataclass
class Config:
    """RAPID robtarget axis-quadrant config — 4 integers ``[cf1, cf4, cf6, cfx]``.

    ``cf1``, ``cf4``, ``cf6`` are rotational quadrant indices for axes 1, 4, and
    6: ``0`` = ``[0, π/2)``, ``1`` = ``[π/2, π)``, etc.; negative values use the
    inverse convention. ``cfx`` is a robot-model-specific encoded configuration
    for the remaining axes (2, 3, 5). Defaults to ``[1, 0, 0, 0]`` — RAPID's most
    common axis-quadrant placeholder. For robot-specific home configurations, use
    ``Config.neutral_for(name)``.
    """

    cf1: int = 1
    cf4: int = 0
    cf6: int = 0
    cfx: int = 0

    def __str__(self) -> str:
        return f"[{self.cf1}, {self.cf4}, {self.cf6}, {self.cfx}]"

    @classmethod
    def neutral_for(cls, robot: str) -> Config:
        """Return a known-good neutral axis-quadrant config for the named robot.

        The "neutral" config is what the controller reports when the arm is at
        its home position with no axes wrapped past their quadrant boundaries —
        useful as a baseline before ``ConfL \\Off`` paths.

        Raises ``ValueError`` for unregistered robots; pass ``Config(...)``
        explicitly instead.

        Example:
            ```python
            config = Config.neutral_for("abb_irb2400")
            target = RapidTarget(pose, config=config)
            ```
        """
        table = {
            "abb_irb1200": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb2400": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb4400": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb6640": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb6700": cls(cf1=1, cf4=0, cf6=0, cfx=0),
        }
        if robot not in table:
            raise ValueError(
                f"no neutral Config registered for robot {robot!r}; "
                f"known robots: {sorted(table.keys())}. "
                f"Pass `Config(cf1, cf4, cf6, cfx)` explicitly."
            )
        return table[robot]


@dataclass
class ExternalAxis:
    """RAPID external-axis values — up to six slots; unset slots serialize to the
    ``9E+09`` sentinel ABB uses for "no value".

    Each entry is either a raw ``float`` (formatted by ``str()`` — the DSL
    hand-authoring convention) or an already-formatted display token ``str``
    (what ``RapidBackend`` emits: unit- and precision-correct mm/deg from the
    ``core.units`` chokepoint). ``str`` is idempotent under ``format_ext_axis``,
    so both render identically and float callers stay byte-for-byte unchanged.
    """

    values: Sequence[float | str] = ()

    def __str__(self) -> str:
        return format_ext_axis(self.values)

    @classmethod
    def from_list(cls, values: Sequence[float | str]) -> ExternalAxis:
        return cls(values=values)


@dataclass
class RapidTarget:
    """RAPID robtarget bundle — pose + axis-quadrant config + external axes.

    The ``.robtarget`` property serializes this to a RAPID robtarget literal,
    handling both axis-convention conversions inline (m → mm; scalar-last
    ``[qx,qy,qz,qw]`` → scalar-first ``[w,x,y,z]``).
    """

    pose: Pose
    config: Config = field(default_factory=Config)
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    @property
    def robtarget(self) -> str:
        """Serialize to ``[[mm xyz], [wxyz quat], [config], [ext]]``."""
        t = self.pose.translation
        qx, qy, qz, qw = Quaterniond(self.pose.linear).coeffs()
        x_mm, y_mm, z_mm = t[0] * 1000.0, t[1] * 1000.0, t[2] * 1000.0
        pos = f"[{x_mm:.4f}, {y_mm:.4f}, {z_mm:.4f}]"
        rot = f"[{qw:.8f}, {qx:.8f}, {qy:.8f}, {qz:.8f}]"
        return "[" + ",".join([pos, rot, str(self.config), str(self.external_axis)]) + "]"


@dataclass
class JointTarget:
    """RAPID jointtarget bundle — 6-axis joint values (radians) + external axes.

    Mirrors ``RapidTarget`` for joint-space moves. ``__str__`` returns the RAPID
    jointtarget literal with joints converted radians → degrees and external axes
    padded to six slots with ``9E+09``.
    """

    joints_rad: ArrayLike
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    def __str__(self) -> str:
        joints = np.asarray(self.joints_rad, dtype=np.float64).ravel()
        if joints.size != 6:
            raise ValueError(
                f"JointTarget requires exactly 6 joint values; got {joints.size} ({joints!r})"
            )
        deg = [math.degrees(float(j)) for j in joints]
        joints_str = (
            f"[{deg[0]:.2f}, {deg[1]:.2f}, {deg[2]:.2f}, {deg[3]:.2f}, {deg[4]:.2f}, {deg[5]:.2f}]"
        )
        return f"[{joints_str}, {self.external_axis}]"


# ---------------------------------------------------------------------------
# Private formatters — consumed by the DSL move/declaration classes.
# ---------------------------------------------------------------------------


def _move_line(keyword: str, robtarget: str, profile: RapidProfile) -> str:
    """Build a ``MoveL`` / ``MoveJ`` line referencing one robtarget."""
    return (
        f"{keyword} {robtarget}, "
        f"{profile.speed}, {profile.zone}, {profile.tool} \\Wobj:={profile.wobj};"
    )


def _movec_line(robtarget_via: str, robtarget_end: str, profile: RapidProfile) -> str:
    """Build a ``MoveC`` line referencing two robtargets (via, end)."""
    return (
        f"MoveC {robtarget_via}, {robtarget_end}, "
        f"{profile.speed}, {profile.zone}, {profile.tool} \\Wobj:={profile.wobj};"
    )


def _xyz_mm(x: float, y: float, z: float) -> str:
    """Format ``[x, y, z]`` for ``tooldata`` / ``wobjdata`` declarations (3-decimal mm)."""
    return f"[ {x:.3f}, {y:.3f}, {z:.3f} ]"


def _quat_wxyz(q: Sequence[float]) -> str:
    """Format scalar-first ``[w, x, y, z]`` quaternion (9-decimal precision)."""
    return f"[ {q[0]:.9f}, {q[1]:.9f}, {q[2]:.9f}, {q[3]:.9f} ]"


def _cog_kg(cog: Sequence[float], kg: float) -> str:
    """Format ``kg, [cog_x, cog_y, cog_z]`` for the ``tooldata`` inertia frame."""
    return f"{kg}, [ {cog[0]:.3f}, {cog[1]:.3f}, {cog[2]:.3f} ]"


def _robtarget_str(t: RapidTarget | str) -> str:
    """Accept a ``RapidTarget`` (→ inline robtarget literal) or a ``str`` (→ an
    already-declared robtarget variable name).

    Lets idiomatic RAPID workflows split between ``MoveL [[...],...], v200, ...``
    (one-shot, all data inline) and ``CONST robtarget pN := [...]; MoveL pN, ...``
    (large declared block, referenced by name). The latter is preferred for long
    toolpaths and operator-tunable programs.
    """
    return t if isinstance(t, str) else t.robtarget
