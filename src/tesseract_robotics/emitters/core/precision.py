"""One display-precision policy for every dialect, derived from physical resolution.

The resolution target is 1 µm at the TCP. Industrial arms repeat to roughly
±20 µm, so 1 µm is an order of magnitude below anything a controller can
reproduce: printing finer adds no information, printing coarser throws away
plan accuracy. Every other quantity is sized to cost no more than that 1 µm:

- an angle error δ (rad) moves a point at reach `L` by `L·δ`; at a 3 m reach
  1 µm is 3.3e-7 rad = 1.9e-5 deg, so degrees print with 4 decimals (5e-5 deg
  half-step → 2.6 µm at 3 m, 0.9 µm at 1 m) and radians with 6 decimals
  (5e-7 rad half-step → 1.5 µm at 3 m);
- a unit-quaternion component error ε rotates by about 2ε, so 9 decimals keep
  the orientation error near 1e-9 rad and the quaternion norm within 2e-9 of 1,
  well inside the normalization check controllers apply to orientation fields.

`fixed` is the one formatter: it rounds to the display precision and folds
negative zero (`-0.0`, or `-1e-16` rounding to zero) to a clean `0`, so output is
deterministic and goldens diff meaningfully.
"""

from __future__ import annotations

from typing import Final

#: Millimetre fields: 3 decimals = 1 µm (see module docstring).
MM_DECIMALS: Final[int] = 3
#: Metre fields (URScript poses): 6 decimals = 1 µm.
M_DECIMALS: Final[int] = 6
#: Degree fields (joints, Euler angles, rotary external axes): 4 decimals.
DEG_DECIMALS: Final[int] = 4
#: Radian fields (URScript joints, rotation vectors): 6 decimals.
RAD_DECIMALS: Final[int] = 6
#: Unit-quaternion components: 9 decimals.
QUAT_DECIMALS: Final[int] = 9


def fixed(value: float, decimals: int) -> str:
    """Fixed-decimal text for `value`, with negative zero folded to `0`.

    Args:
        value: The number to print.
        decimals: Digits after the decimal point.

    Returns:
        `value` rounded to `decimals` places, never rendered as `-0.000…`.
    """
    return f"{round(value, decimals) + 0.0:.{decimals}f}"
