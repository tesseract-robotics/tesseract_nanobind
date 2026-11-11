"""Yaskawa Motoman INFORM (JBI) lexical rules, tool binding and pulse calibration.

Sources: Yaskawa DX100 INFORM manual (RE-CKI-A452): comment `'<string>`,
"String: 32 characters"; `AOUT AO#(<port>) <voltage>`, port 1 to 40, voltage
-14.00 to +14.00 V in 0.01 V units; `DOUT OT#(<n>) ON|OFF`. Motoman knowledge
base, INFORM header fields: `//NAME` up to 32 characters; `///TOOL` 0 to 63;
`///POSTYPE PULSE` rows are encoder pulses per axis.

Pulse positions are joint angles times the axis pulse ratio, which is a
controller parameter of the specific robot. The emitter has no default for it:
the caller passes a `JbiPulseCalibration`.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Final

from ..core.errors import InvalidCalibrationError, InvalidFrameBindingError
from ..core.frames import FrameBinding
from ..core.naming import IdentifierRules
from ..core.text import single_line
from ..core.units import Radians

BRAND: Final[str] = "JBI"

JBI_IDENTIFIERS: Final[IdentifierRules] = IdentifierRules(
    brand=BRAND,
    pattern=re.compile(r"[A-Za-z][A-Za-z0-9_]*"),
    pattern_text="a letter followed by letters, digits or '_'",
    max_len=32,
    reserved=frozenset(),
    case_insensitive=False,
)

#: Maximum INFORM comment length (characters).
COMMENT_MAX_LEN: Final[int] = 32
#: Valid `///TOOL` numbers.
TOOL_NUMBERS: Final[range] = range(0, 64)
#: Valid analog output ports `AO#(n)`.
ANALOG_PORTS: Final[range] = range(1, 41)
#: Analog output voltage range (V).
ANALOG_MAX_V: Final[float] = 14.0
#: Robot axes in a PULSE row (S, L, U, R, B, T).
ROBOT_AXES: Final[int] = 6


def jbi_frames(*, tools: Mapping[str, int]) -> FrameBinding[int, int]:
    """Bind tesseract TCP frames to Motoman tool numbers (0..63).

    JBI positions are emitted as joint pulses, so no working frame is involved.

    Raises:
        InvalidFrameBindingError: a tool number is out of range.
    """
    for frame, number in tools.items():
        if isinstance(number, bool) or not isinstance(number, int) or number not in TOOL_NUMBERS:
            raise InvalidFrameBindingError(
                f"JBI: {frame!r} binds to tool {number}; valid: 0..{TOOL_NUMBERS.stop - 1}"
            )
    return FrameBinding.build(work_frames={}, tools=tools)


@dataclass(frozen=True)
class JbiPulseCalibration:
    """Per-axis encoder calibration: `pulse = zero_pulse + round(angle * pulses_per_rad)`.

    Attributes:
        pulses_per_rad: Signed pulses per radian of each robot axis (the sign
            encodes the encoder direction relative to the tesseract joint axis).
        zero_pulses: Pulse count at tesseract joint angle 0 of each axis.
    """

    pulses_per_rad: tuple[float, float, float, float, float, float]
    zero_pulses: tuple[int, int, int, int, int, int]

    @classmethod
    def build(
        cls, *, pulses_per_rad: Sequence[float], zero_pulses: Sequence[int]
    ) -> JbiPulseCalibration:
        """Validate and construct.

        Raises:
            InvalidCalibrationError: not six entries each, a ratio that is zero or
                non-finite, or a non-integer zero pulse.
        """
        if len(pulses_per_rad) != ROBOT_AXES or len(zero_pulses) != ROBOT_AXES:
            raise InvalidCalibrationError(
                f"JBI calibration needs {ROBOT_AXES} ratios and {ROBOT_AXES} zero pulses; "
                f"got {len(pulses_per_rad)} and {len(zero_pulses)}"
            )
        ratios = tuple(float(r) for r in pulses_per_rad)
        if any(not math.isfinite(r) or r == 0.0 for r in ratios):
            raise InvalidCalibrationError(
                f"pulses_per_rad must be finite and nonzero; got {ratios}"
            )
        if any(isinstance(z, bool) or not isinstance(z, int) for z in zero_pulses):
            raise InvalidCalibrationError(f"zero_pulses must be integers; got {tuple(zero_pulses)}")
        z = tuple(zero_pulses)
        return cls(
            (ratios[0], ratios[1], ratios[2], ratios[3], ratios[4], ratios[5]),
            (z[0], z[1], z[2], z[3], z[4], z[5]),
        )

    def pulses(self, joints: Sequence[Radians]) -> tuple[int, ...]:
        """Encoder pulses of six joint angles (nearest integer, ties to even)."""
        return tuple(
            zero + round(angle * ratio)
            for angle, ratio, zero in zip(
                joints, self.pulses_per_rad, self.zero_pulses, strict=True
            )
        )


def comment_text(text: str) -> str:
    """Validate `text` as an INFORM comment: one line, at most 32 characters."""
    return single_line(text, brand=BRAND, max_len=COMMENT_MAX_LEN)
