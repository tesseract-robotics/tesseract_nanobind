"""`JbiProfile`: Motoman JBI motion-parameter bundle, frozen, built via factory.

`linear_speed_mms` is `MOVL ... V=<mm/s>`; `joint_speed_percent` is
`MOVJ ... VJ=<percent>`. The tool number is not a motion parameter: it follows
each move's `tcp_frame` through the frame binding (`jbi_frames`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.errors import InvalidProfileError

_PERCENT_MIN = 0.0
_PERCENT_MAX = 100.0


@dataclass(frozen=True)
class JbiProfile:
    """Motion parameters for JBI emission. Build via `JbiProfile.build`."""

    linear_speed_mms: float = 100.0
    joint_speed_percent: float = 50.0

    @classmethod
    def build(
        cls,
        *,
        linear_speed_mms: float = 100.0,
        joint_speed_percent: float = 50.0,
    ) -> JbiProfile:
        """Validate and construct.

        Raises:
            InvalidProfileError: a value is non-finite or out of range.
        """
        if not (math.isfinite(linear_speed_mms) and linear_speed_mms > 0.0):
            raise InvalidProfileError(
                f"linear_speed_mms must be finite and > 0; got {linear_speed_mms}"
            )
        if not (_PERCENT_MIN < joint_speed_percent <= _PERCENT_MAX):
            raise InvalidProfileError(
                f"joint_speed_percent must be in ({_PERCENT_MIN}, {_PERCENT_MAX}]; "
                f"got {joint_speed_percent}"
            )
        return cls(linear_speed_mms=linear_speed_mms, joint_speed_percent=joint_speed_percent)
