"""`LsProfile`: Fanuc LS motion-parameter bundle, frozen, built via factory.

`linear_speed_mms` is the `<n>mm/sec` field of `L` moves; `joint_speed_percent`
is the `<n>%` field of `J` moves; `termination` is the `FINE` / `CNT<n>` token
that closes every motion line.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Final

from ..core.errors import InvalidProfileError

_PERCENT_MIN = 0.0
_PERCENT_MAX = 100.0
#: `FINE` or `CNT0`..`CNT100` (continuous-termination smoothness, percent).
_TERMINATION: Final[re.Pattern[str]] = re.compile(r"FINE|CNT(?:100|[1-9]?[0-9])")


@dataclass(frozen=True)
class LsProfile:
    """Motion parameters for Fanuc LS emission. Build via `LsProfile.build`."""

    linear_speed_mms: float = 100.0
    joint_speed_percent: float = 100.0
    termination: str = "FINE"

    @classmethod
    def build(
        cls,
        *,
        linear_speed_mms: float = 100.0,
        joint_speed_percent: float = 100.0,
        termination: str = "FINE",
    ) -> LsProfile:
        """Validate and construct.

        Args:
            linear_speed_mms: Linear move speed (mm/sec); finite and > 0.
            joint_speed_percent: Joint move speed as a percentage; in (0, 100].
            termination: `FINE` (exact stop) or `CNT0`..`CNT100` (blend).

        Raises:
            InvalidProfileError: a field is out of range or the termination is malformed.
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
        if _TERMINATION.fullmatch(termination) is None:
            raise InvalidProfileError(
                f"termination must be 'FINE' or 'CNT0'..'CNT100'; got {termination!r}"
            )
        return cls(
            linear_speed_mms=linear_speed_mms,
            joint_speed_percent=joint_speed_percent,
            termination=termination,
        )
