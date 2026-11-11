"""`KrlProfile`: KRL motion-parameter bundle, frozen, built via factory.

`cp_speed_mms` sets `$VEL.CP` (converted to m/s); `ptp_speed_percent` sets
`BAS(#VEL_PTP, ...)`; `approximation_mm` sets `$APO.CDIS` and appends `C_DIS`
to the move (KSS 8.3 section 11.5.5: "C_DIS Approximation starts, at the
earliest, when the distance to the end point falls below the value of
$APO.CDIS"); `None` means exact stop (no approximation parameter).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.errors import InvalidProfileError

#: `BAS(#VEL_PTP, p)` takes a percentage of maximum axis velocity in (0, 100].
_PTP_PERCENT_MIN = 0.0
_PTP_PERCENT_MAX = 100.0


@dataclass(frozen=True)
class KrlProfile:
    """Motion parameters for KRL emission. Build via `KrlProfile.build`."""

    cp_speed_mms: float = 250.0
    ptp_speed_percent: float = 100.0
    approximation_mm: float | None = None

    @classmethod
    def build(
        cls,
        *,
        cp_speed_mms: float = 250.0,
        ptp_speed_percent: float = 100.0,
        approximation_mm: float | None = None,
    ) -> KrlProfile:
        """Validate and construct.

        Raises:
            InvalidProfileError: a value is non-finite or out of range.
        """
        if not (math.isfinite(cp_speed_mms) and cp_speed_mms > 0.0):
            raise InvalidProfileError(f"cp_speed_mms must be finite and > 0; got {cp_speed_mms}")
        if not (_PTP_PERCENT_MIN < ptp_speed_percent <= _PTP_PERCENT_MAX):
            raise InvalidProfileError(
                f"ptp_speed_percent must be in ({_PTP_PERCENT_MIN}, {_PTP_PERCENT_MAX}]; "
                f"got {ptp_speed_percent}"
            )
        if approximation_mm is not None and not (
            math.isfinite(approximation_mm) and approximation_mm > 0.0
        ):
            raise InvalidProfileError(
                f"approximation_mm must be finite and > 0, or None for exact stop; "
                f"got {approximation_mm}"
            )
        return cls(
            cp_speed_mms=cp_speed_mms,
            ptp_speed_percent=ptp_speed_percent,
            approximation_mm=approximation_mm,
        )
