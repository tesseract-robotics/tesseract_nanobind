"""``KrlProfile`` — KRL motion-parameter bundle, frozen, built via factory.

``cp_speed_mms`` → ``$VEL.CP`` (converted to m/s); ``ptp_speed_percent`` →
``BAS(#VEL_PTP, …)``; ``approximation_mm`` → ``$APO.CDIS`` (None ⇒ exact stop).
"""

from __future__ import annotations

from dataclasses import dataclass

_PTP_PERCENT_MIN = 0.0
_PTP_PERCENT_MAX = 100.0


@dataclass(frozen=True)
class KrlProfile:
    """Motion parameters for KRL emission. Build via ``KrlProfile.build``."""

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
        """Validate and construct. Raises ``ValueError`` on out-of-range inputs."""
        if cp_speed_mms <= 0.0:
            raise ValueError(f"cp_speed_mms must be > 0; got {cp_speed_mms}")
        if not (_PTP_PERCENT_MIN < ptp_speed_percent <= _PTP_PERCENT_MAX):
            raise ValueError(
                f"ptp_speed_percent must be in ({_PTP_PERCENT_MIN}, {_PTP_PERCENT_MAX}]; "
                f"got {ptp_speed_percent}"
            )
        if approximation_mm is not None and approximation_mm < 0.0:
            raise ValueError(f"approximation_mm must be >= 0 or None; got {approximation_mm}")
        return cls(
            cp_speed_mms=cp_speed_mms,
            ptp_speed_percent=ptp_speed_percent,
            approximation_mm=approximation_mm,
        )
