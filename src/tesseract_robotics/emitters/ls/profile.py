"""``LsProfile`` — Fanuc LS motion-parameter bundle, frozen, built via factory.

``linear_speed_mms`` → the ``<n>mm/sec`` field on ``L`` moves; ``joint_speed_percent``
→ the ``<n>%`` field on ``J`` moves; ``termination`` → the ``FINE`` / ``CNT<n>`` token
that closes every motion line. The twin of ``emitters.krl.profile.KrlProfile``.
"""

from __future__ import annotations

from dataclasses import dataclass

_PERCENT_MIN = 0.0
_PERCENT_MAX = 100.0


@dataclass(frozen=True)
class LsProfile:
    """Motion parameters for Fanuc LS emission. Build via ``LsProfile.build``."""

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
        """Validate and construct. Raises ``ValueError`` on out-of-range inputs.

        Args:
            linear_speed_mms: linear move speed (mm/sec); must be > 0.
            joint_speed_percent: joint move speed as a percentage; in (0, 100].
            termination: ``FINE`` (exact stop) or ``CNT<n>`` (blend); non-empty.

        Returns:
            A validated, frozen ``LsProfile``.

        Raises:
            ValueError: any field out of range or an empty termination token.
        """
        if linear_speed_mms <= 0.0:
            raise ValueError(f"linear_speed_mms must be > 0; got {linear_speed_mms}")
        if not (_PERCENT_MIN < joint_speed_percent <= _PERCENT_MAX):
            raise ValueError(
                f"joint_speed_percent must be in ({_PERCENT_MIN}, {_PERCENT_MAX}]; "
                f"got {joint_speed_percent}"
            )
        if not termination:
            raise ValueError("termination must be a non-empty token (e.g. 'FINE' or 'CNT100')")
        return cls(
            linear_speed_mms=linear_speed_mms,
            joint_speed_percent=joint_speed_percent,
            termination=termination,
        )
