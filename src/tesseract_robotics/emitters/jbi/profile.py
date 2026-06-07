"""``JbiProfile`` — Motoman JBI motion-parameter bundle, frozen, built via factory.

``linear_speed_mms`` → ``MOVL … V=<mm/s>``; ``joint_speed_percent`` →
``MOVJ … VJ=<percent>``; ``tool`` → the ``///TOOL`` number written to the
``//POS`` header.
"""

from __future__ import annotations

from dataclasses import dataclass

_PERCENT_MIN = 0.0
_PERCENT_MAX = 100.0


@dataclass(frozen=True)
class JbiProfile:
    """Motion parameters for JBI emission. Build via ``JbiProfile.build``."""

    linear_speed_mms: float = 100.0
    joint_speed_percent: float = 50.0
    tool: int = 0

    @classmethod
    def build(
        cls,
        *,
        linear_speed_mms: float = 100.0,
        joint_speed_percent: float = 50.0,
        tool: int = 0,
    ) -> JbiProfile:
        """Validate and construct. Raises ``ValueError`` on out-of-range inputs."""
        if linear_speed_mms <= 0.0:
            raise ValueError(f"linear_speed_mms must be > 0; got {linear_speed_mms}")
        if not (_PERCENT_MIN < joint_speed_percent <= _PERCENT_MAX):
            raise ValueError(
                f"joint_speed_percent must be in ({_PERCENT_MIN}, {_PERCENT_MAX}]; "
                f"got {joint_speed_percent}"
            )
        if tool < 0:
            raise ValueError(f"tool must be >= 0; got {tool}")
        return cls(
            linear_speed_mms=linear_speed_mms,
            joint_speed_percent=joint_speed_percent,
            tool=tool,
        )
