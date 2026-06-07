"""``UrScriptProfile`` — URScript motion-parameter bundle, frozen, built via factory.

``linear_speed_ms`` / ``linear_accel_mss`` → the ``v`` / ``a`` of ``movel``;
``joint_speed_rads`` / ``joint_accel_radss`` → the ``v`` / ``a`` of ``movej``;
``blend_radius_m`` → the ``r`` blend radius of ``movel`` (0 ⇒ exact stop).
"""

from __future__ import annotations

from dataclasses import dataclass

#: UR e-Series defaults (conservative, safe for a cold cell).
_DEFAULT_LINEAR_SPEED_MS = 0.25
_DEFAULT_LINEAR_ACCEL_MSS = 1.2
_DEFAULT_JOINT_SPEED_RADS = 1.05
_DEFAULT_JOINT_ACCEL_RADSS = 1.4
_DEFAULT_BLEND_RADIUS_M = 0.0


@dataclass(frozen=True)
class UrScriptProfile:
    """Motion parameters for URScript emission. Build via ``UrScriptProfile.build``."""

    linear_speed_ms: float = _DEFAULT_LINEAR_SPEED_MS
    linear_accel_mss: float = _DEFAULT_LINEAR_ACCEL_MSS
    joint_speed_rads: float = _DEFAULT_JOINT_SPEED_RADS
    joint_accel_radss: float = _DEFAULT_JOINT_ACCEL_RADSS
    blend_radius_m: float = _DEFAULT_BLEND_RADIUS_M

    @classmethod
    def build(
        cls,
        *,
        linear_speed_ms: float = _DEFAULT_LINEAR_SPEED_MS,
        linear_accel_mss: float = _DEFAULT_LINEAR_ACCEL_MSS,
        joint_speed_rads: float = _DEFAULT_JOINT_SPEED_RADS,
        joint_accel_radss: float = _DEFAULT_JOINT_ACCEL_RADSS,
        blend_radius_m: float = _DEFAULT_BLEND_RADIUS_M,
    ) -> UrScriptProfile:
        """Validate and construct. Raises ``ValueError`` on out-of-range inputs.

        Speeds and accelerations must be strictly positive; the blend radius must
        be non-negative (0 means exact stop at the waypoint).
        """
        if linear_speed_ms <= 0.0:
            raise ValueError(f"linear_speed_ms must be > 0; got {linear_speed_ms}")
        if linear_accel_mss <= 0.0:
            raise ValueError(f"linear_accel_mss must be > 0; got {linear_accel_mss}")
        if joint_speed_rads <= 0.0:
            raise ValueError(f"joint_speed_rads must be > 0; got {joint_speed_rads}")
        if joint_accel_radss <= 0.0:
            raise ValueError(f"joint_accel_radss must be > 0; got {joint_accel_radss}")
        if blend_radius_m < 0.0:
            raise ValueError(f"blend_radius_m must be >= 0; got {blend_radius_m}")
        return cls(
            linear_speed_ms=linear_speed_ms,
            linear_accel_mss=linear_accel_mss,
            joint_speed_rads=joint_speed_rads,
            joint_accel_radss=joint_accel_radss,
            blend_radius_m=blend_radius_m,
        )
