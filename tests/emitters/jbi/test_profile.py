"""JbiProfile and the JBI tool binding: validating factories."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import InvalidFrameBindingError, InvalidProfileError
from tesseract_robotics.emitters.jbi import JbiProfile, jbi_frames


def test_defaults_sane() -> None:
    p = JbiProfile.build()
    assert p.linear_speed_mms == 100.0 and 0.0 < p.joint_speed_percent <= 100.0


@pytest.mark.parametrize(
    "kwargs",
    [{"joint_speed_percent": 150.0}, {"joint_speed_percent": 0.0}, {"linear_speed_mms": -1.0}],
)
def test_rejects_out_of_range(kwargs: dict[str, float]) -> None:
    with pytest.raises(InvalidProfileError):
        JbiProfile.build(**kwargs)


def test_is_frozen() -> None:
    with pytest.raises(AttributeError):
        JbiProfile.build().linear_speed_mms = 1.0  # type: ignore[misc]


@pytest.mark.parametrize("tool", [-1, 64])
def test_tool_numbers_validated(tool: int) -> None:
    with pytest.raises(InvalidFrameBindingError):
        jbi_frames(tools={"tool0": tool})
