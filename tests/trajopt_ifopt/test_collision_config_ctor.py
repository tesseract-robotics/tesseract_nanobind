"""#251: `TrajOptCollisionConfig`'s trailing defaulted parameters are reachable from Python."""

import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionCheckProgramType,
    CollisionEvaluatorType,
    ContactRequest,
    ContactTestType,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

SEGMENT_LENGTH = 0.25  # m; any value other than the C++ default of 0.005


def test_defaults_match_the_two_argument_form():
    assert TrajOptCollisionConfig(0.1, 2.0) == TrajOptCollisionConfig(
        0.1,
        2.0,
        ContactRequest(),
        TrajOptCollisionConfig(0.1, 2.0).collision_check_config.type,
        TrajOptCollisionConfig(0.1, 2.0).collision_check_config.longest_valid_segment_length,
        TrajOptCollisionConfig(0.1, 2.0).collision_check_config.check_program_mode,
    )


def test_every_trailing_parameter_reaches_the_check_config():
    config = TrajOptCollisionConfig(
        0.1,
        2.0,
        request=ContactRequest(ContactTestType.FIRST),
        type=CollisionEvaluatorType.LVS_CONTINUOUS,
        longest_valid_segment_length=SEGMENT_LENGTH,
        check_program_mode=CollisionCheckProgramType.START_ONLY,
    )
    check = config.collision_check_config
    assert check.contact_request.type == ContactTestType.FIRST
    assert check.type == CollisionEvaluatorType.LVS_CONTINUOUS
    assert check.longest_valid_segment_length == pytest.approx(SEGMENT_LENGTH)
    assert check.check_program_mode == CollisionCheckProgramType.START_ONLY
