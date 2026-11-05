"""The car seat port carries the C++ example's TrajOptIfopt profiles.

Reference: tesseract_planning 0.35.0, examples/src/car_seat_example.cpp, `if (ifopt_)` branch.
"""

from importlib import import_module

from tesseract_robotics.planning.profiles import TRAJOPT_IFOPT_DEFAULT_NAMESPACE
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
    TrajOptIfoptDefaultCompositeProfile,
    TrajOptIfoptDefaultMoveProfile,
    TrajOptIfoptOSQPSolverProfile,
)
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

# `tesseract_robotics.examples` re-exports each example's entry point under the module's
# name, so import the module itself.
example = import_module("tesseract_robotics.examples.car_seat_example")

# C++ registers every profile under "FREESPACE", the name of both its composite and its
# moves. The port's MotionProgram composite is "DEFAULT", so it registers under both.
PROFILE_NAMES = {"DEFAULT", "FREESPACE"}

# C++: collision_constraint_config = TrajOptCollisionConfig(0.0, 10): margin in m, coeff.
CPP_CONSTRAINT_MARGIN = 0.0
CPP_CONSTRAINT_COEFF = 10.0
CPP_CONSTRAINT_MARGIN_BUFFER = 0.005  # m
# C++: collision_cost_config = TrajOptCollisionConfig(0.005, 50).
CPP_COST_MARGIN = 0.005
CPP_COST_COEFF = 50.0
CPP_COST_MARGIN_BUFFER = 0.01  # m
# C++: collision_check_config.longest_valid_segment_length = 0.1 for both, in joint-space units.
CPP_LONGEST_VALID_SEGMENT_LENGTH = 0.1
# C++: opt_params.max_iterations = 200, min_approx_improve = min_trust_box_size = 1e-3.
CPP_MAX_ITERATIONS = 200
CPP_MIN_APPROX_IMPROVE = 1e-3
CPP_MIN_TRUST_BOX_SIZE = 1e-3


def _lvs_continuous(margin, coeff, margin_buffer):
    config = TrajOptCollisionConfig(margin, coeff)
    config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS
    config.collision_check_config.longest_valid_segment_length = CPP_LONGEST_VALID_SEGMENT_LENGTH
    config.collision_margin_buffer = margin_buffer
    return config


def test_ifopt_profiles_match_upstream():
    entries = example.create_car_seat_ifopt_profiles().getAllProfileEntries()
    assert set(entries) == {TRAJOPT_IFOPT_DEFAULT_NAMESPACE}

    found = {}
    for by_name in entries[TRAJOPT_IFOPT_DEFAULT_NAMESPACE].values():
        assert set(by_name) == PROFILE_NAMES
        for profile in by_name.values():
            found.setdefault(type(profile), []).append(profile)
    assert set(found) == {
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptDefaultMoveProfile,
        TrajOptIfoptOSQPSolverProfile,
    }

    for composite in found[TrajOptIfoptDefaultCompositeProfile]:
        assert composite.collision_constraint_config == _lvs_continuous(
            CPP_CONSTRAINT_MARGIN, CPP_CONSTRAINT_COEFF, CPP_CONSTRAINT_MARGIN_BUFFER
        )
        assert composite.collision_cost_config == _lvs_continuous(
            CPP_COST_MARGIN, CPP_COST_COEFF, CPP_COST_MARGIN_BUFFER
        )

    for move in found[TrajOptIfoptDefaultMoveProfile]:
        assert not move.cartesian_cost_config.enabled
        assert move.cartesian_constraint_config.enabled
        assert not move.joint_cost_config.enabled
        assert move.joint_constraint_config.enabled

    expected_solver = TrajOptIfoptOSQPSolverProfile()
    expected_solver.opt_params.max_iterations = CPP_MAX_ITERATIONS
    expected_solver.opt_params.min_approx_improve = CPP_MIN_APPROX_IMPROVE
    expected_solver.opt_params.min_trust_box_size = CPP_MIN_TRUST_BOX_SIZE
    for solver in found[TrajOptIfoptOSQPSolverProfile]:
        # Equality covers opt_params and every OSQP setting.
        assert solver == expected_solver
