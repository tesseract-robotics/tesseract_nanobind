"""Both puzzle-piece auxiliary-axes ports carry the C++ example's TrajOpt profiles.

Reference: tesseract_planning 0.35.0, examples/src/puzzle_piece_auxillary_axes_example.cpp
(the non-ifopt branch of `PuzzlePieceAuxillaryAxesExample::run`).
"""

from importlib import import_module

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
    TrajOptOSQPSolverProfile,
)

# `tesseract_robotics.examples` re-exports each example's entry point under the module's
# name, so `from ... import <module>` yields the function; import the modules themselves.
highlevel = import_module("tesseract_robotics.examples.puzzle_piece_auxillary_axes_example")
lowlevel = import_module(
    "tesseract_robotics.examples.lowlevel.puzzle_piece_auxillary_axes_c_api_example"
)

TRAJOPT_NS = "TrajOptMotionPlannerTask"

# C++: trajopt_common::TrajOptCollisionConfig(0.025, 1): margin in m, then the cost coeff.
CPP_COLLISION_MARGIN = 0.025
CPP_COLLISION_COEFF = 1.0
# C++: cartesian_constraint_config.coeff = Constant(6, 5) with coeff(3) = coeff(4) = 2, coeff(5) = 0.
CPP_CARTESIAN_COEFF = [5.0, 5.0, 5.0, 2.0, 2.0, 0.0]
# C++: trajopt_solver_profile->settings.adaptive_rho = 0, opt_params.* below.
CPP_ADAPTIVE_RHO = 0
CPP_MAX_ITER = 200
CPP_MIN_APPROX_IMPROVE = 1e-3
CPP_MIN_TRUST_BOX_SIZE = 1e-3


def _profiles_by_type(profiles):
    """Index the profiles registered under the TrajOpt namespace by their bound type."""
    found = {}
    for by_name in profiles.getAllProfileEntries()[TRAJOPT_NS].values():
        for name, profile in by_name.items():
            found[type(profile)] = (name, profile)
    return found


@pytest.mark.parametrize("port", [highlevel, lowlevel], ids=["highlevel", "lowlevel"])
def test_profiles_match_cpp_example(port):
    found = _profiles_by_type(port.create_profiles())

    move_name, move = found[TrajOptDefaultMoveProfile]
    assert move_name == "CARTESIAN"
    assert not move.joint_cost_config.enabled
    assert not move.cartesian_cost_config.enabled
    assert move.cartesian_constraint_config.enabled
    assert np.array_equal(move.cartesian_constraint_config.coeff, CPP_CARTESIAN_COEFF)

    composite_name, composite = found[TrajOptDefaultCompositeProfile]
    assert composite_name == "DEFAULT"
    assert not composite.collision_constraint_config.enabled
    cost = composite.collision_cost_config
    assert cost.enabled
    assert cost.contact_manager_config.default_margin == CPP_COLLISION_MARGIN
    assert cost.collision_coeff_data.getDefaultCollisionCoeff() == CPP_COLLISION_COEFF
    assert cost.collision_check_config.type == CollisionEvaluatorType.DISCRETE

    solver_name, solver = found[TrajOptOSQPSolverProfile]
    assert solver_name == "DEFAULT"
    assert solver.settings.adaptive_rho == CPP_ADAPTIVE_RHO
    assert solver.opt_params.max_iter == CPP_MAX_ITER
    assert solver.opt_params.min_approx_improve == CPP_MIN_APPROX_IMPROVE
    assert solver.opt_params.min_trust_box_size == CPP_MIN_TRUST_BOX_SIZE


# C++ `if (ifopt_)` branch: opt_params.max_iterations = 200; the adaptive_rho line is
# commented out there, so the OSQP settings stay at their defaults.
CPP_IFOPT_MAX_ITERATIONS = 200


def test_ifopt_profiles_match_upstream():
    """The high-level port's ifopt profiles are the C++ example's `if (ifopt_)` branch."""
    from tesseract_robotics.planning.profiles import TRAJOPT_IFOPT_DEFAULT_NAMESPACE
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptDefaultMoveProfile,
        TrajOptIfoptOSQPSolverProfile,
    )
    from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

    profiles = highlevel.create_ifopt_profiles()
    entries = profiles.getAllProfileEntries()
    assert set(entries) == {TRAJOPT_IFOPT_DEFAULT_NAMESPACE}
    found = {}
    for by_name in entries[TRAJOPT_IFOPT_DEFAULT_NAMESPACE].values():
        for name, profile in by_name.items():
            found[type(profile)] = (name, profile)

    move_name, move = found[TrajOptIfoptDefaultMoveProfile]
    assert move_name == "CARTESIAN"
    assert not move.joint_cost_config.enabled
    assert not move.cartesian_cost_config.enabled
    assert move.cartesian_constraint_config.enabled
    assert np.array_equal(move.cartesian_constraint_config.coeff, CPP_CARTESIAN_COEFF)

    composite_name, composite = found[TrajOptIfoptDefaultCompositeProfile]
    assert composite_name == "DEFAULT"
    assert not composite.collision_constraint_config.enabled
    expected_cost = TrajOptCollisionConfig(CPP_COLLISION_MARGIN, CPP_COLLISION_COEFF)
    expected_cost.collision_check_config.type = CollisionEvaluatorType.DISCRETE
    assert composite.collision_cost_config == expected_cost

    solver_name, solver = found[TrajOptIfoptOSQPSolverProfile]
    assert solver_name == "DEFAULT"
    expected_solver = TrajOptIfoptOSQPSolverProfile()
    expected_solver.opt_params.max_iterations = CPP_IFOPT_MAX_ITERATIONS
    expected_solver.opt_params.min_approx_improve = CPP_MIN_APPROX_IMPROVE
    expected_solver.opt_params.min_trust_box_size = CPP_MIN_TRUST_BOX_SIZE
    # Equality covers opt_params and every OSQP setting, adaptive_rho included.
    assert solver == expected_solver
