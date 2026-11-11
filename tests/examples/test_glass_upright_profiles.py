"""The UPRIGHT profile matches the C++ glass_upright_example, value for value."""

import numpy as np

from tesseract_robotics.motion_planners_trajopt import TrajOptDefaultMoveProfile
from tesseract_robotics.planning.profiles import create_trajopt_upright_profiles

# C++ glass_upright_example.cpp: cartesian_constraint_config.coeff = Constant(6, 1) with
# coeff(0..2) = 0: position free, orientation held with unit weight.
CPP_CARTESIAN_COEFF = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]


def test_upright_move_profile_matches_cpp_example():
    entries = create_trajopt_upright_profiles().getAllProfileEntries()
    # namespace -> profile type key -> profile name -> profile
    moves = [
        profile
        for by_type in entries.values()
        for by_name in by_type.values()
        for name, profile in by_name.items()
        if name == "UPRIGHT" and isinstance(profile, TrajOptDefaultMoveProfile)
    ]
    assert moves, "no UPRIGHT TrajOptDefaultMoveProfile registered"
    for move in moves:
        assert not move.joint_cost_config.enabled
        assert not move.cartesian_cost_config.enabled
        assert move.cartesian_constraint_config.enabled
        assert np.array_equal(move.cartesian_constraint_config.coeff, CPP_CARTESIAN_COEFF)
