"""The freespace hybrid port carries the C++ example's TrajOptIfopt composite profile.

Reference: tesseract_planning 0.35.0, examples/src/freespace_hybrid_example.cpp, `if (ifopt_)` branch.
"""

from importlib import import_module

import numpy as np

from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.motion_planners_trajopt_ifopt import (
    TrajOptIfoptDefaultCompositeProfile,
)
from tesseract_robotics.planning.profiles import TRAJOPT_IFOPT_DEFAULT_NAMESPACE
from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig

# `tesseract_robotics.examples` re-exports each example's entry point under the module's
# name, so import the module itself.
example = import_module("tesseract_robotics.examples.freespace_hybrid_example")

# C++: trajopt_common::TrajOptCollisionConfig(0.025, 20) for both cost and constraint: margin in m, coeff.
CPP_COLLISION_MARGIN = 0.025
CPP_COLLISION_COEFF = 20.0
# C++: velocity_coeff = acceleration_coeff = jerk_coeff = Eigen::VectorXd::Ones(1).
CPP_SMOOTHING_COEFF = [1.0]


def test_ifopt_profiles_match_upstream():
    profiles = ProfileDictionary()
    example.add_ifopt_profiles(profiles)

    entries = profiles.getAllProfileEntries()
    assert set(entries) == {TRAJOPT_IFOPT_DEFAULT_NAMESPACE}
    (by_name,) = entries[TRAJOPT_IFOPT_DEFAULT_NAMESPACE].values()
    assert set(by_name) == {"FREESPACE"}
    composite = by_name["FREESPACE"]
    assert isinstance(composite, TrajOptIfoptDefaultCompositeProfile)

    expected_collision = TrajOptCollisionConfig(CPP_COLLISION_MARGIN, CPP_COLLISION_COEFF)
    assert composite.collision_cost_config == expected_collision
    assert composite.collision_constraint_config == expected_collision
    assert np.array_equal(composite.velocity_coeff, CPP_SMOOTHING_COEFF)
    assert np.array_equal(composite.acceleration_coeff, CPP_SMOOTHING_COEFF)
    assert np.array_equal(composite.jerk_coeff, CPP_SMOOTHING_COEFF)
    # The smooth_* switches are not set upstream: they keep the profile's defaults.
    default = TrajOptIfoptDefaultCompositeProfile()
    assert composite.smooth_velocities == default.smooth_velocities
    assert composite.smooth_accelerations == default.smooth_accelerations
    assert composite.smooth_jerks == default.smooth_jerks
