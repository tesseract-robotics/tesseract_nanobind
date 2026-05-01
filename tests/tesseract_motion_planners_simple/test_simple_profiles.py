"""Tests for tesseract_motion_planners_simple profile bindings.

Covers the 2 profile bases and 7 concrete move profiles, plus the
ProfileDictionary registration helpers.
"""

import math

import pytest

from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_simple import (
    SimplePlannerCompositeProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerMoveProfile,
)


# ---- Fixed-size profiles ---------------------------------------------------

FIXED_SIZE_CLASSES = [
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", FIXED_SIZE_CLASSES)
class TestFixedSizeProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: freespace_steps=10, linear_steps=10
        assert p.freespace_steps == 10
        assert p.linear_steps == 10

    def test_custom_construction(self, cls):
        p = cls(freespace_steps=25, linear_steps=7)
        assert p.freespace_steps == 25
        assert p.linear_steps == 7

    def test_member_assignment(self, cls):
        p = cls()
        p.freespace_steps = 42
        p.linear_steps = 3
        assert p.freespace_steps == 42
        assert p.linear_steps == 3

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- LVS profiles ----------------------------------------------------------

LVS_CLASSES = [
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", LVS_CLASSES)
class TestLVSProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: 5°, 0.1m, 5°, min_steps=1, max_steps=INT_MAX
        expected_rad = 5.0 * math.pi / 180.0
        assert p.state_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.rotation_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.min_steps == 1
        # max_steps defaults to INT_MAX — just make sure it's big
        assert p.max_steps > 1000

    def test_custom_construction(self, cls):
        p = cls(
            state_longest_valid_segment_length=0.05,
            translation_longest_valid_segment_length=0.2,
            rotation_longest_valid_segment_length=0.1,
            min_steps=3,
            max_steps=99,
        )
        assert p.state_longest_valid_segment_length == pytest.approx(0.05)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.2)
        assert p.rotation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.min_steps == 3
        assert p.max_steps == 99

    def test_member_assignment(self, cls):
        p = cls()
        p.min_steps = 5
        p.max_steps = 50
        p.translation_longest_valid_segment_length = 0.25
        assert p.min_steps == 5
        assert p.max_steps == 50
        assert p.translation_longest_valid_segment_length == pytest.approx(0.25)

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- Composite base --------------------------------------------------------

class TestSimplePlannerCompositeProfile:
    def test_default_construction(self):
        p = SimplePlannerCompositeProfile()
        assert p is not None


# ---- ProfileDictionary helpers ---------------------------------------------

class TestProfileDictionaryHelpers:
    def test_add_move_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerFixedSizeMoveProfile(5, 8)
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True

    def test_add_composite_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerCompositeProfile()
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True
