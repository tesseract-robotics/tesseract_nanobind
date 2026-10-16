"""#329: the time-parameterization profiles are bound under their C++ names, with no alias."""

import pytest

from tesseract_robotics import tesseract_time_parameterization as ttp
from tesseract_robotics.tesseract_command_language import Profile

RENAMES = {
    "TOTGCompositeProfile": "TimeOptimalTrajectoryGenerationCompositeProfile",
    "ISPCompositeProfile": "IterativeSplineParameterizationCompositeProfile",
    "ISPMoveProfile": "IterativeSplineParameterizationMoveProfile",
    "ConstantTCPSpeedCompositeProfile": "ConstantTCPSpeedParameterizationCompositeProfile",
}


@pytest.mark.parametrize("new", sorted(RENAMES.values()))
def test_native_name_is_a_profile(new):
    cls = getattr(ttp, new)
    assert issubclass(cls, Profile)
    assert cls.__name__ == new
    assert cls.createKey() == cls().getKey()


@pytest.mark.parametrize("old", sorted(RENAMES))
def test_old_name_removed(old):
    assert not hasattr(ttp, old)
    assert not hasattr(ttp._tesseract_time_parameterization, old)


def test_package_has_no_all():
    assert not hasattr(ttp, "__all__")
