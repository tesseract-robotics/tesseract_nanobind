"""#334, #335: the planner modules export only the C++ names; profiles go in with addProfile."""

import importlib

import pytest

from tesseract_robotics.tesseract_command_language import ProfileDictionary

REMOVED = {
    "tesseract_motion_planners_ompl": [
        "OMPLMoveProfile_as_ProfileConstPtr",
        "OMPLPlanProfile_as_ProfileConstPtr",
        "ProfileDictionary_addOMPLMoveProfile",
        "ProfileDictionary_addOMPLProfile",
        "OMPLPlanProfile",
        "OMPLRealVectorPlanProfile",
    ],
    "tesseract_motion_planners_trajopt": [
        "ProfileDictionary_addTrajOptMoveProfile",
        "ProfileDictionary_addTrajOptPlanProfile",
        "ProfileDictionary_addTrajOptCompositeProfile",
        "ProfileDictionary_addTrajOptSolverProfile",
        "TrajOptPlanProfile",
        "TrajOptDefaultPlanProfile",
        "TrajOptCollisionConfig",
        "CollisionEvaluatorType",
    ],
    "tesseract_motion_planners_trajopt_ifopt": [
        "ProfileDictionary_addTrajOptIfoptMoveProfile",
        "ProfileDictionary_addTrajOptIfoptPlanProfile",
        "ProfileDictionary_addTrajOptIfoptCompositeProfile",
        "ProfileDictionary_addTrajOptIfoptSolverProfile",
        "TrajOptIfoptPlanProfile",
        "TrajOptIfoptDefaultPlanProfile",
    ],
    "tesseract_motion_planners_descartes": [
        "cast_DescartesSolverProfileD",
        "cast_DescartesMoveProfileD",
        "cast_DescartesPlanProfileD",
        "DescartesPlanProfileD",
        "DescartesDefaultPlanProfileD",
    ],
}

PACKAGES = [
    "tesseract_motion_planners",
    "tesseract_motion_planners_ompl",
    "tesseract_motion_planners_trajopt",
    "tesseract_motion_planners_trajopt_ifopt",
    "tesseract_motion_planners_simple",
    "tesseract_motion_planners_descartes",
]


def _package(name):
    return importlib.import_module(f"tesseract_robotics.{name}")


@pytest.mark.parametrize(("module", "name"), [(m, n) for m, ns in REMOVED.items() for n in ns])
def test_removed(module, name):
    assert not hasattr(_package(module), name)


@pytest.mark.parametrize("module", PACKAGES)
def test_package_has_no_all(module):
    assert not hasattr(_package(module), "__all__")


def test_star_import_binds_the_move_profiles():
    namespace: dict[str, object] = {}
    exec("from tesseract_robotics.tesseract_motion_planners_ompl import *", namespace)
    assert "OMPLRealVectorMoveProfile" in namespace
    assert "RNG_setSeed" in namespace


def test_trajopt_ifopt_keeps_its_config_reexports():
    """#335: these configure the TrajOpt-Ifopt profile fields from one import."""
    pkg = _package("tesseract_motion_planners_trajopt_ifopt")
    from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
    from tesseract_robotics.trajopt_ifopt import TrajOptCollisionConfig
    from tesseract_robotics.trajopt_sqp import SQPParameters

    assert pkg.TrajOptCollisionConfig is TrajOptCollisionConfig
    assert pkg.CollisionEvaluatorType is CollisionEvaluatorType
    assert pkg.SQPParameters is SQPParameters


def _profiles():
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultMoveProfileD,
        DescartesLadderGraphSolverProfileD,
        DescartesMoveProfileD,
    )
    from tesseract_robotics.tesseract_motion_planners_ompl import OMPLRealVectorMoveProfile
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultMoveProfile,
        TrajOptOSQPSolverProfile,
    )
    from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
        TrajOptIfoptDefaultCompositeProfile,
        TrajOptIfoptOSQPSolverProfile,
    )

    class PyMoveProfile(DescartesMoveProfileD):
        pass

    return [
        OMPLRealVectorMoveProfile(),
        TrajOptDefaultMoveProfile(),
        TrajOptDefaultCompositeProfile(),
        TrajOptOSQPSolverProfile(),
        TrajOptIfoptDefaultCompositeProfile(),
        TrajOptIfoptOSQPSolverProfile(),
        DescartesDefaultMoveProfileD(),
        DescartesLadderGraphSolverProfileD(),
        PyMoveProfile(),
    ]


@pytest.mark.parametrize("index", range(9))
def test_add_profile_takes_every_planner_profile(index):
    """#334: `ProfileDictionary.addProfile` is the one way in, for every planner module."""
    profile = _profiles()[index]
    profiles = ProfileDictionary()
    profiles.addProfile("ns", "NAME", profile)
    assert profiles.hasProfile(profile.getKey(), "ns", "NAME")
