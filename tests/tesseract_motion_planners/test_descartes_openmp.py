"""Exercise real parallel Descartes solves in wheel consumers (#136)."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_descartes import (
    DescartesDefaultMoveProfileD,
    DescartesLadderGraphSolverProfileD,
    DescartesMotionPlannerD,
)
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

from . import test_descartes_planner as fixture_source
from .test_descartes_planner import (
    DESCARTES_DEFAULT_NAMESPACE,
    _fixed_pose_program,
)

abb_irb2400_environment = fixture_source.abb_irb2400_environment


@pytest.mark.parametrize("threads", [2, 4])
def test_parallel_descartes_solve(abb_irb2400_environment, threads):
    env, manip_info, joint_names = abb_irb2400_environment
    program = generateInterpolatedProgram(_fixed_pose_program(manip_info), env)
    solver = DescartesLadderGraphSolverProfileD()
    solver.num_threads = threads
    profiles = ProfileDictionary()
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", DescartesDefaultMoveProfileD())
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, program.getProfile(), solver)
    request = PlannerRequest()
    request.instructions = program
    request.env = env
    request.profiles = profiles
    response = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE).solve(request)
    assert response.successful, response.message
    assert len(response.results) == len(program)
    for instruction in response.results:
        waypoint = instruction.asMoveInstruction().getWaypoint().asStateWaypoint()
        assert len(waypoint.getPosition()) == len(joint_names)
        assert np.isfinite(waypoint.getPosition()).all()
