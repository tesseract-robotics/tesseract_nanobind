"""The high-level examples plan what the low-level examples (ports of the C++ ones) plan.

Each pair runs the same scene and program. Parity is judged on the whole planned path, in the
space the program is stated in: joint space for a joint goal, the tool pose along the path for a
Cartesian program (a redundant arm reaches one pose with several joint vectors, so joint
vectors may differ while the tool does not). A sampling planner (OMPL) cannot repeat a path, so
there only the ends are compared, plus validity of both paths.

Tolerances are the largest differences measured between the two ports on the development
machine, rounded up to one significant digit; a regression in either port moves them by orders
of magnitude.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import tesseract_robotics.examples as examples
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning.composer import PlanningResult

from ..planning_validity import JointGoal, assert_trajectory_valid

# Ends of an OMPL path are fixed by the program, not sampled [rad]: measured 6e-10.
OMPL_END_TOL_RAD = 1e-6
# Ends of a TrajOpt phase, joint-space goal or Cartesian goal: the solver's constraint
# tolerance, 1e-4 in position [m] and in orientation [rad].
PHASE_END_POSITION_TOL_M = 1e-4
PHASE_END_ORIENTATION_TOL_RAD = 1e-4
# Joint goal of the freespace ports (rad), the C++ freespace examples' end state.
FREESPACE_GOAL = JointGoal.build([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])


@dataclass(frozen=True)
class PathTolerance:
    """Largest tool-pose difference between the ports along a path.

    Attributes:
        position_m: Position difference [m].
        orientation_rad: Orientation difference [rad].
    """

    position_m: float
    orientation_rad: float


def _joints(result: PlanningResult) -> np.ndarray:
    return np.array([point.positions for point in result.trajectory])


def _tool_poses(robot: Robot, group: str, tip: str, joints: np.ndarray):
    return [robot.fk(group, q, tip) for q in joints]


def _pose_gap(a, b) -> tuple[float, float]:
    position = float(np.linalg.norm(np.asarray(a.translation) - np.asarray(b.translation)))
    relative = np.asarray(a.linear).T @ np.asarray(b.linear)
    return position, float(Rotation.from_matrix(relative).magnitude())


def assert_same_waypoints(high: PlanningResult, low: PlanningResult) -> None:
    """Both ports plan the same number of waypoints."""
    assert len(high) == len(low), f"{len(high)} waypoints vs {len(low)}"


def assert_same_tool_path(
    robot: Robot,
    group: str,
    tip: str,
    high: PlanningResult,
    low: PlanningResult,
    tolerance: PathTolerance,
) -> None:
    """Tool pose agrees at every waypoint within `tolerance`, and at both ends within solver tolerance."""
    assert_same_waypoints(high, low)
    gaps = [
        _pose_gap(a, b)
        for a, b in zip(
            _tool_poses(robot, group, tip, _joints(high)),
            _tool_poses(robot, group, tip, _joints(low)),
            strict=True,
        )
    ]
    for end in (0, -1):
        assert gaps[end][0] <= PHASE_END_POSITION_TOL_M, f"waypoint {end}: {gaps[end][0]:.2e} m"
        assert gaps[end][1] <= PHASE_END_ORIENTATION_TOL_RAD, (
            f"waypoint {end}: {gaps[end][1]:.2e} rad"
        )
    assert max(g[0] for g in gaps) <= tolerance.position_m
    assert max(g[1] for g in gaps) <= tolerance.orientation_rad


@pytest.mark.planning
def test_freespace_ompl_parity():
    """OMPL samples its path: both must be valid and meet the same start and goal."""
    high = examples.freespace_ompl_example()
    low = examples.freespace_ompl_c_api_example()
    for outcome in (high, low):
        assert_trajectory_valid(high["robot"], outcome["result"], FREESPACE_GOAL)
    np.testing.assert_allclose(
        _joints(high["result"])[0], _joints(low["result"])[0], rtol=0, atol=OMPL_END_TOL_RAD
    )
    np.testing.assert_allclose(
        _joints(high["result"])[-1], _joints(low["result"])[-1], rtol=0, atol=OMPL_END_TOL_RAD
    )


# Tool path agreement of the deterministic TrajOpt ports, measured: basic_cartesian 5e-6 m and
# 1.5e-5 rad; glass_upright 2.4e-3 m and 3e-6 rad (its path differs in the middle, not the ends).
@pytest.mark.planning
@pytest.mark.parametrize(
    ("name", "tolerance"),
    [
        ("basic_cartesian", PathTolerance(position_m=1e-5, orientation_rad=1e-4)),
        ("glass_upright", PathTolerance(position_m=3e-3, orientation_rad=1e-5)),
    ],
)
def test_trajopt_parity(name, tolerance):
    high = getattr(examples, f"{name}_example")()
    low = getattr(examples, f"{name}_c_api_example")()
    assert_trajectory_valid(high["robot"], high["result"])
    assert_same_tool_path(
        high["robot"], "manipulator", "tool0", high["result"], low["result"], tolerance
    )


@pytest.mark.planning
def test_car_seat_parity():
    """Same scene and program: identical to the last bit (measured difference 0)."""
    high = examples.car_seat_example()
    low = examples.car_seat_c_api_example()
    tolerance = PathTolerance(position_m=1e-9, orientation_rad=1e-9)
    for phase in ("pick_result", "place_result"):
        assert_same_tool_path(
            high["robot"], "manipulator", "end_effector", high[phase], low[phase], tolerance
        )


@pytest.mark.planning
def test_pick_and_place_parity():
    """Same scene (the C++ constructor defaults) and program; TrajOpt interpolates differently.

    Measured tool-path differences: pick 2.3 cm and 0.11 rad, place 1.3 cm and 0.061 rad, both
    in the middle of the phase; the ends agree to the solver tolerance.
    """
    high = examples.pick_and_place_example()
    low = examples.pick_and_place_c_api_example()
    pick = PathTolerance(position_m=3e-2, orientation_rad=0.2)
    place = PathTolerance(position_m=2e-2, orientation_rad=0.1)
    for phase, tolerance in (("pick_result", pick), ("place_result", place)):
        assert_same_tool_path(
            high["robot"], "manipulator", "iiwa_tool0", high[phase], low[phase], tolerance
        )
