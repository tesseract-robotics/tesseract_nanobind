"""A low-level planner response as the `PlanningResult` that `assert_trajectory_valid` checks."""

from __future__ import annotations

import numpy as np

from tesseract_robotics.motion_planners import PlannerResponse
from tesseract_robotics.planning.composer import PlanningResult, TrajectoryPoint


def planning_result(response: PlannerResponse) -> PlanningResult:
    """Wrap the state waypoints of `response.results` as a `PlanningResult`.

    Args:
        response: A planner's response, successful or not.

    Returns:
        The result, with `successful` and `message` of the response and one point per state
        waypoint; no velocities, accelerations or times.
    """
    points = []
    for instruction in response.results if response.successful else []:
        waypoint = instruction.asMoveInstruction().getWaypoint()
        state = waypoint.asStateWaypoint()
        points.append(
            TrajectoryPoint(
                joint_names=[joint.name() for joint in state.getJointIds()],
                positions=np.array(state.getPosition()).ravel(),
            )
        )
    return PlanningResult(
        successful=response.successful, message=response.message, trajectory=points
    )
