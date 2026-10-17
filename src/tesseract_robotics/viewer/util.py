import json

import numpy as np

from tesseract_robotics.tesseract_command_language import toJointTrajectory
from tesseract_robotics.tesseract_common import Quaterniond


def tesseract_trajectory_to_list(tesseract_trajectory):
    """Convert a CompositeInstruction trajectory to joint names and rows of positions + time.

    Uses `toJointTrajectory`, so child composites (raster segments) are included and the time
    accumulates across them. State waypoints keep their time; joint waypoints (and seeded
    Cartesian waypoints) count one second each.

    Raises:
        ValueError: the program holds no joint, state or seeded Cartesian waypoint.
    """
    states = toJointTrajectory(tesseract_trajectory).states
    if not states:
        raise ValueError("tesseract_trajectory holds no joint states to animate")
    joint_names = list(states[0].joint_names)
    rows = [state.position.flatten().tolist() + [state.time] for state in states]
    return joint_names, rows


def trajectory_list_to_json(joint_names, trajectory_list, use_time=True, loop_time=20):
    trajectory_json = dict()
    trajectory_json["use_time"] = use_time
    trajectory_json["loop_time"] = loop_time
    trajectory_json["joint_names"] = joint_names
    trajectory_json["trajectory"] = trajectory_list
    return json.dumps(trajectory_json)


def joint_positions_to_trajectory_json(joint_names, joint_positions):
    # Create "infinite" animation with constant joint angles

    trajectory_json = dict()
    trajectory_json["use_time"] = True
    trajectory_json["loop_time"] = 10000

    assert joint_names and all(isinstance(s, str) for s in joint_names), (
        "Joint names must all be strings"
    )
    trajectory_json["joint_names"] = joint_names
    assert isinstance(joint_positions, np.ndarray), "Expected numpy array for joint_positions"
    assert joint_positions.dtype == np.float64, "Expected double float array for joint_positions"
    joint_positions = list(joint_positions.flatten())
    assert len(joint_positions) == len(joint_names)
    trajectory_json["trajectory"] = [joint_positions + [0.0], joint_positions + [1e6]]

    return json.dumps(trajectory_json)


def trajectory_list_to_frames(tesseract_env, manipulator_info, joint_names, trajectory_list):
    ret = []
    kin = tesseract_env.getKinematicGroup(manipulator_info.manipulator)
    for i in range(len(trajectory_list)):
        frames = kin.calcFwdKin(np.asarray(trajectory_list[i], dtype=np.float64))
        frame = frames[manipulator_info.tcp_frame]
        p = frame.translation.flatten()
        q1 = Quaterniond(frame.linear)
        q = np.array([q1.w, q1.x, q1.y, q1.z])

        ret.append([p, q])
    return ret
