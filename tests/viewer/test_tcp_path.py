"""TCP path per move type: MoveL is a straight line, MoveJ the FK of its joint interpolation."""

from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import MoveInstructionType as Move
from tesseract_robotics.common import (
    GeneralResourceLocator,
    Isometry3d,
    LinkId,
    ManipulatorInfo,
)
from tesseract_robotics.environment import Environment
from tesseract_robotics.viewer import Trajectory
from tesseract_robotics.viewer.errors import (
    MovingWorkingFrameError,
    NamedTcpOffsetError,
    UnknownJointError,
    UnsupportedMoveTypeError,
)
from tesseract_robotics.viewer.tcp_path import (
    MAX_JOINT_STEP_RAD,
    MAX_PRISMATIC_STEP_M,
    sample_count,
    tcp_path,
)

JOINT_NAMES = [f"joint_{i + 1}" for i in range(6)]
START = np.zeros(6)
# All six joints sweep 1 rad and more: far enough that the arc leaves any chord.
GOAL = np.array([1.2, 0.6, -0.5, 1.5, -1.0, 2.0])
# FK reproduces a waypoint to float precision [m].
EXACT_ATOL_M = 1e-9
# Drawn path against FK sampled 16x finer [m]: 2x the worst of 300 random full-range moves
# on these two robots, 0.102 mm.
DRAWN_PATH_ATOL_M = 2e-4
# A MoveJ this large leaves its chord by far more than the tolerance above [m].
CHORD_DEPARTS_M = 0.05
FINER = 16
PUZZLE_URDF = "package://tesseract/support/urdf/puzzle_piece_workcell.urdf"
PUZZLE_SRDF = "package://tesseract/support/urdf/puzzle_piece_workcell.srdf"


def _load(name):
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    urdf = locator.locateResource(f"package://tesseract/support/urdf/{name}.urdf")
    srdf = locator.locateResource(f"package://tesseract/support/urdf/{name}.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


@pytest.fixture(scope="module")
def env():
    return _load("abb_irb2400")


@pytest.fixture
def manip():
    return ManipulatorInfo("manipulator", "base_link", "tool0")


def _traj(names, rows, moves=None):
    return Trajectory.build(names, rows, np.arange(len(rows), dtype=float), moves)


def _tcp(env, names, q, frame="tool0"):
    return env.getKinematicGroup("manipulator").calcFwdKin(q)[frame].translation


def _distance_to_polyline(points, segments):
    a, b = segments[:, 0], segments[:, 1]
    ab = b - a
    t = np.clip(((points[:, None] - a) * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
    return np.linalg.norm(points[:, None] - (a + t[..., None] * ab), axis=-1).min(axis=1)


def _distance_to_chord(points, a, b):
    return _distance_to_polyline(points, np.array([[a, b]]))


def test_movel_is_one_straight_segment(env, manip):
    path = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL], (Move.FREESPACE, Move.LINEAR)), manip)
    assert path.segments.shape == (1, 2, 3)
    np.testing.assert_allclose(path.segments[0, 0], path.positions[0], atol=EXACT_ATOL_M)
    np.testing.assert_allclose(path.segments[0, 1], path.positions[1], atol=EXACT_ATOL_M)


def test_movej_follows_fk_of_the_joint_interpolation(env, manip):
    path = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip)
    expected_count = int(np.ceil(np.abs(GOAL).max() / MAX_JOINT_STEP_RAD))
    assert len(path.segments) == expected_count

    fine = np.linspace(0.0, 1.0, FINER * expected_count + 1)
    truth = np.stack([_tcp(env, JOINT_NAMES, START + s * GOAL) for s in fine])
    assert _distance_to_polyline(truth, path.segments).max() < DRAWN_PATH_ATOL_M
    assert _distance_to_chord(truth, path.positions[0], path.positions[1]).max() > CHORD_DEPARTS_M


def test_path_is_continuous_through_the_waypoints(env, manip):
    rows = [START, GOAL / 2, GOAL]
    path = tcp_path(
        env, _traj(JOINT_NAMES, rows, (Move.FREESPACE, Move.LINEAR, Move.FREESPACE)), manip
    )
    np.testing.assert_allclose(path.segments[0, 0], path.positions[0], atol=EXACT_ATOL_M)
    np.testing.assert_allclose(path.segments[-1, 1], path.positions[-1], atol=EXACT_ATOL_M)
    np.testing.assert_allclose(path.segments[:-1, 1], path.segments[1:, 0], atol=EXACT_ATOL_M)


def test_move_type_of_the_arrival_waypoint_decides(env, manip):
    rows = [START, GOAL]
    joint_first = tcp_path(env, _traj(JOINT_NAMES, rows, (Move.LINEAR, Move.FREESPACE)), manip)
    linear_last = tcp_path(env, _traj(JOINT_NAMES, rows, (Move.FREESPACE, Move.LINEAR)), manip)
    assert len(joint_first.segments) > 1
    assert len(linear_last.segments) == 1


def test_single_waypoint_has_no_segments(env, manip):
    path = tcp_path(env, _traj(JOINT_NAMES, [GOAL]), manip)
    assert path.segments.shape == (0, 2, 3)
    assert path.positions.shape == (1, 3)


def test_waypoint_poses_are_exact_fk(env, manip):
    path = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip)
    np.testing.assert_allclose(path.positions[1], _tcp(env, JOINT_NAMES, GOAL), atol=EXACT_ATOL_M)
    assert path.wxyzs.shape == (2, 4)


def test_tcp_offset_moves_the_whole_path(env, manip):
    bare = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip)
    offset = Isometry3d.Identity()
    offset.translate(np.array([0.0, 0.0, 0.2]))
    manip.tcp_offset = offset
    moved = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip)

    fk = env.getKinematicGroup("manipulator").calcFwdKin(GOAL)["tool0"]
    np.testing.assert_allclose(moved.positions[1], (fk * offset).translation, atol=EXACT_ATOL_M)
    np.testing.assert_allclose(moved.segments[-1, 1], moved.positions[1], atol=EXACT_ATOL_M)
    assert np.linalg.norm(moved.positions[1] - bare.positions[1]) > 0.1


def test_movel_is_straight_for_the_offset_tcp_not_the_flange(env, manip):
    offset = Isometry3d.Identity()
    offset.translate(np.array([0.0, 0.0, 0.2]))
    manip.tcp_offset = offset
    path = tcp_path(env, _traj(JOINT_NAMES, [START, GOAL], (Move.FREESPACE, Move.LINEAR)), manip)
    assert path.segments.shape == (1, 2, 3)
    np.testing.assert_allclose(path.segments[0, 1], path.positions[1], atol=EXACT_ATOL_M)


def test_named_tcp_offset_raises(env, manip):
    manip.tcp_offset = "some_link"
    with pytest.raises(NamedTcpOffsetError, match="some_link"):
        tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip)


def test_circular_raises(env, manip):
    with pytest.raises(UnsupportedMoveTypeError, match="move 1"):
        tcp_path(env, _traj(JOINT_NAMES, [START, GOAL], (Move.FREESPACE, Move.CIRCULAR)), manip)


def test_movel_in_a_moving_working_frame_raises(env):
    manip = ManipulatorInfo("manipulator", "link_3", "tool0")
    with pytest.raises(MovingWorkingFrameError, match="link_3"):
        tcp_path(env, _traj(JOINT_NAMES, [START, GOAL], (Move.FREESPACE, Move.LINEAR)), manip)


def test_movej_in_a_moving_working_frame_is_still_a_world_path(env):
    manip = ManipulatorInfo("manipulator", "link_3", "tool0")
    assert len(tcp_path(env, _traj(JOINT_NAMES, [START, GOAL]), manip).segments) > 1


def test_group_joint_missing_from_the_trajectory_raises(env, manip):
    with pytest.raises(UnknownJointError):
        tcp_path(env, _traj(JOINT_NAMES[:5], [START[:5]]), manip)


def test_rail_and_arm_stay_on_the_path():
    env = _load("online_planning_example")
    kin = env.getKinematicGroup("manipulator")
    names = [jid.name() for jid in kin.getJointIds()]
    goal = np.zeros(len(names))
    goal[:2] = [2.0, 1.0]  # the gantry's prismatic axes [m]
    goal[2:] = 0.5
    manip = ManipulatorInfo("manipulator", "base_link", "tool0")
    path = tcp_path(env, _traj(names, [np.zeros(len(names)), goal]), manip)

    fine = np.linspace(0.0, 1.0, FINER * len(path.segments) + 1)
    truth = np.stack([kin.calcFwdKin(s * goal)[LinkId("tool0")].translation for s in fine])
    assert _distance_to_polyline(truth, path.segments).max() < DRAWN_PATH_ATOL_M


def test_group_without_an_ik_solver_has_a_path():
    env = Environment()
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    urdf = locator.locateResource(PUZZLE_URDF)
    srdf = locator.locateResource(PUZZLE_SRDF)
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    names = [jid.name() for jid in env.getJointGroup("manipulator_aux").getJointIds()]
    q = np.zeros(len(names))
    manip = ManipulatorInfo("manipulator_aux", "world", "grinder_frame")
    assert tcp_path(env, _traj(names, [q, q + 0.1]), manip).segments.shape[0] > 1


REVOLUTE = np.array([True, True, False])


@pytest.mark.parametrize(
    ("delta", "count"),
    [
        pytest.param([0.0, 0.0, 0.0], 1, id="no-motion"),
        pytest.param([0.0, 0.0, 5.0], 1, id="prismatic-alone-is-straight"),
        pytest.param([MAX_JOINT_STEP_RAD / 2, 0.0, 0.0], 1, id="under-one-step"),
        pytest.param([10.5 * MAX_JOINT_STEP_RAD, -MAX_JOINT_STEP_RAD, 0.0], 11, id="largest-joint"),
        pytest.param([-10.5 * MAX_JOINT_STEP_RAD, 0.0, 0.0], 11, id="sign-ignored"),
        pytest.param(
            [MAX_JOINT_STEP_RAD, 0.0, 20.5 * MAX_PRISMATIC_STEP_M], 21, id="rail-rotating"
        ),
    ],
)
def test_sample_count(delta, count):
    assert sample_count(np.array(delta), REVOLUTE) == count
