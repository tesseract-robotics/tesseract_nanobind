"""TesseractViewer: viser server lifecycle and scene state."""

import socket
import time
from pathlib import Path

import numpy as np
import pytest
from loguru import logger

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
)
from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import TesseractViewer, Trajectory
from tesseract_robotics.viewer.errors import (
    NoEnvironmentError,
    NoTrajectoryError,
    UnknownJointError,
    ViewerPortInUseError,
)
from tesseract_robotics.viewer.poses import link_poses
from tesseract_robotics.viewer.viewer import PLAYBACK_DT_S

# Seconds to wait for the server socket to accept a TCP connection after start.
CONNECT_TIMEOUT_S = 5.0
# Frame position vs. an independent evaluation of the same state [m]; float64 round-off.
POSE_ATOL_M = 1e-12
# viser stores batched axes as float32: ~1e-7 relative on ~1 m positions.
PLOT_ATOL_M = 1e-6
# Wall time for the 30 Hz player to tick at least once [s]; generous for loaded CI.
PLAYER_ADVANCE_TIMEOUT_S = 5.0

# One revolute joint not in the IRB2400, to make a trajectory stale.
OTHER_ROBOT_URDF = """
<robot name="other" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="arm"/>
  <joint name="other_joint" type="revolute">
    <parent link="world"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""

JOINT_NAMES = [f"joint_{i + 1}" for i in range(6)]
Q = np.array([0.3, -0.2, 0.1, 0.4, -0.5, 0.6])


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _assert_port_free(port: int) -> None:
    # Binding without SO_REUSEADDR succeeds only if the listener is gone.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", port))


@pytest.fixture(scope="module")
def env():
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


@pytest.fixture
def viewer():
    with TesseractViewer(port=_free_port()) as viewer:
        yield viewer


def test_serving_on_construction():
    port = _free_port()
    viewer = TesseractViewer(port=port)
    try:
        socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S).close()
        assert viewer.url == f"http://127.0.0.1:{port}"
    finally:
        viewer.close()


def test_close_releases_port_and_is_idempotent():
    port = _free_port()
    viewer = TesseractViewer(port=port)
    viewer.close()
    _assert_port_free(port)
    viewer.close()


def test_context_manager_closes():
    port = _free_port()
    with TesseractViewer(port=port):
        pass
    _assert_port_free(port)


def test_one_frame_per_link(viewer, env):
    viewer.update_environment(env)
    assert set(viewer._link_frames) == set(env.getLinkNames())


def test_link_without_visuals_has_frame_and_no_mesh(viewer, env):
    """Review Focus 1: `base` has no visual; it gets a frame and nothing under it."""
    assert not env.getSceneGraph().getLink("base").visual
    viewer.update_environment(env)
    assert "base" in viewer._link_frames
    assert not [h for h in viewer._visual_handles if h.name.startswith("/links/base/")]


def test_update_environment_twice_no_duplicates(viewer, env):
    """Review Focus 2: a second push replaces the scene instead of adding to it."""
    viewer.update_environment(env)
    first = len(viewer._visual_handles)
    viewer.update_environment(env)
    assert len(viewer._visual_handles) == first > 0


def test_frames_posed_from_current_state(viewer, env):
    viewer.update_environment(env)
    expected = link_poses(env, env.getActiveJointNames(), env.getCurrentJointValues())
    np.testing.assert_allclose(
        viewer._link_frames["tool0"].position, expected["tool0"][0], atol=POSE_ATOL_M
    )


def test_joint_positions_move_tool0(viewer, env):
    viewer.update_environment(env)
    viewer.update_joint_positions(JOINT_NAMES, Q)
    xyz, wxyz = link_poses(env, JOINT_NAMES, Q)["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)
    np.testing.assert_allclose(viewer._link_frames["tool0"].wxyz, wxyz, atol=POSE_ATOL_M)


def test_serve_forever_closes_on_keyboard_interrupt(monkeypatch):
    port = _free_port()
    viewer = TesseractViewer(port=port)

    def interrupt(_seconds):
        socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S).close()
        raise KeyboardInterrupt

    monkeypatch.setattr("tesseract_robotics.viewer.viewer.time.sleep", interrupt)
    viewer.serve_forever()
    _assert_port_free(port)


def _composite(waypoints):
    composite = CompositeInstruction()
    for wp in waypoints:
        move = MoveInstruction(wp, MoveInstructionType.FREESPACE)
        composite.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(move))
    return composite


def _two_waypoints():
    return Trajectory.build(JOINT_NAMES, [np.zeros(6), Q], [0.0, 2.0])


def _load_paused(viewer, trajectory):
    viewer.update_trajectory(trajectory)
    viewer._gui_play.value = False


def test_playback_gui_exists_before_any_client(viewer):
    """Review Focus 5: controls exist from construction, before a browser connects."""
    assert viewer._gui_play.value is True
    assert viewer._gui_loop.value is True
    assert viewer._gui_speed.value == 1.0
    assert viewer._gui_time.value == 0.0


def test_show_time_poses_interpolated(viewer, env):
    viewer.update_environment(env)
    traj = _two_waypoints()
    _load_paused(viewer, traj)
    viewer.show_time(1.0)
    xyz, _ = link_poses(env, JOINT_NAMES, traj.sample(1.0))["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


def test_update_trajectory_from_composite(viewer, env):
    viewer.update_environment(env)
    waypoints = []
    for q, t in ((np.zeros(6), 0.0), (Q, 1.5)):
        wp = StateWaypoint(JOINT_NAMES, q)
        wp.setTime(t)
        waypoints.append(StateWaypointPoly_wrap_StateWaypoint(wp))
    composite = _composite(waypoints)
    _load_paused(viewer, composite)
    assert viewer._gui_time.max == Trajectory.from_composite(composite).duration_s


def test_joint_waypoint_composite_plays(viewer, env):
    """Review Focus 4: joint waypoints carry no time; index is time."""
    viewer.update_environment(env)
    waypoints = [
        JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(JOINT_NAMES, q))
        for q in (np.zeros(6), Q, np.zeros(6))
    ]
    _load_paused(viewer, _composite(waypoints))
    assert viewer._gui_time.max == 2.0
    viewer.show_time(1.0)
    xyz, _ = link_poses(env, JOINT_NAMES, Q)["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


def test_subset_joint_trajectory_poses_by_its_own_names(viewer, env):
    """Review Focus 3: a trajectory over a reordered subset of joints."""
    viewer.update_environment(env)
    traj = Trajectory.build(["joint_3", "joint_1"], [[0.0, 0.0], [0.4, -0.7]], [0.0, 1.0])
    _load_paused(viewer, traj)
    viewer.show_time(1.0)
    xyz, _ = link_poses(env, ["joint_1", "joint_3"], [-0.7, 0.4])["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


def test_player_advances_time(viewer, env):
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    deadline = time.monotonic() + PLAYER_ADVANCE_TIMEOUT_S
    while viewer._gui_time.value == 0.0 and time.monotonic() < deadline:
        time.sleep(PLAYBACK_DT_S)
    assert viewer._gui_time.value > 0.0


def test_update_joint_positions_stops_playback(viewer, env):
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    viewer.update_joint_positions(JOINT_NAMES, Q)
    assert viewer._gui_play.value is False
    xyz, _ = link_poses(env, JOINT_NAMES, Q)["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


def test_plot_trajectory_adds_path_and_axes(viewer, env):
    viewer.update_environment(env)
    manip = ManipulatorInfo("manipulator", "base_link", "tool0")
    traj = Trajectory.build(JOINT_NAMES, [np.zeros(6), Q / 2, Q], [0.0, 1.0, 2.0])
    viewer.plot_trajectory(traj, manip, axes_length=0.05)
    path, axes = viewer._plot_handles
    assert len(axes.batched_positions) == 3
    tcp = env.getKinematicGroup("manipulator").calcFwdKin(Q)["tool0"]
    np.testing.assert_allclose(axes.batched_positions[-1], tcp.translation, atol=PLOT_ATOL_M)
    assert len(path.points) == 3


def test_close_joins_player():
    viewer = TesseractViewer(port=_free_port())
    player = viewer._player
    assert player.is_alive()
    viewer.close()
    assert not player.is_alive()


def test_update_trajectory_restarts_playback(viewer, env):
    viewer.update_environment(env)
    viewer.update_joint_positions(JOINT_NAMES, Q)
    viewer.update_trajectory(_two_waypoints())
    assert viewer._gui_play.value is True


def _wait_for(condition):
    deadline = time.monotonic() + PLAYER_ADVANCE_TIMEOUT_S
    while not condition() and time.monotonic() < deadline:
        time.sleep(PLAYBACK_DT_S)
    assert condition()


@pytest.fixture
def other_env():
    env = Environment()
    assert env.init(OTHER_ROBOT_URDF, GeneralResourceLocator())
    return env


@pytest.fixture
def log_messages():
    messages: list[str] = []
    sink = logger.add(messages.append, format="{level} {message}")
    yield messages
    logger.remove(sink)


def test_update_trajectory_with_unknown_joint_keeps_the_previous_one(viewer, env):
    viewer.update_environment(env)
    good = _two_waypoints()
    _load_paused(viewer, good)
    bad = Trajectory.build(["no_such_joint"], [[0.0], [1.0]], [0.0, 1.0])
    with pytest.raises(UnknownJointError):
        viewer.update_trajectory(bad)
    assert viewer._trajectory is good
    assert viewer._gui_play.value is False
    assert viewer._gui_time.max == good.duration_s


def test_update_environment_with_other_joints_drops_the_trajectory(viewer, env, other_env):
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    viewer.update_environment(other_env)
    assert viewer._trajectory is None
    assert viewer._gui_time.disabled
    time.sleep(3 * PLAYBACK_DT_S)
    assert viewer._player.is_alive()
    with pytest.raises(NoTrajectoryError):
        viewer.show_time(0.0)


def test_update_environment_with_the_same_joints_keeps_the_trajectory(viewer, env):
    """The pointcloud example re-pushes a re-tinted environment."""
    viewer.update_environment(env)
    traj = _two_waypoints()
    viewer.update_trajectory(traj)
    viewer.update_environment(env)
    assert viewer._trajectory is traj
    assert viewer._gui_time.disabled is False


def test_failing_tick_pauses_and_logs_and_player_survives(viewer, env, log_messages):
    """An environment changed behind the viewer's back fails on the player thread."""
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    with viewer._pose_lock:
        viewer._trajectory = Trajectory.build(["no_such_joint"], [[0.0], [1.0]], [0.0, 1.0])
    _wait_for(lambda: viewer._gui_play.value is False)
    assert viewer._player.is_alive()
    assert any("no_such_joint" in m and m.startswith("ERROR") for m in log_messages)
    # The player is still in service: a new trajectory plays.
    viewer.update_trajectory(_two_waypoints())
    _wait_for(lambda: viewer._gui_time.value > 0.0)


def test_update_joint_positions_before_environment_raises(viewer):
    with pytest.raises(NoEnvironmentError):
        viewer.update_joint_positions(JOINT_NAMES, Q)


def test_update_trajectory_before_environment_raises(viewer):
    with pytest.raises(NoEnvironmentError):
        viewer.update_trajectory(_two_waypoints())
    assert viewer._trajectory is None


def test_show_time_without_trajectory_raises(viewer, env):
    viewer.update_environment(env)
    with pytest.raises(NoTrajectoryError):
        viewer.show_time(0.0)


def test_busy_port_raises_instead_of_moving_on():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as holder:
        holder.bind(("127.0.0.1", 0))
        holder.listen()
        port = holder.getsockname()[1]
        with pytest.raises(ViewerPortInUseError, match=str(port)):
            TesseractViewer(port=port)


def test_port_zero_picks_a_free_port():
    with TesseractViewer(port=0) as viewer:
        port = viewer.server.get_port()
        assert port != 0
        assert viewer.url == f"http://127.0.0.1:{port}"
