"""TesseractViewer: viser server lifecycle and scene state."""

import asyncio
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import msgspec
import numpy as np
import pytest
import viser
import websockets
import zstandard
from loguru import logger

from tesseract_robotics import tesseract_geometry as tg
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
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    LinkId,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    Environment,
    RemoveLinkCommand,
)
from tesseract_robotics.tesseract_scene_graph import Joint, JointType, Link, Visual
from tesseract_robotics.viewer import TesseractViewer, Trajectory
from tesseract_robotics.viewer.errors import (
    NoEnvironmentError,
    NoTrajectoryError,
    StaleSceneError,
    UnknownJointError,
    UnsupportedGeometryError,
    UnsupportedMoveTypeError,
    ViewerCloseTimeoutError,
    ViewerPortInUseError,
)
from tesseract_robotics.viewer.poses import link_poses
from tesseract_robotics.viewer.tcp_path import tcp_path
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


def _irb2400():
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


@pytest.fixture(scope="module")
def env():
    return _irb2400()


def _add_link(env, name, geometry):
    link = Link(name)
    visual = Visual()
    visual.geometry = geometry
    link.addVisual(visual)
    joint = Joint(f"{name}_joint")
    joint.parent_link_id = LinkId("base_link")
    joint.child_link_id = LinkId(name)
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = Isometry3d()
    assert env.applyCommand(AddLinkCommand(link, joint))


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
    assert set(viewer._link_frames) == {lid.name() for lid in env.getLinkIds()}


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
    expected = link_poses(
        env, [jid.name() for jid in env.getActiveJointIds()], env.getCurrentJointValues()
    )
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
    expected = tcp_path(env, traj, manip).segments
    np.testing.assert_allclose(path.points, expected, atol=PLOT_ATOL_M)


def test_plot_trajectory_single_waypoint_draws_axes_only(viewer, env):
    viewer.update_environment(env)
    manip = ManipulatorInfo("manipulator", "base_link", "tool0")
    viewer.plot_trajectory(Trajectory.build(JOINT_NAMES, [Q], [0.0]), manip)
    path, axes = viewer._plot_handles
    assert len(path.points) == 0
    assert len(axes.batched_positions) == 1


def test_plot_trajectory_failure_draws_nothing(viewer, env):
    viewer.update_environment(env)
    manip = ManipulatorInfo("manipulator", "base_link", "tool0")
    circular = (MoveInstructionType.FREESPACE, MoveInstructionType.CIRCULAR)
    traj = Trajectory.build(JOINT_NAMES, [np.zeros(6), Q], [0.0, 1.0], circular)
    with pytest.raises(UnsupportedMoveTypeError):
        viewer.plot_trajectory(traj, manip)
    assert viewer._plot_handles == []


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


def test_failing_tick_pauses_and_logs_and_player_survives(viewer, log_messages):
    """An environment changed behind the viewer fails on the player thread: no caller."""
    env = _irb2400()
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    with viewer._pose_lock:
        _add_link(env, "added", tg.Box(0.1, 0.1, 0.1))
    _wait_for(lambda: viewer._gui_play.value is False)
    assert viewer._player.is_alive()
    assert any("added" in m and m.startswith("ERROR") for m in log_messages)
    # The player is still in service: push again and it plays.
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    _wait_for(lambda: viewer._gui_time.value > 0.0)


def test_added_link_raises_before_any_frame_moves(viewer):
    env = _irb2400()
    viewer.update_environment(env)
    before = np.array(viewer._link_frames["tool0"].position)
    _add_link(env, "added", tg.Box(0.1, 0.1, 0.1))
    with pytest.raises(StaleSceneError, match="added"):
        viewer.update_joint_positions(JOINT_NAMES, Q)
    np.testing.assert_array_equal(viewer._link_frames["tool0"].position, before)


def test_removed_link_raises(viewer):
    env = _irb2400()
    _add_link(env, "added", tg.Box(0.1, 0.1, 0.1))
    viewer.update_environment(env)
    assert env.applyCommand(RemoveLinkCommand("added"))
    with pytest.raises(StaleSceneError, match="added"):
        viewer.update_joint_positions(JOINT_NAMES, Q)


def test_failed_push_keeps_the_previous_scene(viewer, env):
    viewer.update_environment(env)
    handles = list(viewer._visual_handles)
    frames = set(viewer._link_frames)
    bad = _irb2400()
    _add_link(bad, "floor", tg.Plane(0.0, 0.0, 1.0, 0.0))
    with pytest.raises(UnsupportedGeometryError, match="floor"):
        viewer.update_environment(bad)
    assert viewer._env is env
    assert viewer._visual_handles == handles
    assert set(viewer._link_frames) == frames
    viewer.update_joint_positions(JOINT_NAMES, Q)
    xyz, _ = link_poses(env, JOINT_NAMES, Q)["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


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


def test_loop_off_stops_at_the_end(viewer, env):
    viewer.update_environment(env)
    traj = Trajectory.build(JOINT_NAMES, [np.zeros(6), Q], [0.0, PLAYBACK_DT_S])
    viewer._gui_loop.value = False
    viewer.update_trajectory(traj)
    _wait_for(lambda: viewer._gui_play.value is False)
    assert viewer._gui_time.value == traj.duration_s
    xyz, _ = link_poses(env, JOINT_NAMES, Q)["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)


def test_loop_on_wraps_to_the_start(viewer, env):
    viewer.update_environment(env)
    traj = Trajectory.build(JOINT_NAMES, [np.zeros(6), Q], [0.0, 3 * PLAYBACK_DT_S])
    viewer.update_trajectory(traj)
    _wait_for(lambda: viewer._t_s > 0.0)
    _wait_for(lambda: viewer._t_s == 0.0)
    assert viewer._gui_play.value is True


def test_speed_zero_holds_time(viewer, env):
    viewer.update_environment(env)
    viewer._gui_speed.value = 0.0
    viewer.update_trajectory(_two_waypoints())
    time.sleep(3 * PLAYBACK_DT_S)
    assert viewer._t_s == 0.0


def test_speed_scales_playback(viewer, env):
    viewer.update_environment(env)
    viewer._gui_speed.value = 2.0
    with viewer._pose_lock:
        viewer.update_trajectory(_two_waypoints())
        viewer._tick()
        assert viewer._t_s == 2.0 * PLAYBACK_DT_S


def test_close_raises_when_the_player_is_wedged(monkeypatch):
    monkeypatch.setattr("tesseract_robotics.viewer.viewer.PLAYER_JOIN_TIMEOUT_S", 2 * PLAYBACK_DT_S)
    viewer = TesseractViewer(port=0)
    release = threading.Event()
    entered = threading.Event()

    def wedged_tick():
        entered.set()
        release.wait()

    viewer._tick = wedged_tick
    assert entered.wait(PLAYER_ADVANCE_TIMEOUT_S)
    try:
        with pytest.raises(ViewerCloseTimeoutError):
            viewer.close()
    finally:
        release.set()
    viewer._player.join(PLAYER_ADVANCE_TIMEOUT_S)
    assert not viewer._player.is_alive()


def test_close_waits_out_a_long_push_on_another_thread(monkeypatch):
    """A long push holds the pose lock past the join timeout; close() must not call it wedged."""
    monkeypatch.setattr("tesseract_robotics.viewer.viewer.PLAYER_JOIN_TIMEOUT_S", 2 * PLAYBACK_DT_S)
    viewer = TesseractViewer(port=0)
    holding = threading.Event()
    release = threading.Event()

    def long_push():
        with viewer._pose_lock:
            holding.set()
            release.wait()

    pusher = threading.Thread(target=long_push)
    pusher.start()
    assert holding.wait(PLAYER_ADVANCE_TIMEOUT_S)
    time.sleep(2 * PLAYBACK_DT_S)  # The player is now waiting on the pose lock.
    errors = []
    closer = threading.Thread(target=_capture, args=(viewer.close, errors))
    closer.start()
    time.sleep(6 * PLAYBACK_DT_S)  # Longer than the join timeout.
    assert closer.is_alive(), "close() must wait for the push before stopping the server"
    release.set()
    closer.join(PLAYER_ADVANCE_TIMEOUT_S)
    pusher.join(PLAYER_ADVANCE_TIMEOUT_S)
    assert not closer.is_alive()
    assert errors == []
    assert not viewer._player.is_alive()


def _capture(function, errors):
    try:
        function()
    except Exception as error:
        errors.append(error)


# A browser, over the real websocket. Wire format from viser's client: frames are
# [uncompressed size, compressed size, zstd payload], little-endian u64 sizes.
WIRE_SIZE_BYTES = 8
# How long a late client listens for the server's replay [s].
REPLAY_WINDOW_S = 1.0
# A client's camera message, which viser waits for before it runs a client's GUI callbacks.
CAMERA_MESSAGE = {
    "type": "ViewerCameraMessage",
    "wxyz": (1.0, 0.0, 0.0, 0.0),
    "position": (3.0, 0.0, 0.0),
    "image_height": 480,
    "image_width": 640,
    "fov": 1.0,
    "near": 0.01,
    "far": 100.0,
    "look_at": (0.0, 0.0, 0.0),
    "up_direction": (0.0, 0.0, 1.0),
}


def _decode_frame(frame):
    size = int.from_bytes(frame[:WIRE_SIZE_BYTES], "little")
    compressed = int.from_bytes(frame[WIRE_SIZE_BYTES : 2 * WIRE_SIZE_BYTES], "little")
    payload = frame[2 * WIRE_SIZE_BYTES : 2 * WIRE_SIZE_BYTES + compressed]
    decompressed = zstandard.ZstdDecompressor().decompress(payload, max_output_size=size)
    return msgspec.msgpack.decode(decompressed)["messages"]


async def _listen(socket_, window_s=REPLAY_WINDOW_S):
    messages = []
    deadline = time.monotonic() + window_s
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            messages += _decode_frame(await asyncio.wait_for(socket_.recv(), remaining))
        except TimeoutError:
            break
    return messages


def _connect(viewer):
    return websockets.connect(
        f"ws://127.0.0.1:{viewer.server.get_port()}",
        subprotocols=[f"viser-v{viser.__version__}"],
        max_size=None,
    )


def _late_client_replay(viewer):
    async def replay():
        async with _connect(viewer) as socket_:
            return await _listen(socket_)

    return asyncio.run(replay())


def _names(messages, message_type):
    return [m["name"] for m in messages if m["type"] == message_type]


def test_late_client_gets_the_scene_and_playback_controls(viewer, env):
    """Review Focus 5, as a browser sees it."""
    viewer.update_environment(env)
    _load_paused(viewer, _two_waypoints())
    messages = _late_client_replay(viewer)
    assert set(_names(messages, "FrameMessage")) >= {
        f"/links/{lid.name()}" for lid in env.getLinkIds()
    }
    assert len(_names(messages, "GlbMessage")) == len(viewer._visual_handles) > 0
    assert [m for m in messages if m["type"] == "GuiSliderMessage"]
    assert [m for m in messages if m["type"] == "GuiCheckboxMessage"]


def test_late_client_after_a_swap_sees_only_the_new_scene(viewer, env, other_env):
    """Review Focus 2, as a browser sees it: replaced meshes and frames are really gone."""
    viewer.update_environment(env)
    viewer.update_environment(other_env)
    messages = _late_client_replay(viewer)
    links = {n for n in _names(messages, "FrameMessage") if n.startswith("/links/")}
    assert links == {"/links/world", "/links/arm"}
    assert _names(messages, "GlbMessage") == []


def test_player_tick_does_not_overwrite_a_drag_waiting_on_the_lock(viewer, env):
    viewer.update_environment(env)
    viewer.update_trajectory(_two_waypoints())
    drag_to_s = 1.5
    with viewer._pose_lock:  # keeps the player thread out; the ticks are ours
        viewer._tick()
        # What viser does on its event loop before the drag callback can take the lock.
        viewer._gui_time._impl.value = drag_to_s
        viewer._tick()
        assert viewer._gui_time.value == drag_to_s
        viewer._on_time_dragged(SimpleNamespace(client=object()))
    assert viewer._t_s == drag_to_s
    assert viewer._gui_play.value is False


def test_browser_drag_pauses_and_scrubs(viewer, env):
    viewer.update_environment(env)
    traj = _two_waypoints()
    viewer.update_trajectory(traj)
    drag_to_s = 1.5

    async def drag():
        async with _connect(viewer) as socket_:
            await _listen(socket_, REPLAY_WINDOW_S)
            await socket_.send(msgspec.msgpack.encode(CAMERA_MESSAGE))
            await _listen(socket_, 2 * PLAYBACK_DT_S)
            update = {"value": drag_to_s}
            message = {"type": "GuiUpdateMessage", "uuid": viewer._gui_time._impl.uuid}
            await socket_.send(msgspec.msgpack.encode({**message, "updates": update}))
            await _listen(socket_, REPLAY_WINDOW_S)

    asyncio.run(drag())
    assert viewer._gui_play.value is False
    assert viewer._t_s == drag_to_s
    xyz, _ = link_poses(env, JOINT_NAMES, traj.sample(drag_to_s))["tool0"]
    np.testing.assert_allclose(viewer._link_frames["tool0"].position, xyz, atol=POSE_ATOL_M)
