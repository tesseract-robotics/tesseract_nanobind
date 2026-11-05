"""Browser viewer for a tesseract environment, backed by viser."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from types import TracebackType

import numpy as np
import viser
from loguru import logger
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment

from .errors import (
    NoEnvironmentError,
    NoTrajectoryError,
    StaleSceneError,
    ViewerCloseTimeoutError,
    ViewerPortInUseError,
)
from .geometry import visual_to_trimesh
from .poses import isometry_to_pose, link_poses
from .tcp_path import tcp_path
from .trajectory import Trajectory

# Loopback by default: the viewer has no auth, so exposing it is an explicit choice.
DEFAULT_HOST = "127.0.0.1"
# The old viewer's port, so documented URLs stay valid.
DEFAULT_PORT = 8000
# serve_forever() wake-up interval [s]; only bounds Ctrl-C latency.
SERVE_POLL_S = 1.0
# Player tick [s]: 30 Hz, a smooth browser frame rate without flooding the websocket.
PLAYBACK_DT_S = 1.0 / 30.0
# Time slider resolution [s]: 10 ms, finer than a 30 Hz frame.
TIME_SLIDER_STEP_S = 0.01
# Playback speed: 1x real time, adjustable in 10 % steps.
DEFAULT_SPEED = 1.0
SPEED_STEP = 0.1
# Slider upper bound before any trajectory [s]; viser needs max > min.
EMPTY_TIME_MAX_S = 1.0
# How long close() waits for the player [s]: many ticks, so only a wedged pose update trips it.
PLAYER_JOIN_TIMEOUT_S = 5.0
# Default TCP axes length for plot_trajectory [m]; the old viewer's default.
PLOT_AXES_LENGTH_M = 0.1
# Default TCP path colour for plot_trajectory, 8-bit RGB: white, the old viewer's default.
PLOT_COLOR_RGB = (255, 255, 255)
# TCP triad axis radius per unit axis length [-]: slim enough that triads don't merge.
PLOT_AXES_RADIUS_RATIO = 0.05
# TCP path line thickness [m]: 2 mm, visible at robot scale.
PLOT_PATH_THICKNESS_M = 0.002


class TesseractViewer:
    """A tesseract environment in the browser, served from construction until `close()`.

    Each link is a viser frame `/links/<link>` posed from tesseract's own scene state;
    its visuals hang under it. Custom markers go through `server.scene`.

    Example:
        ```python
        with TesseractViewer() as viewer:
            viewer.update_environment(env)
            print(viewer.url)
            input("Press Enter to exit...")
        ```
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        """Start serving.

        Args:
            host: Interface to bind.
            port: TCP port to bind; 0 lets the OS pick a free one (see `url`).

        Raises:
            ViewerPortInUseError: `port` is taken.
        """
        self._host = host
        self._server = viser.ViserServer(host=host, port=port, verbose=False)
        # viser silently moves to the next free port; a documented URL must not lie.
        if port != 0 and self._server.get_port() != port:
            self._server.stop()
            raise ViewerPortInUseError(
                f"port {port} on {host} is in use: stop what holds it, or pass "
                "TesseractViewer(port=...) with another port"
            )
        self._closed = False
        self._env: Environment | None = None
        self._link_frames: dict[str, viser.FrameHandle] = {}
        self._visual_handles: list[viser.GlbHandle] = []
        self._plot_handles: list[viser.SceneNodeHandle] = []
        self._trajectory: Trajectory | None = None
        self._t_s = 0.0
        # Re-entrant: a player tick holds it across show_time(), which takes it again.
        self._pose_lock = threading.RLock()

        gui = self._server.gui
        with gui.add_folder("Playback"):
            self._gui_play = gui.add_checkbox("Play", True)
            self._gui_time = gui.add_slider(
                "Time [s]", 0.0, EMPTY_TIME_MAX_S, TIME_SLIDER_STEP_S, 0.0, disabled=True
            )
            self._gui_speed = gui.add_number("Speed", DEFAULT_SPEED, min=0.0, step=SPEED_STEP)
            self._gui_loop = gui.add_checkbox("Loop", True)
        self._gui_time.on_update(self._on_time_dragged)

        self._stop_player = threading.Event()
        self._player = threading.Thread(
            target=self._play, name="tesseract-viewer-player", daemon=True
        )
        self._player.start()

    @property
    def server(self) -> viser.ViserServer:
        """The viser server, for markers and GUI beyond what this class draws."""
        return self._server

    @property
    def url(self) -> str:
        """Address to open in a browser."""
        return f"http://{self._host}:{self._server.get_port()}"

    def update_environment(self, env: Environment) -> None:
        """Replace the scene with `env`'s links and visuals at its current state.

        The current trajectory stays when `env` has all its joints, so a re-pushed
        environment keeps playing; otherwise it is dropped with a warning.

        Args:
            env: Environment to draw; kept for later pose updates.

        Raises:
            UnsupportedGeometryError: a visual has no mesh form; the current scene stays.
        """
        # Converted before anything changes, and off the lock: playback runs on meanwhile.
        scene_graph = env.getSceneGraph()
        link_visuals = {
            link_name: [
                (isometry_to_pose(visual.origin), visual_to_trimesh(visual, link_name))
                for visual in scene_graph.getLink(link_name).visual
            ]
            for link_name in env.getLinkNames()
        }
        with self._pose_lock:
            self._clear_scene()
            self._env = env
            for link_name, visuals in link_visuals.items():
                frame_name = f"/links/{link_name}"
                self._link_frames[link_name] = self._server.scene.add_frame(
                    frame_name, show_axes=False
                )
                for i, ((position, wxyz), meshes) in enumerate(visuals):
                    for j, mesh in enumerate(meshes):
                        self._visual_handles.append(
                            self._server.scene.add_mesh_trimesh(
                                f"{frame_name}/visual_{i}_{j}", mesh, position=position, wxyz=wxyz
                            )
                        )
            traj = self._trajectory
            if traj is not None and not set(traj.joint_names) <= set(env.getJointNames()):
                logger.warning(
                    f"dropping the trajectory: the new environment lacks some of "
                    f"{list(traj.joint_names)}"
                )
                self._clear_trajectory()
            self._pose_links(
                env.getActiveJointNames(),
                np.asarray(env.getCurrentJointValues(), dtype=np.float64),
            )

    def update_joint_positions(self, joint_names: Sequence[str], q: ArrayLike) -> None:
        """Show the environment with the named joints at `q`; stops playback.

        Args:
            joint_names: Joints to set; others keep the environment's current values.
            q: Value per joint.

        Raises:
            NoEnvironmentError: no environment yet.
            UnknownJointError: a name is not a joint of the environment.
            StaleSceneError: the environment gained or lost links; push it again.
        """
        with self._pose_lock:
            self._pose_links(joint_names, np.asarray(q, dtype=np.float64))
            self._gui_play.value = False

    def update_trajectory(self, trajectory: CompositeInstruction | Trajectory) -> None:
        """Animate `trajectory` from its start; loops while `Loop` is on.

        Args:
            trajectory: Planned program or joint trajectory. A program's joint
                waypoints carry no time, so their index is their time [s].

        Raises:
            NoEnvironmentError: no environment yet.
            UnknownJointError: the trajectory names a joint the environment lacks.
            StaleSceneError: the environment gained or lost links; push it again.
        """
        traj = (
            trajectory
            if isinstance(trajectory, Trajectory)
            else Trajectory.from_composite(trajectory)
        )
        with self._pose_lock:
            # Posing first validates: a bad trajectory raises before anything changes.
            self._pose_links(traj.joint_names, traj.sample(0.0))
            self._trajectory = traj
            self._gui_time.max = max(traj.duration_s, TIME_SLIDER_STEP_S)
            self._gui_time.disabled = False
            self._t_s = 0.0
            self._gui_time.value = 0.0
            self._gui_play.value = True

    def show_time(self, t_s: float) -> None:
        """Pose the scene at `t_s` along the current trajectory.

        Args:
            t_s: Trajectory time [s], clamped to the trajectory.

        Raises:
            NoTrajectoryError: no trajectory loaded.
            StaleSceneError: the environment gained or lost links; push it again.
        """
        with self._pose_lock:
            traj = self._trajectory
            if traj is None:
                raise NoTrajectoryError("call update_trajectory() first")
            self._pose_links(traj.joint_names, traj.sample(t_s))

    def plot_trajectory(
        self,
        trajectory: CompositeInstruction | Trajectory,
        manip_info: ManipulatorInfo,
        *,
        axes_length: float = PLOT_AXES_LENGTH_M,
        color: tuple[int, int, int] = PLOT_COLOR_RGB,
    ) -> None:
        """Draw the TCP path `trajectory` commands, with an axes triad per waypoint.

        See `tcp_path` for how each move type is drawn. Nothing is drawn if it raises.

        Args:
            trajectory: Planned program or joint trajectory.
            manip_info: Group (`manipulator`), TCP (`tcp_frame`, `tcp_offset`), `working_frame`.
            axes_length: Triad axis length [m].
            color: Path colour, 8-bit RGB.

        Raises:
            UnknownJointError: the group has a joint the trajectory lacks.
            NamedTcpOffsetError: `tcp_offset` is a link name.
            UnsupportedMoveTypeError: a move is neither `LINEAR` nor `FREESPACE`.
            MovingWorkingFrameError: a `LINEAR` move's working frame moves with the group.
        """
        env = self._require_env()
        traj = (
            trajectory
            if isinstance(trajectory, Trajectory)
            else Trajectory.from_composite(trajectory)
        )
        path = tcp_path(env, traj, manip_info)
        n = len(self._plot_handles) // 2
        self._plot_handles.append(
            self._server.scene.add_line_segments(
                f"/plot/{n}/path", path.segments, color, thickness=PLOT_PATH_THICKNESS_M
            )
        )
        self._plot_handles.append(
            self._server.scene.add_batched_axes(
                f"/plot/{n}/axes",
                path.wxyzs,
                path.positions,
                axes_length=axes_length,
                axes_radius=axes_length * PLOT_AXES_RADIUS_RATIO,
            )
        )

    def close(self) -> None:
        """Stop the player, stop serving and release the port. Idempotent.

        Raises:
            ViewerCloseTimeoutError: the player did not exit within `PLAYER_JOIN_TIMEOUT_S`.
        """
        if self._closed:
            return
        self._closed = True
        self._stop_player.set()
        self._player.join(PLAYER_JOIN_TIMEOUT_S)
        if self._player.is_alive():
            self._server.stop()
            raise ViewerCloseTimeoutError(
                f"player thread still running {PLAYER_JOIN_TIMEOUT_S} s after close()"
            )
        # A push on another thread (a GUI callback) finishes before the server goes.
        with self._pose_lock:
            self._server.stop()

    def serve_forever(self) -> None:
        """Block until Ctrl-C, then `close()`."""
        try:
            while True:
                time.sleep(SERVE_POLL_S)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def __enter__(self) -> TesseractViewer:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def _require_env(self) -> Environment:
        if self._env is None:
            raise NoEnvironmentError("call update_environment() first")
        return self._env

    def _pose_links(self, joint_names: Sequence[str], q: NDArray[np.float64]) -> None:
        with self._pose_lock:
            poses = link_poses(self._require_env(), joint_names, q)
            if poses.keys() != self._link_frames.keys():
                added = sorted(poses.keys() - self._link_frames.keys())
                removed = sorted(self._link_frames.keys() - poses.keys())
                raise StaleSceneError(
                    f"environment links changed (added {added}, removed {removed}): "
                    "call update_environment() again"
                )
            with self._server.atomic():
                for link_name, (position, wxyz) in poses.items():
                    frame = self._link_frames[link_name]
                    frame.position = position
                    frame.wxyz = wxyz

    def _on_time_dragged(self, event: viser.GuiEvent) -> None:
        # The player sets the slider too; only a browser drag pauses and scrubs.
        if event.client is None:
            return
        with self._pose_lock:
            # A drag already in flight when update_environment() dropped the trajectory.
            if self._trajectory is None:
                return
            self._gui_play.value = False
            self._t_s = self._gui_time.value
            self.show_time(self._t_s)

    def _play(self) -> None:
        while not self._stop_player.wait(PLAYBACK_DT_S):
            # Timed: a long push holds the lock, and close() must still be able to stop this.
            if not self._pose_lock.acquire(timeout=PLAYBACK_DT_S):
                continue
            try:
                self._tick()
            except Exception:
                # No caller to raise to, e.g. the environment changed behind the viewer.
                # Pause so it logs once; Play or update_trajectory() try again.
                logger.exception("playback paused: this tick failed")
                self._gui_play.value = False
            finally:
                self._pose_lock.release()

    def _tick(self) -> None:
        # Under the pose lock: a pause from another thread cannot interleave with a frame.
        traj = self._trajectory
        if traj is None or not self._gui_play.value:
            return
        t_s = self._t_s + PLAYBACK_DT_S * self._gui_speed.value
        if t_s > traj.duration_s:
            if self._gui_loop.value:
                t_s = 0.0
            else:
                t_s = traj.duration_s
                self._gui_play.value = False
        self._t_s = t_s
        self._gui_time.value = t_s
        self.show_time(t_s)

    def _clear_trajectory(self) -> None:
        self._trajectory = None
        self._t_s = 0.0
        self._gui_time.value = 0.0
        self._gui_time.max = EMPTY_TIME_MAX_S
        self._gui_time.disabled = True

    def _clear_scene(self) -> None:
        for handle in self._visual_handles:
            handle.remove()
        for frame in self._link_frames.values():
            frame.remove()
        self._visual_handles.clear()
        self._link_frames.clear()
