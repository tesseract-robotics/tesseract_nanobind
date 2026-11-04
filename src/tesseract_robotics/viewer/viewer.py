"""Browser viewer for a tesseract environment, backed by viser."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from types import TracebackType

import numpy as np
import viser
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment

from .errors import NoEnvironmentError, UnknownJointError, ViewerCloseTimeoutError
from .geometry import visual_to_trimesh
from .poses import isometry_to_pose, link_poses
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
# Slider upper bound before any trajectory [s]; viser needs max > min.
EMPTY_TIME_MAX_S = 1.0
# How long close() waits for the player [s]: many ticks, so only a wedged pose update trips it.
PLAYER_JOIN_TIMEOUT_S = 5.0
# Default TCP axes length for plot_trajectory [m]; the old viewer's default.
PLOT_AXES_LENGTH_M = 0.1
# Default TCP path colour for plot_trajectory, 8-bit RGB: white, the old viewer's default.
PLOT_COLOR_RGB = (255, 255, 255)
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
            port: TCP port to bind.
        """
        self._host = host
        self._server = viser.ViserServer(host=host, port=port, verbose=False)
        self._closed = False
        self._env: Environment | None = None
        self._link_frames: dict[str, viser.FrameHandle] = {}
        self._visual_handles: list[viser.GlbHandle] = []
        self._joint_names: list[str] = []
        self._q: NDArray[np.float64] = np.zeros(0)
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
            self._gui_speed = gui.add_number("Speed", 1.0, min=0.0, step=0.1)
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

        Args:
            env: Environment to draw; kept for later pose updates.
        """
        with self._pose_lock:
            self._clear_scene()
            self._env = env
            scene_graph = env.getSceneGraph()
            for link_name in env.getLinkNames():
                frame_name = f"/links/{link_name}"
                self._link_frames[link_name] = self._server.scene.add_frame(
                    frame_name, show_axes=False
                )
                for i, visual in enumerate(scene_graph.getLink(link_name).visual):
                    position, wxyz = isometry_to_pose(visual.origin)
                    for j, mesh in enumerate(visual_to_trimesh(visual, link_name)):
                        self._visual_handles.append(
                            self._server.scene.add_mesh_trimesh(
                                f"{frame_name}/visual_{i}_{j}", mesh, position=position, wxyz=wxyz
                            )
                        )
            self._joint_names = list(env.getActiveJointNames())
            self._q = np.asarray(env.getCurrentJointValues(), dtype=np.float64)
            self._pose_links(self._joint_names, self._q)

    def update_joint_positions(self, joint_names: Sequence[str], q: ArrayLike) -> None:
        """Show the environment with the named joints at `q`; stops playback.

        Args:
            joint_names: Joints to set; others keep the environment's current values.
            q: Value per joint.
        """
        with self._pose_lock:
            self._gui_play.value = False
            self._joint_names = list(joint_names)
            self._q = np.asarray(q, dtype=np.float64)
            self._pose_links(self._joint_names, self._q)

    def update_trajectory(self, trajectory: CompositeInstruction | Trajectory) -> None:
        """Animate `trajectory` from its start; loops while `Loop` is on.

        Args:
            trajectory: Planned program or joint trajectory. A program's joint
                waypoints carry no time, so their index is their time [s].
        """
        traj = (
            trajectory
            if isinstance(trajectory, Trajectory)
            else Trajectory.from_composite(trajectory)
        )
        with self._pose_lock:
            self._trajectory = traj
        self._gui_time.max = max(traj.duration_s, TIME_SLIDER_STEP_S)
        self._gui_time.disabled = False
        self._t_s = 0.0
        self._gui_time.value = 0.0
        self.show_time(0.0)
        self._gui_play.value = True

    def show_time(self, t_s: float) -> None:
        """Pose the scene at `t_s` along the current trajectory.

        Args:
            t_s: Trajectory time [s], clamped to the trajectory.
        """
        traj = self._trajectory
        if traj is None:
            return
        self._pose_links(list(traj.joint_names), traj.sample(t_s))

    def plot_trajectory(
        self,
        trajectory: CompositeInstruction | Trajectory,
        manip_info: ManipulatorInfo,
        *,
        axes_length: float = PLOT_AXES_LENGTH_M,
        color: tuple[int, int, int] = PLOT_COLOR_RGB,
    ) -> None:
        """Draw the TCP path of `trajectory` with an axes triad per waypoint.

        Args:
            trajectory: Planned program or joint trajectory.
            manip_info: Kinematic group (`manipulator`) and TCP link (`tcp_frame`).
            axes_length: Triad axis length [m].
            color: Path colour, 8-bit RGB.

        Raises:
            UnknownJointError: the group has a joint the trajectory lacks.
        """
        env = self._require_env()
        traj = (
            trajectory
            if isinstance(trajectory, Trajectory)
            else Trajectory.from_composite(trajectory)
        )
        kin = env.getKinematicGroup(manip_info.manipulator)
        group_joints = list(kin.getJointNames())
        missing = set(group_joints) - set(traj.joint_names)
        if missing:
            raise UnknownJointError(f"trajectory lacks group joints: {sorted(missing)}")
        columns = [traj.joint_names.index(name) for name in group_joints]
        poses = [
            isometry_to_pose(kin.calcFwdKin(q[columns])[manip_info.tcp_frame])
            for q in traj.positions
        ]
        positions = np.stack([p for p, _ in poses])
        wxyzs = np.stack([w for _, w in poses])
        n = len(self._plot_handles) // 2
        self._plot_handles.append(
            self._server.scene.add_spline_catmull_rom(
                f"/plot/{n}/path", positions, color=color, thickness=PLOT_PATH_THICKNESS_M
            )
        )
        self._plot_handles.append(
            self._server.scene.add_batched_axes(
                f"/plot/{n}/axes",
                wxyzs,
                positions,
                axes_length=axes_length,
                axes_radius=axes_length / 20.0,
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
        self._server.stop()
        if self._player.is_alive():
            raise ViewerCloseTimeoutError(
                f"player thread still running {PLAYER_JOIN_TIMEOUT_S} s after close()"
            )

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
            if self._env is None:
                return
            poses = link_poses(self._env, joint_names, q)
            with self._server.atomic():
                for link_name, (position, wxyz) in poses.items():
                    frame = self._link_frames[link_name]
                    frame.position = position
                    frame.wxyz = wxyz

    def _on_time_dragged(self, event: viser.GuiEvent) -> None:
        # The player sets the slider too; only a browser drag pauses and scrubs.
        if event.client is None:
            return
        self._gui_play.value = False
        self._t_s = self._gui_time.value
        self.show_time(self._t_s)

    def _play(self) -> None:
        while not self._stop_player.wait(PLAYBACK_DT_S):
            with self._pose_lock:
                self._tick()

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

    def _clear_scene(self) -> None:
        for handle in self._visual_handles:
            handle.remove()
        for frame in self._link_frames.values():
            frame.remove()
        self._visual_handles.clear()
        self._link_frames.clear()
