"""Browser viewer for a tesseract environment, backed by viser."""

from __future__ import annotations

import time
from collections.abc import Sequence
from types import TracebackType

import numpy as np
import viser
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.tesseract_environment import Environment

from .geometry import visual_to_trimesh
from .poses import isometry_to_pose, link_poses

# Loopback by default: the viewer has no auth, so exposing it is an explicit choice.
DEFAULT_HOST = "127.0.0.1"
# The old viewer's port, so documented URLs stay valid.
DEFAULT_PORT = 8000
# serve_forever() wake-up interval [s]; only bounds Ctrl-C latency.
SERVE_POLL_S = 1.0


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
        self._clear_scene()
        self._env = env
        scene_graph = env.getSceneGraph()
        for link_name in env.getLinkNames():
            frame_name = f"/links/{link_name}"
            self._link_frames[link_name] = self._server.scene.add_frame(frame_name, show_axes=False)
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
        """Show the environment with the named joints at `q`.

        Args:
            joint_names: Joints to set; others keep the environment's current values.
            q: Value per joint.
        """
        self._joint_names = list(joint_names)
        self._q = np.asarray(q, dtype=np.float64)
        self._pose_links(self._joint_names, self._q)

    def close(self) -> None:
        """Stop serving and release the port. Idempotent."""
        if self._closed:
            return
        self._closed = True
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

    def _pose_links(self, joint_names: Sequence[str], q: NDArray[np.float64]) -> None:
        if self._env is None:
            return
        poses = link_poses(self._env, joint_names, q)
        with self._server.atomic():
            for link_name, (position, wxyz) in poses.items():
                frame = self._link_frames[link_name]
                frame.position = position
                frame.wxyz = wxyz

    def _clear_scene(self) -> None:
        for handle in self._visual_handles:
            handle.remove()
        for frame in self._link_frames.values():
            frame.remove()
        self._visual_handles.clear()
        self._link_frames.clear()
