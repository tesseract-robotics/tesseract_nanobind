"""TesseractViewer: viser server lifecycle and scene state."""

import socket
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import TesseractViewer
from tesseract_robotics.viewer.poses import link_poses

# Seconds to wait for the server socket to accept a TCP connection after start.
CONNECT_TIMEOUT_S = 5.0
# Frame position vs. an independent evaluation of the same state [m]; float64 round-off.
POSE_ATOL_M = 1e-12

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
