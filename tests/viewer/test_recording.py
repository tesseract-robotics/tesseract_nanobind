"""TesseractViewer.record: what a `.viser` file holds, decoded the way the static client does."""

import io
import struct
import time
from pathlib import Path

import msgspec
import numpy as np
import pytest
import viser
import zstandard

from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.viewer import RecordingPhase, TesseractViewer, Trajectory
from tesseract_robotics.viewer.errors import (
    NoEnvironmentError,
    RecordingFpsError,
    StaleSceneError,
)
from tesseract_robotics.viewer.poses import link_poses

JOINT_NAMES = [f"joint_{i + 1}" for i in range(6)]
FPS_HZ = 10.0
# The file starts with the decompressed size as a little-endian u64 [bytes].
SIZE_PREFIX_BYTES = 8
RAMP_DURATION_S = 1.0
# Frames of a ramp: one per 1/FPS_HZ, plus the end.
RAMP_FRAMES = 11
# Recording clock vs. the trajectory's duration [s]; float64 round-off over a few sleeps.
TIME_ATOL_S = 1e-9
# viser's wire format stores positions as float32: ~1e-7 relative on ~1 m positions.
POSITION_ATOL_M = 1e-6
# Long enough for the 30 Hz player to tick if record() let it [s].
PLAYER_TICKS_S = 0.2
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


def _irb2400() -> Environment:
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


@pytest.fixture(scope="module")
def env() -> Environment:
    return _irb2400()


@pytest.fixture
def viewer():
    with TesseractViewer(port=0) as viewer:
        yield viewer


def _ramp(start_rad: float = 0.0, stop_rad: float = 0.5) -> Trajectory:
    """Joint 1 moves, so tool0 changes pose on every frame."""
    return Trajectory.build(
        ["joint_1"], np.array([[start_rad], [stop_rad]]), np.array([0.0, RAMP_DURATION_S])
    )


def _decode(data: bytes) -> dict:
    """Decode: [u64 size], then zstd of [u64 msgpack length][msgpack][binary buffers]."""
    with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(data[SIZE_PREFIX_BYTES:])) as reader:
        payload = reader.read()
    (length,) = struct.unpack("<Q", payload[:8])
    return msgspec.msgpack.decode(payload[8 : 8 + length])


def _tool0_positions(recording: dict) -> list[tuple[float, list[float]]]:
    return [
        (t_s, message["position"])
        for t_s, message in recording["messages"]
        if message["type"] == "SetPositionMessage" and message["name"] == "/links/tool0"
    ]


def test_a_ramp_records_one_pose_per_frame_and_ends_on_its_duration(viewer, env):
    viewer.update_environment(env)
    recording = _decode(viewer.record([RecordingPhase(env, _ramp())], fps=FPS_HZ))

    times = [t_s for t_s, _ in _tool0_positions(recording)]
    assert len(times) == RAMP_FRAMES
    assert recording["durationSeconds"] == pytest.approx(RAMP_DURATION_S, abs=TIME_ATOL_S)
    assert times[-1] == pytest.approx(RAMP_DURATION_S, abs=TIME_ATOL_S)


def test_a_duration_between_frames_still_ends_exactly_on_it(viewer, env):
    viewer.update_environment(env)
    duration_s = 0.25
    traj = Trajectory.build(["joint_1"], np.array([[0.0], [0.5]]), np.array([0.0, duration_s]))

    recording = _decode(viewer.record([RecordingPhase(env, traj)], fps=FPS_HZ))

    assert recording["durationSeconds"] == pytest.approx(duration_s, abs=TIME_ATOL_S)


def test_a_recording_names_the_viser_version_it_was_made_with(viewer, env):
    viewer.update_environment(env)
    recording = _decode(viewer.record([RecordingPhase(env, _ramp())], fps=FPS_HZ))
    assert recording["viserVersion"] == viser.__version__


def test_phases_play_in_order_each_posed_by_its_own_environment(viewer, env):
    other = env.clone()
    other.setState(["joint_2"], np.array([0.7]))
    viewer.update_environment(env)

    recording = _decode(
        viewer.record(
            [RecordingPhase(env, _ramp()), RecordingPhase(other, _ramp())],
            fps=FPS_HZ,
        )
    )

    positions = _tool0_positions(recording)
    assert len(positions) == 2 * RAMP_FRAMES
    times = [t_s for t_s, _ in positions]
    assert times == sorted(times)
    # One frame apart: the second phase starts after the first, not on top of it.
    assert recording["durationSeconds"] == pytest.approx(
        2 * RAMP_DURATION_S + 1.0 / FPS_HZ, abs=TIME_ATOL_S
    )
    q_end = _ramp().sample(RAMP_DURATION_S)
    xyz_first_end, _ = link_poses(env, ["joint_1"], q_end)["tool0"]
    xyz_second_end, _ = link_poses(other, ["joint_1"], q_end)["tool0"]
    np.testing.assert_allclose(positions[RAMP_FRAMES - 1][1], xyz_first_end, atol=POSITION_ATOL_M)
    np.testing.assert_allclose(positions[-1][1], xyz_second_end, atol=POSITION_ATOL_M)


def test_the_player_adds_nothing_while_recording(viewer, env):
    viewer.update_environment(env)
    viewer.update_trajectory(_ramp())
    time.sleep(PLAYER_TICKS_S)

    recording = _decode(viewer.record([RecordingPhase(env, _ramp())], fps=FPS_HZ))

    # t = 0 also holds viser's snapshot of the live pose. A player tick would land after it.
    after_start = [t_s for t_s, _ in _tool0_positions(recording) if t_s > 0.0]
    assert len(after_start) == RAMP_FRAMES - 1


def test_the_live_scene_is_back_after_recording(viewer, env):
    viewer.update_environment(env)
    before = np.array(viewer._link_frames["tool0"].position)

    viewer.record([RecordingPhase(env, _ramp(0.5, 1.0))], fps=FPS_HZ)

    np.testing.assert_allclose(viewer._link_frames["tool0"].position, before, atol=POSITION_ATOL_M)
    assert viewer._env is env


def test_no_phases_records_the_scene_as_it_stands(viewer, env):
    viewer.update_environment(env)
    recording = _decode(viewer.record([], fps=FPS_HZ))
    assert recording["durationSeconds"] == 0.0
    assert any(
        message["type"] == "GlbMessage" or "Mesh" in message["type"]
        for _, message in recording["messages"]
    )


def test_static_scene_content_added_first_is_in_the_recording(viewer, env):
    viewer.update_environment(env)
    viewer.server.scene.add_grid("/floor")
    recording = _decode(viewer.record([RecordingPhase(env, _ramp())], fps=FPS_HZ))
    assert any(message.get("name") == "/floor" for _, message in recording["messages"])


@pytest.mark.parametrize("fps", [0.0, -1.0, float("nan")])
def test_a_frame_rate_that_is_not_positive_raises(viewer, env, fps):
    viewer.update_environment(env)
    with pytest.raises(RecordingFpsError):
        viewer.record([RecordingPhase(env, _ramp())], fps=fps)


def test_recording_before_an_environment_raises(viewer):
    with pytest.raises(NoEnvironmentError):
        viewer.record([], fps=FPS_HZ)


def test_a_phase_with_other_links_raises_and_leaves_the_live_scene(viewer, env):
    other = Environment()
    assert other.init(OTHER_ROBOT_URDF, GeneralResourceLocator())
    viewer.update_environment(env)
    traj = Trajectory.build(["other_joint"], np.array([[0.0], [0.5]]), np.array([0.0, 1.0]))

    with pytest.raises(StaleSceneError):
        viewer.record([RecordingPhase(other, traj)], fps=FPS_HZ)

    assert viewer._env is env
    # The failed recording must not stay registered: a later one still works.
    assert _decode(viewer.record([RecordingPhase(env, _ramp())], fps=FPS_HZ))["viserVersion"]
