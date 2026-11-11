"""The demo recorder, on a scene that needs no planner."""

import hashlib
import importlib
import sys
from pathlib import Path

import numpy as np
import pytest
import viser

from tesseract_robotics import get_tesseract_resource_path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import lock  # noqa: E402
import manifest  # noqa: E402
import record  # noqa: E402
from scenes import SCENES, Scene  # noqa: E402

from tesseract_robotics.common import GeneralResourceLocator  # noqa: E402
from tesseract_robotics.environment import Environment  # noqa: E402
from tesseract_robotics.viewer import RecordingPhase, Trajectory  # noqa: E402

FPS_HZ = 10.0
CAMERA = manifest.Camera((3.0, 3.0, 2.0), (0.0, 0.0, 0.5), 0.6)
ENTRY = manifest.Entry(
    "shapes_viewer", "p.md", "T", "B", manifest.Status.EXCLUDED, "in review", CAMERA, None
)


@pytest.fixture(scope="module")
def scene() -> Scene:
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf")
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    traj = Trajectory.build(["joint_1"], np.array([[0.0], [0.5]]), np.array([0.0, 1.0]))
    return Scene(env, [RecordingPhase(env, traj)])


def test_a_recording_states_its_versions_and_inputs(scene, tmp_path):
    data = record.record_scene(ENTRY, scene, fps=FPS_HZ)
    stored = record.store(ENTRY, data, tmp_path, fps=FPS_HZ)

    assert stored.identity.viser_version == viser.__version__
    assert stored.identity.package_version == importlib.metadata.version(record.PACKAGE)
    assert stored.recording_sha256 == hashlib.sha256(data).hexdigest()
    assert stored.size_bytes == len(data)
    assert (tmp_path / "recordings/shapes_viewer.viser").read_bytes() == data


def test_the_input_hash_follows_the_settings_and_the_example():
    base = record.inputs_sha256("shapes_viewer", 6.0, FPS_HZ)
    assert base == record.inputs_sha256("shapes_viewer", 6.0, FPS_HZ)
    assert base != record.inputs_sha256("shapes_viewer", 7.0, FPS_HZ)
    assert base != record.inputs_sha256("shapes_viewer", 6.0, 2 * FPS_HZ)
    assert base != record.inputs_sha256("raster_example", 6.0, FPS_HZ)


def test_the_client_is_the_installed_vipers_own(tmp_path):
    sha256 = record.install_client(tmp_path)
    assert sha256 == hashlib.sha256(record.client_path().read_bytes()).hexdigest()
    assert (tmp_path / "client/index.html").read_bytes() == record.client_path().read_bytes()


def test_a_recording_from_another_viser_cannot_join_the_lock():
    identity = lock.BuildIdentity("0.1", "0.0.0", "abc")
    with pytest.raises(lock.ClientMismatchError, match="shapes_viewer"):
        lock.DemoLock.build(
            viser.__version__,
            "client",
            {"shapes_viewer": lock.RecordingLock(identity, "d", 1)},
            {},
        )


def test_the_lock_round_trips(tmp_path):
    identity = lock.BuildIdentity("0.1", viser.__version__, "abc")
    written = lock.DemoLock.build(
        viser.__version__, "client", {"m": lock.RecordingLock(identity, "d", 1)}, {"m": "p"}
    )
    path = tmp_path / "lock.json"
    lock.write_lock(written, path)
    assert lock.read_lock(path) == written


def test_an_unknown_entry_raises():
    with pytest.raises(record.UnknownSceneError):
        record.select([ENTRY], ["no_such_example"])


def test_an_entry_without_a_camera_raises(scene):
    no_camera = manifest.Entry(
        "shapes_viewer", "p.md", "T", "B", manifest.Status.EXCLUDED, "r", None, None
    )
    with pytest.raises(record.MissingCameraError):
        record.select([no_camera], ["shapes_viewer"])
    with pytest.raises(record.MissingCameraError):
        record.record_scene(no_camera, scene, fps=FPS_HZ)


def test_every_scene_names_a_manifest_entry():
    assert set(SCENES) <= {entry.module for entry in manifest.load_manifest()}


def test_the_hero_is_recorded_when_nothing_is_named():
    entries = manifest.load_manifest()
    assert "car_seat_example" in {entry.module for entry in record.select(entries, [])}
