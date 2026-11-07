"""Record demo entries into `docs/demos/` and update the lock. `pixi run demo-record`."""

from __future__ import annotations

import argparse
import importlib.metadata
import shutil
import sys
from pathlib import Path

import viser
from lock import (
    LOCK_PATH,
    BuildIdentity,
    DemoLock,
    RecordingLock,
    read_lock,
    sha256_bytes,
    write_lock,
)
from manifest import Entry, load_manifest
from scenes import SCENES, Scene

import tesseract_robotics.examples
from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.viewer import TesseractViewer
from tesseract_robotics.viewer.floor import (
    FLOOR_CELL_M,
    FLOOR_SECTION_M,
    FLOOR_SHADOW_OPACITY,
    MIN_FLOOR_SIZE_M,
)
from tesseract_robotics.viewer.viewer import PLOT_COLOR_RGB, RECORD_FPS_HZ

REPO = Path(__file__).resolve().parents[2]
DEMOS_DIR = REPO / "docs/demos"
PACKAGE = "tesseract-robotics-nanobind"
EXAMPLES_DIR = Path(tesseract_robotics.examples.__file__).parent
# The floor when an entry names no size [m]: the viewer's smallest.
DEFAULT_FLOOR_SIZE_M = MIN_FLOOR_SIZE_M
# Waypoint triad axis length [m]: small, so a dense toolpath's triads don't merge.
PATH_AXES_LENGTH_M = 0.03


class UnknownSceneError(KeyError):
    """A requested entry is not in the manifest, or has no scene to record."""


class MissingCameraError(ValueError):
    """A requested entry has no camera, so nobody could review its recording."""


def client_path() -> Path:
    """The static client inside the installed viser."""
    return Path(viser.__file__).parent / "client/build/index.html"


def inputs_sha256(module: str, floor_size_m: float, fps: float) -> str:
    """Hash what decides a recording's content: the example, the scenes, and the settings."""
    sources = b"".join(
        path.read_bytes()
        for path in (EXAMPLES_DIR / f"{module}.py", Path(__file__).with_name("scenes.py"))
    )
    settings = f"{floor_size_m}|{fps}|{viser.__version__}|{importlib.metadata.version(PACKAGE)}"
    return sha256_bytes(sources + settings.encode())


def record_scene(entry: Entry, scene: Scene, *, fps: float = RECORD_FPS_HZ) -> bytes:
    """Stage `scene` with a floor and its paths, and record it.

    Raises:
        MissingCameraError: `entry` has no camera.
    """
    if entry.camera is None:
        raise MissingCameraError(f"{entry.module} has no camera to review it through")
    floor_size_m = entry.floor_size_m or DEFAULT_FLOOR_SIZE_M
    with TesseractViewer(port=0) as viewer:
        viewer.update_environment(scene.env)
        # Replaces the viewer's own floor: the entry's size, centred where its camera looks.
        viewer.server.scene.add_grid(
            "/floor",
            width=floor_size_m,
            height=floor_size_m,
            plane="xy",
            cell_size=FLOOR_CELL_M,
            section_size=FLOOR_SECTION_M,
            position=(entry.camera.look_at_m[0], entry.camera.look_at_m[1], 0.0),
            shadow_opacity=FLOOR_SHADOW_OPACITY,
        )
        for program in scene.plots:
            viewer.plot_trajectory(
                program,
                program.getManipulatorInfo(),
                axes_length=PATH_AXES_LENGTH_M,
                color=PLOT_COLOR_RGB,
            )
        return viewer.record(scene.phases, fps=fps)


def store(entry: Entry, data: bytes, demos_dir: Path, *, fps: float) -> RecordingLock:
    """Write one recording under `demos_dir`, and state what it is."""
    recordings_dir = demos_dir / "recordings"
    recordings_dir.mkdir(parents=True, exist_ok=True)
    (recordings_dir / f"{entry.module}.viser").write_bytes(data)
    floor_size_m = entry.floor_size_m or DEFAULT_FLOOR_SIZE_M
    identity = BuildIdentity(
        package_version=importlib.metadata.version(PACKAGE),
        viser_version=viser.__version__,
        inputs_sha256=inputs_sha256(entry.module, floor_size_m, fps),
    )
    return RecordingLock(identity, sha256_bytes(data), len(data))


def install_client(demos_dir: Path) -> str:
    """Copy the installed viser's client under `demos_dir`; returns its hash."""
    target = demos_dir / "client/index.html"
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(client_path(), target)
    return sha256_bytes(target.read_bytes())


def select(entries: list[Entry], only: list[str]) -> list[Entry]:
    """The entries to record: every one with a camera, or just the named ones.

    Raises:
        UnknownSceneError: a named entry is not in the manifest, or has no scene.
        MissingCameraError: a named entry has no camera.
    """
    by_module = {entry.module: entry for entry in entries}
    for module in only:
        if module not in by_module or module not in SCENES:
            raise UnknownSceneError(f"nothing to record for {module!r}")
        if by_module[module].camera is None:
            raise MissingCameraError(f"{module} has no camera to review it through")
    if only:
        return [by_module[module] for module in only]
    return [entry for entry in entries if entry.camera is not None]


def main(argv: list[str] | None = None) -> int:
    """Record, then write the lock only if every recording plays in the client just copied."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="*", default=[], help="entries to record; default all")
    args = parser.parse_args(argv)
    selected = select(load_manifest(), args.only)
    # Partial runs keep what's there, and DemoLock.build refuses a mixed viser.
    previous = read_lock() if args.only and LOCK_PATH.exists() else None

    # One composer: every from_config() reloads the plugins (~4.3 s).
    composer = TaskComposer.from_config()
    recordings = dict(previous.recordings) if previous else {}
    posters = dict(previous.posters) if previous else {}
    for entry in selected:
        data = record_scene(entry, SCENES[entry.module](composer))
        recordings[entry.module] = store(entry, data, DEMOS_DIR, fps=RECORD_FPS_HZ)
        posters.pop(entry.module, None)
        print(f"{entry.module}: {len(data) / 1e6:.1f} MB")
    client_sha256 = install_client(DEMOS_DIR)
    write_lock(DemoLock.build(viser.__version__, client_sha256, recordings, posters))
    return 0


if __name__ == "__main__":
    sys.exit(main())
