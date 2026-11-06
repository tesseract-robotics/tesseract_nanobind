"""The demo manifest: one entry per viewer example, published or excluded with a reason.

Standard library only: the docs build reads this without installing the package.
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

MANIFEST_PATH = Path(__file__).with_name("manifest.toml")
# What marks an example as a viewer example: it opens the viewer.
VIEWER_MARKER = "TesseractViewer("
# The player's up axis: tesseract scenes are z-up.
CAMERA_UP = (0, 0, 1)
# Upper bound on a sensible vertical FOV [rad]: 120 deg. Rejects a value given in degrees,
# which the client would take as radians without complaint.
MAX_FOV_RAD = math.radians(120.0)


class ManifestEntryError(ValueError):
    """A manifest entry breaks a rule: a missing reason or camera, or a nonsense value."""


class Status(Enum):
    """Whether readers see the recording."""

    PUBLISHED = "published"
    EXCLUDED = "excluded"


@dataclass(frozen=True)
class Camera:
    """A reviewed view of a scene, in world coordinates [m]."""

    position_m: tuple[float, float, float]
    look_at_m: tuple[float, float, float]
    fov_rad: float

    @classmethod
    def build(cls, module: str, raw: dict[str, Any]) -> Camera:
        """Validate a manifest `camera` table.

        Raises:
            ManifestEntryError: a vector is not 3 values, or the FOV is out of range.
        """
        try:
            position, look_at, fov_rad = raw["position"], raw["look_at"], raw["fov_rad"]
        except KeyError as missing:
            raise ManifestEntryError(f"{module}: camera lacks {missing}") from missing
        if len(position) != 3 or len(look_at) != 3:
            raise ManifestEntryError(f"{module}: camera vectors need 3 values")
        if not 0.0 < fov_rad < MAX_FOV_RAD:
            raise ManifestEntryError(f"{module}: fov_rad {fov_rad} is out of range; radians?")
        return cls(tuple(position), tuple(look_at), fov_rad)


@dataclass(frozen=True)
class Entry:
    """One viewer example's demo."""

    module: str
    page: str
    title: str
    blurb: str
    status: Status
    reason: str | None
    camera: Camera | None
    floor_size_m: float | None

    @classmethod
    def build(cls, raw: dict[str, Any]) -> Entry:
        """Validate a manifest `example` table.

        Raises:
            ManifestEntryError: published without a camera, or excluded without a reason.
        """
        module = raw.get("module")
        if not module:
            raise ManifestEntryError(f"entry without a module: {raw}")
        try:
            status = Status(raw.get("status"))
            page, title, blurb = raw["page"], raw["title"], raw["blurb"]
        except (ValueError, KeyError) as bad:
            raise ManifestEntryError(f"{module}: bad or missing field: {bad}") from bad
        camera = Camera.build(module, raw["camera"]) if "camera" in raw else None
        reason = raw.get("reason")
        if status is Status.PUBLISHED and camera is None:
            raise ManifestEntryError(f"{module}: a published entry needs a reviewed camera")
        if status is Status.PUBLISHED and reason:
            raise ManifestEntryError(f"{module}: published, yet has a reason to leave it out")
        if status is Status.EXCLUDED and not reason:
            raise ManifestEntryError(f"{module}: an excluded entry needs a reason")
        return cls(module, page, title, blurb, status, reason, camera, raw.get("floor_size_m"))


def load_manifest(path: Path = MANIFEST_PATH) -> list[Entry]:
    """Read and validate every entry.

    Raises:
        ManifestEntryError: an entry breaks a rule, or two share a module.
    """
    entries = [Entry.build(raw) for raw in tomllib.loads(path.read_text("utf-8"))["example"]]
    modules = [entry.module for entry in entries]
    duplicates = sorted({module for module in modules if modules.count(module) > 1})
    if duplicates:
        raise ManifestEntryError(f"entries repeat: {duplicates}")
    return entries


def viewer_examples(examples_dir: Path) -> set[str]:
    """The top-level examples that open a viewer; `lowlevel/` is out of scope."""
    return {
        path.stem for path in examples_dir.glob("*.py") if VIEWER_MARKER in path.read_text("utf-8")
    }


def player_query(module: str, camera: Camera) -> str:
    """The static client's query string, relative to its own `client/index.html`."""

    def csv(values: tuple[float, ...]) -> str:
        return ",".join(f"{value:g}" for value in values)

    return (
        f"playbackPath=../recordings/{module}.viser&darkMode&hideViserLogo"
        f"&initialCameraPosition={csv(camera.position_m)}"
        f"&initialCameraLookAt={csv(camera.look_at_m)}"
        f"&initialCameraUp={csv(CAMERA_UP)}"
        f"&initialCameraFov={camera.fov_rad:g}"
    )
