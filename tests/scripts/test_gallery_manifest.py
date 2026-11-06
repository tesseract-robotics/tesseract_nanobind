"""The gallery manifest covers every viewer example, and its entries obey their rules."""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
EXAMPLES_DIR = REPO / "src/tesseract_robotics/examples"
sys.path.insert(0, str(REPO / "scripts/gallery"))

import manifest  # noqa: E402

# Issue #379's count; a new viewer example must land in the manifest and raise this on purpose.
VIEWER_EXAMPLE_COUNT = 19


HERO_QUERY = (
    "playbackPath=../recordings/car_seat_example.viser&darkMode&hideViserLogo"
    "&initialCameraPosition=-3.16,6.85,4.77&initialCameraLookAt=0.81,-0.04,0.55"
    "&initialCameraUp=0,0,1&initialCameraFov=0.6109"
)
CAMERA = {"position": [1.0, 2.0, 3.0], "look_at": [0.0, 0.0, 0.0], "fov_rad": 0.6}
PUBLISHED = {
    "module": "m",
    "title": "T",
    "blurb": "B",
    "status": "published",
    "camera": CAMERA,
}
EXCLUDED = {"module": "m", "title": "T", "blurb": "B", "status": "excluded", "reason": "R"}


def test_every_viewer_example_has_exactly_one_entry():
    examples = manifest.viewer_examples(EXAMPLES_DIR)
    assert len(examples) == VIEWER_EXAMPLE_COUNT
    assert {entry.module for entry in manifest.load_manifest()} == examples


def test_a_new_viewer_example_is_noticed(tmp_path):
    (tmp_path / "new_viewer.py").write_text("with TesseractViewer() as viewer:\n    pass\n")
    (tmp_path / "not_a_viewer.py").write_text("print('no')\n")
    assert manifest.viewer_examples(tmp_path) == {"new_viewer"}


def test_the_hero_keeps_the_camera_jelle_picked():
    entries = {entry.module: entry for entry in manifest.load_manifest()}
    hero = entries["car_seat_example"]
    assert manifest.player_query(hero.module, hero.camera) == HERO_QUERY


def test_a_published_entry_needs_a_camera():
    raw = {key: value for key, value in PUBLISHED.items() if key != "camera"}
    with pytest.raises(manifest.ManifestEntryError, match="camera"):
        manifest.Entry.build(raw)


def test_a_published_entry_cannot_keep_a_reason_to_leave_it_out():
    with pytest.raises(manifest.ManifestEntryError, match="reason"):
        manifest.Entry.build({**PUBLISHED, "reason": "looks wrong"})


def test_an_excluded_entry_needs_a_reason():
    raw = {key: value for key, value in EXCLUDED.items() if key != "reason"}
    with pytest.raises(manifest.ManifestEntryError, match="reason"):
        manifest.Entry.build(raw)


def test_a_field_of_view_in_degrees_is_rejected():
    camera = {**CAMERA, "fov_rad": 35.0}
    with pytest.raises(manifest.ManifestEntryError, match="radians"):
        manifest.Entry.build({**PUBLISHED, "camera": camera})


def test_an_unknown_status_is_rejected():
    with pytest.raises(manifest.ManifestEntryError):
        manifest.Entry.build({**EXCLUDED, "status": "maybe"})


def test_repeated_entries_are_rejected(tmp_path):
    table = '[[example]]\nmodule="m"\ntitle="T"\nblurb="B"\nstatus="excluded"\nreason="R"\n'
    path = tmp_path / "manifest.toml"
    path.write_text(table + table)
    with pytest.raises(manifest.ManifestEntryError, match="repeat"):
        manifest.load_manifest(path)
