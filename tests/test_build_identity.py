"""`build_identity` and `dependencies.rosinstall` name the same upstream commits."""

from pathlib import Path

import yaml

from tesseract_robotics.build_identity import UPSTREAM_SOURCES

ROSINSTALL = Path(__file__).resolve().parents[1] / "dependencies.rosinstall"


def test_sources_match_rosinstall():
    entries = {e["git"]["local-name"]: e["git"] for e in yaml.safe_load(ROSINSTALL.read_text())}
    for source in UPSTREAM_SOURCES:
        entry = entries[source.repository.split("/")[1]]
        assert entry["uri"] == f"https://github.com/{source.fork}.git"
        assert entry["version"] == source.sha
