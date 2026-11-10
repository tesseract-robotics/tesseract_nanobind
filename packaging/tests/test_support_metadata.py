"""Exercise the build backend's metadata resolver without compiling C++."""

import tomllib
from pathlib import Path

import pytest
from scikit_build_core.builder._load_provider import process_dynamic_metadata
from scikit_build_core.settings.skbuild_read_settings import SettingsReader

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("version", ["0.36.0.7", "0.36.0.8.dev12"])
def test_support_extra_matches_resolved_binding_version(monkeypatch, version):
    monkeypatch.chdir(ROOT)
    monkeypatch.setenv("SETUPTOOLS_SCM_PRETEND_VERSION_FOR_TESSERACT_ROBOTICS_NANOBIND", version)
    SettingsReader.from_file("pyproject.toml", {}).validate_may_exit()
    config = tomllib.loads((ROOT / "pyproject.toml").read_text())
    metadata = process_dynamic_metadata(
        config["project"], config["tool"]["scikit-build"]["metadata"]
    )
    assert metadata["version"] == version
    assert metadata["optional-dependencies"] == {
        "support": [f"tesseract-robotics-nanobind-support=={version}"]
    }
