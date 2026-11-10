"""Source alignment must reject a wheel/dev mismatch before compiling dependencies."""

import importlib.util
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "check_wheel_sources", ROOT / "scripts/check_wheel_sources.py"
)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def test_current_wheel_sources_match_development():
    CHECKER.check_sources(ROOT)


def test_drift_fails_with_the_upstream_name(tmp_path):
    subprocess.run(["git", "init", "--quiet", str(tmp_path)], check=True)
    gitlinks = subprocess.run(
        ["git", "ls-files", "--stage", "upstream/"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    subprocess.run(
        ["git", "update-index", "--index-info"],
        cwd=tmp_path,
        input=gitlinks,
        text=True,
        check=True,
    )
    sources = yaml.safe_load((ROOT / "dependencies.rosinstall").read_text())
    for source in sources:
        if source["git"]["local-name"] == "tesseract_planning":
            source["git"]["version"] = "unreviewed-upstream-change"
    (tmp_path / "dependencies.rosinstall").write_text(yaml.safe_dump(sources))
    with pytest.raises(CHECKER.WheelSourceDriftError, match="tesseract_planning: wheel source"):
        CHECKER.check_sources(tmp_path)
