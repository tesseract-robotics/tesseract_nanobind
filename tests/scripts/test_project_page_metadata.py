"""The PyPI project page's sidebar is `[project]` in pyproject.toml: platforms and links."""

import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
PROJECT = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text("utf-8"))["project"]
REPOSITORY = "https://github.com/tesseract-robotics/tesseract_nanobind"
# One classifier per operating system `.github/workflows/wheels.yml` publishes a wheel for.
WHEEL_OPERATING_SYSTEMS = {
    "Operating System :: POSIX :: Linux",
    "Operating System :: MacOS",
    "Operating System :: Microsoft :: Windows",
}
PROJECT_LINKS = {
    "Homepage": REPOSITORY,
    "Repository": REPOSITORY,
    "Documentation": "https://tesseract-robotics.github.io/tesseract_nanobind/",
    "Changelog": f"{REPOSITORY}/blob/main/CHANGELOG.md",
    "Issues": f"{REPOSITORY}/issues",
}


def test_classifiers_name_each_operating_system_a_wheel_ships_for() -> None:
    declared = {c for c in PROJECT["classifiers"] if c.startswith("Operating System ::")}
    assert declared == WHEEL_OPERATING_SYSTEMS


def test_project_page_links_docs_changelog_and_issues() -> None:
    assert PROJECT["urls"] == PROJECT_LINKS


def test_stubs_are_declared_typed() -> None:
    assert (REPO_ROOT / "src" / "tesseract_robotics" / "py.typed").is_file()
    assert "Typing :: Typed" in PROJECT["classifiers"]
