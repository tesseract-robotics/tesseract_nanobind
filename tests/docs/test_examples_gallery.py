"""The examples page lists every example, and shows a card for every published recording."""

from __future__ import annotations

import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GALLERY = (ROOT / "docs" / "examples" / "index.md").read_text("utf-8")
DEMOS = tomllib.loads((ROOT / "scripts" / "demos" / "manifest.toml").read_text("utf-8"))["example"]
SCRIPTS = tomllib.loads((ROOT / "pyproject.toml").read_text("utf-8"))["project"]["scripts"]
EXAMPLE_ENTRY = re.compile(r"^tesseract_robotics\.examples\.(?P<module>\w+):main$")


def _example_scripts() -> dict[str, str]:
    """Console script → example module, for every installed example."""
    found = {}
    for script, entry in SCRIPTS.items():
        match = EXAMPLE_ENTRY.match(entry)
        if match is not None:
            found[script] = match["module"]
    return found


def test_every_console_script_is_in_the_table():
    rows = re.findall(r"^\| \[`(\w+)`\]\([^)]+\) \| `(\w+)` \|", GALLERY, flags=re.MULTILINE)
    assert sorted((module, script) for module, script in rows) == sorted(
        (module, script) for script, module in _example_scripts().items()
    )


def test_every_published_recording_has_a_card():
    posters = set(re.findall(r"\]\(\.\./demos/posters/(\w+)\.png\)", GALLERY))
    published = {demo["module"] for demo in DEMOS if demo["status"] == "published"}
    assert posters == published
