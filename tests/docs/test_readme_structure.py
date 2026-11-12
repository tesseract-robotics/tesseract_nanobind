"""The README leads with the demo and the examples (#379)."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
README = (ROOT / "README.md").read_text("utf-8")
SITE = "https://tesseract-robotics.github.io/tesseract_nanobind/"
POSTER = f"{SITE}demos/posters/car_seat_example.png"
# The docs home, whose first player `demos.js` starts on `#play`: the docs' player, not the
# bare full-window client.
POSTER_TARGET = f"{SITE}#play"
# A linked image: [![alt](image)](target).
LINKED_IMAGE = re.compile(r"^\[!\[[^\]]+\]\((?P<image>[^)]+)\)\]\((?P<target>[^)]+)\)$")
# More lines than this under Development means the detail has crept back: the docs own it.
MAX_DEVELOPMENT_LINES = 3


def _headings() -> list[str]:
    return re.findall(r"^## (.+?)\s*$", README, flags=re.MULTILINE)


def _section(heading: str) -> list[str]:
    body = re.split(rf"^## {re.escape(heading)}\s*$", README, flags=re.MULTILINE)[1]
    return [line for line in body.split("\n## ")[0].splitlines() if line.strip()]


def test_first_content_below_the_title_is_the_poster_linking_to_the_docs_player():
    title, first = [line for line in README.splitlines() if line.strip()][:2]
    assert title.startswith("# ")
    linked = LINKED_IMAGE.match(first)
    assert linked is not None, first
    assert linked["image"] == POSTER
    assert linked["target"] == POSTER_TARGET


def test_examples_is_the_first_section_after_installation():
    headings = _headings()
    assert headings[headings.index("Installation") + 1] == "Examples"


def test_quickstart_stays_on_the_irb2400():
    assert 'Robot.from_tesseract_support("abb_irb2400", locator)' in "\n".join(
        _section("Quick Start")
    )


def test_development_detail_lives_in_the_docs():
    development = _section("Development")
    assert len(development) <= MAX_DEVELOPMENT_LINES, development
    assert f"{SITE}developer/" in "\n".join(development)


def _first_python_block(text: str) -> str:
    match = re.search(r"^```python\n(?P<code>.*?)^```$", text, flags=re.MULTILINE | re.DOTALL)
    assert match is not None
    return match["code"]


def test_docs_home_shows_the_readme_quick_start_verbatim():
    """The docs home skips its copy of the block; this keeps it the block the README runs."""
    home = (ROOT / "docs" / "index.md").read_text("utf-8")
    quick_start = README.split("\n## Quick Start\n", 1)[1]
    assert _first_python_block(home) == _first_python_block(quick_start)


# A markdown link or image target: `](target)`.
LINK_TARGET = re.compile(r"\]\((?P<target>[^)\s]+)\)")


def test_every_link_resolves_on_the_pypi_project_page():
    """The wheel's README is the PyPI project page, where a repository-relative target is dead."""
    relative = [t for t in LINK_TARGET.findall(README) if not t.startswith("https://")]
    assert relative == []
