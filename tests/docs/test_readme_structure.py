"""The README leads with the demo and the examples (#379)."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/demos"))

import manifest  # noqa: E402

README = (ROOT / "README.md").read_text("utf-8")
SITE = "https://tesseract-robotics.github.io/tesseract_nanobind/"
POSTER = f"{SITE}demos/posters/car_seat_example.png"
# A linked image: [![alt](image)](target).
LINKED_IMAGE = re.compile(r"^\[!\[[^\]]+\]\((?P<image>[^)]+)\)\]\((?P<target>[^)]+)\)$")
# More lines than this under Development means the detail has crept back: the docs own it.
MAX_DEVELOPMENT_LINES = 3


def _hero_viewer_url() -> str:
    hero = next(e for e in manifest.load_manifest() if e.module == "car_seat_example")
    assert hero.camera is not None
    return f"{SITE}demos/client/index.html?{manifest.player_query(hero.module, hero.camera)}"


def _headings() -> list[str]:
    return re.findall(r"^## (.+?)\s*$", README, flags=re.MULTILINE)


def _section(heading: str) -> list[str]:
    body = re.split(rf"^## {re.escape(heading)}\s*$", README, flags=re.MULTILINE)[1]
    return [line for line in body.split("\n## ")[0].splitlines() if line.strip()]


def test_first_content_below_the_title_is_the_poster_linking_to_the_live_viewer():
    title, first = [line for line in README.splitlines() if line.strip()][:2]
    assert title.startswith("# ")
    linked = LINKED_IMAGE.match(first)
    assert linked is not None, first
    assert linked["image"] == POSTER
    assert linked["target"] == _hero_viewer_url()


def test_examples_is_the_first_section_after_installation():
    headings = _headings()
    assert headings[headings.index("Installation") + 1] == "Examples"


def test_quickstart_stays_on_the_irb2400():
    assert 'Robot.from_tesseract_support("abb_irb2400")' in "\n".join(_section("Quick Start"))


def test_development_detail_lives_in_the_docs():
    development = _section("Development")
    assert len(development) <= MAX_DEVELOPMENT_LINES, development
    assert f"{SITE}developer/" in "\n".join(development)
