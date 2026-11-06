"""Check every demo file the built site references exists. `docs.yml` runs it after the build.

The docs builder checks neither `data-src` nor a recording named in a query string, so a broken
one would otherwise deploy and fail only when a reader clicks. Standard library only.
"""

from __future__ import annotations

import argparse
import sys
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import parse_qs, urljoin, urlsplit

from manifest import Entry, Status, load_manifest
from players import HERO_MODULE

REPO = Path(__file__).resolve().parents[2]
SITE_DIR = REPO / "site"
# Links are resolved as deployed, under the Pages project path: a link that climbs out of it
# works from a local server's root and 404s on Pages.
SITE_ROOT_URL = "https://pages.invalid/tesseract_nanobind/"


class MissingSiteFileError(RuntimeError):
    """The site lacks a demo file it references, or a published entry's player."""


class _Players(HTMLParser):
    """Collects each demo player's `data-src` and poster `src`."""

    def __init__(self) -> None:
        super().__init__()
        self.links: list[str] = []
        self._inside_player = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = dict(attrs)
        if tag == "button" and "demo-player" in (attributes.get("class") or ""):
            self._inside_player = True
            self.links.append(attributes["data-src"] or "")
        elif tag == "img" and self._inside_player:
            self.links.append(attributes["src"] or "")

    def handle_endtag(self, tag: str) -> None:
        if tag == "button":
            self._inside_player = False


def _targets(link: str, page_url: str) -> list[str]:
    """A link's absolute URL, and the recording's too when its query names one."""
    url = urljoin(page_url, link)
    recording = parse_qs(urlsplit(url).query).get("playbackPath", [])
    return [urlsplit(target).path for target in [url, *(urljoin(url, r) for r in recording)]]


def missing_files(site: Path, entries: list[Entry]) -> list[str]:
    """Every problem found, each naming the page and the file."""
    root_path = urlsplit(SITE_ROOT_URL).path
    problems = []
    referenced: dict[str, set[str]] = {}
    for page in sorted(site.rglob("index.html")):
        relative = page.parent.relative_to(site).as_posix()
        page_path = "" if relative == "." else f"{relative}/"
        parser = _Players()
        parser.feed(page.read_text("utf-8"))
        referenced[page_path] = set()
        for link in parser.links:
            for path in _targets(link, SITE_ROOT_URL + page_path):
                referenced[page_path].add(path)
                if not path.startswith(root_path):
                    problems.append(f"{page_path or '/'}: {link} leaves the site's path")
                elif not (site / path.removeprefix(root_path)).is_file():
                    problems.append(
                        f"{page_path or '/'}: {path.removeprefix(root_path)} is missing"
                    )
    published = [e for e in entries if e.status is Status.PUBLISHED]
    expected = [(e.page.removesuffix(".md") + "/", e.module) for e in published]
    expected += [("", e.module) for e in published if e.module == HERO_MODULE]
    for page_path, module in expected:
        recording = f"{root_path}demos/recordings/{module}.viser"
        if recording not in referenced.get(page_path, set()):
            problems.append(f"{page_path or '/'}: no player for published {module}")
    return problems


def main(argv: list[str] | None = None) -> int:
    """Check `site/`.

    Raises:
        MissingSiteFileError: anything is missing.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", type=Path, default=SITE_DIR)
    args = parser.parse_args(argv)
    problems = missing_files(args.site, load_manifest())
    if problems:
        raise MissingSiteFileError("\n".join(problems))
    return 0


if __name__ == "__main__":
    sys.exit(main())
