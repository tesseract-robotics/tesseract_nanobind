"""Generate the player snippets the docs pages include. `pixi run -e docs demo-players`.

Every entry gets a file, empty until published, so a page can include one before a person
has reviewed it. Standard library only, like `manifest.py`.
"""

from __future__ import annotations

import argparse
import html
import re
import sys
from pathlib import Path

from lock import DemoLock, read_lock
from manifest import Entry, Status, load_manifest, player_query

REPO = Path(__file__).resolve().parents[2]
PLAYERS_DIR = REPO / "docs/demos/players"
HERO_MODULE = "car_seat_example"
HERO_SNIPPET = "hero.html"
# From a page's URL, /<section>/<page>/, to the site root. For `data-src`, which only the
# browser resolves.
PAGE_URL_TO_ROOT = "../../"
# From a page's source, docs/<section>/<page>.md, to docs/. For `<img src>`, which the docs
# builder rewrites for the page's URL: `../../` there comes out one level too high.
PAGE_SOURCE_TO_DOCS = "../"
# The docs home sits at the root both ways.
HOME_TO_ROOT = ""
BYTES_PER_MB = 1e6
# The label shows one decimal [MB]: below this it would say 0.0.
SMALLEST_SHOWN_MB = 0.1
_INCLUDE = re.compile(r'^--8<-- "docs/demos/players/(\w+)\.html"$', re.MULTILINE)


class MissingRecordingError(RuntimeError):
    """A published entry has no recording in the lock."""


class MissingPosterError(RuntimeError):
    """An entry with a player has no poster in the lock: it would show as an empty box."""


def size_text(size_bytes: int) -> str:
    """A download size for the player's label, which never reads as nothing."""
    if size_bytes < SMALLEST_SHOWN_MB * BYTES_PER_MB:
        return f"under {SMALLEST_SHOWN_MB} MB"
    return f"{size_bytes / BYTES_PER_MB:.1f} MB"


def _player(entry: Entry, lock: DemoLock, url_root: str, source_root: str) -> str:
    assert entry.camera is not None  # Entry.build: a published or recorded entry has one.
    recording = lock.recordings[entry.module]
    src = f"{url_root}demos/client/index.html?{player_query(entry.module, entry.camera)}"
    poster = f"{source_root}demos/posters/{entry.module}.png"
    action = "Open" if entry.static else "Play"
    label = html.escape(f"{action} {entry.title} ({size_text(recording.size_bytes)})")
    identity = recording.identity
    return (
        f'<button class="demo-player" type="button" data-src="{html.escape(src)}">'
        f'<img src="{poster}" alt="" loading="lazy">'
        f'<span class="demo-play">{label}</span></button>\n\n'
        f"`{entry.module}`, tesseract_robotics {identity.package_version}\n"
    )


def snippets(entries: list[Entry], lock: DemoLock, *, preview: bool = False) -> dict[str, str]:
    """Every snippet's file name and content.

    Args:
        entries: The manifest.
        lock: What is recorded.
        preview: Also give a player to excluded entries with a recording and a camera, so a
            reviewer can see them in place.

    Raises:
        MissingRecordingError: a published entry has no recording.
        MissingPosterError: an entry with a player has no poster.
    """

    def plays(entry: Entry) -> bool:
        if entry.status is Status.PUBLISHED:
            return True
        return preview and entry.camera is not None and entry.module in lock.recordings

    missing = sorted(
        e.module
        for e in entries
        if e.status is Status.PUBLISHED and e.module not in lock.recordings
    )
    if missing:
        raise MissingRecordingError(f"published without a recording: {missing}")
    unposed = sorted(e.module for e in entries if plays(e) and e.module not in lock.posters)
    if unposed:
        raise MissingPosterError(f"a player without a poster: {unposed}")

    files = {
        f"{e.module}.html": _player(e, lock, PAGE_URL_TO_ROOT, PAGE_SOURCE_TO_DOCS)
        if plays(e)
        else ""
        for e in entries
    }
    hero = next((e for e in entries if e.module == HERO_MODULE and plays(e)), None)
    files[HERO_SNIPPET] = _player(hero, lock, HOME_TO_ROOT, HOME_TO_ROOT) if hero else ""
    return files


def included_modules(page_text: str) -> list[str]:
    """The player snippets a page includes, by module."""
    return _INCLUDE.findall(page_text)


def main(argv: list[str] | None = None) -> int:
    """Write the snippets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", action="store_true", help="include unreviewed recordings")
    args = parser.parse_args(argv)
    PLAYERS_DIR.mkdir(parents=True, exist_ok=True)
    for name, text in snippets(load_manifest(), read_lock(), preview=args.preview).items():
        (PLAYERS_DIR / name).write_text(text, "utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
