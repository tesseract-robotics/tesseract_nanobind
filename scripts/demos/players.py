"""Generate the player snippets the docs pages include. `pixi run -e docs demo-players`.

Every entry gets a file, empty until published, so a page can include one before a person
has reviewed it. Standard library only, like `manifest.py`.
"""

from __future__ import annotations

import argparse
import html
import re
import sys
from collections.abc import Callable
from pathlib import Path

from annotate import warn
from lock import DemoLock, read_lock
from manifest import Entry, Status, load_manifest, player_query

REPO = Path(__file__).resolve().parents[2]
DEMOS_DIR = REPO / "docs/demos"
PLAYERS_DIR = DEMOS_DIR / "players"
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


def size_text(size_bytes: int) -> str:
    """A download size for the player's label, which never reads as nothing."""
    if size_bytes < SMALLEST_SHOWN_MB * BYTES_PER_MB:
        return f"under {SMALLEST_SHOWN_MB} MB"
    return f"{size_bytes / BYTES_PER_MB:.1f} MB"


def _player(
    entry: Entry, lock: DemoLock, url_root: str, source_root: str, with_poster: bool
) -> str:
    assert entry.camera is not None  # Entry.build: a published or recorded entry has one.
    recording = lock.recordings[entry.module]
    src = f"{url_root}demos/client/index.html?{player_query(entry.module, entry.camera)}"
    poster = f"{source_root}demos/posters/{entry.module}.png"
    action = "Open" if entry.static else "Play"
    label = html.escape(f"{action} {entry.title} ({size_text(recording.size_bytes)})")
    identity = recording.identity
    return (
        f'<button class="demo-player" type="button" data-src="{html.escape(src)}">'
        + (f'<img src="{poster}" alt="" loading="lazy">' if with_poster else "")
        + f'<span class="demo-play">{label}</span></button>\n\n'
        f"`{entry.module}`, tesseract_robotics {identity.package_version}\n"
    )


def _always_present(_: str) -> bool:
    return True


def snippets(
    entries: list[Entry],
    lock: DemoLock,
    *,
    preview: bool = False,
    present: Callable[[str], bool] = _always_present,
) -> dict[str, str]:
    """Every snippet's file name and content.

    A published entry whose recording is not in the lock, or not present, gets an empty snippet;
    one without a poster plays without it. Both are warned about, not failed (#446).

    Args:
        entries: The manifest.
        lock: What is recorded.
        preview: Also give a player to excluded entries with a recording and a camera, so a
            reviewer can see them in place.
        present: Whether a file under `docs/demos/` (`recordings/x.viser`, `posters/x.png`) is
            there to serve; `main` checks the disk after `fetch.py`.
    """

    def recorded(entry: Entry) -> bool:
        path = f"recordings/{entry.module}.viser"
        return entry.module in lock.recordings and present(path)

    def plays(entry: Entry) -> bool:
        if entry.status is Status.PUBLISHED:
            return recorded(entry)
        return preview and entry.camera is not None and recorded(entry)

    def poster(entry: Entry) -> bool:
        return entry.module in lock.posters and present(f"posters/{entry.module}.png")

    for e in entries:
        if e.status is Status.PUBLISHED and not plays(e):
            warn(f"recordings/{e.module}.viser: missing; {e.page} builds without its player")
        elif plays(e) and not poster(e):
            warn(f"posters/{e.module}.png: missing; the {e.module} player has no poster")

    files = {
        f"{e.module}.html": _player(e, lock, PAGE_URL_TO_ROOT, PAGE_SOURCE_TO_DOCS, poster(e))
        if plays(e)
        else ""
        for e in entries
    }
    hero = next((e for e in entries if e.module == HERO_MODULE and plays(e)), None)
    files[HERO_SNIPPET] = (
        _player(hero, lock, HOME_TO_ROOT, HOME_TO_ROOT, poster(hero)) if hero else ""
    )
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
    on_disk = snippets(
        load_manifest(),
        read_lock(),
        preview=args.preview,
        present=lambda path: (DEMOS_DIR / path).is_file(),
    )
    for name, text in on_disk.items():
        (PLAYERS_DIR / name).write_text(text, "utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
