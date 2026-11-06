"""Generate the docs gallery page from the manifest and the lock. `pixi run gallery-page`.

Standard library only, like `manifest.py`.
"""

from __future__ import annotations

import argparse
import html
import sys
from pathlib import Path

from lock import GalleryLock, read_lock
from manifest import Entry, Status, load_manifest, player_query

REPO = Path(__file__).resolve().parents[2]
PAGE_PATH = REPO / "docs/examples/gallery.md"
# From the page's URL, /examples/gallery/, to the site root.
PAGE_TO_ROOT = "../../"
BYTES_PER_MB = 1e6
INTRO = """\
# Gallery

Real planned trajectories, recorded from the examples. They play in your browser from static
files, with no Python and no server. Click one to load it, then drag to orbit, and use the
timeline to scrub.
"""
PREVIEW_NOTE = """\
!!! warning "Local preview"
    This page includes recordings still awaiting review. It must not be committed.
"""


class MissingRecordingError(RuntimeError):
    """A published entry has no recording in the lock."""


def _player(entry: Entry, lock: GalleryLock) -> str:
    assert entry.camera is not None  # Entry.build: a published or recorded entry has one.
    recording = lock.recordings[entry.module]
    src = f"{PAGE_TO_ROOT}demos/client/index.html?{player_query(entry.module, entry.camera)}"
    label = html.escape(f"Play {entry.title} ({recording.size_bytes / BYTES_PER_MB:.1f} MB)")
    identity = recording.identity
    return (
        f"## {entry.title}\n\n{entry.blurb}\n\n"
        f'<button class="gallery-player" type="button" data-src="{html.escape(src)}">'
        f"{label}</button>\n\n"
        f"`{entry.module}`, tesseract_robotics {identity.package_version}, "
        f"viser {identity.viser_version}\n"
    )


def render(entries: list[Entry], lock: GalleryLock, *, preview: bool = False) -> str:
    """The page's markdown.

    Args:
        entries: The manifest.
        lock: What is recorded.
        preview: Also give a player to excluded entries that have a recording and a camera,
            so a reviewer can see them. Never for a committed page.

    Raises:
        MissingRecordingError: a published entry has no recording.
    """

    def plays(entry: Entry) -> bool:
        if entry.status is Status.PUBLISHED:
            return True
        return preview and entry.camera is not None and entry.module in lock.recordings

    missing = sorted(
        entry.module
        for entry in entries
        if entry.status is Status.PUBLISHED and entry.module not in lock.recordings
    )
    if missing:
        raise MissingRecordingError(f"published without a recording: {missing}")

    parts = [INTRO]
    if preview:
        parts.append(PREVIEW_NOTE)
    parts += [_player(entry, lock) for entry in entries if plays(entry)]
    rest = [entry for entry in entries if not plays(entry)]
    in_preview = [entry for entry in entries if plays(entry) and entry.status is Status.EXCLUDED]
    if in_preview:
        rows = "\n".join(f"- **{entry.title}**: {entry.reason}" for entry in in_preview)
        parts.append(f"## Awaiting review\n\n{rows}\n")
    if rest:
        rows = "\n".join(f"| {entry.title} | {entry.blurb} | {entry.reason} |" for entry in rest)
        parts.append(
            f"## Not in the gallery yet\n\n| Example | Shows | Why not |\n|---|---|---|\n{rows}\n"
        )
    return "\n".join(parts)


def main(argv: list[str] | None = None) -> int:
    """Write the page."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", action="store_true", help="include unreviewed recordings")
    args = parser.parse_args(argv)
    PAGE_PATH.write_text(render(load_manifest(), read_lock(), preview=args.preview), "utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
