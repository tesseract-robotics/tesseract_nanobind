"""Fetch the published demo files from their release, verified against the lock.

Recordings stay out of git. `demos.lock.json` pins each file's hash and the release holds it
under that hash, so a re-record adds an asset and an older commit still builds. Standard library
only: `docs.yml` runs this without installing the package.

`--stage DIR` is the other direction: it names the local files by hash, ready for
`gh release upload demo-assets DIR/*`.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import urllib.error
import urllib.request
from pathlib import Path

from lock import DemoLock, read_lock, sha256_bytes
from manifest import Entry, Status, load_manifest

REPO = Path(__file__).resolve().parents[2]
DEMOS_DIR = REPO / "docs/demos"
RELEASE_URL = (
    "https://github.com/tesseract-robotics/tesseract_nanobind/releases/download/demo-assets"
)
# Largest recording is ~15 MB; a stalled connection should fail the build, not hang it [s].
DOWNLOAD_TIMEOUT_S = 120.0


class AssetDownloadError(RuntimeError):
    """A pinned file is not in the release, or the download failed."""


class AssetHashError(RuntimeError):
    """A file does not have the hash the lock pins."""


def wanted(entries: list[Entry], lock: DemoLock) -> dict[str, str]:
    """What the site needs, path under `docs/demos/` to hash: the client and published entries."""
    files = {"client/index.html": lock.client_sha256}
    for entry in entries:
        if entry.status is Status.PUBLISHED:
            files[f"recordings/{entry.module}.viser"] = lock.recordings[
                entry.module
            ].recording_sha256
            files[f"posters/{entry.module}.png"] = lock.posters[entry.module]
    return files


def _has_hash(path: Path, sha256: str) -> bool:
    return path.exists() and sha256_bytes(path.read_bytes()) == sha256


def fetch_all(entries: list[Entry], lock: DemoLock, base_url: str, demos_dir: Path) -> None:
    """Download every wanted file that is not already here.

    Raises:
        AssetDownloadError: a file could not be downloaded.
        AssetHashError: a download does not match the lock; nothing is written for it.
    """
    for name, sha256 in wanted(entries, lock).items():
        target = demos_dir / name
        if _has_hash(target, sha256):
            continue
        url = f"{base_url}/{sha256}"
        try:
            with urllib.request.urlopen(url, timeout=DOWNLOAD_TIMEOUT_S) as reply:
                data = reply.read()
        except (urllib.error.URLError, TimeoutError) as error:
            raise AssetDownloadError(f"{name}: {url}: {error}") from error
        if sha256_bytes(data) != sha256:
            raise AssetHashError(f"{name}: {url} is not the file the lock pins")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)


def stage(entries: list[Entry], lock: DemoLock, demos_dir: Path, out_dir: Path) -> list[Path]:
    """Copy every wanted file into `out_dir`, named by its hash.

    Raises:
        AssetHashError: a local file does not match the lock.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    staged = []
    for name, sha256 in wanted(entries, lock).items():
        if not _has_hash(demos_dir / name, sha256):
            raise AssetHashError(f"{name}: missing, or not the file the lock pins")
        shutil.copyfile(demos_dir / name, out_dir / sha256)
        staged.append(out_dir / sha256)
    return staged


def main(argv: list[str] | None = None) -> int:
    """Fetch into `docs/demos/`, or stage for upload."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default=RELEASE_URL, help="where the assets are")
    parser.add_argument("--stage", type=Path, help="name local files by hash here; fetch nothing")
    args = parser.parse_args(argv)
    entries, lock = load_manifest(), read_lock()
    if args.stage:
        print(f"{len(stage(entries, lock, DEMOS_DIR, args.stage))} files in {args.stage}")
    else:
        fetch_all(entries, lock, args.base_url, DEMOS_DIR)
    return 0


if __name__ == "__main__":
    sys.exit(main())
