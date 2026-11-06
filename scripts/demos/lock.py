"""The demo lock file: what each recording was made from, and the hash of what came out.

Committed, while the recordings stay out of git. Standard library only, like `manifest.py`.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path

LOCK_PATH = Path(__file__).with_name("demos.lock.json")
# Bumped when the lock's layout changes.
LOCK_VERSION = 1


class ClientMismatchError(RuntimeError):
    """A recording was made with another viser version than the client it would play in."""


@dataclass(frozen=True)
class BuildIdentity:
    """What a recording states about its origin.

    Planners are randomized, so equal inputs can give different bytes: this identifies the
    inputs, and `RecordingLock.recording_sha256` the output.
    """

    package_version: str
    viser_version: str
    inputs_sha256: str


@dataclass(frozen=True)
class RecordingLock:
    """One recording in the lock."""

    identity: BuildIdentity
    recording_sha256: str
    size_bytes: int


@dataclass(frozen=True)
class DemoLock:
    """The embedded client and every recording."""

    viser_version: str
    client_sha256: str
    recordings: dict[str, RecordingLock]
    # Module to the hash of its poster. A re-record drops the entry: the poster no longer matches.
    posters: dict[str, str]

    @classmethod
    def build(
        cls,
        viser_version: str,
        client_sha256: str,
        recordings: dict[str, RecordingLock],
        posters: dict[str, str],
    ) -> DemoLock:
        """Validate that every recording plays in this client.

        Raises:
            ClientMismatchError: a recording names another viser version.
        """
        stale = sorted(
            module
            for module, recording in recordings.items()
            if recording.identity.viser_version != viser_version
        )
        if stale:
            raise ClientMismatchError(
                f"made with another viser than {viser_version}: {stale}; re-record them"
            )
        return cls(viser_version, client_sha256, recordings, posters)


def sha256_bytes(data: bytes) -> str:
    """Hex SHA-256 of `data`."""
    return hashlib.sha256(data).hexdigest()


def read_lock(path: Path = LOCK_PATH) -> DemoLock:
    """Read and validate the lock.

    Raises:
        ClientMismatchError: a recording names another viser version than the client.
    """
    raw = json.loads(path.read_text("utf-8"))
    recordings = {
        module: RecordingLock(
            BuildIdentity(**entry["identity"]), entry["recording_sha256"], entry["size_bytes"]
        )
        for module, entry in raw["recordings"].items()
    }
    return DemoLock.build(raw["viser_version"], raw["client_sha256"], recordings, raw["posters"])


def write_lock(lock: DemoLock, path: Path = LOCK_PATH) -> None:
    """Write the lock with sorted keys, so a re-record diffs by recording."""
    document = {"lock_version": LOCK_VERSION, **asdict(lock)}
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", "utf-8")
