"""Content-addressed cache of the facts libclang extracts from one translation unit.

The audit's cost is libclang: parsing each binding (twice, once with function bodies) and
walking the whole TU from Python. The facts it keeps from a TU (bound classes, declaration
names, class bases, GIL releases) are plain data, a pure function of the TU's inputs. This
cache stores each fact as one JSON entry under `FactKey.digest()`: the toolchain that made
it (`FactKey.toolchain`, an `audit_toolchain.Toolchain`: parse arguments, environment,
extractor sources) plus the content hash of every file the parse read. An entry is a hit only when every one of those files is
byte-identical, so an edit anywhere a binding's headers reach re-parses that binding and no
other. The rules that turn facts into report rows always run; only extraction is cached.

`fetch` holds a lock per entry: xdist workers asking for one fact at once compute it once.
CI restores the cache directory between runs and prunes what a run did not use (`prune`).
Each answered fetch is one event of `audit_telemetry.EventLog`: a hit or a miss, the time
`compute` took and the time spent waiting for the lock.

`scripts/audit_cache_prune.py` is the prune CLI (`pixi run -e audit audit-cache-prune`).
"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import tempfile
import time
from collections.abc import Callable, Generator, Iterable
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path

from audit_telemetry import EventLog
from audit_toolchain import Sha256, Toolchain

type Json = None | bool | int | float | str | list[Json] | dict[str, Json]
# Layout of an entry; part of every digest, so a layout change starts a fresh cache.
ENTRY_FORMAT = 2
ENTRY_SUFFIX = ".json"
LOCK_SUFFIX = ".lock"
# Read size for hashing a dependency, bytes.
HASH_CHUNK = 1 << 20


class CacheError(RuntimeError):
    """The fact cache cannot answer."""


class MissingDependencyError(CacheError):
    """A fact names a file it read that does not exist: the parse's file list is wrong."""


class CorruptEntryError(CacheError):
    """An entry on disk is not one `fetch` wrote (entries are replaced atomically)."""


class EventsInCacheError(CacheError):
    """The event log lies under the cache root: CI would save it and restore it into later runs."""


@dataclass(frozen=True)
class FactKey:
    """What a fact is, apart from the files its parse read.

    Attributes:
        kind: The fact (`decl`, `bound`, `gil`): one extractor's output.
        source: The TU's main file.
        toolchain: Everything else the extractor's output depends on.
    """

    kind: str
    source: Path
    toolchain: Toolchain

    def digest(self) -> str:
        """The entry's file name: stable across processes and machines."""
        text = json.dumps([ENTRY_FORMAT, self.kind, str(self.source), self.toolchain.canonical()])
        return hashlib.sha256(text.encode()).hexdigest()


def sha256_of(path: Path) -> Sha256:
    """Content hash of one file."""
    h = hashlib.sha256()
    with path.open("rb") as f:
        while chunk := f.read(HASH_CHUNK):
            h.update(chunk)
    return Sha256(h.hexdigest())


@contextmanager
def _locked(path: Path) -> Generator[None]:
    with path.open("a") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


@dataclass
class FactCache:
    """Facts under `root/<kind>/<digest>.json`.

    Attributes:
        root: The cache directory; created on first store.
        events: Where each answered fetch is recorded; `BINDING_AUDIT_EVENTS` by default.

    Raises:
        EventsInCacheError: the event log is under `root`.
    """

    root: Path
    events: EventLog = field(default_factory=EventLog.from_env)
    # (path, inode, size, mtime, ctime) -> hash, so a process hashes each header once. ctime
    # moves on every write and cannot be set back, unlike mtime.
    _hashes: dict[tuple[str, int, int, int, int], Sha256] = field(default_factory=dict)

    def __post_init__(self) -> None:
        log = self.events.path
        if log is not None and log.resolve().is_relative_to(self.root.resolve()):
            raise EventsInCacheError(f"{log} is inside the cache directory {self.root}")

    def entry_path(self, key: FactKey) -> Path:
        return self.root / key.kind / f"{key.digest()}{ENTRY_SUFFIX}"

    def fetch(self, key: FactKey, compute: Callable[[], tuple[object, Iterable[Path]]]) -> Json:
        """The fact for `key`: stored if every file its parse read is unchanged, else computed.

        Args:
            key: The fact and its toolchain.
            compute: Parses and extracts; returns the fact (anything `json.dumps` writes) and
                every file the parse read.

        Returns:
            The fact as JSON reads it back (tuples become lists), on a hit and a miss alike.

        Raises:
            MissingDependencyError: `compute` names a file that does not exist.
            TypeError: the fact is not JSON-serialisable (`json.dumps`).
            CorruptEntryError: the stored entry is not valid.
        """
        entry = self.entry_path(key)
        entry.parent.mkdir(parents=True, exist_ok=True)
        asked = time.perf_counter()
        with _locked(entry.with_suffix(LOCK_SUFFIX)):
            wait_s = time.perf_counter() - asked
            if entry.exists():
                deps_then, stored = self._read(entry)
                if self._current(deps_then):
                    os.utime(entry)  # used by this run: `prune` keeps it
                    self.events.fetched(key.kind, hit=True, compute_s=0.0, wait_s=wait_s)
                    return stored
            started = time.perf_counter()
            value, deps = compute()
            compute_s = time.perf_counter() - started
            record = {"deps": self._hash_all(deps), "value": value}
            text = json.dumps(record, sort_keys=True)
            self._write(entry, text)
            self.events.fetched(key.kind, hit=False, compute_s=compute_s, wait_s=wait_s)
            return json.loads(text)["value"]

    def prune(self, unused_since_ns: int) -> list[Path]:
        """Remove the entries last used before `unused_since_ns` (an mtime); returns them."""
        stale = sorted(
            e
            for e in self.root.glob(f"*/*{ENTRY_SUFFIX}")
            if e.stat().st_mtime_ns < unused_since_ns
        )
        for e in stale:
            e.unlink()
            e.with_suffix(LOCK_SUFFIX).unlink(missing_ok=True)
        return stale

    def _hash(self, path: Path) -> Sha256:
        st = path.stat()
        stat_key = (str(path), st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)
        if stat_key not in self._hashes:
            self._hashes[stat_key] = sha256_of(path)
        return self._hashes[stat_key]

    def _hash_all(self, deps: Iterable[Path]) -> dict[str, str]:
        out: dict[str, str] = {}
        for p in sorted(set(deps)):
            if not p.is_file():
                raise MissingDependencyError(f"{p}: read by the parse, but not a file")
            out[str(p)] = self._hash(p)
        return out

    def _current(self, deps: dict[str, str]) -> bool:
        return all(Path(p).is_file() and self._hash(Path(p)) == h for p, h in deps.items())

    @staticmethod
    def _read(entry: Path) -> tuple[dict[str, str], Json]:
        """The stored dependency hashes and fact.

        Raises:
            CorruptEntryError: not JSON, or not the shape `fetch` writes.
        """
        try:
            record = json.loads(entry.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            raise CorruptEntryError(f"{entry}: {e}") from e
        if (
            not isinstance(record, dict)
            or set(record) != {"deps", "value"}
            or not isinstance(record["deps"], dict)
            or not all(isinstance(h, str) for h in record["deps"].values())
        ):
            raise CorruptEntryError(f"{entry}: not a fact entry")
        return record["deps"], record["value"]

    @staticmethod
    def _write(entry: Path, text: str) -> None:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=entry.parent, suffix=".tmp", delete=False
        ) as f:
            f.write(text)
        Path(f.name).replace(entry)
