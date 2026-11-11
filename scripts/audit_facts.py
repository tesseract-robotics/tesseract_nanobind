"""The facts the audit reads from a binding TU, cached on the TU's inputs (`audit_cache`).

Three facts, each one extractor's output on one parse:

- `decl_facts`: declaration names, class ancestors and public bases of the declarations-only
  parse (`PARSE_OPTIONS`), three walks of the whole TU from Python;
- `bound_facts`: the `nb::class_` / `nb::enum_` / `nb::exception` constructions of the
  bodies parse (`BODY_PARSE_OPTIONS`);
- `gil_releases`: `audit_gil.Scanner.scan` of the GIL audit's own bodies parse.

A fact's `FactKey.toolchain` holds whatever the parse reads besides the files it records:
the parse arguments and options, the environment's package set (libclang, every header
package, nanobind), the header-search variables libclang reads, the file listing of each
include dir in the repo (a new file there can shadow a header found later on the search
path), and the source of every script that runs while the fact is extracted.
"""

from __future__ import annotations

import functools
import hashlib
import json
import os
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import audit_types
import clang.cindex as ci
from audit_bound import bound_classes, bound_exceptions
from audit_cache import CacheError, FactCache, FactKey, Json, sha256_of
from audit_ownership import REPO_ROOT
from audit_parse import (
    BODY_PARSE_OPTIONS,
    CONDA_PREFIX,
    INCLUDE_DIRS,
    PARSE_ARGS,
    PARSE_OPTIONS,
    decl_names,
)
from gil_entry_points import Release

# Gitignored (`.cache/`); CI restores and saves it between runs (.github/workflows/audit.yml).
CACHE_ROOT = REPO_ROOT / ".cache" / "binding-audit"
FACT_CACHE = FactCache(CACHE_ROOT)
SCRIPTS = Path(__file__).resolve().parent
# Environment variables that add to libclang's header search path.
HEADER_SEARCH_ENV = ("CPATH", "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH")
# Written by the interpreter, never #included; their churn must not invalidate the cache.
LISTING_IGNORED_DIRS = frozenset({"__pycache__"})
# The scripts each fact's extraction runs: the cache, this module, the parse and the walks.
# Not `audit_ownership.py`: the parse reads only its path constants (in the key as `args`) and
# its `header_rel` for error text, and binding PRs edit its tables all the time.
COMMON_EXTRACTORS = ("audit_cache.py", "audit_facts.py", "audit_parse.py")
DECL_EXTRACTORS = (*COMMON_EXTRACTORS, "audit_types.py")
BOUND_EXTRACTORS = (*COMMON_EXTRACTORS, "audit_types.py", "audit_bound.py")
GIL_EXTRACTORS = (*COMMON_EXTRACTORS, "audit_gil.py", "gil_entry_points.py")


class FactShapeError(CacheError):
    """A stored fact is valid JSON but not the shape its extractor writes."""


@dataclass(frozen=True)
class DeclFacts:
    """What `audit_tu` reads from the declarations-only parse besides `cpp_api`.

    Attributes:
        names: `decl_names`.
        ancestors: `audit_types.class_ancestors`.
        bases: `audit_types.public_bases`.
    """

    names: frozenset[str]
    ancestors: Mapping[str, frozenset[str]]
    bases: Mapping[str, tuple[str, ...]]


@dataclass(frozen=True)
class BoundFacts:
    """What a binding binds, from its bodies parse.

    Attributes:
        classes: `bound_classes`.
        exceptions: the Python names of `bound_exceptions`.
    """

    classes: Mapping[str, str]
    exceptions: frozenset[str]


@functools.cache
def environment_fingerprint() -> str:
    """The audit env's package set: every conda record and Python distribution, by version
    and build. Any package change (libclang, a header package) changes it."""
    records = sorted(p.name for p in (CONDA_PREFIX / "conda-meta").glob("*.json"))
    dists = sorted(p.name for p in CONDA_PREFIX.glob("lib/python*/site-packages/*.dist-info"))
    return _digest([records, dists])


@functools.cache
def repo_dir_listing(directory: Path) -> str:
    """Every file under one include dir inside the repo, as one digest."""
    files = sorted(
        str(p.relative_to(directory))
        for p in directory.rglob("*")
        if p.is_file() and LISTING_IGNORED_DIRS.isdisjoint(p.relative_to(directory).parts)
    )
    return _digest(files)


@functools.cache
def extractor_digest(names: tuple[str, ...]) -> str:
    """The source of the scripts that extract a fact."""
    return _digest({n: sha256_of(SCRIPTS / n) for n in names})


def fact_key(
    kind: str,
    cpp: Path,
    args: Sequence[str],
    options: int,
    extractors: tuple[str, ...],
) -> FactKey:
    """The key of one fact of the parse of `cpp` with `args` and `options`.

    Args:
        kind: The fact.
        cpp: The TU's main file.
        args: The full libclang argument list, include dirs included.
        options: libclang `TranslationUnit.PARSE_*` flags.
        extractors: The script files whose code produces the fact (`*_EXTRACTORS`).
    """
    cpp = cpp.resolve()
    include_dirs = [Path(a.removeprefix("-I")) for a in args if a.startswith("-I")]
    repo_dirs = sorted({d.resolve() for d in (cpp.parent, *include_dirs) if _in_repo(d)})
    toolchain = {
        "args": list(args),
        "options": options,
        "env": environment_fingerprint(),
        "search_env": {v: os.environ.get(v) for v in HEADER_SEARCH_ENV},
        "repo_dirs": {str(d): repo_dir_listing(d) for d in repo_dirs},
        "extractors": extractor_digest(extractors),
    }
    return FactKey(kind=kind, source=cpp, toolchain=json.dumps(toolchain, sort_keys=True))


def _in_repo(directory: Path) -> bool:
    """`directory` holds repo files. The pixi env can live inside the repo (`.pixi/envs/audit`
    in CI); its dirs are covered by `environment_fingerprint`, not by a listing."""
    d = directory.resolve()
    return d.is_relative_to(REPO_ROOT) and not d.is_relative_to(CONDA_PREFIX.resolve())


def tu_files(tu: ci.TranslationUnit) -> list[Path]:
    """Every file the parse read: the main file and each header it included."""
    return [Path(tu.spelling), *(Path(i.include.name) for i in tu.get_includes())]


def audit_args(extra_include_dirs: Sequence[Path]) -> list[str]:
    """`parse_tu`'s argument list for `extra_include_dirs`."""
    return [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]


def decl_facts(
    cpp: Path, extra_include_dirs: Sequence[Path], tu: Callable[[], ci.TranslationUnit]
) -> DeclFacts:
    """`DeclFacts` of `parse_tu(cpp, extra_include_dirs)`, which `tu` returns on a miss.

    `audit_tu` parses `cpp` anyway for `cpp_api`; a hit saves the three whole-TU walks.
    """

    def compute() -> tuple[object, Iterable[Path]]:
        parsed = tu()
        value = {
            "names": sorted(decl_names(parsed)),
            "ancestors": {k: sorted(v) for k, v in audit_types.class_ancestors(parsed).items()},
            "bases": {k: list(v) for k, v in audit_types.public_bases(parsed).items()},
        }
        return value, tu_files(parsed)

    key = fact_key("decl", cpp, audit_args(extra_include_dirs), PARSE_OPTIONS, DECL_EXTRACTORS)
    value = _object(FACT_CACHE.fetch(key, compute))
    return DeclFacts(
        names=frozenset(_strings(value["names"])),
        ancestors={k: frozenset(_strings(v)) for k, v in _object(value["ancestors"]).items()},
        bases={k: tuple(_strings(v)) for k, v in _object(value["bases"]).items()},
    )


def bound_facts(
    cpp: Path,
    extra_include_dirs: Sequence[Path],
    parse: Callable[..., ci.TranslationUnit],
) -> BoundFacts:
    """`BoundFacts` of `cpp`; `parse` (`parse_tu`) parses it with bodies on a miss."""

    def compute() -> tuple[object, Iterable[Path]]:
        parsed = parse(cpp, extra_include_dirs, bodies=True)
        value = {
            "classes": dict(bound_classes(parsed)),
            "exceptions": sorted(bound_exceptions(parsed)),
        }
        return value, tu_files(parsed)

    args = audit_args(extra_include_dirs)
    key = fact_key("bound", cpp, args, BODY_PARSE_OPTIONS, BOUND_EXTRACTORS)
    value = _object(FACT_CACHE.fetch(key, compute))
    return BoundFacts(
        classes={k: _string(v) for k, v in _object(value["classes"]).items()},
        exceptions=frozenset(_strings(value["exceptions"])),
    )


def gil_releases(
    cpp: Path,
    args: Sequence[str],
    scan: Callable[[ci.TranslationUnit], Mapping[Release, str]],
    parse: Callable[[Path, Sequence[str]], ci.TranslationUnit],
) -> dict[Release, str]:
    """Each GIL release `scan` finds in `parse(cpp, args)` -> where it is.

    Args:
        cpp: The binding source.
        args: The GIL audit's argument list.
        scan: The release scan of one TU (`audit_gil.Scanner(tu, module).scan`).
        parse: The GIL audit's bodies parse (`audit_gil.parse`, default options).
    """

    def compute() -> tuple[object, Iterable[Path]]:
        parsed = parse(cpp, args)
        value = sorted([r.module, r.scope, r.name, where] for r, where in scan(parsed).items())
        return value, tu_files(parsed)

    key = fact_key("gil", cpp, args, ci.TranslationUnit.PARSE_NONE, GIL_EXTRACTORS)
    out: dict[Release, str] = {}
    for row in _list(FACT_CACHE.fetch(key, compute)):
        module, scope, name, where = _strings(row)
        out[Release(module, scope, name)] = where
    return out


def _digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def _object(value: Json) -> dict[str, Json]:
    if not isinstance(value, dict):
        raise FactShapeError(f"expected a JSON object, got {type(value).__name__}")
    return value


def _list(value: Json) -> list[Json]:
    if not isinstance(value, list):
        raise FactShapeError(f"expected a JSON list, got {type(value).__name__}")
    return value


def _string(value: Json) -> str:
    if not isinstance(value, str):
        raise FactShapeError(f"expected a JSON string, got {type(value).__name__}")
    return value


def _strings(value: Json) -> list[str]:
    return [_string(v) for v in _list(value)]
