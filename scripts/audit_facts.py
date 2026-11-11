"""The facts the audit reads from a TU, cached on the TU's inputs (`audit_cache`).

Five facts, each one extractor's output on one parse:

- `include_facts`: the inclusion directives and the included files of the declarations-only
  parse (`PARSE_OPTIONS`): what header ownership and the `api` key are computed from, without
  a parse;
- `api_facts`: `audit_parse.extract_api` on that parse, for one `Selection` of headers. What
  other modules bind is not in it (`audit_symbols.resolve_bound` runs after the fetch);
- `decl_facts`: declaration names, class ancestors and public bases of that parse, three
  walks of the whole TU from Python;
- `bound_facts`: the `nb::class_` / `nb::enum_` / `nb::exception` constructions of the
  bodies parse (`BODY_PARSE_OPTIONS`);
- `gil_releases`: `audit_gil.Scanner.scan` of the GIL audit's own bodies parse.

A fact's `FactKey.toolchain` (`audit_toolchain.Toolchain`, built by `fact_key`) holds whatever
the parse reads besides the files it records: the parse arguments and options, the
environment's package set (libclang, every header package, nanobind), the header-search
variables libclang reads, the file listing of each include dir in the repo (a new file there
can shadow a header found later on the search path), and the source of every script that
runs while the fact is extracted.

`include_facts` and `api_facts` also serve a `Synthetic` TU, whose main file exists only in
memory (the unincluded headers of a binding): its text is `FactKey.unsaved`, and the main
file is no dependency. A parse that fails stores nothing.

No `compute` here fetches another fact (no nested entry locks), and none calls through a
name a test patches on `audit_match`: a patched extractor would write its result into the
entry of the unpatched key.
"""

from __future__ import annotations

import functools
import os
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import audit_types
import clang.cindex as ci
from audit_bound import bound_classes, bound_exceptions
from audit_cache import CacheError, FactCache, FactKey, Json, sha256_of
from audit_native import NATIVE_DIRS
from audit_ownership import REPO_ROOT
from audit_parse import (
    BODY_PARSE_OPTIONS,
    CONDA_PREFIX,
    INCLUDE_DIRS,
    PARSE_ARGS,
    PARSE_OPTIONS,
    decl_names,
    direct_includes,
    extract_api,
    inclusions,
)
from audit_symbols import CppSymbol, decode_symbols, encode_symbols
from audit_toolchain import (
    Selection,
    Sha256,
    Toolchain,
    digest_of,
    repo_dir_listing,
    require_listed,
)
from gil_entry_points import Release

# Gitignored (`.cache/`); CI restores and saves it between runs (.github/workflows/audit.yml).
CACHE_ROOT = REPO_ROOT / ".cache" / "binding-audit"
FACT_CACHE = FactCache(CACHE_ROOT)
SCRIPTS = Path(__file__).resolve().parent
# Environment variables that add to libclang's header search path.
HEADER_SEARCH_ENV = ("CPATH", "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH")
# The scripts each fact's extraction runs: the cache and its key, this module, the parse and
# the walks. Not `audit_telemetry.py`: it records a fetch and decides nothing about a fact.
# Not `audit_ownership.py`: the parse reads only its path constants (in the key as `args`) and
# its `header_rel` for error text, and binding PRs edit its tables all the time.
COMMON_EXTRACTORS = (
    "audit_cache.py",
    "audit_toolchain.py",
    "audit_facts.py",
    "audit_native.py",
    "audit_parse.py",
)
DECL_EXTRACTORS = (*COMMON_EXTRACTORS, "audit_types.py")
INCLUDE_EXTRACTORS = COMMON_EXTRACTORS
API_EXTRACTORS = (*COMMON_EXTRACTORS, "audit_types.py", "audit_symbols.py")
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
class Synthetic:
    """A TU whose main file exists only in memory.

    Attributes:
        text: The main file's source text.
        binding: The binding source `text` includes first, from its own directory.
    """

    text: str
    binding: Path


@dataclass(frozen=True)
class IncludeFacts:
    """What a TU includes, as paths: what `audit_tu` reads besides declarations.

    Attributes:
        direct: `audit_parse.direct_includes`.
        inclusions: `audit_parse.inclusions`.
        included: Every file the parse included (`get_includes()`), resolved.
    """

    direct: tuple[Path, ...]
    inclusions: tuple[tuple[str, Path], ...]
    included: frozenset[Path]


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
def environment_fingerprint() -> Sha256:
    """The audit env's package set: every conda record and Python distribution, by version
    and build. Any package change (libclang, a header package) changes it."""
    records = sorted(p.name for p in (CONDA_PREFIX / "conda-meta").glob("*.json"))
    dists = sorted(p.name for p in CONDA_PREFIX.glob("lib/python*/site-packages/*.dist-info"))
    return digest_of([records, dists])


@functools.cache
def extractor_digest(names: tuple[str, ...]) -> Sha256:
    """The source of the scripts that extract a fact."""
    return digest_of({n: sha256_of(SCRIPTS / n) for n in names})


def fact_key(
    kind: str,
    cpp: Path,
    args: Sequence[str],
    options: int,
    extractors: tuple[str, ...],
    *,
    unsaved: str | None = None,
    selection: Selection | None = None,
) -> FactKey:
    """The key of one fact of the parse of `cpp` with `args` and `options`.

    Args:
        kind: The fact.
        cpp: The TU's main file.
        args: The full libclang argument list, include dirs included.
        options: libclang `TranslationUnit.PARSE_*` flags.
        extractors: The script files whose code produces the fact (`*_EXTRACTORS`).
        unsaved: The main file's text, when it exists only in memory.
        selection: What the extractor reads of the TU that a table decides.
    """
    cpp = cpp.resolve()
    include_dirs = [Path(a.removeprefix("-I")) for a in args if a.startswith("-I")]
    repo_dirs = {d.resolve() for d in (cpp.parent, *include_dirs) if _in_repo(d)}
    toolchain = Toolchain.build(
        args=args,
        options=options,
        env=environment_fingerprint(),
        search_env={v: os.environ.get(v) for v in HEADER_SEARCH_ENV},
        repo_dirs={d: repo_dir_listing(d) for d in repo_dirs},
        extractors=extractor_digest(extractors),
    )
    return FactKey(kind=kind, source=cpp, toolchain=toolchain, unsaved=unsaved, selection=selection)


def _in_repo(directory: Path) -> bool:
    """`directory` holds repo files. The pixi env can live inside the repo (`.pixi/envs/audit`
    in CI) and so can the native stack (`.native/<triplet>` locally; in CI its root is the
    workspace, which holds the repo, so only its own `ws/` and `vcpkg/` are left out); their files are covered by
    `environment_fingerprint` and by the content hash of each header a parse read, not by a
    listing."""
    d = directory.resolve()
    outside = (CONDA_PREFIX.resolve(), *(n.resolve() for n in NATIVE_DIRS))
    return d.is_relative_to(REPO_ROOT) and not any(d.is_relative_to(o) for o in outside)


def tu_files(tu: ci.TranslationUnit, synthetic: Synthetic | None = None) -> list[Path]:
    """Every file on disk the parse read: the main file and each file it included. The main
    file of a `synthetic` TU is not on disk (its text is in the key) and is left out.

    Raises:
        UnlistedIncludeError: the parse included a file `repo_dir_listing` skips, so the
            key would not see a shadowing file of that kind (`require_listed`). The binding
            a synthetic TU includes from its own directory is exempt.
        NotASiblingError: a synthetic TU's binding is not in the main file's directory.
    """
    source = Path(tu.spelling)
    included = [Path(i.include.name) for i in tu.get_includes()]
    if synthetic is None:
        files = [source, *included]
        require_listed(source, files)
        return files
    require_listed(
        source.resolve(), [p.resolve() for p in included], siblings=[synthetic.binding.resolve()]
    )
    return included


def audit_args(extra_include_dirs: Sequence[Path]) -> list[str]:
    """`parse_tu`'s argument list for `extra_include_dirs`."""
    return [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]


def decl_facts(
    cpp: Path, extra_include_dirs: Sequence[Path], tu: Callable[[], ci.TranslationUnit]
) -> DeclFacts:
    """`DeclFacts` of `parse_tu(cpp, extra_include_dirs)`, which `tu` returns on a miss.

    A hit saves the three whole-TU walks, and with `include_facts` and `api_facts` the parse.
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


def include_facts(
    cpp: Path,
    extra_include_dirs: Sequence[Path],
    tu: Callable[[], ci.TranslationUnit],
    synthetic: Synthetic | None = None,
) -> IncludeFacts:
    """`IncludeFacts` of `parse_tu(cpp, extra_include_dirs)`, which `tu` returns on a miss.

    Args:
        cpp: The TU's main file; the name of a synthetic TU.
        extra_include_dirs: Searched before `INCLUDE_DIRS`.
        tu: Parses the TU; called on a miss only.
        synthetic: Set when `cpp` exists only in memory.
    """

    def compute() -> tuple[object, Iterable[Path]]:
        parsed = tu()
        value = {
            "direct": [str(p) for p in direct_includes(parsed)],
            "inclusions": [[spelling, str(p)] for spelling, p in inclusions(parsed)],
            "included": sorted(
                {str(Path(i.include.name).resolve()) for i in parsed.get_includes()}
            ),
        }
        return value, tu_files(parsed, synthetic)

    key = fact_key(
        "includes",
        cpp,
        audit_args(extra_include_dirs),
        PARSE_OPTIONS,
        INCLUDE_EXTRACTORS,
        unsaved=None if synthetic is None else synthetic.text,
    )
    value = _object(FACT_CACHE.fetch(key, compute))
    return IncludeFacts(
        direct=tuple(Path(p) for p in _strings(value["direct"])),
        inclusions=tuple((s, Path(p)) for s, p in map(_strings, _list(value["inclusions"]))),
        included=frozenset(Path(p) for p in _strings(value["included"])),
    )


def api_facts(
    cpp: Path,
    extra_include_dirs: Sequence[Path],
    selection: Selection,
    tu: Callable[[], ci.TranslationUnit],
    synthetic: Synthetic | None = None,
) -> dict[str, CppSymbol]:
    """`audit_parse.extract_api` of `parse_tu(cpp, extra_include_dirs)` for `selection`,
    R13 unresolved (`audit_symbols.resolve_bound`).

    Args:
        cpp: The TU's main file; the name of a synthetic TU.
        extra_include_dirs: Searched before `INCLUDE_DIRS`.
        selection: `audit_parse.api_selection`: the audited and not-API headers, and the
            unbound-dependency roots the extraction reads.
        tu: Parses the TU; called on a miss only.
        synthetic: Set when `cpp` exists only in memory.
    """

    def compute() -> tuple[object, Iterable[Path]]:
        parsed = tu()
        symbols = extract_api(
            parsed,
            frozenset(Path(h) for h in selection.audited),
            frozenset(Path(h) for h in selection.not_api),
        )
        return encode_symbols(symbols), tu_files(parsed, synthetic)

    key = fact_key(
        "api",
        cpp,
        audit_args(extra_include_dirs),
        PARSE_OPTIONS,
        API_EXTRACTORS,
        unsaved=None if synthetic is None else synthetic.text,
        selection=selection,
    )
    return decode_symbols(FACT_CACHE.fetch(key, compute))


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
