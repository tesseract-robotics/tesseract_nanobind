"""The part of a fact's key that is not a file the parse read (`audit_cache.FactKey.toolchain`).

A cached fact is valid while every file its parse read is unchanged and the parse would read
the same files. The second half is the `Toolchain`: the parse arguments and options, the
environment's package set, the header-search variables, the source of the extractor scripts,
and one listing per include dir inside the repo. The listing is there because a new file in a
dir searched early shadows a header found later on the search path, without any recorded
file changing.

The listing names every file a parse could include. That is not "every header suffix":
`<vector>` and `<Eigen/Core>` have no suffix, and libstdc++ includes `.tcc` files. So it lists
everything except the suffixes no parse reads (`NOT_INCLUDED_SUFFIXES`), and `require_listed`
proves that premise on every parse whose facts are stored.

Needs no libclang.
"""

from __future__ import annotations

import dataclasses
import functools
import hashlib
import json
from collections.abc import Collection, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import NewType

Sha256 = NewType("Sha256", str)
# Files in a repo include dir that no parse includes: Python sources and stubs, docs, the
# `py.typed` marker and the binding sources themselves (each is a TU's main file, never an
# include). Their churn must not invalidate the cache; `require_listed` checks the premise.
NOT_INCLUDED_SUFFIXES = frozenset({".py", ".pyi", ".md", ".typed", ".cpp"})
# Written by the interpreter, never #included.
LISTING_IGNORED_DIRS = frozenset({"__pycache__"})


class UnlistedIncludeError(RuntimeError):
    """A parse read a file the listing skips: a shadowing file of that kind would go unseen."""


class RelativeRepoDirError(ValueError):
    """A repo include dir is relative: it names another directory from another cwd."""


class RelativeHeaderError(ValueError):
    """A selected header path is relative: it names another file from another cwd."""


class NotASiblingError(ValueError):
    """A file passed as a sibling of the main file is not in the main file's directory."""


def digest_of(value: object) -> Sha256:
    """One digest of anything `json.dumps` writes; mapping order does not matter."""
    return Sha256(hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest())


def _listed(relative: Path) -> bool:
    """A parse could include the file at `relative` (to its include dir, or absolute)."""
    return relative.suffix not in NOT_INCLUDED_SUFFIXES and LISTING_IGNORED_DIRS.isdisjoint(
        relative.parts
    )


@functools.cache
def repo_dir_listing(directory: Path) -> Sha256:
    """The name of every file under `directory` that a parse could include, as one digest.

    Names only: the content of each file a parse read is hashed by the cache entry.
    """
    return digest_of(
        sorted(
            str(relative)
            for p in directory.rglob("*")
            if p.is_file() and _listed(relative := p.relative_to(directory))
        )
    )


def require_listed(source: Path, files: Iterable[Path], *, siblings: Collection[Path] = ()) -> None:
    """Check that every file a parse read, except its main file, is one a listing names.

    Args:
        source: The TU's main file.
        files: Every file the parse read.
        siblings: Files the main text includes from its own directory by `#include "name"`
            (a synthetic TU includes its binding source so). That directory is searched
            first for a quoted include, so no listed dir can shadow them, and each is a
            recorded dependency: they need no listing.

    Raises:
        NotASiblingError: a sibling is not in the main file's directory.
        UnlistedIncludeError: an included file has a suffix or a directory the listing skips.
    """
    strangers = sorted(str(s) for s in siblings if s.parent != source.parent)
    if strangers:
        raise NotASiblingError(f"not beside {source}: {', '.join(strangers)}")
    exempt = {source, *siblings}
    unlisted = sorted(str(f) for f in files if f not in exempt and not _listed(f))
    if unlisted:
        raise UnlistedIncludeError(
            f"{source} includes {', '.join(unlisted)}: `repo_dir_listing` skips such files "
            "(NOT_INCLUDED_SUFFIXES, LISTING_IGNORED_DIRS), so its facts cannot be cached"
        )


@dataclass(frozen=True)
class Toolchain:
    """Everything a fact depends on besides the files its parse read.

    Attributes:
        args: The libclang argument list, in order (argument order is search order).
        options: libclang `TranslationUnit.PARSE_*` flags.
        env: Digest of the environment's package set.
        search_env: Each header-search variable and its value (`None`: unset), by name.
        repo_dirs: Each include dir inside the repo and its `repo_dir_listing`, by path.
        extractors: Digest of the scripts that extract the fact.
    """

    args: tuple[str, ...]
    options: int
    env: Sha256
    search_env: tuple[tuple[str, str | None], ...]
    repo_dirs: tuple[tuple[str, Sha256], ...]
    extractors: Sha256

    @classmethod
    def build(
        cls,
        *,
        args: Sequence[str],
        options: int,
        env: Sha256,
        search_env: Mapping[str, str | None],
        repo_dirs: Mapping[Path, Sha256],
        extractors: Sha256,
    ) -> Toolchain:
        """A toolchain whose mappings are in one order, whatever order they were built in.

        Raises:
            RelativeRepoDirError: a key of `repo_dirs` is not absolute.
        """
        relative = sorted(str(d) for d in repo_dirs if not d.is_absolute())
        if relative:
            raise RelativeRepoDirError(", ".join(relative))
        return cls(
            args=tuple(args),
            options=options,
            env=env,
            search_env=tuple(sorted(search_env.items())),
            repo_dirs=tuple(sorted((str(d), listing) for d, listing in repo_dirs.items())),
            extractors=extractors,
        )

    def canonical(self) -> str:
        """The one serialisation: every field by name. `FactKey.digest` hashes it."""
        return json.dumps(dataclasses.asdict(self), sort_keys=True)


@dataclass(frozen=True)
class Selection:
    """What an `api` extraction reads of a TU that a table decides, not the parse.

    Each field is sorted and unique: the extraction reads them as sets.

    Attributes:
        audited: The headers whose declarations are read, absolute.
        not_api: The owned not-API headers the TU includes, absolute.
        dependency_roots: `audit_parse.UNBOUND_DEPENDENCY_ROOTS` at extraction.
        dependency_namespaces: `audit_parse.UNBOUND_DEPENDENCY_NAMESPACES` at extraction.
    """

    audited: tuple[str, ...]
    not_api: tuple[str, ...]
    dependency_roots: tuple[str, ...]
    dependency_namespaces: tuple[str, ...]

    @classmethod
    def build(
        cls,
        *,
        audited: Iterable[Path],
        not_api: Iterable[Path],
        dependency_roots: Iterable[str],
        dependency_namespaces: Iterable[str],
    ) -> Selection:
        """A selection in one order, whatever order its sets were built in.

        Raises:
            RelativeHeaderError: a header path is not absolute.
        """
        audited, not_api = set(audited), set(not_api)
        relative = sorted(str(h) for h in audited | not_api if not h.is_absolute())
        if relative:
            raise RelativeHeaderError(", ".join(relative))
        return cls(
            audited=tuple(sorted(str(h) for h in audited)),
            not_api=tuple(sorted(str(h) for h in not_api)),
            dependency_roots=tuple(sorted(set(dependency_roots))),
            dependency_namespaces=tuple(sorted(set(dependency_namespaces))),
        )

    def canonical(self) -> str:
        """The one serialisation: every field by name. `FactKey.digest` hashes it."""
        return json.dumps(dataclasses.asdict(self), sort_keys=True)
