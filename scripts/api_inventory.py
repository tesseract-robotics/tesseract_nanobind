"""Inventory of the public stub API added between two git revisions.

The committed `.pyi` stubs are the public API of the extension modules. This
script diffs them between `--base` and `--head` and emits one row per added
entry: a new name, a new overload of an existing name, or a changed signature.

Identity of a symbol is (stub module, class-qualified name). Identity of an
overload is its tuple of normalised parameter types, with parameter names and
defaults dropped, so a parameter rename is not reported as new capability.
Annotations are normalised before comparison: quotes, module paths and C++
`ns::` prefixes are stripped, and C++ spellings the stub generator emitted for
a type that was not yet registered are mapped to their Python name. Without
that, an import fix such as #218 shows up as dozens of false new overloads.

Each added entry is attributed to the first commit in `base..head` whose stub
contains it. The default base is the 0.35.0.9 release cut: no `0.35.0.9` git
tag exists, so it is pinned by hash (`21930d81`, "docs: cut 0.35.0.9
changelog").

Example:
    ```bash
    python scripts/api_inventory.py --out build/example-api-coverage/inventory.json
    ```
"""

from __future__ import annotations

import argparse
import ast
import enum
import json
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

# The 0.35.0.9 changelog cut; the release has no git tag, so it is pinned by hash.
BASE_REV = "21930d81"
HEAD_REV = "HEAD"
STUB_ROOT = "src/tesseract_robotics/"
STUB_SUFFIX = ".pyi"
COMMIT_ABBREV = 8  # hex digits of a commit hash in a row

# Stub base classes that make a class an enum, whose assignments are enum members.
ENUM_BASES = frozenset(
    {"enum.Enum", "enum.IntEnum", "enum.Flag", "enum.IntFlag", "Enum", "IntEnum"}
)

# C++ spellings the stub generator emitted for an unregistered type, mapped to the
# Python name the type got once its defining module was imported (#218, Isometry3d fix).
CXX_ALIASES = {"Transform<double, 3, 1, 0>": "Isometry3d"}

# A dotted (`a.b.`) or C++ (`a::b::`) qualifier in front of a name.
_QUALIFIER = re.compile(r"\b(?:[A-Za-z_]\w*(?:\.|::))+(?=[A-Za-z_]\w*)")

# The ompl stub uses the keyword `from` as a parameter name, which `ast` rejects.
_KEYWORD_PARAM = re.compile(r"\bfrom: ")
_KEYWORD_PARAM_SAFE = "from_: "

UNANNOTATED = "?"
NO_ARITY = -1


class StubSyntaxError(Exception):
    """A stub file is not valid Python, even after the keyword-parameter rewrite."""


class GitRevisionError(Exception):
    """A revision given as base or head does not name a commit in the repository."""


class Change(str, enum.Enum):
    """How an added entry relates to the base stubs."""

    NEW_NAME = "new_name"
    NEW_OVERLOAD = "new_overload"
    # Heuristic label: an overload of the same name and arity disappeared.
    CHANGED_SIGNATURE = "changed_signature"


@dataclass(frozen=True)
class Entry:
    """One public API entry of a stub module.

    Attributes:
        qualname: Class-qualified name, e.g. `Environment.applyCommand`.
        kind: `class`, `function`, `method`, `staticmethod`, `property`,
            `attribute` or `enum_member`.
        overload: Normalised parameter-type tuple, e.g. `(Mapping[str, float])`;
            empty for non-callables.
        arity: Parameter count excluding `self`; -1 for non-callables.
    """

    qualname: str
    kind: str
    overload: str
    arity: int = NO_ARITY


@dataclass(frozen=True)
class Row:
    """An added entry with its module and the commit that first added it."""

    module: str
    qualname: str
    kind: str
    overload: str
    arity: int
    change: str
    commit: str
    subject: str


@dataclass(frozen=True)
class Inventory:
    """The rows added between two resolved revisions."""

    base: str
    head: str
    rows: list[Row]


def normalise_annotation(node: ast.expr | None) -> str:
    """Annotation text with quotes, module paths and `ns::` prefixes stripped.

    Args:
        node: The annotation expression, or None for an unannotated parameter.

    Returns:
        The normalised annotation text.
    """
    if node is None:
        return UNANNOTATED
    text = ast.unparse(node).replace("'", "").replace('"', "")
    text = _QUALIFIER.sub("", text)
    for cxx, py in CXX_ALIASES.items():
        text = text.replace(cxx, py)
    return text


def _overload(fn: ast.FunctionDef, drop_self: bool) -> str:
    args = fn.args
    params = [*args.posonlyargs, *args.args]
    if drop_self and params:
        params = params[1:]
    parts = [normalise_annotation(p.annotation) for p in params]
    if args.vararg:
        parts.append("*" + normalise_annotation(args.vararg.annotation))
    parts.extend("kw:" + normalise_annotation(p.annotation) for p in args.kwonlyargs)
    if args.kwarg:
        parts.append("**")
    return "(" + ", ".join(parts) + ")"


def _is_public(name: str) -> bool:
    # Every dunder the stub declares is API (operators, protocols); `_private` helpers are not.
    return not name.startswith("_") or (name.startswith("__") and name.endswith("__"))


def _parse(src: str) -> ast.Module:
    try:
        return ast.parse(_KEYWORD_PARAM.sub(_KEYWORD_PARAM_SAFE, src))
    except SyntaxError as exc:
        raise StubSyntaxError(f"stub does not parse: {exc}") from exc


def _walk(body: list[ast.stmt], prefix: str, enum_cls: bool, out: list[Entry]) -> None:
    for node in body:
        if isinstance(node, ast.ClassDef):
            if node.name.startswith("_"):
                continue
            qualname = prefix + node.name
            out.append(Entry(qualname, "class", ""))
            is_enum = any(ast.unparse(b) in ENUM_BASES for b in node.bases)
            _walk(node.body, qualname + ".", is_enum, out)
        elif isinstance(node, ast.FunctionDef):
            if not _is_public(node.name):
                continue
            decorators = {ast.unparse(d) for d in node.decorator_list}
            if "property" in decorators or any(d.endswith(".setter") for d in decorators):
                out.append(Entry(prefix + node.name, "property", ""))
                continue
            static = "staticmethod" in decorators
            kind = "function" if not prefix else ("staticmethod" if static else "method")
            drop_self = bool(prefix) and not static
            args = node.args
            arity = len(args.posonlyargs) + len(args.args) + len(args.kwonlyargs) - int(drop_self)
            out.append(Entry(prefix + node.name, kind, _overload(node, drop_self), arity))
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if not node.target.id.startswith("_"):
                out.append(Entry(prefix + node.target.id, "attribute", ""))
        elif isinstance(node, ast.Assign) and prefix:
            for target in node.targets:
                if isinstance(target, ast.Name) and not target.id.startswith("_"):
                    out.append(
                        Entry(prefix + target.id, "enum_member" if enum_cls else "attribute", "")
                    )


def _ordered_entries(src: str) -> list[Entry]:
    out: list[Entry] = []
    _walk(_parse(src).body, "", False, out)
    return out


def stub_entries(src: str) -> frozenset[Entry]:
    """Public entries of one stub module.

    Args:
        src: The stub source.

    Returns:
        The set of entries; a property's getter and setter are one entry.

    Raises:
        StubSyntaxError: `src` is not valid Python.
    """
    return frozenset(_ordered_entries(src))


def stub_overloads(src: str) -> dict[str, list[str]]:
    """Normalised overloads of every callable, in stub order.

    The stub generator emits overloads in `__nb_signature__` order, which is
    nanobind's dispatch order, so index `i` here is overload `i` at runtime.

    Args:
        src: The stub source.

    Returns:
        Class-qualified callable name → its overloads, in order.

    Raises:
        StubSyntaxError: `src` is not valid Python.
    """
    out: dict[str, list[str]] = {}
    for entry in _ordered_entries(src):
        if entry.kind in ("function", "method", "staticmethod"):
            out.setdefault(entry.qualname, []).append(entry.overload)
    return out


def diff_stub(base_src: str | None, head_src: str) -> list[tuple[Entry, Change]]:
    """Entries of `head_src` that `base_src` lacks, each with its change label.

    Args:
        base_src: The base stub, or None when the module is new.
        head_src: The head stub.

    Returns:
        Added entries sorted by (qualname, overload).

    Raises:
        StubSyntaxError: either stub is not valid Python.
    """
    base = stub_entries(base_src) if base_src is not None else frozenset()
    head = stub_entries(head_src)
    base_names = {e.qualname for e in base}
    removed = base - head
    out = []
    for entry in sorted(head - base, key=lambda e: (e.qualname, e.overload)):
        if entry.qualname not in base_names:
            change = Change.NEW_NAME
        elif any(r.qualname == entry.qualname and r.arity == entry.arity for r in removed):
            change = Change.CHANGED_SIGNATURE
        else:
            change = Change.NEW_OVERLOAD
        out.append((entry, change))
    return out


def _git(repo: Path, *args: str) -> str:
    proc = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=True)
    return proc.stdout


def _show(repo: Path, rev: str, path: str) -> str | None:
    proc = subprocess.run(
        ["git", "show", f"{rev}:{path}"], cwd=repo, capture_output=True, text=True
    )
    return proc.stdout if proc.returncode == 0 else None


def resolve(repo: Path, rev: str) -> str:
    """Full commit hash of `rev`.

    Raises:
        GitRevisionError: `rev` does not name a commit in `repo`.
    """
    proc = subprocess.run(
        ["git", "rev-parse", "--verify", "--quiet", f"{rev}^{{commit}}"],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        raise GitRevisionError(f"{rev!r} is not a commit in {repo}")
    return proc.stdout.strip()


def build_inventory(repo: Path, base: str, head: str) -> Inventory:
    """Rows for every public stub entry added between `base` and `head`.

    Args:
        repo: The git repository.
        base: Base revision (exclusive).
        head: Head revision (inclusive).

    Returns:
        The inventory, with both revisions resolved to full hashes.

    Raises:
        GitRevisionError: `base` or `head` is not a commit.
        StubSyntaxError: a stub at some revision is not valid Python.
    """
    base_sha, head_sha = resolve(repo, base), resolve(repo, head)
    commits = _git(repo, "rev-list", "--reverse", f"{base_sha}..{head_sha}").split()
    subjects = {c: _git(repo, "log", "-1", "--format=%s", c).strip() for c in commits}
    paths = [
        p
        for p in _git(repo, "ls-tree", "-r", "--name-only", head_sha, STUB_ROOT).split()
        if p.endswith(STUB_SUFFIX)
    ]
    rows: list[Row] = []
    for path in paths:
        added = diff_stub(_show(repo, base_sha, path), _show(repo, head_sha, path) or "")
        if not added:
            continue
        wanted = {e for e, _ in added}
        touched = set(_git(repo, "rev-list", f"{base_sha}..{head_sha}", "--", path).split())
        first: dict[Entry, str] = {}
        for commit in commits:
            if commit not in touched:
                continue
            src = _show(repo, commit, path)
            if src is None:
                continue
            for entry in stub_entries(src) & wanted:
                first.setdefault(entry, commit)
        module = path[len(STUB_ROOT) :].split("/")[0]
        for entry, change in added:
            commit = first.get(entry, "")
            rows.append(
                Row(
                    module,
                    **asdict(entry),
                    change=change.value,
                    commit=commit[:COMMIT_ABBREV],
                    subject=subjects.get(commit, ""),
                )
            )
    return Inventory(base_sha, head_sha, rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument(
        "--base", default=BASE_REV, help=f"base revision (default {BASE_REV}, the 0.35.0.9 cut)"
    )
    parser.add_argument("--head", default=HEAD_REV)
    parser.add_argument("--out", type=Path, required=True, help="inventory JSON to write")
    args = parser.parse_args(argv)

    inventory = build_inventory(args.repo, args.base, args.head)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "base": inventory.base,
        "head": inventory.head,
        "rows": [asdict(r) for r in inventory.rows],
    }
    args.out.write_text(json.dumps(payload, indent=1) + "\n")

    rows = inventory.rows
    print(
        f"{inventory.base[:COMMIT_ABBREV]}..{inventory.head[:COMMIT_ABBREV]}: {len(rows)} entries, {len({(r.module, r.qualname) for r in rows})} names"
    )
    for change in Change:
        print(f"  {change.value}: {sum(r.change == change.value for r in rows)}")
    print(f"  unattributed: {sum(not r.commit for r in rows)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
