"""Fixture stub hygiene: the fixture stubs share one tree, so names must not clash (#256).

`_binding_stub` rglobs every `_*.pyi` below a report's stub root, and a report rooted at
`tests/audit/fixtures/` sees every fixture subfolder. A top-level class declared in two
fixture stubs can turn one fixture's unbound class into `bound-in-other-module`.

Run in the `audit` env only: `pixi run -e audit audit-test`.
"""

import ast
from collections import defaultdict
from pathlib import Path

FIXTURES = Path(__file__).resolve().parent / "fixtures"
# Same-leaf pairs a test needs: a class bound under one leaf in two modules' stubs.
SAME_LEAF_PAIRS = {
    # I7: RemoteRunner's stub base is `fixture_remote._fixture_remote.Runner`.
    ("Runner", frozenset({"_fixture.pyi", "fixture_remote/_fixture_remote.pyi"})),
    # T5: `bases_remote` binds another `Jogger` under the leaf of the bases fixture's own.
    ("Jogger", frozenset({"bases/_bases.pyi", "bases/bases_remote/_bases_remote.pyi"})),
}


def _stubs() -> list[Path]:
    return sorted(FIXTURES.rglob("_*.pyi"))


def _declarations() -> dict[str, set[str]]:
    """Top-level class name -> the fixture stubs (relative to `FIXTURES`) declaring it."""
    out: dict[str, set[str]] = defaultdict(set)
    for stub in _stubs():
        for node in ast.parse(stub.read_text()).body:
            if isinstance(node, ast.ClassDef):
                out[node.name].add(stub.relative_to(FIXTURES).as_posix())
    return out


def test_fixture_stub_module_names_are_unique():
    """A stub's stem names its module: two `_x.pyi` would be one module to the lookup."""
    stems = defaultdict(list)
    for stub in _stubs():
        stems[stub.stem].append(stub.relative_to(FIXTURES).as_posix())
    assert {s: p for s, p in stems.items() if len(p) > 1} == {}


def test_fixture_stub_class_names_are_unique():
    """Every top-level fixture class is declared in one stub, but for `SAME_LEAF_PAIRS`."""
    clashes = {}
    for name, stubs in _declarations().items():
        rest = set(stubs)
        for pair_name, pair in SAME_LEAF_PAIRS:
            if pair_name == name and pair <= rest:
                rest -= pair
                rest |= {min(pair)}  # the pair counts as one declaration
        if len(rest) > 1:
            clashes[name] = sorted(stubs)
    assert clashes == {}


def test_every_same_leaf_pair_is_declared():
    declarations = _declarations()
    assert all(pair <= declarations[name] for name, pair in SAME_LEAF_PAIRS)
