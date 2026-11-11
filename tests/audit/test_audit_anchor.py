"""Contract tests of PLUGIN_ANCHOR_INCLUDES: a header a binding #includes only to call its
plugin anchor is not audited, and the audit raises when such a header declares API.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from pathlib import Path

import pytest

from tests.audit._load import audit

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"
FIXTURE_INCLUDE = FIXTURES / "include"
ANCHOR_PREFIX = ("tesseract/anchor/",)
SOLVER_HEADER = "tesseract/anchor/solver_factories.h"
LEAKY_HEADER = "tesseract/anchor/leaky_factories.h"
REASON = "Included only for its plugin anchor (fixture)."
# The plugin plumbing in solver_factories.h: one factory and the anchor.
PLUMBING = {"AnchorSolverFactory", "AnchorFactoriesAnchor"}


def _audit(cpp: str):
    return audit.audit_tu(
        "anchor",
        FIXTURES / cpp,
        FIXTURES / "_anchor.pyi",
        FIXTURES / "package_init.py",
        ANCHOR_PREFIX,
        (FIXTURE_INCLUDE,),
        stub_roots=(FIXTURES,),
        prefixes={"anchor": ANCHOR_PREFIX},
    )


def _resolved(spelling: str) -> Path:
    return (FIXTURE_INCLUDE / spelling).resolve()


def test_directly_included_anchor_header_is_audited_without_an_entry():
    """Control: without the entry, the direct include makes the plumbing gap rows."""
    gaps = {g.symbol for g in _audit("anchor_bindings.cpp").gaps}
    assert gaps >= PLUMBING


def test_listed_anchor_header_is_not_audited(monkeypatch):
    monkeypatch.setitem(audit.PLUGIN_ANCHOR_INCLUDES, SOLVER_HEADER, REASON)
    gaps = {g.symbol for g in _audit("anchor_bindings.cpp").gaps}
    assert not gaps & PLUMBING
    # The interface header it pulls in is not listed, so it stays audited.
    assert "InvKinFactory" in gaps


def test_anchor_header_declaring_api_fails_the_audit(monkeypatch):
    """Listing a header never hides API it declares: the audit raises and names it."""
    monkeypatch.setitem(audit.PLUGIN_ANCHOR_INCLUDES, LEAKY_HEADER, REASON)
    with pytest.raises(audit.AnchorHeaderApiError) as err:
        _audit("leaky_anchor_bindings.cpp")
    message = str(err.value)
    for api in ("leakyHelper", "Leaky ", "LeakyMixedFactory"):
        assert api in message
    for plumbing in ("LeakyFactory ", "LeakyFactoriesAnchor"):
        assert plumbing not in message


def test_anchor_header_api_lists_only_non_plumbing():
    tu = audit.parse_tu(FIXTURES / "leaky_anchor_bindings.cpp", (FIXTURE_INCLUDE,))
    names = {a.split(" ")[0] for a in audit.anchor_header_api(tu, _resolved(LEAKY_HEADER))}
    assert names == {"leakyHelper", "Leaky", "LeakyMixedFactory"}
    tu = audit.parse_tu(FIXTURES / "anchor_bindings.cpp", (FIXTURE_INCLUDE,))
    assert audit.anchor_header_api(tu, _resolved(SOLVER_HEADER)) == []


def test_stale_anchor_entry_fails_the_audit(monkeypatch):
    """An entry the owning binding does not #include (dropped include, misspelt path)."""
    monkeypatch.setitem(audit.PLUGIN_ANCHOR_INCLUDES, SOLVER_HEADER, REASON)
    monkeypatch.setitem(audit.PLUGIN_ANCHOR_INCLUDES, LEAKY_HEADER, REASON)
    with pytest.raises(audit.StaleAnchorIncludeError, match="leaky_factories.h"):
        _audit("anchor_bindings.cpp")


def test_every_entry_is_an_audited_modules_header_its_binding_includes():
    """Cheap form of the stale check over the real table: the owner's binding #includes it."""
    prefixes = audit.owner_prefixes()
    assert audit.PLUGIN_ANCHOR_INCLUDES
    for spelling, reason in audit.PLUGIN_ANCHOR_INCLUDES.items():
        owner = audit.header_owner(spelling, prefixes)
        assert owner in audit.AUDITED_HEADER_PREFIX, spelling
        assert reason
        includes = audit.direct_include_spellings(audit.binding_sources(owner))
        assert spelling in includes, (owner, spelling)


def test_anchor_includes_rendered_with_reasons():
    report = audit.ModuleReport("fixture", 0, [], [], [], [])
    coverage = audit.Coverage(audited=("fixture",), unaudited={}, total=1)
    prov = {"tesseract-robotics": "==0.35.0", "libclang": "clang version 23", "stubs": "abc1234"}
    text = audit.render_markdown([report], prov, coverage, {})
    for spelling, reason in audit.PLUGIN_ANCHOR_INCLUDES.items():
        assert f"`{spelling}`: {reason}" in text
