"""Hold each `PLUGIN_ANCHOR_INCLUDES` header to its claim: plugin plumbing only.

A listed header is not audited although its binding #includes it directly. That is sound only
while the header declares nothing a Python user could call. The check is class-granular: it
accepts a function only by signature (`const void* f()`) and a class only if every base is a
plugin factory interface, without inspecting the class's members. Any other declaration there
is API the audit would hide, so it raises instead of producing no row.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from pathlib import Path

import clang.cindex as ci
from audit_ownership import PLUGIN_ANCHOR_INCLUDES, header_owner
from audit_rules import ACCEPTED_SYMBOLS

# The plugin factory interfaces, by unqualified name: the classes the rule tables accept as
# `plugin-author-factory-interface`. A concrete factory derives from one of them.
FACTORY_INTERFACE_RULE = "plugin-author-factory-interface"
PLUGIN_FACTORY_INTERFACES = frozenset(
    name for (_, name), rule in ACCEPTED_SYMBOLS.items() if rule == FACTORY_INTERFACE_RULE
)
# `PLUGIN_ANCHOR_DECL(NAME)` expands to `const void* NAME();` (boost_plugin_loader/macros.h).
ANCHOR_RESULT = "const void *"
RECORD_DECLS = frozenset({ci.CursorKind.CLASS_DECL, ci.CursorKind.STRUCT_DECL})


class AnchorHeaderApiError(ValueError):
    """A `PLUGIN_ANCHOR_INCLUDES` header declares more than its anchor and plugin factories."""


class StaleAnchorIncludeError(LookupError):
    """A `PLUGIN_ANCHOR_INCLUDES` header that its owning module's binding does not #include."""


def _declarations(cursor: ci.Cursor, header: Path) -> Iterator[ci.Cursor]:
    """Declarations located in `header` among `cursor`'s children, through namespaces opened
    there; preprocessing cursors (inclusions, macro definitions and expansions) excluded."""
    for c in cursor.get_children():
        if c.kind.is_preprocessing() or c.location.file is None:
            continue
        if Path(c.location.file.name).resolve() != header:
            continue
        if c.kind == ci.CursorKind.NAMESPACE:
            yield from _declarations(c, header)
        else:
            yield c


def _is_anchor(c: ci.Cursor) -> bool:
    """The declaration `PLUGIN_ANCHOR_DECL` expands to: `const void* NAME();`."""
    return (
        c.kind == ci.CursorKind.FUNCTION_DECL
        and not list(c.get_arguments())
        and c.result_type.spelling == ANCHOR_RESULT
    )


def _is_plugin_factory(c: ci.Cursor) -> bool:
    """A class forward declaration, or a class whose every base is a plugin factory
    interface (it has at least one)."""
    if c.kind not in RECORD_DECLS:
        return False
    if not c.is_definition():
        return True
    bases = [
        b.type.get_canonical().get_declaration().spelling
        for b in c.get_children()
        if b.kind == ci.CursorKind.CXX_BASE_SPECIFIER
    ]
    return bool(bases) and all(b in PLUGIN_FACTORY_INTERFACES for b in bases)


def anchor_header_api(tu: ci.TranslationUnit, header: Path) -> list[str]:
    """Declarations in `header` (resolved) that are neither its plugin anchor nor a plugin
    factory, as `name (kind, line N)`."""
    return [
        f"{c.spelling} ({c.kind.name}, line {c.location.line})"
        for c in _declarations(tu.cursor, header)
        if not (_is_anchor(c) or _is_plugin_factory(c))
    ]


def check_anchor_headers(tu: ci.TranslationUnit, headers: Mapping[Path, str]) -> None:
    """Every header in `headers` (resolved path -> include spelling) declares only its anchor
    and plugin factories.

    Raises:
        AnchorHeaderApiError: one lists the other declarations, per header; the header holds
            API and belongs out of `PLUGIN_ANCHOR_INCLUDES`.
    """
    found = {s: api for h, s in sorted(headers.items()) if (api := anchor_header_api(tu, h))}
    if found:
        listed = "; ".join(f"{s}: {', '.join(api)}" for s, api in found.items())
        raise AnchorHeaderApiError(
            f"PLUGIN_ANCHOR_INCLUDES headers declare more than a plugin anchor and plugin "
            f"factories, so the audit would hide API: {listed}"
        )


def check_anchor_includes_current(
    module: str, direct: frozenset[str], prefixes: Mapping[str, tuple[str, ...]]
) -> None:
    """Every `PLUGIN_ANCHOR_INCLUDES` header that `module` owns is among `direct`, the include
    spellings of its binding.

    Raises:
        StaleAnchorIncludeError: an entry the binding no longer #includes (or a misspelt one).
    """
    owned = {s for s in PLUGIN_ANCHOR_INCLUDES if header_owner(s, prefixes) == module}
    if stale := sorted(owned - direct):
        raise StaleAnchorIncludeError(
            f"{module}: PLUGIN_ANCHOR_INCLUDES entries its binding does not #include: {stale}"
        )
