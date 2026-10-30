"""What a binding TU binds: the `nb::class_`/`nb::enum_`/`nb::exception` constructions in its
NB_MODULE body."""

from __future__ import annotations

from pathlib import Path

import audit_types
import clang.cindex as ci

# Canonical spellings of the nanobind constructions that bind a C++ type to a Python name.
NB_CLASS_TEMPLATES = ("nanobind::class_<", "nanobind::enum_<")
NB_EXCEPTION_TEMPLATE = "nanobind::exception<"
NB_CLASS_CTORS = frozenset({"class_", "enum_", "exception"})
NB_MODULE_SCOPE = "nanobind::module_"
# NB_MODULE(name, m) defines `PyInit_<name>`; a TU parsed without bodies only declares it.
NB_MODULE_INIT_PREFIX = "PyInit_"


class SkippedBodiesError(ValueError):
    """`bound_classes` got a TU parsed without function bodies: NB_MODULE's body is empty."""


def _in_main_file(c: ci.Cursor, main: Path) -> bool:
    return c.location.file is not None and Path(c.location.file.name).resolve() == main


def _scope_type(arg: ci.Cursor) -> str:
    """Canonical type of the scope a construction binds into, under nanobind's `handle`
    conversion: `nanobind::module_` for a top-level class, a `class_<…>` for a nested one."""
    for c in arg.walk_preorder():
        spelling = c.type.get_canonical().spelling.removeprefix("const ")
        if spelling.startswith((NB_MODULE_SCOPE, *NB_CLASS_TEMPLATES)):
            return spelling
    return ""


def bound_classes(tu: ci.TranslationUnit) -> dict[str, str]:
    """Top-level Python class name -> qualified C++ class, as the binding's `nb::class_` and
    `nb::enum_` constructions in the TU's main file say (`TOTGCompositeProfile` ->
    `tesseract::time_parameterization::TimeOptimalTrajectoryGenerationCompositeProfile`).

    A template instance maps to its template (`Planner<double>` -> `…::Planner`), as
    `cpp_api` keys it. Nested classes (bound into a `class_`, not the module) are skipped.

    Raises:
        SkippedBodiesError: `tu` was parsed without function bodies (`parse_tu(bodies=True)`).
    """
    return _constructions(tu, NB_CLASS_TEMPLATES)


def bound_exceptions(tu: ci.TranslationUnit) -> dict[str, str]:
    """Python exception name -> qualified C++ exception type, as the binding's `nb::exception`
    registrations in the TU's main file say (`SimplePlannerEmptyGenerateError` ->
    `::EmptyGenerateError`, declared in the binding's anonymous namespace).

    Raises:
        SkippedBodiesError: `tu` was parsed without function bodies (`parse_tu(bodies=True)`).
    """
    return _constructions(tu, (NB_EXCEPTION_TEMPLATE,))


def _constructions(tu: ci.TranslationUnit, templates: tuple[str, ...]) -> dict[str, str]:
    """Python name -> qualified C++ type of each top-level construction of `templates`."""
    main = Path(tu.spelling).resolve()
    out: dict[str, str] = {}
    # Walk only the main file's top-level declarations: the headers' bodies are most of a TU.
    own = (
        c
        for top in tu.cursor.get_children()
        if _in_main_file(top, main)
        for c in top.walk_preorder()
    )
    for c in own:
        if (
            c.kind == ci.CursorKind.FUNCTION_DECL
            and c.spelling.startswith(NB_MODULE_INIT_PREFIX)
            and not c.is_definition()
            and c.get_definition() is None
        ):
            raise SkippedBodiesError(f"{tu.spelling}: parsed without bodies; NB_MODULE is empty")
        if c.kind != ci.CursorKind.CALL_EXPR or c.spelling not in NB_CLASS_CTORS:
            continue
        t = c.type.get_canonical()
        if not t.spelling.startswith(templates):
            continue
        args = list(c.get_arguments())
        if len(args) < 2 or not _scope_type(args[0]).startswith(NB_MODULE_SCOPE):
            continue
        literal = next(
            (a for a in args[1].walk_preorder() if a.kind == ci.CursorKind.STRING_LITERAL), None
        )
        decl = t.get_template_argument_type(0).get_canonical().get_declaration()
        if literal is not None and decl.kind != ci.CursorKind.NO_DECL_FOUND:
            out[literal.spelling.strip('"')] = audit_types.qualified_path(decl)
    return out
