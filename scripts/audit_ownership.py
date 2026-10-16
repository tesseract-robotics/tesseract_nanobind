"""Binding module tables and header ownership for the binding API audit."""

from __future__ import annotations

import fnmatch
import re
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
STUB_ROOT = SRC / "tesseract_robotics"

# One extension module per binding TU: src/<module>/<module>_bindings.cpp.
# tests/scripts/test_generate_stubs.py pins this set == generate_stubs.discover_modules().
BINDING_GLOB = "*/*_bindings.cpp"

# Directly included headers under these prefixes (relative to an include dir) are a module's
# audited API; every other declaration in the TU only resolves Python names. Explicit per
# module because binding TUs also include other components' headers (spec amendment A2).
# A module may own several prefixes; a header belongs to the module with the longest
# matching prefix (`owning_prefix`), so a nested prefix is never its parent's.
AUDITED_HEADER_PREFIX: dict[str, tuple[str, ...]] = {
    "tesseract_collision": ("tesseract/collision/",),
    "tesseract_common": ("tesseract/common/",),
    "tesseract_environment": ("tesseract/environment/",),
    "tesseract_geometry": ("tesseract/geometry/",),
    "tesseract_kinematics": ("tesseract/kinematics/",),
    "tesseract_scene_graph": ("tesseract/scene_graph/",),
    # No such directory: the module wraps templates in other modules' headers, so it audits
    # only the foreign headers it is the first binding to #include (`header_auditor`).
    "tesseract_serialization": ("tesseract/serialization/",),
    "tesseract_srdf": ("tesseract/srdf/",),
    "tesseract_state_solver": ("tesseract/state_solver/",),
    "tesseract_urdf": ("tesseract/urdf/",),
}
# Binding modules the audit does not cover yet: module → (reason, owned prefixes). The prefixes
# decide header ownership already, so a module moves to AUDITED_HEADER_PREFIX unchanged. Every
# binding module is in exactly one of the two tables (`check_coverage`).
UNAUDITED_MODULES: dict[str, tuple[str, tuple[str, ...]]] = {
    "ompl_base": ("To be removed (#258); binds OMPL, not tesseract.", ()),
    "tesseract_command_language": (
        "Not yet audited: #256 group 1.",
        ("tesseract/command_language/",),
    ),
    "tesseract_time_parameterization": (
        "Not yet audited: #256 group 1.",
        ("tesseract/time_parameterization/",),
    ),
    # trajopt_ifopt registers `TrajOptCollisionConfig` (trajopt_common/collision_types.h).
    "trajopt_ifopt": ("Not yet audited: #256 group 2.", ("trajopt_ifopt/", "trajopt_common/")),
    "trajopt_sqp": ("Not yet audited: #256 group 2.", ("trajopt_sqp/",)),
    "tesseract_motion_planners": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/",),
    ),
    "tesseract_motion_planners_simple": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/simple/",),
    ),
    "tesseract_motion_planners_ompl": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/ompl/",),
    ),
    "tesseract_motion_planners_trajopt": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/trajopt/", "trajopt_sco/", "trajopt/"),
    ),
    "tesseract_motion_planners_trajopt_ifopt": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/trajopt_ifopt/",),
    ),
    "tesseract_motion_planners_descartes": (
        "Not yet audited: #256 group 3.",
        ("tesseract/motion_planners/descartes/", "descartes_light/"),
    ),
    "tesseract_task_composer": ("Not yet audited: #256 group 4.", ("tesseract/task_composer/",)),
    "tesseract_task_composer_planning": (
        "Not yet audited: #256 group 4.",
        ("tesseract/task_composer/planning/",),
    ),
}
# Headers under a prefix that are not Python API, as fnmatch patterns on the path below the
# prefix (`*` also matches `/`). A header the binding #includes directly is audited anyway.
UNAUDITED_HEADERS = {
    "test_suite/*": "gtest and Google Benchmark sources installed for plugin authors' tests.",
    "*_impl.hpp": "cereal implementation fragment, valid only after its `cereal_serialization.h`.",
    "bullet/*": "Bullet backend internals, loaded as a contact manager plugin; Python reaches "
    "them through the `DiscreteContactManager`/`ContinuousContactManager` interfaces.",
    "fcl/*": "FCL backend internals, loaded as a contact manager plugin; Python reaches them "
    "through the `DiscreteContactManager` interface.",
    "bullet/convex_decomposition_hacd.h": "Declared but not built in 0.35.0: no HACD symbol in any collision library and no CMake target.",
    # Patterns match below every module prefix; these four exist only under tesseract/common.
    "cereal_make_array.h": "cereal text-I/O helper for serialization, not API.",
    "serialization_extensions.h": "Macros and SFINAE checks behind `TESSERACT_CLASS_EXTENSION`, not API.",
    "sfinae_utils.h": "Compile-time member-detection templates, not API.",
    "unit_test_utils.h": "gtest serialization helpers for plugin authors' tests.",
    "environment_cache.h": "No consumer: only `fwd.h` names `EnvironmentCache`, and no installed tesseract library uses it.",
    "environment_monitor.h": "Abstract ROS-side interface; no implementation in the installed tesseract libraries.",
    "environment_monitor_interface.h": "Abstract ROS-side interface; no implementation in the installed tesseract libraries.",
    # Kinematics solver plugins, loaded by `KinematicsPluginFactory`; Python reaches them
    # through `ForwardKinematics`/`InverseKinematics`. `kdl/*` also matches
    # `state_solver/kdl/*`, whose `KDLStateSolver` stays audited: the binding includes it.
    "ikfast/*": "IKFast solver plugin template, loaded by `KinematicsPluginFactory`.",
    "kdl/*": "KDL solver plugins, loaded by `KinematicsPluginFactory`.",
    "opw/*": "OPW solver plugin, loaded by `KinematicsPluginFactory`.",
    "ur/*": "Universal Robots solver plugin, loaded by `KinematicsPluginFactory`.",
    "rep_*": "Robot-with-external-positioner solver plugin, loaded by `KinematicsPluginFactory`.",
    "rop_*": "Robot-on-positioner solver plugin, loaded by `KinematicsPluginFactory`.",
    "ofkt/ofkt_node*.h": "Optimized forward kinematics tree nodes, internal to `OFKTStateSolver`.",
}
# Vendored third-party libraries installed under a module prefix, as fnmatch patterns like
# UNAUDITED_HEADERS. Never audited, not even when the binding #includes one directly: such an
# include is build plumbing, not a statement that the library is module API.
THIRD_PARTY_HEADERS = {
    "vhacd/VHACD.h": "Vendored third-party V-HACD library (namespace `VHACD`). The collision "
    "binding #includes it directly only so that `convex_decomposition_vhacd.h` does not compile "
    "a second copy of its implementation (#179).",
}
# Direct `<…>` includes decide header ownership: a header its prefix owner's binding does not
# #include, but another binding does, is audited with `header_auditor` of those modules.
INCLUDE_DIRECTIVE = re.compile(r"^\s*#\s*include\s*<([^>]+)>", re.MULTILINE)
HEADER_SUFFIXES = frozenset({".h", ".hpp"})


class UnknownModuleError(LookupError):
    """The requested module is not an auditable binding module."""


class DuplicatePrefixError(ValueError):
    """Two modules claim the same header prefix."""


class MalformedPrefixError(ValueError):
    """A header prefix does not end with `/`."""


class UnclassifiedModuleError(LookupError):
    """A binding module is in neither AUDITED_HEADER_PREFIX nor UNAUDITED_MODULES."""


class ConflictingCoverageError(ValueError):
    """A module is in both AUDITED_HEADER_PREFIX and UNAUDITED_MODULES."""


class UnauditedModuleError(LookupError):
    """The module is listed in UNAUDITED_MODULES; the message carries the reason."""


def binding_modules() -> list[str]:
    """Short names of all binding modules, sorted."""
    return sorted(p.parent.name for p in SRC.glob(BINDING_GLOB))


def binding_source(module: str) -> Path:
    """`tesseract_collision` → `src/tesseract_collision/tesseract_collision_bindings.cpp`."""
    return SRC / module / f"{module}_bindings.cpp"


def committed_stub(module: str) -> Path:
    """`tesseract_collision` → `src/tesseract_robotics/tesseract_collision/_tesseract_collision.pyi`."""
    return STUB_ROOT / module / f"_{module}.pyi"


def header_auditor(includers: Iterable[str]) -> str:
    """The one module that audits a header its prefix owner's binding does not #include.

    The first of the #including modules by name, audited or not, so the choice does not move
    when another module gets an `AUDITED_HEADER_PREFIX` entry.
    """
    return min(includers)


def owning_prefix(spelling: str, prefixes: Mapping[str, tuple[str, ...]]) -> tuple[str, str] | None:
    """(module, prefix) of the longest prefix holding the header, or None."""
    hits = [(m, p) for m, ps in prefixes.items() for p in ps if spelling.startswith(p)]
    return max(hits, key=lambda mp: len(mp[1]), default=None)


def longest_prefix(spelling: str, prefixes: Iterable[str]) -> str | None:
    """The longest of `prefixes` that holds the header, or None."""
    return max((p for p in prefixes if spelling.startswith(p)), key=len, default=None)


def header_owner(spelling: str, prefixes: Mapping[str, tuple[str, ...]]) -> str | None:
    """The module whose prefix holds the header (longest prefix wins), or None."""
    owner = owning_prefix(spelling, prefixes)
    return None if owner is None else owner[0]


def check_coverage(modules: Sequence[str]) -> None:
    """Every binding module is in exactly one of AUDITED_HEADER_PREFIX and UNAUDITED_MODULES,
    and both tables hold only binding modules.

    Args:
        modules: The binding modules (`binding_modules()`).

    Raises:
        UnknownModuleError: a table entry is not a binding module.
        ConflictingCoverageError: a module is in both tables.
        UnclassifiedModuleError: a binding module is in neither table.
    """
    known = set(modules)
    if unknown := sorted((AUDITED_HEADER_PREFIX.keys() | UNAUDITED_MODULES.keys()) - known):
        raise UnknownModuleError(f"table entries that are not binding modules: {unknown}")
    if both := sorted(AUDITED_HEADER_PREFIX.keys() & UNAUDITED_MODULES.keys()):
        raise ConflictingCoverageError(
            f"in both AUDITED_HEADER_PREFIX and UNAUDITED_MODULES: {both}"
        )
    if unclassified := sorted(known - AUDITED_HEADER_PREFIX.keys() - UNAUDITED_MODULES.keys()):
        raise UnclassifiedModuleError(
            f"binding modules in neither AUDITED_HEADER_PREFIX nor UNAUDITED_MODULES: "
            f"{unclassified}"
        )


@dataclass(frozen=True)
class Coverage:
    """Which binding modules the audit covers: the report states it before any result."""

    audited: tuple[str, ...]
    unaudited: Mapping[str, str]  # module → reason, ordered by reason (so by group), then name
    total: int  # binding modules (`binding_modules()`)


def coverage() -> Coverage:
    """The coverage the two module tables state.

    Raises:
        UnknownModuleError, ConflictingCoverageError, UnclassifiedModuleError: see
            `check_coverage`.
    """
    modules = binding_modules()
    check_coverage(modules)
    rows = sorted(((m, reason) for m, (reason, _) in UNAUDITED_MODULES.items()), key=_by_reason)
    return Coverage(tuple(sorted(AUDITED_HEADER_PREFIX)), dict(rows), len(modules))


def _by_reason(row: tuple[str, str]) -> tuple[str, str]:
    """Reason, then module: the reasons name their #256 group, so rows sort by group."""
    module, reason = row
    return reason, module


def check_prefixes() -> None:
    """Every prefix in both tables ends with `/`: `tesseract/x` would also own `tesseract/xy/`.

    Raises:
        MalformedPrefixError: a prefix without a trailing `/`.
    """
    tables = [
        *AUDITED_HEADER_PREFIX.items(),
        *((m, ps) for m, (_, ps) in UNAUDITED_MODULES.items()),
    ]
    if bad := sorted((m, p) for m, ps in tables for p in ps if not p.endswith("/")):
        raise MalformedPrefixError(f"header prefixes must end with '/': {bad}")


def owner_prefixes() -> dict[str, tuple[str, ...]]:
    """Every module's header prefixes, from AUDITED_HEADER_PREFIX and UNAUDITED_MODULES: who
    owns a header, whether its module is audited or not.

    Raises:
        MalformedPrefixError: see `check_prefixes`.
        ConflictingCoverageError: a module is in both tables.
        DuplicatePrefixError: two modules claim one prefix.
    """
    check_prefixes()
    if both := sorted(AUDITED_HEADER_PREFIX.keys() & UNAUDITED_MODULES.keys()):
        raise ConflictingCoverageError(
            f"in both AUDITED_HEADER_PREFIX and UNAUDITED_MODULES: {both}"
        )
    out = {
        **AUDITED_HEADER_PREFIX,
        **{m: prefixes for m, (_, prefixes) in UNAUDITED_MODULES.items()},
    }
    claimed: dict[str, str] = {}
    for m, ps in out.items():
        for p in ps:
            if p in claimed:
                raise DuplicatePrefixError(f"{p!r} claimed by {claimed[p]} and {m}")
            claimed[p] = m
    return out


def is_third_party(below: str) -> bool:
    """`below`: a header path relative to its module prefix."""
    return any(fnmatch.fnmatchcase(below, pattern) for pattern in THIRD_PARTY_HEADERS)


def is_unaudited(spelling: str, prefixes: tuple[str, ...]) -> bool:
    """Whether `UNAUDITED_HEADERS` or `THIRD_PARTY_HEADERS` match the header's path below the
    longest of `prefixes` that holds it."""
    prefix = longest_prefix(spelling, prefixes)
    below = spelling if prefix is None else spelling.removeprefix(prefix)
    return is_third_party(below) or any(
        fnmatch.fnmatchcase(below, pattern) for pattern in UNAUDITED_HEADERS
    )


check_prefixes()
