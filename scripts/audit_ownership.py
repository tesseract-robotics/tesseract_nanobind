"""Binding module tables and header ownership for the binding API audit."""

from __future__ import annotations

import fnmatch
import functools
import re
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
STUB_ROOT = SRC / "tesseract_robotics"

# An extension module is an ordered set of translation units in src/bindings/: its primary
# `<module>.cpp`, the one TU with `NB_MODULE`, then the parts in MODULE_PARTS, each a TU of its own
# that binds one concern and is linked into the module (CMakeLists.txt `target_sources`). The
# audit reads every TU of a module as that module's binding (`binding_sources`). Every `.cpp`
# there is exactly one of a primary, a declared part or a NON_MODULE_SOURCES helper
# (`module_sources`). tests/scripts/test_generate_stubs.py pins the primaries ==
# generate_stubs.discover_modules().
BINDINGS_DIR = SRC / "bindings"
BINDING_GLOB = "*.cpp"
# `NB_MODULE(_common, m)` at the start of a line (a comment that mentions the macro is no entry).
MODULE_ENTRY = re.compile(r"^NB_MODULE\(\s*(\w+)\s*,", re.MULTILINE)
# Module -> its parts in registration order (the order NB_MODULE calls their `bind_*`).
MODULE_PARTS: dict[str, tuple[str, ...]] = {
    "common": ("common_eigen", "common_logging"),
    "environment": ("environment_commands",),
    "task_composer": ("task_composer_plugin_factory", "task_composer_anypoly"),
    "trajopt_ifopt": ("trajopt_ifopt_collision",),
}
# Sources in src/bindings/ that are linked into a module but bind nothing (no `nb::`): no part.
NON_MODULE_SOURCES = frozenset(
    {"serialization_planner_cereal_registration", "task_composer_plugin_pinning"}
)
# A source that uses nanobind binds something: it is a part, never a helper.
NANOBIND_USE = re.compile(r"\bnb::|\bnanobind::")

# Directly included headers under these prefixes (relative to an include dir) are a module's
# audited API; every other declaration in the TU only resolves Python names. Explicit per
# module because binding TUs also include other components' headers (spec amendment A2).
# A module may own several prefixes; a header belongs to the module with the longest
# matching prefix (`owning_prefix`), so a nested prefix is never its parent's.
# The core modules audited before #256.
CORE_MODULES = frozenset(
    {
        "collision",
        "common",
        "environment",
        "geometry",
        "kinematics",
        "scene_graph",
        "serialization",
        "srdf",
        "state_solver",
        "urdf",
    }
)
AUDITED_HEADER_PREFIX: dict[str, tuple[str, ...]] = {
    "collision": ("tesseract/collision/",),
    "command_language": ("tesseract/command_language/",),
    "common": ("tesseract/common/",),
    "environment": ("tesseract/environment/",),
    "geometry": ("tesseract/geometry/",),
    "kinematics": ("tesseract/kinematics/",),
    "scene_graph": ("tesseract/scene_graph/",),
    # No such directory: the module wraps templates in other modules' headers, so it audits
    # only the foreign headers it is the first binding to #include (`header_auditor`).
    "serialization": ("tesseract/serialization/",),
    "srdf": ("tesseract/srdf/",),
    "state_solver": ("tesseract/state_solver/",),
    "task_composer": ("tesseract/task_composer/",),
    "task_composer_planning": ("tesseract/task_composer/planning/",),
    "time_parameterization": ("tesseract/time_parameterization/",),
    "urdf": ("tesseract/urdf/",),
    "motion_planners": ("tesseract/motion_planners/",),
    "motion_planners_simple": ("tesseract/motion_planners/simple/",),
    "motion_planners_ompl": ("tesseract/motion_planners/ompl/",),
    # Classic TrajOpt: its SQP layer (`trajopt_sco/`) and problem description (`trajopt/`)
    # are reached only through this planner (spec D2).
    "motion_planners_trajopt": (
        "tesseract/motion_planners/trajopt/",
        "trajopt_sco/",
        "trajopt/",
    ),
    "motion_planners_trajopt_ifopt": ("tesseract/motion_planners/trajopt_ifopt/",),
    # descartes_light, the graph-search library behind this planner (spec D2).
    "motion_planners_descartes": (
        "tesseract/motion_planners/descartes/",
        "descartes_light/",
    ),
    # trajopt_ifopt registers `TrajOptCollisionConfig` (trajopt_common/collision_types.h).
    "trajopt_ifopt": ("trajopt_ifopt/", "trajopt_common/"),
    "trajopt_sqp": ("trajopt_sqp/",),
}
# Binding modules the audit does not cover yet: module → (reason, owned prefixes). The prefixes
# decide header ownership already, so a module moves to AUDITED_HEADER_PREFIX unchanged. Every
# binding module is in exactly one of the two tables (`check_coverage`).
UNAUDITED_MODULES: dict[str, tuple[str, tuple[str, ...]]] = {}
# Patterns that apply below every owning prefix.
ANY_PREFIX = "*"
# Built-in task-composer node plugins (C19, not API).
NODES_NOT_API = (
    "Built-in task plugins, created by name from the task-composer YAML through "
    "`TaskComposerPluginFactory.createTaskComposerNode`; Python tunes them through their profiles."
)
# A `*_plugin_factories.h` header (#256 group 4).
PLUGIN_ANCHOR = (
    "`PLUGIN_ANCHOR_DECL` link anchor only: keeps a plugin library's factory symbols exported; "
    "no declarations."
)
# Headers under a prefix that are not Python API: owning prefix (or `ANY_PREFIX`) -> {fnmatch
# pattern on the path below that prefix (`*` also matches `/`): reason}. A header is matched
# only against the patterns of the longest prefix that holds it, so a parent's pattern never
# hides a nested module's header. A header the binding #includes directly is audited anyway.
UNAUDITED_HEADERS: dict[str, dict[str, str]] = {
    ANY_PREFIX: {
        "test_suite/*": "gtest and Google Benchmark sources installed for plugin authors' tests.",
        "*_impl.hpp": "cereal implementation fragment, valid only after its "
        "`cereal_serialization.h`.",
    },
    "tesseract/collision/": {
        "bullet/*": "Bullet backend internals, loaded as a contact manager plugin; Python reaches "
        "them through the `DiscreteContactManager`/`ContinuousContactManager` interfaces.",
        "fcl/*": "FCL backend internals, loaded as a contact manager plugin; Python reaches them "
        "through the `DiscreteContactManager` interface.",
        "bullet/convex_decomposition_hacd.h": "Declared but not built in 0.35.0: no HACD symbol "
        "in any collision library and no CMake target.",
    },
    "tesseract/common/": {
        "cereal_make_array.h": "cereal text-I/O helper for serialization, not API.",
        "serialization_extensions.h": "Macros and SFINAE checks behind "
        "`TESSERACT_CLASS_EXTENSION`, not API.",
        "sfinae_utils.h": "Compile-time member-detection templates, not API.",
        "unit_test_utils.h": "gtest serialization helpers for plugin authors' tests.",
    },
    "tesseract/environment/": {
        "environment_cache.h": "No consumer: only `fwd.h` names `EnvironmentCache`, and no "
        "installed tesseract library uses it.",
        "environment_monitor.h": "Abstract ROS-side interface; no implementation in the "
        "installed tesseract libraries.",
        "environment_monitor_interface.h": "Abstract ROS-side interface; no implementation in "
        "the installed tesseract libraries.",
    },
    # Kinematics solver plugins, loaded by `KinematicsPluginFactory`; Python reaches them
    # through `ForwardKinematics`/`InverseKinematics`.
    "tesseract/kinematics/": {
        "ikfast/*": "IKFast solver plugin template, loaded by `KinematicsPluginFactory`.",
        "kdl/*": "KDL solver plugins, loaded by `KinematicsPluginFactory`.",
        "opw/*": "OPW solver plugin, loaded by `KinematicsPluginFactory`.",
        "ur/*": "Universal Robots solver plugin, loaded by `KinematicsPluginFactory`.",
        "rep_*": "Robot-with-external-positioner solver plugin, loaded by "
        "`KinematicsPluginFactory`.",
        "rop_*": "Robot-on-positioner solver plugin, loaded by `KinematicsPluginFactory`.",
    },
    "tesseract/state_solver/": {
        "ofkt/ofkt_node*.h": "Optimized forward kinematics tree nodes, internal to "
        "`OFKTStateSolver`.",
    },
    # Group 2 (trajopt). Never `*.hpp`: below another prefix it would match `impl/*.hpp`.
    "trajopt_sqp/": {
        "callbacks/*": "Visualization callbacks: each takes a "
        "`tesseract::visualization::Visualization`, which no module binds; Python writes an "
        "`SQPCallback` subclass instead.",
        "expressions.h": "`AffExprs`/`QuadExprs` convexification helpers behind "
        "`TrajOptQPProblem`'s PIMPL; no installed header's signature names them (only `fwd.h` "
        "forward-declares them).",
        "ifopt_qp_problem.h": "`IfoptQPProblem` is being removed upstream "
        "(tesseract-robotics/trajopt#595); the binding dropped it in 0.35.0.9 for "
        "`TrajOptQPProblem`.",
    },
    "trajopt_ifopt/": {
        "constraints/collision/weighted_average_methods.h": "Gradient averaging internal to the "
        "continuous collision constraints (`getWeightedAvgGradientT0/T1`).",
        "core/problem.h": "The `trajopt_ifopt::Problem` NLP serves only `IfoptQPProblem`, which "
        "is not Python API; `TrajOptQPProblem` takes `ConstraintSet`s directly.",
        "core/cost_term.h": "`CostTerm`, the cost base of the `trajopt_ifopt::Problem` path, "
        "which only `IfoptQPProblem` uses.",
        "costs/*": "`SquaredCost`/`AbsoluteCost` wrap constraints as `CostTerm`s for the "
        "`trajopt_ifopt::Problem` path; `TrajOptQPProblem.addCostSet` takes a `ConstraintSet` "
        "and a `CostPenaltyType`.",
        "core/composite.h": "`CompositeDifferentiable`, the component container of the "
        "`trajopt_ifopt::Problem` path.",
        "variable_sets/nodes_observer.h": "Abstract `NodesObserver` with no implementation in "
        "trajopt 0.35.0.",
    },
    "trajopt_common/": {
        "basic_array.hpp": "trajopt-internal array helper from the sco era; numpy covers it.",
        "eigen_conversions.hpp": "trajopt-internal Eigen/`std::vector` conversions; nanobind's "
        "Eigen casters cover them.",
        "eigen_slicing.hpp": "trajopt-internal index-slicing helpers; numpy indexing covers them.",
        "interpolation.hpp": "trajopt-internal interpolation helpers; the bound "
        "`trajopt_ifopt.interpolate` is the Python form.",
        "utils.hpp": "trajopt-internal `std::vector` and string helpers; the Python standard "
        "library covers them.",
        "vector_ops.hpp": "trajopt-internal `std::vector` arithmetic; numpy covers it.",
        "stl_to_string.hpp": "trajopt-internal `std::vector` to-string helpers; Python's "
        "`str`/`repr` cover them.",
        "clock.hpp": "Process-global `StartClock`/`GetClock` timer for trajopt's own timing logs.",
        "config.hpp": "boost::program_options parameter registry for trajopt's command-line tools.",
        "collision_utils.h": "Gradient and hashing helpers behind the collision constraints "
        "(`getHash`, `getGradient(GradientResults&, …)`, `debugPrintInfo`).",
        "cache.h": "LRU pool template holding `CollisionCacheData` inside the collision "
        "constraints.",
    },
    # Group 4 (task composer).
    "tesseract/task_composer/": {
        "task_composer_plugin_factory_utils.h": "`TaskComposerTaskFactory<T>` and "
        "`TaskComposerExecutorFactoryImpl<T>` are templates a C++ plugin library instantiates; "
        "Python creates nodes and executors through `TaskComposerPluginFactory`.",
        "task_composer_task_plugin_factory.h": PLUGIN_ANCHOR,
        "*_plugin_factories.h": PLUGIN_ANCHOR,
        "taskflow/taskflow_task_composer_future.h": "Taskflow implementation of "
        "`TaskComposerFuture`; its constructor takes `tf::Taskflow` internals, and Python receives "
        "it through the `TaskComposerFuture` that `TaskComposerExecutor.run` returns.",
        "yaml_utils.h": "`loadSubTask`, `loadSubTaskConfig` and `validateSubTask` parse a node's "
        "`YAML::Node` config inside C++ node constructors; every function takes a `YAML::Node`.",
        "nodes/*": NODES_NOT_API,
    },
    "tesseract/task_composer/planning/": {
        "*_plugin_factories.h": PLUGIN_ANCHOR,
        "nodes/*": NODES_NOT_API,
    },
    # Group 3: OMPL plugin internals and classic TrajOpt's solver layers. `*` is safe only
    # because each pattern is scoped to its owning prefix.
    "tesseract/motion_planners/ompl/": {
        "*_validator.h": "OMPL `StateValidityChecker`/`MotionValidator` subclasses that "
        "`OMPLRealVectorMoveProfile::createSimpleSetup` installs into the `SimpleSetup`; Python "
        "reaches them only through OMPL types, which the package does not bind (#258).",
        "collision_cost_objective.h": "OMPL `StateCostIntegralObjective` subclass the OMPL "
        "profile installs; it has no Python form without OMPL types.",
        "weighted_real_vector_state_sampler.h": "OMPL `StateSampler` subclass the OMPL profile "
        "installs; it has no Python form without OMPL types.",
        "utils.h": "Helpers the OMPL planner and profile call while building and reading the "
        "`SimpleSetup`; all but `checkStateInCollision` take OMPL types, and that one is "
        "`JointGroup.calcFwdKin` plus `DiscreteContactManager.contactTest`, both bound.",
    },
    "tesseract/motion_planners/trajopt/": {
        "trajopt_utils.h": "Factories for classic-TrajOpt `TermInfo`s that the default TrajOpt "
        "profiles' `create` calls; all but `createKinematicGroup` return `trajopt::TermInfo`, "
        "and that one duplicates `Environment.getKinematicGroup`/`getJointGroup`.",
    },
    "trajopt_sco/": {
        "*": "Classic TrajOpt's SQP layer: expressions, convex models, QP-solver backends (OSQP, "
        "qpOASES, Gurobi, BPMPD) and numeric differentiation. `TrajOptMotionPlanner` builds and "
        "solves it from the profiles; Python sets its parameters through "
        "`TrajOptSolverProfile.opt_params`. `optimizers.hpp`, #included by the binding, stays "
        "audited.",
    },
    "trajopt/": {
        "*": "Classic TrajOpt's problem description (`ProblemConstructionInfo`, `TermInfo` and "
        "its cost and constraint terms, JSON marshalling, plot and file callbacks), which "
        "`TrajOptMotionPlanner::createProblem` builds from the profiles; Python configures the "
        "profiles.",
    },
    # C25 = not API: the Descartes building blocks stay internal to the default profiles.
    "descartes_light/": {
        "bgl/*": "Boost-Graph solver alternatives; tesseract builds only the ladder-graph solver "
        "(`DescartesLadderGraphSolverProfile::create`).",
        "solvers/*": "Ladder-graph solver internals, built by `DescartesLadderGraphSolverProfileD` "
        "inside `DescartesMotionPlannerD.solve`.",
        "core/solver.h": "The `Solver`/`BuildStatus`/`SearchResult` interface "
        "`DescartesMotionPlanner::solve` builds and searches; Python configures it through "
        "`DescartesLadderGraphSolverProfileD.num_threads`.",
        "edge_evaluators/*": "Concrete descartes_light edge evaluators; Python implements the "
        "bound `DescartesEdgeEvaluatorD` interface or delegates to "
        "`DescartesDefaultMoveProfileD.createEdgeEvaluator`.",
        "state_evaluators/*": "Concrete descartes_light state evaluators; Python implements the "
        "bound `DescartesStateEvaluatorD` interface or delegates to "
        "`DescartesDefaultMoveProfileD.createStateEvaluator`.",
        "samplers/*": "Concrete descartes_light waypoint samplers; Python implements the bound "
        "`DescartesWaypointSamplerD` interface or delegates to "
        "`DescartesDefaultMoveProfileD.createWaypointSampler`.",
    },
    "tesseract/motion_planners/descartes/": {
        "descartes_collision.h": "Collision building block behind `DescartesDefaultMoveProfile`'s "
        "`create*` hooks; Python reaches it through `DescartesDefaultMoveProfileD`.",
        "descartes_collision_edge_evaluator.h": "Edge evaluator behind "
        "`DescartesDefaultMoveProfile::createEdgeEvaluator`; Python reaches it through "
        "`DescartesDefaultMoveProfileD`.",
        "descartes_robot_sampler.h": "Waypoint sampler behind "
        "`DescartesDefaultMoveProfile::createWaypointSampler` and its protected "
        "`createPoseSampler` hook; Python reaches it through `DescartesDefaultMoveProfileD`.",
        "descartes_vertex_evaluator.h": "Vertex evaluator behind `DescartesDefaultMoveProfile`'s "
        "protected `createVertexEvaluator` hook; Python reaches it through "
        "`DescartesDefaultMoveProfileD`.",
        "yaml_extensions.h": "Does not compile upstream: it names the removed "
        "`tesseract_planning` namespace (yaml_extensions.h:42).",
    },
}
# Vendored third-party libraries installed under a module prefix, as fnmatch patterns like
# UNAUDITED_HEADERS. Never audited, not even when the binding #includes one directly: such an
# include is build plumbing, not a statement that the library is module API.
THIRD_PARTY_HEADERS = {
    "vhacd/VHACD.h": "Vendored third-party V-HACD library (namespace `VHACD`). The collision "
    "binding #includes it directly only so that `convex_decomposition_vhacd.h` does not compile "
    "a second copy of its implementation (#179).",
}
# Headers a binding #includes only to call the plugin anchor they declare (`PLUGIN_ANCHOR_DECL`):
# the binding passes the anchor to `addSymbolLibraryToSearchLibrariesEnv`, which puts the plugin
# library linked into the module on the plugin loader's search path. Such an include is build
# plumbing, not a statement that the header is module API, so the header is not audited although
# the binding #includes it directly. Exact include spellings, never patterns: header -> reason.
# `audit_anchor.check_anchor_headers` holds every listed header to that claim (its functions
# are anchors by signature, `const void* f()`; its classes derive only from plugin factory
# interfaces, members not inspected) and raises on any other declaration, so a new function or
# class in one of these headers still fails the audit.
ANCHOR_CALL_ONLY = (
    "Included only for its plugin anchor, which the binding calls at import to put the bundled "
    "plugin library on the plugin search path; the plugins in that library are created by name "
    "from the plugin config through `{factory}`."
)
PLUGIN_ANCHOR_INCLUDES = {
    **dict.fromkeys(
        (
            "tesseract/collision/bullet/bullet_factories.h",
            "tesseract/collision/fcl/fcl_factories.h",
        ),
        ANCHOR_CALL_ONLY.format(factory="ContactManagersPluginFactory"),
    ),
    **dict.fromkeys(
        (
            "tesseract/kinematics/kdl/kdl_factories.h",
            "tesseract/kinematics/opw/opw_factory.h",
            "tesseract/kinematics/rep_factory.h",
            "tesseract/kinematics/rop_factory.h",
            "tesseract/kinematics/ur/ur_factory.h",
        ),
        ANCHOR_CALL_ONLY.format(factory="KinematicsPluginFactory"),
    ),
    **dict.fromkeys(
        (
            "tesseract/task_composer/task_composer_task_plugin_factory.h",
            "tesseract/task_composer/taskflow/taskflow_task_composer_plugin_factories.h",
        ),
        ANCHOR_CALL_ONLY.format(factory="TaskComposerPluginFactory"),
    ),
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


class UnknownPrefixError(LookupError):
    """An `UNAUDITED_HEADERS` key is neither `ANY_PREFIX` nor any module's header prefix."""


class StrayBindingSourceError(LookupError):
    """A source in src/bindings/ has no `NB_MODULE` and is neither a declared part
    (`MODULE_PARTS`) nor a helper (`NON_MODULE_SOURCES`): the audit would read it as no
    module's binding."""


class MissingBindingSourceError(FileNotFoundError):
    """A `MODULE_PARTS` or `NON_MODULE_SOURCES` entry has no source file."""


class ModuleEntryMismatchError(ValueError):
    """A primary's `NB_MODULE` names another module than its file, or appears more than once."""


class DeclaredPrimaryError(ValueError):
    """A declared part or helper holds an `NB_MODULE`: it is a module of its own."""


class UnknownPartOwnerError(LookupError):
    """A `MODULE_PARTS` key is no module primary."""


class DuplicateSourceError(ValueError):
    """A source is declared twice: as a part of two modules, twice in one, or as part and
    helper."""


class BindingHelperError(ValueError):
    """A `NON_MODULE_SOURCES` helper uses nanobind: it binds, so it is a part the audit reads."""


def module_sources(
    directory: Path = BINDINGS_DIR,
    parts: Mapping[str, tuple[str, ...]] = MODULE_PARTS,
    helpers: frozenset[str] = NON_MODULE_SOURCES,
) -> dict[str, tuple[Path, ...]]:
    """Each extension module -> its translation units, primary first, then its parts in order.

    Args:
        directory: The binding sources (`BINDINGS_DIR`).
        parts: Module -> its part stems (`MODULE_PARTS`).
        helpers: Stems of the sources that bind nothing (`NON_MODULE_SOURCES`).

    Raises:
        DuplicateSourceError: a stem declared twice.
        MissingBindingSourceError: a declared part or helper without a file.
        DeclaredPrimaryError: a declared part or helper holds an `NB_MODULE`.
        BindingHelperError: a helper uses nanobind.
        ModuleEntryMismatchError: a primary's `NB_MODULE` is not exactly one, named after it.
        StrayBindingSourceError: a source that is no primary, part or helper.
        UnknownPartOwnerError: a `parts` key that is no primary.
    """
    declared = [s for stems in parts.values() for s in stems] + sorted(helpers)
    if twice := sorted({s for s in declared if declared.count(s) > 1}):
        raise DuplicateSourceError(f"declared more than once in MODULE_PARTS / helpers: {twice}")
    texts = {p.stem: p.read_text(encoding="utf-8") for p in sorted(directory.glob(BINDING_GLOB))}
    if missing := sorted(set(declared) - texts.keys()):
        raise MissingBindingSourceError(f"declared sources with no file in {directory}: {missing}")
    if entries := sorted(s for s in declared if MODULE_ENTRY.search(texts[s])):
        raise DeclaredPrimaryError(f"declared parts or helpers that hold an NB_MODULE: {entries}")
    if binding := sorted(s for s in helpers if NANOBIND_USE.search(texts[s])):
        raise BindingHelperError(
            f"helpers that use nanobind; declare them in MODULE_PARTS: {binding}"
        )
    primaries: list[str] = []
    for stem, text in texts.items():
        if stem in declared:
            continue
        names = MODULE_ENTRY.findall(text)
        if not names:
            raise StrayBindingSourceError(
                f"{directory / f'{stem}.cpp'} has no NB_MODULE and is in neither MODULE_PARTS "
                "nor NON_MODULE_SOURCES"
            )
        if [n.removeprefix("_") for n in names] != [stem]:
            raise ModuleEntryMismatchError(
                f"{directory / f'{stem}.cpp'}: NB_MODULE {names}, expected one naming {stem!r}"
            )
        primaries.append(stem)
    if unknown := sorted(parts.keys() - set(primaries)):
        raise UnknownPartOwnerError(f"MODULE_PARTS keys that are no module primary: {unknown}")
    return {
        m: tuple(directory / f"{s}.cpp" for s in (m, *parts.get(m, ()))) for m in sorted(primaries)
    }


@functools.cache
def _layout() -> Mapping[str, tuple[Path, ...]]:
    """`module_sources()` of the repo, validated once per process."""
    return MappingProxyType(module_sources())


def binding_modules() -> list[str]:
    """Short names of all binding modules, sorted.

    Raises:
        StrayBindingSourceError, MissingBindingSourceError, ModuleEntryMismatchError,
            DeclaredPrimaryError, UnknownPartOwnerError, DuplicateSourceError,
            BindingHelperError: see `module_sources`.
    """
    return list(_layout())


def binding_sources(module: str) -> tuple[Path, ...]:
    """The translation units of a binding module, primary first: `common` →
    `(src/bindings/common.cpp, src/bindings/common_eigen.cpp, src/bindings/common_logging.cpp)`.

    Raises:
        UnknownModuleError: not a binding module.
        StrayBindingSourceError, MissingBindingSourceError, ModuleEntryMismatchError,
            DeclaredPrimaryError, UnknownPartOwnerError, DuplicateSourceError,
            BindingHelperError: see `module_sources`.
    """
    layout = _layout()
    if module not in layout:
        raise UnknownModuleError(f"{module!r} is not a binding module; known: {', '.join(layout)}")
    return layout[module]


def primary_source(module: str) -> Path:
    """The TU of a binding module that holds its `NB_MODULE`: `collision` →
    `src/bindings/collision.cpp`. The audit reads all of `binding_sources`; this one alone is
    the TU a synthetic TU includes, or a test parses for a class the primary binds.

    Raises:
        UnknownModuleError: see `binding_sources`.
    """
    return binding_sources(module)[0]


def direct_include_spellings(sources: Sequence[Path]) -> frozenset[str]:
    """`<…>` spellings the files #include, read as text (other bindings are not parsed)."""
    return frozenset(
        m for s in sources for m in INCLUDE_DIRECTIVE.findall(s.read_text(encoding="utf-8"))
    )


def foreign_headers(
    module: str,
    own_includes: frozenset[str],
    other_includes: Mapping[str, frozenset[str]],
    prefixes: Mapping[str, tuple[str, ...]],
    include_dirs: Sequence[Path],
) -> dict[Path, str]:
    """Headers under another module's prefix that this binding #includes directly, that the
    owner's binding does not, and that this module audits (`header_auditor`)."""
    found: dict[Path, str] = {}
    for spelling in own_includes:
        owner = header_owner(spelling, prefixes)
        if owner is None or owner == module or spelling in other_includes.get(owner, frozenset()):
            continue
        if is_unaudited(spelling, prefixes[owner]):
            continue
        includers = [module, *(m for m, inc in other_includes.items() if spelling in inc)]
        if header_auditor(includers) != module:
            continue
        for d in include_dirs:
            if (d / spelling).is_file():
                found[(d / spelling).resolve()] = spelling
                break
    return found


def package_init(module: str) -> Path | None:
    """The `__init__.py` of a wrapped module's package, or None for a module that is itself the
    compiled extension (`tesseract_robotics.collision`)."""
    init = STUB_ROOT / module / "__init__.py"
    return init if init.is_file() else None


def committed_stub(module: str) -> Path:
    """The committed stub of a module.

    A compiled module is its own stub, `common` → `src/tesseract_robotics/common/_common.pyi`
    when the package wraps the extension `_common` (`package_init`), else
    `collision` → `src/tesseract_robotics/collision.pyi`.
    """
    if package_init(module) is not None:
        return STUB_ROOT / module / f"_{module}.pyi"
    return STUB_ROOT / f"{module}.pyi"


def header_auditor(includers: Iterable[str]) -> str:
    """The one module that audits a header its prefix owner's binding does not #include.

    The first of the #including modules by name, audited or not, so the choice does not move
    when another module gets an `AUDITED_HEADER_PREFIX` entry.
    """
    return min(includers)


def owning_prefix(spelling: str, prefixes: Mapping[str, tuple[str, ...]]) -> tuple[str, str] | None:
    """(module, prefix) of the longest prefix holding the header, or None."""
    hits = [(m, p) for m, ps in prefixes.items() for p in ps if spelling.startswith(p)]
    if not hits:
        return None
    return max(hits, key=lambda mp: len(mp[1]))


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


def check_unaudited_keys() -> None:
    """Every `UNAUDITED_HEADERS` key is `ANY_PREFIX` or a module's prefix. Not cached: the
    tables are small, and a cache would miss a key added after the first call.

    Raises:
        UnknownPrefixError: a key that is no module's prefix (a typo would silently audit).
    """
    known = {p for ps in owner_prefixes().values() for p in ps} | {ANY_PREFIX}
    if unknown := sorted(set(UNAUDITED_HEADERS) - known):
        raise UnknownPrefixError(f"UNAUDITED_HEADERS keys that are no module's prefix: {unknown}")


def is_unaudited(spelling: str, prefixes: tuple[str, ...]) -> bool:
    """Whether `spelling` is a `PLUGIN_ANCHOR_INCLUDES` header, or `THIRD_PARTY_HEADERS` or the
    `UNAUDITED_HEADERS` patterns of the longest of `prefixes` that holds the header and of
    `ANY_PREFIX` match its path below that prefix.

    Raises:
        UnknownPrefixError: see `check_unaudited_keys`.
    """
    check_unaudited_keys()
    prefix = longest_prefix(spelling, prefixes)
    below = spelling if prefix is None else spelling.removeprefix(prefix)
    patterns = {**UNAUDITED_HEADERS[ANY_PREFIX], **UNAUDITED_HEADERS.get(prefix or "", {})}
    return (
        spelling in PLUGIN_ANCHOR_INCLUDES
        or is_third_party(below)
        or any(fnmatch.fnmatchcase(below, p) for p in patterns)
    )


def header_is_unaudited(spelling: str, prefixes: Mapping[str, tuple[str, ...]]) -> bool:
    """Whether a module in `prefixes` (module -> owned prefixes, as `owner_prefixes` returns)
    owns `spelling` and one of that module's not-API patterns matches it (R13's not-API half).

    Raises:
        UnknownPrefixError: see `check_unaudited_keys`.
    """
    hit = owning_prefix(spelling, prefixes)
    return hit is not None and is_unaudited(spelling, (hit[1],))


check_prefixes()
