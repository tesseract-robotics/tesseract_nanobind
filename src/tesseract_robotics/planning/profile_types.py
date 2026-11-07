"""Refuse Descartes profiles of the scalar type a pipeline's Descartes node does not read.

`DescartesMotionPlanner<FloatType>` looks its profiles up under `Profile::createKey<T>()` of its own
scalar type and runs the library default when nothing is stored there
(`ProfileDictionary::getProfile<T>`). A `D` profile is never found by an `F` node: the pipeline
plans with the defaults and reports success (#350). Every shipped Cartesian and raster task is
`DescartesFMotionPlannerTaskFactory`; only `DescartesDPipeline` / `DescartesDNPCPipeline` are `D`.

`check_descartes_profile_types` finds every Descartes node a pipeline runs and refuses a dictionary
in which a profile name in that node's namespace holds the other scalar type's profile of a family
(move or solver) but not the node's own. A namespace may hold both types: the profile helpers
register both, and a pipeline reads only its own.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import yaml

from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_common import PluginInfo
from tesseract_robotics.tesseract_motion_planners_descartes import (
    DescartesMoveProfileD,
    DescartesMoveProfileF,
    DescartesSolverProfileD,
    DescartesSolverProfileF,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerGraph,
    TaskComposerNode,
    TaskComposerPluginFactory,
)


class ProfileTypeMismatchError(TypeError):
    """A Descartes node's namespace holds a profile of the other scalar type, not its own."""


_DescartesBase = (
    type[DescartesMoveProfileD]
    | type[DescartesMoveProfileF]
    | type[DescartesSolverProfileD]
    | type[DescartesSolverProfileF]
)

# Per Descartes task factory (its config class name), the profile families its planner reads, each
# as (the base it reads, the other scalar type's base of the same family).
_DESCARTES_FAMILIES: dict[str, tuple[tuple[_DescartesBase, _DescartesBase], ...]] = {
    "DescartesFMotionPlannerTaskFactory": (
        (DescartesMoveProfileF, DescartesMoveProfileD),
        (DescartesSolverProfileF, DescartesSolverProfileD),
    ),
    "DescartesDMotionPlannerTaskFactory": (
        (DescartesMoveProfileD, DescartesMoveProfileF),
        (DescartesSolverProfileD, DescartesSolverProfileF),
    ),
}

# Every Descartes profile key: a dictionary holding none of them cannot mismatch.
_DESCARTES_KEYS = frozenset(
    base.createKey() for pairs in _DESCARTES_FAMILIES.values() for pair in pairs for base in pair
)

# Config keys under which a task factory names the sub-tasks it builds when it runs, not when the
# pipeline is built; they are not children of the built node, so the walk builds them. These are
# every caller of `loadSubTask` in tesseract_planning a648382 apart from graphs
# (raster_motion_task.cpp, raster_only_motion_task.cpp, for_each_task.cpp).
_RUNTIME_SUBTASK_KEYS: dict[str, tuple[str, ...]] = {
    "RasterMotionTaskFactory": ("freespace", "raster", "transition"),
    "RasterOnlyMotionTaskFactory": ("raster", "transition"),
    "ForEachTaskFactory": ("operation",),
}

# The name a sub-task built by class gets when its parent runs is per item ("Raster #1: ...", one
# per ForEach item); the walk builds it under this placeholder instead. Without a configured
# `namespace` it is also the node's namespace: no profile can be registered for such a node.
RUNTIME_NAME = "<named at run time>"


@dataclass(frozen=True)
class DescartesNode:
    """A Descartes planner node of a pipeline.

    Attributes:
        path: Node names from the pipeline down, joined by `/`.
        namespace: The profile namespace its planner reads.
        factory: Its task factory's config class name, a key of `_DESCARTES_FAMILIES`.
    """

    path: str
    namespace: str
    factory: str


def descartes_nodes(
    factory: TaskComposerPluginFactory, pipeline: str, node: TaskComposerNode
) -> list[DescartesNode]:
    """Every Descartes planner node `pipeline` runs, including those built when it runs.

    Args:
        factory: The plugin factory that builds `pipeline`.
        pipeline: A configured node name (a key of `getTaskComposerNodePlugins()`).
        node: `factory.createTaskComposerNode(pipeline)`.

    Returns:
        The Descartes nodes, in graph order.

    Raises:
        TaskComposerPluginError: `factory` cannot build one of the sub-tasks `pipeline` runs.
    """
    plugins = factory.getTaskComposerNodePlugins()
    info = plugins[pipeline]
    return list(_Walk(factory, plugins).node(node, info.class_name, _config(info), pipeline))


def check_descartes_profile_types(
    factory: TaskComposerPluginFactory,
    pipeline: str,
    node: TaskComposerNode,
    profiles: ProfileDictionary,
) -> None:
    """Refuse `profiles` if a Descartes node of `pipeline` would miss a profile of its own type.

    Args:
        factory: The plugin factory that builds `pipeline`.
        pipeline: The pipeline about to be planned with `profiles`.
        node: `factory.createTaskComposerNode(pipeline)`, the node about to run.
        profiles: The profiles it will read.

    Raises:
        ProfileTypeMismatchError: a profile name in a Descartes node's namespace holds the other
            scalar type's move or solver profile and not the node's own.
    """
    entries = profiles.getAllProfileEntries()
    if not any(key in by_key for by_key in entries.values() for key in _DESCARTES_KEYS):
        return
    mismatches = []
    for descartes in descartes_nodes(factory, pipeline, node):
        by_key = entries.get(descartes.namespace, {})
        for reads, other in _DESCARTES_FAMILIES[descartes.factory]:
            names = by_key.get(other.createKey(), {}).keys() - by_key.get(reads.createKey(), {})
            mismatches.extend(
                f"node '{descartes.path}' reads '{reads.__name__}' in namespace "
                f"'{descartes.namespace}', but profile '{name}' there is registered only as "
                f"'{other.__name__}'"
                for name in sorted(names)
            )
    if mismatches:
        raise ProfileTypeMismatchError(
            f"Pipeline '{pipeline}' would never find these profiles and would plan with the "
            "planner's defaults:\n  "
            + "\n  ".join(mismatches)
            + "\nRegister the type the node reads (the planning profile helpers register both)."
        )


class _Walk:
    """Builds and walks a pipeline's nodes; a node's namespace is read from the built node."""

    def __init__(self, factory: TaskComposerPluginFactory, plugins: dict[str, PluginInfo]) -> None:
        self._factory = factory
        self._plugins = plugins

    def task(self, name: str, path: str) -> Iterator[DescartesNode]:
        """Build the configured node `name` as the factory builds it, and walk it."""
        info = self._plugins[name]
        node = self._factory.createTaskComposerNode(name)
        return self.node(node, info.class_name, _config(info), path)

    def node(
        self, node: TaskComposerNode, class_name: str, config: dict[str, Any], path: str
    ) -> Iterator[DescartesNode]:
        """Walk the built `node` of factory `class_name` with its `config`."""
        if class_name in _DESCARTES_FAMILIES:
            yield DescartesNode(path, node.getNamespace(), class_name)
            return
        children = node.getNodes().values() if isinstance(node, TaskComposerGraph) else ()
        for child in children:
            child_path = f"{path}/{child.getName()}"
            entry = config["nodes"][child.getName()]
            if "task" in entry:
                info = self._plugins[entry["task"]]
                yield from self.node(child, info.class_name, _config(info), child_path)
            else:
                yield from self.node(child, entry["class"], entry.get("config") or {}, child_path)
        for key in _RUNTIME_SUBTASK_KEYS.get(class_name, ()):
            if key in config:
                yield from self.runtime_subtask(config[key], f"{path}/{key}")

    def runtime_subtask(self, entry: dict[str, Any], path: str) -> Iterator[DescartesNode]:
        """Walk a sub-task entry (`task:` or `class:`) that its parent builds when it runs."""
        if "task" in entry:
            return self.task(entry["task"], path)
        config = entry.get("config") or {}
        info = PluginInfo()
        info.class_name = entry["class"]
        info.config = yaml.safe_dump(config)
        node = self._factory.createTaskComposerNode(RUNTIME_NAME, info)
        return self.node(node, entry["class"], config, path)


def _config(info: PluginInfo) -> dict[str, Any]:
    """A plugin's config as a mapping (an empty YAML document loads as None)."""
    return yaml.safe_load(info.config) or {}
