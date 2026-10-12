"""
Plugin Configuration Example
============================

Builds the plugin information that tesseract's YAML configs hold, contact
managers, profiles and task composer, in Python instead of YAML, merges it with
`insert`, applies the contact-manager part to an environment, and introspects
the task composer's configured and available plugins.

C++ Reference:
    common/src/plugin_info.cpp  (ProfilesPluginInfo::insert :71-81,
        ContactManagersPluginInfo::insert :162-184, TaskComposerPluginInfo::insert :214-230)
    common/test/tesseract_common_unit.cpp  (ContactManagersPluginInfoUnit :1234-1266,
        ProfilePluginInfoUnit :1268-1298, TaskComposerPluginInfoUnit :1300-1332)
    support/urdf/contact_manager_plugins.yaml
    tesseract_planning task_composer/config/task_composer_plugins.yaml

Overview
--------
1. `ContactManagersPluginInfo` as in `contact_manager_plugins.yaml`, built as a
   Bullet part and an FCL part and merged with `insert`; it equals the shipped
   file's information and, applied to a URDF-only IIWA environment with
   `AddContactManagersPluginInfoCommand`, creates each manager by name.
2. `ProfilesPluginInfo` with upstream's unit-test values (there is no shipped
   profile-plugin YAML), merged from two parts.
3. `TaskComposerPluginInfo` as in `task_composer_plugins.yaml`: its search
   libraries, the executor block and two of its task entries, merged from a
   tasks part and an executor part.
4. `TaskComposerPluginFactory.getTaskComposerNodePlugins()` (configured name ->
   `PluginInfo`) and `getAvailableTaskComposerNodePlugins()` (the factory
   classes the libraries export): every configured class is available, and
   `TaskComposer.get_available_pipelines()` lists only configured names.

Key Concepts
------------
**insert merges, it does not replace**:
    `insert(other)` appends `other`'s search paths and libraries, adds or
    overwrites its plugins by name, and takes its default plugin only when
    `other` names one. Merging two parts in the YAML's order therefore rebuilds
    the whole file; inserting the same part twice would list its libraries twice.

**Configured vs available**:
    A configured node is a name in the YAML with a `PluginInfo` whose
    `class_name` is a factory class. The available node plugins are the factory
    classes the search libraries export. A configured name can be created only
    if its class is available.

**No Python consumer yet for two of them**:
    Upstream builds a `ProfilePluginFactory` from a `ProfilesPluginInfo` and a
    `TaskComposerPluginFactory` from a `TaskComposerPluginInfo`; neither
    constructor is bound, so in Python these two are data only. The
    contact-manager information is consumed by `AddContactManagersPluginInfoCommand`.
"""

from pathlib import Path
from typing import Any

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_common import (
    ContactManagersPluginInfo,
    GeneralResourceLocator,
    PluginInfo,
    PluginInfoContainer,
    ProfilesPluginInfo,
    TaskComposerPluginInfo,
)
from tesseract_robotics.tesseract_environment import (
    AddContactManagersPluginInfoCommand,
    Environment,
)

URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
SRDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
# contact_manager_plugins.yaml and task_composer_plugins.yaml
SEARCH_PATH = "/usr/local/lib"
BULLET_LIBRARY = "tesseract_collision_bullet_factories"
FCL_LIBRARY = "tesseract_collision_fcl_factories"
TASK_LIBRARIES = ["tesseract_task_composer_factories", "tesseract_task_composer_planning_factories"]
TASKFLOW_LIBRARY = "tesseract_task_composer_taskflow_factories"
EXECUTOR_CONFIG = "threads: 5"
MIRRORED_TASKS = ["FreespacePipeline", "TrajOptPipeline"]


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def plugin(class_name: str, config: str = "") -> PluginInfo:
    """A `PluginInfo` for a factory class, with an optional YAML config."""
    info = PluginInfo()
    info.class_name = class_name
    if config:
        info.config = config
    return info


def container(plugins: dict[str, PluginInfo], default: str = "") -> PluginInfoContainer:
    """A `PluginInfoContainer`; an empty `default` leaves the default unset."""
    result = PluginInfoContainer()
    result.plugins = plugins
    result.default_plugin = default
    return result


def classes(plugins: dict[str, PluginInfo]) -> dict[str, str]:
    """Plugin name -> factory class."""
    return {name: info.class_name for name, info in sorted(plugins.items())}


def contact_summary(info: ContactManagersPluginInfo) -> dict[str, Any]:
    """Every field of a `ContactManagersPluginInfo` as plain values."""
    return {
        "search_paths": list(info.search_paths),
        "search_libraries": list(info.search_libraries),
        "discrete_default": info.discrete_plugin_infos.default_plugin,
        "discrete_plugins": classes(info.discrete_plugin_infos.plugins),
        "continuous_default": info.continuous_plugin_infos.default_plugin,
        "continuous_plugins": classes(info.continuous_plugin_infos.plugins),
    }


def contact_managers(locator: GeneralResourceLocator) -> dict[str, Any]:
    """contact_manager_plugins.yaml as two parts merged with insert, applied to an environment."""
    # --8<-- [start:contact_managers]
    bullet = ContactManagersPluginInfo()
    bullet.search_paths = [SEARCH_PATH]
    bullet.search_libraries = [BULLET_LIBRARY]
    bullet.discrete_plugin_infos = container(
        {
            "BulletDiscreteBVHManager": plugin("BulletDiscreteBVHManagerFactory"),
            "BulletDiscreteSimpleManager": plugin("BulletDiscreteSimpleManagerFactory"),
        },
        default="BulletDiscreteBVHManager",
    )
    bullet.continuous_plugin_infos = container(
        {
            "BulletCastBVHManager": plugin("BulletCastBVHManagerFactory"),
            "BulletCastSimpleManager": plugin("BulletCastSimpleManagerFactory"),
        },
        default="BulletCastBVHManager",
    )
    # No default: insert keeps the Bullet one
    fcl = ContactManagersPluginInfo()
    fcl.search_libraries = [FCL_LIBRARY]
    fcl.discrete_plugin_infos = container(
        {"FCLDiscreteBVHManager": plugin("FCLDiscreteBVHManagerFactory")}
    )

    merged = ContactManagersPluginInfo()
    new_empty = merged.empty()
    merged.insert(bullet)
    merged.insert(fcl)
    # --8<-- [end:contact_managers]

    # The shipped file's information, as the IIWA SRDF loads it
    shipped = Environment()
    shipped.init(locate(locator, URDF_URL), locate(locator, SRDF_URL), locator)
    shipped_summary = contact_summary(shipped.getContactManagersPluginInfo())
    merged_summary = contact_summary(merged)

    # --8<-- [start:apply]
    env = Environment()
    env.init(locate(locator, URDF_URL), locator)  # no SRDF: no contact-manager plugins
    managers_before = {
        "discrete": env.getDiscreteContactManager(),
        "continuous": env.getContinuousContactManager(),
    }
    applied = env.applyCommand(AddContactManagersPluginInfoCommand(merged))
    created = {
        name: env.getDiscreteContactManager(name).getName()
        for name in merged.discrete_plugin_infos.plugins
    }
    created.update(
        {
            name: env.getContinuousContactManager(name).getName()
            for name in merged.continuous_plugin_infos.plugins
        }
    )
    # --8<-- [end:apply]
    environment_info_equal = env.getContactManagersPluginInfo() == merged

    cleared = ContactManagersPluginInfo()
    cleared.insert(merged)
    cleared.clear()
    return {
        "empty": {
            "new": new_empty,
            "part": fcl.empty(),
            "merged": merged.empty(),
            "cleared": cleared.empty(),
        },
        "merged": merged_summary,
        "matches_shipped_except_search_paths": {
            k: v for k, v in merged_summary.items() if k != "search_paths"
        }
        == {k: v for k, v in shipped_summary.items() if k != "search_paths"},
        "managers_before": managers_before,
        "applied": applied,
        "environment_info_equal": environment_info_equal,
        "created": created,
    }


def profiles() -> dict[str, Any]:
    """Upstream's ProfilePluginInfoUnit values (a fixture), merged from two parts."""
    # --8<-- [start:profiles]
    fwd = ProfilesPluginInfo()
    fwd.search_paths = [SEARCH_PATH]
    fwd.search_libraries = ["tesseract_collision"]
    fwd.plugin_infos = {"manipulator": {"KDLFwdKin": plugin("Profile")}}
    inv = ProfilesPluginInfo()
    inv.plugin_infos = {"manipulator": {"KDLInvKin": plugin("Profile")}}

    merged = ProfilesPluginInfo()
    new_empty = merged.empty()
    merged.insert(fwd)
    merged.insert(inv)  # merges into "manipulator", does not replace it
    # --8<-- [end:profiles]

    # upstream's single build of the same information
    single = ProfilesPluginInfo()
    single.search_paths = [SEARCH_PATH]
    single.search_libraries = ["tesseract_collision"]
    single.plugin_infos = {
        "manipulator": {"KDLFwdKin": plugin("Profile"), "KDLInvKin": plugin("Profile")}
    }

    summary = {
        "search_paths": list(merged.search_paths),
        "search_libraries": list(merged.search_libraries),
        "plugin_infos": {group: classes(p) for group, p in sorted(merged.plugin_infos.items())},
    }
    equals_single_build = merged == single
    merged_empty = merged.empty()
    merged.clear()
    return {
        "empty": {
            "new": new_empty,
            "part": inv.empty(),
            "merged": merged_empty,
            "cleared": merged.empty(),
        },
        "merged": summary,
        "equals_single_build": equals_single_build,
    }


def task_composer() -> dict[str, Any]:
    """task_composer_plugins.yaml's libraries, executor block and two task entries."""
    # --8<-- [start:task_composer]
    tasks = TaskComposerPluginInfo()
    tasks.search_paths = [SEARCH_PATH]
    tasks.search_libraries = TASK_LIBRARIES
    tasks.task_plugin_infos = container(
        {name: plugin("PipelineTaskFactory") for name in MIRRORED_TASKS}
    )
    executors = TaskComposerPluginInfo()
    executors.search_libraries = [TASKFLOW_LIBRARY]
    executors.executor_plugin_infos = container(
        {"TaskflowExecutor": plugin("TaskflowTaskComposerExecutorFactory", EXECUTOR_CONFIG)},
        default="TaskflowExecutor",
    )

    merged = TaskComposerPluginInfo()
    new_empty = merged.empty()
    merged.insert(tasks)
    merged.insert(executors)  # libraries in the YAML's order
    # --8<-- [end:task_composer]

    executor = merged.executor_plugin_infos
    summary = {
        "search_paths": list(merged.search_paths),
        "search_libraries": list(merged.search_libraries),
        "executor_default": executor.default_plugin,
        "executor_plugins": {
            name: (info.class_name, info.config.strip()) for name, info in executor.plugins.items()
        },
        "task_plugins": classes(merged.task_plugin_infos.plugins),
    }
    merged_empty = merged.empty()
    merged.clear()
    return {
        "empty": {
            "new": new_empty,
            "part": executors.empty(),
            "merged": merged_empty,
            "cleared": merged.empty(),
        },
        "merged": summary,
    }


def factory_introspection() -> dict[str, Any]:
    """Configured vs available node plugins of the shipped task composer config."""
    # --8<-- [start:factory]
    composer = TaskComposer.from_config()
    factory = composer.factory
    configured = factory.getTaskComposerNodePlugins()  # name -> PluginInfo
    available = factory.getAvailableTaskComposerNodePlugins()  # factory class names
    pipelines = composer.get_available_pipelines()  # configured names that load
    # --8<-- [end:factory]
    return {
        "default_executor": factory.getDefaultTaskComposerExecutorPlugin(),
        "mirrored_tasks_match": all(
            configured[name].class_name == "PipelineTaskFactory" for name in MIRRORED_TASKS
        ),
        "configured_classes_available": {p.class_name for p in configured.values()}
        <= set(available),
        "pipelines_are_configured": set(pipelines) <= set(configured),
        "num_configured": len(configured),
        "num_available": len(available),
        "num_pipelines": len(pipelines),
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator()
    return {
        "contact_managers": contact_managers(locator),
        "profiles": profiles(),
        "task_composer": task_composer(),
        "factory": factory_introspection(),
    }


def main() -> None:
    """Console entry point (`tesseract_plugin_configuration_example`): run() and print the results."""
    result = run()
    contact = result["contact_managers"]
    print(f"ContactManagersPluginInfo (Bullet part + FCL part): {contact['merged']}")
    print(
        f"  matches the shipped YAML (search paths aside): {contact['matches_shipped_except_search_paths']}"
    )
    print(
        f"  managers before the command {contact['managers_before']}, created after {contact['created']}"
    )
    print(f"  empty: {contact['empty']}")
    print(
        f"ProfilesPluginInfo: {result['profiles']['merged']}; empty: {result['profiles']['empty']}"
    )
    print(
        f"TaskComposerPluginInfo: {result['task_composer']['merged']}; empty: {result['task_composer']['empty']}"
    )
    factory = result["factory"]
    print(
        f"TaskComposerPluginFactory: {factory['num_configured']} configured nodes, "
        f"{factory['num_available']} available node classes, {factory['num_pipelines']} loadable pipelines"
    )
    print(
        f"  every configured class available {factory['configured_classes_available']}; "
        f"pipelines are configured names {factory['pipelines_are_configured']}"
    )


if __name__ == "__main__":
    main()
