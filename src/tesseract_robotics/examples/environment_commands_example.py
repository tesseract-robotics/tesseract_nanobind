"""
Environment Commands Example
============================

Builds ABB IRB2400 environments from files, from an SRDF carrying a calibration,
and from a list of commands, and reads back what each one recorded: the command
history, the contact-manager plugin information and the events a command fires.

C++ Reference:
    environment/src/environment.cpp  (getInitCommands :95-137, Implementation::operator== :393-413,
        Implementation::clone :424-444, triggerEnvironmentChangedCallbacks :717-725)
    srdf/test/tesseract_srdf_unit.cpp  (LoadSRDFSaveUnit calibration fixture :672-690)
    common/src/resource_locator.cpp  (GeneralResourceLocator :52-200)

Overview
--------
1. `GeneralResourceLocator` built three ways (default variables, explicit
   `paths`, explicit `environment_variables`), `loadEnvironmentVariable` and
   `addPath`; then `Environment.init(Path, Path, locator)` and `init(Path, locator)`.
2. `getCommandHistory()` is `getInitCommands`' sequence: AddSceneGraph, then,
   with an SRDF, AddContactManagersPluginInfo and AddKinematicsInformation.
3. The shipped `abb_irb2400.srdf` plus a `calibration_config` element pointing
   at upstream's unit-test calibration (a fixture, not a real calibration) adds
   one ChangeJointOrigin per calibrated joint. `CalibrationInfo` and
   `SRDFModel.contact_managers_plugin_info` are read back.
4. On a URDF-only environment, `AddContactManagersPluginInfoCommand` makes the
   contact managers available, and `applyCommands` applies the calibration as
   one batch. An event callback receives one `CommandAppliedEvent` per call.
5. `init(commands)` replays the calibrated history. The histories are equal, the
   environments are not: `Environment ==` compares timestamps, and the replay
   was built later. A `clone()` copies the timestamps, so it is equal until a
   command is applied to it.
6. `BytesResource` from `bytes` and from a list of byte values, with a parent
   locator that resolves a relative url next to the resource.

Key Concepts
------------
**The command history is the environment**:
    Every change is a command. `init(urdf, srdf)` is `init(getInitCommands(...))`,
    so replaying `getCommandHistory()` rebuilds the same scene graph, state and
    plugin information.

**Equality includes time**:
    `Environment.__eq__` compares revisions, every command, the current state and
    both timestamps (environment.cpp:393-413). Two environments built from the
    same commands at different times are therefore unequal.

**Packages**:
    `package://name/...` resolves through a directory named `name` that holds a
    `package.xml`. `addPath` and the `paths` constructor search a directory and
    its subdirectories for such packages; `loadEnvironmentVariable` does the
    same for each entry of a path-list variable.

**Comparing states**:
    `SceneState` binds no `==` yet (#232), so `==` on two states compares
    identity. The replay check compares joints and transforms itself, with
    upstream's `SceneState ==` precision; the IRB2400 has no floating joints,
    whose values `SceneState` does not expose yet either.
"""

import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import (
    BytesResource,
    CalibrationInfo,
    GeneralResourceLocator,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import (
    AddContactManagersPluginInfoCommand,
    ChangeJointOriginCommand,
    ChangeLinkVisibilityCommand,
    Command,
    Environment,
    EventCallbackFn,
    Events,
)
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_state_solver import SceneState
from tesseract_robotics.tesseract_urdf import parseURDFFile

URDF_URL = "package://tesseract/support/urdf/abb_irb2400.urdf"
SRDF_URL = "package://tesseract/support/urdf/abb_irb2400.srdf"
# A variable no installation sets: loadEnvironmentVariable returns False for it
UNSET_VARIABLE = "TESSERACT_EXAMPLE_UNSET_VARIABLE"

# Upstream's unit-test calibration (tesseract_srdf_unit.cpp:672-690), verbatim. A fixture
# for the mechanism, not a calibration of a real IRB2400.
CALIBRATION_YAML = """calibration:
  joints:
    joint_1:
      position:
        x: 1
        y: 2
        z: 3
      orientation:
        x: 0
        y: 0
        z: 0
        w: 1
    joint_2:
      position:
        x: 4
        y: 5
        z: 6
      orientation:
        x: 0
        y: 0
        z: 0
        w: 1
"""
# The fixture's positions, m; both orientations are the identity
CALIBRATED_POSITIONS = {"joint_1": [1.0, 2.0, 3.0], "joint_2": [4.0, 5.0, 6.0]}
FIXTURE_PACKAGE = "calibration_fixture"
FIXTURE_URL = f"package://{FIXTURE_PACKAGE}/calibration.yaml"
CALIBRATION_ELEMENT = f'    <calibration_config filename="{FIXTURE_URL}"/>\n'
# SceneState::operator== isApprox precision (scene_graph/src/scene_state.cpp:56)
SCENE_STATE_PRECISION = 1e-5


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def history_types(commands: list[Command]) -> list[str]:
    """`CommandType` names of a command list, in order."""
    return [command.getType().name for command in commands]


def write_fixture_package(root: Path) -> None:
    """A package `calibration_fixture` under `root` holding the calibration YAML."""
    package = root / FIXTURE_PACKAGE
    package.mkdir()
    (package / "package.xml").write_text(
        f'<package format="3"><name>{FIXTURE_PACKAGE}</name></package>\n'
    )
    (package / "calibration.yaml").write_text(CALIBRATION_YAML)


def plugin_info_summary(srdf: SRDFModel) -> dict[str, Any]:
    """Libraries, defaults and plugin classes of the SRDF's contact-manager plugin information."""
    info = srdf.contact_managers_plugin_info
    discrete, continuous = info.discrete_plugin_infos, info.continuous_plugin_infos
    return {
        "search_libraries": list(info.search_libraries),
        "discrete_default": discrete.default_plugin,
        "discrete_plugins": {name: p.class_name for name, p in discrete.plugins.items()},
        "continuous_default": continuous.default_plugin,
        "continuous_plugins": {name: p.class_name for name, p in continuous.plugins.items()},
    }


def is_approx(a: Isometry3d, b: Isometry3d) -> bool:
    """Eigen's `a.isApprox(b, SCENE_STATE_PRECISION)` on the 4x4 matrices."""
    diff = np.linalg.norm(a.matrix - b.matrix)
    return bool(
        diff <= SCENE_STATE_PRECISION * min(np.linalg.norm(a.matrix), np.linalg.norm(b.matrix))
    )


def states_equal(a: SceneState, b: SceneState) -> bool:
    """Upstream `SceneState ==` (scene_state.cpp:53-67), minus `floating_joints` (not bound, #232)."""

    def maps_equal(x: dict[str, Isometry3d], y: dict[str, Isometry3d]) -> bool:
        return x.keys() == y.keys() and all(is_approx(tf, y[name]) for name, tf in x.items())

    return (
        a.joints == b.joints
        and maps_equal(a.link_transforms, b.link_transforms)
        and maps_equal(a.joint_transforms, b.joint_transforms)
    )


def origins_applied(env: Environment, joints: dict[str, Isometry3d]) -> bool:
    """Each joint's origin in the environment is exactly the calibrated one."""
    return all(
        np.array_equal(env.getJoint(name).parent_to_joint_origin_transform.matrix, origin.matrix)
        for name, origin in joints.items()
    )


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    with tempfile.TemporaryDirectory() as tmp:
        return run_in(Path(tmp))


def run_in(fixture_root: Path) -> dict[str, Any]:
    """The example, with the calibration fixture package written under `fixture_root`."""
    # --8<-- [start:locators]
    # Default: the TESSERACT_RESOURCE_PATH, ROS_PACKAGE_PATH and AMENT_PREFIX_PATH variables
    locator = GeneralResourceLocator()
    urdf_path, srdf_path = locate(locator, URDF_URL), locate(locator, SRDF_URL)
    # The directory that holds the `tesseract` package, found through the package itself
    share = locate(locator, "package://tesseract/package.xml").parent.parent

    by_paths = GeneralResourceLocator(paths=[share], environment_variables=[])
    by_variable = GeneralResourceLocator(environment_variables=["TESSERACT_RESOURCE_PATH"])
    late = GeneralResourceLocator(environment_variables=[])
    unresolved_before_load = late.locateResource(URDF_URL) is None
    load_set_variable = late.loadEnvironmentVariable("TESSERACT_RESOURCE_PATH")
    load_unset_variable = late.loadEnvironmentVariable(UNSET_VARIABLE)

    write_fixture_package(fixture_root)
    add_path = locator.addPath(fixture_root)
    # --8<-- [end:locators]
    locators = {
        "paths_same_file": locate(by_paths, URDF_URL) == urdf_path,
        "environment_variables_same_file": locate(by_variable, URDF_URL) == urdf_path,
        "unresolved_before_load": unresolved_before_load,
        "load_set_variable": load_set_variable,
        "load_unset_variable": load_unset_variable,
        "resolved_after_load": locate(late, URDF_URL) == urdf_path,
        "add_path": add_path,
        "fixture_resolved": locate(locator, FIXTURE_URL)
        == fixture_root / FIXTURE_PACKAGE / "calibration.yaml",
    }

    # --8<-- [start:init_history]
    env = Environment()
    env.init(urdf_path, srdf_path, locator)  # a Path is a file; a str would be content
    init_history = history_types(env.getCommandHistory())

    urdf_only = Environment()
    urdf_only.init(urdf_path, locator)
    urdf_only_history = history_types(urdf_only.getCommandHistory())
    # --8<-- [end:init_history]

    # --8<-- [start:calibration]
    srdf_xml = srdf_path.read_text().replace("</robot>", CALIBRATION_ELEMENT + "</robot>")
    scene_graph = parseURDFFile(str(urdf_path), locator)
    srdf = SRDFModel()
    srdf.initString(scene_graph, srdf_xml, locator)
    joints = srdf.calibration_info.joints

    calibrated = Environment()
    calibrated.init(scene_graph, srdf)
    calibrated_history = calibrated.getCommandHistory()
    # --8<-- [end:calibration]

    # --8<-- [start:calibration_info]
    copy = CalibrationInfo()
    empty_before_insert = copy.empty()
    copy.insert(srdf.calibration_info)
    insert_copies = copy == srdf.calibration_info
    # insert overwrites an existing joint: joint_2 takes joint_1's origin
    override = CalibrationInfo()
    override.joints = {"joint_2": joints["joint_1"]}
    copy.insert(override)
    insert_overrides = {name: tf.translation.tolist() for name, tf in sorted(copy.joints.items())}
    copy.clear()
    # --8<-- [end:calibration_info]
    calibration = {
        "joints": {name: tf.translation.tolist() for name, tf in sorted(joints.items())},
        "rotations_identity": all(np.array_equal(tf.rotation, np.eye(3)) for tf in joints.values()),
        "history": history_types(calibrated_history),
        # the calibration is a hash map, so its joint order is unspecified: compare sorted
        "calibrated_joints": sorted(
            c.getJointName() for c in calibrated_history if isinstance(c, ChangeJointOriginCommand)
        ),
        "origins_applied": origins_applied(calibrated, joints),
        "empty_before_insert": empty_before_insert,
        "insert_copies": insert_copies,
        "insert_overrides": insert_overrides,
        "empty_after_clear": copy.empty(),
    }
    plugin_info = plugin_info_summary(srdf)
    plugin_info["equals_environment_info"] = (
        srdf.contact_managers_plugin_info == calibrated.getContactManagersPluginInfo()
    )

    # --8<-- [start:commands]
    events = []

    def on_event(event):
        # copy what is needed: the event is valid only inside the callback
        if event.type == Events.COMMAND_APPLIED:
            events.append((event.revision, history_types(event.commands)))

    urdf_only.addEventCallback(1, EventCallbackFn(on_event))

    # A URDF-only environment has no contact-manager plugins until the command adds them
    plugin_info_empty_before = urdf_only.getContactManagersPluginInfo().empty()
    manager_before = urdf_only.getDiscreteContactManager()
    command = AddContactManagersPluginInfoCommand(srdf.contact_managers_plugin_info)
    plugin_info_applied = urdf_only.applyCommand(command)
    manager_after = urdf_only.getDiscreteContactManager().getName()

    # The calibration as one batch: one event for both commands
    batch = [ChangeJointOriginCommand(name, origin) for name, origin in joints.items()]
    batch_applied = urdf_only.applyCommands(batch)
    # --8<-- [end:commands]
    commands = {
        "plugin_info_empty_before": plugin_info_empty_before,
        "manager_before": manager_before,
        "plugin_info_applied": plugin_info_applied,
        "command_round_trips": command.getContactManagersPluginInfo()
        == srdf.contact_managers_plugin_info,
        "environment_info_equal": urdf_only.getContactManagersPluginInfo()
        == srdf.contact_managers_plugin_info,
        "manager_after": manager_after,
        "batch_applied": batch_applied,
        "batch_origins_applied": origins_applied(urdf_only, joints),
        "events": events,
        "events_match_history": events[-1][1] == history_types(urdf_only.getCommandHistory()),
    }

    # --8<-- [start:replay]
    replay = Environment()
    initialized = replay.init(calibrated.getCommandHistory())
    histories_equal = all(
        a == b for a, b in zip(calibrated.getCommandHistory(), replay.getCommandHistory())
    ) and len(replay.getCommandHistory()) == len(calibrated_history)
    # Everything but the timestamps matches, so the timestamps decide equality
    timestamps_differ = (
        replay.getTimestamp() != calibrated.getTimestamp()
        and replay.getCurrentStateTimestamp() != calibrated.getCurrentStateTimestamp()
    )
    replay_equals_env = replay == calibrated

    clone = calibrated.clone()
    clone_equals_env = clone == calibrated
    clone.applyCommand(ChangeLinkVisibilityCommand("link_1", False))
    clone_after_command_equals_env = clone == calibrated
    # --8<-- [end:replay]
    replayed = {
        "initialized": initialized,
        "histories_equal": histories_equal,
        "revisions_equal": replay.getRevision() == calibrated.getRevision()
        and replay.getInitRevision() == calibrated.getInitRevision(),
        "states_equal": states_equal(replay.getState(), calibrated.getState()),
        "timestamps_differ": timestamps_differ,
        "replay_equals_env": replay_equals_env,
        "clone_equals_env": clone_equals_env,
        "clone_after_command_equals_env": clone_after_command_equals_env,
    }

    # --8<-- [start:bytes_resource]
    data = CALIBRATION_YAML.encode()
    from_bytes = BytesResource(FIXTURE_URL, data, locator)
    from_list = BytesResource(FIXTURE_URL, list(data), locator)
    # The parent resolves a relative url next to the resource's own url
    sibling = from_bytes.locateResource("package.xml")
    without_parent = BytesResource(FIXTURE_URL, data)
    # --8<-- [end:bytes_resource]
    bytes_resource = {
        "url": from_bytes.getUrl(),
        "is_file": from_bytes.isFile(),
        "contents_equal": from_bytes.getResourceContents() == from_list.getResourceContents(),
        "constructors_equal": from_bytes == from_list,
        "contents_are_fixture": from_bytes.getResourceContents()
        == locate(locator, FIXTURE_URL).read_bytes(),
        "sibling_resolved": sibling is not None
        and Path(sibling.getFilePath()) == fixture_root / FIXTURE_PACKAGE / "package.xml",
        "without_parent_resolves": without_parent.locateResource("package.xml") is not None,
        # BytesResource == compares url, bytes and whether a parent is set
        "without_parent_equal": without_parent == from_bytes,
    }

    return {
        "locators": locators,
        "init_history": init_history,
        "urdf_only_history": urdf_only_history,
        "calibration": calibration,
        "contact_managers_plugin_info": plugin_info,
        "commands": commands,
        "replay": replayed,
        "bytes_resource": bytes_resource,
    }


def main() -> None:
    """Console entry point (`tesseract_environment_commands_example`): run() and print the results."""
    result = run()
    print(f"GeneralResourceLocator: {result['locators']}")
    print(f"init(urdf, srdf) history: {result['init_history']}")
    print(f"init(urdf) history: {result['urdf_only_history']}")
    calibration = result["calibration"]
    print(f"Calibrated joint positions {calibration['joints']}; history {calibration['history']}")
    print(
        f"  origins applied {calibration['origins_applied']}; insert copies {calibration['insert_copies']}, "
        f"overrides {calibration['insert_overrides']}; empty after clear {calibration['empty_after_clear']}"
    )
    print(f"SRDF contact-manager plugins: {result['contact_managers_plugin_info']}")
    commands = result["commands"]
    print(
        f"Discrete manager before AddContactManagersPluginInfoCommand {commands['manager_before']}, "
        f"after {commands['manager_after']}"
    )
    for revision, history in commands["events"]:
        print(f"CommandAppliedEvent revision {revision}: {history}")
    print(f"Replay and clone: {result['replay']}")
    print(f"BytesResource: {result['bytes_resource']}")


if __name__ == "__main__":
    main()
