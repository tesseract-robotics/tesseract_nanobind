"""Tests for Environment Command bindings"""

import gc
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import tesseract_environment
from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    ContactManagersPluginInfo,
    GeneralResourceLocator,
    Isometry3d,
    JointState,
    JointTrajectory,
)
from tesseract_robotics.tesseract_environment import (
    AddContactManagersPluginInfoCommand,
    AddLinkCommand,
    AddTrajectoryLinkCommand,
    ChangeCollisionMarginsCommand,
    ChangeJointAccelerationLimitsCommand,
    ChangeJointOriginCommand,
    ChangeJointPositionLimitsCommand,
    ChangeJointVelocityLimitsCommand,
    ChangeLinkCollisionEnabledCommand,
    ChangeLinkOriginCommand,
    ChangeLinkVisibilityCommand,
    Command,
    Environment,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    MoveJointCommand,
    MoveLinkCommand,
    RemoveAllowedCollisionLinkCommand,
    RemoveJointCommand,
    RemoveLinkCommand,
    ReplaceJointCommand,
)
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_scene_graph import Joint, JointType, Link, Visual

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

SIMPLE_URDF = """
<robot name="test_robot" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">
  <link name="world"/>
  <link name="link1">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="link2">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
</robot>
"""


@pytest.fixture
def env():
    """Create a test environment"""
    environment = Environment()
    locator = GeneralResourceLocator()
    environment.init(SIMPLE_URDF, locator)
    return environment


class TestCommandImports:
    """Test that all command classes can be imported"""

    def test_command_base_class(self):
        assert Command is not None


class TestAddLinkCommand:
    """Tests for AddLinkCommand"""

    def test_constructor_link_only(self):
        link = Link("test_link")
        cmd = AddLinkCommand(link)
        assert cmd.getLink().getName() == "test_link"
        assert cmd.getJoint() is None
        assert not cmd.replaceAllowed()

    def test_constructor_with_joint(self):
        link = Link("test_link")
        joint = Joint("test_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "test_link"
        cmd = AddLinkCommand(link, joint)
        assert cmd.getLink().getName() == "test_link"
        assert cmd.getJoint().getName() == "test_joint"

    def test_apply_command(self, env):
        link = Link("new_link")
        box = Box(0.05, 0.05, 0.05)
        visual = Visual()
        visual.geometry = box
        link.addVisual(visual)

        joint = Joint("new_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link1"
        joint.child_link_name = "new_link"

        cmd = AddLinkCommand(link, joint)
        assert "new_link" not in env.getLinkNames()
        env.applyCommand(cmd)
        assert "new_link" in env.getLinkNames()


class TestRemoveLinkCommand:
    """Tests for RemoveLinkCommand"""

    def test_constructor(self):
        cmd = RemoveLinkCommand("test_link")
        assert cmd.getLinkName() == "test_link"

    def test_apply_command(self, env):
        # First add a link
        link = Link("temp_link")
        joint = Joint("temp_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link2"
        joint.child_link_name = "temp_link"
        env.applyCommand(AddLinkCommand(link, joint))
        assert "temp_link" in env.getLinkNames()

        # Then remove it
        cmd = RemoveLinkCommand("temp_link")
        env.applyCommand(cmd)
        assert "temp_link" not in env.getLinkNames()


class TestRemoveJointCommand:
    """Tests for RemoveJointCommand"""

    def test_constructor(self):
        cmd = RemoveJointCommand("test_joint")
        assert cmd.getJointName() == "test_joint"


class TestChangeJointPositionLimitsCommand:
    """Tests for ChangeJointPositionLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointPositionLimitsCommand("joint1", -3.14, 3.14)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == (-3.14, 3.14)

    def test_constructor_multiple_joints(self):
        limits_dict = {"joint1": (-2.0, 2.0), "joint2": (-1.5, 1.5)}
        cmd = ChangeJointPositionLimitsCommand(limits_dict)
        limits = cmd.getLimits()
        assert limits["joint1"] == (-2.0, 2.0)
        assert limits["joint2"] == (-1.5, 1.5)

    def test_apply_command(self, env):
        cmd = ChangeJointPositionLimitsCommand("joint1", -3.14, 3.14)
        env.applyCommand(cmd)
        # Command applied successfully (no exception)


class TestChangeJointVelocityLimitsCommand:
    """Tests for ChangeJointVelocityLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointVelocityLimitsCommand("joint1", 2.0)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == 2.0

    def test_constructor_multiple_joints(self):
        limits_dict = {"joint1": 2.5, "joint2": 3.0}
        cmd = ChangeJointVelocityLimitsCommand(limits_dict)
        limits = cmd.getLimits()
        assert limits["joint1"] == 2.5
        assert limits["joint2"] == 3.0

    def test_apply_command(self, env):
        cmd = ChangeJointVelocityLimitsCommand("joint1", 2.0)
        env.applyCommand(cmd)


class TestChangeJointAccelerationLimitsCommand:
    """Tests for ChangeJointAccelerationLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == 5.0

    def test_apply_command(self, env):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        env.applyCommand(cmd)


class TestChangeLinkCollisionEnabledCommand:
    """Tests for ChangeLinkCollisionEnabledCommand"""

    def test_constructor(self):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert not cmd.getEnabled()

    def test_apply_command(self, env):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        env.applyCommand(cmd)


class TestChangeLinkVisibilityCommand:
    """Tests for ChangeLinkVisibilityCommand"""

    def test_constructor(self):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert not cmd.getEnabled()

    def test_apply_command(self, env):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        env.applyCommand(cmd)


class TestModifyAllowedCollisionsCommand:
    """Tests for ModifyAllowedCollisionsCommand"""

    def test_constructor(self):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "Adjacent")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        assert cmd.getModifyType() == ModifyAllowedCollisionsType.ADD

    def test_apply_command(self, env):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "TestReason")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        env.applyCommand(cmd)


class TestRemoveAllowedCollisionLinkCommand:
    """Tests for RemoveAllowedCollisionLinkCommand"""

    def test_constructor(self):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        assert cmd.getLinkName() == "link1"

    def test_apply_command(self, env):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        env.applyCommand(cmd)


class TestChangeCollisionMarginsCommand:
    """Tests for ChangeCollisionMarginsCommand"""

    def test_constructor_with_default_margin(self):
        # 0.33 API: Simplified constructor for default margin
        cmd = ChangeCollisionMarginsCommand(0.01)
        # Verify command was created (no getCollisionMarginOverrideType with single arg)
        assert cmd is not None

    def test_constructor_with_pair_data(self):
        # 0.33 API: For pair overrides, use CollisionMarginPairData
        from tesseract_robotics.tesseract_common import (
            CollisionMarginPairData,
            CollisionMarginPairOverrideType,
        )

        pair_data = CollisionMarginPairData()
        cmd = ChangeCollisionMarginsCommand(pair_data, CollisionMarginPairOverrideType.REPLACE)
        assert cmd.getCollisionMarginPairOverrideType() == CollisionMarginPairOverrideType.REPLACE

    def test_apply_command(self, env):
        # 0.33 API: Simple default margin constructor
        cmd = ChangeCollisionMarginsCommand(0.01)
        env.applyCommand(cmd)


class TestChangeJointOriginCommand:
    """Tests for ChangeJointOriginCommand"""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        assert cmd.getJointName() == "joint1"

    def test_apply_command(self, env):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        env.applyCommand(cmd)


class TestChangeLinkOriginCommand:
    """Tests for ChangeLinkOriginCommand"""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        assert cmd.getLinkName() == "link1"

    def test_apply_command_not_implemented(self, env):
        """ChangeLinkOriginCommand exists but Environment.applyCommand raises RuntimeError.

        The C++ Environment::applyChangeLinkOriginCommand is a stub that throws:
        'Unhandled environment command: CHANGE_LINK_ORIGIN'
        """
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        with pytest.raises(RuntimeError, match="Unhandled environment command: CHANGE_LINK_ORIGIN"):
            env.applyCommand(cmd)


class TestMoveJointCommand:
    """Tests for MoveJointCommand"""

    def test_constructor(self):
        cmd = MoveJointCommand("joint2", "world")
        assert cmd.getJointName() == "joint2"
        assert cmd.getParentLink() == "world"


class TestMoveLinkCommand:
    """Tests for MoveLinkCommand"""

    def test_constructor(self):
        joint = Joint("move_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link2"
        cmd = MoveLinkCommand(joint)
        assert cmd.getJoint().getName() == "move_joint"


class TestReplaceJointCommand:
    """Tests for ReplaceJointCommand"""

    def test_constructor(self):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        assert cmd.getJoint().getName() == "joint1"

    def test_apply_command(self, env):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        env.applyCommand(cmd)


def _contact_managers_plugin_info(search_path: str) -> ContactManagersPluginInfo:
    info = ContactManagersPluginInfo()
    info.search_paths = [search_path]
    return info


class TestAddContactManagersPluginInfoCommand:
    """Tests for AddContactManagersPluginInfoCommand"""

    def test_add_contact_managers_plugin_info_command_roundtrip(self):
        cmd = AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/opt/plugins"))
        assert isinstance(cmd, Command)

        info = cmd.getContactManagersPluginInfo()
        assert info.search_paths == ["/opt/plugins"]

        # The getter returns a copy: editing it leaves the command's own info unchanged.
        info.search_paths = ["/elsewhere"]
        assert cmd.getContactManagersPluginInfo().search_paths == ["/opt/plugins"]

    def test_eq(self):
        a = AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/opt/plugins"))
        b = AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/opt/plugins"))
        c = AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/elsewhere"))
        assert a == b
        assert not (a != b)
        assert a != c
        assert not (a == c)

    def test_unhashable(self):
        with pytest.raises(TypeError):
            hash(AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/opt/plugins")))

    def test_apply_add_contact_managers_plugin_info_command(self, env):
        revision = env.getRevision()
        cmd = AddContactManagersPluginInfoCommand(_contact_managers_plugin_info("/opt/plugins"))
        assert env.applyCommand(cmd)
        assert env.getRevision() == revision + 1


def _two_state_trajectory():
    names = ["joint1", "joint2"]
    return JointTrajectory(
        [JointState(names, np.array([0.0, 0.0])), JointState(names, np.array([0.5, -0.5]))]
    )


class TestAddTrajectoryLinkCommand:
    """Tests for AddTrajectoryLinkCommand (gh-167)"""

    def test_method_enum(self):
        """Exactly the four members of add_trajectory_link_command.h:50-73, nested, no SWIG constants."""
        Method = AddTrajectoryLinkCommand.Method
        assert {m.name for m in Method} == {
            "PER_STATE_OBJECTS",
            "PER_STATE_CONVEX_HULL",
            "GLOBAL_PER_LINK_CONVEX_HULL",
            "GLOBAL_CONVEX_HULL",
        }
        assert not [name for name in dir(tesseract_environment) if name.startswith("Method_")]

    def test_getters(self):
        cmd = AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory())
        assert isinstance(cmd, Command)
        assert cmd.getLinkName() == "traj_link"
        assert cmd.getParentLinkName() == "world"
        assert len(cmd.getTrajectory()) == 2
        assert cmd.replaceAllowed() is False
        assert cmd.getMethod() == AddTrajectoryLinkCommand.Method.PER_STATE_OBJECTS

        cmd = AddTrajectoryLinkCommand(
            "traj_link",
            "world",
            _two_state_trajectory(),
            replace_allowed=True,
            method=AddTrajectoryLinkCommand.Method.GLOBAL_CONVEX_HULL,
        )
        assert cmd.replaceAllowed() is True
        assert cmd.getMethod() == AddTrajectoryLinkCommand.Method.GLOBAL_CONVEX_HULL

    def test_get_trajectory_is_copy(self):
        cmd = AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory())
        traj = cmd.getTrajectory()
        traj.push_back(JointState(["joint1", "joint2"], np.array([1.0, 1.0])))
        traj.description = "edited"
        assert len(cmd.getTrajectory()) == 2
        assert cmd.getTrajectory().description == ""

    def test_eq(self):
        a = AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory())
        b = AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory())
        c = AddTrajectoryLinkCommand("other_link", "world", _two_state_trajectory())
        assert a == b
        assert not (a != b)
        assert a != c
        assert not (a == c)

    def test_unhashable(self):
        with pytest.raises(TypeError):
            hash(AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory()))

    @pytest.mark.parametrize(
        "method",
        [
            "PER_STATE_OBJECTS",
            "PER_STATE_CONVEX_HULL",
            "GLOBAL_PER_LINK_CONVEX_HULL",
            "GLOBAL_CONVEX_HULL",
        ],
    )
    def test_apply(self, env, method):
        revision = env.getRevision()
        cmd = AddTrajectoryLinkCommand(
            "traj_link",
            "world",
            _two_state_trajectory(),
            method=AddTrajectoryLinkCommand.Method[method],
        )
        assert env.applyCommand(cmd)
        assert env.getRevision() == revision + 1
        assert "traj_link" in env.getLinkNames()


# gh-186: CommandType, the command history, applyCommands, init(commands) and the
# SetActive*ContactManagerCommands.

# command.h:41-65 (tesseract 0.35.0), name -> value
_COMMAND_TYPE_VALUES = {
    "UNINITIALIZED": -1,
    "ADD_LINK": 0,
    "MOVE_LINK": 1,
    "MOVE_JOINT": 2,
    "REMOVE_LINK": 3,
    "REMOVE_JOINT": 4,
    "CHANGE_LINK_ORIGIN": 5,
    "CHANGE_JOINT_ORIGIN": 6,
    "CHANGE_LINK_COLLISION_ENABLED": 7,
    "CHANGE_LINK_VISIBILITY": 8,
    "MODIFY_ALLOWED_COLLISIONS": 9,
    "REMOVE_ALLOWED_COLLISION_LINK": 10,
    "ADD_SCENE_GRAPH": 11,
    "CHANGE_JOINT_POSITION_LIMITS": 12,
    "CHANGE_JOINT_VELOCITY_LIMITS": 13,
    "CHANGE_JOINT_ACCELERATION_LIMITS": 14,
    "ADD_KINEMATICS_INFORMATION": 15,
    "REPLACE_JOINT": 16,
    "CHANGE_COLLISION_MARGINS": 17,
    "ADD_CONTACT_MANAGERS_PLUGIN_INFO": 18,
    "SET_ACTIVE_DISCRETE_CONTACT_MANAGER": 19,
    "SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER": 20,
    "ADD_TRAJECTORY_LINK": 21,
}


@pytest.fixture
def iiwa_env():
    """URDF + SRDF environment: the SRDF brings the contact-manager plugins SIMPLE_URDF lacks."""
    urdf_dir = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf"
    environment = Environment()
    assert environment.init(
        urdf_dir / "lbr_iiwa_14_r820.urdf",
        urdf_dir / "lbr_iiwa_14_r820.srdf",
        TesseractSupportResourceLocator(),
    )
    return environment


def _set_active_commands(env):
    """(command class, CommandType name, active manager name, Environment setter) per manager kind."""
    return [
        (
            tesseract_environment.SetActiveDiscreteContactManagerCommand,
            "SET_ACTIVE_DISCRETE_CONTACT_MANAGER",
            env.getDiscreteContactManager().getName(),
            env.setActiveDiscreteContactManager,
        ),
        (
            tesseract_environment.SetActiveContinuousContactManagerCommand,
            "SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER",
            env.getContinuousContactManager().getName(),
            env.setActiveContinuousContactManager,
        ),
    ]


class TestCommandHistory:
    """Tests for CommandType, getCommandHistory, applyCommands and init(commands) (gh-186)"""

    def test_command_get_type(self):
        CommandType = tesseract_environment.CommandType
        assert RemoveLinkCommand("x").getType() == CommandType.REMOVE_LINK
        assert (
            ChangeLinkVisibilityCommand("x", False).getType() == CommandType.CHANGE_LINK_VISIBILITY
        )

    def test_command_type_values(self):
        CommandType = tesseract_environment.CommandType
        assert {m.name: int(m.value) for m in CommandType} == _COMMAND_TYPE_VALUES

    def test_get_command_history_is_polymorphic(self, env):
        assert env.applyCommand(ChangeLinkVisibilityCommand("link1", False))
        history = env.getCommandHistory()
        assert history
        assert all(isinstance(cmd, Command) for cmd in history)
        # init's own commands come back as their classes too, not as the base
        assert type(history[0]) is tesseract_environment.AddSceneGraphCommand
        assert type(history[-1]) is ChangeLinkVisibilityCommand
        assert history[-1].getType() == tesseract_environment.CommandType.CHANGE_LINK_VISIBILITY
        assert history[-1].getLinkName() == "link1"

    def test_get_command_history_add_trajectory_link(self, env):
        """ADD_TRAJECTORY_LINK maps to AddTrajectoryLinkCommand (gh-167 left it to this issue)."""
        assert env.applyCommand(
            AddTrajectoryLinkCommand("traj_link", "world", _two_state_trajectory())
        )
        last = env.getCommandHistory()[-1]
        assert type(last) is AddTrajectoryLinkCommand
        assert last.getType() == tesseract_environment.CommandType.ADD_TRAJECTORY_LINK
        assert last.getLinkName() == "traj_link"

    def test_apply_commands_list(self, env):
        revision = env.getRevision()
        assert env.applyCommands(
            [RemoveLinkCommand("link2"), ChangeLinkVisibilityCommand("link1", False)]
        )
        assert env.getRevision() == revision + 2
        assert "link2" not in env.getLinkNames()
        assert env.getLinkVisibility("link1") is False
        assert [type(cmd) for cmd in env.getCommandHistory()[-2:]] == [
            RemoveLinkCommand,
            ChangeLinkVisibilityCommand,
        ]

    def test_apply_commands_outlives_python_refs(self, env):
        commands = [RemoveLinkCommand("link2"), ChangeLinkVisibilityCommand("link1", False)]
        assert env.applyCommands(commands)
        del commands
        gc.collect()
        last_two = env.getCommandHistory()[-2:]
        assert last_two[0].getLinkName() == "link2"
        assert last_two[1].getLinkName() == "link1"
        assert last_two[1].getEnabled() is False

    def test_command_applied_event_commands_copy(self, env):
        stored = []

        def on_event(evt):
            if evt.type == tesseract_environment.Events.COMMAND_APPLIED:
                stored.append((evt.commands, evt.revision))

        env.addEventCallback(1, tesseract_environment.EventCallbackFn(on_event))
        assert env.applyCommand(ChangeLinkVisibilityCommand("link1", False))
        assert env.applyCommand(ChangeLinkVisibilityCommand("link2", False))
        env.clearEventCallbacks()

        assert len(stored) == 2
        (first, first_revision), (second, second_revision) = stored
        # Each list is the history as it was when its event fired, still readable afterwards.
        assert len(second) == len(first) + 1
        assert second_revision == first_revision + 1
        assert type(first[-1]) is ChangeLinkVisibilityCommand
        assert first[-1].getLinkName() == "link1"
        assert second[-1].getLinkName() == "link2"

    def test_init_from_command_history_roundtrip(self, iiwa_env):
        env = iiwa_env
        assert env.applyCommand(ChangeLinkVisibilityCommand("link_1", False))
        name = env.getDiscreteContactManager().getName()
        assert env.applyCommand(tesseract_environment.SetActiveDiscreteContactManagerCommand(name))

        env2 = Environment()
        assert env2.init(env.getCommandHistory())
        assert env2.isInitialized()
        assert env2.getLinkNames() == env.getLinkNames()
        assert env2.getJointNames() == env.getJointNames()
        assert env2.getRevision() == env.getRevision()
        assert [c.getType() for c in env2.getCommandHistory()] == [
            c.getType() for c in env.getCommandHistory()
        ]
        assert env2.getLinkVisibility("link_1") is False

    def test_init_commands_requires_add_scene_graph_first(self):
        """environment.cpp initHelper: an empty list, or one not led by ADD_SCENE_GRAPH, fails."""
        assert not Environment().init([])
        assert not Environment().init([RemoveLinkCommand("link1")])

    @pytest.mark.parametrize("call", ["applyCommands", "init"])
    def test_none_in_command_list_raises(self, call):
        """nanobind passes a None element of list[Command] as a null shared_ptr; C++ would
        dereference it. A subprocess, so the unguarded segfault fails the test, not the run."""
        code = (
            "from tesseract_robotics.tesseract_environment import Environment, RemoveLinkCommand\n"
            "try:\n"
            f"    Environment().{call}([RemoveLinkCommand('a'), None])\n"
            "except TypeError as e:\n"
            "    print('TypeError:', e)\n"
        )
        proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        assert proc.returncode == 0, proc.stderr
        assert "TypeError:" in proc.stdout
        assert "commands[1] is None" in proc.stdout

    def test_init_commands_keeps_scene_graph_overload(self, env):
        env2 = Environment()
        assert env2.init(env.getSceneGraph())
        assert env2.getLinkNames() == env.getLinkNames()


class TestSetActiveContactManagerCommands:
    """Tests for SetActive{Discrete,Continuous}ContactManagerCommand (gh-186)"""

    def test_set_active_contact_manager_commands(self, iiwa_env):
        env = iiwa_env
        for cls, type_name, name, _ in _set_active_commands(env):
            cmd = cls(name)
            assert isinstance(cmd, Command)
            assert cmd.getName() == name
            assert cmd.getType() == tesseract_environment.CommandType[type_name]
            revision = env.getRevision()
            assert env.applyCommand(cmd)
            assert env.getRevision() == revision + 1
            last = env.getCommandHistory()[-1]
            assert type(last) is cls
            assert last.getName() == name

    def test_set_active_contact_manager_command_eq(self, iiwa_env):
        for cls, _, name, _ in _set_active_commands(iiwa_env):
            assert cls(name) == cls(name)
            assert not (cls(name) != cls(name))
            assert cls(name) != cls("other")
            assert not (cls(name) == cls("other"))
            with pytest.raises(TypeError):
                hash(cls(name))

    def test_set_active_contact_manager_method_not_in_history(self, iiwa_env):
        """Upstream (environment.cpp 0.35.0): only the command is recorded, not the setter."""
        env = iiwa_env
        for _, _, name, setter in _set_active_commands(env):
            revision = env.getRevision()
            history_length = len(env.getCommandHistory())
            assert setter(name)
            assert env.getRevision() == revision
            assert len(env.getCommandHistory()) == history_length
