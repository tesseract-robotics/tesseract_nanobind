"""Tests for Environment Command bindings."""
import pytest
import numpy as np

from tesseract_robotics.tesseract_environment import (
    Command,
    AddLinkCommand,
    RemoveLinkCommand,
    RemoveJointCommand,
    ReplaceJointCommand,
    MoveJointCommand,
    MoveLinkCommand,
    ChangeJointPositionLimitsCommand,
    ChangeJointVelocityLimitsCommand,
    ChangeJointAccelerationLimitsCommand,
    ChangeJointOriginCommand,
    ChangeLinkOriginCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    ModifyAllowedCollisionsType_ADD,
    ModifyAllowedCollisionsType_REMOVE,
    ModifyAllowedCollisionsType_REPLACE,
    RemoveAllowedCollisionLinkCommand,
    ChangeLinkCollisionEnabledCommand,
    ChangeLinkVisibilityCommand,
)
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix, Isometry3d
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType, Visual
from tesseract_robotics.tesseract_geometry import Box


class TestCommandImports:
    """Test that all command classes can be imported."""

    def test_command_base_class(self):
        assert Command is not None

    def test_modify_allowed_collisions_type_enum(self):
        assert ModifyAllowedCollisionsType.ADD == ModifyAllowedCollisionsType_ADD
        assert ModifyAllowedCollisionsType.REMOVE == ModifyAllowedCollisionsType_REMOVE
        assert ModifyAllowedCollisionsType.REPLACE == ModifyAllowedCollisionsType_REPLACE


class TestAddLinkCommand:
    """Tests for AddLinkCommand."""

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

    def test_apply_command(self, simple_env):
        link = Link("new_link")
        box = Box(0.05, 0.05, 0.05)
        visual = Visual()
        visual.geometry = box
        link.visual.append(visual)

        joint = Joint("new_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link1"
        joint.child_link_name = "new_link"

        cmd = AddLinkCommand(link, joint)
        assert "new_link" not in simple_env.getLinkNames()
        simple_env.applyCommand(cmd)
        assert "new_link" in simple_env.getLinkNames()


class TestRemoveLinkCommand:
    """Tests for RemoveLinkCommand."""

    def test_constructor(self):
        cmd = RemoveLinkCommand("test_link")
        assert cmd.getLinkName() == "test_link"

    def test_apply_command(self, simple_env):
        # First add a link
        link = Link("temp_link")
        joint = Joint("temp_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link2"
        joint.child_link_name = "temp_link"
        simple_env.applyCommand(AddLinkCommand(link, joint))
        assert "temp_link" in simple_env.getLinkNames()

        # Then remove it
        cmd = RemoveLinkCommand("temp_link")
        simple_env.applyCommand(cmd)
        assert "temp_link" not in simple_env.getLinkNames()


class TestRemoveJointCommand:
    """Tests for RemoveJointCommand."""

    def test_constructor(self):
        cmd = RemoveJointCommand("test_joint")
        assert cmd.getJointName() == "test_joint"


class TestChangeJointPositionLimitsCommand:
    """Tests for ChangeJointPositionLimitsCommand."""

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

    def test_apply_command(self, simple_env):
        cmd = ChangeJointPositionLimitsCommand("joint1", -3.14, 3.14)
        simple_env.applyCommand(cmd)


class TestChangeJointVelocityLimitsCommand:
    """Tests for ChangeJointVelocityLimitsCommand."""

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

    def test_apply_command(self, simple_env):
        cmd = ChangeJointVelocityLimitsCommand("joint1", 2.0)
        simple_env.applyCommand(cmd)


class TestChangeJointAccelerationLimitsCommand:
    """Tests for ChangeJointAccelerationLimitsCommand."""

    def test_constructor_single_joint(self):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == 5.0

    def test_apply_command(self, simple_env):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        simple_env.applyCommand(cmd)


class TestChangeLinkCollisionEnabledCommand:
    """Tests for ChangeLinkCollisionEnabledCommand."""

    def test_constructor(self):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert cmd.getEnabled() == False

    def test_apply_command(self, simple_env):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        simple_env.applyCommand(cmd)


class TestChangeLinkVisibilityCommand:
    """Tests for ChangeLinkVisibilityCommand."""

    def test_constructor(self):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert cmd.getEnabled() == False

    def test_apply_command(self, simple_env):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        simple_env.applyCommand(cmd)


class TestModifyAllowedCollisionsCommand:
    """Tests for ModifyAllowedCollisionsCommand."""

    def test_constructor(self):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "Adjacent")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        assert cmd.getModifyType() == ModifyAllowedCollisionsType.ADD

    def test_apply_command(self, simple_env):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "TestReason")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        simple_env.applyCommand(cmd)


class TestRemoveAllowedCollisionLinkCommand:
    """Tests for RemoveAllowedCollisionLinkCommand."""

    def test_constructor(self):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        assert cmd.getLinkName() == "link1"

    def test_apply_command(self, simple_env):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        simple_env.applyCommand(cmd)


@pytest.mark.skip(reason="CollisionMarginOverrideType enum not bound")
class TestChangeCollisionMarginsCommand:
    """Tests for ChangeCollisionMarginsCommand."""
    pass


class TestChangeJointOriginCommand:
    """Tests for ChangeJointOriginCommand."""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        assert cmd.getJointName() == "joint1"

    def test_apply_command(self, simple_env):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        simple_env.applyCommand(cmd)


class TestChangeLinkOriginCommand:
    """Tests for ChangeLinkOriginCommand."""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        assert cmd.getLinkName() == "link1"

    @pytest.mark.xfail(reason="CHANGE_LINK_ORIGIN not implemented in tesseract C++ Environment")
    def test_apply_command(self, simple_env):
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        simple_env.applyCommand(cmd)


class TestMoveJointCommand:
    """Tests for MoveJointCommand."""

    def test_constructor(self):
        cmd = MoveJointCommand("joint2", "world")
        assert cmd.getJointName() == "joint2"
        assert cmd.getParentLink() == "world"


class TestMoveLinkCommand:
    """Tests for MoveLinkCommand."""

    def test_constructor(self):
        joint = Joint("move_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link2"
        cmd = MoveLinkCommand(joint)
        assert cmd.getJoint().getName() == "move_joint"


class TestReplaceJointCommand:
    """Tests for ReplaceJointCommand."""

    def test_constructor(self):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        assert cmd.getJoint().getName() == "joint1"

    def test_apply_command(self, simple_env):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        simple_env.applyCommand(cmd)
