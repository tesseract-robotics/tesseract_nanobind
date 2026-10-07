"""Value equality on the tesseract_environment commands and Environment (#169).

Each class binds its C++ operator==/operator!= as __eq__/__ne__ and sets __hash__ = None.
"""

import pytest

from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    GeneralResourceLocator,
    Isometry3d,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import (
    AddKinematicsInformationCommand,
    AddLinkCommand,
    AddSceneGraphCommand,
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
from tesseract_robotics.tesseract_scene_graph import Joint, Link, SceneGraph
from tesseract_robotics.tesseract_srdf import KinematicsInformation

from .test_environment_commands import SIMPLE_URDF


def _offset(x):
    return Isometry3d(Translation3d(x, 0.0, 0.0))


def _kinematics_information():
    info = KinematicsInformation()
    info.addChainGroup("manipulator", [("base_link", "tool0")])
    return info


def _acm():
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("link1", "link2", "adjacent")
    return acm


# (make, make_other): two make() calls give equal commands; make_other() differs in one field.
COMMAND_CASES = {
    "AddKinematicsInformationCommand": (
        AddKinematicsInformationCommand,
        lambda: AddKinematicsInformationCommand(_kinematics_information()),
    ),
    "AddLinkCommand": (lambda: AddLinkCommand(Link("l")), lambda: AddLinkCommand(Link("m"))),
    "AddSceneGraphCommand": (
        lambda: AddSceneGraphCommand(SceneGraph("g")),
        lambda: AddSceneGraphCommand(SceneGraph("g"), prefix="p_"),
    ),
    "ChangeCollisionMarginsCommand": (
        lambda: ChangeCollisionMarginsCommand(0.1),
        lambda: ChangeCollisionMarginsCommand(0.2),
    ),
    "ChangeJointAccelerationLimitsCommand": (
        lambda: ChangeJointAccelerationLimitsCommand("j", 1.0),
        lambda: ChangeJointAccelerationLimitsCommand("j", 2.0),
    ),
    "ChangeJointOriginCommand": (
        lambda: ChangeJointOriginCommand("j", _offset(0.0)),
        lambda: ChangeJointOriginCommand("j", _offset(0.1)),
    ),
    "ChangeJointPositionLimitsCommand": (
        lambda: ChangeJointPositionLimitsCommand("j", -1.0, 1.0),
        lambda: ChangeJointPositionLimitsCommand("j", -1.0, 2.0),
    ),
    "ChangeJointVelocityLimitsCommand": (
        lambda: ChangeJointVelocityLimitsCommand("j", 1.0),
        lambda: ChangeJointVelocityLimitsCommand("j", 2.0),
    ),
    "ChangeLinkCollisionEnabledCommand": (
        lambda: ChangeLinkCollisionEnabledCommand("l", True),
        lambda: ChangeLinkCollisionEnabledCommand("l", False),
    ),
    "ChangeLinkOriginCommand": (
        lambda: ChangeLinkOriginCommand("l", _offset(0.0)),
        lambda: ChangeLinkOriginCommand("l", _offset(0.1)),
    ),
    "ChangeLinkVisibilityCommand": (
        lambda: ChangeLinkVisibilityCommand("l", True),
        lambda: ChangeLinkVisibilityCommand("l", False),
    ),
    "ModifyAllowedCollisionsCommand": (
        lambda: ModifyAllowedCollisionsCommand(_acm(), ModifyAllowedCollisionsType.ADD),
        lambda: ModifyAllowedCollisionsCommand(_acm(), ModifyAllowedCollisionsType.REMOVE),
    ),
    "MoveJointCommand": (lambda: MoveJointCommand("j", "p"), lambda: MoveJointCommand("j", "q")),
    "MoveLinkCommand": (lambda: MoveLinkCommand(Joint("j")), lambda: MoveLinkCommand(Joint("k"))),
    "RemoveAllowedCollisionLinkCommand": (
        lambda: RemoveAllowedCollisionLinkCommand("a"),
        lambda: RemoveAllowedCollisionLinkCommand("b"),
    ),
    "RemoveJointCommand": (lambda: RemoveJointCommand("a"), lambda: RemoveJointCommand("b")),
    "RemoveLinkCommand": (lambda: RemoveLinkCommand("a"), lambda: RemoveLinkCommand("b")),
    "ReplaceJointCommand": (
        lambda: ReplaceJointCommand(Joint("j")),
        lambda: ReplaceJointCommand(Joint("k")),
    ),
}


@pytest.mark.parametrize(("make", "make_other"), COMMAND_CASES.values(), ids=COMMAND_CASES.keys())
def test_command_equal_when_built_alike(make, make_other):
    a, b = make(), make()
    assert a is not b
    assert a == b
    assert not (a != b)


@pytest.mark.parametrize(("make", "make_other"), COMMAND_CASES.values(), ids=COMMAND_CASES.keys())
def test_command_unequal_when_one_field_differs(make, make_other):
    assert make() != make_other()
    assert not (make() == make_other())


@pytest.mark.parametrize(("make", "make_other"), COMMAND_CASES.values(), ids=COMMAND_CASES.keys())
def test_command_unhashable(make, make_other):
    obj = make()
    assert type(obj).__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(obj)


@pytest.mark.parametrize(("make", "make_other"), COMMAND_CASES.values(), ids=COMMAND_CASES.keys())
def test_command_foreign_operand_is_unequal_without_raising(make, make_other):
    obj = make()
    assert (obj == object()) is False
    assert (obj != object()) is True


def test_command_base_binds_value_equality():
    """Command is not constructible: check the binding itself (command.h:87-88)."""
    assert "__eq__" in vars(Command)
    assert "__ne__" in vars(Command)
    assert Command.__hash__ is None


def test_commands_of_different_classes_never_compare_equal():
    """Command::operator== is non-virtual; Python dispatches on the derived classes, which both decline."""
    assert RemoveLinkCommand("a") != RemoveJointCommand("a")
    assert not (RemoveLinkCommand("a") == RemoveJointCommand("a"))


def test_changing_joint_origin_within_tolerance_is_equal():
    """ChangeJointOriginCommand::operator== uses isApprox(1e-5) (change_joint_origin_command.cpp, 0.35.0)."""
    assert ChangeJointOriginCommand("j", _offset(1.0)) == ChangeJointOriginCommand(
        "j", _offset(1.0 + 1e-9)
    )


@pytest.fixture
def env():
    environment = Environment()
    assert environment.init(SIMPLE_URDF, GeneralResourceLocator())
    return environment


def test_environment_equals_its_clone(env):
    """clone() copies every field operator== reads, the timestamps included (environment.cpp:442-444)."""
    clone = env.clone()
    assert env == clone
    assert not (env != clone)


def test_environment_unequal_after_state_change(env):
    clone = env.clone()
    clone.setState({"joint1": 0.5})
    assert env != clone
    assert not (env == clone)


def test_environments_built_separately_are_unequal():
    """operator== compares the timestamps that each init sets (environment.cpp:412-413, 0.35.0)."""
    a, b = Environment(), Environment()
    assert a.init(SIMPLE_URDF, GeneralResourceLocator())
    assert b.init(SIMPLE_URDF, GeneralResourceLocator())
    assert a.getTimestamp() != b.getTimestamp()
    assert a != b


# Links and joints only: upstream Geometry::operator== compares each geometry's uuid
# (geometry.cpp:40, 0.35.0), and every URDF parse mints new uuids.
NO_GEOMETRY_URDF = """
<robot name="bare" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
</robot>
"""


def _history(urdf):
    environment = Environment()
    assert environment.init(urdf, GeneralResourceLocator())
    return environment.getCommandHistory()


def test_environment_command_histories_compare_by_value():
    """A list compares element-wise, each element as its own derived class. Separately built
    environments hold distinct command objects, so this is value equality, not list identity."""
    history_a, history_b = _history(NO_GEOMETRY_URDF), _history(NO_GEOMETRY_URDF)
    assert len(history_a) == len(history_b)  # zip(strict=True) is 3.10+; the floor is 3.9
    assert all(x is not y for x, y in zip(history_a, history_b))
    assert history_a == history_b


def test_scene_graph_commands_from_separate_parses_differ_by_geometry_uuid():
    """Upstream semantics: the same URDF parsed twice gives geometries with different uuids."""
    history_a, history_b = _history(SIMPLE_URDF), _history(SIMPLE_URDF)
    assert history_a[0] != history_b[0]


def test_environment_unhashable(env):
    assert Environment.__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(env)


def test_environment_foreign_operand_is_unequal_without_raising(env):
    assert (env == object()) is False
    assert (env != object()) is True
