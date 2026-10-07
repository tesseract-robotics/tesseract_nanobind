import datetime
import gc
import json
import os
import subprocess
import sys
import threading
import traceback
import weakref
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import (
    tesseract_common,
    tesseract_environment,
    tesseract_srdf,
    tesseract_urdf,
)
from tesseract_robotics.tesseract_common import Isometry3d, ManipulatorInfo

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator


def get_scene_graph():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator = TesseractSupportResourceLocator()
    # nanobind automatically extracts from unique_ptr, no .release() needed
    return tesseract_urdf.parseURDFFile(path, locator)


def get_srdf_model(scene_graph):
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")
    srdf = tesseract_srdf.SRDFModel()
    locator = TesseractSupportResourceLocator()
    srdf.initFile(scene_graph, path, locator)
    return srdf


def get_environment():
    scene_graph = get_scene_graph()
    assert scene_graph is not None

    srdf = get_srdf_model(scene_graph)
    assert srdf is not None

    env = tesseract_environment.Environment()
    assert env is not None

    assert env.getRevision() == 0

    success = env.init(scene_graph, srdf)
    assert success
    assert env.getRevision() == 3

    joint_names = [f"joint_a{i + 1}" for i in range(7)]
    joint_values = np.array([1, 2, 1, 2, 1, 2, 1], dtype=np.float64)

    scene_state_changed = [False]
    command_applied = [False]

    def event_cb_py(evt):
        try:
            if evt.type == tesseract_environment.Events_SCENE_STATE_CHANGED:
                evt2 = tesseract_environment.cast_SceneStateChangedEvent(evt)
                if len(evt2.state.joints) != 7:
                    print("joint state length error")
                    return
                for i in range(len(joint_names)):
                    if evt2.state.joints[joint_names[i]] != joint_values[i]:
                        print("joint value mismatch")
                        return
                scene_state_changed[0] = True
            if evt.type == tesseract_environment.Events_COMMAND_APPLIED:
                evt2 = tesseract_environment.cast_CommandAppliedEvent(evt)
                print(evt2.revision)
                if evt2.revision == 4:
                    command_applied[0] = True
        except Exception:
            traceback.print_exc()

    event_cb = tesseract_environment.EventCallbackFn(event_cb_py)

    env.addEventCallback(12345, event_cb)

    env.setState(joint_names, joint_values)
    assert scene_state_changed[0]

    cmd = tesseract_environment.RemoveJointCommand("joint_a7-tool0")
    assert env.applyCommand(cmd)
    assert command_applied[0]

    return env


def _fresh_env():
    scene_graph = get_scene_graph()
    srdf = get_srdf_model(scene_graph)
    env = tesseract_environment.Environment()
    assert env.init(scene_graph, srdf)
    return env


# GH #43: every case below used to SIGSEGV the interpreter (unchecked name
# lookup in the state solver). The binding owns the boundary: ValueError.
def test_set_state_dict_unknown_joint_raises():
    env = _fresh_env()
    with pytest.raises(ValueError, match="not_a_joint"):
        env.setState({"not_a_joint": 1.0})


def test_set_state_dict_mixed_known_unknown_raises():
    env = _fresh_env()
    with pytest.raises(ValueError, match="bogus"):
        env.setState({"joint_a1": 0.5, "bogus": 1.0})


def test_set_state_names_values_unknown_joint_raises():
    env = _fresh_env()
    with pytest.raises(ValueError, match="not_a_joint"):
        env.setState(["not_a_joint"], np.array([1.0]))


def test_set_state_names_values_length_mismatch_raises():
    env = _fresh_env()
    with pytest.raises(ValueError, match="length"):
        env.setState([f"joint_a{i + 1}" for i in range(7)], np.array([1.0, 2.0]))


def test_set_state_by_names_and_values_unknown_joint_raises():
    env = _fresh_env()
    with pytest.raises(ValueError, match="not_a_joint"):
        env.setStateByNamesAndValues(["not_a_joint"], np.array([1.0]))


def test_set_state_valid_dict_still_works():
    env = _fresh_env()
    env.setState({f"joint_a{i + 1}": 0.1 * i for i in range(7)})
    state = env.getState()
    assert abs(state.joints["joint_a3"] - 0.2) < 1e-12


def test_env():
    get_environment()


def test_anypoly_wrap_environment_const():
    """Test wrapping Environment in AnyPoly for TaskComposerDataStorage."""
    from tesseract_robotics.tesseract_environment import AnyPoly_wrap_EnvironmentConst

    env = get_environment()
    # AnyPoly_wrap_EnvironmentConst expects shared_ptr<const Environment>
    # The environment is already a shared_ptr from Python's perspective
    any_poly = AnyPoly_wrap_EnvironmentConst(env)
    assert any_poly is not None
    assert not any_poly.isNull()


def test_get_discrete_contact_manager():
    """Test getDiscreteContactManager returns valid manager."""
    env = get_environment()
    manager = env.getDiscreteContactManager()
    assert manager is not None


def test_get_continuous_contact_manager():
    """Test getContinuousContactManager returns valid manager."""
    env = get_environment()
    manager = env.getContinuousContactManager()
    assert manager is not None


def test_contact_test_api():
    """Test contactTest with ContactRequest and ContactResultMap."""
    from tesseract_robotics.tesseract_collision import (
        ContactRequest,
        ContactResultMap,
        ContactTestType,
    )

    env = get_environment()
    manager = env.getDiscreteContactManager()

    request = ContactRequest(ContactTestType.FIRST)
    results = ContactResultMap()

    # Should not raise - performs collision check
    manager.contactTest(results, request)

    # Results may be empty (no collision in default pose)
    assert results.size() >= 0


def test_clear_cached_contact_managers():
    """Test clearCachedDiscreteContactManager and clearCachedContinuousContactManager.

    Uses a fully loaded robot environment to verify cache clearing with actual geometry.
    """
    from tesseract_robotics.tesseract_collision import (
        ContactRequest,
        ContactResultMap,
        ContactTestType,
    )

    # Get environment with robot - this has collision geometry
    env = get_environment()

    # Set joint state to ensure robot has meaningful configuration
    joint_names = [f"joint_a{i + 1}" for i in range(7)]
    joint_values = np.array([0.5, 0.3, 0.2, 0.1, 0.4, 0.2, 0.1], dtype=np.float64)
    env.setState(joint_names, joint_values)

    # Get managers - this populates the cache with managers configured for the robot
    discrete_mgr1 = env.getDiscreteContactManager()
    continuous_mgr1 = env.getContinuousContactManager()
    assert discrete_mgr1 is not None
    assert continuous_mgr1 is not None

    # Verify manager has collision objects from the robot
    active_links = discrete_mgr1.getActiveCollisionObjects()
    assert len(active_links) > 0, "Manager should have collision objects from robot"

    # Perform collision check - verifies manager works with robot geometry
    request = ContactRequest(ContactTestType.ALL)
    results = ContactResultMap()
    discrete_mgr1.contactTest(results, request)

    # Clear the cache - this invalidates cached managers
    env.clearCachedDiscreteContactManager()
    env.clearCachedContinuousContactManager()

    # Get new managers after clearing - they should be recreated fresh
    discrete_mgr2 = env.getDiscreteContactManager()
    continuous_mgr2 = env.getContinuousContactManager()
    assert discrete_mgr2 is not None
    assert continuous_mgr2 is not None

    # New managers should still have robot's collision objects
    active_links2 = discrete_mgr2.getActiveCollisionObjects()
    assert len(active_links2) > 0, "New manager should also have collision objects"

    # New managers should be functional
    results2 = ContactResultMap()
    discrete_mgr2.contactTest(results2, request)


def test_apply_command_releases_gil_and_still_calls_back():
    """applyCommand drops the GIL, so its Python re-entry path has to still work.

    The environment fires event callbacks from inside applyCommand, under its own unique lock
    and now with the GIL released, so the callback wrapper has to re-acquire it.
    """
    from tesseract_robotics.tesseract_geometry import Box
    from tesseract_robotics.tesseract_scene_graph import Collision, Joint, JointType, Link

    link = Link("gil_link")
    collision = Collision()
    collision.geometry = Box(0.1, 0.1, 0.1)
    link.addCollision(collision)

    joint = Joint("gil_joint")
    joint.type = JointType.FIXED
    joint.parent_link_name = "base_link"
    joint.child_link_name = "gil_link"

    env = get_environment()
    # warm the caches so applyCommand builds the collision shape - the work the release is for -
    # instead of deferring it to the first contact-manager call
    assert env.getDiscreteContactManager() is not None
    assert env.getContinuousContactManager() is not None

    events = []
    cb = tesseract_environment.EventCallbackFn(lambda evt: events.append(evt.type))
    env.addEventCallback(1, cb)

    assert env.applyCommand(tesseract_environment.AddLinkCommand(link, joint))
    assert "gil_link" in env.getLinkNames()
    # the callback re-entered the interpreter from the GIL-released region
    assert events


def test_gil_probe_detects_held_gil():
    """The oracle is not vacuous: a copy made with the GIL held counts as such."""
    from tesseract_robotics.tesseract_environment._tesseract_environment import _GilProbe

    probe = _GilProbe()
    probe.attach(get_environment())  # the binding copies the callback with the GIL held
    assert probe.copies_with_gil >= 1
    assert probe.copies_without_gil == 0


def test_clone_releases_gil():
    """Environment.clone() runs its native copy without the GIL (gh-134)."""
    from tesseract_robotics.tesseract_environment._tesseract_environment import _GilProbe

    env = get_environment()
    probe = _GilProbe()
    probe.attach(env)
    probe.reset()

    clone = env.clone()

    assert len(clone.getLinkNames()) == len(env.getLinkNames())
    assert probe.copies_without_gil >= 1, "clone() never copied the probe"
    assert probe.copies_with_gil == 0, "clone() copied the probe with the GIL held"


def _iiwa_files():
    urdf_dir = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf"
    return urdf_dir / "lbr_iiwa_14_r820.urdf", urdf_dir / "lbr_iiwa_14_r820.srdf"


def test_init_from_paths():
    """gh-165: a `pathlib.Path` reaches the native `std::filesystem::path` overloads."""
    urdf, srdf = _iiwa_files()
    locator = TesseractSupportResourceLocator()
    assert tesseract_environment.Environment().init(urdf, srdf, locator)
    assert tesseract_environment.Environment().init(urdf, locator)


def test_init_from_strings_is_content():
    """gh-165: a `str` is URDF/SRDF content, as in the native `std::string` overloads."""
    urdf, srdf = _iiwa_files()
    locator = TesseractSupportResourceLocator()
    assert tesseract_environment.Environment().init(urdf.read_text(), srdf.read_text(), locator)
    assert tesseract_environment.Environment().init(urdf.read_text(), locator)


def test_init_str_path_is_not_a_path():
    """gh-165 (breaking): a path spelled as `str` is parsed as content, so init fails."""
    urdf, srdf = _iiwa_files()
    locator = TesseractSupportResourceLocator()
    assert not tesseract_environment.Environment().init(str(urdf), str(srdf), locator)
    assert not tesseract_environment.Environment().init(str(urdf), locator)


@pytest.mark.parametrize("mixed", ["content_then_path", "path_then_content"])
def test_init_mixed_raises(mixed):
    """gh-165: one path and one content string match no overload."""
    urdf, srdf = _iiwa_files()
    args = (urdf.read_text(), srdf) if mixed == "content_then_path" else (urdf, srdf.read_text())
    with pytest.raises(TypeError):
        tesseract_environment.Environment().init(*args, TesseractSupportResourceLocator())


@pytest.mark.parametrize("name", ["initFromUrdf", "initFromUrdfSrdf"])
def test_init_from_urdf_removed(name):
    """gh-165: the Python-only content initialisers are gone; `init(str, ...)` replaces them."""
    assert not hasattr(tesseract_environment.Environment, name)


# gh-188: the read-only Environment getters.

# Both transforms come out of the same state solver for the same joint values, so they
# agree to floating-point roundoff (m for translation, unitless for rotation entries).
FK_ATOL = 1e-12

_IIWA_JOINTS = [f"joint_a{i + 1}" for i in range(7)]

_FLOATING_URDF = """<robot name="floater" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="body"/>
  <joint name="float_joint" type="floating">
    <origin xyz="1 2 3" rpy="0 0 0"/>
    <parent link="world"/>
    <child link="body"/>
  </joint>
</robot>"""


def test_get_init_revision():
    env = _fresh_env()
    init_revision = env.getInitRevision()
    assert init_revision == env.getRevision()
    assert env.applyCommand(tesseract_environment.ChangeLinkVisibilityCommand("link_1", False))
    assert env.getRevision() == init_revision + 1
    assert env.getInitRevision() == init_revision


def test_timestamps_are_datetime():
    env = _fresh_env()
    assert isinstance(env.getTimestamp(), datetime.datetime)
    assert isinstance(env.getCurrentStateTimestamp(), datetime.datetime)

    before = datetime.datetime.now()  # naive local time, like the caster's result
    env.setState(_IIWA_JOINTS, np.full(7, 0.1))
    assert env.getCurrentStateTimestamp() >= before

    stamp = env.getTimestamp()
    assert env.applyCommand(tesseract_environment.ChangeLinkVisibilityCommand("link_1", False))
    assert env.getTimestamp() > stamp


def test_get_joint_limits():
    env = _fresh_env()
    limits = env.getJointLimits("joint_a1")
    assert limits.lower == -2.9668
    assert limits.upper == 2.9668
    with pytest.raises(KeyError, match="not_a_joint"):
        env.getJointLimits("not_a_joint")


def test_link_collision_enabled_and_visibility():
    env = _fresh_env()
    assert env.getLinkCollisionEnabled("link_1") is True
    assert env.getLinkVisibility("link_1") is True
    assert env.applyCommand(
        tesseract_environment.ChangeLinkCollisionEnabledCommand("link_1", False)
    )
    assert env.applyCommand(tesseract_environment.ChangeLinkVisibilityCommand("link_1", False))
    assert env.getLinkCollisionEnabled("link_1") is False
    assert env.getLinkVisibility("link_1") is False
    with pytest.raises(KeyError, match="not_a_link"):
        env.getLinkCollisionEnabled("not_a_link")
    with pytest.raises(KeyError, match="not_a_link"):
        env.getLinkVisibility("not_a_link")


def test_get_link_transforms_overloads():
    env = _fresh_env()
    assert len(env.getLinkTransforms()) == len(env.getLinkNames())

    values = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    transforms = env.getLinkTransforms(_IIWA_JOINTS, values)
    assert isinstance(transforms, dict)
    assert set(transforms) == set(env.getLinkNames())

    with_floating = env.getLinkTransforms(_IIWA_JOINTS, values, {})
    np.testing.assert_allclose(
        with_floating["tool0"].matrix, transforms["tool0"].matrix, atol=FK_ATOL
    )

    env.setState(_IIWA_JOINTS, values)
    np.testing.assert_allclose(
        transforms["tool0"].matrix, env.getLinkTransform("tool0").matrix, atol=FK_ATOL
    )

    with pytest.raises(ValueError, match="not_a_joint"):
        env.getLinkTransforms(["not_a_joint"], np.array([1.0]))
    with pytest.raises(ValueError, match="length"):
        env.getLinkTransforms(_IIWA_JOINTS, np.array([1.0]))
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.getLinkTransforms(
            _IIWA_JOINTS, values, {"not_a_floating_joint": env.getLinkTransform("tool0")}
        )


def test_get_current_floating_joint_values():
    env = _fresh_env()
    assert env.getCurrentFloatingJointValues() == {}
    assert env.getCurrentFloatingJointValues([]) == {}

    floater = tesseract_environment.Environment()
    scene_graph = tesseract_urdf.parseURDFString(_FLOATING_URDF, TesseractSupportResourceLocator())
    assert floater.init(scene_graph)
    for values in (
        floater.getCurrentFloatingJointValues(),
        floater.getCurrentFloatingJointValues(["float_joint"]),
    ):
        assert set(values) == {"float_joint"}
        np.testing.assert_allclose(values["float_joint"].translation, [1.0, 2.0, 3.0], atol=FK_ATOL)
    with pytest.raises(KeyError, match="not_a_joint"):
        floater.getCurrentFloatingJointValues(["not_a_joint"])


def test_get_contact_managers_plugin_info():
    info = _fresh_env().getContactManagersPluginInfo()
    assert info.discrete_plugin_infos.default_plugin == "BulletDiscreteBVHManager"
    assert info.continuous_plugin_infos.default_plugin == "BulletCastBVHManager"


# gh-187: native Environment overloads (getState/setState with floating joints, the name-taking
# joint/link getters, getJointGroup(name, joint_names), contact managers by name).

# A floating base carrying one prismatic joint: "arm_joint" is the only active joint and
# "float_joint" the only floating joint, so every getState/setState form has a target.
_FLOATING_ARM_URDF = """<robot name="floating_arm" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="body"/>
  <link name="arm"/>
  <joint name="float_joint" type="floating">
    <origin xyz="1 2 3" rpy="0 0 0"/>
    <parent link="world"/>
    <child link="body"/>
  </joint>
  <joint name="arm_joint" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="body"/>
    <child link="arm"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"""

_IIWA_VALUES = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])


def _floating_arm_env():
    env = tesseract_environment.Environment()
    scene_graph = tesseract_urdf.parseURDFString(
        _FLOATING_ARM_URDF, TesseractSupportResourceLocator()
    )
    assert env.init(scene_graph)
    return env


def _translation(x, y, z):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def test_get_state_native_overloads():
    env = _fresh_env()
    joints = dict(zip(_IIWA_JOINTS, _IIWA_VALUES))
    by_names = env.getState(_IIWA_JOINTS, _IIWA_VALUES)
    by_map = env.getState(joints)
    alias_names = env.getStateByNamesAndValues(_IIWA_JOINTS, _IIWA_VALUES)
    alias_map = env.getStateByMap(joints)

    expected_tool0 = env.getLinkTransforms(_IIWA_JOINTS, _IIWA_VALUES)["tool0"].matrix
    for state in (by_names, by_map, alias_names, alias_map):
        assert state.joints == joints
        np.testing.assert_allclose(
            state.link_transforms["tool0"].matrix, expected_tool0, atol=FK_ATOL
        )

    # getState computes a state; it never moves the environment.
    np.testing.assert_array_equal(env.getCurrentJointValues(), np.zeros(len(_IIWA_JOINTS)))


def test_get_state_unknown_joint_raises():
    # Before #187 these returned a state with the bogus key inserted, and a names/values length
    # mismatch read joint_values out of bounds.
    env = _fresh_env()
    for get_state in (env.getState, env.getStateByMap):
        with pytest.raises(ValueError, match="not_a_joint"):
            get_state({"not_a_joint": 1.0})
        with pytest.raises(ValueError, match="joint_a7-tool0"):
            get_state({"joint_a7-tool0": 1.0})  # fixed joint: not in the state solver
    for get_state in (env.getState, env.getStateByNamesAndValues):
        with pytest.raises(ValueError, match="not_a_joint"):
            get_state(["not_a_joint"], np.array([1.0]))
        with pytest.raises(ValueError, match="length"):
            get_state(_IIWA_JOINTS, np.array([1.0]))


def test_get_state_floating_joints():
    env = _floating_arm_env()
    pose = _translation(4.0, 5.0, 6.0)

    state = env.getState({"float_joint": pose})
    np.testing.assert_allclose(
        state.link_transforms["body"].translation, [4.0, 5.0, 6.0], atol=FK_ATOL
    )
    for state in (
        env.getState({"arm_joint": 0.5}, {"float_joint": pose}),
        env.getState(["arm_joint"], np.array([0.5]), {"float_joint": pose}),
        env.getStateByMap({"arm_joint": 0.5}, {"float_joint": pose}),
        env.getStateByNamesAndValues(["arm_joint"], np.array([0.5]), {"float_joint": pose}),
    ):
        np.testing.assert_allclose(
            state.link_transforms["arm"].translation, [4.5, 5.0, 6.0], atol=FK_ATOL
        )

    # The URDF origin is still the current floating-joint value.
    np.testing.assert_allclose(
        env.getCurrentFloatingJointValues()["float_joint"].translation,
        [1.0, 2.0, 3.0],
        atol=FK_ATOL,
    )


def test_set_state_floating_joints():
    env = _floating_arm_env()

    env.setState({"float_joint": _translation(4.0, 5.0, 6.0)})
    np.testing.assert_allclose(
        env.getCurrentFloatingJointValues()["float_joint"].translation,
        [4.0, 5.0, 6.0],
        atol=FK_ATOL,
    )

    env.setState({"arm_joint": 0.25}, {"float_joint": _translation(7.0, 8.0, 9.0)})
    np.testing.assert_array_equal(env.getCurrentJointValues(), [0.25])
    np.testing.assert_allclose(
        env.getLinkTransform("arm").translation, [7.25, 8.0, 9.0], atol=FK_ATOL
    )

    for set_state in (env.setState, env.setStateByNamesAndValues):
        set_state(["arm_joint"], np.array([-0.5]), {"float_joint": _translation(0.0, 0.0, 1.0)})
        np.testing.assert_array_equal(env.getCurrentJointValues(), [-0.5])
        np.testing.assert_allclose(
            env.getLinkTransform("arm").translation, [-0.5, 0.0, 1.0], atol=FK_ATOL
        )


def test_floating_joints_unknown_raises():
    env = _floating_arm_env()
    pose = _translation(4.0, 5.0, 6.0)
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.getState({"not_a_floating_joint": pose})
    with pytest.raises(ValueError, match="arm_joint"):
        env.getState({"arm_joint": pose})  # active, not floating
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.getState(["arm_joint"], np.array([0.5]), {"not_a_floating_joint": pose})
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.setState({"not_a_floating_joint": pose})

    # Validated before any value is stored: a rejected call leaves the joint values untouched
    # (upstream throws from the floating-joint lookup after storing them).
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.setState({"arm_joint": 0.5}, {"not_a_floating_joint": pose})
    with pytest.raises(ValueError, match="not_a_floating_joint"):
        env.setState(["arm_joint"], np.array([0.5]), {"not_a_floating_joint": pose})
    np.testing.assert_array_equal(env.getCurrentJointValues(), [0.0])


def test_get_current_joint_values_by_names():
    env = _fresh_env()
    env.setState(_IIWA_JOINTS, _IIWA_VALUES)
    np.testing.assert_array_equal(
        env.getCurrentJointValues(_IIWA_JOINTS), env.getCurrentJointValues()
    )
    for get_values in (env.getCurrentJointValues, env.getCurrentJointValuesByNames):
        np.testing.assert_array_equal(get_values(["joint_a3", "joint_a1"]), [0.3, 0.1])
        with pytest.raises(KeyError, match="not_a_joint"):
            get_values(["not_a_joint"])
        with pytest.raises(KeyError, match="joint_a7-tool0"):
            get_values(["joint_a7-tool0"])  # fixed joint: no value in the state


def test_link_names_by_joint_names():
    env = _fresh_env()
    active_joints = env.getActiveJointNames()
    assert set(env.getActiveLinkNames(active_joints)) == set(env.getActiveLinkNames())
    assert set(env.getStaticLinkNames(active_joints)) == set(env.getStaticLinkNames())

    downstream = {"link_7", "tool0"}
    assert set(env.getActiveLinkNames(["joint_a7"])) == downstream
    assert set(env.getStaticLinkNames(["joint_a7"])) == set(env.getLinkNames()) - downstream
    # Any joint type: the fixed tool joint moves only tool0.
    assert env.getActiveLinkNames(["joint_a7-tool0"]) == ["tool0"]

    with pytest.raises(ValueError, match="not_a_joint"):
        env.getActiveLinkNames(["not_a_joint"])
    with pytest.raises(ValueError, match="not_a_joint"):
        env.getStaticLinkNames(["not_a_joint"])


def test_get_joint_group_from_joint_names():
    env = _fresh_env()
    names = env.getGroupJointNames("manipulator")
    group = env.getJointGroup("from_names", names)
    assert group.getName() == "from_names"
    assert group.getJointNames() == names
    np.testing.assert_allclose(
        group.calcFwdKin(_IIWA_VALUES)["tool0"].matrix,
        env.getJointGroup("manipulator").calcFwdKin(_IIWA_VALUES)["tool0"].matrix,
        atol=FK_ATOL,
    )

    with pytest.raises(ValueError, match="not_a_joint"):
        env.getJointGroup("bad", ["joint_a1", "not_a_joint"])
    # A fixed joint exists but has no degree of freedom; upstream's KDL sub-tree check rejects it.
    with pytest.raises(RuntimeError, match="sub-tree"):
        env.getJointGroup("bad", ["joint_a7-tool0"])


def test_get_contact_manager_by_name():
    env = _fresh_env()
    discrete = env.getDiscreteContactManager("BulletDiscreteSimpleManager")
    continuous = env.getContinuousContactManager("BulletCastSimpleManager")
    assert discrete.getName() == "BulletDiscreteSimpleManager"
    assert continuous.getName() == "BulletCastSimpleManager"
    assert set(discrete.getActiveCollisionObjects()) == set(env.getActiveLinkNames())
    assert set(continuous.getActiveCollisionObjects()) == set(env.getActiveLinkNames())

    # A copy by name: the active managers stay the defaults.
    assert env.getDiscreteContactManager().getName() == "BulletDiscreteBVHManager"
    assert env.getContinuousContactManager().getName() == "BulletCastBVHManager"

    with pytest.raises(KeyError, match="not_a_manager"):
        env.getDiscreteContactManager("not_a_manager")
    with pytest.raises(KeyError, match="not_a_manager"):
        env.getContinuousContactManager("not_a_manager")
    # Registered names are per kind: a continuous plugin is not a discrete manager.
    with pytest.raises(KeyError, match="BulletCastBVHManager"):
        env.getDiscreteContactManager("BulletCastBVHManager")


# Allowed pair: SRDF disable_collisions entry (Adjacent). Disallowed pair: no SRDF entry.
_ACM_ALLOWED_PAIR = ("link_1", "link_2")
_ACM_DISALLOWED_PAIR = ("base_link", "link_5")


# gh-189: Environment installs an EnvironmentContactAllowedValidator on every contact manager
# it creates; unbound, it came back as the bare ContactAllowedValidator base.
def test_env_contact_manager_validator_type():
    env = _fresh_env()
    for manager in (env.getDiscreteContactManager(), env.getContinuousContactManager()):
        validator = manager.getContactAllowedValidator()
        assert isinstance(validator, tesseract_environment.EnvironmentContactAllowedValidator)
        assert isinstance(validator, tesseract_common.ContactAllowedValidator)


def test_env_contact_allowed_validator_matches_acm():
    env = _fresh_env()
    validator = tesseract_environment.EnvironmentContactAllowedValidator(env.getSceneGraph())
    acm = env.getAllowedCollisionMatrix()
    assert validator(*_ACM_ALLOWED_PAIR) is True
    assert validator(*_ACM_DISALLOWED_PAIR) is False
    for pair in (_ACM_ALLOWED_PAIR, _ACM_DISALLOWED_PAIR):
        assert validator(*pair) == acm.isCollisionAllowed(*pair)


def test_env_contact_allowed_validator_tracks_scene_graph():
    # The validator holds the environment's live scene graph, not a snapshot of its ACM.
    env = _fresh_env()
    validator = tesseract_environment.EnvironmentContactAllowedValidator(env.getSceneGraph())
    acm = tesseract_common.AllowedCollisionMatrix()
    acm.addAllowedCollision(*_ACM_DISALLOWED_PAIR, "test")
    assert env.applyCommand(
        tesseract_environment.ModifyAllowedCollisionsCommand(
            acm, tesseract_environment.ModifyAllowedCollisionsType_ADD
        )
    )
    assert validator(*_ACM_DISALLOWED_PAIR) is True


def test_env_contact_allowed_validator_keeps_scene_graph_alive():
    scene_graph = get_scene_graph()
    scene_graph.addAllowedCollision(*_ACM_ALLOWED_PAIR, "Adjacent")
    validator = tesseract_environment.EnvironmentContactAllowedValidator(scene_graph)
    del scene_graph
    gc.collect()
    assert validator(*_ACM_ALLOWED_PAIR) is True
    assert validator(*_ACM_DISALLOWED_PAIR) is False

    env = _fresh_env()
    installed = env.getDiscreteContactManager().getContactAllowedValidator()
    del env
    gc.collect()
    assert installed(*_ACM_ALLOWED_PAIR) is True


def test_env_contact_allowed_validator_rejects_none():
    # A null scene graph would be dereferenced by operator(); neither path may build one.
    with pytest.raises(TypeError):
        tesseract_environment.EnvironmentContactAllowedValidator(None)
    with pytest.raises(TypeError):
        tesseract_environment.EnvironmentContactAllowedValidator()


def test_env_contact_allowed_validator_on_contact_manager():
    env = _fresh_env()
    validator = tesseract_environment.EnvironmentContactAllowedValidator(env.getSceneGraph())
    manager = env.getDiscreteContactManager()
    manager.setContactAllowedValidator(validator)
    assert manager.getContactAllowedValidator() is validator
    combined = tesseract_common.CombinedContactAllowedValidator(
        [validator, tesseract_common.ACMContactAllowedValidator()],
        tesseract_common.CombinedContactAllowedValidatorType.OR,
    )
    assert combined(*_ACM_ALLOWED_PAIR) is True
    assert combined(*_ACM_DISALLOWED_PAIR) is False


# gh-191: events reach Python callbacks as C++ stack references that dangle once the
# callback returns. The tests below keep only classes, booleans, numbers and strings
# from a callback, never an event object (nor an exception, whose traceback frame would
# hold one).


def test_event_callback_receives_derived_type():
    """Callbacks get the most-derived event class, so they need no cast_*Event."""
    env = _fresh_env()
    received = []

    env.addEventCallback(
        1, tesseract_environment.EventCallbackFn(lambda evt: received.append(type(evt)))
    )
    env.setState(_IIWA_JOINTS, np.zeros(7))
    assert env.applyCommand(tesseract_environment.RemoveJointCommand("joint_a7-tool0"))

    assert tesseract_environment.SceneStateChangedEvent in received
    assert tesseract_environment.CommandAppliedEvent in received
    assert tesseract_environment.Event not in received


def test_cast_event_matching_type_roundtrips():
    """A matching cast returns the event object itself, with its fields readable."""
    env = _fresh_env()
    values = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    command_applied = []
    state_changed = []

    def on_event(evt):
        if evt.type == tesseract_environment.Events.COMMAND_APPLIED:
            cast = tesseract_environment.cast_CommandAppliedEvent(evt)
            command_applied.append((type(cast), cast is evt, cast.revision))
        else:
            cast = tesseract_environment.cast_SceneStateChangedEvent(evt)
            state_changed.append(
                (type(cast), cast is evt, {name: cast.state.joints[name] for name in _IIWA_JOINTS})
            )

    env.addEventCallback(1, tesseract_environment.EventCallbackFn(on_event))
    env.setState(_IIWA_JOINTS, values)
    revision = env.getRevision()
    assert env.applyCommand(tesseract_environment.RemoveJointCommand("joint_a7-tool0"))

    assert command_applied == [(tesseract_environment.CommandAppliedEvent, True, revision + 1)]
    assert state_changed, "no SCENE_STATE_CHANGED event"
    first_type, first_is_evt, first_joints = state_changed[0]
    assert first_type is tesseract_environment.SceneStateChangedEvent
    assert first_is_evt
    assert first_joints == dict(zip(_IIWA_JOINTS, values.tolist()))


# Before the fix the mismatched cast was an unchecked static_cast (undefined behaviour),
# so it runs in a child interpreter: a crash fails this test instead of the test runner.
_WRONG_CAST_SCRIPT = """\
import json
from pathlib import Path
import numpy as np
from tesseract_robotics import tesseract_environment as te
from tesseract_robotics.tesseract_common import GeneralResourceLocator
locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf").getFilePath()
env = te.Environment()
assert env.init(Path(urdf), Path(srdf), locator)
wrong_cast = {
    te.Events.SCENE_STATE_CHANGED: te.cast_CommandAppliedEvent,
    te.Events.COMMAND_APPLIED: te.cast_SceneStateChangedEvent,
}
results = []
def on_event(evt):
    cast = wrong_cast[evt.type]
    try:
        cast(evt)
    except Exception as exc:
        results.append({
            "cast": cast.__name__,
            "raised": type(exc).__name__,
            "is_event_type_error": type(exc) is te.EventTypeError,
            "is_type_error": isinstance(exc, TypeError),
            "message": str(exc),
        })
    else:
        results.append({"cast": cast.__name__, "raised": None})
env.addEventCallback(1, te.EventCallbackFn(on_event))
env.setState([f"joint_a{i + 1}" for i in range(7)], np.zeros(7))
assert env.applyCommand(te.RemoveJointCommand("joint_a7-tool0"))
print("RESULTS " + json.dumps(results))
"""

_WRONG_CAST_MESSAGES = {
    "cast_CommandAppliedEvent": (
        "cast_CommandAppliedEvent: event type is Events.SCENE_STATE_CHANGED, "
        "expected Events.COMMAND_APPLIED"
    ),
    "cast_SceneStateChangedEvent": (
        "cast_SceneStateChangedEvent: event type is Events.COMMAND_APPLIED, "
        "expected Events.SCENE_STATE_CHANGED"
    ),
}


def test_cast_event_wrong_type_raises():
    """Casting an event to the other event class raises EventTypeError, in both directions."""
    proc = subprocess.run(
        [sys.executable, "-c", _WRONG_CAST_SCRIPT], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGSEGV is -11/139): {proc.stderr[-1000:]}"
    )
    lines = [line for line in proc.stdout.splitlines() if line.startswith("RESULTS ")]
    assert len(lines) == 1, proc.stdout[-1000:]
    results = json.loads(lines[0].removeprefix("RESULTS "))

    assert {r["cast"] for r in results} == set(_WRONG_CAST_MESSAGES), results
    for r in results:
        assert r["raised"] == "EventTypeError", r
        assert r["is_event_type_error"], r
        assert r["is_type_error"], r
        assert r["message"] == _WRONG_CAST_MESSAGES[r["cast"]], r

    assert issubclass(tesseract_environment.EventTypeError, TypeError)


# gh-183: Python find-TCP callbacks.

# A tcp_offset name that is neither a link of the iiwa scene nor a group TCP in its SRDF, so
# findTCPOffset can only resolve it through a registered callback.
_PY_TCP_NAME = "py_tcp"
# Translation of the callback's TCP offset, in m; differs from both SRDF group TCPs.
_PY_TCP_XYZ = (0.1, 0.2, 0.3)

_FIND_TCP_TEARDOWN_SCRIPT = """\
from pathlib import Path
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf").getFilePath()
env = Environment()
assert env.init(Path(urdf), Path(srdf), locator)
# The default argument is a nanobind instance owned by the callback: if the binding never releases
# the callback, nanobind reports it as a leaked instance at exit. The callback gets its own globals:
# a function defined here would hold this module's globals, and so env, which holds the callback
# in C++, a cycle the garbage collector cannot see (the documented "must not hold a reference to
# it" rule), which leaks at exit whatever the binding does.
find_tcp = eval(
    "lambda manip_info, offset=Isometry3d.Identity(): offset", {"Isometry3d": Isometry3d}
)
env.addFindTCPOffsetCallback(find_tcp)
clone = env.clone()
tcp = clone.findTCPOffset(ManipulatorInfo("manipulator", "base_link", "tool0", "py_tcp"))
print("OK:", tcp.translation.tolist())
"""


def _py_tcp_offset():
    matrix = np.eye(4)
    matrix[:3, 3] = _PY_TCP_XYZ
    return Isometry3d(matrix)


def _manip_info(tcp_offset=_PY_TCP_NAME):
    return ManipulatorInfo("manipulator", "base_link", "tool0", tcp_offset)


class _TcpSource:
    """Find-TCP callback that records the tcp_offset names it is asked for; weakref-able."""

    def __init__(self):
        self.names = []

    def __call__(self, manip_info):
        self.names.append(manip_info.tcp_offset)
        return _py_tcp_offset()


def test_find_tcp_offset_callback():
    env = _fresh_env()
    source = _TcpSource()
    env.addFindTCPOffsetCallback(source)

    tcp = env.findTCPOffset(_manip_info())

    np.testing.assert_array_equal(tcp.matrix, _py_tcp_offset().matrix)
    assert source.names == [_PY_TCP_NAME]
    # an SRDF group TCP resolves before any callback is asked
    env.findTCPOffset(_manip_info("laser"))
    assert source.names == [_PY_TCP_NAME]


def test_find_tcp_offset_callback_survives_clone():
    """clone() copies the callback with the GIL released; the copy outlives the original."""
    env = _fresh_env()
    env.addFindTCPOffsetCallback(_TcpSource())
    clone = env.clone()
    del env
    gc.collect()

    tcp = clone.findTCPOffset(_manip_info())
    np.testing.assert_array_equal(tcp.matrix, _py_tcp_offset().matrix)

    results, errors = [], []

    def worker():
        try:
            results.append(clone.findTCPOffset(_manip_info()))
        except BaseException as exc:  # re-raised in the main thread below
            errors.append(exc)

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()
    assert not errors, errors
    np.testing.assert_array_equal(results[0].matrix, _py_tcp_offset().matrix)


def test_find_tcp_offset_callback_on_native_thread():
    """A thread without a Python thread state calls the callback and drops its last owner."""
    from tesseract_robotics.tesseract_environment._tesseract_environment import (
        _find_tcp_offset_on_native_thread,
    )

    env = _fresh_env()
    source = _TcpSource()
    source_ref = weakref.ref(source)
    names = source.names  # outlives source, which the oracle's thread releases
    env.addFindTCPOffsetCallback(source)
    del source
    clone = env.clone()  # C++-constructed, so its ownership can move to the oracle
    del env
    gc.collect()
    assert source_ref() is not None, "the clone's copy keeps the callback alive"

    tcp = _find_tcp_offset_on_native_thread(clone, _manip_info())

    np.testing.assert_array_equal(tcp.matrix, _py_tcp_offset().matrix)
    assert names == [_PY_TCP_NAME]
    gc.collect()
    assert source_ref() is None, "the native thread did not release the callback"


def _raises(manip_info):
    raise ValueError("not my tcp")


@pytest.mark.parametrize(
    "callback",
    [_raises, lambda manip_info: None, lambda manip_info: "not a transform"],
    ids=["raises", "returns-none", "returns-str"],
)
def test_find_tcp_offset_callback_failure_is_not_found(callback):
    """Upstream catches every callback exception and reports the name as not found."""
    env = _fresh_env()
    env.addFindTCPOffsetCallback(callback)
    with pytest.raises(RuntimeError, match=f"Could not find tcp by name {_PY_TCP_NAME}"):
        env.findTCPOffset(_manip_info())


def test_find_tcp_offset_callbacks_tried_in_order():
    """A failing callback is skipped and the next one asked."""
    env = _fresh_env()
    source = _TcpSource()
    env.addFindTCPOffsetCallback(_raises)
    env.addFindTCPOffsetCallback(source)

    np.testing.assert_array_equal(env.findTCPOffset(_manip_info()).matrix, _py_tcp_offset().matrix)
    assert source.names == [_PY_TCP_NAME]


def test_find_tcp_offset_callback_lifetime():
    """The environment keeps the callback alive, and releases it with its last copy."""
    env = _fresh_env()
    source = _TcpSource()
    source_ref = weakref.ref(source)
    env.addFindTCPOffsetCallback(source)
    del source
    gc.collect()

    np.testing.assert_array_equal(env.findTCPOffset(_manip_info()).matrix, _py_tcp_offset().matrix)

    clone = env.clone()
    del env
    gc.collect()
    assert source_ref() is not None, "the clone's copy keeps the callback alive"
    del clone
    gc.collect()
    assert source_ref() is None, "the callback leaked"


def test_find_tcp_offset_callback_interpreter_teardown():
    """Clean exit with env, clone and callback as module globals; the callback is released."""
    proc = subprocess.run(
        [sys.executable, "-c", _FIND_TCP_TEARDOWN_SCRIPT],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, (
        f"interpreter teardown died (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[-500:]}"
    )
    assert "OK: [0.0, 0.0, 0.0]" in proc.stdout
    assert "leaked" not in proc.stderr, proc.stderr[-500:]
