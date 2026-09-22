import os
import traceback

import numpy as np
import pytest

from tesseract_robotics import tesseract_environment, tesseract_srdf, tesseract_urdf

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
