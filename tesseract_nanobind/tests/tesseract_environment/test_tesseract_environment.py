import os

from tesseract_robotics import tesseract_environment
from tesseract_robotics import tesseract_urdf
from tesseract_robotics import tesseract_srdf
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
import traceback
import numpy as np

def get_scene_graph():
    """Return (scene_graph, locator) - locator must be kept alive."""
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator = TesseractSupportResourceLocator()
    # nanobind automatically extracts from unique_ptr, no .release() needed
    return tesseract_urdf.parseURDFFile(path, locator), locator

def get_srdf_model(scene_graph):
    """Return (srdf, locator) - locator must be kept alive."""
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path = os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")
    srdf = tesseract_srdf.SRDFModel()
    locator = TesseractSupportResourceLocator()
    srdf.initFile(scene_graph, path, locator)
    return srdf, locator

def get_environment():
    """Return (env, locators_list) - locators must be kept alive."""
    scene_graph, sg_locator = get_scene_graph()
    assert scene_graph is not None

    srdf, srdf_locator = get_srdf_model(scene_graph)
    assert srdf is not None

    env = tesseract_environment.Environment()
    assert env is not None

    assert env.getRevision() == 0

    success = env.init(scene_graph, srdf)
    assert success
    assert env.getRevision() == 3

    joint_names = [f"joint_a{i+1}" for i in range(7)]
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

    # Return env and all locators that must be kept alive
    return env, [sg_locator, srdf_locator, scene_graph, srdf]

def test_env():
    import gc
    env, refs = get_environment()
    # Cleanup in correct order
    del env
    del refs
    gc.collect()


def test_anypoly_wrap_environment_const():
    """Test wrapping Environment in AnyPoly for TaskComposerDataStorage."""
    import gc
    from tesseract_robotics.tesseract_environment import AnyPoly_wrap_EnvironmentConst

    env, refs = get_environment()
    # AnyPoly_wrap_EnvironmentConst expects shared_ptr<const Environment>
    # The environment is already a shared_ptr from Python's perspective
    any_poly = AnyPoly_wrap_EnvironmentConst(env)
    assert any_poly is not None
    assert not any_poly.isNull()
    # Cleanup in correct order
    del any_poly
    del env
    del refs
    gc.collect()