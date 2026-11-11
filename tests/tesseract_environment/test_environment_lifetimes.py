"""Environment objects handed to Python outlive the environment change that made them.

- Events: a callback may keep the event; it refers to its own copy of the history / state.
- getLink / getJoint: the shared scene-graph object, valid after a command removes it.
- Event callbacks with concurrent writers: every Environment binding takes the environment's
  lock without the GIL, so a Python callback (which needs the GIL under the environment's
  shared lock) and a writer on another thread no longer deadlock.
"""

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import environment as te
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator

IIWA_JOINTS = [f"joint_a{i + 1}" for i in range(7)]

# [s] wall time for the concurrent writer/callback run below; it finishes in well under a
# second, so only a deadlock reaches it.
DEADLOCK_TIMEOUT_S = 60


# [s] Per-test bound: ten times the slowest test measured in this file (0.33 s, test_event_callback_with_concurrent_writer_does_not_deadlock),
# rounded up to a multiple of 5 s, and never under 60 s: the bound includes fixture setup, and a
# cold CI worker (Windows above all) spends seconds loading plugins before the first test runs.
# A deadlock or hang never ends; this fails it instead of
# waiting for the session-wide timeout.
TEST_TIMEOUT_S = 60
pytestmark = pytest.mark.timeout(TEST_TIMEOUT_S)


def _env() -> te.Environment:
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
    env = te.Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    return env


def test_kept_events_stay_valid_after_the_callback():
    env = _env()
    kept = []
    key = env.addEventCallback(kept.append)
    env.setState(IIWA_JOINTS, np.full(7, 0.1))
    assert env.applyCommand(te.RemoveJointCommand("joint_a7-tool0"))
    env.removeEventCallback(key)
    env.setState(IIWA_JOINTS, np.zeros(7))  # changes the state the events were taken from

    states = [e for e in kept if e.type == te.Events.SCENE_STATE_CHANGED]
    applied = [e for e in kept if e.type == te.Events.COMMAND_APPLIED]
    assert states and applied
    assert states[0].state.joints["joint_a1"] == pytest.approx(0.1)
    assert applied[-1].commands[-1].getType() == te.CommandType.REMOVE_JOINT
    assert applied[-1].revision == env.getRevision()


def test_add_event_callback_returns_distinct_keys():
    env = _env()
    calls = []
    first = env.addEventCallback(lambda e: calls.append("first"))
    second = env.addEventCallback(lambda e: calls.append("second"))
    assert first != second
    env.removeEventCallback(first)
    env.setState(IIWA_JOINTS, np.zeros(7))
    assert calls == ["second"]


def test_get_link_and_joint_survive_their_removal():
    env = _env()
    link = env.getLink("tool0")
    joint = env.getJoint("joint_a7-tool0")
    assert env.applyCommand(te.RemoveLinkCommand("tool0"))
    assert link.getName() == "tool0"
    assert joint.getName() == "joint_a7-tool0"
    with pytest.raises(KeyError, match="tool0"):
        env.getLink("tool0")
    with pytest.raises(KeyError, match="joint_a7-tool0"):
        env.getJoint("joint_a7-tool0")


@pytest.mark.parametrize(
    "call",
    [
        lambda env: env.getJointGroup("not_a_group"),
        lambda env: env.getGroupJointIds("not_a_group"),
        lambda env: env.getKinematicGroup("not_a_group"),
        lambda env: env.getKinematicGroup("manipulator", "not_a_solver"),
        lambda env: env.getDiscreteContactManager("not_a_manager"),
        lambda env: env.getLinkTransform("not_a_link"),
    ],
    ids=[
        "joint_group",
        "group_joint_ids",
        "kinematic_group",
        "ik_solver",
        "contact_manager",
        "link",
    ],
)
def test_missing_ids_raise_key_error(call):
    with pytest.raises(KeyError, match="not_a_"):
        call(_env())


_DEADLOCK_SCRIPT = """\
import threading
from pathlib import Path
import numpy as np
from tesseract_robotics import environment as te
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf").getFilePath()
env = te.Environment()
assert env.init(Path(urdf), Path(srdf), locator)
joints = [f"joint_a{i + 1}" for i in range(7)]
events = []
env.addEventCallback(lambda e: events.append(e.type))

def writer():
    for i in range(200):
        env.setState(joints, np.full(7, 0.001 * i))

thread = threading.Thread(target=writer)
thread.start()
from tesseract_robotics.geometry import Box
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link
env.getDiscreteContactManager()  # applyCommand then builds collision shapes, the released work
for i in range(100):
    link = Link(f"box{i}")
    collision = Collision()
    collision.geometry = Box(0.1, 0.1, 0.1)
    link.addCollision(collision)
    joint = Joint(f"box{i}_joint")
    joint.type = JointType.FIXED
    joint.parent_link_id = "base_link"
    joint.child_link_id = f"box{i}"
    # AddLinkCommand released the GIL before this fix too: its callback waited for the GIL
    # under the shared lock while setState held the GIL waiting for the unique lock.
    assert env.applyCommand(te.AddLinkCommand(link, joint))
    env.setName(f"env{i}")
thread.join()
print("DONE", len(events) > 0)
"""


def test_event_callback_with_concurrent_writer_does_not_deadlock():
    """Formerly: applyCommand fired the callback under the shared lock without the GIL, while a
    setState on another thread held the GIL waiting for the unique lock."""
    proc = subprocess.run(
        [sys.executable, "-c", _DEADLOCK_SCRIPT],
        capture_output=True,
        text=True,
        timeout=DEADLOCK_TIMEOUT_S,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "DONE True" in proc.stdout
