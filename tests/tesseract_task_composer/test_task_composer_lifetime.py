"""Lifetimes across a running task-composer run, and callbacks that re-enter their container.

Each case runs in a subprocess: the failures are crashes (use-after-free), wrong results from
freed memory, and hangs, so a case must not take the test process with it.
"""

import subprocess
import sys

import pytest

# A run lasts well under a second here; a hang (dropping the executor mid-run, a re-entrant
# container lock) never ends. Generous enough for a loaded machine: plugin loading alone is ~5 s.
TIMEOUT_S = 60

# The TrajOpt freespace problem of conftest.make_storage, for a fresh interpreter.
_PREAMBLE = """
import gc
import numpy as np
import tesseract_robotics
from tesseract_robotics.planning import JointTarget, MotionProgram, Robot
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
    TaskComposerPluginFactory,
    TaskComposerServer,
)

J_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
J_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
config_path = tesseract_robotics.get_task_composer_config_path()
factory = TaskComposerPluginFactory(config_path, GeneralResourceLocator())
robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
joint_names = robot.get_joint_names("manipulator")
robot.set_joints(J_START, joint_names=joint_names)
program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .set_joint_names(joint_names)
    .move_to(JointTarget(J_START))
    .move_to(JointTarget(J_END))
)
composite = program.to_composite_instruction(joint_names)
assignCurrentStateAsSeed(composite, robot.env)
profiles = create_trajopt_default_profiles()

def make_storage(node):
    keys = node.getInputKeys()
    port = "planning_input" if keys.has("planning_input") else "program"
    storage = TaskComposerDataStorage()
    storage.setData(keys.get(port), AnyPoly_wrap_CompositeInstruction(composite))
    storage.setData("environment", AnyPoly_wrap_EnvironmentConst(robot.env))
    storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))
    return storage
"""


def _run(script: str, *, planning: bool = True) -> str:
    """Run `script` in a fresh interpreter; return its stdout, failing on a crash or a hang."""
    source = (_PREAMBLE if planning else "") + script
    proc = subprocess.run(
        [sys.executable, "-c", source],
        capture_output=True,
        text=True,
        timeout=TIMEOUT_S,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode}: {proc.stderr[-1500:]}"
    return proc.stdout


class TestFutureLifetime:
    """A future keeps everything its run uses alive until the run ends."""

    def test_dropped_future_then_dropped_node(self):
        """Discarding the future at once released the node while the executor still ran it."""
        out = _run("""
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
for _ in range(5):
    node = factory.createTaskComposerNode("TrajOptPipeline")
    executor.run(node, make_storage(node))
    del node
    gc.collect()
print("TASKS", executor.getTaskCount())
""")
        assert "TASKS 0" in out, out

    def test_copy_then_drop_original_and_node(self):
        """A copy held no reference to the node: dropping the original and the node freed it."""
        out = _run("""
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
duplicate = future.copy()
del future, node
gc.collect()
duplicate.wait()
print("SUCCESS", duplicate.context.isSuccessful())
""")
        assert "SUCCESS True" in out, out

    def test_copy_module_then_drop_original_and_node(self):
        out = _run("""
import copy
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
duplicate = copy.copy(future)
del future, node
gc.collect()
duplicate.wait()
print("SUCCESS", duplicate.context.isSuccessful())
""")
        assert "SUCCESS True" in out, out

    def test_cleared_future_then_dropped_node(self):
        """clear() drops the future's shared state, not its hold on the run."""
        out = _run("""
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
context = future.context
future.clear()
del future, node
gc.collect()
print("SUCCESS", context.isSuccessful())
""")
        assert "SUCCESS True" in out, out

    @pytest.mark.parametrize("storage_overload", [True, False])
    def test_dropped_executor_mid_run(self, storage_overload):
        """Dropping the executor mid-run joined its workers from inside the run: a hang."""
        run = (
            "executor.run(node, make_storage(node))"
            if storage_overload
            else 'executor.run(node, TaskComposerContext("run", make_storage(node)))'
        )
        out = _run(f"""
from tesseract_robotics.tesseract_task_composer import TaskComposerContext
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = {run}
del executor
gc.collect()
future.wait()
print("SUCCESS", future.context.isSuccessful())
""")
        assert "SUCCESS True" in out, out

    def test_server_dropped_future_then_dropped_node(self):
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
for _ in range(3):
    node = factory.createTaskComposerNode("TrajOptPipeline")
    server.run(node, make_storage(node), False, "TaskflowExecutor")
    del node
    gc.collect()
print("TASKS", server.getTaskCount("TaskflowExecutor"))
""")
        assert "TASKS 0" in out, out


_CONTAINER = """
import gc
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerGraph,
    TaskComposerNodeInfo,
    TaskComposerNodeInfoContainer,
)
infos = TaskComposerNodeInfoContainer()
for i in range(50):
    infos.addInfo(TaskComposerNodeInfo(TaskComposerGraph("graph_name_long_enough_to_heap_allocate_%d" % i)))
"""


class TestContainerCallbacks:
    """`find` and `prune` call Python on a snapshot, outside the container's lock."""

    def test_prune_argument_outlives_container_and_edits_land(self):
        out = _run(
            _CONTAINER
            + """
kept = []
def rename(info):
    info.name = "pruned_" + info.name
    kept.append(info)
infos.prune(rename)
print("EDITED", all(i.name.startswith("pruned_") for i in infos.getInfoMap().values()))
infos.clear()
del infos
gc.collect()
junk = [("x" * 60 + str(i)).encode() for i in range(20000)]
print("KEPT", len(kept), all(i.name.startswith("pruned_graph_name") for i in kept))
""",
            planning=False,
        )
        assert "EDITED True" in out, out
        assert "KEPT 50 True" in out, out

    def test_find_reenters_container(self):
        out = _run(
            _CONTAINER
            + """
print("FOUND", len(infos.find(lambda i: infos.getInfo(i.uuid) is not None)))
""",
            planning=False,
        )
        assert "FOUND 50" in out, out

    def test_prune_reenters_container(self):
        out = _run(
            _CONTAINER
            + """
infos.prune(lambda i: setattr(i, "color", "green" if infos.getInfo(i.uuid) is not None else "red"))
print("GREEN", all(i.color == "green" for i in infos.getInfoMap().values()))
""",
            planning=False,
        )
        assert "GREEN True" in out, out
