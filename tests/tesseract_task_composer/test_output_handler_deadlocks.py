"""Task-composer workers logging into Python log handlers.

console_bridge calls its handler with a global mutex held; a Python handler called there waited
for the GIL with the mutex held, against a call that logs while holding the GIL (a failed plugin
lookup). Each case runs in a subprocess with a timeout: the failures are hangs.
"""

import subprocess
import sys

# A load is a few seconds; a deadlock never ends. Generous for a loaded machine.
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
)

J_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
J_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
config_path = tesseract_robotics.get_task_composer_config_path()
factory = TaskComposerPluginFactory(str(config_path), GeneralResourceLocator())
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


_WORKER_HANDLER = """
import threading
import time
from tesseract_robotics import tesseract_common
MAIN = threading.get_ident()

class WorkerHook(tesseract_common.OutputHandler):
    def __init__(self, hook):
        super().__init__()
        self.hook = hook
        self.fired = False

    def log(self, text, level, filename, line):
        if threading.get_ident() != MAIN and not self.fired:
            self.fired = True
            self.hook()
"""


class TestWorkersLoggingIntoPython:
    def test_workers_log_while_a_gil_holding_call_logs(self):
        """A worker logging into a Python handler held console_bridge's mutex while it waited for
        the GIL; the main thread held the GIL in a call that logs (a failed
        createTaskComposerNode) and waited for that mutex: a deadlock."""
        out = _run(
            _WORKER_HANDLER
            + """
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginError
handler = WorkerHook(lambda: None)
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
storage = make_storage(node)
future = executor.run(node, storage)
failed = 0
while not future.ready():
    try:
        factory.createTaskComposerNode("NoSuchPlugin")
    except TaskComposerPluginError:
        failed += 1
future.wait()
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("DONE", failed > 0, handler.fired, future.context.isSuccessful())
"""
        )
        assert "DONE True True True" in out, out
