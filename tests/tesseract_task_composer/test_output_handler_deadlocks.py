"""Task-composer workers and server reloads against Python log handlers.

console_bridge calls its handler with a global mutex held; a Python handler called there waited
for the GIL with the mutex held, against a call that logs while holding the GIL (a failed plugin
lookup, a reload). A handler now runs on a delivery thread and calls a server like any other
thread, waiting for a reload; a reload never waits for the handlers. Each case runs in a
subprocess with a timeout: the failures are hangs.
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


def _including_config(tmp_path) -> str:
    """The default config with its plugins behind an `!include`, so loading it calls the
    ResourceLocator mid-load, while the server is marked as reloading."""
    import yaml

    import tesseract_robotics

    config = yaml.safe_load(tesseract_robotics.get_task_composer_config_path().read_text())
    included = tmp_path / "plugins.yaml"
    included.write_text(yaml.safe_dump(config["task_composer_plugins"]))
    return f"task_composer_plugins: !include file://{included}"


# A handler that calls the server from the delivery thread, at the message "TOUCH" that a
# ResourceLocator logs mid-load, and records what the call returned or raised.
_TOUCH_SERVER = """
import threading
import time
from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_common import ResourceLocator
from tesseract_robotics.tesseract_task_composer import TaskComposerServer
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
touched = threading.Event()
results = []

class TouchServer(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        if "TOUCH" in text and not touched.is_set():
            touched.set()
            try:
                results.append(server.hasTask("TrajOptPipeline"))
            except Exception as e:
                results.append(type(e).__name__)

handler = TouchServer()
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)

def touch_mid_load():
    tesseract_common.log("locator", 0, tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_INFO, "TOUCH")
    print("TOUCHED", touched.wait(30), flush=True)
    time.sleep(0.5)  # the handler is in the server call while the reload still runs
"""


class TestServerReloadAgainstLogHandlers:
    """A Python log handler calls the server like any other thread: it waits for a reload on
    another thread. A reload never waits for the handlers, so that wait always ends."""

    def test_reload_while_workers_log_into_python(self):
        """loadConfig logged through console_bridge with the GIL held, while a worker held
        console_bridge's output lock waiting for the GIL in a Python log handler: a deadlock.
        The handler's server calls wait for the reloads instead of raising."""
        out = _run(
            _WORKER_HANDLER
            + """
from tesseract_robotics.tesseract_task_composer import TaskComposerServer
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
errors = []

class TouchServer(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        try:
            server.hasTask("TrajOptPipeline")
        except Exception as e:
            errors.append(type(e).__name__)

handler = TouchServer()
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
reloads = 0
while not future.ready():
    server.loadConfig(config_path, GeneralResourceLocator())
    reloads += 1
future.wait()
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("RELOADS", reloads > 0, future.context.isSuccessful(), sorted(set(errors)))
"""
        )
        assert "RELOADS True True []" in out, out

    def test_reload_while_a_gil_holding_call_logs(self):
        """The reload, without the GIL, logged into a Python handler: it held console_bridge's
        mutex waiting for the GIL, while a thread holding the GIL logged and waited for that
        mutex (a failed createTaskComposerNode logs before it raises)."""
        out = _run("""
import threading
from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginError, TaskComposerServer
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())

class Quiet(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        pass

handler = Quiet()
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
stop = []
reloads = []

def reload_forever():
    while not stop:
        server.loadConfig(config_path, GeneralResourceLocator())
        reloads.append(1)

reloader = threading.Thread(target=reload_forever)
reloader.start()
failed = 0
for _ in range(100):
    try:
        factory.createTaskComposerNode("NoSuchPlugin")
    except TaskComposerPluginError:
        failed += 1
stop.append(True)
reloader.join()
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("FAILED", failed, len(reloads) > 0)
""")
        assert "FAILED 100 True" in out, out

    def test_log_handler_waits_for_a_reload(self, tmp_path):
        """A server call from a Python log handler while another thread reloaded raised
        TaskComposerServerBusyError; it waits for the reload and gets its answer."""
        out = _run(
            _TOUCH_SERVER
            + f"""
class LogMidLoad(ResourceLocator):
    def __init__(self):
        super().__init__()
        self.inner = GeneralResourceLocator()
        self.done = False

    def locateResource(self, url):
        if not self.done:
            self.done = True
            touch_mid_load()
        return self.inner.locateResource(url)

server.loadConfig({_including_config(tmp_path)!r}, LogMidLoad())
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("RESULTS", results)
"""
        )
        assert "TOUCHED True" in out, out
        assert "RESULTS [True]" in out, out

    def test_flush_inside_a_reload_raises(self, tmp_path):
        """flushOutputHandlers from a ResourceLocator mid-load waited for the handlers while a
        handler waited for that load (a server call): it raises instead, and both finish."""
        out = _run(
            _TOUCH_SERVER
            + f"""
from tesseract_robotics.tesseract_common import OutputHandlerFlushForbiddenError

class FlushMidLoad(ResourceLocator):
    def __init__(self):
        super().__init__()
        self.inner = GeneralResourceLocator()
        self.done = False

    def locateResource(self, url):
        if not self.done:
            self.done = True
            touch_mid_load()
            try:
                tesseract_common.flushOutputHandlers()
                print("FLUSHED", flush=True)
            except OutputHandlerFlushForbiddenError as e:
                print("FORBIDDEN", "TaskComposerServer.loadConfig" in str(e), flush=True)
        return self.inner.locateResource(url)

server.loadConfig({_including_config(tmp_path)!r}, FlushMidLoad())
tesseract_common.flushOutputHandlers()  # outside the load: waits as before
tesseract_common.restorePreviousOutputHandler()
print("RESULTS", results)
"""
        )
        assert "TOUCHED True" in out, out
        assert "FORBIDDEN True" in out, out
        assert "RESULTS [True]" in out, out
