"""Task-composer workers and server reloads against Python log record handlers.

console_bridge called its handler with a global mutex held; a Python handler called there waited
for the GIL with the mutex held, against a call that logs while holding the GIL (a failed plugin
lookup, a reload). 0.36 logs through spdlog and calls each record handler with no tesseract lock
held (logging.cpp `dispatchHandlers`), on the thread that logged. A handler on another thread
calls a server like any other thread, waiting for a reload. Each case runs in a subprocess with a
timeout: the failures are hangs.
"""

import subprocess
import sys

# A load is a few seconds; a deadlock never ends. Generous for a loaded machine.
TIMEOUT_S = 60
# s: how long a script waits for callers to reach the reload gate (`_reload_gate_waiters`)
# before it fails; well inside TIMEOUT_S, generous for a loaded machine.
GATE_DEADLINE_S = 30

# Polls `_reload_gate_waiters(server)` until it reads at least `count`; exits the script with a
# message on the deadline, so a missed overlap fails instead of passing untested (#370).
_WAIT_AT_GATE = f"""
import sys
import time
from tesseract_robotics.tesseract_task_composer._tesseract_task_composer import _reload_gate_waiters

def wait_at_gate(server, count):
    deadline = time.monotonic() + {GATE_DEADLINE_S}
    while _reload_gate_waiters(server) < count:
        if time.monotonic() > deadline:
            sys.exit(f"{{_reload_gate_waiters(server)}} of {{count}} callers reached the reload "
                     "gate in {GATE_DEADLINE_S} s")
        time.sleep(0.001)
"""

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
    ports = node.getInputPortMappings()
    port = "planning_input" if ports.contains("planning_input") else "program"
    storage = TaskComposerDataStorage()
    storage.setData(ports.single(port), AnyPoly_wrap_CompositeInstruction(composite))
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


# Executor workers log at debug during a TrajOpt run ("Seed Min Length Task Succeeded!", the
# solver's iterations): the handler runs on a worker.
_WORKER_HANDLER = """
import threading
import time
from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_common import LogLevel
MAIN = threading.get_ident()
tesseract_common.getLogger().set_level(LogLevel.debug)

class WorkerHook:
    def __init__(self, hook):
        self.hook = hook
        self.fired = False

    def __call__(self, record):
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
handler_id = tesseract_common.addLogRecordHandler(handler)
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
tesseract_common.removeLogRecordHandler(handler_id)
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
    # A plain absolute path: upstream's GeneralResourceLocator reads only `file:///`, which
    # leaves `/C:/...` on Windows, not an absolute path there.
    return f"task_composer_plugins: !include {included}"


# A handler that calls the server at the record "TOUCH", logged mid-load by another thread (a
# handler runs on the thread that logs, and the reloading thread's own server calls raise), and
# records what the call returned or raised.
_TOUCH_SERVER = (
    _WAIT_AT_GATE
    + """
import threading
import time
from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_common import LogLevel, ResourceLocator
from tesseract_robotics.tesseract_task_composer import TaskComposerServer
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
touched = threading.Event()
results = []

def touch_server(record):
    if "TOUCH" in record.message and not touched.is_set():
        touched.set()
        try:
            results.append(server.hasTask("TrajOptPipeline"))
        except Exception as e:
            results.append(type(e).__name__)

handler_id = tesseract_common.addLogRecordHandler(touch_server)
tesseract_common.getLogger().set_level(LogLevel.debug)
# GeneralResourceLocator logs "Resource not handled: <url>" (a warning) for a relative path.
toucher = threading.Thread(
    target=lambda: GeneralResourceLocator().locateResource("TOUCH/relative/not/handled.stl")
)

def touch_mid_load():
    toucher.start()
    print("TOUCHED", touched.wait(30), flush=True)
    wait_at_gate(server, 1)  # the handler is in the server call while the reload still runs
"""
)


# Several rounds of several callers: the GIL's handover at the end of a reload is a race, which a
# caller on an unfair gate wins now and then, but not every caller of every round.
ROUNDS = 5
CALLERS = 8
# How long a caller that was waiting at the gate gets to finish its call. Generous for a loaded
# machine; it only costs time on a failure.
THROUGH_DEADLINE_S = 5


def _waiting_calls_script(tmp_path) -> str:
    """A server, a config that loads fast and calls the ResourceLocator mid-load (at its
    `!include`), and callers that block on the server until that load ends."""
    import yaml

    import tesseract_robotics

    config = yaml.safe_load(tesseract_robotics.get_task_composer_config_path().read_text())
    plugins = config["task_composer_plugins"]
    included = tmp_path / "plugins.yaml"
    # No executors or tasks: a reload is the gate and the locator, not the plugin loading.
    included.write_text(
        yaml.safe_dump({key: plugins[key] for key in ("search_paths", "search_libraries")})
    )
    # A plain absolute path: see _including_config.
    text = f"task_composer_plugins: !include {included}"
    return (
        _WAIT_AT_GATE
        + f"""
import threading
import time
from tesseract_robotics.tesseract_common import GeneralResourceLocator, ResourceLocator
from tesseract_robotics.tesseract_task_composer import TaskComposerServer
ROUNDS = {ROUNDS}
CALLERS = {CALLERS}
config = {text!r}
server = TaskComposerServer()
done = []
through = []

class MidLoad(ResourceLocator):
    def __init__(self):
        super().__init__()
        self.inner = GeneralResourceLocator()
        self.mid_load = None

    def locateResource(self, url):
        mid_load, self.mid_load = self.mid_load, None
        if mid_load is not None:
            mid_load()
        return self.inner.locateResource(url)

locator = MidLoad()

def call_server():
    server.hasTask("TrajOptPipeline")
    done.append(True)

def start_callers_and_let_them_reach_the_gate(callers):
    for caller in callers:
        caller.start()
    wait_at_gate(server, len(callers))

def record_calls_through():
    # This reload is running, so a call that has not got through waits for it, whatever the wait.
    deadline = time.monotonic() + {THROUGH_DEADLINE_S}
    while len(done) < CALLERS and time.monotonic() < deadline:
        time.sleep(0.01)
    through.append(len(done))
"""
    )


class TestServerReloadAgainstLogHandlers:
    """A Python log handler on another thread calls the server like any other thread: it waits
    for a reload, and the reload waits for it only to take the GIL, so that wait always ends."""

    def test_reload_while_workers_log_into_python(self):
        """loadConfig logged through console_bridge with the GIL held, while a worker held
        console_bridge's output lock waiting for the GIL in a Python log handler: a deadlock.
        The workers' server calls wait for the reloads instead of raising. (Records the
        reloading thread logs itself would raise TaskComposerServerBusyError by design: the
        handler only calls the server from a worker.)"""
        out = _run(
            _WORKER_HANDLER
            + """
from tesseract_robotics.tesseract_task_composer import TaskComposerServer
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
errors = []

def touch_server(record):
    if threading.get_ident() == MAIN:
        return
    try:
        server.hasTask("TrajOptPipeline")
    except Exception as e:
        errors.append(type(e).__name__)

handler_id = tesseract_common.addLogRecordHandler(touch_server)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
reloads = 0
while not future.ready():
    server.loadConfig(config_path, GeneralResourceLocator())
    reloads += 1
future.wait()
tesseract_common.removeLogRecordHandler(handler_id)
print("RELOADS", reloads > 0, future.context.isSuccessful(), sorted(set(errors)))
"""
        )
        assert "RELOADS True True []" in out, out

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
toucher.join()
tesseract_common.removeLogRecordHandler(handler_id)
print("RESULTS", results)
"""
        )
        assert "TOUCHED True" in out, out
        assert "RESULTS [True]" in out, out

    def test_reload_gate_probe_counts_waiters(self, tmp_path):
        """`_reload_gate_waiters` counts the calls blocked on a running reload: one caller
        blocked mid-load reads 1, and 0 once the reload ended and the caller got through."""
        out = _run(
            _waiting_calls_script(tmp_path)
            + """
print("IDLE", _reload_gate_waiters(server), flush=True)
caller = threading.Thread(target=call_server)
seen = []

def block_one_caller():
    caller.start()
    wait_at_gate(server, 1)
    seen.append(_reload_gate_waiters(server))

locator.mid_load = block_one_caller
server.loadConfig(config, locator)
caller.join()
print("WAITERS", seen, _reload_gate_waiters(server), done)
""",
            planning=False,
        )
        assert "IDLE 0" in out, out
        assert "WAITERS [1] 0 [True]" in out, out

    def test_waiting_calls_get_through_between_back_to_back_reloads(self, tmp_path):
        """A call waiting for a reload woke up when it ended and then needed the GIL, which the
        reloading thread kept into its next reload: the call saw that reload running and waited
        again, for as many reloads as the thread kept starting. Every call waiting when a reload
        ends now gets through before the next one starts."""
        out = _run(
            _waiting_calls_script(tmp_path)
            + """
for _ in range(ROUNDS):
    done.clear()
    callers = [threading.Thread(target=call_server) for _ in range(CALLERS)]
    locator.mid_load = lambda: start_callers_and_let_them_reach_the_gate(callers)
    server.loadConfig(config, locator)
    # Back to back, as a `while not future.ready()` loop does: no Python that gives up the GIL.
    locator.mid_load = record_calls_through
    server.loadConfig(config, locator)
    for caller in callers:
        caller.join()
print("THROUGH", through)
""",
            planning=False,
        )
        assert f"THROUGH {[CALLERS] * ROUNDS}" in out, out
