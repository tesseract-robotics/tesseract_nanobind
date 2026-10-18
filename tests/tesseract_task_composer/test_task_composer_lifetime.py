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
import time
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
for _ in range(5):
    node = factory.createTaskComposerNode("TrajOptPipeline")
    executor.run(node, make_storage(node))
    del node
    gc.collect()
while executor.getTaskCount():
    time.sleep(0.1)
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
import time
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
context = future.context
future.clear()
del future, node
gc.collect()
while executor.getTaskCount():
    time.sleep(0.1)
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
import time
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
for _ in range(3):
    node = factory.createTaskComposerNode("TrajOptPipeline")
    server.run(node, make_storage(node), False, "TaskflowExecutor")
    del node
    gc.collect()
while server.getTaskCount("TaskflowExecutor"):
    time.sleep(0.1)
print("TASKS", server.getTaskCount("TaskflowExecutor"))
""")
        assert "TASKS 0" in out, out

    def test_long_copy_chain(self):
        """Each copy kept the one it was made from alive: releasing a long chain recursed
        through every dealloc and overflowed the stack."""
        out = _run("""
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
duplicate = future
for _ in range(200000):
    duplicate = duplicate.copy()
duplicate.wait()
del future, node
duplicate = None
print("RELEASED")
""")
        assert "RELEASED" in out, out


# A Python console_bridge handler: executor workers call it while a run is in flight.
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


class TestFutureReleasedOffTheMainThread:
    """Releasing a future never waits for its run: the last reference can go on an executor
    worker (a callback, or gc there), where waiting for the run is waiting for itself."""

    def test_last_reference_dropped_on_a_worker(self):
        out = _run(
            _WORKER_HANDLER
            + """
def drop():
    globals()["future"] = None
    print("DROPPED ON WORKER", flush=True)

sys_handler = WorkerHook(drop)
tesseract_common.useOutputHandler(sys_handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
del node
while not sys_handler.fired or executor.getTaskCount():
    time.sleep(0.1)
print("TASKS", executor.getTaskCount())
"""
        )
        assert "DROPPED ON WORKER" in out, out
        assert "TASKS 0" in out, out

    def test_gc_on_a_worker_collects_a_running_future(self):
        out = _run(
            _WORKER_HANDLER
            + """
def collect():
    gc.collect()
    print("COLLECTED ON WORKER", flush=True)

sys_handler = WorkerHook(collect)
tesseract_common.useOutputHandler(sys_handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
gc.disable()
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
cycle = [executor.run(node, make_storage(node))]
cycle.append(cycle)
del cycle, node
while not sys_handler.fired or executor.getTaskCount():
    time.sleep(0.1)
print("TASKS", executor.getTaskCount())
"""
        )
        assert "COLLECTED ON WORKER" in out, out
        assert "TASKS 0" in out, out

    def test_exit_with_runs_in_flight(self):
        """Interpreter exit with runs in flight, whose workers call a Python log handler."""
        out = _run(
            _WORKER_HANDLER
            + """
import sys
sys.handler = WorkerHook(lambda: None)
tesseract_common.useOutputHandler(sys.handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
futures = []
for _ in range(4):
    node = factory.createTaskComposerNode("TrajOptPipeline")
    futures.append(executor.run(node, make_storage(node)))
print("EXITING", flush=True)
"""
        )
        assert "EXITING" in out, out


class TestServerAndExitDrain:
    """Server runs hold the executor they run on; the exit drain catches runs started during it."""

    def test_executor_replaced_mid_run(self):
        """addExecutor replaced the running executor with the GIL held: its destructor joined
        workers blocked on the GIL in a Python log handler (a deadlock), and a run that survived
        the join lost its executor."""
        out = _run(
            _WORKER_HANDLER
            + """
handler = WorkerHook(lambda: None)
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
name = "TaskflowExecutor"
node = factory.createTaskComposerNode("TrajOptPipeline")
futures = [server.run(node, make_storage(node), False, name) for _ in range(2)]
futures.append(server.run("TrajOptPipeline", make_storage(node), False, name))
server.addExecutor(factory.createTaskComposerExecutor(name))
for future in futures:
    future.wait()
print("SUCCESS", all(future.context.isSuccessful() for future in futures))
tesseract_common.restorePreviousOutputHandler()
"""
        )
        assert "SUCCESS True" in out, out

    def test_run_started_during_exit_drain_finishes(self):
        """A worker callback that starts a run while the exit drain waits: the drain repeats
        until no run is left, so the run is not cut off by interpreter teardown."""
        out = _run(
            """
import atexit
report = {}
atexit.register(lambda: print("NESTED TASKS", report["executor"].getTaskCount(), flush=True))
"""
            + _PREAMBLE
            + _WORKER_HANDLER
            + """
exiting = []
atexit.register(lambda: exiting.append(True))
from tesseract_robotics.tesseract_task_composer import TaskflowTaskComposerExecutor
# One worker, four runs: the nested runs outlast the run the drain is waiting for.
nested = TaskflowTaskComposerExecutor("nested", 1)
report["executor"] = nested
nested_node = factory.createTaskComposerNode("TrajOptPipeline")
nested_storage = make_storage(nested_node)

class StartAtExit(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.started = False

    def log(self, text, level, filename, line):
        if exiting and not self.started and threading.get_ident() != MAIN:
            self.started = True
            for _ in range(4):
                nested.run(nested_node, nested_storage)
            print("NESTED STARTED", flush=True)

handler = StartAtExit()
tesseract_common.useOutputHandler(handler)
tesseract_common.setLogLevel(tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_DEBUG)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
node = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(node, make_storage(node))
""",
            planning=False,
        )
        assert "NESTED STARTED" in out, out
        assert "NESTED TASKS 0" in out, out


class TestServerTaskReplacement:
    """A server task cannot be replaced (loadConfig, addTask with its name) while Python or a run
    can reach it: upstream exposes it only as a `const&`, so the bindings cannot hold it, and
    replacing it freed it under the run or behind a getTask() reference."""

    def test_load_config_mid_run_raises(self):
        out = _run("""
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
futures = [server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor") for _ in range(4)]
for load in (lambda: server.loadConfig(config_path, GeneralResourceLocator()),
             lambda: server.loadConfig(config_path.read_text(), GeneralResourceLocator())):
    try:
        load()
    except TaskComposerServerBusyError as e:
        print("BUSY", "TrajOptPipeline" in str(e))
for future in futures:
    future.wait()
print("SUCCESS", all(future.context.isSuccessful() for future in futures))
""")
        assert out.count("BUSY True") == 2, out
        assert "SUCCESS True" in out, out

    def test_add_task_with_the_running_name_raises(self):
        out = _run("""
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
future = server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor")
try:
    server.addTask(factory.createTaskComposerNode("TrajOptPipeline"))
except TaskComposerServerBusyError as e:
    print("BUSY", "TrajOptPipeline" in str(e))
future.wait()
print("SUCCESS", future.context.isSuccessful())
""")
        assert "BUSY True" in out, out
        assert "SUCCESS True" in out, out

    def test_server_task_run_on_an_executor_blocks_reload(self):
        """A task reached through getTask and run on any executor (or by server.run(node)) is
        the server's: reloading freed it under the run."""
        out = _run("""
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
task = server.getTask("TrajOptPipeline")
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
futures = [executor.run(task, make_storage(task)) for _ in range(2)]
futures.append(server.run(task, make_storage(task), False, "TaskflowExecutor"))
del task
try:
    server.loadConfig(config_path, GeneralResourceLocator())
except TaskComposerServerBusyError as e:
    print("BUSY", "running" in str(e))
for future in futures:
    future.wait()
print("SUCCESS", all(future.context.isSuccessful() for future in futures))
server.loadConfig(config_path, GeneralResourceLocator())
print("RELOADED")
""")
        assert "BUSY True" in out, out
        assert "SUCCESS True" in out, out
        assert "RELOADED" in out, out

    def test_get_task_reference_blocks_reload_until_deleted(self):
        """With no run at all, a getTask() result dangled after a reload."""
        out = _run("""
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
task = server.getTask("TrajOptPipeline")
for replace in (lambda: server.loadConfig(config_path, GeneralResourceLocator()),
                lambda: server.addTask(factory.createTaskComposerNode("TrajOptPipeline"))):
    try:
        replace()
    except TaskComposerServerBusyError as e:
        print("BUSY", "referenced from Python" in str(e) and "del" in str(e))
print("NAME", task.getName())
del task
server.loadConfig(config_path, GeneralResourceLocator())
server.addTask(factory.createTaskComposerNode("TrajOptPipeline"))
print("RELOADED")
""")
        assert out.count("BUSY True") == 2, out
        assert "NAME TrajOptPipeline" in out, out
        assert "RELOADED" in out, out

    def test_task_added_from_python_stays_replaceable(self):
        """The server's shared_ptr keeps the wrapper of a task added from Python alive, so the
        wrapper always exists; it owns the task and never dangles, so it must not block."""
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())

def my_task():
    task = factory.createTaskComposerNode("OMPLPipeline")
    task.setName("MyTask")
    return task

task = my_task()
server.addTask(task)
del task
gc.collect()
server.addTask(my_task())
server.loadConfig(config_path, GeneralResourceLocator())
kept = my_task()
server.addTask(kept)
server.addTask(my_task())  # replacing it while Python holds it: Python still owns it
print("REPLACED", kept.getName())
""")
        assert "REPLACED MyTask" in out, out

    def test_shared_child_added_from_python_stays_replaceable(self):
        """A getNodes() child is a shared_ptr the graph shares: its wrapper holds a shared_ptr
        instead of owning the node outright, and it still owns it, so it must not block."""
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())

def child():
    graph = factory.createTaskComposerNode("TrajOptPipeline")
    node = next(iter(graph.getNodes().values()))
    node.setName("Child")
    return node

server.addTask(child())
gc.collect()
server.addTask(child())
server.loadConfig(config_path, GeneralResourceLocator())
print("REPLACED", server.hasTask("Child"))
""")
        assert "REPLACED True" in out, out

    def test_reload_racing_by_name_runs(self):
        """A run registered only after releasing the GIL: a reload on another thread could free
        the task between the lookup and the registration."""
        out = _run("""
import threading
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
stop = []
stats = {"reloads": 0, "busy": 0}

def reload_forever():
    while not stop:
        try:
            server.loadConfig(config_path, GeneralResourceLocator())
            stats["reloads"] += 1
        except TaskComposerServerBusyError:
            stats["busy"] += 1

reloader = threading.Thread(target=reload_forever)
reloader.start()
results = []
for _ in range(15):
    while True:
        try:
            future = server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor")
            break
        except TaskComposerServerBusyError:
            pass  # the reloader is mid-load
    future.wait()
    results.append(future.context.isSuccessful())
stop.append(True)
reloader.join()
print("SUCCESS", all(results), len(results))
""")
        assert "SUCCESS True 15" in out, out

    def test_server_is_busy_while_loading(self, tmp_path):
        """A Python ResourceLocator runs mid-load (and may switch threads there): getTask and
        run on the loading server raise instead of reaching a task being replaced."""
        import yaml

        import tesseract_robotics

        config = yaml.safe_load(tesseract_robotics.get_task_composer_config_path().read_text())
        included = tmp_path / "plugins.yaml"
        included.write_text(yaml.safe_dump(config["task_composer_plugins"]))
        out = _run(f"""
from tesseract_robotics.tesseract_common import ResourceLocator
from tesseract_robotics.tesseract_task_composer import TaskComposerServerBusyError
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")

class Reentrant(ResourceLocator):
    def __init__(self):
        super().__init__()
        self.inner = GeneralResourceLocator()

    def locateResource(self, url):
        for call in (lambda: server.getTask("TrajOptPipeline"),
                     lambda: server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor"),
                     lambda: server.run(node, make_storage(node), False, "TaskflowExecutor")):
            try:
                call()
            except TaskComposerServerBusyError:
                print("BUSY", flush=True)
        return self.inner.locateResource(url)

server.loadConfig("task_composer_plugins: !include file://{included}", Reentrant())
print("LOADED", server.hasTask("TrajOptPipeline"))
""")
        assert out.count("BUSY") == 3, out
        assert "LOADED True" in out, out

    def test_add_task_with_another_name_mid_run_succeeds(self):
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
future = server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor")
other = factory.createTaskComposerNode("OMPLPipeline")
other.setName("AnotherTask")
server.addTask(other)
future.wait()
print("SUCCESS", future.context.isSuccessful(), server.hasTask("AnotherTask"))
""")
        assert "SUCCESS True True" in out, out

    def test_load_config_after_wait_succeeds(self):
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
future = server.run("TrajOptPipeline", make_storage(node), False, "TaskflowExecutor")
future.wait()
server.loadConfig(config_path, GeneralResourceLocator())
server.addTask(factory.createTaskComposerNode("TrajOptPipeline"))
print("RELOADED", future.context.isSuccessful())
""")
        assert "RELOADED True" in out, out

    def test_caller_owned_node_run_does_not_block_reload(self):
        """A run of a node the caller built uses no server task."""
        out = _run("""
server = TaskComposerServer()
server.loadConfig(config_path, GeneralResourceLocator())
node = factory.createTaskComposerNode("TrajOptPipeline")
future = server.run(node, make_storage(node), False, "TaskflowExecutor")
server.loadConfig(config_path, GeneralResourceLocator())
future.wait()
print("SUCCESS", future.context.isSuccessful())
""")
        assert "SUCCESS True" in out, out


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

    def test_prune_conflict_raises(self):
        """An info changed in the container while prune ran is not overwritten by a stale copy."""
        out = _run(
            _CONTAINER
            + """
from tesseract_robotics.tesseract_task_composer import TaskComposerPruneConflictError
target = next(iter(infos.getInfoMap()))

def update_then_edit(info):
    if info.uuid == target:
        stored = infos.getInfo(info.uuid)
        stored.name = "concurrent"
        infos.addInfo(stored)
        info.name = "stale edit"

try:
    infos.prune(update_then_edit)
except TaskComposerPruneConflictError as e:
    print("CONFLICT", target in str(e))
print("STORED", infos.getInfo(target).name)
""",
            planning=False,
        )
        assert "CONFLICT True" in out, out
        assert "STORED concurrent" in out, out

    def test_unedited_infos_are_not_written_back(self):
        out = _run(
            _CONTAINER
            + """
target = next(iter(infos.getInfoMap()))

def update_only(info):
    if info.uuid == target:
        stored = infos.getInfo(info.uuid)
        stored.name = "concurrent"
        infos.addInfo(stored)

infos.prune(update_only)
print("STORED", infos.getInfo(target).name)
""",
            planning=False,
        )
        assert "STORED concurrent" in out, out

    def test_prune_edit_of_a_removed_info_raises(self):
        out = _run(
            _CONTAINER
            + """
from tesseract_robotics.tesseract_task_composer import TaskComposerPruneConflictError

def clear_then_edit(info):
    infos.clear()
    info.name = "edited"

try:
    infos.prune(clear_then_edit)
except TaskComposerPruneConflictError:
    print("CONFLICT")
print("SIZE", len(infos.getInfoMap()))
""",
            planning=False,
        )
        assert "CONFLICT" in out, out
        assert "SIZE 0" in out, out
