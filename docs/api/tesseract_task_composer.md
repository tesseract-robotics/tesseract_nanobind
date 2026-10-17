# tesseract_robotics.tesseract_task_composer

Task composition and planning pipelines.

## Overview

TaskComposer orchestrates complex planning tasks by chaining planners,
post-processors, and validators into pipelines.

```python
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory, TaskflowTaskComposerExecutor,
    TaskComposerDataStorage, TaskComposerContext, TaskComposerServer,
)
```

## Quick Start

```python
import tesseract_robotics
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_CompositeInstruction, AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst, AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage, TaskComposerPluginFactory,
)

# Create factory and executor
config = tesseract_robotics.get_task_composer_config_path()  # a pathlib.Path
factory = TaskComposerPluginFactory(config, GeneralResourceLocator())
executor = factory.createTaskComposerExecutor("TaskflowExecutor")

# Get pipeline task
task = factory.createTaskComposerNode("TrajOptPipeline")

# Setup data storage
data = TaskComposerDataStorage()
data.setData("planning_input", AnyPoly_wrap_CompositeInstruction(program))
data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

# Execute
future = executor.run(task, data)
future.wait()

# Get result
context = future.context
output_key = task.getOutputKeys().get("program")
result = AnyPoly_as_CompositeInstruction(context.data_storage.getData(output_key))
```

## Components

### TaskComposerPluginFactory

Loads pipeline definitions from YAML config. As on the kinematics and contact-manager plugin
factories, an `os.PathLike` (`pathlib.Path`) is a config file and a `str` is YAML text:

```python
from pathlib import Path

from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory

factory = TaskComposerPluginFactory(Path(config_path), locator)       # a file
factory = TaskComposerPluginFactory(Path(config_path).read_text(), locator)  # YAML text

# Or build it up
factory = TaskComposerPluginFactory()
factory.loadConfig(Path(config_path), locator)

# Get available pipelines
# Common pipelines: TrajOptPipeline, OMPLPipeline, FreespacePipeline
task = factory.createTaskComposerNode("TrajOptPipeline")

# Inspect and edit the loaded config (after `!include` expansion)
factory.getTaskComposerNodePlugins()               # {node name: PluginInfo}, a copy
factory.getTaskComposerExecutorPlugins()           # {executor name: PluginInfo}, a copy
factory.getAvailableTaskComposerNodePlugins()      # node factory class names; loads the plugin libraries
factory.getAvailableTaskComposerExecutorPlugins()  # executor factory class names; the same
factory.addTaskComposerNodePlugin("MyTrajOpt", plugin_info)
factory.createTaskComposerNode("Unconfigured", plugin_info)  # from an explicit PluginInfo
factory.getConfig()                                # the config as a YAML document string
factory.saveConfig(Path("task_composer_plugins.yaml"))
```

A path passed as `str` is parsed as YAML text and raises `TaskComposerConfigError` (a
`ValueError`) naming the text; so does YAML text without a top-level `task_composer_plugins`
mapping (empty, `~`, a list, an unrelated mapping), which upstream loads as an empty config.

`createTaskComposerNode` and `createTaskComposerExecutor` raise `TaskComposerPluginError` (a
`RuntimeError`) when the name is not configured, listing the configured names, or when its plugin
fails to load or build; they never return `None`. Upstream only logs a console_bridge warning
and returns null, so the warning on stderr carries the build failure's cause.
`remove*Plugin` and `setDefault*Plugin` raise `KeyError` for an unknown name.

### TaskflowTaskComposerExecutor

Executes tasks with parallel support.

```python
from tesseract_robotics.tesseract_task_composer import TaskflowTaskComposerExecutor

executor = TaskflowTaskComposerExecutor("MyExecutor", 4)  # name, threads

future = executor.run(task, data_storage)
future.wait()

# Check success
if future.context.isSuccessful():
    print("Planning succeeded")
```

### TaskComposerDataStorage

Key-value storage for pipeline data.

```python
from tesseract_robotics.tesseract_task_composer import TaskComposerDataStorage

data = TaskComposerDataStorage()  # or TaskComposerDataStorage("name")

# Set inputs
data.setData("planning_input", AnyPoly_wrap_CompositeInstruction(program))
data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

# Get outputs
result = data.getData("program")
everything = data.getData()        # {key: AnyPoly}
data.remapData({"program": "seed"})  # rename keys; copy=True keeps both
```

### TaskComposerContext

Execution context with results and status. `executor.run(task, data)` builds one from the data
storage (named `"run"`, or `name=`); build one yourself to choose its name, share a node-info
container, or abort it:

```python
from tesseract_robotics.tesseract_task_composer import TaskComposerContext

context = TaskComposerContext("my_run", data)
future = executor.run(task, context)
future.wait()
assert future.context is context

# Check status
if context.isSuccessful():
    output = context.data_storage.getData(output_key)
else:
    print("Failed")

# Access task info
info = context.task_infos
```

`task.run(context, executor)` runs a node synchronously in the calling thread; graphs and
pipelines still need an executor for their children.

### TaskComposerFuture

Async execution handle.

```python
future = executor.run(task, data)

# Wait for completion
future.wait()

# Or wait with timeout (seconds)
status = future.waitFor(5.0)  # FutureStatus.ready / .timeout

# Get context
context = future.context
```

The run uses the node, the data and the executor after `run` returns. The bindings hold them
until the run ends, independent of the future: dropping the future, the node or the executor
mid-run is safe and never blocks. A finished run's node and executor are released when a future
is seen done (`wait()`, `waitFor()` returning `FutureStatus.ready`, `ready()` returning `True`)
or at the next `run`. A server run also holds the executor it runs on, so
`TaskComposerServer.addExecutor` may replace it mid-run. At interpreter exit, runs still in
flight, including runs started while waiting, are waited for (an `atexit` hook).

### Node infos

Every node that ran records a `TaskComposerNodeInfo` in `context.task_infos`, keyed by node
uuid (a canonical uuid `str`):

```python
infos = future.context.task_infos
root = infos.getInfo(infos.getRootNode())   # the node passed to run()
print(root.name, root.status_message, root.start_time, root.elapsed_time)

for uuid, info in infos.getInfoMap().items():
    print(uuid, info.name, info.return_value)

trajopt = infos.find(lambda i: i.name == "TrajOptMotionPlannerTask")
aborting = infos.getInfo(infos.getAbortingNode())  # None if nothing aborted

# prune edits every stored info: the callback gets a copy, which is stored when prune returns
infos.prune(lambda i: setattr(i, "data_storage", TaskComposerDataStorage()))
```

`find` and `prune` call back on a snapshot taken before the first call, outside the
container's lock, so a callback may call into the container. `prune` stores the copies its
callback changed, each only if the stored info is still the one the callback was given; when one
was changed or removed meanwhile, the other edits are stored and `TaskComposerPruneConflictError`
(a `RuntimeError`) names the conflicting uuids. An info kept from a `prune` callback is an
independent copy: edits made to it after `prune` returns are not stored.

### Graphs and pipelines

Pipelines from the factory are `TaskComposerPipeline` objects (a `TaskComposerGraph`):
their children, terminals and override keys are readable, and a graph can be built in Python
from factory nodes:

```python
from tesseract_robotics.tesseract_task_composer import TaskComposerGraph

pipeline = factory.createTaskComposerNode("TrajOptPipeline")
pipeline.getNodes()     # {uuid: TaskComposerNode}
pipeline.getTerminals()

graph = TaskComposerGraph("two_pass")
first = graph.addNode(factory.createTaskComposerNode("TrajOptPipeline"))
second = graph.addNode(factory.createTaskComposerNode("TrajOptPipeline"))
graph.addEdges(first, [second])
graph.setTerminals([second])
assert graph.isValid()[0]
future = executor.run(graph, data)
```

### TaskComposerServer

Upstream's top-level entry point: one config, executors and tasks by name.

```python
from tesseract_robotics.tesseract_task_composer import TaskComposerServer

server = TaskComposerServer()
server.loadConfig(config, GeneralResourceLocator())  # Path; a str is YAML text
future = server.run("TrajOptPipeline", data, False, "TaskflowExecutor")
future.wait()
```

The server's tasks cannot be held: upstream exposes them only as `const&`. So a server task is
never replaced while a run or Python can reach it. `loadConfig` (which re-creates every task) and
`addTask` with an existing task's name raise `TaskComposerServerBusyError` (a `RuntimeError`)
naming the task and the reason:

- the task is running: a run by name, `server.run(task, ...)` or `executor.run(task, ...)` with a
  `getTask()` result is unfinished. `wait()` for its future first;
- the task is referenced from Python: a `getTask()` result is alive. `del` it first.

```python
task = server.getTask("TrajOptPipeline")
future = executor.run(task, data)
future.wait()
del task
server.loadConfig(config, GeneralResourceLocator())  # now allowed
```

While the server is inside `loadConfig` or `addTask`, where a Python `ResourceLocator` or log
handler can run, `getTask` and `run` on that server raise the same error. Adding a task under a
new name, and running a node you built yourself, are unaffected.

### TaskComposerLog

Upstream's record of a run for replay: the input data, the finished context and the dot graph.
`tesseract_serialization` saves and loads it:

```python
from tesseract_robotics.tesseract_serialization import (
    task_composer_log_from_file, task_composer_log_to_file,
)
from tesseract_robotics.tesseract_task_composer import TaskComposerLog

log = TaskComposerLog("failing plan")
log.initial_data = data            # a copy, taken before the run
future = executor.run(task, data)
future.wait()
log.context = future.context
log.dotgraph = task.getDotgraph(future.context.task_infos.getInfoMap())
task_composer_log_to_file(log, "failing_plan.tclx")
replay = task_composer_log_from_file("failing_plan.tclx")
```

The archive holds only types registered with `tesseract_serialization`'s cereal registry: a data
storage holding planner profiles (`TrajOptDefaultMoveProfile`, …) raises `RuntimeError`
("unregistered polymorphic type"). A loaded log is not `==` to the original: `AnyPoly` compares
`shared_ptr` payloads (environment, profiles, nested data storages) by pointer.

### DOT Graph Export

Every `TaskComposerNode` (pipelines included) exports its task graph as Graphviz DOT, mirroring the C++ `dump()`:

```python
task = factory.createTaskComposerNode("FreespacePipeline")

# Structure only
dot = task.getDotgraph()             # DOT source as a string
task.saveDotgraph("pipeline.dot")    # write to file (str or Path); OSError if it cannot

# Annotated with execution results — pass the run's info map:
# successful nodes render green with execution time, failed/aborted nodes red
future = executor.run(task, data)
future.wait()
results = future.context.task_infos.getInfoMap()
dot = task.getDotgraph(results)
task.saveDotgraph("debug.dot", results)
```

Render with graphviz: `dot -Tsvg pipeline.dot -o pipeline.svg`.

See [Visualizing Pipelines](../user-guide/task-composer.md#visualizing-pipelines) for rendered graphs of the stock pipelines.

## Available Pipelines

### High-Level API (TaskComposer)

The `TaskComposer` class provides user-friendly pipeline names:

| Pipeline | Description |
|----------|-------------|
| `FreespacePipeline` | OMPL + TrajOpt smoothing + time param |
| `CartesianPipeline` | Cartesian path with TrajOpt |
| `OMPLPipeline` | OMPL sampling-based planning only |
| `TrajOptPipeline` | TrajOpt optimization only |
| `DescartesDPipeline` | Descartes graph search, double precision (`DescartesFPipeline` in float; the `*NPC` variants skip the post-plan contact check) |

### Raw Config Pipelines

All pipelines available via `createTaskComposerNode()`:

| Pipeline | Description |
|----------|-------------|
| `FreespacePipeline` | OMPL + TrajOpt + time param |
| `FreespaceIfoptPipeline` | OMPL + TrajOptIfopt (OSQP) |
| `CartesianPipeline` | Cartesian path planning |
| `OMPLPipeline` | OMPL only |
| `TrajOptPipeline` | TrajOpt only |
| `TrajOptIfoptPipeline` | TrajOptIfopt (OSQP solver) |
| `DescartesFPipeline` | Descartes forward search |
| `DescartesDPipeline` | Descartes backward search |
| `RasterFtPipeline` | Raster with freespace transitions |
| `RasterCtPipeline` | Raster with cartesian transitions |
| `RasterFtOnlyPipeline` | Raster freespace only |
| `RasterCtOnlyPipeline` | Raster cartesian only |

See `task_composer_plugins.yaml` for the full list including global variants.

## Pipeline Input/Output Keys

Different pipelines use different data keys:

```python
# Get keys from task
input_keys = task.getInputKeys()   # TaskComposerKeys
output_keys = task.getOutputKeys()

# Common patterns:
# TrajOptPipeline: input="planning_input", output="program"
# OMPLPipeline: input="program", output="program"

# Check available keys
if input_keys.has("planning_input"):
    data.setData("planning_input", ...)
elif input_keys.has("program"):
    data.setData("program", ...)
```

## AnyPoly Wrapping

TaskComposer uses type-erased `AnyPoly` for data storage:

```python
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
)

# Wrap for storage
wrapped_program = AnyPoly_wrap_CompositeInstruction(program)
wrapped_env = AnyPoly_wrap_EnvironmentConst(env)
wrapped_profiles = AnyPoly_wrap_ProfileDictionary(profiles)

# Unwrap from storage
program = AnyPoly_as_CompositeInstruction(any_poly)
```

## Complete Example

```python
import tesseract_robotics
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_CompositeInstruction, AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst, AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage, TaskComposerPluginFactory,
)

# Setup
factory = TaskComposerPluginFactory(
    tesseract_robotics.get_task_composer_config_path(), GeneralResourceLocator()
)
executor = factory.createTaskComposerExecutor("TaskflowExecutor")
task = factory.createTaskComposerNode("FreespacePipeline")

# Prepare data
data = TaskComposerDataStorage()
data.setData("planning_input", AnyPoly_wrap_CompositeInstruction(program))
data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

# Execute pipeline
future = executor.run(task, data)
future.wait()

# Extract result
context = future.context
if context.isSuccessful():
    output_key = task.getOutputKeys().get("program")
    result = AnyPoly_as_CompositeInstruction(
        context.data_storage.getData(output_key)
    )
    print(f"Planned trajectory with {len(result)} instructions")
else:
    print("Planning failed")
    for info in context.task_infos.getInfoMap().values():
        if info.status_message:
            print(f"  {info.name}: {info.status_message}")
```

## High-Level Alternative

For most users the `tesseract_robotics.planning` module is simpler:

```python
from tesseract_robotics.planning import Robot, TaskComposer, plan_freespace

robot = Robot.from_tesseract_support("abb_irb2400")

# One-liner — picks pipeline defaults per entry point
result = plan_freespace(robot, program)

# Or reuse a composer to amortize plugin loading across calls
composer = TaskComposer.from_config(warmup=True)
result = composer.plan(robot, program, pipeline="TrajOptPipeline")

if result.successful:
    for point in result:
        print(point.positions)
```

## Auto-generated API Reference

::: tesseract_robotics.tesseract_task_composer._tesseract_task_composer
    options:
      show_root_heading: false
      show_source: false
      members_order: source
