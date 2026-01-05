# tesseract_robotics.tesseract_task_composer

Task composition and planning pipelines.

## Overview

TaskComposer orchestrates complex planning tasks by chaining planners,
post-processors, and validators into pipelines.

```python
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory, TaskflowTaskComposerExecutor,
    TaskComposerDataStorage, TaskComposerContext,
    createTaskComposerPluginFactory, createTaskComposerDataStorage,
)
```

## Quick Start

```python
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory, createTaskComposerDataStorage,
    TaskflowTaskComposerExecutor,
)
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_wrap_CompositeInstruction, AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
)

# Create factory and executor
factory = createTaskComposerPluginFactory()
executor = TaskflowTaskComposerExecutor(factory, 4)  # 4 threads

# Get pipeline task
task = factory.createTaskComposerNode("TrajOptPipeline")

# Setup data storage
data = createTaskComposerDataStorage()
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

Loads pipeline definitions from YAML config.

```python
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory, createTaskComposerPluginFactory
)

# Use bundled config (auto-configured)
factory = createTaskComposerPluginFactory()

# Or specify config file
factory = TaskComposerPluginFactory(config_path)

# Get available pipelines
# Common pipelines: TrajOptPipeline, OMPLPipeline, FreespaceMotionPipeline
task = factory.createTaskComposerNode("TrajOptPipeline")
```

### TaskflowTaskComposerExecutor

Executes tasks with parallel support.

```python
from tesseract_robotics.tesseract_task_composer import TaskflowTaskComposerExecutor

executor = TaskflowTaskComposerExecutor(factory, num_threads=4)

future = executor.run(task, data_storage)
future.wait()

# Check success
if future.context.isSuccessful():
    print("Planning succeeded")
```

### TaskComposerDataStorage

Key-value storage for pipeline data.

```python
from tesseract_robotics.tesseract_task_composer import createTaskComposerDataStorage
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_wrap_CompositeInstruction, AnyPoly_wrap_ProfileDictionary,
)

data = createTaskComposerDataStorage()

# Set inputs
data.setData("planning_input", AnyPoly_wrap_CompositeInstruction(program))
data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

# Get outputs
result = data.getData("program")
```

### TaskComposerContext

Execution context with results and status.

```python
context = future.context

# Check status
if context.isSuccessful():
    output = context.data_storage.getData(output_key)
else:
    print("Failed")

# Access task info
info = context.task_infos
```

### TaskComposerFuture

Async execution handle.

```python
future = executor.run(task, data)

# Wait for completion
future.wait()

# Or wait with timeout (milliseconds)
completed = future.waitFor(5000)  # 5 seconds

# Get context
context = future.context
```

## Available Pipelines

| Pipeline | Description |
|----------|-------------|
| `TrajOptPipeline` | TrajOpt optimization |
| `OMPLPipeline` | OMPL sampling-based planning |
| `FreespaceMotionPipeline` | OMPL + TrajOpt + time param |
| `CartesianMotionPipeline` | Cartesian path planning |
| `RasterMotionPipeline` | Industrial raster patterns |

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
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory, createTaskComposerDataStorage,
    TaskflowTaskComposerExecutor, AnyPoly_wrap_EnvironmentConst,
)
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_wrap_CompositeInstruction, AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
)

# Setup
factory = createTaskComposerPluginFactory()
executor = TaskflowTaskComposerExecutor(factory, 4)
task = factory.createTaskComposerNode("FreespaceMotionPipeline")

# Prepare data
data = createTaskComposerDataStorage()
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
    for name, info in context.task_infos.items():
        if info.return_value != 0:
            print(f"  {name}: {info.message}")
```

## High-Level Alternative

For simpler usage, use the `tesseract_robotics.planning` module:

```python
from tesseract_robotics.planning import Robot, plan

robot = Robot.from_tesseract_support("abb_irb2400")
result = plan(robot, program, pipeline="FreespaceMotionPipeline")

if result.successful:
    trajectory = result.trajectory
```
