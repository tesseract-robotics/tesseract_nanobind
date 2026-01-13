# Serialization

Persist motion programs, environments, and robot states to XML or binary format using Boost.Serialization.

## Overview

Three root types are serializable:

| Type | Contents | Use Case |
|------|----------|----------|
| `CompositeInstruction` | Motion program (waypoints, instructions, profiles) | Cache planned trajectories |
| `Environment` | Scene via command history replay | Save/restore full scene state |
| `SceneState` | Joint positions + link transforms | Checkpoint robot state |

Boost.Serialization handles nested types recursively - only root types need explicit serialization calls.

## CompositeInstruction

Save and load planned trajectories:

```python
from tesseract_robotics.tesseract_serialization import (
    composite_instruction_to_xml,
    composite_instruction_from_xml,
    composite_instruction_to_file,
    composite_instruction_from_file,
    composite_instruction_to_binary,
    composite_instruction_from_binary,
)

# After planning
result = composer.plan(robot, program, "TrajOptPipeline")

# Save to file
composite_instruction_to_file(result.program, "trajectory.xml")

# Or get XML string (for DB storage, network transfer)
xml = composite_instruction_to_xml(result.program)

# Load next session
program = composite_instruction_from_file("trajectory.xml")

# Binary format (faster, smaller)
binary = composite_instruction_to_binary(result.program)
program = composite_instruction_from_binary(binary)
```

## Environment

Save and restore complete scene state including all applied commands:

```python
from tesseract_robotics.tesseract_serialization import (
    environment_to_xml,
    environment_from_xml,
    environment_to_file,
    environment_from_file,
)

# After modifying environment
env.applyCommand(AddLinkCommand(obstacle_link, obstacle_joint))
env.applyCommand(ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD))

# Save entire environment
environment_to_file(env, "scene.xml")

# Load in new session - commands replayed automatically
env = environment_from_file("scene.xml")
```

!!! note "Environment returns shared_ptr"
    Due to C++ implementation details, `environment_from_*` functions return a shared pointer. In Python this behaves identically to a regular object.

## SceneState

Checkpoint joint positions and link transforms:

```python
from tesseract_robotics.tesseract_serialization import (
    scene_state_to_xml,
    scene_state_from_xml,
    scene_state_to_file,
    scene_state_from_file,
)

# Get current state
state = env.getState()

# Save state snapshot
scene_state_to_file(state, "state.xml")

# Restore later
state = scene_state_from_file("state.xml")
```

## Format Comparison

| Format | Speed | Size | Human Readable |
|--------|-------|------|----------------|
| XML | Slower | Larger | Yes |
| Binary | ~3-5x faster | ~3-5x smaller | No |

Use XML for debugging and interoperability. Use binary for production caching.
