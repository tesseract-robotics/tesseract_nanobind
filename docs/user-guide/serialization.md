# Serialization

<!-- snippet: ambient
from pathlib import Path

from tesseract_robotics.common import AllowedCollisionMatrix
from tesseract_robotics.environment import AddLinkCommand, ModifyAllowedCollisionsCommand, ModifyAllowedCollisionsType
from tesseract_robotics.geometry import Box
from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link
from tests.docs.doc_assets import enter_scratch_dir, joint_program

enter_scratch_dir()
composer = TaskComposer.from_config()
program = joint_program(robot)
obstacle_link = Link("obstacle")
_collision = Collision()
_collision.geometry = Box(0.1, 0.1, 0.1)
obstacle_link.addCollision(_collision)
obstacle_joint = Joint("obstacle_joint")
obstacle_joint.type = JointType.FIXED
obstacle_joint.parent_link_id = "base_link"
obstacle_joint.child_link_id = "obstacle"
acm = AllowedCollisionMatrix()
acm.addAllowedCollision("link_1", "link_2", "adjacent")
-->

Save programs, environments and scene states to XML or binary, and load them back in another
process. Use it to cache a planned trajectory, to ship a scene to a worker, or to reproduce a
failed plan exactly. The archives are tesseract's own (cereal), so a C++ program built on the
same tesseract version reads what Python writes.

## Overview

Three root types are serializable:

| Type | Contents | Use Case |
|------|----------|----------|
| `CompositeInstruction` | Motion program (waypoints, instructions, profiles) | Cache planned trajectories |
| `Environment` | Scene via command history replay | Save/restore full scene state |
| `SceneState` | Joint positions + link transforms | Checkpoint robot state |

Nested types serialize with their root; only the three root types have calls of their own.

## CompositeInstruction

Save and load planned trajectories:

```python
from tesseract_robotics.serialization import (
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
composite_instruction_to_file(result.raw_results, "trajectory.xml")

# Or get XML string (for DB storage, network transfer)
xml = composite_instruction_to_xml(result.raw_results)

# Load next session
program = composite_instruction_from_file("trajectory.xml")

# Binary format
binary = composite_instruction_to_binary(result.raw_results)
program = composite_instruction_from_binary(binary)
```

## Environment

Save and restore complete scene state including all applied commands:

```python
from tesseract_robotics.serialization import (
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

!!! note "Runs without the GIL"
    The `environment_to_*` and `environment_from_*` functions release the GIL:
    serializing or rebuilding a mesh-heavy scene can take seconds, and other
    Python threads keep running meanwhile. See
    [Threading and the GIL](../developer/threading.md).

## SceneState

Checkpoint joint positions and link transforms:

```python
from tesseract_robotics.serialization import (
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

## Format comparison

| Format | Human Readable |
|--------|----------------|
| XML | Yes |
| Binary | No |

Use XML for debugging and interoperability. Binary archives are the compact choice for caching;
measure both on your own data, as their relative speed and size depend on what is archived.
