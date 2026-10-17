# Self-test fixture: a page whose plan targets an unreachable pose

The first block must run; the skip-marked block must not (it would raise);
the third block must fail on its own assert.

```python
from tesseract_robotics.planning import (
    CartesianTarget,
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    plan_freespace,
)

robot = Robot.from_tesseract_support("abb_irb2400")
```

<!-- snippet: skip -->
```python
raise RuntimeError("skip-marked block was executed")
```

```python
import numpy as np

# 10 m out: far outside the IRB2400's ~1.8 m reach.
target = Pose.from_xyz_quat(10.0, 0.0, 0.8, 0.0, -1.0, 0.0, 0.0)
program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget(np.zeros(6)))
    .move_to(CartesianTarget(target))
)
result = plan_freespace(robot, program)
assert result.successful, result.message
```
