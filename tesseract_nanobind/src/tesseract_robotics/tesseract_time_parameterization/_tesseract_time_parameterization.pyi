from typing import Annotated

import numpy
from numpy.typing import NDArray

class TrajectoryContainer:
    def size(self) -> int: ...
    def dof(self) -> int: ...
    def empty(self) -> bool: ...
    def getTimeFromStart(self, i: int) -> float: ...

class InstructionsTrajectory(TrajectoryContainer):
    def __init__(self, program: "tesseract_planning::CompositeInstruction") -> None: ...

class TimeParameterization:
    def compute(
        self,
        trajectory: TrajectoryContainer,
        velocity_limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)],
        acceleration_limits: Annotated[
            NDArray[numpy.float64], dict(shape=(None, 2), writable=False)
        ],
        jerk_limits: Annotated[NDArray[numpy.float64], dict(shape=(None, 2), writable=False)],
        velocity_scaling_factors: Annotated[
            NDArray[numpy.float64], dict(shape=(None,), writable=False)
        ] = ...,
        acceleration_scaling_factors: Annotated[
            NDArray[numpy.float64], dict(shape=(None,), writable=False)
        ] = ...,
        jerk_scaling_factors: Annotated[
            NDArray[numpy.float64], dict(shape=(None,), writable=False)
        ] = ...,
    ) -> bool: ...

class TimeOptimalTrajectoryGeneration(TimeParameterization):
    def __init__(self, path_tolerance: float = 0.1, min_angle_change: float = 0.001) -> None: ...

class IterativeSplineParameterization(TimeParameterization):
    def __init__(self, add_points: bool = True) -> None: ...
