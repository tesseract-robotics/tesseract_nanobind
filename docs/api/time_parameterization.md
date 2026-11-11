# tesseract_robotics.time_parameterization

Time parameterization: the algorithms (time-optimal trajectory generation, iterative spline parameterization, constant TCP speed, Ruckig smoothing) and the profiles that configure them in the `TimeParameterization` task composer nodes. `InstructionsTrajectory` is the view of a `CompositeInstruction` the algorithms consume; it raises `StaleTrajectoryError` when the program changed under it. The [Task Composer User Guide](../user-guide/task-composer.md) covers the pipelines that run them.

## Reference

::: tesseract_robotics.time_parameterization
