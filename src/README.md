# Source Directory Structure

## Python Package
- `tesseract_robotics/` - Pure Python code and high-level API
  - `planning/` - High-level planning API (Robot, Pose, TaskComposer)
  - `__init__.py` - Package initialization, env var setup
  - `selftest.py` - Self-test entry point

## C++ nanobind Bindings
`bindings/<module>.cpp` is the nanobind source of the Python module `tesseract_robotics.<module>`.
It wraps the upstream library `tesseract_<module>` (upstream C++ names keep their prefix):

- `common`, `geometry`, `scene_graph`, `collision`, `environment`, `kinematics`, `state_solver`
- `urdf`, `srdf`, `command_language`, `time_parameterization`
- `motion_planners`, `motion_planners_ompl`, `motion_planners_trajopt`, `motion_planners_descartes`, `motion_planners_simple`, `motion_planners_trajopt_ifopt`
- `task_composer`, `task_composer_planning`, `serialization`
- `trajopt_ifopt`, `trajopt_sqp` - TrajOpt optimization

Files that belong to a module but are not its entry point carry its name as a prefix
(`serialization_planner_cereal_registration.cpp`).

Sixteen modules install directly as `tesseract_robotics/<module>.abi3.so` with a
`tesseract_robotics/<module>.pyi` stub. Six keep a Python package around a compiled `_<module>`
because they hold Python code: `common`, `scene_graph`, `motion_planners`,
`motion_planners_simple`, `motion_planners_descartes`, `motion_planners_trajopt_ifopt`.

## Shared Header
- `tesseract_nb.h` - Common nanobind includes and macros
