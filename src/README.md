# Source Directory Structure

## Python Package
- `tesseract_robotics/` - Pure Python code and high-level API
  - `planning/` - High-level planning API (Robot, Pose, TaskComposer)
  - `__init__.py` - Package initialization, env var setup
  - `selftest.py` - Self-test entry point

## C++ nanobind Bindings
Each subdirectory contains C++ binding code that wraps the corresponding tesseract library:

- `tesseract_common/` - Core utilities, types, resource locators
- `tesseract_geometry/` - Geometry types (Box, Sphere, Mesh, etc.)
- `tesseract_scene_graph/` - Scene graph, links, joints
- `tesseract_collision/` - Collision checking (Bullet, FCL)
- `tesseract_environment/` - Environment management
- `tesseract_kinematics/` - Forward/inverse kinematics
- `tesseract_state_solver/` - State solving
- `tesseract_urdf/` - URDF parsing
- `tesseract_srdf/` - SRDF parsing
- `tesseract_command_language/` - Motion planning instructions
- `tesseract_motion_planners*/` - Motion planners (OMPL, TrajOpt, Descartes)
- `tesseract_task_composer/` - Task composition framework
- `tesseract_time_parameterization/` - Time parameterization
- `trajopt_ifopt/`, `trajopt_sqp/` - TrajOpt optimization
- `ifopt/`, `ompl_base/` - Third-party bindings

## Shared Header
- `tesseract_nb.h` - Common nanobind includes and macros
