"""
tesseract_robotics.planning - Pythonic API for Tesseract Motion Planning

This module provides a high-level, Pythonic interface to the Tesseract motion
planning libraries. It wraps the lower-level nanobind bindings with convenient
factory functions, automatic type conversions, and builder patterns.

Key Features:
- Automatic poly-type wrapping (no more `_wrap_` functions)
- Simple robot/environment loading from URDF/SRDF
- Builder pattern for motion programs
- Simplified task composer workflow
- Pose helpers

Example Usage:
    ```python
    from tesseract_robotics.common import GeneralResourceLocator
    from tesseract_robotics import get_tesseract_resource_path
    from tesseract_robotics.planning import (
        CartesianTarget,
        JointTarget,
        MotionProgram,
        Robot,
        plan_freespace,
    )

    # Load a robot from its URDF/SRDF
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400", locator)

    # Create motion program with builder pattern; quaternions are scalar-last [qx, qy, qz, qw]
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(robot.get_joint_names("manipulator"))
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        .move_to(CartesianTarget.from_xyz_quat([0.8, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0]))
    )

    # Plan and inspect
    result = plan_freespace(robot, program)
    if result.successful:
        print(f"Found trajectory with {len(result.trajectory)} waypoints")
    ```
"""

# The imports below are this package's public names: no `__all__`; each `X as X` is an explicit
# re-export (PEP 484), so type checkers and linters treat it as public.

from tesseract_robotics.planning.composer import (
    TaskComposer as TaskComposer,
)
from tesseract_robotics.planning.composer import (
    TrajectoryPoint as TrajectoryPoint,
)
from tesseract_robotics.planning.core import (
    Robot as Robot,
)
from tesseract_robotics.planning.core import (
    RobotState as RobotState,
)
from tesseract_robotics.planning.errors import (
    EmptyProgramError as EmptyProgramError,
)
from tesseract_robotics.planning.errors import (
    EnvironmentCommandError as EnvironmentCommandError,
)
from tesseract_robotics.planning.errors import (
    IKNoSolutionError as IKNoSolutionError,
)
from tesseract_robotics.planning.errors import (
    UnknownPipelineError as UnknownPipelineError,
)
from tesseract_robotics.planning.geometry import (
    box as box,
)
from tesseract_robotics.planning.geometry import (
    cone as cone,
)
from tesseract_robotics.planning.geometry import (
    convex_mesh_from_file as convex_mesh_from_file,
)
from tesseract_robotics.planning.geometry import (
    create_fixed_joint as create_fixed_joint,
)
from tesseract_robotics.planning.geometry import (
    create_obstacle as create_obstacle,
)
from tesseract_robotics.planning.geometry import (
    cylinder as cylinder,
)
from tesseract_robotics.planning.geometry import (
    mesh_from_file as mesh_from_file,
)
from tesseract_robotics.planning.geometry import (
    meshes_from_file as meshes_from_file,
)
from tesseract_robotics.planning.geometry import (
    sphere as sphere,
)
from tesseract_robotics.planning.planner import (
    PlanningResult as PlanningResult,
)
from tesseract_robotics.planning.planner import (
    assign_current_state_as_seed as assign_current_state_as_seed,
)
from tesseract_robotics.planning.planner import (
    plan_cartesian as plan_cartesian,
)
from tesseract_robotics.planning.planner import (
    plan_freespace as plan_freespace,
)
from tesseract_robotics.planning.planner import (
    plan_ompl as plan_ompl,
)
from tesseract_robotics.planning.profile_types import (
    ProfileTypeMismatchError as ProfileTypeMismatchError,
)
from tesseract_robotics.planning.profiles import (
    STANDARD_PROFILE_NAMES as STANDARD_PROFILE_NAMES,
)
from tesseract_robotics.planning.profiles import (
    create_cartesian_pipeline_profiles as create_cartesian_pipeline_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_descartes_default_profiles as create_descartes_default_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_freespace_ifopt_pipeline_profiles as create_freespace_ifopt_pipeline_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_freespace_pipeline_profiles as create_freespace_pipeline_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_iterative_spline_parameterization as create_iterative_spline_parameterization,
)
from tesseract_robotics.planning.profiles import (
    create_ompl_default_profiles as create_ompl_default_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_ompl_planner_configurators as create_ompl_planner_configurators,
)
from tesseract_robotics.planning.profiles import (
    create_raster_pipeline_profiles as create_raster_pipeline_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_time_optimal_parameterization as create_time_optimal_parameterization,
)
from tesseract_robotics.planning.profiles import (
    create_trajopt_default_profiles as create_trajopt_default_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_trajopt_ifopt_default_profiles as create_trajopt_ifopt_default_profiles,
)
from tesseract_robotics.planning.profiles import (
    create_trajopt_upright_profiles as create_trajopt_upright_profiles,
)
from tesseract_robotics.planning.program import (
    CartesianTarget as CartesianTarget,
)
from tesseract_robotics.planning.program import (
    JointTarget as JointTarget,
)
from tesseract_robotics.planning.program import (
    MotionProgram as MotionProgram,
)
from tesseract_robotics.planning.program import (
    MoveType as MoveType,
)
from tesseract_robotics.planning.program import (
    StateTarget as StateTarget,
)
from tesseract_robotics.planning.transforms import (
    Pose as Pose,
)
from tesseract_robotics.planning.transforms import (
    rotation_from_axis_angle as rotation_from_axis_angle,
)
from tesseract_robotics.planning.transforms import (
    rotation_from_quaternion as rotation_from_quaternion,
)
from tesseract_robotics.planning.transforms import (
    rotation_x as rotation_x,
)
from tesseract_robotics.planning.transforms import (
    rotation_y as rotation_y,
)
from tesseract_robotics.planning.transforms import (
    rotation_z as rotation_z,
)
from tesseract_robotics.planning.transforms import (
    translation as translation,
)
