"""The freespace scaling problem of the parallel-scaling benchmarks: one robot, N sphere obstacles.

Lives apart from the benchmark file so the correctness tests of the problem run in the normal
suite (`tests/test_freespace_benchmark_problems.py`), which skips `tests/benchmarks`.
"""

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
)
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import (
    JointTarget,
    MotionProgram,
    Pose,
    Robot,
    TaskComposer,
    create_obstacle,
    sphere,
)
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

from ..planning_validity import JointGoal

# Start and end of the freespace motion (rad), the freespace examples' joint states.
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
GOAL = JointGoal.build(JOINT_END)
# Radius of every obstacle sphere (m).
OBSTACLE_RADIUS_M = 0.12
# Penetration depth below which a state counts as colliding (m): any overlap.
COLLISION_DISTANCE_M = 0.0
# OMPL solutions kept when optimizing, and when taking the first solution.
MAX_SOLUTIONS_OPTIMIZE = 10
MAX_SOLUTIONS_FIRST = 1


class FixtureStateInCollisionError(RuntimeError):
    """A benchmark's start or goal state collides, so the planner could never solve it."""


def assert_state_collision_free(robot: Robot, joint_names: list[str], joints: np.ndarray) -> None:
    """Raise when `joints` collides: an obstacle edit fails here, not inside OMPL (#341).

    Raises:
        FixtureStateInCollisionError: names the links and the distance of each contact.
    """
    env = robot.env
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkIds())
    manager.setCollisionObjectsTransform(env.getState(joint_names, joints).link_transforms)
    contacts = ContactResultMap()
    manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
    results = ContactResultVector()
    contacts.flattenMoveResults(results)
    hits = [(list(r.link_ids), r.distance) for r in results if r.distance < COLLISION_DISTANCE_M]
    if hits:
        raise FixtureStateInCollisionError(f"{joints} collides: {hits}")


def setup_robot_with_obstacles(num_obstacles: int = 1):
    """Create robot with configurable obstacle count for difficulty scaling."""
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )

    # Add obstacles - more obstacles = harder problem
    obstacle_positions = [
        (0.5, 0, 0.55),  # center
        (0.4, 0.2, 0.45),  # front-left
        (0.4, -0.2, 0.45),  # front-right
        (0.6, 0.0, 0.65),  # back-upper; 41 mm clear of link_5 at the goal (#341)
        (0.3, 0, 0.35),  # low-center
    ]

    for i in range(min(num_obstacles, len(obstacle_positions))):
        x, y, z = obstacle_positions[i]
        create_obstacle(
            robot,
            name=f"sphere_{i}",
            geometry=sphere(OBSTACLE_RADIUS_M),
            transform=Pose.from_xyz(x, y, z),
        )

    return robot


def freespace_problem(num_obstacles: int = 1):
    """Robot with obstacles and its start-to-end freespace program, built outside the timer."""
    robot = setup_robot_with_obstacles(num_obstacles)
    joint_names = robot.get_joint_names("manipulator")

    assert_state_collision_free(robot, joint_names, JOINT_START)
    assert_state_collision_free(robot, joint_names, JOINT_END)
    robot.set_joints(JOINT_START, joint_names=joint_names)

    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(joint_names)
        .move_to(JointTarget(JOINT_START))
        .move_to(JointTarget(JOINT_END))
    )
    return robot, program


def plan_freespace(
    composer: TaskComposer,
    robot: Robot,
    program: MotionProgram,
    num_planners: int = 1,
    optimize: bool = True,
    planning_time: float = 5.0,
):
    """Plan freespace motion with configurable settings; the caller validates the result."""
    profiles = create_freespace_pipeline_profiles(
        num_planners=num_planners,
        planning_time=planning_time,
        optimize=optimize,
        max_solutions=MAX_SOLUTIONS_OPTIMIZE if optimize else MAX_SOLUTIONS_FIRST,
    )

    return composer.plan(robot, program, pipeline="FreespacePipeline", profiles=profiles)
