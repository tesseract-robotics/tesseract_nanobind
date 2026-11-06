"""
Parallel scaling benchmarks: OMPL planner count vs solve time on one sphere obstacle.

Measured 2026-10-15 on an Apple M1 Max, one held composer (no plugin load in the
timer, #137), FreespacePipeline, 5 rounds, mean:

| planners | optimize=False (planning_time 30 s) | optimize=True (planning_time 2 s) |
|---|---|---|
| 1 | 85 ms | 128 ms |
| 2 | 85 ms | 106 ms |
| 4 | 102 ms | 106 ms |
| 8 | 133 ms | 150 ms |

More planners do not make this problem faster: it is solved within ~100 ms by
one planner, so extra threads only add overhead (8 planners: 1.6x slower in
first-solution mode). Neither mode runs to its planning_time here; why
optimize=True returns well before 2 s is not established.

The table this replaces (1 planner 6.05 s vs 4 planners 3.83 s, "1.58x faster")
timed a fresh TaskComposer per call, i.e. ~4.3 s of plugin load per round.

Run with: pytest tests/benchmarks/test_parallel_scaling.py -v --benchmark-enable
Or use:   ./scripts/run_scaling_benchmarks.sh
"""

import numpy as np
import pytest

pytest.importorskip("pytest_benchmark")

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
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
)


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
    hits = [(list(r.link_ids), r.distance) for r in results if r.distance < 0.0]
    if hits:
        raise FixtureStateInCollisionError(f"{joints} collides: {hits}")


def setup_robot_with_obstacles(num_obstacles: int = 1):
    """Create robot with configurable obstacle count for difficulty scaling."""
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

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
            geometry=sphere(0.12),
            transform=Pose.from_xyz(x, y, z),
        )

    return robot


@pytest.fixture(scope="module")
def composer():
    """One composer for the module, so the plugin load (#137) stays out of the timed call."""
    return TaskComposer.from_config()


def freespace_problem(num_obstacles: int = 1):
    """Robot with obstacles and its start-to-end freespace program, built outside the timer."""
    robot = setup_robot_with_obstacles(num_obstacles)
    joint_names = robot.get_joint_names("manipulator")

    # Start and end positions
    joint_start = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    assert_state_collision_free(robot, joint_names, joint_start)
    assert_state_collision_free(robot, joint_names, joint_end)
    robot.set_joints(joint_start, joint_names=joint_names)

    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(joint_names)
        .move_to(JointTarget(joint_start))
        .move_to(JointTarget(joint_end))
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
    """Plan freespace motion with configurable settings."""
    profiles = create_freespace_pipeline_profiles(
        num_planners=num_planners,
        planning_time=planning_time,
        optimize=optimize,
        max_solutions=1 if not optimize else 10,
    )

    result = composer.plan(robot, program, pipeline="FreespacePipeline", profiles=profiles)
    assert result.successful, f"Planning failed: {result.message}"
    return result


class TestOptimizeModeScaling:
    """Planner count vs solve time with optimize=True (planning_time 2 s)."""

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_optimize_true(self, benchmark, composer, num_planners):
        """optimize=True with `num_planners` parallel planners."""
        robot, program = freespace_problem(1)
        benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=True,
            planning_time=2.0,  # shorter for faster tests
        )
        benchmark.extra_info["mode"] = "optimize=True"
        benchmark.extra_info["num_planners"] = num_planners


class TestFirstSolutionScaling:
    """Planner count vs solve time with optimize=False (first solution wins)."""

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_optimize_false(self, benchmark, composer, num_planners):
        """optimize=False with `num_planners` parallel planners."""
        robot, program = freespace_problem(1)
        benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=False,
            planning_time=30.0,  # high timeout, but exits on first solution
        )
        benchmark.extra_info["mode"] = "optimize=False"
        benchmark.extra_info["num_planners"] = num_planners


def test_collision_check_raises_for_a_colliding_state():
    """#341: the invariant fires, naming the links, on the state the old `sphere_3` overlapped."""
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    create_obstacle(
        robot, name="probe", geometry=sphere(0.12), transform=Pose.from_xyz(0.6, 0.1, 0.65)
    )
    joint_names = robot.get_joint_names("manipulator")
    goal = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    with pytest.raises(FixtureStateInCollisionError, match="probe"):
        assert_state_collision_free(robot, joint_names, goal)


@pytest.mark.parametrize("num_obstacles", [1, 3, 5])
def test_problem_states_are_collision_free(num_obstacles):
    """#341: `freespace_problem` builds only when both states clear every obstacle."""
    robot, _ = freespace_problem(num_obstacles)
    assert robot is not None


class TestHarderProblemScaling:
    """Benchmark with harder problems where parallelism helps more.

    Easy problems (1 obstacle) find solutions in < 100ms - overhead dominates.
    Harder problems (3+ obstacles) need more exploration - parallelism helps.
    """

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    @pytest.mark.parametrize("num_obstacles", [1, 3, 5])
    def test_difficulty_scaling(self, benchmark, composer, num_planners, num_obstacles):
        """Test how problem difficulty affects parallel scaling."""
        robot, program = freespace_problem(num_obstacles)
        benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=False,
            planning_time=30.0,
        )
        benchmark.extra_info["mode"] = "optimize=False"
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["num_obstacles"] = num_obstacles


class TestScalingSummary:
    """Direct comparison of scaling behavior.

    Run these tests together to see the difference:
    pytest -k TestScalingSummary --benchmark-group-by=param:mode
    """

    @pytest.mark.parametrize("num_planners", [1, 4])
    @pytest.mark.parametrize("mode", ["optimize", "first_solution"])
    def test_comparison(self, benchmark, composer, num_planners, mode):
        """Side-by-side comparison of modes."""
        optimize = mode == "optimize"
        robot, program = freespace_problem(3)  # medium difficulty
        benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=optimize,
            planning_time=2.0 if optimize else 30.0,
        )
        benchmark.extra_info["mode"] = mode
        benchmark.extra_info["num_planners"] = num_planners
