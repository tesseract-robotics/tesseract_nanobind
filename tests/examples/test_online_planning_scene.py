"""The online planning task: the person to scale inside its envelope, walking up to the arm
and back beside the robot's run, checked against the arm's links rather than the robot's estop
zone, and the velocity weights that make a gantry step cost more than an arm step.

The envelope is tesseract_support's `human_estop_zone` cylinder. Its visual spans the floor to
2 m; its collision geometry has the same radius but is centred on the floor, so it spans -1 to
1 m. The figure must fit the visual, and stay within the radius both share.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
)
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.examples import online_planning_scene as scene
from tesseract_robotics.geometry import Box, Capsule, Cylinder, Sphere
from tesseract_robotics.planning import EnvironmentCommandError, Robot

# The cell between the gantry's rails (y = 0 and 6 m, 0.5 m square posts) and inside its outer
# posts (x = -0.5 and 6.7 m) [m]; the person's centre keeps its 0.5 m envelope radius off them.
CELL_X_M = (-0.25 + 0.5, 6.45 - 0.5)
CELL_Y_M = (0.25 + 0.5, 5.75 - 0.5)
# Waypoints of the straight start-to-target plan: the SQP example's.
STRAIGHT_WAYPOINTS = 12
# Contact distance reported up to this far [m]: past the whole cell, so every link reports.
REPORT_DISTANCE_M = 10.0
# Gantry links the SRDF leaves unchecked against the person: they run above its cylinder.
GANTRY_LINKS = ("gantry_axis_1", "gantry_axis_2")
# The gantry's prismatic joints, in the `manipulator` group.
GANTRY_JOINTS = ("gantry_axis_1", "gantry_axis_2")
# Comfortable walking speeds of adults 20 to 79 (Bohannon 1997) [m/s].
COMFORTABLE_SPEED_M_S = (1.27, 1.46)


@pytest.fixture(scope="module")
def robot():
    robot = Robot.from_tesseract_support(
        "online_planning_example", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    scene.add_person(robot)
    scene.check_arm_against_person(robot)
    return robot


def _arm_clearance_m(robot, joints) -> float:
    """Closest distance between the robot's links and the person's cylinder at `joints` [m]."""
    env = robot.env
    env.setState(dict(zip(robot.get_joint_names("manipulator"), map(float, joints), strict=True)))
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkIds())
    manager.setDefaultCollisionMargin(REPORT_DISTANCE_M)
    manager.setCollisionObjectsTransform(env.getState().link_transforms)
    results = ContactResultMap()
    manager.contactTest(results, ContactRequest(ContactTestType.ALL))
    contacts = ContactResultVector()
    results.flattenMoveResults(contacts)
    return min(
        contact.distance
        for contact in contacts
        if scene.ENVELOPE_LINK in [link.name() for link in contact.link_ids]
    )


def _extent(geometry, origin) -> tuple[float, float, float]:
    """Largest distance from the link's z axis, lowest and highest z of an upright part [m]."""
    x, y, z = np.asarray(origin.translation)
    if isinstance(geometry, Box):
        half = np.array([geometry.getX(), geometry.getY(), geometry.getZ()]) / 2.0
        corner = np.hypot(abs(x) + half[0], abs(y) + half[1])
        return corner, z - half[2], z + half[2]
    if isinstance(geometry, Sphere):
        r = geometry.getRadius()
        return np.hypot(x, y) + r, z - r, z + r
    if isinstance(geometry, Capsule):
        r, half = geometry.getRadius(), geometry.getLength() / 2.0
        return np.hypot(x, y) + r, z - half - r, z + half + r
    if isinstance(geometry, Cylinder):
        r, half = geometry.getRadius(), geometry.getLength() / 2.0
        return np.hypot(x, y) + r, z - half, z + half
    raise TypeError(f"no extent for {type(geometry).__name__}")


def _cylinder(geometries) -> tuple[float, float, float]:
    """Radius, lowest and highest z of the envelope's one cylinder [m]."""
    (part,) = geometries
    assert isinstance(part.geometry, Cylinder)
    z = np.asarray(part.origin.translation)[2]
    half = part.geometry.getLength() / 2.0
    return part.geometry.getRadius(), z - half, z + half


def test_the_person_fits_inside_its_envelope(robot):
    graph = robot.env.getSceneGraph()
    envelope = graph.getLink(scene.ENVELOPE_LINK)
    radius_m, bottom_m, top_m = _cylinder(envelope.visual)
    collision_radius_m, _, _ = _cylinder(envelope.collision)
    person = graph.getLink(scene.PERSON_LINK)
    extents = [_extent(v.geometry, v.origin) for v in person.visual]

    reach_m = max(e[0] for e in extents)
    assert reach_m < min(radius_m, collision_radius_m)
    assert min(e[1] for e in extents) >= bottom_m
    assert max(e[2] for e in extents) <= top_m
    assert max(e[2] for e in extents) == pytest.approx(scene.STATURE_M)


def test_the_person_is_visual_only_and_rides_the_envelope(robot):
    graph = robot.env.getSceneGraph()
    assert len(graph.getLink(scene.PERSON_LINK).collision) == 0
    assert graph.getJoint(
        f"{scene.ENVELOPE_LINK}_to_{scene.PERSON_LINK}"
    ).parent_link_id.name() == (scene.ENVELOPE_LINK)
    scene.move_person(robot.env, 7)
    transforms = robot.env.getState().link_transforms
    person_xy = np.asarray(transforms[scene.PERSON_LINK].translation)[:2]
    assert person_xy == pytest.approx(scene.walk_position_m(7))


def test_the_walk_returns_to_its_start_at_walking_speed():
    n = scene.ticks_per_lap()
    points = np.array([scene.walk_position_m(k) for k in range(n + 1)])
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)

    assert points[n] == pytest.approx(points[0])
    # Constant speed: equal arc steps; chords across the turns are a little shorter.
    speed = scene.walk_length_m() / (n * scene.TICK_S)
    assert COMFORTABLE_SPEED_M_S[0] <= speed <= COMFORTABLE_SPEED_M_S[1]
    assert steps.max() == pytest.approx(speed * scene.TICK_S)


def test_the_arm_not_the_robot_zone_is_checked_against_the_person(robot):
    acm = robot.env.getAllowedCollisionMatrix()
    assert scene.ROBOT_ZONE_LINK not in [link.name() for link in robot.env.getLinkIds()]
    for link in scene.ARM_LINKS:
        assert not acm.isCollisionAllowed((link, scene.ENVELOPE_LINK)), link
    for link in GANTRY_LINKS:
        assert acm.isCollisionAllowed((link, scene.ENVELOPE_LINK)), link


def test_a_second_arm_check_raises_before_changing_anything(robot):
    commands = len(robot.env.getCommandHistory())
    with pytest.raises(EnvironmentCommandError):
        scene.check_arm_against_person(robot)
    assert len(robot.env.getCommandHistory()) == commands


def test_a_run_shows_the_robot_down_the_cell_and_back():
    """A run is whole laps, long enough for a recording to sweep a plan forward and back."""
    assert scene.ticks_per_run() == scene.LAPS_PER_RUN * scene.ticks_per_lap()
    assert scene.ticks_per_run() >= 2 * (STRAIGHT_WAYPOINTS - 1)


def test_the_walk_stays_in_the_cell_and_leaves_start_and_target_free(robot):
    ticks = range(scene.ticks_per_lap())
    points = np.array([scene.walk_position_m(k) for k in ticks])

    assert CELL_X_M[0] <= points[:, 0].min() and points[:, 0].max() <= CELL_X_M[1]
    assert CELL_Y_M[0] <= points[:, 1].min() and points[:, 1].max() <= CELL_Y_M[1]
    for joints in (scene.START_JOINTS, scene.TARGET_JOINTS):
        clearances = []
        for k in ticks:
            scene.move_person(robot.env, k)
            clearances.append(_arm_clearance_m(robot, joints))
        assert min(clearances) > scene.COLLISION_MARGIN_M


def test_the_walk_never_crosses_the_robots_run(robot):
    """The person's cylinder stays on the +y side of the gantry flange's line down the cell."""
    names = robot.get_joint_names("manipulator")
    robot.env.setState(dict(zip(names, scene.START_JOINTS, strict=True)))
    flange_y_m = np.asarray(robot.env.getState().link_transforms["gantry_flange"].translation)[1]
    envelope_radius_m, _, _ = _cylinder(
        robot.env.getSceneGraph().getLink(scene.ENVELOPE_LINK).collision
    )
    nearest_y_m = min(scene.walk_position_m(k)[1] for k in range(scene.ticks_per_lap()))
    assert nearest_y_m - envelope_radius_m > flange_y_m


def test_the_person_walks_into_the_straight_plans_way(robot):
    """Part of the lap the straight plan's arm runs into the person; the rest it stays clear."""
    straight = ti.interpolate(
        np.array(scene.START_JOINTS), np.array(scene.TARGET_JOINTS), STRAIGHT_WAYPOINTS
    )
    clearances = []
    for k in range(scene.ticks_per_lap()):
        scene.move_person(robot.env, k)
        clearances.append(min(_arm_clearance_m(robot, q) for q in straight))
    assert min(clearances) < 0.0
    assert max(clearances) > scene.COLLISION_MARGIN_M


def test_a_gantry_step_costs_more_than_an_arm_step(robot):
    """Weights are the inverse velocity limits, the reference joint at 1."""
    names = robot.get_joint_names("manipulator")
    limits = robot.env.getKinematicGroup("manipulator").getLimits().velocity_limits[:, 1]
    weights = scene.velocity_weights(robot)

    assert weights[names.index(scene.WEIGHT_REFERENCE_JOINT)] == pytest.approx(1.0)
    assert weights * limits == pytest.approx(np.full(len(names), weights[0] * limits[0]))
    gantry = [names.index(name) for name in GANTRY_JOINTS]
    arm = [i for i in range(len(names)) if i not in gantry]
    assert weights[gantry].min() > weights[arm].max()


def test_playback_sweeps_each_plan_forward_and_back_while_the_person_walks():
    plans = [np.array([[0.0], [1.0], [2.0]]), np.array([[10.0], [11.0], [12.0]])]
    rows = scene.playback_rows(plans, [0, 0, 0, 0, 1, 1])

    assert [row[0] for row in rows] == [0.0, 1.0, 2.0, 1.0, 10.0, 11.0]
    for tick, row in enumerate(rows):
        assert row[1:3] == pytest.approx(scene.human_joint_values(tick).tolist())
        assert row[3] == pytest.approx(tick * scene.TICK_S)


def test_a_plan_is_swept_forward_and_back_one_waypoint_a_tick():
    assert [scene.sweep_waypoint(tick, 4) for tick in range(8)] == [0, 1, 2, 3, 2, 1, 0, 1]


def test_the_person_is_placed_where_asked(robot):
    """What the SQP example's slider sets: the person at a world (x, y), off its scripted walk."""
    position_m = (scene.WALK_X_M, 0.5 * (scene.WALK_NEAR_Y_M + scene.WALK_FAR_Y_M))
    scene.place_person(robot.env, position_m)
    transforms = robot.env.getState().link_transforms
    assert np.asarray(transforms[scene.ENVELOPE_LINK].translation)[:2] == pytest.approx(position_m)


def test_the_person_walks_towards_its_goal_one_ticks_step_at_a_time():
    """A slider dragged end to end moves the person at walking speed, as the scripted walk."""
    step_m = scene.WALKING_SPEED_M_S * scene.TICK_S
    far, near = scene.WALK_FAR_Y_M, scene.WALK_NEAR_Y_M
    assert scene.walk_towards(far, near) == pytest.approx(far - step_m)
    assert scene.walk_towards(near, far) == pytest.approx(near + step_m)
    # Within one step it lands on the goal.
    assert scene.walk_towards(far, far - 0.5 * step_m) == pytest.approx(far - 0.5 * step_m)
