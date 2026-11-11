"""The online planning person: to scale, inside its envelope, walking a loop the plan can avoid.

The envelope is tesseract_support's `human_estop_zone` cylinder. Its visual spans the floor to
2 m; its collision geometry has the same radius but is centred on the floor, so it spans -1 to
1 m. The figure must fit the visual, and stay within the radius both share.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.examples import online_planning_scene as scene
from tesseract_robotics.geometry import Box, Capsule, Cylinder, Sphere
from tesseract_robotics.planning import Robot

# The cell between the gantry's rails (y = 0 and 6 m, 0.5 m square posts) and inside its outer
# posts (x = -0.5 and 6.7 m) [m]; the person's centre keeps its 0.5 m envelope radius off them.
CELL_X_M = (-0.25 + 0.5, 6.45 - 0.5)
CELL_Y_M = (0.25 + 0.5, 5.75 - 0.5)
# Gantry flange (x, y) at the start and target joints, joint zero at (0.2, 0.92) [m].
FLANGE_START_M = np.array([0.2, 0.92])
FLANGE_TARGET_M = np.array([5.7, 3.92])
# Robot zone radius + person zone radius + the examples' 0.1 m collision margin [m].
ZONE_CLEARANCE_M = 2.0 + 0.5 + 0.1
# Comfortable walking speeds of adults 20 to 79 (Bohannon 1997) [m/s].
COMFORTABLE_SPEED_M_S = (1.27, 1.46)


@pytest.fixture(scope="module")
def robot():
    robot = Robot.from_tesseract_support(
        "online_planning_example", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    scene.add_person(robot)
    return robot


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


def test_the_walk_is_a_closed_loop_at_walking_speed():
    n = scene.ticks_per_lap()
    points = np.array([scene.walk_position_m(k) for k in range(n + 1)])
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)

    assert points[n] == pytest.approx(points[0])
    # Constant speed: equal arc steps; chords across the turns are a little shorter.
    speed = scene.walk_length_m() / (n * scene.TICK_S)
    assert COMFORTABLE_SPEED_M_S[0] <= speed <= COMFORTABLE_SPEED_M_S[1]
    assert steps.max() == pytest.approx(speed * scene.TICK_S)


def test_the_walk_stays_in_the_cell_and_leaves_start_and_target_free():
    points = np.array([scene.walk_position_m(k) for k in range(scene.ticks_per_lap())])

    assert CELL_X_M[0] <= points[:, 0].min() and points[:, 0].max() <= CELL_X_M[1]
    assert CELL_Y_M[0] <= points[:, 1].min() and points[:, 1].max() <= CELL_Y_M[1]
    for flange in (FLANGE_START_M, FLANGE_TARGET_M):
        assert np.linalg.norm(points - flange, axis=1).min() > ZONE_CLEARANCE_M


def test_the_walk_crosses_into_the_straight_path():
    """Part of the lap is closer to the straight start-to-target path than the zones allow."""
    points = np.array([scene.walk_position_m(k) for k in range(scene.ticks_per_lap())])
    direction = (FLANGE_TARGET_M - FLANGE_START_M) / np.linalg.norm(
        FLANGE_TARGET_M - FLANGE_START_M
    )
    offset = points - FLANGE_START_M
    distance = np.abs(offset[:, 0] * direction[1] - offset[:, 1] * direction[0])
    assert distance.min() < ZONE_CLEARANCE_M
    assert distance.max() > ZONE_CLEARANCE_M


def test_playback_sweeps_each_plan_forward_and_back_while_the_person_walks():
    plans = [np.array([[0.0], [1.0], [2.0]]), np.array([[10.0], [11.0], [12.0]])]
    rows = scene.playback_rows(plans, [0, 0, 0, 0, 1, 1])

    assert [row[0] for row in rows] == [0.0, 1.0, 2.0, 1.0, 10.0, 11.0]
    for tick, row in enumerate(rows):
        assert row[1:3] == pytest.approx(scene.human_joint_values(tick).tolist())
        assert row[3] == pytest.approx(tick * scene.TICK_S)
