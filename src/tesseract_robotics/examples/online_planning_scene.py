"""The task of the online planning examples: the robot's run, the person, and the walk it takes.

Both Python examples plan the same task from this module. The robot runs down the cell along
the gantry's x axis, from `START_JOINTS` to `TARGET_JOINTS`, its arm at zero and pointing +y.
A person walks up to the arm from the +y side and back, on a straight line across the robot's
run, without ever crossing it.

tesseract_support's `online_planning_example` scene models the person as `human_estop_zone`, a
cylinder of radius 0.5 m and height 2 m on two prismatic joints, `human_x_joint` and
`human_y_joint`. Its SRDF checks the person only against `robot_estop_zone`, a 2 m radius
cylinder fixed to the gantry flange around the whole arm, so collision depends on the two
gantry axes alone and the arm never takes part in avoiding the person.
`check_arm_against_person` removes that zone and checks the arm's links against the person
instead. `velocity_weights` makes a gantry step cost more than an arm step, so we get more
impressive arm motion.

The figure inside the cylinder is visual only.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Final

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.common import AllowedCollisionMatrix, Isometry3d
from tesseract_robotics.environment import (
    Environment,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
)
from tesseract_robotics.geometry import Box, Capsule, Cylinder, Geometry, Sphere
from tesseract_robotics.planning import EnvironmentCommandError, Pose, Robot
from tesseract_robotics.scene_graph import Joint, JointType, Link, Material, Visual

# The person's link in the scene, and the cylinder link it is fixed to.
PERSON_LINK: Final = "person"
ENVELOPE_LINK: Final = "human_estop_zone"
# The robot's own estop zone: a 2 m radius cylinder on the gantry flange, around the whole arm.
# `check_arm_against_person` removes it.
ROBOT_ZONE_LINK: Final = "robot_estop_zone"
# The arm's links, which the SRDF never checks against the person. The gantry's beams and
# carriage are left unchecked: they run at and above 2 m, the top of the person's cylinder.
ARM_LINKS: Final = ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
# Collision margin both examples keep between the arm and the person [m]: the C++ example's.
COLLISION_MARGIN_M: Final = 0.1

# --- The robot's run ------------------------------------------------------------------------
# The `manipulator` group's joints: the gantry's x and y axes [m], then the arm's six [rad].
# Both ends hold gantry y at GANTRY_Y_M, the flange at world y = 2.42 m, and the arm at zero.
GANTRY_Y_M: Final = 1.5
START_JOINTS: Final = (0.0, GANTRY_Y_M, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
TARGET_JOINTS: Final = (5.5, GANTRY_Y_M, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# The arm joint whose velocity limit `velocity_weights` scales to a weight of 1.
WEIGHT_REFERENCE_JOINT: Final = "joint_1"

# The joints that move the cylinder, along world x and y [m].
HUMAN_JOINTS: Final = ("human_x_joint", "human_y_joint")
# World position of the cylinder at joint zero [m]: the joints' origins in the URDF.
HUMAN_ORIGIN_M: Final = (3.0, 4.0)

# --- The figure -----------------------------------------------------------------------------
# Stature [m]: an adult of average height.
STATURE_M: Final = 1.75
# Heights above the floor and widths as fractions of stature: Winter, D.A., Biomechanics and
# Motor Control of Human Movement, 4th ed., Wiley 2009, Fig. 4.1 (segment lengths after
# Drillis and Contini).
CHIN_HEIGHT_RATIO: Final = 0.870
SHOULDER_HEIGHT_RATIO: Final = 0.818
HIP_HEIGHT_RATIO: Final = 0.530
ANKLE_HEIGHT_RATIO: Final = 0.039
SHOULDER_WIDTH_RATIO: Final = 0.259
HIP_WIDTH_RATIO: Final = 0.191
FOOT_LENGTH_RATIO: Final = 0.152
FOOT_WIDTH_RATIO: Final = 0.055
# Shoulder to fingertip: upper arm 0.186 + forearm 0.146 + hand 0.108 (same figure).
ARM_LENGTH_RATIO: Final = 0.186 + 0.146 + 0.108
# Girths are not stature ratios. Radius = circumference / 2 pi, from rounded adult means
# (ANSUR II, Gordon et al. 2014) [m]: neck 0.37, upper arm 0.31, thigh 0.58 and calf 0.37
# averaged over the leg. Torso depth is the rounded chest depth, 0.25 m.
NECK_RADIUS_M: Final = 0.37 / (2.0 * np.pi)
ARM_RADIUS_M: Final = 0.31 / (2.0 * np.pi)
LEG_RADIUS_M: Final = (0.58 + 0.37) / 2.0 / (2.0 * np.pi)
TORSO_DEPTH_M: Final = 0.25
# Work clothes and skin, RGBA.
CLOTHES_RGBA: Final = (0.15, 0.25, 0.55, 1.0)
SKIN_RGBA: Final = (0.85, 0.66, 0.53, 1.0)
# The URDF's `estop_red`, with the opacity raised so the viewer shows the person's envelope.
ENVELOPE_RGBA: Final = (0.8, 0.0, 0.0, 0.25)

# --- The walk -------------------------------------------------------------------------------
# Back and forth along world y at x = WALK_X_M, halfway down the robot's run, at constant
# speed: from the far end towards the robot and back, one lap a round trip. At the near end the
# person's cylinder reaches past the tip of the arm at zero (world y 3.36 m). The walk spans
# only 0.8 m, so the person stays in the arm's way: the straight start-to-target plan comes
# within the collision margin on 9 of the lap's 12 ticks and runs into the person on 7, up to
# 0.39 m deep, while the start and target stay at least 1.95 m clear, measured. A near end of
# 3.1 m left the task composer's TrajOpt no feasible plan with the person there; 3.3 m keeps
# every plan feasible.
WALK_X_M: Final = 3.0
WALK_NEAR_Y_M: Final = 3.3
WALK_FAR_Y_M: Final = 4.1
# Comfortable walking speed [m/s]: Bohannon, R.W., Age and Ageing 26(1), 1997, 15-19,
# comfortable gait speed of adults 20 to 79 years, 1.27 to 1.46 m/s.
WALKING_SPEED_M_S: Final = 1.3
# One replan per tick [s]: the C++ example's spacing of the planned waypoints in time.
TICK_S: Final = 0.1
LAPS_PER_RUN: Final = 4


def _capsule(radius_m: float, bottom_m: float, top_m: float) -> Capsule:
    """A vertical capsule whose rounded ends sit at `bottom_m` and `top_m`."""
    return Capsule(radius_m, top_m - bottom_m - 2.0 * radius_m)


def _at(x_m: float, y_m: float, z_m: float) -> Isometry3d:
    return Pose.from_xyz(x_m, y_m, z_m)


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    material = Material(name)
    material.color = np.array(rgba, dtype=np.float64)
    return material


def person_parts() -> list[tuple[Geometry, Isometry3d, Material]]:
    """The figure's primitives, posed in the envelope link's frame (floor level, centred).

    Returns:
        Geometry, its origin and its material per part: head, neck, torso, arms, legs, feet.
    """
    h = STATURE_M
    clothes = _material("person_clothes", CLOTHES_RGBA)
    skin = _material("person_skin", SKIN_RGBA)
    head_radius_m = (1.0 - CHIN_HEIGHT_RATIO) * h / 2.0
    chin_m, shoulder_m = CHIN_HEIGHT_RATIO * h, SHOULDER_HEIGHT_RATIO * h
    hip_m, ankle_m = HIP_HEIGHT_RATIO * h, ANKLE_HEIGHT_RATIO * h
    fingertip_m = shoulder_m - ARM_LENGTH_RATIO * h
    arm_y_m = SHOULDER_WIDTH_RATIO * h / 2.0
    leg_y_m = HIP_WIDTH_RATIO * h / 4.0
    parts: list[tuple[Geometry, Isometry3d, Material]] = [
        (Sphere(head_radius_m), _at(0.0, 0.0, h - head_radius_m), skin),
        (
            Cylinder(NECK_RADIUS_M, chin_m - shoulder_m),
            _at(0.0, 0.0, (chin_m + shoulder_m) / 2),
            skin,
        ),
        (
            Box(TORSO_DEPTH_M, HIP_WIDTH_RATIO * h, shoulder_m - hip_m),
            _at(0.0, 0.0, (shoulder_m + hip_m) / 2.0),
            clothes,
        ),
    ]
    for side in (-1.0, 1.0):
        parts += [
            (
                _capsule(ARM_RADIUS_M, fingertip_m, shoulder_m),
                _at(0.0, side * arm_y_m, (fingertip_m + shoulder_m) / 2.0),
                clothes,
            ),
            (
                _capsule(LEG_RADIUS_M, ankle_m, hip_m),
                _at(0.0, side * leg_y_m, (ankle_m + hip_m) / 2.0),
                clothes,
            ),
            (
                Box(FOOT_LENGTH_RATIO * h, FOOT_WIDTH_RATIO * h, ankle_m),
                _at(0.0, side * leg_y_m, ankle_m / 2.0),
                clothes,
            ),
        ]
    return parts


def add_person(robot: Robot) -> None:
    """Fix the figure to the envelope cylinder's link, as visuals without collision geometry.

    Raises:
        EnvironmentCommandError: the scene has no `ENVELOPE_LINK`, or already a `PERSON_LINK`.
    """
    link = Link(PERSON_LINK)
    for geometry, origin, material in person_parts():
        visual = Visual()
        visual.geometry = geometry
        visual.origin = origin
        visual.material = material
        link.addVisual(visual)
    joint = Joint(f"{ENVELOPE_LINK}_to_{PERSON_LINK}")
    joint.parent_link_id = ENVELOPE_LINK
    joint.child_link_id = PERSON_LINK
    joint.type = JointType.FIXED
    robot.add_link(link, joint)


def check_arm_against_person(robot: Robot) -> None:
    """Check the arm's links against the person, in place of the robot's estop zone.

    Removes `ROBOT_ZONE_LINK`, which the SRDF checks against the person and nothing else, and
    re-enables collision between each of `ARM_LINKS` and `ENVELOPE_LINK`.

    Raises:
        EnvironmentCommandError: the scene has no `ROBOT_ZONE_LINK` (a second call fails here,
            before it changes anything), or the environment refused the collision pairs.
    """
    robot.remove_link(ROBOT_ZONE_LINK)
    pairs = AllowedCollisionMatrix()
    for link in ARM_LINKS:
        pairs.addAllowedCollision(link, ENVELOPE_LINK, "")
    if not robot.env.applyCommand(
        ModifyAllowedCollisionsCommand(pairs, ModifyAllowedCollisionsType.REMOVE)
    ):
        raise EnvironmentCommandError(
            f"Environment refused to check {', '.join(ARM_LINKS)} against '{ENVELOPE_LINK}'"
        )


def style_for_viewer(env: Environment) -> None:
    """Colour the person's envelope so the person shows through it."""
    for visual in env.getSceneGraph().getLink(ENVELOPE_LINK).visual:
        visual.material.color = np.array(ENVELOPE_RGBA, dtype=np.float64)


def velocity_weights(robot: Robot) -> NDArray[np.float64]:
    """Velocity cost weight of each `manipulator` joint: the inverse of its velocity limit.

    Scaled so `WEIGHT_REFERENCE_JOINT` weighs 1. The URDF rates the gantry at 0.2 and 0.5 m/s
    and the arm at 2.6 to 7.9 rad/s, so a gantry step costs about 13 and 5 times an arm step of
    the same size: roughly what each takes in time. Both TrajOpt's velocity cost and
    TrajOpt-Ifopt's squared `JointVelConstraint` sum weight times squared step, so the two
    examples weigh a plan alike.

    Returns:
        One weight per joint, in the group's joint order.
    """
    names = robot.get_joint_names("manipulator")
    limits = robot.env.getKinematicGroup("manipulator").getLimits().velocity_limits[:, 1]
    return limits[names.index(WEIGHT_REFERENCE_JOINT)] / limits


def walk_length_m() -> float:
    """One lap of the walk, there and back [m]."""
    return 2.0 * (WALK_FAR_Y_M - WALK_NEAR_Y_M)


def ticks_per_lap() -> int:
    """Ticks of `TICK_S` in one lap at `WALKING_SPEED_M_S`, rounded to whole ticks."""
    return round(walk_length_m() / (WALKING_SPEED_M_S * TICK_S))


def ticks_per_run() -> int:
    """Ticks in `LAPS_PER_RUN` laps: what the examples plan by default and the demos record."""
    return LAPS_PER_RUN * ticks_per_lap()


def walk_position_m(tick: int) -> NDArray[np.float64]:
    """World (x, y) of the person at `tick` [m]; the walk repeats every `ticks_per_lap()`.

    Tick 0 is at the far end; the person reaches the near end, by the robot, halfway round.
    """
    s = (tick % ticks_per_lap()) * walk_length_m() / ticks_per_lap()
    leg = WALK_FAR_Y_M - WALK_NEAR_Y_M
    towards_robot = s <= leg
    y = WALK_FAR_Y_M - s if towards_robot else WALK_NEAR_Y_M + (s - leg)
    return np.array([WALK_X_M, y])


def walk_towards(y_m: float, goal_y_m: float) -> float:
    """The person's y after one tick walking from `y_m` towards `goal_y_m` [m].

    One tick's step at `WALKING_SPEED_M_S` at most: the scripted walk's step, which the SQP
    example's replans are measured to keep the margin against.
    """
    reach_m = WALKING_SPEED_M_S * TICK_S
    return y_m + float(np.clip(goal_y_m - y_m, -reach_m, reach_m))


def human_joints_at(position_m: ArrayLike) -> NDArray[np.float64]:
    """`HUMAN_JOINTS` values that put the person at world (x, y) `position_m` [m]."""
    return np.asarray(position_m, dtype=np.float64) - np.asarray(HUMAN_ORIGIN_M)


def human_joint_values(tick: int) -> NDArray[np.float64]:
    """`HUMAN_JOINTS` values that put the person at `walk_position_m(tick)` [m]."""
    return human_joints_at(walk_position_m(tick))


def place_person(env: Environment, position_m: ArrayLike) -> None:
    """Set the person's joints so it stands at world (x, y) `position_m` [m]."""
    env.setState(dict(zip(HUMAN_JOINTS, human_joints_at(position_m).tolist(), strict=True)))


def move_person(env: Environment, tick: int) -> None:
    """Set the person's joints to where the walk is at `tick`."""
    place_person(env, walk_position_m(tick))


def sweep_waypoint(tick: int, waypoints: int) -> int:
    """The waypoint tick `tick` shows of a plan of `waypoints`: forward, then back, repeating."""
    period = 2 * (waypoints - 1)
    phase = tick % period
    return phase if phase < waypoints else period - phase


def playback_rows(plans: Sequence[ArrayLike], plan_of_tick: Sequence[int]) -> list[list[float]]:
    """Rows of `[q..., human_x, human_y, t]` that show the plans while the person walks.

    The C++ example replays the current plan, its waypoints 0.1 s apart, and replaces it as
    each replan lands. Here tick `k` shows waypoint `w(k)` of plan `plan_of_tick[k]`, where `w`
    runs through the waypoints forward and back, so the robot sweeps along its current plan
    without jumping from the target back to the start.

    Args:
        plans: Joint rows per plan, `(waypoints, joints)` each.
        plan_of_tick: Per tick, the index into `plans` of the plan in force.

    Returns:
        One row per tick: the manipulator joints, the two `HUMAN_JOINTS` and the time [s].
    """
    rows = []
    for tick, index in enumerate(plan_of_tick):
        plan = np.asarray(plans[index], dtype=np.float64)
        waypoint = sweep_waypoint(tick, len(plan))
        rows.append([*plan[waypoint].tolist(), *human_joint_values(tick).tolist(), tick * TICK_S])
    return rows
