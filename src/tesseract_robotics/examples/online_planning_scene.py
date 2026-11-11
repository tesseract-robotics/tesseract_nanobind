"""The person of the online planning examples: a figure to scale, and the walk it takes.

tesseract_support's `online_planning_example` scene models a person as `human_estop_zone`, a
cylinder of radius 0.5 m and height 2 m on two prismatic joints, `human_x_joint` and
`human_y_joint`. In the C++ example a person drags those joints with a joint state publisher.
Both Python examples script the same walk instead, from this module: a closed loop at walking
speed beside the robot's path, which pushes the planned path aside on every lap.

The figure inside the cylinder is visual only: the planner still sees the cylinder and nothing
else. It cannot turn, because the scene gives the person no rotational joint.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Final

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.common import Isometry3d
from tesseract_robotics.environment import Environment
from tesseract_robotics.geometry import Box, Capsule, Cylinder, Geometry, Sphere
from tesseract_robotics.planning import Pose, Robot
from tesseract_robotics.scene_graph import Joint, JointType, Link, Material, Visual

# The person's link in the scene, and the cylinder link it is fixed to.
PERSON_LINK: Final = "person"
ENVELOPE_LINK: Final = "human_estop_zone"
# The robot's own estop zone: a 2 m radius cylinder on the gantry flange.
ROBOT_ZONE_LINK: Final = "robot_estop_zone"
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
# The zones' red, as the URDF's `estop_red`, with the opacity raised so the viewer shows the
# person's envelope; the robot's zone stays faint, as it surrounds the whole arm.
ENVELOPE_RGBA: Final = (0.8, 0.0, 0.0, 0.25)
ROBOT_ZONE_RGBA: Final = (0.8, 0.0, 0.0, 0.08)

# --- The walk -------------------------------------------------------------------------------
# A stadium (two straights joined by half circles) at constant speed, counterclockwise from
# the top straight's right end. It stays in the part of the cell the plan can leave free: its
# points keep more than the zones' 2.5 m radius sum plus the 0.1 m collision margin from the
# gantry flange at the start (0.2, 0.92) and at the target (5.7, 3.92), at least 2.93 m and
# 2.79 m. Its lower half comes within 1.42 m of the straight start-to-target line, so the plan
# has to bend away from the person on every lap.
WALK_CENTER_M: Final = (1.65, 4.45)
WALK_STRAIGHT_LENGTH_M: Final = 1.2
WALK_TURN_RADIUS_M: Final = 0.7
# Comfortable walking speed [m/s]: Bohannon, R.W., Age and Ageing 26(1), 1997, 15-19,
# comfortable gait speed of adults 20 to 79 years, 1.27 to 1.46 m/s.
WALKING_SPEED_M_S: Final = 1.3
# One replan per tick [s]: the C++ example's spacing of the planned waypoints in time.
TICK_S: Final = 0.1


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


def style_for_viewer(env: Environment) -> None:
    """Colour the two estop zones so the person shows through its envelope."""
    scene = env.getSceneGraph()
    for link, rgba in ((ENVELOPE_LINK, ENVELOPE_RGBA), (ROBOT_ZONE_LINK, ROBOT_ZONE_RGBA)):
        for visual in scene.getLink(link).visual:
            visual.material.color = np.array(rgba, dtype=np.float64)


def walk_length_m() -> float:
    """One lap of the walk [m]."""
    return 2.0 * WALK_STRAIGHT_LENGTH_M + 2.0 * np.pi * WALK_TURN_RADIUS_M


def ticks_per_lap() -> int:
    """Ticks of `TICK_S` in one lap at `WALKING_SPEED_M_S`, rounded to whole ticks."""
    return round(walk_length_m() / (WALKING_SPEED_M_S * TICK_S))


def walk_position_m(tick: int) -> NDArray[np.float64]:
    """World (x, y) of the person at `tick` [m]; the walk repeats every `ticks_per_lap()`."""
    s = (tick % ticks_per_lap()) * walk_length_m() / ticks_per_lap()
    cx, cy = WALK_CENTER_M
    half, r = WALK_STRAIGHT_LENGTH_M / 2.0, WALK_TURN_RADIUS_M
    turn = np.pi * r
    if s < WALK_STRAIGHT_LENGTH_M:  # top straight, walking towards -x
        return np.array([cx + half - s, cy + r])
    s -= WALK_STRAIGHT_LENGTH_M
    if s < turn:  # left half circle, top to bottom
        angle = np.pi / 2.0 + s / r
        return np.array([cx - half + r * np.cos(angle), cy + r * np.sin(angle)])
    s -= turn
    if s < WALK_STRAIGHT_LENGTH_M:  # bottom straight, walking towards +x, nearest the path
        return np.array([cx - half + s, cy - r])
    s -= WALK_STRAIGHT_LENGTH_M
    angle = -np.pi / 2.0 + s / r  # right half circle, bottom to top
    return np.array([cx + half + r * np.cos(angle), cy + r * np.sin(angle)])


def human_joint_values(tick: int) -> NDArray[np.float64]:
    """`HUMAN_JOINTS` values that put the person at `walk_position_m(tick)` [m]."""
    return walk_position_m(tick) - np.asarray(HUMAN_ORIGIN_M)


def move_person(env: Environment, tick: int) -> None:
    """Set the person's joints to where the walk is at `tick`."""
    env.setState(dict(zip(HUMAN_JOINTS, human_joint_values(tick).tolist(), strict=True)))


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
        period = 2 * (len(plan) - 1)
        phase = tick % period
        waypoint = phase if phase < len(plan) else period - phase
        rows.append([*plan[waypoint].tolist(), *human_joint_values(tick).tolist(), tick * TICK_S])
    return rows
