"""Seven Python-defined constraints in one live loop.

An ABB IRB 2400 rides a rail, with a grinder on its flange. The tool tip rests on a mark on a
workpiece. One TrajOpt problem is solved per frame, a single waypoint warm-started from the
last one, and the rules of ``tesseract_robotics.trajopt_ifopt.python_constraints`` are switched
on one after another:

1. No rule: the rail moves and the tool goes with it.
2. ``WorldHold``: the tip stays on the mark while the rail moves; the tool axis wanders.
3. ``HoldAxis``: the tool axis locks onto vertical, to half a degree.
4. ``WatchTheTool``: a camera on a mast on the carriage has stared ahead so far, losing the
   tip as the rail moved. Now its pan and tilt are the solver's, and it turns to keep the tip
   within 3 degrees of its optical axis. It stays on from here.
5. The rail becomes the solver's seventh axis. ``PointsDown`` gives the tool a 20 degree cone,
   and ``ElbowUp`` raises a plane under the elbow: the carriage drives to where the elbow fits.
6. ``StayDexterous``: manipulability stays above a rising bound.
7. ``Clearance``: a ball drives in, and the forearm keeps its distance while the tip stays put.
8. All seven at once, while the mark moves: the tip traces a circle on the workpiece.

Every bound that moves ramps from the live value, so nothing jumps when a rule comes on. The
viewer shows the cell, a glyph per rule (mark, cone, axis, plane, camera cone, keep-out shell)
and a panel of gauges: each rule's live value on a ruler, with its bound beside it while the
rule is on, and what the frame's solve took against the 30 Hz frame. Captions, gauges and
glyphs are links of the environment, posed by the same trajectory as the robot, so a recording
carries them.

Run it::

    tesseract_python_constraints_example            # the acts play by themselves
    tesseract_python_constraints_example --sliders  # you drive: rail, ball and every rule

With ``--sliders`` the mark has a handle to drag, and handing the rail to the solver switches
on an eighth rule, written here: ``CarriageFollows`` keeps the carriage abeam of the tip.
Either way the viewer's control panel has a slider for the opacity of the rules' glyphs.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import numpy as np
import trimesh

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_geometry import (
    Geometry,
    Mesh,
    MeshMaterial,
    Sphere,
    createMeshFromResource,
)
from tesseract_robotics.tesseract_kinematics import JointGroup, calcManipulability
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointLimits,
    JointType,
    Link,
    Visual,
)
from tesseract_robotics.trajopt_ifopt.python_constraints import (
    Clearance,
    ElbowUp,
    HoldAxis,
    PointsDown,
    PyConstraint,
    StayDexterous,
    WatchTheTool,
    WorldHold,
)

# One solve per frame of the viewer's 30 Hz playback [Hz].
FRAME_HZ = 30.0
# What a solve may take for the loop to keep that rate [ms].
FRAME_MS = 1e3 / FRAME_HZ
# The arm's joints at the start [rad]: the tool points straight down, in front of the robot.
START = np.array([0.0, 0.30, 0.10, 0.0, 1.17, 0.0])
# How far the rail swings while it is driven [m].
RAIL_SWING_M = 0.8
# Merit coefficient of every rule against the stay-close cost.
RULE_COEFF = 50.0
# Where a bound that ramps gets to its target, as a fraction of its act.
RAMP_FRACTION = 0.8
# A rule counts as violated beyond this, in its own unit.
VIOLATION_TOLERANCE = 1e-3
# How long one swing of the rail takes when it swings by itself under `--sliders` [s].
RAIL_PERIOD_S = 6.0
# How solid the rules' translucent glyphs are drawn [-].
GLYPH_OPACITY = 0.2
# Under `--sliders`: how far the tip may lead or trail the carriage once the rail is the
# solver's [m], and the size of the mark's handle [m].
FOLLOW_BAND_M = 0.1
MARK_HANDLE_M = 0.22
# Under `--sliders`: how many frames the solve time shown in the scene is the median of.
READOUT_FRAMES = 8
# Under `--sliders`, a bound that comes on closes in from the live value, so nothing jumps:
# how fast, per second in the quantity's unit, and whether it is a lower bound.
CLOSING = {
    "tilt_deg": (40.0, False),
    "elbow_m": (0.1, True),
    "view_deg": (60.0, False),
    "dexterity": (0.4, True),
    "lead_m": (0.5, False),
}

WORLD = "positioner_base_link"
RAIL_JOINT = "positioner_joint_1"
TCP = "tcp"
ELBOW = "link_3"
CAMERA = "camera"
FOREARM = ("link_3", "link_4")
OBSTACLE_JOINT = "obstacle_joint"
DOWN = np.array([0.0, 0.0, -1.0])

# The grinder of the puzzle-piece cell: its base at z = -0.075 m, its tip at z = +0.106 m.
GRINDER_MESH = "package://tesseract/support/meshes/puzzle_piece/grinder_visual.dae"
GRINDER_BASE_M = 0.075
GRINDER_TIP_M = 0.106
# The mesh is a hand grinder; scaled, it is a spindle in proportion to the IRB 2400.
GRINDER_SCALE = 2.2
ADAPTER_M = 0.03
# The camera: on a pan-tilt head on a mast that rides the carriage, behind the robot's
# shoulder. At zero pan and tilt it looks at the mark from the carriage's start position.
CAMERA_MAST_XY_M = (-0.32, 0.45)
CAMERA_HEIGHT_M = 2.0
CAMERA_JOINTS = ("camera_pan_joint", "camera_joint")
# Half angle of the cone WatchTheTool keeps the tip in [deg].
VIEW_DEG = 3.0
# The ball: its radius, and the line it drives along (parallel to the rail, at forearm height).
BALL_RADIUS_M = 0.12
BALL_LINE_XZ_M = (0.68, 1.05)
BALL_PARKED_M = -2.6
# Clearance: the forearm's distance to the nearest body stays in this window; the contact
# manager reports nothing beyond the margin.
CLEARANCE_MIN_M = 0.15
CLEARANCE_MAX_M = 0.6
CLEARANCE_MARGIN_M = 0.8

# Instrument links are parked this far away while their rule is off [m].
PARKED_M = -1000.0
# Captions and gauges stand on a plane parallel to the rail, behind the robot [m].
BOARD_X_M = -0.85
# Along the rail [m] and up [m]: where the captions start, and the first ruler's left end.
CAPTION_ANCHOR_M = (-1.0, 2.7)
PANEL_CORNER_M = (-2.75, 2.2)
# The view the cell is laid out for (the demo manifest's camera): from the front, a little to
# the side the ball comes from.
VIEW_POSITION_M = np.array([3.9, -2.3, 2.0])
VIEW_LOOK_AT_M = np.array([-0.6, -0.3, 1.25])
VIEW_FOV_RAD = 0.7
RULER_M = 1.6
ROW_PITCH_M = 0.18


@dataclass(frozen=True)
class Finish:
    """A surface as the viewer's PBR shading takes it: colour, metalness and roughness."""

    rgba: tuple[float, float, float, float]
    metallic: float
    roughness: float


TOOL_STEEL = Finish((0.82, 0.84, 0.87, 1.0), 1.0, 0.25)
ANODISED = Finish((0.06, 0.06, 0.07, 1.0), 0.7, 0.4)
RAIL_STEEL = Finish((0.3, 0.31, 0.34, 1.0), 0.9, 0.45)
ALUMINIUM = Finish((0.75, 0.77, 0.8, 1.0), 0.95, 0.35)
CAST_IRON = Finish((0.1, 0.1, 0.11, 1.0), 0.3, 0.65)
POLISHED = Finish((0.9, 0.9, 0.92, 1.0), 1.0, 0.18)
RED_LACQUER = Finish((0.85, 0.08, 0.05, 1.0), 0.0, 0.18)
WHITE_ENAMEL = Finish((1.0, 1.0, 1.0, 1.0), 0.0, 0.3)
ORANGE_ENAMEL = Finish((1.0, 0.55, 0.0, 1.0), 0.0, 0.35)
ORANGE_RGB = (255, 170, 0)
GREY_RGB = (150, 150, 150)
# Captions and panel text: white on nothing, rendered at this many pixels per line.
TEXT_LINE_PX = 72


@dataclass(frozen=True)
class Gauge:
    """One ruler of the panel: a quantity's live value, and its bound while the rule is on."""

    key: str
    label: str
    lo: float
    hi: float

    def offset_m(self, value: float) -> float:
        """Where `value` sits along the ruler [m], clamped to its ends."""
        return RULER_M * float(np.clip((value - self.lo) / (self.hi - self.lo), 0.0, 1.0))


GAUGES = (
    Gauge("hold_mm", "WorldHold  ·  tip off the mark [mm]", 0.0, 800.0),
    Gauge("tilt_deg", "PointsDown / HoldAxis  ·  tool tilt [°]", 0.0, 25.0),
    Gauge("elbow_m", "ElbowUp  ·  elbow height [m]", 1.10, 1.35),
    Gauge("view_deg", "WatchTheTool  ·  tip off the camera axis [°]", 0.0, 25.0),
    Gauge("dexterity", "StayDexterous  ·  manipulability", 0.7, 1.5),
    Gauge("clearance_m", "Clearance  ·  forearm to the nearest body [m]", 0.0, CLEARANCE_MARGIN_M),
    # Not a rule: what the frame's solve took. Its bar is the frame, and it is always there.
    Gauge("solve_ms", "Solve  ·  time per frame [ms], 33 at 30 Hz", 0.0, 40.0),
)
# What switches a rule on, in an `Act` and in an `Inputs` alike.
RULE_SWITCHES = ("hold", "axis_deg", "cone_deg", "elbow_m", "view_deg", "dexterity", "clearance")


# --8<-- [start:acts]
@dataclass(frozen=True)
class Act:
    """One stretch of the loop: which rules are on, and what moves the rail and the ball."""

    caption: str
    seconds: float
    # The rail position over the act's 0..1 [m] while something else drives it; None once the
    # rail is the solver's seventh axis.
    rail: Callable[[float], float] | None = None
    # The ball's closest approach along its line [m]; it drives in from the far side and back.
    ball_m: float | None = None
    hold: bool = True
    axis_deg: float | None = None
    cone_deg: float | None = None
    # Bounds that ramp from the live value to these targets.
    elbow_m: float | None = None
    view_deg: float | None = None
    dexterity: float | None = None
    clearance: bool = False
    # Radius of the circle the mark moves on [m], once round in the act; None: it stays put.
    trace_m: float | None = None


def _swing(cycles: float, phase: float = 0.0) -> Callable[[float], float]:
    return lambda u: RAIL_SWING_M * np.sin(2.0 * np.pi * cycles * u + phase)


ACTS = (
    Act("No rule\nThe rail moves and the tool goes with it", 3.0, rail=_swing(0.5), hold=False),
    Act(
        "WorldHold\nThe tip stays on the mark; the tool axis wanders",
        4.5,
        rail=_swing(0.75, np.pi),
    ),
    Act(
        "+ HoldAxis 0.5°\nThe tool locks vertical",
        4.0,
        rail=_swing(0.5, 0.5 * np.pi),
        axis_deg=0.5,
    ),
    Act(
        "+ WatchTheTool 3°\nThe camera on the mast turns to keep the tip in its cone",
        5.0,
        rail=_swing(1.0, -0.5 * np.pi),
        axis_deg=0.5,
        view_deg=VIEW_DEG,
    ),
    Act(
        "PointsDown 20° + ElbowUp, and the rail is now the solver's\n"
        "The carriage drives to where the elbow clears the plane",
        4.0,
        cone_deg=20.0,
        view_deg=VIEW_DEG,
        elbow_m=1.25,
    ),
    Act(
        "+ StayDexterous\nManipulability stays above its bound",
        4.0,
        cone_deg=20.0,
        view_deg=VIEW_DEG,
        dexterity=1.25,
    ),
    Act(
        "+ Clearance\nThe forearm keeps 15 cm from the ball",
        6.0,
        cone_deg=20.0,
        view_deg=VIEW_DEG,
        ball_m=-0.10,
        clearance=True,
    ),
    Act(
        "All seven at once\nThe mark moves, and the tip traces the circle",
        7.0,
        ball_m=-0.45,
        axis_deg=20.0,
        cone_deg=20.0,
        elbow_m=1.22,
        view_deg=VIEW_DEG,
        dexterity=0.8,
        clearance=True,
        trace_m=0.12,
    ),
)
# --8<-- [end:acts]


@dataclass
class ActResult:
    """What one act recorded: a row of joint values per frame, and how the solver did."""

    act: Act
    # Per frame: every recorded joint, then the time since the act began [s].
    rows: list[list[float]] = field(default_factory=list)
    # Per frame: the quantity each gauge shows, by `Gauge.key`.
    live: list[dict[str, float]] = field(default_factory=list)
    solve_ms: list[float] = field(default_factory=list)
    # Largest bound violation of any active rule, in the rule's own unit.
    worst_violation: float = 0.0


def rule_count(switches) -> int:
    """How many of the seven rules an `Act` or an `Inputs` has on."""
    values = (getattr(switches, name) for name in RULE_SWITCHES)
    return sum(value is not None and value is not False for value in values)


def solve_line(rules: int, solve_ms: float, *, median: bool = False) -> str:
    """What a caption says of the solve: how many rules, and what a frame took."""
    took = f"median {solve_ms:.1f}" if median else f"{solve_ms:.1f}"
    return f"{rules} rule{'' if rules == 1 else 's'}  ·  {took} ms a frame"


def _isometry(xyz=(0.0, 0.0, 0.0), rotation: np.ndarray | None = None) -> Isometry3d:
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    if rotation is not None:
        matrix[:3, :3] = rotation
    return Isometry3d(matrix)


def _finished(vertices: np.ndarray, triangles: np.ndarray, finish: Finish) -> Mesh:
    """A tesseract mesh that carries its own PBR material, which the viewer shades with."""
    faces = np.column_stack([np.full(len(triangles), 3), triangles]).astype(np.int32).ravel()
    material = MeshMaterial(np.array(finish.rgba), finish.metallic, finish.roughness, np.zeros(4))
    return Mesh(np.asarray(vertices, dtype=np.float64), faces, mesh_material=material)


def _box(x: float, y: float, z: float, finish: Finish) -> Mesh:
    shape = trimesh.creation.box((x, y, z))
    return _finished(shape.vertices, shape.faces, finish)


def _cylinder(radius: float, height: float, finish: Finish) -> Mesh:
    shape = trimesh.creation.cylinder(radius, height, sections=72)
    return _finished(shape.vertices, shape.faces, finish)


def _ball(radius: float, finish: Finish) -> Mesh:
    shape = trimesh.creation.icosphere(subdivisions=4, radius=radius)
    return _finished(shape.vertices, shape.faces, finish)


def _visual(geometry: Geometry, origin: Isometry3d | None = None) -> Visual:
    # No visual material: the viewer then shades with the mesh's own.
    visual = Visual()
    visual.origin = origin or Isometry3d.Identity()
    visual.geometry = geometry
    return visual


def _add_link(
    robot: Robot,
    name: str,
    parent: str,
    origin: Isometry3d,
    visuals: tuple[Visual, ...] = (),
    collision: Geometry | None = None,
    axis: tuple[float, float, float] | np.ndarray | None = None,
    revolute: bool = False,
) -> None:
    """Add `name` under `parent`: fixed, or on a joint `<name>_joint` along or about `axis`."""
    link = Link(name)
    for visual in visuals:
        link.addVisual(visual)
    if collision is not None:
        shape = Collision()
        shape.origin = Isometry3d.Identity()
        shape.geometry = collision
        link.addCollision(shape)
    joint = Joint(f"{name}_joint")
    joint.parent_link_id = parent
    joint.child_link_id = name
    joint.parent_to_joint_origin_transform = origin
    if axis is None:
        joint.type = JointType.FIXED
    elif revolute:
        joint.type = JointType.REVOLUTE
        joint.axis = np.asarray(axis, dtype=np.float64)
        joint.limits = JointLimits(-np.pi, np.pi, 0.0, 10.0, 10.0, 10.0)
    else:
        joint.type = JointType.PRISMATIC
        joint.axis = np.asarray(axis, dtype=np.float64)
        joint.limits = JointLimits(-2000.0, 2000.0, 0.0, 10.0, 10.0, 10.0)
    if not robot.add_link(link, joint):
        raise RuntimeError(f"could not add link {name!r}")


def panel_frame() -> tuple[np.ndarray, np.ndarray]:
    """The first ruler's left end [m] and the unit axis the rulers and the text run along."""
    along, up = PANEL_CORNER_M
    return np.array([BOARD_X_M, along, up]), np.array([0.0, 1.0, 0.0])


def caption_anchor() -> np.ndarray:
    """Where the captions start [m]: their top left corner."""
    along, up = CAPTION_ANCHOR_M
    return np.array([BOARD_X_M, along, up])


# --8<-- [start:cell]
def build_cell() -> Robot:
    """The IRB 2400 on its rail, with a tool, a workpiece, a forearm camera and a ball."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("abb_irb2400_on_positioner", locator)

    # The tool: an adapter plate, then the grinder with its base on the plate and its axis
    # along the flange normal. `tcp` is its tip.
    grinder = createMeshFromResource(
        locator.locateResource(GRINDER_MESH), np.full(3, GRINDER_SCALE), True, True
    )
    _add_link(
        robot,
        "adapter",
        "tool0",
        _isometry((0, 0, ADAPTER_M / 2)),
        (_visual(_cylinder(0.07, ADAPTER_M, ANODISED)),),
    )
    _add_link(
        robot,
        "grinder",
        "tool0",
        _isometry((0, 0, ADAPTER_M + GRINDER_BASE_M * GRINDER_SCALE)),
        tuple(
            _visual(
                _finished(
                    np.asarray(part.getVertices()),
                    np.asarray(part.getFaces()).reshape(-1, 4)[:, 1:],
                    TOOL_STEEL,
                )
            )
            for part in grinder
        ),
    )
    tool_length = ADAPTER_M + (GRINDER_BASE_M + GRINDER_TIP_M) * GRINDER_SCALE
    _add_link(robot, TCP, "tool0", _isometry((0, 0, tool_length)))

    # The rail and the carriage the robot stands on.
    _add_link(
        robot,
        "rail",
        WORLD,
        _isometry(),
        (
            _visual(_box(0.08, 3.4, 0.06, RAIL_STEEL), _isometry((0.33, 0, 0.03))),
            _visual(_box(0.08, 3.4, 0.06, RAIL_STEEL), _isometry((-0.33, 0, 0.03))),
        ),
    )
    _add_link(
        robot,
        "carriage",
        "positioner_tool0",
        _isometry(),
        (_visual(_box(0.86, 0.7, 0.04, ALUMINIUM), _isometry((0, 0, 0.02))),),
    )

    # The workpiece: a plate on a pedestal, its top where the tool tip is at the start.
    tip = robot.env.getJointGroup("manipulator").calcFwdKin(START)[TCP].translation
    top = float(tip[2])
    _add_link(
        robot,
        "workpiece",
        WORLD,
        _isometry((tip[0], tip[1], 0)),
        (
            _visual(_box(0.36, 0.36, top - 0.03, CAST_IRON), _isometry((0, 0, (top - 0.03) / 2))),
            _visual(_cylinder(0.26, 0.03, POLISHED), _isometry((0, 0, top - 0.015))),
        ),
    )

    # The camera: a mast on the carriage, a pan joint about the vertical, a tilt joint about
    # the horizontal. The pan frame's x axis heads for the mark and the tilt frame's x axis
    # points at it, so at zero pan and tilt the camera looks at the mark. `camera` is the
    # optical frame: z forward, x right, y down.
    mast_x, mast_y = CAMERA_MAST_XY_M
    to_mark = tip - np.array([mast_x, mast_y, CAMERA_HEIGHT_M])
    heading = np.arctan2(to_mark[1], to_mark[0])
    pitch = np.arctan2(-to_mark[2], np.hypot(to_mark[0], to_mark[1]))
    yaw = np.array(
        [[np.cos(heading), -np.sin(heading), 0], [np.sin(heading), np.cos(heading), 0], [0, 0, 1]]
    )
    nod = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
    )
    optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    _add_link(
        robot,
        "camera_mast",
        "positioner_tool0",
        _isometry((mast_x, mast_y, 0)),
        (
            _visual(
                _cylinder(0.025, CAMERA_HEIGHT_M - 0.06, RAIL_STEEL),
                _isometry((0, 0, (CAMERA_HEIGHT_M - 0.06) / 2)),
            ),
        ),
    )
    _add_link(
        robot,
        "camera_pan",
        "camera_mast",
        _isometry((0, 0, CAMERA_HEIGHT_M), yaw),
        (_visual(_cylinder(0.045, 0.06, ANODISED), _isometry((0, 0, -0.03))),),
        axis=(0, 0, 1),
        revolute=True,
    )
    _add_link(
        robot,
        CAMERA,
        "camera_pan",
        _isometry(rotation=nod @ optical),
        (
            _visual(_box(0.1, 0.1, 0.16, ANODISED)),
            _visual(_cylinder(0.035, 0.05, TOOL_STEEL), _isometry((0, 0, 0.105))),
        ),
        # The tilt axis, the pan frame's y, in the optical frame: its -x.
        axis=(-1, 0, 0),
        revolute=True,
    )

    # The ball, on a slide parallel to the rail. The only body with collision geometry
    # besides the robot, so it is what `Clearance` keeps the forearm away from.
    _add_link(
        robot,
        "obstacle",
        WORLD,
        _isometry((BALL_LINE_XZ_M[0], 0, BALL_LINE_XZ_M[1])),
        (_visual(_ball(BALL_RADIUS_M, RED_LACQUER)),),
        collision=Sphere(BALL_RADIUS_M),
        axis=(0, 1, 0),
    )
    _add_instruments(robot)
    return robot


# --8<-- [end:cell]


def _add_instruments(robot: Robot) -> None:
    """Links that carry the captions, the gauges' markers and the glyphs of the rules.

    Each is on a prismatic joint, so the trajectory that moves the robot also shows, moves and
    parks them: that is all a recording can animate.
    """
    corner, right = panel_frame()
    for row, gauge in enumerate(GAUGES):
        start = corner - np.array([0.0, 0.0, row * ROW_PITCH_M])
        _add_link(
            robot,
            f"{gauge.key}_value",
            WORLD,
            _isometry(start),
            (_visual(_ball(0.024, WHITE_ENAMEL)),),
            axis=right,
        )
        _add_link(
            robot,
            f"{gauge.key}_bound",
            WORLD,
            _isometry(start),
            (_visual(_box(0.014, 0.014, 0.085, ORANGE_ENAMEL)),),
            axis=right,
        )
    for index in range(len(ACTS)):
        _add_link(robot, f"caption_{index}", WORLD, _isometry(), axis=(0, 0, 1))
    tip = robot.env.getJointGroup("manipulator").calcFwdKin(START)[TCP].translation
    # The mark, on three slides: the last act and the handle of `--sliders` move it.
    _add_link(robot, "mark_x", WORLD, _isometry(tip), axis=(1, 0, 0))
    _add_link(robot, "mark_y", "mark_x", _isometry(), axis=(0, 1, 0))
    _add_link(
        robot,
        "mark",
        "mark_y",
        _isometry(),
        (_visual(_ball(0.016, ORANGE_ENAMEL)),),
        axis=(0, 0, 1),
    )
    # Shown at the mark, and moving with it: the PointsDown cone and the HoldAxis line.
    _add_link(robot, "cone_stage", "mark", _isometry(), axis=(0, 0, 1))
    _add_link(robot, "axis_stage", "mark", _isometry(), axis=(0, 0, 1))
    # The circle the mark moves on in the last act.
    _add_link(robot, "trace_stage", WORLD, _isometry(tip), axis=(0, 0, 1))
    # The ElbowUp plane: its joint value is the bound [m].
    _add_link(robot, "plane_stage", WORLD, _isometry(), axis=(0, 0, 1))
    # The camera's cone of view, parked far behind the camera while WatchTheTool is off.
    _add_link(robot, "view_stage", CAMERA, _isometry(), axis=(0, 0, 1))


INSTRUMENT_JOINTS = (
    *(f"{gauge.key}_value_joint" for gauge in GAUGES),
    *(f"{gauge.key}_bound_joint" for gauge in GAUGES),
    *(f"caption_{index}_joint" for index in range(len(ACTS))),
    "mark_x_joint",
    "mark_y_joint",
    "mark_joint",
    "cone_stage_joint",
    "axis_stage_joint",
    "trace_stage_joint",
    "plane_stage_joint",
    "view_stage_joint",
)


def _angle_deg(cosine: float) -> float:
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def measure(cell: JointGroup, q: np.ndarray, target: np.ndarray, clearance_m: float) -> dict:
    """The quantity each rule bounds, at the rail-and-arm joints `q`."""
    poses = cell.calcFwdKin(q)
    tcp, camera = poses[TCP], poses[CAMERA]
    to_tip = tcp.translation - camera.translation
    return {
        "hold_mm": float(np.linalg.norm(tcp.translation - target) * 1e3),
        "tilt_deg": _angle_deg(tcp.rotation[:, 2] @ DOWN),
        "elbow_m": float(poses[ELBOW].translation[2]),
        "view_deg": _angle_deg(camera.rotation[:, 2] @ to_tip / np.linalg.norm(to_tip)),
        "dexterity": float(calcManipulability(cell.calcJacobian(q, TCP)).m.volume),
        "clearance_m": clearance_m,
        # How far the tip leads the carriage along the rail [m]; no gauge shows it.
        "lead_m": float(tcp.translation[1] - q[0]),
    }


def _ramped(target: float | None, live: float, u: float, *, lower_bound: bool) -> float | None:
    """A bound on its way from the value the act began at to `target`, if it has to move."""
    if target is None:
        return None
    satisfied = live >= target if lower_bound else live <= target
    if satisfied:
        return target
    s = min(u / RAMP_FRACTION, 1.0)
    return live + (target - live) * s * s * (3.0 - 2.0 * s)


def _closing(key: str, bound: float, live: dict[str, float]) -> float:
    """`bound` on its way in from the live value of `key`: one frame's step past it at most."""
    per_s, lower_bound = CLOSING[key]
    step = per_s / FRAME_HZ
    if lower_bound:
        return min(bound, live[key] + step)
    return max(bound, abs(live[key]) - step)


@dataclass(frozen=True)
class Inputs:
    """What one frame is solved for: what is measured, and which rules are on at what bound."""

    # The measured rail position [m] while something else drives it; None once the rail is
    # the solver's seventh axis.
    rail_m: float | None
    ball_m: float
    mark: np.ndarray
    hold: bool = True
    axis_deg: float | None = None
    cone_deg: float | None = None
    elbow_m: float | None = None
    view_deg: float | None = None
    dexterity: float | None = None
    clearance: bool = False
    # How far the tip may lead or trail the carriage along the rail [m], once the rail is the
    # solver's; None for no such rule.
    follow_m: float | None = None


@dataclass(frozen=True)
class Frame:
    """One solved frame: the rail and arm joints, the gauges' quantities, and how it went."""

    q: np.ndarray
    live: dict[str, float]
    solve_ms: float
    # Largest bound violation of any active rule, in the rule's own unit.
    violation: float


# --8<-- [start:own_rule]
class CarriageFollows(PyConstraint):
    """A rule of this example's own: the carriage stays abeam of the tip, within `band` [m].

    Along the rail, the tip may lead or trail the carriage by `band` at most: the carriage does
    the travelling and the arm the reaching. The rail is the first joint of `q`, and runs along
    the world's y.
    """

    def __init__(self, var: ti.Var, cell: JointGroup, tcp: str, band: float, coeff: float = 1.0):
        super().__init__("carriage_follows", var, [-band], [band], coeff=coeff)
        self._cell, self._tcp = cell, tcp

    def f(self, q: np.ndarray) -> np.ndarray:
        return np.array([self._cell.calcFwdKin(q)[self._tcp].translation[1] - q[0]])


# --8<-- [end:own_rule]


# --8<-- [start:rules]
def rules(
    inputs: Inputs, var: ti.Var, group: JointGroup, robot: Robot, base_pose: Isometry3d
) -> list[PyConstraint]:
    """The constraint sets of one frame: the rules `inputs` has on, at its bounds."""
    sets: list[PyConstraint] = []
    if inputs.hold:
        sets.append(WorldHold(var, group, TCP, inputs.mark, base_pose, tol=0.0, coeff=RULE_COEFF))
    if inputs.axis_deg is not None:
        sets.append(HoldAxis(var, group, TCP, DOWN, np.radians(inputs.axis_deg), coeff=RULE_COEFF))
    if inputs.cone_deg is not None:
        sets.append(PointsDown(var, group, TCP, np.radians(inputs.cone_deg), coeff=RULE_COEFF))
    if inputs.elbow_m is not None:
        sets.append(ElbowUp(var, group, ELBOW, WORLD, inputs.elbow_m, coeff=RULE_COEFF))
    if inputs.view_deg is not None:
        sets.append(
            WatchTheTool(var, group, CAMERA, TCP, np.radians(inputs.view_deg), coeff=RULE_COEFF)
        )
    if inputs.dexterity is not None:
        sets.append(StayDexterous(var, group, TCP, inputs.dexterity, coeff=RULE_COEFF))
    if inputs.clearance:
        sets.append(
            Clearance(
                var,
                robot.env,
                [joint_id.name() for joint_id in group.getJointIds()],
                FOREARM,
                CLEARANCE_MIN_M,
                CLEARANCE_MAX_M,
                margin=CLEARANCE_MARGIN_M,
                coeff=RULE_COEFF,
            )
        )
    if inputs.follow_m is not None and inputs.rail_m is None:
        sets.append(CarriageFollows(var, group, TCP, inputs.follow_m, coeff=RULE_COEFF))
    return sets


# --8<-- [end:rules]


def _violation(sets: list[PyConstraint]) -> float:
    worst = 0.0
    for rule in sets:
        for value, bound in zip(rule.getValues(), rule.getBounds()):
            worst = max(worst, bound.getLower() - value, value - bound.getUpper())
    return worst


# --8<-- [start:loop]
class LiveLoop:
    """The cell and its state between frames: `step` solves one frame from the last."""

    def __init__(self) -> None:
        self.robot = build_cell()
        env = self.robot.env
        arm_joints = [joint_id.name() for joint_id in env.getGroupJointIds("manipulator")]
        # A joint group keeps the other joints where they are when it is made. With the rail
        # at zero, the first group works in the carriage's frame: the rail is someone else's.
        env.setState({RAIL_JOINT: 0.0, OBSTACLE_JOINT: BALL_PARKED_M})
        self._arm_names = [*arm_joints, *CAMERA_JOINTS]
        self._arm = env.getJointGroup("arm_and_camera", self._arm_names)
        # The second has the rail too: the solver's seventh axis.
        self._cell_names = [RAIL_JOINT, *self._arm_names]
        self._cell = env.getJointGroup("rail_arm_and_camera", self._cell_names)
        self._arm_limits = ti.toBounds(self._arm.getLimits().joint_limits)
        self._cell_limits = ti.toBounds(self._cell.getLimits().joint_limits)
        # Where the tip rests at the start [m]: the mark, until an act moves it.
        start = np.array([*START, 0.0, 0.0])
        self.target = np.array(self._arm.calcFwdKin(start)[TCP].translation)
        self.joint_names = [*self._cell_names, OBSTACLE_JOINT, *INSTRUMENT_JOINTS]

        # Measures the forearm's clearance for the gauge, whether or not the rule is on.
        self._probe_nodes = ti.createNodesVariables(
            "probe", self._cell_names, [np.zeros(len(self._cell_names))], self._cell_limits
        )
        self._probe = Clearance(
            self._probe_nodes.getNodes()[0].getVar("joints"),
            env,
            self._cell_names,
            FOREARM,
            0.0,
            CLEARANCE_MAX_M,
            margin=CLEARANCE_MARGIN_M,
        )
        self.q = np.array([0.0, *start])  # the rail, the arm, the camera's pan and tilt
        self.live = self._measure(self.target, 0.0)

    def _measure(self, mark: np.ndarray, solve_ms: float) -> dict[str, float]:
        measured = measure(self._cell, self.q, mark, float(self._probe.f(self.q)[0]))
        return {**measured, "solve_ms": solve_ms}

    def step(self, inputs: Inputs) -> Frame:
        """Solve one frame: a single waypoint, warm-started from the frame before."""
        env = self.robot.env
        if inputs.rail_m is not None:
            # The rail is measured, not commanded: the arm solves in its own base frame.
            env.setState({RAIL_JOINT: inputs.rail_m, OBSTACLE_JOINT: inputs.ball_m})
            group, names, limits = self._arm, self._arm_names, self._arm_limits
            seed = self.q[1:]
            base_pose = _isometry((0.0, inputs.rail_m, 0.0))
        else:
            env.setState({OBSTACLE_JOINT: inputs.ball_m})
            group, names, limits = self._cell, self._cell_names, self._cell_limits
            seed = self.q
            base_pose = Isometry3d.Identity()

        nodes = ti.createNodesVariables("frame", names, [seed], limits)
        var = nodes.getNodes()[0].getVar("joints")
        problem = tsqp.TrajOptQPProblem(nodes)
        problem.addCostSet(
            ti.JointPosConstraint(seed, var, np.ones(len(seed)), "stay_close"),
            tsqp.CostPenaltyType.SQUARED,
        )
        sets = rules(inputs, var, group, self.robot, base_pose)
        for rule in sets:
            problem.addConstraintSet(rule)
        problem.setup()
        started = time.perf_counter()
        if sets:
            tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver()).solve(problem)
        solve_ms = (time.perf_counter() - started) * 1e3

        solved = np.asarray(var.value())
        self.q = solved.copy() if inputs.rail_m is None else np.array([inputs.rail_m, *solved])
        self.live = self._measure(inputs.mark, solve_ms)
        return Frame(self.q, self.live, solve_ms, _violation(sets))


def run(acts: tuple[Act, ...] = ACTS) -> dict:
    """Play the acts: one solve per frame.

    Returns:
        `robot`, the recorded `joint_names`, an `ActResult` per act in `acts`, the `target` the
        tip holds [m], and `success`: no rule was violated by more than 1e-3 of its unit.
    """
    loop = LiveLoop()
    results = []
    for index, act in enumerate(acts):
        result = ActResult(act)
        began = dict(loop.live)
        frames = int(round(act.seconds * FRAME_HZ))
        for frame in range(frames):
            u = frame / (frames - 1)
            ball = BALL_PARKED_M
            if act.ball_m is not None:
                ball += (act.ball_m - BALL_PARKED_M) * np.sin(np.pi * u) ** 2
            mark = loop.target
            if act.trace_m is not None:
                turn = 2.0 * np.pi * u
                mark = mark + act.trace_m * np.array([np.cos(turn) - 1.0, np.sin(turn), 0.0])
            inputs = Inputs(
                rail_m=None if act.rail is None else float(act.rail(u)),
                ball_m=float(ball),
                mark=mark,
                hold=act.hold,
                axis_deg=act.axis_deg,
                cone_deg=act.cone_deg,
                elbow_m=_ramped(act.elbow_m, began["elbow_m"], u, lower_bound=True),
                view_deg=_ramped(act.view_deg, began["view_deg"], u, lower_bound=False),
                dexterity=_ramped(act.dexterity, began["dexterity"], u, lower_bound=True),
                clearance=act.clearance,
            )
            solved = loop.step(inputs)
            result.solve_ms.append(solved.solve_ms)
            result.live.append(solved.live)
            result.worst_violation = max(result.worst_violation, solved.violation)
            result.rows.append(
                [
                    *solved.q,
                    inputs.ball_m,
                    *instrument_values(
                        inputs, solved.live, loop.target, caption=index, tracing=act.trace_m
                    ),
                    frame / FRAME_HZ,
                ]
            )
        results.append(result)
    return {
        "robot": loop.robot,
        "joint_names": loop.joint_names,
        "acts": results,
        "target": loop.target,
        "success": all(result.worst_violation < VIOLATION_TOLERANCE for result in results),
    }


# --8<-- [end:loop]


def act_solve_ms(results: dict) -> list[float]:
    """The median solve of each act's frames [ms], for its caption."""
    return [float(np.median(result.solve_ms)) for result in results["acts"]]


def instrument_values(
    inputs: Inputs,
    live: dict[str, float],
    home: np.ndarray,
    *,
    caption: int | None,
    tracing: float | None = None,
) -> list[float]:
    """This frame's joint values of INSTRUMENT_JOINTS.

    Args:
        inputs: The frame's rules and bounds: a bound's marker and glyph show while it is on.
        live: The gauges' quantities.
        home: Where the mark started [m]: its slides carry it from there to `inputs.mark`.
        caption: The act whose caption shows; None for none.
        tracing: Whether the mark is on its circle: the circle shows in place of the cone.
    """
    tilt_bounds = [b for b in (inputs.axis_deg, inputs.cone_deg) if b is not None]
    shown = {
        "hold_mm": 0.0 if inputs.hold else None,
        "tilt_deg": min(tilt_bounds) if tilt_bounds else None,
        "elbow_m": inputs.elbow_m,
        "view_deg": inputs.view_deg,
        "dexterity": inputs.dexterity,
        "clearance_m": CLEARANCE_MIN_M if inputs.clearance else None,
        "solve_ms": FRAME_MS,
    }
    values = [gauge.offset_m(live[gauge.key]) for gauge in GAUGES]
    for gauge in GAUGES:
        bound = shown[gauge.key]
        values.append(PARKED_M if bound is None else gauge.offset_m(bound))
    values += [0.0 if index == caption else PARKED_M for index in range(len(ACTS))]
    values += [float(travel) for travel in inputs.mark - home]
    values += [
        0.0 if inputs.cone_deg is not None and tracing is None else PARKED_M,
        0.0 if inputs.axis_deg is not None and inputs.cone_deg is None else PARKED_M,
        0.0 if tracing is not None else PARKED_M,
        PARKED_M if inputs.elbow_m is None else inputs.elbow_m,
        PARKED_M if inputs.view_deg is None else 0.0,
    ]
    return values


def trajectory_rows(results: dict) -> list[list[float]]:
    """Every act's rows on one clock, for the live viewer: `[q..., t]` per frame."""
    rows, offset = [], 0.0
    for result in results["acts"]:
        rows += [[*row[:-1], row[-1] + offset] for row in result.rows]
        offset += len(result.rows) / FRAME_HZ
    return rows


def _cone_mesh(half_angle: float, length: float, segments: int = 48):
    """A cone with its apex at the origin, opening along +z: vertices and faces."""
    angles = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    radius = length * np.tan(half_angle)
    rim = np.column_stack(
        [radius * np.cos(angles), radius * np.sin(angles), np.full(segments, length)]
    )
    vertices = np.vstack([[0.0, 0.0, 0.0], rim])
    faces = np.array([[0, 1 + i, 1 + (i + 1) % segments] for i in range(segments)])
    return vertices, faces


def _text_image(text: str) -> np.ndarray:
    """`text` as white glyphs on a transparent RGBA image, TEXT_LINE_PX high per line."""
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.load_default(size=round(0.78 * TEXT_LINE_PX))
    lines = text.split("\n")
    pad = TEXT_LINE_PX // 8
    width = max(round(font.getlength(line)) for line in lines) + 2 * pad
    mask = Image.new("L", (width, TEXT_LINE_PX * len(lines)), 0)
    draw = ImageDraw.Draw(mask)
    for row, line in enumerate(lines):
        middle = row * TEXT_LINE_PX + TEXT_LINE_PX // 2
        draw.text((pad, middle), line, fill=255, font=font, anchor="lm")
    alpha = np.asarray(mask)
    return np.dstack([np.full((*alpha.shape, 3), 255, dtype=np.uint8), alpha])


def _add_text(
    scene,
    name: str,
    text: str,
    anchor: np.ndarray,
    right: np.ndarray,
    line_height_m: float,
    *,
    align: str = "left",
    valign: str = "bottom",
) -> None:
    """Draw `text` in white on the plane that faces the view, its box anchored at `anchor`."""
    import viser.transforms as vtf

    image = _text_image(text)
    height = line_height_m * image.shape[0] / TEXT_LINE_PX
    width = height * image.shape[1] / image.shape[0]
    up = np.array([0.0, 0.0, 1.0])
    across = {"left": 0.5, "center": 0.0, "right": -0.5}[align] * width
    along = {"bottom": 0.5, "center": 0.0, "top": -0.5}[valign] * height
    # The image's x runs along `right` and its y down, so its face looks back at the view.
    axes = np.column_stack([right, -up, np.cross(right, -up)])
    scene.add_image(
        name,
        image,
        width,
        height,
        format="png",
        wxyz=vtf.SO3.from_matrix(axes).wxyz,
        position=tuple(anchor + across * right + along * up),
        cast_shadow=False,
        receive_shadow=False,
    )


def decorate(viewer, target: np.ndarray, solve_ms: Sequence[float] | None = None) -> list:
    """Draw what the instrument links carry: captions, the gauge panel and the rules' glyphs.

    Args:
        viewer: A `TesseractViewer` showing the cell.
        target: Where the mark starts [m].
        solve_ms: Each act's median solve [ms], for a third line under its caption.

    Returns:
        The handles of the translucent glyphs, for `add_opacity_slider`.
    """
    scene = viewer.server.scene
    # Image-based lighting, for the metal of the tool, the rail and the workpiece.
    scene.configure_environment_map(hdri="warehouse", environment_intensity=0.35)

    # Captions, one per act, each on its own link.
    corner, right = panel_frame()
    for index, act in enumerate(ACTS):
        caption = act.caption
        if solve_ms is not None and rule_count(act):
            caption += "\n" + solve_line(rule_count(act), solve_ms[index], median=True)
        _add_text(
            scene,
            f"/links/caption_{index}/text",
            caption,
            caption_anchor(),
            right,
            0.15,
            valign="top",
        )

    # The panel: a ruler per gauge, with its label above and its ends numbered.
    _add_text(
        scene,
        "/panel/legend",
        "ball: live value  ·  bar: its bound, while the rule is on",
        corner + [0, 0, ROW_PITCH_M],
        right,
        0.062,
    )
    rulers, ticks = [], []
    for row, gauge in enumerate(GAUGES):
        start = corner - np.array([0.0, 0.0, row * ROW_PITCH_M])
        end = start + RULER_M * right
        rulers.append([start, end])
        for point in (start, end):
            ticks.append([point - [0, 0, 0.025], point + [0, 0, 0.025]])
        _add_text(
            scene, f"/panel/{gauge.key}/label", gauge.label, start + [0, 0, 0.03], right, 0.085
        )
        _add_text(
            scene,
            f"/panel/{gauge.key}/lo",
            f"{gauge.lo:g}",
            start - 0.04 * right,
            right,
            0.062,
            align="right",
            valign="center",
        )
        _add_text(
            scene,
            f"/panel/{gauge.key}/hi",
            f"{gauge.hi:g}",
            end + 0.04 * right,
            right,
            0.062,
            valign="center",
        )
    scene.add_line_segments("/panel/rulers", np.array(rulers), GREY_RGB, thickness=0.006)
    scene.add_line_segments("/panel/ticks", np.array(ticks), GREY_RGB, thickness=0.006)

    # PointsDown: the cone the tool axis stays in, apex on the mark.
    glyphs = []
    cone_deg = next(act.cone_deg for act in ACTS if act.cone_deg is not None)
    vertices, faces = _cone_mesh(np.radians(cone_deg), 0.55)
    glyphs.append(
        scene.add_mesh_simple(
            "/links/cone_stage/cone",
            vertices,
            faces,
            color=ORANGE_RGB,
            opacity=GLYPH_OPACITY,
            side="double",
            cast_shadow=False,
        )
    )
    # HoldAxis: the line the tool axis locks onto.
    scene.add_line_segments(
        "/links/axis_stage/axis",
        np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.7]]]),
        ORANGE_RGB,
        thickness=0.008,
    )
    # The circle the mark moves on in the last act, through the mark.
    radius = next(act.trace_m for act in ACTS if act.trace_m is not None)
    turns = np.linspace(0.0, 2.0 * np.pi, 73)
    circle = radius * np.column_stack([np.cos(turns) - 1.0, np.sin(turns), np.zeros(73)])
    scene.add_line_segments(
        "/links/trace_stage/circle",
        np.stack([circle[:-1], circle[1:]], axis=1) + [0.0, 0.0, 0.002],
        ORANGE_RGB,
        thickness=0.006,
    )
    # ElbowUp: the plane the elbow stays above, along the rail.
    half_x, half_y = 0.45, 1.7
    plane = np.array(
        [[-half_x, -half_y, 0], [half_x, -half_y, 0], [half_x, half_y, 0], [-half_x, half_y, 0]]
    ) + [0.35, 0.0, 0.0]
    glyphs.append(
        scene.add_mesh_simple(
            "/links/plane_stage/plane",
            plane,
            np.array([[0, 1, 2], [0, 2, 3]]),
            color=ORANGE_RGB,
            opacity=GLYPH_OPACITY,
            side="double",
            cast_shadow=False,
        )
    )
    # WatchTheTool: the cone the tip stays in and the camera's optical axis, out to the mark.
    reach = float(np.linalg.norm(target - [*CAMERA_MAST_XY_M, CAMERA_HEIGHT_M])) + 0.25
    vertices, faces = _cone_mesh(np.radians(VIEW_DEG), reach)
    glyphs.append(
        scene.add_mesh_simple(
            "/links/view_stage/cone",
            vertices,
            faces,
            color=ORANGE_RGB,
            opacity=GLYPH_OPACITY,
            side="double",
            cast_shadow=False,
        )
    )
    scene.add_line_segments(
        "/links/view_stage/axis",
        np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, reach]]]),
        ORANGE_RGB,
        thickness=0.004,
    )
    # Clearance: the ball's slide, and the shell the forearm stays out of.
    slide_x, slide_z = BALL_LINE_XZ_M
    scene.add_line_segments(
        "/slide",
        np.array([[[slide_x, BALL_PARKED_M - 0.3, slide_z], [slide_x, 1.2, slide_z]]]),
        GREY_RGB,
        thickness=0.006,
    )
    glyphs.append(
        scene.add_icosphere(
            "/links/obstacle/shell",
            radius=BALL_RADIUS_M + CLEARANCE_MIN_M,
            color=(255, 90, 60),
            opacity=GLYPH_OPACITY,
            cast_shadow=False,
        )
    )
    return glyphs


def face_view(viewer) -> None:
    """Start the live viewer on the view the panel and the captions are laid out for."""
    camera = viewer.server.initial_camera
    camera.position = tuple(VIEW_POSITION_M)
    camera.look_at = tuple(VIEW_LOOK_AT_M)
    camera.fov = VIEW_FOV_RAD


def add_opacity_slider(viewer, glyphs: list):
    """A slider for how solid the rules' translucent glyphs are drawn; returns its handle."""
    slider = viewer.server.gui.add_slider(
        "Constraint opacity", min=0.0, max=1.0, step=0.01, initial_value=GLYPH_OPACITY
    )

    @slider.on_update
    def _(_event) -> None:
        for glyph in glyphs:
            glyph.opacity = slider.value

    return slider


# --8<-- [start:sliders]
def drive_with_sliders(viewer, loop: LiveLoop, stop: threading.Event) -> None:
    """Solve a frame per tick from the GUI's state, until `stop` is set.

    The rail swings by itself or follows its slider, or is handed to the solver, which then
    keeps the carriage abeam of the tip; the ball follows its slider; the mark follows the
    handle dragged in the scene; every rule has a checkbox, and a slider where it has a bound.
    A bound that comes on closes in from the live value, as the acts' bounds ramp.
    """
    gui = viewer.server.gui
    handle = viewer.server.scene.add_transform_controls(
        "/mark_handle",
        scale=MARK_HANDLE_M,
        position=tuple(loop.target),
        disable_rotations=True,
        depth_test=False,
    )
    with gui.add_folder("Inputs"):
        swings = gui.add_checkbox("Rail swings by itself", True)
        rail = gui.add_slider(
            "Rail [m]", min=-RAIL_SWING_M, max=RAIL_SWING_M, step=0.01, initial_value=0.0
        )
        ours = gui.add_checkbox("Rail is the solver's 7th axis", False)
        follow = gui.add_slider(
            "Carriage follows the tip within [m]",
            min=0.02,
            max=1.0,
            step=0.01,
            initial_value=FOLLOW_BAND_M,
        )
        gui.add_markdown(
            "Ticked, the carriage drives to abeam of the tip and stays there: drag the mark by "
            "its handle. Widen the band for the other rules to move the carriage."
        )
        ball = gui.add_slider(
            "Ball on its slide [m]",
            min=BALL_PARKED_M,
            max=0.3,
            step=0.01,
            initial_value=BALL_PARKED_M,
        )
    with gui.add_folder("Rules"):
        hold = gui.add_checkbox("WorldHold", True)
        axis_on = gui.add_checkbox("HoldAxis", False)
        axis = gui.add_slider(
            "HoldAxis tolerance [°]", min=0.5, max=20.0, step=0.5, initial_value=0.5
        )
        cone_on = gui.add_checkbox("PointsDown", False)
        cone = gui.add_slider(
            "PointsDown cone [°]", min=5.0, max=40.0, step=1.0, initial_value=20.0
        )
        elbow_on = gui.add_checkbox("ElbowUp", False)
        elbow = gui.add_slider(
            "Elbow at least [m]", min=1.10, max=1.35, step=0.005, initial_value=1.2
        )
        view_on = gui.add_checkbox("WatchTheTool", True)
        view = gui.add_slider(
            "Tip off the camera axis at most [°]",
            min=1.0,
            max=25.0,
            step=0.5,
            initial_value=VIEW_DEG,
        )
        dexterous_on = gui.add_checkbox("StayDexterous", False)
        dexterous = gui.add_slider(
            "Manipulability at least", min=0.7, max=1.5, step=0.01, initial_value=0.8
        )
        clearance = gui.add_checkbox("Clearance", False)
    status = gui.add_text("Solve", initial_value="", disabled=True)

    began = time.perf_counter()
    recent: list[float] = []
    while not stop.is_set():
        tick = time.perf_counter()
        if swings.value and not ours.value:
            rail.value = float(RAIL_SWING_M * np.sin(2.0 * np.pi * (tick - began) / RAIL_PERIOD_S))
        live = loop.live
        inputs = Inputs(
            rail_m=None if ours.value else float(rail.value),
            ball_m=float(ball.value),
            mark=np.array(handle.position, dtype=np.float64),
            hold=hold.value,
            axis_deg=_closing("tilt_deg", axis.value, live) if axis_on.value else None,
            cone_deg=_closing("tilt_deg", cone.value, live) if cone_on.value else None,
            elbow_m=_closing("elbow_m", elbow.value, live) if elbow_on.value else None,
            view_deg=_closing("view_deg", view.value, live) if view_on.value else None,
            dexterity=_closing("dexterity", dexterous.value, live) if dexterous_on.value else None,
            clearance=clearance.value,
            follow_m=_closing("lead_m", follow.value, live) if ours.value else None,
        )
        frame = loop.step(inputs)
        if ours.value:
            # Show where the solver put the carriage; the slider takes over from there.
            rail.value = float(np.clip(frame.q[0], -RAIL_SWING_M, RAIL_SWING_M))
        viewer.update_joint_positions(
            loop.joint_names,
            [
                *frame.q,
                inputs.ball_m,
                *instrument_values(inputs, frame.live, loop.target, caption=None),
            ],
        )
        feasible = frame.violation < VIOLATION_TOLERANCE
        status.value = f"{frame.solve_ms:.1f} ms" + ("" if feasible else "  ·  rules conflict")
        recent.append(frame.solve_ms)
        if len(recent) == READOUT_FRAMES:
            # Where an act's caption would be: what the solves of the last frames took.
            on = rule_count(inputs) + (inputs.follow_m is not None)
            _add_text(
                viewer.server.scene,
                "/readout",
                solve_line(on, float(np.median(recent))),
                caption_anchor(),
                panel_frame()[1],
                0.15,
                valign="top",
            )
            recent.clear()
        stop.wait(max(0.0, 1.0 / FRAME_HZ - (time.perf_counter() - tick)))


# --8<-- [end:sliders]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument(
        "--sliders",
        action="store_true",
        help="drive the rail, the ball and every rule yourself, instead of playing the acts",
    )
    args = parser.parse_args(argv)

    from tesseract_robotics.viewer import TesseractViewer, Trajectory

    if args.sliders:
        loop = LiveLoop()
        with TesseractViewer() as viewer:
            viewer.update_environment(loop.robot.env)
            face_view(viewer)
            add_opacity_slider(viewer, decorate(viewer, loop.target))
            stop = threading.Event()
            driver = threading.Thread(
                target=drive_with_sliders, args=(viewer, loop, stop), daemon=True
            )
            driver.start()
            print(f"Viewer at {viewer.url}: the sliders are in its control panel")
            input("Press Enter to exit...")
            stop.set()
            driver.join()
        return 0

    results = run()
    print(f"{'act':<62} {'frames':>6} {'solve [ms] median':>18} {'max':>7} {'violation':>10}")
    for result in results["acts"]:
        print(
            f"{result.act.caption.splitlines()[0][:60]:<62} {len(result.rows):>6} "
            f"{np.median(result.solve_ms):>18.2f} {max(result.solve_ms):>7.2f} "
            f"{result.worst_violation:>10.1e}"
        )
    with TesseractViewer() as viewer:
        viewer.update_environment(results["robot"].env)
        face_view(viewer)
        add_opacity_slider(viewer, decorate(viewer, results["target"], act_solve_ms(results)))
        # The viewer's own Playback controls start, stop and scrub the acts.
        viewer.update_trajectory(
            Trajectory.from_rows(results["joint_names"], trajectory_rows(results))
        )
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0 if results["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
