# Glossary

Each term is used with this meaning throughout the documentation. Units: lengths in metres,
angles in radians, time in seconds, quaternions scalar-last `[qx, qy, qz, qw]`.

## World model

Environment
:   The robot cell: scene graph, state, kinematic groups and contact managers, changed only by
    commands. `tesseract_robotics.environment.Environment`; see [Concepts](index.md#the-environment).

Scene graph
:   The tree of links joined by joints, with the allowed collision matrix. Built from the URDF.

Link
:   A rigid body with visual and collision geometry. Identified by name (`"tool0"`) or `LinkId`.

Joint
:   A connection between a parent and a child link: fixed, revolute, continuous, prismatic,
    planar or floating. Joint values are rad (revolute) or m (prismatic).

State
:   Joint values plus the pose of every link in the root frame, computed by the **state
    solver**. `Environment.getState()`.

Command
:   One change to the environment (`AddLinkCommand`, `MoveJointCommand`, …). The environment
    records each command it applies, so its history replays the same world.

Allowed collision matrix (ACM)
:   The link pairs a collision check skips: adjacent links, links that never touch, a gripper and
    the part it holds. Defined in the SRDF, changed by commands.

Resource locator
:   Resolves `package://` URLs in URDF and SRDF files to paths. `GeneralResourceLocator`; see
    [Resource locators](bindings.md#resource-locators).

## Kinematics

Kinematic group
:   A set of joints that move together, defined in the SRDF (`"manipulator"`), with a forward and
    an inverse kinematics solver. `Environment.getKinematicGroup()`; a **joint group** is the
    same without IK.

Forward kinematics (FK)
:   Link poses from joint values.

Inverse kinematics (IK)
:   Joint values that put a link at a pose. May return several solutions or none
    (`IKNoSolutionError`). The **seed** is the joint state a numerical solver starts from.

## Frames

Root frame
:   The frame of the scene graph's root link. Every link pose resolves to it.

TCP (tool centre point)
:   The point and orientation on the tool that a Cartesian target places.
    TCP = `tcp_frame` · `tcp_offset`; see [Frames and poses](frames.md).

`tcp_frame`
:   The link the tool is mounted on, typically the flange `tool0`.

`tcp_offset`
:   The fixed transform from `tcp_frame` to the TCP: an `Isometry3d` or the name of an SRDF
    `group_tcps` entry.

Working frame
:   The frame a Cartesian target is expressed in: the robot base, or a workpiece link the targets
    move with.

Pose
:   A rigid transform: position (m) and orientation. `Isometry3d`, or its subclass
    `planning.Pose`.

## Programs

Waypoint
:   A target for one move. A **joint waypoint** gives joint values; a **Cartesian waypoint** a
    TCP pose in the working frame; a **state waypoint** a full joint state with velocities,
    accelerations and time, the form planners return.

Move instruction
:   One move to a waypoint, with a **move type** (`FREESPACE`: any collision-free path;
    `LINEAR`: the TCP on a straight line; `CIRCULAR`: an arc) and a profile name.

Composite instruction
:   An ordered list of instructions, nested to any depth, with the `ManipulatorInfo` (group,
    `tcp_frame`, `working_frame`, `tcp_offset`). A whole **program** is one.

Profile
:   Named planner settings, looked up by the pair (task name, profile name). See
    [Profiles](index.md#profiles).

Trajectory
:   The planned sequence of joint states. After time parameterization each state carries
    velocities, accelerations and a time stamp.

Time parameterization
:   Assigning times to a geometric path within velocity, acceleration (and, for Ruckig, jerk)
    limits. TOTG, ISP, constant TCP speed, Ruckig; see
    [`time_parameterization`](../api/time_parameterization.md).

## Planning

Planner
:   An algorithm that turns a program into a trajectory: OMPL (sampling), TrajOpt and
    TrajOpt-Ifopt (sequential convex optimization), Descartes (graph search over IK solutions),
    the simple planner (interpolation). See [Choosing a planner](../user-guide/planning.md#choosing-a-planner).

Task
:   One node of a pipeline: a planner, a check (contact, minimum length), a fix-up, or a time
    parameterization.

Pipeline
:   A graph of tasks with an error branch, defined in YAML and loaded as a plugin
    (`TrajOptPipeline`, `FreespacePipeline`, …).

Task composer
:   Loads pipelines and runs them. `planning.TaskComposer`; see
    [Task composer](../user-guide/task-composer.md).

Cost and constraint
:   A cost is minimised (soft); a constraint must hold at the checked points (hard). See
    [Costs vs constraints](../user-guide/planning.md#costs-vs-constraints-soft-vs-hard).

## Collision

Contact manager
:   A collision world. **Discrete** managers check one state; **continuous** managers check the
    swept volume between two states. Bullet and FCL implementations, loaded as plugins.

Contact distance
:   The signed distance between two collision objects: positive when apart, negative when
    penetrating.

Margin
:   The distance below which a contact manager reports a pair (its contact threshold). A margin
    of 0.05 reports every pair closer than 5 cm, touching or not.

LVS (longest valid segment)
:   A trajectory check that subdivides each motion into segments no longer than a set length and
    checks each, discretely (`LVS_DISCRETE`) or as a swept volume (`LVS_CONTINUOUS`). See
    [Collision](../user-guide/collision.md).
