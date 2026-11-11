"""Files the reference pages' snippets read from the reader's working directory.

A page's ambient comment calls `enter_scratch_dir()` first, so the files land in a fresh
temporary directory that is the process's working directory, never in the repository.
"""

import atexit
import os
import shutil
import struct
import tempfile
import zlib
from pathlib import Path

# A unit tetrahedron: four vertices, four outward-facing triangles.
TETRAHEDRON_VERTICES = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
TETRAHEDRON_FACES = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
# A 2 x 2 white PNG: signature, IHDR, IDAT, IEND chunks.
PNG_SIDE_PX = 2


def enter_scratch_dir() -> Path:
    """Create a temporary working directory, enter it and remove it at interpreter exit."""
    scratch = Path(tempfile.mkdtemp(prefix="doc-snippets-"))
    previous = Path.cwd()
    os.chdir(scratch)

    def cleanup() -> None:
        os.chdir(previous)
        shutil.rmtree(scratch, ignore_errors=True)

    atexit.register(cleanup)
    return scratch


def write_stl(path: Path) -> None:
    """Write the tetrahedron as an ASCII STL."""
    lines = ["solid tetrahedron"]
    for a, b, c in TETRAHEDRON_FACES:
        lines.append("facet normal 0 0 0\n outer loop")
        lines.extend(
            f"  vertex {x} {y} {z}" for x, y, z in (TETRAHEDRON_VERTICES[i] for i in (a, b, c))
        )
        lines.append(" endloop\nendfacet")
    lines.append("endsolid tetrahedron")
    path.write_text("\n".join(lines) + "\n")


def write_collada(path: Path) -> None:
    """Write the tetrahedron as a minimal Collada document with one triangle list."""
    positions = " ".join(str(c) for v in TETRAHEDRON_VERTICES for c in v)
    indices = " ".join(str(i) for face in TETRAHEDRON_FACES for i in face)
    path.write_text(
        f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="tetra" name="tetra">
      <mesh>
        <source id="tetra-pos">
          <float_array id="tetra-pos-array" count="12">{positions}</float_array>
          <technique_common>
            <accessor source="#tetra-pos-array" count="4" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="tetra-vtx"><input semantic="POSITION" source="#tetra-pos"/></vertices>
        <triangles count="4">
          <input semantic="VERTEX" source="#tetra-vtx" offset="0"/>
          <p>{indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="tetra-node"><instance_geometry url="#tetra"/></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
    )


def write_png(path: Path) -> None:
    """Write a 2 x 2 white RGB PNG."""

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", PNG_SIDE_PX, PNG_SIDE_PX, 8, 2, 0, 0, 0)
    row = b"\x00" + b"\xff\xff\xff" * PNG_SIDE_PX
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(row * PNG_SIDE_PX))
        + chunk(b"IEND", b"")
    )


# Joint 1 of the IRB2400 swings from its zero configuration to this angle (rad): a short
# freespace move with no obstacle in the way.
PROGRAM_GOAL_JOINT_1_RAD = 0.3


def joint_program(robot):  # noqa: ANN001, ANN201 - the planning types are imported lazily
    """A two-waypoint joint-space `CompositeInstruction` for the IRB2400 `robot`."""
    import numpy as np

    from tesseract_robotics.planning import JointTarget, MotionProgram

    names = robot.get_joint_names("manipulator")
    goal = np.zeros(len(names))
    goal[0] = PROGRAM_GOAL_JOINT_1_RAD
    return (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(names)
        .move_to(JointTarget(np.zeros(len(names))))
        .move_to(JointTarget(goal))
        .to_composite_instruction()
    )


# Waypoints of the interpolated trajectory the SQP pages' snippets optimise.
SQP_STEPS = 5
# Joint 1 goal of that trajectory (rad), as in `joint_program`.
SQP_GOAL_JOINT_1_RAD = 0.3
# Collision margin (m) and coefficient of the collision config the snippets build.
SQP_COLLISION_MARGIN_M = 0.1
SQP_COLLISION_COEFF = 10.0
# Coefficient of the start-position constraint and of the velocity cost.
SQP_HOME_COEFF = 5.0


def sqp_scene(robot) -> dict:  # noqa: ANN001 - the SQP types are imported lazily
    """The names the SQP pages' snippets use, built for the IRB2400 `robot`.

    Returns:
        `joint_names`, `initial_states`, `states`, `bounds`, `nodes_variables`, `vars_list`,
        `var`, `manip`, `n_steps`, `trajectory`, `problem`, `qp_solver` and `solver`: a
        `TrajOptQPProblem` of a home constraint and a squared velocity cost, set up, and a
        `TrustRegionSQPSolver` initialised on it.
    """
    import numpy as np

    from tesseract_robotics import trajopt_ifopt as ti
    from tesseract_robotics import trajopt_sqp as tsqp

    manip = robot.env.getKinematicGroup("manipulator")
    joint_names = robot.get_joint_names("manipulator")
    start = np.zeros(len(joint_names))
    goal = start.copy()
    goal[0] = SQP_GOAL_JOINT_1_RAD
    initial_states = list(ti.interpolate(start, goal, SQP_STEPS))
    bounds = ti.toBounds(manip.getLimits().joint_limits)
    nodes_variables = ti.createNodesVariables("trajectory", joint_names, initial_states, bounds)
    vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]

    problem = tsqp.TrajOptQPProblem(nodes_variables)
    problem.addConstraintSet(
        ti.JointPosConstraint(
            start, vars_list[0], np.full(len(joint_names), SQP_HOME_COEFF), "Home"
        )
    )
    problem.addCostSet(
        ti.JointVelConstraint(np.zeros(len(joint_names)), vars_list, np.ones(1), "JointVelocity"),
        tsqp.CostPenaltyType.SQUARED,
    )
    problem.setup()

    collision_config = ti.TrajOptCollisionConfig(SQP_COLLISION_MARGIN_M, SQP_COLLISION_COEFF)
    evaluator = ti.SingleTimestepCollisionEvaluator(manip, robot.env, collision_config)
    qp_solver = tsqp.OSQPEigenSolver()
    solver = tsqp.TrustRegionSQPSolver(qp_solver)
    solver.init(problem)
    return {
        "collision_config": collision_config,
        "evaluator": evaluator,
        "collision_constraint": ti.DiscreteCollisionConstraint(
            evaluator, vars_list[1], 1, False, "Collision"
        ),
        "joint_constraint": ti.JointPosConstraint(
            start, vars_list[0], np.ones(len(joint_names)), "Joints"
        ),
        "my_constraint": ti.JointPosConstraint(
            start, vars_list[0], np.ones(len(joint_names)), "Mine"
        ),
        "vel_cost": ti.JointVelConstraint(np.zeros(len(joint_names)), vars_list, np.ones(1), "Vel"),
        "q": start,
        "joint_names": joint_names,
        "initial_states": initial_states,
        "states": initial_states,
        "bounds": bounds,
        "nodes_variables": nodes_variables,
        "vars_list": vars_list,
        "var": vars_list[0],
        "manip": manip,
        "n_steps": SQP_STEPS,
        "trajectory": np.array(initial_states),
        "problem": problem,
        "qp_solver": qp_solver,
        "solver": solver,
    }
