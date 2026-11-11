"""Hold the tool still in the world while the base moves: a closed-loop simulation.

The arm (ABB IRB 2400, tesseract_support) rides on a base whose world pose is "measured" every
tick: a steady drive plus the rocking of a mobile base. Each tick solves a one-waypoint
TrajOpt problem with two Python rules from trajopt_ifopt.python_constraints:

- WorldHold: the tool point stays on a fixed world point,
- HoldAxis: the tool axis stays on a fixed world direction,

and a squared stay-close cost to the previous joints. The solution is executed over the next
period, so the error reported is the one at the *next* base pose (one tick of latency).
Synthetic base motion, no robot: this shows the loop and its error budget, not field accuracy.

    pixi run python scripts/sim_world_hold.py out.json [--rate 50] [--speed 0.05] [--seconds 8]
"""

from __future__ import annotations

import argparse
import json
import statistics
import time

import numpy as np
from scipy.spatial.transform import Rotation

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.planning import Robot
from tesseract_robotics.trajopt_ifopt.python_constraints import HoldAxis, WorldHold

TCP = "tool0"
# start pose of the arm in its own base frame: tool out in front, pointing forward and down
Q0 = np.array([0.0, 0.35, 0.25, 0.0, 0.6, 0.0])
# rocking of the base: amplitude and frequency per axis
SWAY_M = np.array([0.0, 0.010, 0.003])  # y, z [m]
SWAY_HZ = np.array([0.0, 0.5, 1.3])
ROCK_DEG = np.array([0.4, 0.5, 1.0])  # roll, pitch, yaw [deg]
ROCK_HZ = np.array([1.1, 0.9, 0.3])
# tool-axis tolerance [rad] and merit weight of the rules against the stay-close cost
AXIS_TOL = np.radians(0.1)
RULE_COEFF = 50.0


def base_pose(t: float, speed: float) -> Isometry3d:
    """World pose of the base at time t: a steady drive along x, plus rocking."""
    sway = SWAY_M * np.sin(2 * np.pi * SWAY_HZ * t)
    rock = np.radians(ROCK_DEG) * np.sin(2 * np.pi * ROCK_HZ * t)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", rock).as_matrix()
    pose[:3, 3] = np.array([speed * t, 0.0, 0.0]) + sway
    return Isometry3d(pose)


def world(pose: Isometry3d, manip, q):
    tf = manip.calcFwdKin(q)[TCP]
    return pose.rotation @ tf.translation + pose.translation, pose.rotation @ tf.rotation[:, 2]


def solve_tick(manip, joint_names, limits, q_prev, pose, target, axis_world):
    nodes = ti.createNodesVariables("tick", joint_names, [q_prev], limits)
    var = nodes.getNodes()[0].getVar("joints")
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addCostSet(
        ti.JointPosConstraint(q_prev, var, np.ones(len(q_prev)), "stay_close"),
        tsqp.CostPenaltyType.SQUARED,
    )
    problem.addConstraintSet(WorldHold(var, manip, TCP, target, pose, tol=0.0, coeff=RULE_COEFF))
    problem.addConstraintSet(
        HoldAxis(var, manip, TCP, pose.rotation.T @ axis_world, AXIS_TOL, coeff=RULE_COEFF)
    )
    problem.setup()
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())
    solver.solve(problem)
    return np.asarray(var.value())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--rate", type=float, default=50.0, help="control rate [Hz]")
    ap.add_argument("--speed", type=float, default=0.05, help="base speed [m/s]")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    robot = Robot.from_tesseract_support(
        "abb_irb2400", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    manip = robot.env.getKinematicGroup("manipulator")
    joint_names = robot.get_joint_names("manipulator")
    limits = ti.toBounds(manip.getLimits().joint_limits)
    dt = 1.0 / args.rate
    ticks = int(args.seconds * args.rate)

    target, axis_world = world(base_pose(0.0, args.speed), manip, Q0)
    q = Q0.copy()
    rows = []
    for k in range(ticks):
        t = k * dt
        now, nxt = base_pose(t, args.speed), base_pose(t + dt, args.speed)
        t0 = time.perf_counter()
        q = solve_tick(manip, joint_names, limits, q, now, target, axis_world)
        solve_ms = (time.perf_counter() - t0) * 1e3
        p_now, _ = world(now, manip, q)  # at the pose the solve saw
        p_exec, a_exec = world(nxt, manip, q)  # when the command is carried out
        p_frozen, _ = world(nxt, manip, Q0)  # an arm that does not compensate
        rows.append(
            {
                "t": t + dt,
                "held_mm": float(np.linalg.norm(p_exec - target) * 1e3),
                "solved_mm": float(np.linalg.norm(p_now - target) * 1e3),
                "frozen_mm": float(np.linalg.norm(p_frozen - target) * 1e3),
                "axis_deg": float(np.degrees(np.arccos(np.clip(a_exec @ axis_world, -1, 1)))),
                "solve_ms": solve_ms,
            }
        )

    def stat(key):
        vals = [r[key] for r in rows]
        return {"median": statistics.median(vals), "max": max(vals)}

    summary = {k: stat(k) for k in ("held_mm", "solved_mm", "frozen_mm", "axis_deg", "solve_ms")}
    with open(args.out, "w") as fh:
        json.dump({"args": vars(args), "summary": summary, "rows": rows}, fh)
    for key, s in summary.items():
        print(f"{key:10s} median {s['median']:8.3f}  max {s['max']:8.3f}")


if __name__ == "__main__":
    main()
