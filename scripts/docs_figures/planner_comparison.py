"""OMPL, OMPL + TrajOpt, TrajOpt and TrajOptIfopt on one freespace problem, measured.

The problem is upstream's freespace example (`tesseract_examples/src/freespace_ompl_example.cpp`):
the KUKA LBR IIWA 14 R820 swings joint_a1 from -0.4 to 0.4 rad past a 0.15 m sphere at
(0.5, 0, 0.55) m. Each pipeline plans it once per seed, each seed in a fresh process: OMPL's RNG
can be seeded only before its first solve (`RNG_setSeed`). Descartes is not compared: it follows
Cartesian toolpaths and has no joint-to-joint freespace query.

Measuring is separate from drawing. `pixi run docs-figures-measure planner_comparison` runs the
benchmark and writes `docs/assets/figures/data/planner_comparison.json` (raw rows plus hardware,
OS, library build, seeds, N and timestamp); the figure renders deterministically from that file.

Per plan:

- joint path length: sum of joint-space step norms over the trajectory, densified (rad);
- TCP path length: length of tool0's path over the same densified states (m);
- clearance: smallest distance between any robot link and the sphere over those states (m);
- plan time: wall time of `TaskComposer.plan` (the whole pipeline, including its time
  parameterisation and contact check), the first plan of that pipeline in the process,
  after `TaskComposer.warmup` loaded every pipeline's plugins (s).
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Final

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from matplotlib.ticker import NullFormatter, ScalarFormatter
from numpy.typing import NDArray

from scripts.docs_figures.style import (
    BLUE,
    COLUMN_WIDTH_IN,
    FIGURES_DIR,
    GREEN,
    ORANGE,
    PURPLE,
    REPO_ROOT,
    Theme,
)

DATA_PATH: Final[Path] = FIGURES_DIR / "data" / "planner_comparison.json"
DATA_FORMAT: Final[int] = 1
# Seeds 1..N, one process each. N = 20: medians and spreads of the sampling planners settle
# (a seed costs about 10 s, so a measurement takes under 4 min).
SEEDS: Final[tuple[int, ...]] = tuple(range(1, 21))
# s: one child's budget: plugin loading (~5 s), four plans each capped by OMPL_PLANNING_TIME_S
# or TrajOpt's iteration limits, and the metric scans; 10 x the slowest child seen (~12 s).
CHILD_TIMEOUT_S: Final[float] = 120.0

# The problem, from upstream's freespace_ompl_example.cpp.
ROBOT: Final[str] = "lbr_iiwa_14_r820"
GROUP: Final[str] = "manipulator"
TCP_FRAME: Final[str] = "tool0"
OBSTACLE: Final[str] = "sphere_attached"
OBSTACLE_RADIUS_M: Final[float] = 0.15
OBSTACLE_XYZ_M: Final[tuple[float, float, float]] = (0.5, 0.0, 0.55)
JOINT_START: Final[tuple[float, ...]] = (-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0)
JOINT_END: Final[tuple[float, ...]] = (0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0)

# OMPL settings: one planner thread (one RNG stream per seed), the first solution (so plan time
# measures time to a path, not a fixed optimisation budget), the library's default range.
OMPL_PLANNERS: Final[int] = 1
OMPL_PLANNING_TIME_S: Final[float] = 10.0
# rad: densification step of every trajectory before the metrics, joint-space norm; finer than
# any planner's own output spacing, so all four are measured on the same footing.
DENSIFY_STEP_RAD: Final[float] = 0.005
# m: contact distance the clearance query reports up to; larger than any clearance on this
# problem (the arm passes within ~0.3 m of the sphere at the start state).
CLEARANCE_QUERY_M: Final[float] = 1.0

# (label, pipeline, profile factory key, colour)
PIPELINES: Final[tuple[tuple[str, str, str, str], ...]] = (
    ("OMPL", "OMPLPipeline", "ompl", BLUE),
    ("OMPL + TrajOpt", "FreespacePipeline", "freespace", GREEN),
    ("TrajOpt", "TrajOptPipeline", "trajopt", ORANGE),
    ("TrajOptIfopt", "TrajOptIfoptPipeline", "trajopt_ifopt", PURPLE),
)
# (row key, panel title, display scale): quality on the top row, time on the bottom.
QUALITY: Final[tuple[tuple[str, str, float], ...]] = (
    ("joint_path_rad", "joint path length (rad)", 1.0),
    ("tcp_path_m", "TCP path length (m)", 1.0),
    ("clearance_m", "min clearance to sphere (mm)", 1e3),
)
TIMING: Final[tuple[tuple[str, str, float], ...]] = (
    ("planner_time_s", "planner task time (ms, log scale)", 1e3),
    ("plan_time_s", "whole pipeline time (ms)", 1e3),
)
METRICS: Final[tuple[tuple[str, str, float], ...]] = QUALITY + TIMING


class MeasurementDataError(ValueError):
    """The committed measurement JSON does not have the shape the figure draws."""


# --------------------------------------------------------------------------- measuring


def _profiles(key: str):
    from tesseract_robotics.planning.profiles import (
        create_freespace_pipeline_profiles,
        create_ompl_default_profiles,
        create_trajopt_default_profiles,
        create_trajopt_ifopt_default_profiles,
    )

    if key == "ompl":
        return create_ompl_default_profiles(
            planning_time=OMPL_PLANNING_TIME_S,
            optimize=False,
            max_solutions=1,
            num_planners=OMPL_PLANNERS,
        )
    if key == "freespace":
        return create_freespace_pipeline_profiles(
            num_planners=OMPL_PLANNERS,
            planning_time=OMPL_PLANNING_TIME_S,
            optimize=False,
            max_solutions=1,
        )
    if key == "trajopt":
        return create_trajopt_default_profiles()
    if key == "trajopt_ifopt":
        return create_trajopt_ifopt_default_profiles()
    raise KeyError(key)


def densify(positions: NDArray[np.float64], step: float) -> NDArray[np.float64]:
    """Joint states every at most `step` (joint-space norm) along a piecewise-linear path."""
    states = [positions[0]]
    for a, b in zip(positions[:-1], positions[1:], strict=True):
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        states.extend(a + (b - a) * (i / count) for i in range(1, count + 1))
    return np.array(states)


def _child(seed: int) -> list[dict[str, object]]:
    """One seed: every pipeline planned once, in this fresh process."""
    from tesseract_robotics import get_tesseract_resource_path
    from tesseract_robotics.collision import ContactRequest, ContactResultMap, ContactTestType
    from tesseract_robotics.common import GeneralResourceLocator, LinkId
    from tesseract_robotics.motion_planners_ompl import RNG_setSeed
    from tesseract_robotics.planning import (
        JointTarget,
        MotionProgram,
        Pose,
        Robot,
        TaskComposer,
        create_obstacle,
        sphere,
    )

    tcp_link = LinkId(TCP_FRAME)  # calcFwdKin keys its result by LinkId
    RNG_setSeed(seed)
    composer = TaskComposer.from_config()
    composer.warmup([pipeline for _, pipeline, _, _ in PIPELINES])
    rows = []
    for label, pipeline, key, _ in PIPELINES:
        robot = Robot.from_tesseract_support(
            ROBOT, GeneralResourceLocator(paths=[get_tesseract_resource_path()])
        )
        create_obstacle(robot, OBSTACLE, sphere(OBSTACLE_RADIUS_M), Pose.from_xyz(*OBSTACLE_XYZ_M))
        names = robot.get_joint_names(GROUP)
        robot.set_joints(np.array(JOINT_START), joint_names=names)
        program = (
            MotionProgram(GROUP, tcp_frame=TCP_FRAME, working_frame="base_link")
            .set_joint_names(names)
            .move_to(JointTarget(np.array(JOINT_START)))
            .move_to(JointTarget(np.array(JOINT_END)))
        )
        profiles = _profiles(key)
        start = time.perf_counter()
        result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
        elapsed = time.perf_counter() - start
        row: dict[str, object] = {
            "seed": seed,
            "planner": label,
            "pipeline": pipeline,
            "success": bool(result.successful),
            "message": result.message,
            "plan_time_s": elapsed,
            # s: the motion planner nodes alone (OMPL, TrajOpt, ...), from the run's node infos;
            # the rest of plan_time_s is the pipeline's own tasks (checks, interpolation, time
            # parameterisation, contact check).
            "planner_time_s": sum(
                info.elapsed_time
                for info in result.task_infos
                if info.name.endswith("MotionPlannerTask")
            ),
            "planner_nodes": sorted(
                info.name for info in result.task_infos if info.name.endswith("MotionPlannerTask")
            ),
            "waypoints": len(result),
        }
        if result.successful:
            positions = np.array([p.positions for p in result])
            dense = densify(positions, DENSIFY_STEP_RAD)
            group = robot.env.getKinematicGroup(GROUP)
            tcp = np.array([group.calcFwdKin(q)[tcp_link].matrix[:3, 3] for q in dense])
            manager = robot.env.getDiscreteContactManager()
            manager.setActiveCollisionObjects(robot.env.getActiveLinkIds())
            manager.setDefaultCollisionMargin(CLEARANCE_QUERY_M)
            clearance = np.inf
            for q in dense:
                manager.setCollisionObjectsTransform(robot.env.getState(names, q).link_transforms)
                contacts = ContactResultMap()
                manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
                for contact in contacts.flattenCopyResults():
                    if OBSTACLE in [link.name() for link in contact.link_ids]:
                        clearance = min(clearance, contact.distance)
            if not np.isfinite(clearance):
                raise RuntimeError(
                    f"{label}: no robot link within {CLEARANCE_QUERY_M} m of the sphere"
                )
            row |= {
                "joint_path_rad": float(np.sum(np.linalg.norm(np.diff(dense, axis=0), axis=1))),
                "tcp_path_m": float(np.sum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))),
                "clearance_m": float(clearance),
                "goal_error_rad": float(np.max(np.abs(positions[-1] - np.array(JOINT_END)))),
            }
        rows.append(row)
    return rows


def _command_output(command: list[str]) -> str:
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout.strip()


def _hardware() -> dict[str, object]:
    if sys.platform == "darwin":
        cpu = _command_output(["sysctl", "-n", "machdep.cpu.brand_string"])
        memory = int(_command_output(["sysctl", "-n", "hw.memsize"]))
    else:
        info = Path("/proc/cpuinfo").read_text()
        cpu = next(
            line.split(":", 1)[1].strip()
            for line in info.splitlines()
            if line.startswith("model name")
        )
        memory = os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES")
    return {
        "cpu": cpu,
        "logical_cpus": os.cpu_count(),
        "memory_gib": round(memory / 2**30, 1),
        "os": platform.platform(),
        # Machine load when the measurement started: other work on the host inflates times.
        "load_average_1_5_15": [round(x, 2) for x in os.getloadavg()],
        "python": platform.python_version(),
    }


def measure() -> Path:
    """Run every seed in its own process and write the measurement JSON."""
    from importlib.metadata import version

    from tesseract_robotics.build_identity import UPSTREAM_SOURCES

    rows = []
    for seed in SEEDS:
        output = subprocess.run(
            [sys.executable, "-m", "scripts.docs_figures.planner_comparison", "--child", str(seed)],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=CHILD_TIMEOUT_S,
        ).stdout
        seed_rows = json.loads(output.strip().splitlines()[-1])
        rows.extend(seed_rows)
        print(
            f"seed {seed}: "
            + ", ".join(f"{r['planner']} {r['plan_time_s']:.2f}s" for r in seed_rows)
        )
    status = _command_output(["git", "-C", str(REPO_ROOT), "status", "--porcelain"])
    data = {
        "format": DATA_FORMAT,
        "measured_at": datetime.now(UTC).isoformat(timespec="seconds"),
        "hardware": _hardware(),
        "library": {
            "tesseract_robotics": version("tesseract-robotics-nanobind"),
            "upstream": {s.repository: s.sha for s in UPSTREAM_SOURCES},
            "git_commit": _command_output(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"]),
            "git_dirty": bool(status),
        },
        "problem": {
            "robot": ROBOT,
            "obstacle": {"radius_m": OBSTACLE_RADIUS_M, "xyz_m": OBSTACLE_XYZ_M},
            "joint_start": JOINT_START,
            "joint_end": JOINT_END,
        },
        "settings": {
            "seeds": list(SEEDS),
            "n": len(SEEDS),
            "ompl_planners": OMPL_PLANNERS,
            "ompl_planning_time_s": OMPL_PLANNING_TIME_S,
            "ompl_first_solution": True,
            "densify_step_rad": DENSIFY_STEP_RAD,
            "pipelines": {label: pipeline for label, pipeline, _, _ in PIPELINES},
        },
        "rows": rows,
    }
    DATA_PATH.parent.mkdir(parents=True, exist_ok=True)
    DATA_PATH.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    return DATA_PATH


# --------------------------------------------------------------------------- drawing


@dataclass(frozen=True)
class Measurement:
    """The committed measurement, validated."""

    rows: list[dict[str, object]]
    hardware: dict[str, object]
    n: int


def compute() -> Measurement:
    """Load and validate the committed measurement (never measures)."""
    if not DATA_PATH.exists():
        raise MeasurementDataError(
            f"{DATA_PATH.relative_to(REPO_ROOT)} is missing: run `pixi run docs-figures-measure planner_comparison`"
        )
    data = json.loads(DATA_PATH.read_text())
    if data.get("format") != DATA_FORMAT:
        raise MeasurementDataError(f"format {data.get('format')!r}, expected {DATA_FORMAT}")
    labels = {label for label, _, _, _ in PIPELINES}
    for row in data["rows"]:
        if row["planner"] not in labels:
            raise MeasurementDataError(f"unknown planner {row['planner']!r}")
        if row["success"] and any(key not in row for key, _, _ in METRICS):
            raise MeasurementDataError(f"successful row without every metric: {row}")
    return Measurement(rows=data["rows"], hardware=data["hardware"], n=data["settings"]["n"])


@dataclass(frozen=True)
class Strip:
    """One planner's values of one metric, in display units."""

    values: NDArray[np.float64]
    failed: int
    total: int


def strip(data: Measurement, label: str, key: str, scale: float) -> Strip:
    """Successful seeds' values of `key` for planner `label`, times `scale`."""
    rows = [r for r in data.rows if r["planner"] == label]
    values = np.array([r[key] for r in rows if r["success"]], dtype=np.float64) * scale
    return Strip(values=values, failed=len(rows) - len(values), total=len(rows))


def _strip_panel(ax, data: Measurement, key: str, title: str, scale: float, theme: Theme) -> None:
    rng = np.random.default_rng(0)  # jitter of the strip plot only; fixed for reproducibility
    for y, (label, _, _, color) in enumerate(PIPELINES):
        s = strip(data, label, key, scale)
        if len(s.values):
            ax.plot(
                s.values,
                y + rng.uniform(-0.2, 0.2, len(s.values)),
                "o",
                ms=3.0,
                color=color,
                alpha=0.7,
                mew=0,
            )
            median = float(np.median(s.values))
            ax.plot([median, median], [y - 0.32, y + 0.32], color=theme.ink, lw=1.6)
        if s.failed:
            ax.text(
                0.98,
                y,
                f"{s.failed}/{s.total} failed",
                transform=ax.get_yaxis_transform(),
                ha="right",
                va="center",
                fontsize=7.5,
                color=theme.muted,
            )
    ax.set_title(title, fontsize=8.5, loc="left", fontweight="normal")
    ax.grid(axis="y", visible=False)
    ax.tick_params(axis="x", labelsize=7.5)
    ax.set_ylim(len(PIPELINES) - 0.5, -0.5)
    ax.set_yticks(range(len(PIPELINES)), [label for label, _, _, _ in PIPELINES])


def draw(data: Measurement, theme: Theme) -> Figure:
    """Path quality (top) and time (bottom): every successful seed, and the median."""
    fig = plt.figure(figsize=(COLUMN_WIDTH_IN, 4.4))
    grid = fig.add_gridspec(
        2, 6, left=0.15, right=0.985, bottom=0.14, top=0.94, hspace=0.45, wspace=0.5
    )
    panels = [
        (fig.add_subplot(grid[0, 2 * i : 2 * i + 2]), metric) for i, metric in enumerate(QUALITY)
    ]
    panels += [
        (fig.add_subplot(grid[1, 3 * i : 3 * i + 3]), metric) for i, metric in enumerate(TIMING)
    ]
    for ax, (key, title, scale) in panels:
        _strip_panel(ax, data, key, title, scale, theme)
    for ax, _ in panels:
        if ax not in (panels[0][0], panels[3][0]):
            ax.tick_params(axis="y", labelleft=False)
    timing_ax = panels[3][0]
    timing_ax.set_xscale("log")
    timing_ax.xaxis.set_major_formatter(ScalarFormatter())
    timing_ax.xaxis.set_minor_formatter(NullFormatter())
    timing_ax.set_xticks([2, 5, 10, 20, 50, 100])
    hw = data.hardware
    load = float(np.asarray(hw["load_average_1_5_15"])[0])
    fig.text(
        0.0,
        0.005,
        f"N = {data.n} seeds, one process each; bar: median. {hw['cpu']}, "
        f"{hw['logical_cpus']} logical CPUs, 1-min load {load:.0f} at the start of the run.",
        fontsize=7.0,
        color=theme.muted,
        ha="left",
        va="bottom",
    )
    return fig


def inputs() -> list[Path]:
    """The committed measurement: the figure draws nothing else."""
    return [DATA_PATH]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--child", type=int, required=True, help="plan every pipeline for this seed"
    )
    print(json.dumps(_child(parser.parse_args().child)))
