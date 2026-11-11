"""Smoke check of an installed tesseract_robotics: `tesseract_nanobind_selftest`.

Needs neither the test suite nor pytest. It imports every module of the installed package, loads
a robot from the bundled support data, solves forward and inverse kinematics, checks collisions
and runs a short plan, then writes `tesseract_selftest_report.txt` (environment, per-check
result, and a GitHub issue template when a check failed).

Usage:
    tesseract_nanobind_selftest
"""

from __future__ import annotations

import importlib
import os
import pkgutil
import platform
import sys
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Final

import numpy as np

import tesseract_robotics

# Subpackages that need optional dependencies or only hold runnable examples.
SKIPPED_PACKAGES: Final = frozenset({"examples", "viewer", "selftest"})

# Joint configuration of the ABB IRB 2400 the kinematics check starts from (rad): away from the
# singularities at the zero pose, inside every joint limit.
FK_JOINTS: Final = (0.1, -0.2, 0.3, 0.1, 0.2, 0.1)

# Largest tip-position error (m) and rotation error (rad) of the IK solution refolded through FK.
# The solver is analytic (OPW), so these only absorb floating point.
IK_ROUNDTRIP_POSITION_TOL_M: Final = 1e-6
IK_ROUNDTRIP_ROTATION_TOL_RAD: Final = 1e-6

# Edge of the box dropped on the tool in the collision check (m).
OBSTACLE_EDGE_M: Final = 0.2

# Target of the short plan: one joint moved by 0.3 rad from the zero pose.
PLAN_GOAL: Final = (0.3, 0.0, 0.0, 0.0, 0.0, 0.0)

REPORT_NAME: Final = "tesseract_selftest_report.txt"
ENVIRONMENT_VARIABLES: Final = (
    "TESSERACT_TASK_COMPOSER_CONFIG_FILE",
    "TESSERACT_PYTHON_DLL_PATH",
    "DYLD_LIBRARY_PATH",
    "LD_LIBRARY_PATH",
)


class SelftestCheckError(AssertionError):
    """A selftest check ran and found the installation wrong."""


@dataclass(frozen=True)
class CheckResult:
    """Outcome of one check."""

    name: str
    passed: bool
    seconds: float
    detail: str


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SelftestCheckError(message)


def check_imports() -> str:
    """Import every module of the installed package."""
    imported = 0
    for info in pkgutil.walk_packages(tesseract_robotics.__path__, "tesseract_robotics."):
        top = info.name.split(".")[1]
        if top in SKIPPED_PACKAGES:
            continue
        importlib.import_module(info.name)
        imported += 1
    _require(imported > 0, "no module of tesseract_robotics was found")
    return f"{imported} modules"


def _load_robot():
    from tesseract_robotics.common import GeneralResourceLocator
    from tesseract_robotics.planning import Robot

    locator = GeneralResourceLocator(paths=[tesseract_robotics.get_tesseract_resource_path()])
    return Robot.from_tesseract_support("abb_irb2400", locator)


def check_robot_loads() -> str:
    """Load a robot from the bundled support data."""
    robot = _load_robot()
    joints = robot.get_joint_names("manipulator")
    _require(len(joints) == len(FK_JOINTS), f"expected {len(FK_JOINTS)} joints, got {joints}")
    return f"{len(robot.get_link_names())} links"


def check_kinematics() -> str:
    """Forward kinematics, then inverse kinematics back to the same tool pose."""
    robot = _load_robot()
    q = np.array(FK_JOINTS)
    pose = robot.fk("manipulator", q, "tool0")
    solution = robot.ik("manipulator", pose, "tool0", seed=q)
    again = robot.fk("manipulator", solution, "tool0")
    position_error = float(np.linalg.norm(np.asarray(again.translation - pose.translation)))
    relative = np.asarray(pose.linear).T @ np.asarray(again.linear)
    rotation_error = float(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
    _require(
        position_error <= IK_ROUNDTRIP_POSITION_TOL_M,
        f"IK round trip moved the tool by {position_error:.2e} m",
    )
    _require(
        rotation_error <= IK_ROUNDTRIP_ROTATION_TOL_RAD,
        f"IK round trip rotated the tool by {rotation_error:.2e} rad",
    )
    return f"round trip {position_error:.1e} m, {rotation_error:.1e} rad"


def check_collision() -> str:
    """A free trajectory is free; a box on the tool collides."""
    from tesseract_robotics.collision import CollisionCheckConfig, CollisionEvaluatorType
    from tesseract_robotics.environment import checkTrajectory
    from tesseract_robotics.planning import Pose, box, create_obstacle

    robot = _load_robot()
    q = np.array(FK_JOINTS)
    trajectory = np.stack([q, q])
    config = CollisionCheckConfig()
    config.type = CollisionEvaluatorType.DISCRETE

    def collides() -> bool:
        results, _ = checkTrajectory(
            robot.env.getDiscreteContactManager(),
            robot.env.getJointGroup("manipulator"),
            trajectory,
            config,
        )
        return bool(results)

    _require(not collides(), "the free configuration reports a collision")
    tool = robot.fk("manipulator", q, "tool0")
    create_obstacle(
        robot,
        "selftest_box",
        box(OBSTACLE_EDGE_M, OBSTACLE_EDGE_M, OBSTACLE_EDGE_M),
        Pose(tool),
        parent_link="base_link",
    )
    _require(collides(), "a box on the tool reports no collision")
    return "free state clear, box on tool detected"


def check_plan() -> str:
    """Plan a short joint-space motion."""
    from tesseract_robotics.planning import JointTarget, MotionProgram, TaskComposer

    robot = _load_robot()
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
        .set_joint_names(robot.get_joint_names("manipulator"))
        .move_to(JointTarget(np.zeros(len(FK_JOINTS))))
        .move_to(JointTarget(list(PLAN_GOAL)))
    )
    result = TaskComposer.from_config().plan_freespace(robot, program)
    _require(result.successful, f"planning failed: {result.message}")
    _require(len(result) > 0, "planning succeeded with an empty trajectory")
    return f"{len(result)} waypoints"


CHECKS: Final[tuple[tuple[str, Callable[[], str]], ...]] = (
    ("import every module", check_imports),
    ("load a robot", check_robot_loads),
    ("forward and inverse kinematics", check_kinematics),
    ("collision check", check_collision),
    ("short plan", check_plan),
)


def run_check(name: str, check: Callable[[], str]) -> CheckResult:
    """Run one check; any exception is its failure, reported with its traceback."""
    start = time.monotonic()
    try:
        detail = check()
    except Exception:  # noqa: BLE001 - a selftest reports every failure instead of stopping at one
        return CheckResult(name, False, time.monotonic() - start, traceback.format_exc())
    return CheckResult(name, True, time.monotonic() - start, detail)


def environment_info() -> dict[str, str]:
    """The settings the package reads, and where it found its data."""
    info = {key: os.environ.get(key, "not set") for key in ENVIRONMENT_VARIABLES}
    for name, resolve in (
        ("support data", tesseract_robotics.get_tesseract_resource_path),
        ("task composer config", tesseract_robotics.get_task_composer_config_path),
    ):
        try:
            info[name] = str(resolve())
        except FileNotFoundError as e:
            info[name] = f"not found ({e})"
    return info


def build_report(results: list[CheckResult], info: dict[str, str]) -> str:
    """The report text: header, environment, results, and an issue template on failure."""
    py = sys.version_info
    header = [
        "TESSERACT ROBOTICS SELFTEST REPORT",
        f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"Version: {tesseract_robotics.__version__}",
        f"Python: {py.major}.{py.minor}.{py.micro}",
        f"Platform: {platform.platform()}",
        f"NumPy: {np.__version__}",
        "",
        "ENVIRONMENT",
        *(f"  {key}: {value}" for key, value in info.items()),
        "",
        "RESULTS",
    ]
    body = [
        f"  {'PASS' if r.passed else 'FAIL'}  {r.name} ({r.seconds:.1f}s)"
        + (f": {r.detail}" if r.passed else "")
        for r in results
    ]
    failures = [r for r in results if not r.passed]
    lines = [*header, *body]
    if failures:
        lines += ["", "FAILURES"]
        for r in failures:
            lines += [f"--- {r.name}", r.detail.rstrip()]
        lines += [
            "",
            "GITHUB ISSUE TEMPLATE (copy below this line)",
            "## Selftest Failure Report",
            "",
            f"- tesseract_robotics version: {tesseract_robotics.__version__}",
            f"- Python: {py.major}.{py.minor}.{py.micro}",
            f"- Platform: {platform.platform()}",
            "",
            "Failed checks: " + ", ".join(r.name for r in failures),
            "",
            "```",
            *(r.detail.rstrip().splitlines()[-1] for r in failures),
            "```",
        ]
    return "\n".join(lines) + "\n"


def main() -> int:
    """Run every check, write the report, and return the exit code (1 if any failed)."""
    print(f"tesseract_robotics selftest v{tesseract_robotics.__version__}")
    results = []
    for name, check in CHECKS:
        print(f"  [{name}] ", end="", flush=True)
        result = run_check(name, check)
        print(f"{'PASS' if result.passed else 'FAIL'} ({result.seconds:.1f}s)")
        results.append(result)
    report_path = Path.cwd() / REPORT_NAME
    report_path.write_text(build_report(results, environment_info()))
    failed = [r for r in results if not r.passed]
    print(f"\nReport saved to: {report_path}")
    print(f"{len(results) - len(failed)} passed, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
