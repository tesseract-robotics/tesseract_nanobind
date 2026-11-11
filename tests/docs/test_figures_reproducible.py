"""The docs figures reproduce from the repository, and their data functions are right.

`pixi run docs-figures --check` regenerates every figure into build/ and compares it with
`docs/assets/figures/figures.lock.json`; the measured figure renders from its committed JSON,
so the check never re-measures. What "reproduce" can promise depends on the platform:

- On the lock's render platform (same OS, CPU architecture, matplotlib, FreeType, numpy,
  trimesh) the output bytes must equal the committed SVGs and the lock's hashes.
- Elsewhere libm, BLAS and solver round-off may move a coordinate in the last digit an SVG
  keeps, so equal bytes are not a property of the code. There the check asserts what is: every
  input hashes as the lock says, and every figure regenerates into a well-formed SVG that draws.
  This is a different assertion for a different guarantee, not a skip.
"""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from scripts.docs_figures import lock
from scripts.docs_figures.__main__ import FIGURES
from scripts.docs_figures.collision_distance import (
    BOX_EDGE_M,
    SPHERE_RADIUS_M,
    analytic_distance,
)
from scripts.docs_figures.continuous_collision import substates
from scripts.docs_figures.planner_comparison import densify
from scripts.docs_figures.style import FIGURES_DIR, REPO_ROOT, THEMES
from scripts.docs_figures.time_parameterization import spread

# s: the whole regeneration takes ~13 s alone (one TaskComposer plan, the rest closed-form);
# 20 x that on a loaded runner.
CHECK_TIMEOUT_S = 300


def test_figures_reproduce():
    """Bit for bit on the render platform; elsewhere matching inputs and well-formed SVGs."""
    run = subprocess.run(
        [sys.executable, "-m", "scripts.docs_figures", "--check"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=CHECK_TIMEOUT_S,
    )
    assert run.returncode == 0, run.stderr[-4000:]


def test_lock_lists_exactly_the_committed_figures():
    """Every figure has both theme variants in the lock, and every locked output exists."""
    locked = json.loads(lock.LOCK_PATH.read_text())["figures"]
    assert sorted(locked) == sorted(FIGURES)
    expected = {
        (FIGURES_DIR / f"{name}{theme.suffix}.svg").relative_to(REPO_ROOT).as_posix()
        for name in FIGURES
        for theme in THEMES
    }
    outputs = {key for entry in locked.values() for key in entry["outputs"]}
    assert outputs == expected
    on_disk = {p.relative_to(REPO_ROOT).as_posix() for p in FIGURES_DIR.glob("*.svg")}
    assert on_disk == expected


@pytest.mark.parametrize(
    ("center", "expected"),
    [
        # Face-on gaps along x: centre minus half edge minus radius.
        ((0.30, 0.0, 0.0), 0.30 - BOX_EDGE_M / 2 - SPHERE_RADIUS_M),
        ((0.15, 0.0, 0.0), 0.15 - BOX_EDGE_M / 2 - SPHERE_RADIUS_M),
        # Off the corner in the z = 0 section: Euclidean distance to the edge, minus radius.
        ((0.4, 0.5, 0.0), np.hypot(0.3, 0.4) - SPHERE_RADIUS_M),
        # Centre inside the cube: depth to the nearest face, negative, minus radius.
        ((0.0, 0.0, 0.0), -BOX_EDGE_M / 2 - SPHERE_RADIUS_M),
    ],
)
def test_analytic_sphere_box_distance(center, expected):
    assert analytic_distance(np.array(center)) == pytest.approx(expected, abs=1e-15)


def test_substates_follow_upstream_interpolation():
    """ceil(|dq| / lvs) + 1 states per step, step ends shared, endpoints kept."""
    trajectory = np.array([[0.0, 0.0], [0.3, 0.4], [0.3, 0.4]])  # a 0.5 step, then a zero step
    states = substates(trajectory, 0.2)
    # ceil(0.5 / 0.2) + 1 = 4 states for the first step (3 after the shared start); the zero
    # step has ceil(0) + 1 = 1 state, its shared start, so it adds none.
    assert len(states) == 1 + 3
    np.testing.assert_allclose(states[0], [0.0, 0.0])
    np.testing.assert_allclose(states[3], [0.3, 0.4])
    steps = np.linalg.norm(np.diff(np.array(states[:4]), axis=0), axis=1)
    np.testing.assert_allclose(steps, 0.5 / 3)


def test_densify_keeps_vertices_and_bounds_the_step():
    path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.25]])
    dense = densify(path, 0.1)
    steps = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    assert steps.max() <= 0.1 + 1e-12
    assert steps.sum() == pytest.approx(1.25)
    for vertex in path:
        assert np.min(np.linalg.norm(dense - vertex, axis=1)) == 0.0


def test_spread_keeps_order_and_gap():
    values = np.array([0.5, 0.4, 0.1, 0.0, -0.3, -0.5])
    placed = spread(values, 0.17)
    assert list(np.argsort(placed)) == list(np.argsort(values))
    assert np.diff(np.sort(placed)).min() >= 0.17 - 1e-12
    assert placed.mean() == pytest.approx(values.mean())


def test_measured_data_names_its_hardware_and_build():
    """The committed benchmark JSON carries what a reader needs to judge the timings."""
    data = json.loads((FIGURES_DIR / "data" / "planner_comparison.json").read_text())
    for key in ("cpu", "logical_cpus", "os", "load_average_1_5_15"):
        assert key in data["hardware"]
    assert {"upstream", "git_commit", "tesseract_robotics"} <= set(data["library"])
    assert data["settings"]["n"] == len(data["settings"]["seeds"])
    assert Path(REPO_ROOT / "scripts/docs_figures/planner_comparison.py").exists()


def _committed_entries() -> dict[str, lock.FigureEntry]:
    figures = json.loads(lock.LOCK_PATH.read_text())["figures"]
    return {
        name: lock.FigureEntry(inputs=entry["inputs"], outputs=entry["outputs"])
        for name, entry in figures.items()
    }


def test_other_platform_checks_structure_not_bytes(monkeypatch):
    """Off the render platform, the committed SVGs pass as well-formed and bytes are not compared."""
    monkeypatch.setattr(lock, "render_platform", lambda: {"system": "elsewhere"})
    same_platform, problems = lock.compare(_committed_entries(), FIGURES_DIR)
    assert not same_platform
    assert problems == []


def test_other_platform_still_requires_matching_inputs(monkeypatch):
    """A changed input fails the check on any platform."""
    monkeypatch.setattr(lock, "render_platform", lambda: {"system": "elsewhere"})
    entries = _committed_entries()
    name, entry = next(iter(entries.items()))
    key = next(iter(entry.inputs))
    entries[name] = lock.FigureEntry(inputs={**entry.inputs, key: "0" * 64}, outputs=entry.outputs)
    _, problems = lock.compare(entries, FIGURES_DIR)
    assert problems == [f"{name}: input {key} changed since the lock"]


@pytest.mark.parametrize(
    ("content", "reason"),
    [
        ("<svg", "not well-formed XML"),
        ('<html xmlns="http://www.w3.org/1999/xhtml"/>', "root element"),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" viewBox="0 0 1 1"/>',
            "draws no path",
        ),
    ],
)
def test_svg_problems_flags_broken_output(content, reason):
    scratch = REPO_ROOT / "build" / "test-figures-svg"
    scratch.mkdir(parents=True, exist_ok=True)
    try:
        path = scratch / "broken.svg"
        path.write_text(content)
        problems = lock.svg_problems(path)
        assert any(reason in p for p in problems), problems
    finally:
        shutil.rmtree(scratch)


def test_committed_svgs_are_well_formed():
    for path in FIGURES_DIR.glob("*.svg"):
        assert lock.svg_problems(path) == []
