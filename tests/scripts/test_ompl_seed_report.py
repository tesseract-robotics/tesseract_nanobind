"""scripts/ompl_seed_report.py: the per-seed OMPL / TrajOptIfopt report of the clearance example (#362)."""

import enum
import json
import re
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest
import yaml

from tesseract_robotics import get_task_composer_config_path
from tesseract_robotics.trajopt_sqp import SQPParameters

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "scripts"
sys.path.insert(0, str(SCRIPTS))

import ompl_seed_report as report  # noqa: E402

example = report.example

# The remap check plans the threshold case with one planner, which reproduces on a seed. With the
# example's two, 1 of 42 runs differed from its seed's usual path; the planners' race is the
# inferred cause, and 36 repeats did not reproduce it. The threshold ends the optimize loop, so
# the time cap never decides the plan; this cap only bounds a failed run (s).
REMAP_CHECK_SEED = 1
REMAP_CHECK_PLANNERS = 1
REMAP_CHECK_PLANNING_TIME_S = 30.0

# s: OMPL's cap on this child (30) + 3 x the worst whole-child pass seen (32.6, a first cold child
# on a loaded macOS machine), 127.8 rounded up. The `--seed` child fits: 2 x 10 s caps + the same
# 97.8.
CHILD_HANG_DUMP_S = 130
# s: the child dumps its stacks and exits this long before the parent would kill it, so a real hang
# reports where it stuck (test_task_composer_lifetime.py).
HANG_DUMP_MARGIN_S = 30
CHILD_TIMEOUT_S = CHILD_HANG_DUMP_S + HANG_DUMP_MARGIN_S
# The first line `faulthandler.dump_traceback_later` writes (faulthandler.c: "Timeout (H:MM:SS)!").
_HANG_DUMP_HEADER = re.compile(r"^Timeout \(\d+:\d{2}:\d{2}(\.\d+)?\)!$", re.MULTILINE)

# rad: how far TrajOptIfopt's SQP may leave a joint row, its default `cnt_tolerance`. Every path ends
# inside it; the largest endpoint residual over 12 seeds was 7.8e-6.
ENDPOINT_TOLERANCE_RAD = SQPParameters().cnt_tolerance

# The parsed variant config against stock's. `search_paths` is per install: stock's is resolved at
# import, the variant's is the file's. In the two plugins it copies, the variant renames the task
# and moves OMPL's output and TrajOptIfopt's input to its own key. Nothing else may differ.
_HEADER_KEYS_NOT_COMPARED = {"search_paths", "tasks"}
_VARIANT_PLUGIN_OF_STOCK = {
    "FreespaceIfoptTask": "FreespaceIfoptOMPLOutputTask",
    "FreespaceIfoptPipeline": report.VARIANT_PIPELINE,
}
_NODES = ("config", "nodes")
_ALLOWED_PLUGIN_DIFFERENCES = {
    (*_NODES, "OMPLMotionPlannerTask", "config", "outputs", "program"): (
        "output_data",
        report.OMPL_OUTPUT_KEY,
    ),
    (*_NODES, "TrajOptIfoptMotionPlannerTask", "config", "inputs", "program"): (
        "output_data",
        report.OMPL_OUTPUT_KEY,
    ),
    (*_NODES, "MotionPlanningTask", "task"): (
        "FreespaceIfoptTask",
        "FreespaceIfoptOMPLOutputTask",
    ),
}
_ABSENT = "<absent>"

_REMAP_CHILD = textwrap.dedent(
    """
    import importlib, json, sys
    sys.path.insert(0, {scripts!r})
    import ompl_seed_report as report
    from tesseract_robotics.planning import TaskComposer
    from tesseract_robotics.motion_planners_ompl import RNG_setSeed

    ex = importlib.import_module("tesseract_robotics.examples.freespace_clearance_objective_example")
    variant = sys.argv[1] == "variant"
    composer = (
        TaskComposer.from_config(report.VARIANT_CONFIG) if variant else TaskComposer.from_config()
    )
    pipeline = report.VARIANT_PIPELINE if variant else ex.PIPELINE
    composer.warmup([pipeline])
    RNG_setSeed({seed})
    case = ex.run_case(
        "threshold",
        ex.LengthAndClearanceProfile(ex.PATH_LENGTH_THRESHOLD_RAD),
        {planning_time},
        composer=composer,
        pipeline=pipeline,
        num_planners={planners},
    )
    final = [
        {{
            "positions": [float(v).hex() for v in point.positions],
            "velocities": None if point.velocities is None else [float(v).hex() for v in point.velocities],
            "accelerations": None
            if point.accelerations is None
            else [float(v).hex() for v in point.accelerations],
            "time": None if point.time is None else float(point.time).hex(),
        }}
        for point in case["result"]
    ]
    try:
        ompl_states = len(report.program_positions(report.ompl_program(case["result"])))
    except report.OMPLProgramMissingError:
        ompl_states = None
    print(json.dumps({{"final": final, "ompl_states": ompl_states}}))
    """
)


class HangRoute(enum.Enum):
    """How a hung child ended."""

    DUMPED = "dumped its stacks and exited"
    KILLED = "killed by the parent, no exit of its own"


class ChildHungError(RuntimeError):
    """A child outlived its bound; `stdout` and `stderr` hold what it printed until then."""

    def __init__(self, arm: str, route: HangRoute, stdout: str, stderr: str) -> None:
        super().__init__(
            f"{arm} child hung: {route.value}.\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        )
        self.arm = arm
        self.route = route
        self.stdout = stdout
        self.stderr = stderr


class ChildFailedError(RuntimeError):
    """A child exited non-zero on its own, without a hang dump."""

    def __init__(self, arm: str, returncode: int, stdout: str, stderr: str) -> None:
        super().__init__(
            f"{arm} child exited {returncode}.\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        )
        self.arm = arm
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def _run_child(
    arm: str,
    code: str,
    *args: str,
    hang_dump_s: float = CHILD_HANG_DUMP_S,
    timeout_s: float = CHILD_TIMEOUT_S,
) -> dict | list:
    """Run `code` in a fresh process, bounded, and return its last stdout line as JSON.

    Args:
        arm: Names the child in a failure.
        code: The child's source, run after arming its hang dump.
        *args: The child's `sys.argv[1:]`.
        hang_dump_s: When the child dumps its stacks and exits (s).
        timeout_s: When the parent kills it (s).

    Raises:
        ChildHungError: The child hit either bound.
        ChildFailedError: The child exited non-zero with no hang dump.
    """
    armed = f"import faulthandler\nfaulthandler.dump_traceback_later({hang_dump_s}, exit=True)\n"
    try:
        proc = subprocess.run(
            # -u: what the child printed survives its dump's exit and the parent's kill.
            [sys.executable, "-u", "-c", armed + code, *args],
            capture_output=True,
            encoding="utf-8",
            errors="replace",
            check=False,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as expired:
        raise ChildHungError(
            arm,
            HangRoute.KILLED,
            report.captured_text(expired.stdout),
            report.captured_text(expired.stderr),
        ) from expired
    if proc.returncode != 0:
        if _HANG_DUMP_HEADER.search(proc.stderr):
            raise ChildHungError(arm, HangRoute.DUMPED, proc.stdout, proc.stderr)
        raise ChildFailedError(arm, proc.returncode, proc.stdout, proc.stderr)
    return json.loads(proc.stdout.strip().splitlines()[-1])


def _remap_child(arm: str) -> dict:
    code = _REMAP_CHILD.format(
        scripts=str(SCRIPTS),
        seed=REMAP_CHECK_SEED,
        planning_time=REMAP_CHECK_PLANNING_TIME_S,
        planners=REMAP_CHECK_PLANNERS,
    )
    return _run_child(arm, code, arm)


@pytest.fixture(scope="module")
def stock_arm() -> dict:
    """One stock child, shared so a hang costs its bound once on a worker."""
    return _remap_child("stock")


@pytest.fixture(scope="module")
def variant_arm(stock_arm: dict) -> dict:
    """One variant child, after stock: a hang here clears the arms before it."""
    return _remap_child("variant")


def _load_composer_config(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["task_composer_plugins"]


def _leaf_differences(stock, variant, path: tuple = ()) -> dict[tuple, tuple]:
    """Every leaf where two parsed YAML trees differ, as {path: (stock, variant)}."""
    if isinstance(stock, dict) and isinstance(variant, dict):
        differences: dict[tuple, tuple] = {}
        for key in stock.keys() | variant.keys():
            differences |= _leaf_differences(
                stock.get(key, _ABSENT), variant.get(key, _ABSENT), (*path, key)
            )
        return differences
    return {} if stock == variant else {path: (stock, variant)}


def test_ompl_output_variant_config_differs_from_stock_only_in_the_hand_off():
    """Parsed against the installed stock config: plans nothing and needs no RNG."""
    stock = _load_composer_config(get_task_composer_config_path())
    variant = _load_composer_config(report.VARIANT_CONFIG)

    stock_plugins = stock["tasks"]["plugins"]
    variant_plugins = variant["tasks"]["plugins"]
    assert set(variant_plugins) == set(_VARIANT_PLUGIN_OF_STOCK.values())

    differences: dict[tuple, tuple] = {}
    for stock_name, variant_name in _VARIANT_PLUGIN_OF_STOCK.items():
        differences |= _leaf_differences(stock_plugins[stock_name], variant_plugins[variant_name])
    assert differences == _ALLOWED_PLUGIN_DIFFERENCES

    header = _leaf_differences(
        {k: v for k, v in stock.items() if k not in _HEADER_KEYS_NOT_COMPARED},
        {k: v for k, v in variant.items() if k not in _HEADER_KEYS_NOT_COMPARED},
    )
    assert header == {}
    assert set(stock) == set(variant)


@pytest.mark.planning
def test_ompl_output_variant_keeps_the_ompl_program_and_stock_does_not(stock_arm, variant_arm):
    assert stock_arm["ompl_states"] is None
    # TrajOptIfopt optimizes the OMPL states one for one, so the counts match.
    assert variant_arm["ompl_states"] == len(variant_arm["final"])


def _endpoints_rad(arm: dict) -> np.ndarray:
    final = arm["final"]
    return np.array([[float.fromhex(v) for v in final[index]["positions"]] for index in (0, -1)])


@pytest.mark.planning
def test_ompl_output_variant_is_bit_identical_to_stock(stock_arm, variant_arm):
    """The variant pipeline only re-keys the OMPL -> TrajOptIfopt hand-off: same final program.

    Each arm is a fresh process seeded before it plans. Both paths first end where the example
    asks, so a broken plan fails here rather than as a program mismatch.
    """
    for arm in (stock_arm, variant_arm):
        np.testing.assert_allclose(
            _endpoints_rad(arm),
            [example.JOINT_START, example.JOINT_END],
            atol=ENDPOINT_TOLERANCE_RAD,
            rtol=0.0,
        )

    # Stock runs twice as the control: a difference there would be a reproducibility problem,
    # not the remap's.
    stock_again = _remap_child("stock")
    assert stock_arm["final"] == stock_again["final"], "control: stock does not reproduce"
    assert variant_arm["final"] == stock_arm["final"]


CANNED_ARM = "canned"
CANNED_STDOUT = "printed before the hang"
CANNED_STDERR = "failed in the canned child"
CANNED_EXIT_CODE = 3
# s: longer than any bound below, so only the bound can end the child.
CANNED_SLEEP_S = 600
# s: counted from when the child arms the dump, after its interpreter start; it then prints and
# sleeps, so this leaves it all the time it needs to print. Below `CANNED_TIMEOUT_S`, so the
# dump, not the kill, ends the dump case.
CANNED_HANG_DUMP_S = 2
# s: from spawn, so it must cover the child's interpreter start before its print. Measured here:
# 0.02 to 0.03 s warm at load 54 on 10 cores, 0.25 to 0.5 s cold. A cold start on a loaded CI
# runner is unmeasured, so 60 x the cold figure: 30. Only a hang case waits this long.
CANNED_TIMEOUT_S = 30
_CANNED_HANG = f"import time\nprint({CANNED_STDOUT!r})\ntime.sleep({CANNED_SLEEP_S})\n"


@pytest.mark.parametrize(
    ("hang_dump_s", "timeout_s", "route"),
    [
        pytest.param(CANNED_HANG_DUMP_S, CANNED_TIMEOUT_S, HangRoute.DUMPED, id="dumps its stack"),
        pytest.param(CANNED_SLEEP_S, CANNED_TIMEOUT_S, HangRoute.KILLED, id="parent kills it"),
    ],
)
def test_a_hung_child_raises_with_what_it_printed(hang_dump_s, timeout_s, route):
    """Plans nothing. The print is unflushed, so this also proves `-u` on either route."""
    with pytest.raises(ChildHungError) as hung:
        _run_child(CANNED_ARM, _CANNED_HANG, hang_dump_s=hang_dump_s, timeout_s=timeout_s)

    assert hung.value.arm == CANNED_ARM
    assert hung.value.route is route
    assert CANNED_STDOUT in hung.value.stdout
    assert CANNED_STDOUT in str(hung.value)
    if route is HangRoute.DUMPED:
        assert "most recent call first" in hung.value.stderr
        assert "most recent call first" in str(hung.value)
    else:
        assert hung.value.stderr == ""


def test_a_child_exiting_non_zero_raises_with_its_stderr():
    code = f"import sys\nsys.stderr.write({CANNED_STDERR!r})\nsys.exit({CANNED_EXIT_CODE})\n"

    with pytest.raises(ChildFailedError) as failed:
        _run_child(CANNED_ARM, code)

    assert failed.value.arm == CANNED_ARM
    assert failed.value.returncode == CANNED_EXIT_CODE
    assert CANNED_STDERR in failed.value.stderr
    assert CANNED_STDERR in str(failed.value)


def test_a_child_returns_its_last_line_as_json():
    code = "print('a plugin chatters')\nprint('{\"row\": 1}')\n"
    assert _run_child(CANNED_ARM, code) == {"row": 1}


def _row(seed: int, case: str) -> report.SeedRow:
    return report.SeedRow(
        seed=seed,
        case=case,
        ompl_wall_time_s=1.0,
        ompl_min_clearance_m=0.03,
        trajopt_iterations=5,
        trajopt_status="Found valid solution",
        final_min_clearance_m=0.04,
        final_length_rad=3.5,
        total_wall_time_s=2.0,
    )


def test_to_markdown_has_one_row_per_seed_and_case():
    rows = [_row(seed, case) for seed in (1, 2) for case in ("default", "threshold")]

    table = report.to_markdown(rows, commit="abc1234", platform="Darwin arm64")

    lines = table.splitlines()
    data_rows = [line for line in lines if line.startswith("| ") and line[2].isdigit()]
    assert len(data_rows) == 4
    header = "\n".join(line for line in lines if not line.startswith("|"))
    assert "N = 2" in header
    assert "abc1234" in header
    assert "Darwin arm64" in header


@pytest.mark.planning
def test_run_seed_child_emits_rows():
    code = f"import runpy\nrunpy.run_path({str(SCRIPTS / 'ompl_seed_report.py')!r}, run_name='__main__')\n"
    rows = _run_child("run_seed", code, "--seed", "1", "--json")

    assert len(rows) == 2
    assert {row["case"] for row in rows} == {"default", "threshold"}
    for row in rows:
        assert row["trajopt_iterations"] > 0
        assert row["final_min_clearance_m"] > 0
        assert row["ompl_min_clearance_m"] > 0
        assert row["ompl_wall_time_s"] > 0


def _seed_child(monkeypatch, tmp_path, source):
    """Make `report.report` run `source` as its child, a real process that plans nothing."""
    script = tmp_path / "child.py"
    script.write_text(source, encoding="utf-8")
    monkeypatch.setattr(report, "__file__", str(script))


def test_a_hung_seed_child_raises_with_what_it_printed(monkeypatch, tmp_path):
    _seed_child(
        monkeypatch,
        tmp_path,
        "import sys, time\n"
        f"print({CANNED_STDOUT!r})\n"
        f"sys.stderr.write({CANNED_STDERR!r})\n"
        f"time.sleep({CANNED_SLEEP_S})\n",
    )
    with pytest.raises(report.SeedHungError) as hung:
        report.report([2], timeout_s=CANNED_TIMEOUT_S)
    assert isinstance(hung.value, report.SeedRunError)
    assert CANNED_STDOUT in hung.value.stdout
    assert CANNED_STDERR in hung.value.stderr
    assert CANNED_STDOUT in str(hung.value)
    assert CANNED_STDERR in str(hung.value)
    assert "seed 2" in str(hung.value)


def test_a_failed_seed_child_raises_with_its_stderr(monkeypatch, tmp_path):
    _seed_child(
        monkeypatch,
        tmp_path,
        f"import sys\nsys.stderr.write({CANNED_STDERR!r})\nsys.exit({CANNED_EXIT_CODE})\n",
    )
    with pytest.raises(report.SeedRunError) as failed:
        report.report([2])
    assert not isinstance(failed.value, report.SeedHungError)
    assert CANNED_STDERR in str(failed.value)


@pytest.mark.parametrize(
    ("stream", "text"),
    [
        pytest.param(None, "", id="nothing read"),
        pytest.param(b"row", "row", id="bytes, the parent read the pipes"),
        pytest.param("row", "row", id="str, Windows collects them after the kill"),
    ],
)
def test_captured_text(stream, text):
    assert report.captured_text(stream) == text


CANNED_HEAD = "abc1234"
CANNED_DIRTY = " M scripts/ompl_seed_report.py"
GIT_REV_PARSE = "rev-parse"
GIT_STATUS = "status"


def _canned_git(tmp_path, *, hung: str | None = None, status: str = "") -> tuple[str, ...]:
    """A real process standing in for `git`; the `hung` subcommand never returns."""
    script = tmp_path / "canned_git.py"
    script.write_text(
        "import sys, time\n"
        f"if sys.argv[1] == {hung!r}:\n"
        f"    print({CANNED_STDOUT!r})\n"
        f"    sys.stderr.write({CANNED_STDERR!r})\n"
        f"    time.sleep({CANNED_SLEEP_S})\n"
        f"print({CANNED_HEAD!r} if sys.argv[1] == {GIT_REV_PARSE!r} else {status!r})\n",
        encoding="utf-8",
    )
    return (sys.executable, "-u", str(script))


@pytest.mark.parametrize("hung", [GIT_REV_PARSE, GIT_STATUS])
def test_a_hung_git_raises_with_what_it_printed(tmp_path, hung):
    """Each of the two calls has its own bound: neither call may run unbounded."""
    with pytest.raises(report.GitHungError) as hung_git:
        report._commit(git=_canned_git(tmp_path, hung=hung), timeout_s=CANNED_TIMEOUT_S)

    assert hung in hung_git.value.command
    assert hung in str(hung_git.value)
    assert CANNED_STDOUT in hung_git.value.stdout
    assert CANNED_STDERR in hung_git.value.stderr
    assert CANNED_STDOUT in str(hung_git.value)
    assert CANNED_STDERR in str(hung_git.value)


@pytest.mark.parametrize(
    ("status", "commit"),
    [
        pytest.param("", CANNED_HEAD, id="clean"),
        pytest.param(CANNED_DIRTY, f"{CANNED_HEAD}+dirty", id="dirty"),
    ],
)
def test_commit_names_head_and_whether_the_tree_is_dirty(tmp_path, status, commit):
    assert report._commit(git=_canned_git(tmp_path, status=status)) == commit
