"""scripts/ompl_first_path_probe.py: the row checks, flags and child failures of the probe (#362).

No test here plans: the rows are synthetic and timing is never asserted. The probe's own run
(`pixi run ompl-first-path-probe`) is the evidence on real paths.
"""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

import ompl_first_path_probe as probe  # noqa: E402

LIMIT = probe.example.PATH_LENGTH_THRESHOLD_RAD
MARGIN = probe.example.OMPL_CONTACT_MARGIN_M
# rad: a length apart from the threshold, on either side of it
LENGTH_OFFSET = 0.05
SHORT = LIMIT - LENGTH_OFFSET
LONG = LIMIT + LENGTH_OFFSET
STATES = 300
CHILD_EXIT_CODE = 3
CHILD_STDERR = "planning failed in the child"


def _row(mode, length=SHORT, states=STATES, margin=MARGIN):
    return {
        "seed": 1,
        "mode": mode,
        "ompl_s": 0.1,
        "ompl_len": length,
        "ompl_states": states,
        "margin": margin,
    }


def _threshold(**fields):
    return _row(probe.THRESHOLD_MODE, **fields)


def _first(**fields):
    return _row(probe.FIRST_PATH_MODE, **fields)


@pytest.mark.parametrize(
    ("threshold", "first"),
    [
        pytest.param(_threshold(), _first(), id="first path meets the threshold and comes back"),
        pytest.param(_threshold(length=LIMIT), _first(length=LIMIT), id="exactly at the threshold"),
        pytest.param(_threshold(), _first(length=LONG), id="first path too long, shorter one kept"),
    ],
)
def test_check_pair_passes(threshold, first):
    probe.check_pair(threshold, first)


def test_check_pair_refuses_a_threshold_path_over_the_limit():
    with pytest.raises(probe.ThresholdNotMetError, match="longer than"):
        probe.check_pair(_threshold(length=LONG), _first(length=LONG))


@pytest.mark.parametrize(
    "threshold",
    [
        pytest.param(_threshold(length=SHORT - LENGTH_OFFSET), id="other length"),
        pytest.param(_threshold(states=STATES + 1), id="other state count"),
    ],
)
def test_check_pair_refuses_another_path_when_the_first_met_the_threshold(threshold):
    with pytest.raises(probe.FirstPathMismatchError):
        probe.check_pair(threshold, _first())


@pytest.mark.parametrize(
    ("threshold", "first"),
    [
        pytest.param(_threshold(margin=None), _first(), id="threshold row"),
        pytest.param(_threshold(), _first(margin=None), id="first-path row"),
    ],
)
def test_check_pair_refuses_a_row_without_a_margin(threshold, first):
    with pytest.raises(probe.MarginMissingError):
        probe.check_pair(threshold, first)


def test_check_pair_refuses_a_row_lacking_the_margin_field():
    first = _first()
    del first["margin"]
    with pytest.raises(probe.MarginMissingError):
        probe.check_pair(_threshold(), first)


@pytest.mark.parametrize(
    ("threshold", "first"),
    [
        pytest.param(_threshold(margin=2 * MARGIN), _first(), id="threshold row"),
        pytest.param(_threshold(), _first(margin=0.0), id="first-path row"),
    ],
)
def test_check_pair_refuses_a_margin_other_than_the_examples(threshold, first):
    with pytest.raises(probe.MarginMismatchError):
        probe.check_pair(threshold, first)


def test_check_pair_reads_the_margin_before_the_paths():
    """A path planned at another margin says nothing about the rules, whatever its length."""
    with pytest.raises(probe.MarginMismatchError):
        probe.check_pair(_threshold(length=LONG, margin=0.0), _first())


@pytest.mark.parametrize(
    "argv",
    [
        pytest.param(["--seed", "1"], id="seed only"),
        pytest.param(["--mode", probe.FIRST_PATH_MODE], id="mode only"),
    ],
)
def test_seed_and_mode_are_given_together(monkeypatch, capsys, argv):
    def planned(*_):
        pytest.fail("a bad flag combination must stop before planning")

    monkeypatch.setattr(probe, "probe", planned)
    monkeypatch.setattr(probe, "probe_in_subprocess", planned)
    with pytest.raises(SystemExit) as exit_info:
        probe.main(argv)
    assert exit_info.value.code == 2  # argparse's exit code for a usage error
    assert "--seed and --mode" in capsys.readouterr().err


def test_seed_and_mode_plan_that_row_only(monkeypatch, capsys):
    monkeypatch.setattr(probe, "probe", lambda seed, mode: {"seed": seed, "mode": mode})
    probe.main(["--seed", "4", "--mode", probe.THRESHOLD_MODE])
    assert json.loads(capsys.readouterr().out) == {"seed": 4, "mode": probe.THRESHOLD_MODE}


def test_no_flags_sweep_every_seed_in_both_modes(monkeypatch, capsys):
    monkeypatch.setattr(
        probe, "probe_in_subprocess", lambda seed, mode: {**_row(mode), "seed": seed}
    )
    probe.main([])
    rows = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert [(row["seed"], row["mode"]) for row in rows] == [
        (seed, mode)
        for seed in probe.PROBE_SEEDS
        for mode in (probe.THRESHOLD_MODE, probe.FIRST_PATH_MODE)
    ]


def _child(monkeypatch, tmp_path, source):
    """Make `probe_in_subprocess` run `source` as its child, a real process that plans nothing."""
    script = tmp_path / "child.py"
    script.write_text(source, encoding="utf-8")
    monkeypatch.setattr(probe, "__file__", str(script))


def test_a_failed_child_raises_with_its_stderr(monkeypatch, tmp_path):
    _child(
        monkeypatch,
        tmp_path,
        f"import sys\nsys.stderr.write({CHILD_STDERR!r})\nsys.exit({CHILD_EXIT_CODE})\n",
    )
    with pytest.raises(probe.ProbeRunError) as error:
        probe.probe_in_subprocess(2, probe.FIRST_PATH_MODE)
    message = str(error.value)
    assert CHILD_STDERR in message
    assert str(CHILD_EXIT_CODE) in message
    assert "seed 2" in message
    assert probe.FIRST_PATH_MODE in message


def test_a_child_returns_its_last_line_as_the_row(monkeypatch, tmp_path):
    _child(monkeypatch, tmp_path, "print('a plugin chatters')\nprint('{\"seed\": 2}')\n")
    assert probe.probe_in_subprocess(2, probe.THRESHOLD_MODE) == {"seed": 2}
