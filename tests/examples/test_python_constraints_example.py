"""python_constraints_example: every act does what its caption says, and the loop steps alone."""

from __future__ import annotations

import importlib

import numpy as np
import pytest

example = importlib.import_module("tesseract_robotics.examples.python_constraints_example")

# What the solver leaves of an active bound, in the bound's own unit.
BOUND_ATOL = 1e-3
# The square cone of HoldAxis reaches sqrt(2) times its tolerance on the diagonal.
SQUARE_CONE = np.sqrt(2.0)


@pytest.fixture(scope="module")
def acts():
    results = example.run()
    assert results["success"]
    return results["acts"]


def _series(act, key):
    return np.array([frame[key] for frame in act.live])


def test_every_act_solves_without_violating_a_rule(acts):
    assert [act.act for act in acts] == list(example.ACTS)
    for act in acts:
        assert act.worst_violation < example.VIOLATION_TOLERANCE, act.act.caption
        assert len(act.rows) == round(act.act.seconds * example.FRAME_HZ)


def test_without_a_rule_the_tool_goes_with_the_rail(acts):
    assert _series(acts[0], "hold_mm").max() == pytest.approx(example.RAIL_SWING_M * 1e3, rel=0.01)


def test_world_hold_keeps_the_tip_on_the_mark_while_the_axis_wanders(acts):
    assert _series(acts[1], "hold_mm").max() < BOUND_ATOL
    assert _series(acts[1], "tilt_deg").max() > 10.0


def test_hold_axis_locks_the_tool_axis(acts):
    assert _series(acts[2], "hold_mm").max() < BOUND_ATOL
    assert _series(acts[2], "tilt_deg").max() <= 0.5 * SQUARE_CONE + BOUND_ATOL


def test_the_camera_loses_the_tip_until_watch_the_tool_turns_it(acts):
    # Staring ahead from the carriage, the camera sees the tip leave its cone as the rail moves.
    assert _series(acts[1], "view_deg").max() > 3.0 * example.VIEW_DEG
    watching = _series(acts[3], "view_deg")
    assert watching[-1] <= example.VIEW_DEG + BOUND_ATOL
    # From there on the rule stays on, through every later act.
    for act in acts[4:]:
        assert _series(act, "view_deg").max() <= example.VIEW_DEG + BOUND_ATOL
    # The camera did the turning: its pan moved, and the tip never left the mark.
    pan = np.array(
        [row[example.LiveLoop().joint_names.index("camera_pan_joint")] for row in acts[3].rows]
    )
    assert np.ptp(pan) > np.radians(10.0)
    assert _series(acts[3], "hold_mm").max() < BOUND_ATOL


@pytest.mark.parametrize(
    ("index", "key", "target"),
    [(4, "elbow_m", 1.25), (5, "dexterity", 1.25)],
)
def test_a_ramped_bound_takes_its_quantity_to_the_target(acts, index, key, target):
    series = _series(acts[index], key)
    # It starts on the wrong side of the target and ends on it: the rule did the moving.
    assert abs(series[0] - target) > 0.02 * abs(target)
    assert series[-1] == pytest.approx(target, abs=BOUND_ATOL)
    assert _series(acts[index], "hold_mm").max() < BOUND_ATOL


def test_the_forearm_keeps_its_clearance_while_the_ball_comes_in(acts):
    clearance = _series(acts[6], "clearance_m")
    assert clearance.min() >= example.CLEARANCE_MIN_M - BOUND_ATOL
    # The ball got close enough for the rule to bind, and the carriage got out of its way.
    assert clearance.min() < example.CLEARANCE_MIN_M + 0.01
    rail = np.array([row[0] for row in acts[6].rows])
    assert rail.max() - rail.min() > 0.5
    assert _series(acts[6], "hold_mm").max() < BOUND_ATOL


def test_all_seven_hold_while_the_tip_traces_the_circle(acts):
    last = acts[-1]
    assert _series(last, "hold_mm").max() < BOUND_ATOL
    assert _series(last, "tilt_deg").max() <= last.act.cone_deg + BOUND_ATOL
    assert _series(last, "elbow_m").min() >= last.act.elbow_m - BOUND_ATOL
    assert _series(last, "view_deg").max() <= last.act.view_deg + BOUND_ATOL
    assert _series(last, "dexterity").min() >= last.act.dexterity - BOUND_ATOL
    assert _series(last, "clearance_m").min() >= example.CLEARANCE_MIN_M - BOUND_ATOL


def test_a_row_names_every_recorded_joint_and_its_time():
    results = example.run(example.ACTS[:1])
    names = results["joint_names"]
    assert len(set(names)) == len(names)
    env_joints = {joint_id.name() for joint_id in results["robot"].env.getActiveJointIds()}
    assert set(names) <= env_joints
    rows = results["acts"][0].rows
    assert all(len(row) == len(names) + 1 for row in rows)
    assert rows[1][-1] == pytest.approx(1.0 / example.FRAME_HZ)


def test_the_loop_steps_from_any_inputs():
    loop = example.LiveLoop()
    # The rail has moved 0.3 m; nothing holds the tip.
    free = loop.step(
        example.Inputs(rail_m=0.3, ball_m=example.BALL_PARKED_M, mark=loop.target, hold=False)
    )
    assert free.live["hold_mm"] == pytest.approx(300.0, abs=0.1)
    # WorldHold brings it back, in one frame.
    held = loop.step(example.Inputs(rail_m=0.3, ball_m=example.BALL_PARKED_M, mark=loop.target))
    assert held.live["hold_mm"] < BOUND_ATOL
    assert held.violation < example.VIOLATION_TOLERANCE
    assert held.q[0] == pytest.approx(0.3)


def test_the_carriage_drives_to_abeam_of_the_tip_once_the_rail_is_the_solvers():
    loop = example.LiveLoop()
    parked = example.BALL_PARKED_M
    # Something else has driven the rail 0.6 m off; the arm reaches back to hold the tip.
    for _ in range(3):
        loop.step(example.Inputs(rail_m=0.6, ball_m=parked, mark=loop.target))
    assert loop.live["lead_m"] == pytest.approx(-0.6, abs=1e-3)
    # Handed the rail, with nothing asking for motion, the solver leaves the carriage there.
    still = loop.step(example.Inputs(rail_m=None, ball_m=parked, mark=loop.target))
    assert still.q[0] == pytest.approx(0.6, abs=1e-3)
    # CarriageFollows asks: the band closes in, and the carriage drives back under the tip.
    for band in np.linspace(0.6, 0.1, 30):
        frame = loop.step(
            example.Inputs(rail_m=None, ball_m=parked, mark=loop.target, follow_m=float(band))
        )
        assert frame.violation < example.VIOLATION_TOLERANCE
    assert abs(frame.q[0]) <= 0.1 + BOUND_ATOL
    assert frame.live["hold_mm"] < BOUND_ATOL
    # While something else drives the rail there is no carriage to command: the rule is off.
    measured = example.Inputs(rail_m=0.6, ball_m=parked, mark=loop.target, follow_m=0.1)
    assert loop.step(measured).violation < example.VIOLATION_TOLERANCE


def test_a_bound_marker_and_its_glyph_show_only_while_the_rule_is_on():
    loop = example.LiveLoop()
    names = list(example.INSTRUMENT_JOINTS)
    off = example.Inputs(rail_m=0.0, ball_m=example.BALL_PARKED_M, mark=loop.target, hold=False)
    on = example.Inputs(rail_m=None, ball_m=0.0, mark=loop.target, cone_deg=20.0, elbow_m=1.2)

    parked = dict(zip(names, example.instrument_values(off, loop.live, loop.target, caption=None)))
    shown = dict(zip(names, example.instrument_values(on, loop.live, loop.target, caption=3)))

    for key in ("hold_mm", "tilt_deg", "elbow_m", "view_deg", "dexterity", "clearance_m"):
        assert parked[f"{key}_bound_joint"] == example.PARKED_M
        assert 0.0 <= parked[f"{key}_value_joint"] <= example.RULER_M
    assert shown["hold_mm_bound_joint"] == 0.0
    assert shown["elbow_m_bound_joint"] == pytest.approx(example.GAUGES[2].offset_m(1.2))
    assert shown["plane_stage_joint"] == 1.2
    assert shown["cone_stage_joint"] == 0.0
    assert shown["view_stage_joint"] == example.PARKED_M
    assert parked["view_stage_joint"] == example.PARKED_M
    assert [shown[f"caption_{i}_joint"] for i in range(len(example.ACTS))].count(0.0) == 1
    assert shown["caption_3_joint"] == 0.0


def test_the_marks_slides_carry_it_to_where_the_frame_holds_the_tip():
    loop = example.LiveLoop()
    names = list(example.INSTRUMENT_JOINTS)
    travel = np.array([0.1, -0.2, 0.05])
    moved = example.Inputs(
        rail_m=0.0, ball_m=example.BALL_PARKED_M, mark=loop.target + travel, cone_deg=20.0
    )

    values = dict(
        zip(names, example.instrument_values(moved, loop.live, loop.target, caption=None))
    )

    slides = [values["mark_x_joint"], values["mark_y_joint"], values["mark_joint"]]
    assert slides == pytest.approx(travel)
    # The cone rides on the mark: shown, it needs no travel of its own.
    assert values["cone_stage_joint"] == 0.0


@pytest.mark.parametrize(
    ("key", "bound", "live", "expected"),
    [
        # An upper bound far below the live value comes in a frame's step at a time.
        ("view_deg", 3.0, 45.0, 45.0 - 60.0 / example.FRAME_HZ),
        ("lead_m", 0.1, -0.7, 0.7 - 0.5 / example.FRAME_HZ),
        # A lower bound far above it rises the same way.
        ("elbow_m", 1.3, 1.15, 1.15 + 0.1 / example.FRAME_HZ),
        # A bound the live value already meets is itself.
        ("view_deg", 3.0, 1.0, 3.0),
        ("dexterity", 0.8, 1.2, 0.8),
    ],
)
def test_under_sliders_a_bound_closes_in_from_the_live_value(key, bound, live, expected):
    assert example._closing(key, bound, {key: live}) == pytest.approx(expected)


def test_the_solve_gauge_shows_each_frames_time_against_the_frame():
    loop = example.LiveLoop()
    names = list(example.INSTRUMENT_JOINTS)
    gauge = next(g for g in example.GAUGES if g.key == "solve_ms")
    inputs = example.Inputs(rail_m=0.2, ball_m=example.BALL_PARKED_M, mark=loop.target)

    frame = loop.step(inputs)
    values = dict(
        zip(names, example.instrument_values(inputs, frame.live, loop.target, caption=None))
    )

    assert frame.live["solve_ms"] == frame.solve_ms > 0.0
    assert values["solve_ms_value_joint"] == pytest.approx(gauge.offset_m(frame.solve_ms))
    # The bar is the frame the loop has, 1/30 s: it is there whether or not a rule is on.
    assert values["solve_ms_bound_joint"] == pytest.approx(gauge.offset_m(1e3 / example.FRAME_HZ))
    assert 0.0 < values["solve_ms_bound_joint"] < example.RULER_M


def test_a_caption_counts_the_rules_that_are_on_and_says_what_a_frame_took():
    assert [example.rule_count(act) for act in example.ACTS] == [0, 1, 2, 3, 4, 4, 4, 7]
    off = example.Inputs(rail_m=0.0, ball_m=0.0, mark=np.zeros(3), hold=False)
    assert example.rule_count(off) == 0
    assert example.solve_line(1, 0.94, median=True) == "1 rule  ·  median 0.9 ms a frame"
    assert example.solve_line(7, 11.26) == "7 rules  ·  11.3 ms a frame"


def test_each_act_has_a_median_solve_for_its_caption(acts):
    medians = example.act_solve_ms({"acts": acts})
    assert len(medians) == len(example.ACTS)
    # No rule, no solve: the first act's frames take no time worth the name.
    assert medians[0] < min(medians[1:])
