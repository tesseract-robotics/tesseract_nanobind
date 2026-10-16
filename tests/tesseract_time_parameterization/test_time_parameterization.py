"""Tests for tesseract_time_parameterization bindings.

Tests cover:
- TimeOptimalTrajectoryGeneration (TOTG)
- IterativeSplineParameterization (ISP)
- InstructionsTrajectory container
- Profile-based time parameterization (0.33 API)
"""

import gc
import subprocess
import sys

import numpy as np
import pytest

from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_time_parameterization import (
    ConstantTCPSpeedCompositeProfile,
    ConstantTCPSpeedParameterization,
    InstructionsTrajectory,
    ISPCompositeProfile,
    ISPMoveProfile,
    IterativeSplineParameterization,
    RuckigTrajectorySmoothing,
    RuckigTrajectorySmoothingCompositeProfile,
    ScalingsLengthError,
    StaleTrajectoryError,
    TimeOptimalTrajectoryGeneration,
    TimeParameterization,
    TOTGCompositeProfile,
    rescaleTimings,
)


class TestTimeOptimalTrajectoryGeneration:
    """Test TOTG time parameterization."""

    def test_default_constructor(self):
        # 0.33 API: constructor takes name
        totg = TimeOptimalTrajectoryGeneration()
        assert totg is not None
        assert totg.getName() == "TOTG"

    def test_custom_name(self):
        # 0.33 API: constructor takes name parameter
        totg = TimeOptimalTrajectoryGeneration("MyTOTG")
        assert totg is not None
        assert totg.getName() == "MyTOTG"


class TestIterativeSplineParameterization:
    """Test ISP time parameterization."""

    def test_default_constructor(self):
        # 0.33 API: constructor takes name
        isp = IterativeSplineParameterization()
        assert isp is not None
        assert isp.getName() == "ISP"

    def test_custom_name(self):
        # 0.33 API: constructor takes name parameter
        isp = IterativeSplineParameterization("MyISP")
        assert isp is not None
        assert isp.getName() == "MyISP"


class TestTOTGCompositeProfile:
    """Test TOTGCompositeProfile."""

    def test_default_constructor(self):
        profile = TOTGCompositeProfile()
        assert profile is not None
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0
        assert profile.path_tolerance == 0.1
        assert profile.min_angle_change == 0.001

    def test_custom_constructor(self):
        profile = TOTGCompositeProfile(0.5, 0.5, 0.05, 0.002)
        assert profile.max_velocity_scaling_factor == 0.5
        assert profile.max_acceleration_scaling_factor == 0.5
        assert profile.path_tolerance == 0.05
        assert profile.min_angle_change == 0.002

    def test_override_limits(self):
        profile = TOTGCompositeProfile()
        profile.override_limits = True
        profile.velocity_limits = np.column_stack((-np.ones(6), np.ones(6)))
        profile.acceleration_limits = np.column_stack((-np.ones(6) * 2, np.ones(6) * 2))
        assert profile.override_limits is True
        assert profile.velocity_limits.shape == (6, 2)
        assert profile.acceleration_limits.shape == (6, 2)


class TestISPCompositeProfile:
    """Test ISPCompositeProfile."""

    def test_default_constructor(self):
        profile = ISPCompositeProfile()
        assert profile is not None
        assert profile.add_points is True
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0

    def test_custom_constructor(self):
        profile = ISPCompositeProfile(0.5, 0.5)
        assert profile.max_velocity_scaling_factor == 0.5
        assert profile.max_acceleration_scaling_factor == 0.5

    def test_add_points_setting(self):
        profile = ISPCompositeProfile()
        profile.add_points = False
        assert profile.add_points is False


class TestISPMoveProfile:
    """Test ISPMoveProfile."""

    def test_default_constructor(self):
        profile = ISPMoveProfile()
        assert profile is not None
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0


class TestInstructionsTrajectory:
    """Test InstructionsTrajectory container."""

    def test_create_from_composite(self):
        """Test creating InstructionsTrajectory from CompositeInstruction."""
        program = CompositeInstruction("DEFAULT")
        manip = ManipulatorInfo()
        manip.manipulator = "manipulator"
        program.setManipulatorInfo(manip)

        # Add joint waypoints
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        positions = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
            np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        ]

        for pos in positions:
            wp = StateWaypoint(joint_names, pos)
            mi = MoveInstruction(
                StateWaypointPoly_wrap_StateWaypoint(wp),
                MoveInstructionType_FREESPACE,
                "DEFAULT",
            )
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        assert len(program) == 3

        # Create trajectory container
        trajectory = InstructionsTrajectory(program)
        assert trajectory.size() == 3
        assert trajectory.dof() == 6
        assert not trajectory.empty()


class TestConstantTCPSpeedParameterization:
    def test_default_constructor(self):
        tp = ConstantTCPSpeedParameterization()
        assert tp is not None
        assert tp.getName() == "ConstantTCPSpeed"

    def test_custom_name(self):
        tp = ConstantTCPSpeedParameterization("MyTCP")
        assert tp.getName() == "MyTCP"

    def test_is_time_parameterization(self):
        tp = ConstantTCPSpeedParameterization()
        assert isinstance(tp, TimeParameterization)


class TestConstantTCPSpeedCompositeProfile:
    def test_default_construction(self):
        p = ConstantTCPSpeedCompositeProfile()
        assert p is not None
        # Defaults (from C++ header)
        assert p.max_translational_velocity == pytest.approx(1.0)
        assert p.max_rotational_velocity == pytest.approx(1.0)
        assert p.max_translational_acceleration == pytest.approx(1.0)
        assert p.max_rotational_acceleration == pytest.approx(1.0)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_full_constructor(self):
        p = ConstantTCPSpeedCompositeProfile(
            max_translational_velocity=0.5,
            max_rotational_velocity=0.8,
            max_translational_acceleration=1.5,
            max_rotational_acceleration=2.0,
            max_velocity_scaling_factor=0.75,
            max_acceleration_scaling_factor=0.6,
        )
        assert p.max_translational_velocity == pytest.approx(0.5)
        assert p.max_rotational_velocity == pytest.approx(0.8)
        assert p.max_translational_acceleration == pytest.approx(1.5)
        assert p.max_rotational_acceleration == pytest.approx(2.0)
        assert p.max_velocity_scaling_factor == pytest.approx(0.75)
        assert p.max_acceleration_scaling_factor == pytest.approx(0.6)

    def test_constructor_scaling_defaults(self):
        # scaling factors default to 1.0 when omitted
        p = ConstantTCPSpeedCompositeProfile(0.2, 0.3, 0.4, 0.5)
        assert p.max_translational_velocity == pytest.approx(0.2)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_member_assignment(self):
        p = ConstantTCPSpeedCompositeProfile()
        p.max_translational_velocity = 0.123
        p.max_velocity_scaling_factor = 0.5
        assert p.max_translational_velocity == pytest.approx(0.123)
        assert p.max_velocity_scaling_factor == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Helpers: programs on the ABB IRB2400 joints
# ---------------------------------------------------------------------------

ABB_JOINTS = [f"joint_{i}" for i in range(1, 7)]

# Relative slack on Ruckig's jerk bound when it is checked by finite differences of the
# waypoint accelerations: Ruckig meets the limit exactly, so this only absorbs float rounding.
JERK_RELATIVE_TOLERANCE = 1e-6


def _move(position, time=None):
    swp = StateWaypoint(ABB_JOINTS, np.asarray(position, dtype=np.float64))
    if time is not None:
        swp.setTime(time)
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE, "DEFAULT"
        )
    )


def _manipulator_info():
    manip = ManipulatorInfo()
    manip.manipulator = "manipulator"
    manip.working_frame = "base_link"
    manip.tcp_frame = "tool0"
    return manip


def _abb_program(num=10, span=1.0):
    """A flat FREESPACE program on the IRB2400 joints, joint_1 swept over `span` rad."""
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_manipulator_info())
    for i in range(num + 1):
        p = np.zeros(6)
        p[0] = i * span / num
        p[1] = 0.5 * i * span / num
        program.appendMoveInstruction(_move(p))
    return program


@pytest.fixture(scope="module")
def abb_env():
    return Robot.from_tesseract_support("abb_irb2400").env


def _times(program):
    traj = InstructionsTrajectory(program)
    return np.array([traj.getTimeFromStart(i) for i in range(traj.size())])


def _max_jerk(program):
    """Largest joint jerk, by finite differences of the waypoint accelerations."""
    traj = InstructionsTrajectory(program)
    acc = np.array([traj.getAcceleration(i) for i in range(traj.size())])
    return (np.abs(np.diff(acc, axis=0)) / np.diff(_times(program))[:, None]).max()


# ---------------------------------------------------------------------------
# #277: the stub names Environment, not the C++ type
# ---------------------------------------------------------------------------


def test_compute_signature_names_python_environment():
    assert "tesseract::" not in TimeParameterization.compute.__doc__


# ---------------------------------------------------------------------------
# #281: InstructionsTrajectory keeps its program alive
# ---------------------------------------------------------------------------


def test_instructions_trajectory_keeps_program_alive():
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_manipulator_info())
    program.appendMoveInstruction(_move(np.zeros(6)))
    refs = sys.getrefcount(program)
    traj = InstructionsTrajectory(program)
    assert sys.getrefcount(program) == refs + 1
    del traj
    gc.collect()
    assert sys.getrefcount(program) == refs


# Every bound method of InstructionsTrajectory, with arguments for a 6-dof program.
_TRAJECTORY_CALLS = {
    "getPosition": (0,),
    "getVelocity": (0,),
    "getAcceleration": (0,),
    "getTimeFromStart": (0,),
    "setData": (0, np.zeros(6), np.zeros(6), 1.0),
    "size": (),
    "dof": (),
    "empty": (),
    "isTimeStrictlyIncreasing": (),
}


@pytest.mark.parametrize("method", sorted(_TRAJECTORY_CALLS))
def test_instructions_trajectory_raises_after_program_append(method):
    program = _abb_program()
    traj = InstructionsTrajectory(program)
    # 5 appends: without the guard the read returned stale values, with no error
    for i in range(5):
        program.appendMoveInstruction(_move(np.full(6, 2.0 + i)))
    with pytest.raises(StaleTrajectoryError, match="build a new InstructionsTrajectory"):
        getattr(traj, method)(*_TRAJECTORY_CALLS[method])


def test_instructions_trajectory_raises_after_program_clear():
    program = _abb_program()
    traj = InstructionsTrajectory(program)
    program.clear()
    with pytest.raises(StaleTrajectoryError):
        traj.getPosition(0)
    assert issubclass(StaleTrajectoryError, RuntimeError)


@pytest.mark.parametrize("appends", [50, 20000])
def test_instructions_trajectory_stale_read_does_not_crash(appends):
    # Without the guard these read freed memory: a RuntimeError at 50, SIGSEGV at 20000.
    # Subprocess so a crash fails this test instead of killing the worker.
    code = f"""
import numpy as np
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction, MoveInstruction, MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE, StateWaypoint, StateWaypointPoly_wrap_StateWaypoint,
)
from tesseract_robotics.tesseract_time_parameterization import (
    InstructionsTrajectory, StaleTrajectoryError,
)

def move(x):
    p = np.zeros(6)
    p[0] = x
    swp = StateWaypointPoly_wrap_StateWaypoint(StateWaypoint([f"joint_{{i}}" for i in range(1, 7)], p))
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(swp, MoveInstructionType_FREESPACE, "DEFAULT")
    )

program = CompositeInstruction("DEFAULT")
for i in range(11):
    program.appendMoveInstruction(move(0.2 * i))
traj = InstructionsTrajectory(program)
for i in range({appends}):
    program.appendMoveInstruction(move(10.0 + i))
try:
    traj.getPosition(0)
except StaleTrajectoryError:
    print("stale")
"""
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == "stale"


def test_instructions_trajectory_unmutated_program_reads_and_writes_through():
    program = _abb_program()
    traj = InstructionsTrajectory(program)
    np.testing.assert_allclose(traj.getPosition(3), [0.3, 0.15, 0.0, 0.0, 0.0, 0.0])
    velocity = np.full(6, 0.25)
    traj.setData(3, velocity, np.full(6, 0.5), 7.0)
    # Same instructions, same addresses: no error, the written values are read back
    np.testing.assert_array_equal(traj.getVelocity(3), velocity)
    assert traj.getTimeFromStart(3) == 7.0
    # setData wrote into the user's program, not a copy
    waypoint = WaypointPoly_as_StateWaypointPoly(program[3].asMoveInstruction().getWaypoint())
    assert waypoint.getTime() == 7.0
    assert InstructionsTrajectory(program).getTimeFromStart(3) == 7.0


def test_instructions_trajectory_rebuilt_after_append_works():
    program = _abb_program()
    InstructionsTrajectory(program)
    program.appendMoveInstruction(_move(np.full(6, 2.0)))
    traj = InstructionsTrajectory(program)
    assert traj.size() == 12
    np.testing.assert_array_equal(traj.getPosition(11), np.full(6, 2.0))


# ---------------------------------------------------------------------------
# #280: ISPCompositeProfile.minimum_time_delta
# ---------------------------------------------------------------------------


def test_isp_composite_profile_minimum_time_delta():
    profile = ISPCompositeProfile()
    assert profile.minimum_time_delta == sys.float_info.epsilon
    profile.minimum_time_delta = 1e-3
    assert profile.minimum_time_delta == 1e-3


# ---------------------------------------------------------------------------
# #279: rescaleTimings
# ---------------------------------------------------------------------------


def _two_segment_program():
    """Two timed sub-composites: segment A at t = 0, 1, 2; segment B at t = 3, 4, 5."""
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_manipulator_info())
    for times in ([0.0, 1.0, 2.0], [3.0, 4.0, 5.0]):
        segment = CompositeInstruction("DEFAULT")
        segment.setManipulatorInfo(_manipulator_info())
        for t in times:
            segment.appendMoveInstruction(_move(np.full(6, 0.1 * t), time=t))
        program.push_back(segment)
    return program


def test_rescale_timings_halves_speed_of_second_segment():
    program = _two_segment_program()
    before = _times(program)
    rescaleTimings(program, [1.0, 0.5])
    after = _times(program)
    np.testing.assert_allclose(after[:3], before[:3])
    np.testing.assert_allclose(np.diff(after[3:]), 2.0 * np.diff(before[3:]))


def test_rescale_timings_wrong_length_raises():
    program = _two_segment_program()
    with pytest.raises(ScalingsLengthError, match="scalings has 1 elements, program has 2"):
        rescaleTimings(program, [1.0])
    assert issubclass(ScalingsLengthError, ValueError)


# ---------------------------------------------------------------------------
# #278: RuckigTrajectorySmoothing
# ---------------------------------------------------------------------------


class TestRuckigTrajectorySmoothing:
    def test_profile_defaults(self):
        profile = RuckigTrajectorySmoothingCompositeProfile()
        assert profile.duration_extension_fraction == 1.1
        assert profile.max_duration_extension_factor == 10
        assert profile.override_limits is False

    def test_profile_constructor(self):
        profile = RuckigTrajectorySmoothingCompositeProfile(1.5, 4.0)
        assert profile.duration_extension_fraction == 1.5
        assert profile.max_duration_extension_factor == 4.0

    def test_profile_limits_round_trip(self):
        profile = RuckigTrajectorySmoothingCompositeProfile()
        for name, bound in (
            ("velocity_limits", 1.0),
            ("acceleration_limits", 2.0),
            ("jerk_limits", 30.0),
        ):
            limits = np.column_stack((-np.full(6, bound), np.full(6, bound)))
            setattr(profile, name, limits)
            np.testing.assert_array_equal(getattr(profile, name), limits)

    def test_is_time_parameterization(self):
        smoothing = RuckigTrajectorySmoothing()
        assert isinstance(smoothing, TimeParameterization)
        assert smoothing.getName() == "RuckigTrajectorySmoothing"

    def test_compute_respects_jerk_limit(self, abb_env):
        program = _abb_program()
        profiles = ProfileDictionary()
        profiles.addProfile("TOTG", "DEFAULT", TOTGCompositeProfile())
        assert TimeOptimalTrajectoryGeneration().compute(program, abb_env, profiles)
        # TOTG limits velocity and acceleration only: its output exceeds the jerk limit below,
        # so the bound checked at the end is Ruckig's doing.
        jerk_max = 5.0
        assert _max_jerk(program) > jerk_max

        ruckig_profile = RuckigTrajectorySmoothingCompositeProfile()
        ruckig_profile.override_limits = True
        ruckig_profile.velocity_limits = np.column_stack((-np.full(6, 2.0), np.full(6, 2.0)))
        ruckig_profile.acceleration_limits = np.column_stack((-np.full(6, 4.0), np.full(6, 4.0)))
        ruckig_profile.jerk_limits = np.column_stack((-np.full(6, jerk_max), np.full(6, jerk_max)))
        profiles.addProfile("RuckigTrajectorySmoothing", "DEFAULT", ruckig_profile)
        assert RuckigTrajectorySmoothing().compute(program, abb_env, profiles)

        assert _max_jerk(program) <= jerk_max * (1.0 + JERK_RELATIVE_TOLERANCE)


# ---------------------------------------------------------------------------
# #261: value equality on the profiles; #273: createKey()
# ---------------------------------------------------------------------------

# (class, field, value that differs from the default)
PROFILE_FIELD_CHANGES = [
    (TOTGCompositeProfile, "path_tolerance", 0.05),
    (ISPCompositeProfile, "add_points", False),
    (ISPMoveProfile, "max_velocity_scaling_factor", 0.5),
    (ConstantTCPSpeedCompositeProfile, "max_translational_velocity", 0.5),
    (RuckigTrajectorySmoothingCompositeProfile, "duration_extension_fraction", 1.5),
]


@pytest.mark.parametrize(
    ("cls", "field", "value"), PROFILE_FIELD_CHANGES, ids=lambda x: getattr(x, "__name__", None)
)
def test_profile_value_equality(cls, field, value):
    a, b = cls(), cls()
    assert a == b
    assert not (a != b)
    setattr(b, field, value)
    assert a != b
    assert not (a == b)
    with pytest.raises(TypeError):
        hash(a)


@pytest.mark.parametrize("cls", [c for c, _, _ in PROFILE_FIELD_CHANGES], ids=lambda c: c.__name__)
def test_profile_create_key_matches_instance_key(cls):
    assert cls.createKey() == cls().getKey()


def test_profile_create_keys_distinct():
    keys = {c.createKey() for c, _, _ in PROFILE_FIELD_CHANGES}
    assert len(keys) == len(PROFILE_FIELD_CHANGES)
