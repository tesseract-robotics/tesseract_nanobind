"""Contract tests of `tests/examples/overload_matcher.py`.

The matcher decides which nanobind overload an example call exercised. It must
be conservative: two overloads that both accept a call make it ambiguous, an
annotation it does not model makes it ambiguous, and only a unique, provable
acceptor counts. The effect tests at the end pin the matcher against nanobind's
real dispatch: each calls a multi-overload binding and checks that the overload
whose observable effect happened is the one the matcher names.
"""

import re
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    JointState,
    JointTrajectory,
    VectorVector3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tests.examples.overload_matcher import (
    IMPLICIT_CONVERSION_TARGETS,
    Match,
    ParamKind,
    SignatureParseError,
    match_call,
    parse_signature,
)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
ISO = "tesseract_robotics.tesseract_common._tesseract_common.Isometry3d"
SET_STATE = [s[0] for s in Environment.__dict__["setState"].__nb_signature__]
TRAJ_INIT = [s[0] for s in JointTrajectory.__dict__["__init__"].__nb_signature__]


def _sig(params: str) -> str:
    return f"def f({params}) -> None"


def test_parse_marks_positional_only_keyword_only_and_defaults():
    params = parse_signature("def f(self, a: int, /, b: float = \\0, *, c: str) -> None")
    assert [(p.name, p.kind, p.has_default) for p in params] == [
        ("self", ParamKind.POSITIONAL_ONLY, False),
        ("a", ParamKind.POSITIONAL_ONLY, False),
        ("b", ParamKind.POSITIONAL_OR_KEYWORD, True),
        ("c", ParamKind.KEYWORD_ONLY, False),
    ]
    assert params[0].annotation is None


def test_parse_keeps_bracketed_commas_in_one_annotation():
    (p,) = parse_signature(_sig("x: numpy.ndarray[dtype=float64, shape=(*, 2), order='F']"))
    assert p.annotation == "numpy.ndarray[dtype=float64, shape=(*, 2), order='F']"


def test_parse_rejects_non_signature():
    with pytest.raises(SignatureParseError):
        parse_signature("setState(self)")


def test_empty_mapping_between_competing_mappings_is_ambiguous():
    """`setState({})` loads into both the joint-value and the floating-joint mapping."""
    m = match_call(SET_STATE, (object(), {}), {})
    assert m == Match(None, (0, 2), False)
    assert m.ambiguous


def test_joint_value_mapping_picks_the_first_overload():
    assert match_call(SET_STATE, (object(), {"j": 0.1}), {}).index == 0


def test_floating_joint_mapping_picks_the_floating_overload():
    assert match_call(SET_STATE, (object(), {"j": Isometry3d.Identity()}), {}).index == 2


def test_unbound_defaulted_parameter_is_not_fully_bound():
    """Full binding: an old-style call must not demonstrate the new `floating_joints` parameter."""
    assert match_call(SET_STATE, (object(), {"j": 0.1}), {}).fully_bound is False
    assert match_call(SET_STATE, (object(), {"j": 0.1}, {}), {}).fully_bound is True
    assert (
        match_call(SET_STATE, (object(), {"j": 0.1}), {"floating_joints": {}}).fully_bound is True
    )


def test_list_for_ndarray_resolves_in_the_converting_pass():
    m = match_call(SET_STATE, (object(), ["j"], [0.1]), {})
    assert m.index == 1


def test_exact_int_and_float_decide_in_the_first_pass():
    sigs = [_sig("x: float"), _sig("x: int")]
    assert match_call(sigs, (1.0,), {}).index == 0
    assert match_call(sigs, (1,), {}).index == 1


def test_numpy_float_is_not_exact_float():
    """`load_f64` takes only an exact float without conversion; numpy.float64 converts."""
    assert match_call([_sig("x: float"), _sig("x: str")], (np.float64(1.0),), {}).index == 0
    m = match_call([_sig("x: float"), _sig("x: int")], (np.float64(1.0),), {})
    assert m.index == 0  # int rejects every float, even when converting


def test_bool_is_not_an_exact_int():
    assert match_call([_sig("x: int"), _sig("x: bool")], (True,), {}).index == 1


def test_sequence_rejects_a_string():
    sigs = [_sig("names: collections.abc.Sequence[str]"), _sig("name: str")]
    assert match_call(sigs, ("abc",), {}).index == 1


def test_bound_opaque_sequence_loads_into_a_sequence_parameter():
    """`seq_get` takes anything `PySequence_Check` accepts, e.g. a bound `VectorVector3d`.

    Found by the baseline run: `Mesh(VectorVector3d, faces)` succeeded while the
    matcher rejected every overload (geometry_showcase_example.py).
    """
    vertices = VectorVector3d()
    vertices.append(np.array([0.0, 0.0, 0.1]))
    sigs = [_sig("v: collections.abc.Sequence[numpy.ndarray[dtype=float64, shape=(3), order='C']]")]
    assert match_call(sigs, (vertices,), {}).index == 0
    assert match_call([_sig("v: collections.abc.Sequence[int]")], ({1: 2},), {}).index is None


def test_bound_type_is_checked_by_isinstance():
    sigs = [_sig(f"pose: {ISO}"), _sig("name: str")]
    assert match_call(sigs, (Isometry3d.Identity(),), {}).index == 0


def test_fortran_order_ndarray_needs_conversion_for_a_c_array():
    sigs = [_sig("m: numpy.ndarray[dtype=float64, shape=(4, 4), order='F']"), _sig(f"pose: {ISO}")]
    assert match_call(sigs, (np.asfortranarray(np.eye(4)),), {}).index == 0  # pass 1
    # pass 2: the C array converts; Isometry3d has no implicit conversion, so it rejects.
    assert match_call(sigs, (np.eye(4),), {}).index == 0


def test_implicit_conversion_target_stays_undecided_when_converting():
    """`InstructionPoly` accepts other instruction types in pass 2 (`nb::implicitly_convertible`)."""
    target = (
        "tesseract_robotics.tesseract_command_language._tesseract_command_language.InstructionPoly"
    )
    sigs = [_sig("x: float"), _sig(f"x: {target}")]
    m = match_call(sigs, (1,), {})
    assert m.index is None and m.candidates == (0, 1)


def test_implicit_conversion_targets_match_the_binding_sources():
    """The matcher's list of implicit-conversion targets is exactly what the bindings register."""
    sources = [p.read_text() for p in (REPO_ROOT / "src").rglob("*.cpp")]
    targets = {
        m.group(1).split("::")[-1]
        for text in sources
        for m in re.finditer(r"implicitly_convertible<\s*[\w:<>]+\s*,\s*([\w:]+)\s*>", text)
    }
    assert targets == IMPLICIT_CONVERSION_TARGETS
    assert not any("init_implicit" in text for text in sources)


def test_numpy_scalar_values_resolve_the_joint_value_mapping():
    """Found by the baseline run (online_planning_sqp_example.py): numpy.float64 joint values."""
    assert match_call(SET_STATE, (object(), {"j": np.float64(0.1)}), {}).index == 0


def test_unmodelled_annotation_is_ambiguous_not_a_guess():
    sigs = [_sig("x: some.module.Thing"), _sig("x: str")]
    m = match_call(sigs, ("abc",), {})
    assert m.ambiguous and m.candidates == (0, 1)


def test_variadic_arguments_are_ambiguous():
    m = match_call([_sig("*args"), _sig("x: int")], (1,), {})
    assert m.index is None


def test_call_no_overload_accepts_is_not_ambiguous():
    m = match_call([_sig("x: int")], ("abc",), {})
    assert m == Match(None, (), False) and not m.ambiguous


# --- the matcher against nanobind's real dispatch -------------------------------------

FLOATING_URDF = (
    '<robot name="floaty" xmlns:tesseract="http://ros.org/wiki/tesseract" '
    'tesseract:make_convex="true"><link name="base"/><link name="arm"/><link name="free"/>'
    '<joint name="j1" type="revolute"><parent link="base"/><child link="arm"/>'
    '<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
    '<joint name="jf" type="floating"><parent link="base"/><child link="free"/></joint>'
    "</robot>"
)


@pytest.fixture
def env():
    environment = Environment()
    assert environment.init(FLOATING_URDF, GeneralResourceLocator())
    return environment


def _set_state_effect(env, *args):
    """Run setState and report which overload's effect is visible."""
    before = env.getCurrentJointValues(["j1"])[0]
    env.setState(*args)
    moved_joint = env.getCurrentJointValues(["j1"])[0] != before
    free = env.getLinkTransform("free").translation
    return moved_joint, tuple(np.round(free, 6))


def test_effect_joint_mapping(env):
    args = ({"j1": 0.5},)
    assert match_call(SET_STATE, (env, *args), {}).index == 0
    assert _set_state_effect(env, *args) == (True, (0.0, 0.0, 0.0))


def test_effect_names_and_values(env):
    args = (["j1"], np.array([0.25]))
    assert match_call(SET_STATE, (env, *args), {}).index == 1
    assert _set_state_effect(env, *args) == (True, (0.0, 0.0, 0.0))


def test_effect_floating_mapping(env):
    pose = Isometry3d.Identity()
    pose.translate(np.array([1.25, 0.0, 0.0]))
    args = ({"jf": pose},)
    assert match_call(SET_STATE, (env, *args), {}).index == 2
    assert _set_state_effect(env, *args) == (False, (1.25, 0.0, 0.0))


def test_effect_trajectory_constructors():
    description_only = ("desc",)
    assert match_call(TRAJ_INIT, (object(), *description_only), {}).index == 0
    assert len(JointTrajectory(*description_only)) == 0

    states = ([JointState(["j1"], np.array([0.1]))], "desc")
    assert match_call(TRAJ_INIT, (object(), *states), {}).index == 1
    assert len(JointTrajectory(*states)) == 1
