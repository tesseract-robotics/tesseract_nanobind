"""Kinematics utilities from utils.h and validate.h (gh-213), on the KUKA iiwa of test_kdl_kinematics."""

import sys

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import kinematics as tk
from tesseract_robotics import state_solver as tesseract_state_solver
from tesseract_robotics.common import Isometry3d

from . import test_kdl_kinematics as kdl
from . import test_opw_kinematics as opw

# [m/rad, rad/rad] numericalJacobian is a forward difference with delta = 1e-8
# (utils.cpp:44 @ 0.35.0). Truncation is <= 0.5 * max|d2f/dq2| * delta ~ 1e-8 for the iiwa's
# ~1.3 m reach; rounding is ~ eps * |f| / delta ~ 2.2e-16 * 1.3 / 1e-8 ~ 3e-8. 1e-6 is ~25x their sum.
NUMERICAL_JACOBIAN_ATOL = 1e-6
# [rad] the harmonizers round twice on sums up to 2*pi ((q + pi) then -+ pi): 2 ulp(2*pi), x2 headroom.
ANGLE_ATOL = 4 * np.spacing(2 * np.pi)
_REGULAR_Q = kdl._IIWA_Q
# All joints zero: the iiwa's four z axes line up, so J has rank < 6.
_SINGULAR_Q = np.zeros(7)


def _eig_atol(a):
    """Weyl bound for a backward-stable symmetric eigensolver: |dlambda| <= c * n * eps * ||A||_2."""
    return 16 * a.shape[0] * np.finfo(float).eps * np.linalg.norm(a, 2)


@pytest.fixture
def joint_group():
    jg, _ = kdl._joint_group(kdl._IIWA_JOINTS)
    return jg


@pytest.fixture
def fwd_kin():
    fwd, factory, locator = kdl._iiwa_fwd_kin()
    yield fwd
    del fwd, factory, locator


# --- numericalJacobian ---


def test_numerical_jacobian_joint_group_matches_analytic(joint_group):
    numeric = tk.numericalJacobian(
        Isometry3d.Identity(), joint_group, _REGULAR_Q, "tool0", np.zeros(3)
    )
    assert numeric.shape == (6, 7)
    nptest.assert_allclose(
        numeric, joint_group.calcJacobian(_REGULAR_Q, "tool0"), rtol=0, atol=NUMERICAL_JACOBIAN_ATOL
    )


def test_numerical_jacobian_forward_kinematics_matches_analytic(fwd_kin):
    numeric = tk.numericalJacobian(Isometry3d.Identity(), fwd_kin, _REGULAR_Q, "tool0", np.zeros(3))
    nptest.assert_allclose(
        numeric, fwd_kin.calcJacobian(_REGULAR_Q, "tool0"), rtol=0, atol=NUMERICAL_JACOBIAN_ATOL
    )


def test_numerical_jacobian_base_link_overload_matches_analytic(joint_group):
    """utils.cpp:106-133 @ 0.35.0 against calcJacobian(q, base_link_name, link_name) (gh-212)."""
    numeric = tk.numericalJacobian(
        joint_group, _REGULAR_Q, "link_3", Isometry3d.Identity(), "tool0", Isometry3d.Identity()
    )
    nptest.assert_allclose(
        numeric,
        joint_group.calcJacobian(_REGULAR_Q, "link_3", "tool0"),
        rtol=0,
        atol=NUMERICAL_JACOBIAN_ATOL,
    )


@pytest.mark.parametrize(
    "call",
    [
        lambda jg, fk: tk.numericalJacobian(
            Isometry3d.Identity(), jg, _REGULAR_Q, "no_such_link", np.zeros(3)
        ),
        lambda jg, fk: tk.numericalJacobian(
            Isometry3d.Identity(), fk, _REGULAR_Q, "no_such_link", np.zeros(3)
        ),
        lambda jg, fk: tk.numericalJacobian(
            jg, _REGULAR_Q, "no_such_link", Isometry3d.Identity(), "tool0", Isometry3d.Identity()
        ),
        lambda jg, fk: tk.numericalJacobian(
            jg, _REGULAR_Q, "link_3", Isometry3d.Identity(), "no_such_link", Isometry3d.Identity()
        ),
    ],
    ids=["joint_group", "forward_kinematics", "base_link", "link"],
)
def test_numerical_jacobian_unknown_link_raises_key_error(joint_group, fwd_kin, call):
    with pytest.raises(KeyError, match="no_such_link"):
        call(joint_group, fwd_kin)


@pytest.mark.parametrize(
    "call",
    [
        lambda jg, fk, q: tk.numericalJacobian(Isometry3d.Identity(), jg, q, "tool0", np.zeros(3)),
        lambda jg, fk, q: tk.numericalJacobian(Isometry3d.Identity(), fk, q, "tool0", np.zeros(3)),
        lambda jg, fk, q: tk.numericalJacobian(
            jg, q, "link_3", Isometry3d.Identity(), "tool0", Isometry3d.Identity()
        ),
    ],
    ids=["joint_group", "forward_kinematics", "base_link"],
)
def test_numerical_jacobian_wrong_length_raises_value_error(joint_group, fwd_kin, call):
    with pytest.raises(ValueError, match="6 joint values, expected 7"):
        call(joint_group, fwd_kin, _REGULAR_Q[:6])


# --- calcManipulability / isNearSingularity ---


# condition = (s_max / s_min)**2 rounds twice; the ratio of the reported eigenvalues s**2 rounds
# three times (each square, then the quotient). Five roundings of half an ulp: 2.5 eps, so 3 eps.
_CONDITION_RTOL = 3 * np.finfo(np.float64).eps


def test_calc_manipulability_full_rank(joint_group):
    J = joint_group.calcJacobian(_REGULAR_Q, "tool0")
    manip = tk.calcManipulability(J)
    a = J @ J.T
    expected = np.linalg.eigvalsh(a)
    nptest.assert_allclose(np.sort(manip.m.eigen_values), expected, rtol=0, atol=_eig_atol(a))
    # Upstream takes both from the SVD of J (tesseract#1396): condition is measure squared,
    # bit for bit; the eigenvalue ratio rounds differently, so it agrees only to rounding.
    assert manip.m.condition == manip.m.measure * manip.m.measure
    ratio = manip.m.eigen_values.max() / manip.m.eigen_values.min()
    assert manip.m.condition == pytest.approx(ratio, rel=_CONDITION_RTOL, abs=0)
    assert isinstance(manip.m_linear, tk.ManipulabilityEllipsoid)
    assert "measure=" in repr(manip.m)
    assert "m_linear=" in repr(manip)


def test_manipulability_default_constructors():
    """The structs' implicit default constructors (utils.h:132, :162): header defaults, empty values."""
    ellipsoid = tk.ManipulabilityEllipsoid()
    assert (ellipsoid.measure, ellipsoid.condition, ellipsoid.volume) == (0.0, 0.0, 0.0)
    assert ellipsoid.eigen_values.shape == (0,)
    assert tk.Manipulability().f_angular.measure == 0.0


def test_calc_manipulability_singular_is_max_double(joint_group):
    """utils.cpp:240-244 @ 0.35.0: a ~zero eigenvalue sets measure and condition to max double."""
    manip = tk.calcManipulability(joint_group.calcJacobian(_SINGULAR_Q, "tool0"))
    assert manip.m.measure == sys.float_info.max
    assert manip.m.condition == sys.float_info.max


# sigma_min 1e-6 sits below the ~3e-4 cutoff of the eigenvalue-clipping path (tesseract#1393).
NEAR_SINGULAR_SIGMA = np.array([3.0, 2.0, 1.0, 0.5, 0.2, 1e-6])
# [1] relative: the SVD's backward error is ~ eps * sigma_max; on the 1e-6 factor that is
# ~2.2e-16 * 3 / 1e-6 ~ 7e-10 relative, x1000 headroom for the six-factor product.
NEAR_SINGULAR_VOLUME_RTOL = 1e-6
# Seed of every random jacobian below (the upstream issue number; any seed serves).
JACOBIAN_SEED = 1393


def _rotated_singular_values(sigma):
    """A (6, 6) jacobian with singular values `sigma`, between two random orthogonal bases."""
    rng = np.random.default_rng(JACOBIAN_SEED)
    q1, _ = np.linalg.qr(rng.standard_normal((6, 6)))
    q2, _ = np.linalg.qr(rng.standard_normal((6, 6)))
    return q1 @ np.diag(sigma) @ q2.T


def _jacobian_stack(count, cols):
    return np.random.default_rng(JACOBIAN_SEED).standard_normal((count, 6, cols))


def test_calc_manipulability_volume_near_singular_is_singular_value_product():
    """tesseract#1396: volume is prod(sigma_i) of J, not zero, for sigma_min below the old cutoff."""
    manip = tk.calcManipulability(_rotated_singular_values(NEAR_SINGULAR_SIGMA))
    assert manip.m.volume == pytest.approx(
        float(np.prod(NEAR_SINGULAR_SIGMA)), rel=NEAR_SINGULAR_VOLUME_RTOL
    )


@pytest.mark.parametrize("cols", [6, 7, 5], ids=["square", "redundant", "tall"])
def test_calc_manipulability_volume_is_the_full_calls_volume_bitwise(cols):
    for jac in _jacobian_stack(8, cols):
        assert tk.calcManipulabilityVolume(jac) == tk.calcManipulability(jac).m.volume


@pytest.mark.parametrize("cols", [6, 7, 5], ids=["square", "redundant", "tall"])
def test_calc_manipulability_volume_each_row_is_the_per_jacobian_call_bitwise(cols):
    stack = _jacobian_stack(8, cols)
    volumes = tk.calcManipulabilityVolume_each(stack)
    assert volumes.shape == (8,)
    assert volumes.tolist() == [tk.calcManipulability(jac).m.volume for jac in stack]


def test_calc_manipulability_volume_each_near_singular_and_exactly_singular():
    near = _rotated_singular_values(NEAR_SINGULAR_SIGMA)
    volumes = tk.calcManipulabilityVolume_each(np.stack([near, np.zeros((6, 6))]))
    assert volumes[0] == pytest.approx(
        float(np.prod(NEAR_SINGULAR_SIGMA)), rel=NEAR_SINGULAR_VOLUME_RTOL
    )
    assert volumes[1] == 0.0


def test_calc_manipulability_volume_each_empty_stack():
    assert tk.calcManipulabilityVolume_each(np.empty((0, 6, 6))).shape == (0,)


def test_calc_manipulability_volume_each_reads_a_fortran_ordered_stack_by_value():
    stack = _jacobian_stack(3, 6)
    expected = tk.calcManipulabilityVolume_each(stack)
    assert tk.calcManipulabilityVolume_each(np.asfortranarray(stack)).tolist() == expected.tolist()


@pytest.mark.parametrize("shape", [(2, 5, 7), (6, 6), (2, 2, 6, 6)])
def test_calc_manipulability_volume_each_needs_a_stack_of_six_rows(shape):
    with pytest.raises(TypeError):
        tk.calcManipulabilityVolume_each(np.ones(shape))


def test_calc_manipulability_volume_needs_six_rows():
    with pytest.raises(TypeError):
        tk.calcManipulabilityVolume(np.asfortranarray(np.ones((5, 7))))


@pytest.mark.parametrize(
    "call",
    [
        lambda: tk.calcManipulability(np.zeros((6, 0))),
        lambda: tk.calcManipulabilityVolume(np.zeros((6, 0))),
        lambda: tk.calcManipulabilityVolume_each(np.zeros((2, 6, 0))),
        lambda: tk.calcManipulabilityVolume_each(np.zeros((0, 6, 0))),
    ],
    ids=["full", "volume", "each", "each_empty"],
)
def test_calc_manipulability_refuses_a_jacobian_without_columns(call):
    """A jacobian without columns has no singular values for upstream to read."""
    with pytest.raises(tk.EmptyJacobianError, match="no columns"):
        call()


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf], ids=["nan", "inf", "-inf"])
@pytest.mark.parametrize(
    "call",
    [
        lambda jac: tk.calcManipulability(jac),
        lambda jac: tk.calcManipulabilityVolume(jac),
        lambda jac: tk.calcManipulabilityVolume_each(np.stack([np.eye(6), jac])),
    ],
    ids=["full", "volume", "each"],
)
def test_calc_manipulability_refuses_a_non_finite_jacobian(call, bad):
    """The SVD flags non-finite input and rank() then reads 0: a blown-up FK chain would read as a singularity."""
    jac = np.eye(6)
    jac[2, 3] = bad
    with pytest.raises(tk.NonFiniteJacobianError, match="not finite"):
        call(jac)


def test_calc_manipulability_volume_each_names_the_non_finite_jacobian():
    stack = np.stack([np.eye(6), np.eye(6), np.eye(6)])
    stack[2, 0, 0] = np.nan
    with pytest.raises(tk.NonFiniteJacobianError, match="jacobian 2"):
        tk.calcManipulabilityVolume_each(stack)


def test_jacobian_errors_are_value_errors():
    assert issubclass(tk.EmptyJacobianError, ValueError)
    assert issubclass(tk.NonFiniteJacobianError, ValueError)


# sigma_min / sigma_max from well-conditioned to far below eps: the sweep crosses the SVD's numerical
# rank threshold (~ 6 * eps ~ 1.3e-15), where the volume steps from prod(sigma_i) to exactly 0.0.
RANK_THRESHOLD_SWEEP_EXPONENTS = np.arange(-1.0, -20.0, -0.25)


def test_calc_manipulability_volume_is_the_full_calls_volume_across_the_rank_threshold():
    stack = np.stack(
        [
            _rotated_singular_values([3.0, 2.0, 1.0, 0.5, 0.2, 3.0 * 10.0**e])
            for e in RANK_THRESHOLD_SWEEP_EXPONENTS
        ]
    )
    full = [tk.calcManipulability(jac).m.volume for jac in stack]
    assert full[0] > 0.0 and full[-1] == 0.0  # the sweep does cross the threshold
    assert tk.calcManipulabilityVolume_each(stack).tolist() == full
    assert [tk.calcManipulabilityVolume(jac) for jac in stack] == full


def test_calc_manipulability_needs_six_rows():
    with pytest.raises(TypeError):
        tk.calcManipulability(np.asfortranarray(np.ones((5, 7))))


def test_is_near_singularity(joint_group):
    assert tk.isNearSingularity(joint_group.calcJacobian(_SINGULAR_Q, "tool0"))
    assert not tk.isNearSingularity(joint_group.calcJacobian(_REGULAR_Q, "tool0"))


@pytest.mark.parametrize("shape", [(0, 0), (6, 0), (0, 7)])
def test_is_near_singularity_empty_raises(shape):
    with pytest.raises(ValueError, match="empty"):
        tk.isNearSingularity(np.zeros(shape))


def test_is_near_singularity_empty_is_the_named_error():
    with pytest.raises(tk.EmptyJacobianError, match="empty"):
        tk.isNearSingularity(np.zeros((6, 0)))


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf], ids=["nan", "inf", "-inf"])
def test_is_near_singularity_refuses_a_non_finite_jacobian(bad):
    """Otherwise a non-finite jacobian reads as near a singularity (True on NaN and on inf)."""
    jac = np.eye(6)
    jac[2, 3] = bad
    with pytest.raises(tk.NonFiniteJacobianError, match="not finite"):
        tk.isNearSingularity(jac)


# --- harmonizers / isValid ---


def test_harmonize_toward_zero_in_place():
    qs = np.array([3 * np.pi / 2, -3 * np.pi / 2, 5.0])
    assert tk.harmonizeTowardZero(qs, [0, 1]) is None
    nptest.assert_allclose(qs, [-np.pi / 2, np.pi / 2, 5.0], rtol=0, atol=ANGLE_ATOL)
    assert qs[2] == 5.0


def test_harmonize_toward_median_in_place():
    # Joint 0 limits [0, 2*pi] (median pi), joint 1 [-pi, pi] (median 0); joint 2 untouched.
    limits = np.array([[0.0, 2 * np.pi], [-np.pi, np.pi], [-1.0, 1.0]])
    qs = np.array([-0.5, 3 * np.pi / 2, 5.0])
    tk.harmonizeTowardMedian(qs, [0, 1], limits)
    nptest.assert_allclose(qs, [2 * np.pi - 0.5, -np.pi / 2, 5.0], rtol=0, atol=ANGLE_ATOL)


@pytest.mark.parametrize("index", [3, -1])
def test_harmonize_toward_zero_bad_index_leaves_qs(index):
    qs = np.array([3 * np.pi / 2, -3 * np.pi / 2, 5.0])
    before = qs.copy()
    with pytest.raises(IndexError):
        tk.harmonizeTowardZero(qs, [0, index])
    nptest.assert_array_equal(qs, before)


@pytest.mark.parametrize(
    ("indices", "limit_rows"), [([0, 3], 3), ([0, 1], 1)], ids=["qs", "limits"]
)
def test_harmonize_toward_median_bad_index_leaves_qs(indices, limit_rows):
    qs = np.array([3 * np.pi / 2, -3 * np.pi / 2, 5.0])
    before = qs.copy()
    limits = np.tile([-np.pi, np.pi], (limit_rows, 1))
    with pytest.raises(IndexError):
        tk.harmonizeTowardMedian(qs, indices, limits)
    nptest.assert_array_equal(qs, before)


def test_harmonize_rejects_float32():
    qs = np.array([3 * np.pi / 2, 0.0], dtype=np.float32)
    with pytest.raises(TypeError):
        tk.harmonizeTowardZero(qs, [0])
    with pytest.raises(TypeError):
        tk.harmonizeTowardMedian(qs, [0], np.tile([-np.pi, np.pi], (2, 1)))


def test_is_valid():
    assert tk.isValid([0.0] * 6)
    assert not tk.isValid([0.0] * 5 + [float("nan")])
    assert not tk.isValid([0.0] * 5 + [float("inf")])
    with pytest.raises(TypeError):
        tk.isValid([0.0] * 5)


# --- checkKinematics ---


def test_check_kinematics_kdl_group():
    kg = kdl._iiwa_env().getKinematicGroup("manipulator")
    assert tk.checkKinematics(kg)


def test_check_kinematics_opw_group():
    """An OPW group built with the KinematicGroup constructor (gh-212), no Environment."""
    factory, _locator = opw.get_plugin_factory()
    scene_graph = opw.get_scene_graph()
    scene_state = tesseract_state_solver.KDLStateSolver(scene_graph).getState(np.zeros(6))
    inv_kin = factory.createInvKin("manipulator", "OPWInvKin", scene_graph, scene_state)
    joints = [f"joint_{i}" for i in range(1, 7)]
    kg = tk.KinematicGroup("manipulator", joints, inv_kin, scene_graph, scene_state)
    assert tk.checkKinematics(kg)
