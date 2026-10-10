"""
Kinematics Analysis Example
===========================

Checks IIWA Jacobians against finite differences, then analyses them:
manipulability, singularity, joint-angle harmonizing, limits, and the pose
error functions that build task-space Jacobians.

C++ Reference:
    kinematics/test/kinematics_test_utils.h  (runJacobianTest, runJacobianIIWATest,
        runKinGroupJacobianIIWATest)
    common/test/tesseract_common_unit.cpp  (calcTransformError, applyTolerances,
        calcJacobianTransformErrorDiff_Toleranced)

Overview
--------
1. Port of upstream `runJacobianTest`: every native `calcJacobian` overload, and
   `jacobianChangeBase` / `jacobianChangeRefPoint`, against all three
   `numericalJacobian` overloads, at upstream's q with its link points and
   `change_base`.
2. `twistChangeBase` / `twistChangeRefPoint` on J·q̇ equal the same change
   applied to J, then multiplied by q̇ (a linear identity).
3. `calcManipulability` and `isNearSingularity` at upstream's q and at q = 0,
   where the IIWA is singular.
4. `harmonizeTowardZero` / `harmonizeTowardMedian` undo a 2π shift.
5. `isValid`, `checkKinematics`.
6. `KinematicLimits` (`resize`, `jerk_limits`), `isWithinLimits`, `enforceLimits`.
7. `calcTransformError`, `calcRotationalError`, `applyTolerances` and the four
   `calcJacobianTransformErrorDiff` overloads, on upstream's unit-test values.

Key Concepts
------------
**Finite-difference tolerance**:
    `numericalJacobian` is a forward difference with δ = 1e-8
    (kinematics/core/src/utils.cpp:44, 80). Its error is truncation, δ/2 times the
    second derivative (for a revolute joint at most the point's distance from
    the axis), plus rounding: two FK results, each off by a few ulps, divided by
    δ. `JACOBIAN_TOL` adds the two; upstream's 1e-3 is a loose ceiling.

**Why q = 0 is singular**:
    `joint_a1`, `joint_a5` and `joint_a7` all turn about the base z-axis at
    q = 0: every axis is `0 0 1`, and the x offsets −0.00043624 (joint_a2) and
    +0.00043624 (joint_a4) cancel. Three identical Jacobian columns leave rank 5.
    (`joint_a3` sits at x = −0.00043624, so it is parallel but not collinear.)
"""

from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import (
    AngleAxisd,
    GeneralResourceLocator,
    Isometry3d,
    KinematicLimits,
    Translation3d,
    applyTolerances,
    calcJacobianTransformErrorDiff,
    calcRotationalError,
    calcTransformError,
    enforceLimits,
    isWithinLimits,
    jacobianChangeBase,
    jacobianChangeRefPoint,
    twistChangeBase,
    twistChangeRefPoint,
)
from tesseract_robotics.tesseract_kinematics import (
    KinematicGroup,
    KinematicsPluginFactory,
    Manipulability,
    ManipulabilityEllipsoid,
    calcManipulability,
    checkKinematics,
    harmonizeTowardMedian,
    harmonizeTowardZero,
    isNearSingularity,
    isValid,
    numericalJacobian,
)
from tesseract_robotics.tesseract_state_solver import KDLStateSolver
from tesseract_robotics.tesseract_urdf import parseURDFFile

URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
PLUGINS_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820_plugins.yaml"
GROUP = "manipulator"
TIP_LINK = "tool0"
JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)]
# runKinGroupJacobianIIWATest link_names (kinematics_test_utils.h:671-672)
LINK_NAMES = [
    "base_link",
    "link_1",
    "link_2",
    "link_3",
    "link_4",
    "link_5",
    "link_6",
    "link_7",
    "tool0",
]

# Upstream jvals (kinematics_test_utils.h:598-604), rad
Q = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
# Upstream change_base rotation (kinematics_test_utils.h:635-638): Rz(90°)
RZ90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

EPS = np.finfo(float).eps
# numericalJacobian forward-difference step (kinematics/core/src/utils.cpp:44, 80), rad
FD_DELTA = 1e-8
# Largest point the checks difference: tool0 at full stretch (1.306 m,
# runFwdKinIIWATest) + the 1 m link point + the 1 m change_base translation, m
MAX_POINT_NORM = 1.306 + 1.0 + 1.0
# FK composes 8 transforms (joint_a1..a7, joint_a7-tool0); each 3-term dot
# product rounds by at most γ_3 ≈ 3ε relative
FK_ROUNDING = 8 * 3 * EPS
# Forward-difference error: truncation δ/2·|f''| (|f''| ≤ point norm) plus
# rounding 2·FK_ROUNDING·|f|/δ; m/rad for linear rows, ≤ the same for angular rows
JACOBIAN_TOL = FD_DELTA / 2 * MAX_POINT_NORM + 2 * FK_ROUNDING * MAX_POINT_NORM / FD_DELTA
# Twist identity: either side is a 7-term product J·q̇ (γ_7), a 3-term rotation
# (γ_3) and a cross product plus add (γ_3); two sides, relative to |R||J||q̇|
TWIST_ROUNDING = 2 * (7 + 3 + 3) * EPS
# isNearSingularity default threshold (kinematics/core/include/tesseract/kinematics/utils.h:129)
SINGULARITY_THRESHOLD = 0.01
# volume = sqrt(product of ≤ 6 eigenvalues): the product rounds by ≤ γ_5 in either
# summation order, halved by the sqrt, plus the sqrt's own ½ε; two sides
VOLUME_RTOL = 2 * (5 / 2 + 1 / 2) * EPS
# q + 2π, then + π inside the harmonizer: two roundings of values below 16 rad,
# each at most ½ ulp(8.0), plus the final ±π at most ½ ulp(8.0); fmod is exact
HARMONIZE_TOL = 3 * 0.5 * float(np.spacing(8.0))
# Eigen isApprox precision NumTraits<double>::dummy_precision(), which upstream's
# calcTransformError test uses
EIGEN_PRECISION = 1e-12
# Upstream calcJacobianTransformErrorDiff_Toleranced (tesseract_common_unit.cpp:2901-2904, 3009)
ERROR_DIFF_EPS = 1e-6
ERROR_DIFF_ATOL = 1e-10
DYNAMIC_BAND_EDGE_ATOL = 1e-4
ERROR_DIFF_BASE_ANGLE = 0.5
ERROR_DIFF_BIG_STEP = 0.2

UNIT_X = np.array([1.0, 0.0, 0.0])


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def isometry(rotation: np.ndarray = np.eye(3), translation: np.ndarray = np.zeros(3)) -> Isometry3d:
    """Isometry3d from a rotation matrix and a translation."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return Isometry3d(matrix)


def is_approx(a: np.ndarray, b: np.ndarray) -> bool:
    """Eigen's `a.isApprox(b)`: ‖a − b‖ ≤ precision · min(‖a‖, ‖b‖)."""
    return bool(
        np.linalg.norm(a - b) <= EIGEN_PRECISION * min(np.linalg.norm(a), np.linalg.norm(b))
    )


def unit(k: int) -> np.ndarray:
    """Unit vector e_k in R³."""
    v = np.zeros(3)
    v[k] = 1.0
    return v


def ellipsoid_fields(ellipsoid: ManipulabilityEllipsoid) -> dict[str, Any]:
    """Every field of a ManipulabilityEllipsoid, plus the oracles the test checks."""
    eigen_values = ellipsoid.eigen_values
    volume = ellipsoid.volume
    expected_volume = float(np.sqrt(np.prod(eigen_values)))
    return {
        "eigen_values": eigen_values,
        "min_eigen_value": float(eigen_values.min()),
        "max_eigen_value": float(eigen_values.max()),
        "measure": ellipsoid.measure,
        "condition": ellipsoid.condition,
        "volume": volume,
        "volume_residual": abs(volume - expected_volume) / expected_volume
        if expected_volume
        else abs(volume),
    }


def manipulability_fields(manip: Manipulability) -> dict[str, dict[str, Any]]:
    """The six ellipsoids of a Manipulability."""
    return {
        "m": ellipsoid_fields(manip.m),
        "m_linear": ellipsoid_fields(manip.m_linear),
        "m_angular": ellipsoid_fields(manip.m_angular),
        "f": ellipsoid_fields(manip.f),
        "f_linear": ellipsoid_fields(manip.f_linear),
        "f_angular": ellipsoid_fields(manip.f_angular),
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator()
    factory = KinematicsPluginFactory(locate(locator, PLUGINS_URL), locator)
    scene_graph = parseURDFFile(str(locate(locator, URDF_URL)), locator)
    scene_state = KDLStateSolver(scene_graph).getState()
    fwd_kin = factory.createFwdKin(GROUP, "KDLFwdKinChain", scene_graph, scene_state)
    inv_kin = factory.createInvKin(GROUP, "KDLInvKinChainLMA", scene_graph, scene_state)
    group = KinematicGroup(GROUP, JOINT_NAMES, inv_kin, scene_graph, scene_state)

    # --8<-- [start:jacobian_fwd_kin]
    # runJacobianIIWATest: tool0 at the origin and at e_k, with and without change_base
    cases = [(np.zeros(3), Isometry3d.Identity())]
    cases += [(unit(k), Isometry3d.Identity()) for k in range(3)]
    cases += [(np.zeros(3), isometry(RZ90, unit(k))) for k in range(3)]
    cases += [(unit(k), isometry(RZ90, unit(k))) for k in range(3)]

    tip_pose = fwd_kin.calcFwdKin(Q)[TIP_LINK]
    errors = []
    for link_point, change_base in cases:
        jacobian = np.asfortranarray(fwd_kin.calcJacobian(Q, TIP_LINK))
        jacobianChangeBase(jacobian, change_base)
        jacobianChangeRefPoint(jacobian, (change_base * tip_pose).linear @ link_point)
        numerical = numericalJacobian(change_base, fwd_kin, Q, TIP_LINK, link_point)
        errors.append(np.abs(numerical - jacobian).max())
    # --8<-- [end:jacobian_fwd_kin]

    # --8<-- [start:jacobian_group]
    # runKinGroupJacobianIIWATest: every link, every static and active base link
    poses = group.calcFwdKin(Q)
    relative_errors = []
    for k in range(3):
        link_point = unit(k)
        link_offset = Isometry3d(Translation3d(link_point))
        for link in LINK_NAMES:
            identity = Isometry3d.Identity()
            errors.append(
                np.abs(
                    group.calcJacobian(Q, link, link_point)
                    - numericalJacobian(identity, group, Q, link, link_point)
                ).max()
            )
            errors.append(
                np.abs(
                    group.calcJacobian(Q, link)
                    - numericalJacobian(identity, group, Q, link, np.zeros(3))
                ).max()
            )
            for base in group.getStaticLinkNames():
                change_base = poses[base].inverse()
                errors.append(
                    np.abs(
                        group.calcJacobian(Q, base, link, link_point)
                        - numericalJacobian(change_base, group, Q, link, link_point)
                    ).max()
                )
                errors.append(
                    np.abs(
                        group.calcJacobian(Q, base, link)
                        - numericalJacobian(change_base, group, Q, link, np.zeros(3))
                    ).max()
                )
            for base in group.getActiveLinkNames():
                relative_errors.append(
                    np.abs(
                        group.calcJacobian(Q, base, link, link_point)
                        - numericalJacobian(group, Q, base, identity, link, link_offset)
                    ).max()
                )
                relative_errors.append(
                    np.abs(
                        group.calcJacobian(Q, base, link)
                        - numericalJacobian(group, Q, base, identity, link, identity)
                    ).max()
                )
    # --8<-- [end:jacobian_group]
    jacobian_result = {
        "fwd_kin_cases": len(cases),
        "group_cases": len(errors) - len(cases) + len(relative_errors),
        "max_error": float(max(errors)),
        "max_error_relative_base": float(max(relative_errors)),
    }

    # --8<-- [start:twist]
    jacobian = np.asfortranarray(fwd_kin.calcJacobian(Q, TIP_LINK))
    q_dot = np.arange(1.0, 8.0) / 10.0
    change_base, ref_point = isometry(RZ90, unit(0)), unit(2)

    twist = jacobian @ q_dot
    twistChangeBase(twist, change_base)
    twistChangeRefPoint(twist, ref_point)

    changed = jacobian.copy(order="F")
    jacobianChangeBase(changed, change_base)
    jacobianChangeRefPoint(changed, ref_point)
    # --8<-- [end:twist]
    scale = float((np.abs(jacobian) @ np.abs(q_dot)).max()) * (1.0 + np.linalg.norm(ref_point))
    twist_result = {
        "residual": float(np.abs(twist - changed @ q_dot).max()),
        "bound": TWIST_ROUNDING * scale,
    }

    # --8<-- [start:singularity]
    singularity = {}
    manipulability = {}
    for name, q in [("upstream", Q), ("zero", np.zeros(len(JOINT_NAMES)))]:
        jacobian = fwd_kin.calcJacobian(q, TIP_LINK)
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        singularity[name] = {
            "sigma_min": float(singular_values.min()),
            "rank": int(np.linalg.matrix_rank(jacobian)),
            "near_singularity": isNearSingularity(jacobian),
            # joint_a1, a5, a7 columns (indices 0, 4, 6)
            "identical_columns": bool(
                np.array_equal(jacobian[:, 0], jacobian[:, 4])
                and np.array_equal(jacobian[:, 0], jacobian[:, 6])
            ),
        }
        manip = calcManipulability(jacobian)
        manipulability[name] = (
            manipulability_fields(manip) if name == "upstream" else {"m": ellipsoid_fields(manip.m)}
        )
        if name == "upstream":
            manipulability["repr"] = repr(manip)
    # --8<-- [end:singularity]
    manipulability["default_repr"] = (repr(Manipulability()), repr(ManipulabilityEllipsoid()))

    # --8<-- [start:harmonize]
    redundant = group.getRedundancyCapableJointIndices()
    position_limits = group.getLimits().joint_limits
    shifted = Q.copy()
    shifted[redundant] += 2 * np.pi

    toward_zero = shifted.copy()
    harmonizeTowardZero(toward_zero, redundant)
    toward_median = shifted.copy()
    harmonizeTowardMedian(toward_median, redundant, position_limits)
    # --8<-- [end:harmonize]
    harmonize = {
        "toward_zero": float(np.abs(toward_zero - Q).max()),
        "toward_median": float(np.abs(toward_median - Q).max()),
    }

    # --8<-- [start:validity]
    six = Q[:6].tolist()
    is_valid = (isValid(six), isValid([*six[:5], float("nan")]))
    check_kinematics = checkKinematics(group)
    # --8<-- [end:validity]

    # --8<-- [start:limits]
    group_limits = group.getLimits()
    limits = KinematicLimits()
    limits.resize(len(JOINT_NAMES))
    shapes_after_resize = [
        limits.joint_limits.shape,
        limits.velocity_limits.shape,
        limits.acceleration_limits.shape,
        limits.jerk_limits.shape,
    ]
    limits.joint_limits = group_limits.joint_limits
    limits.velocity_limits = group_limits.velocity_limits
    limits.acceleration_limits = group_limits.acceleration_limits
    limits.jerk_limits = group_limits.jerk_limits

    within = (isWithinLimits(Q, limits.joint_limits), isWithinLimits(shifted, limits.joint_limits))
    shifted_before = shifted.copy()
    enforced = enforceLimits(shifted, limits.joint_limits)
    # --8<-- [end:limits]
    limits_result = {
        "shapes_after_resize": [tuple(shape) for shape in shapes_after_resize],
        "equal_to_group_limits": limits == group_limits,
        "jerk_limits": limits.jerk_limits,
        "within": within,
        # every shifted value lies above its upper limit, so all clamp to it
        "enforced_is_upper_bound": bool(np.array_equal(enforced, limits.joint_limits[:, 1])),
        "input_unchanged": bool(np.array_equal(shifted, shifted_before)),
    }

    # --8<-- [start:transform_error]
    identity = Isometry3d.Identity()
    rot_x = Isometry3d(AngleAxisd(np.pi / 2, UNIT_X))
    translated = Isometry3d(Translation3d(1.0, 2.0, 3.0))
    rotation_error = calcTransformError(identity, rot_x)
    translation_error = calcTransformError(identity, translated)
    # --8<-- [end:transform_error]
    transform_error = {
        "rotation_part": is_approx(rotation_error[3:], np.array([np.pi / 2, 0.0, 0.0])),
        "rotation_translation_zero": bool(np.array_equal(rotation_error[:3], np.zeros(3))),
        "translation_part": is_approx(translation_error[:3], np.array([1.0, 2.0, 3.0])),
        "translation_rotation_zero": bool(np.array_equal(translation_error[3:], np.zeros(3))),
        "rotational_error": is_approx(
            calcRotationalError(rot_x.rotation), np.array([np.pi / 2, 0.0, 0.0])
        ),
    }

    # --8<-- [start:tolerances]
    v = np.array([-2.0, -0.25, 0.25, 2.0])
    applyTolerances(v, np.array([-1.0, -0.5, -0.5, -1.0]), np.array([1.0, 0.5, 0.5, 1.0]))

    target = Isometry3d(Translation3d(1.0, 2.0, 3.0))
    target_perturbed = Isometry3d(AngleAxisd(-ERROR_DIFF_EPS, UNIT_X)) * target
    source = Isometry3d(AngleAxisd(ERROR_DIFF_BASE_ANGLE, UNIT_X))
    source_perturbed = Isometry3d(AngleAxisd(ERROR_DIFF_BASE_ANGLE + ERROR_DIFF_EPS, UNIT_X))
    source_big = Isometry3d(AngleAxisd(ERROR_DIFF_BASE_ANGLE + ERROR_DIFF_BIG_STEP, UNIT_X))

    # Band around both errors on the x-rotation row: clamped to zero
    inside_lower = np.array([-5.0, -5.0, -5.0, 0.0, -0.5, -0.5])
    inside_upper = np.array([5.0, 5.0, 5.0, 1.0, 0.5, 0.5])
    # Band below both errors: the same offset on both, the raw difference remains
    band_top = ERROR_DIFF_BASE_ANGLE - 0.1
    below_lower = np.array([-5.0, -5.0, -5.0, band_top - 0.01, -0.5, -0.5])
    below_upper = np.array([5.0, 5.0, 5.0, band_top, 0.5, 0.5])
    # Band edge between the two errors: 0.5 inside [0, 0.55], 0.7 above it -> 0.15
    edge_upper = np.array([5.0, 5.0, 5.0, 0.55, 0.5, 0.5])

    raw = calcJacobianTransformErrorDiff(target, source, source_perturbed)
    raw_dynamic = calcJacobianTransformErrorDiff(target, target_perturbed, source, source_perturbed)
    error_diff = {
        "raw": raw,
        "inside_band": np.abs(
            calcJacobianTransformErrorDiff(
                target, source, source_perturbed, inside_lower, inside_upper
            )
        ).max(),
        "above_band_vs_raw": np.abs(
            calcJacobianTransformErrorDiff(
                target, source, source_perturbed, below_lower, below_upper
            )
            - raw
        ).max(),
        "band_edge": calcJacobianTransformErrorDiff(
            target, source, source_big, inside_lower, edge_upper
        )[3],
        "dynamic_inside_band": np.abs(
            calcJacobianTransformErrorDiff(
                target, target_perturbed, source, source_perturbed, inside_lower, inside_upper
            )
        ).max(),
        "dynamic_above_band_vs_raw": abs(
            calcJacobianTransformErrorDiff(
                target, target_perturbed, source, source_perturbed, below_lower, below_upper
            )[3]
            - raw_dynamic[3]
        ),
        "dynamic_band_edge": calcJacobianTransformErrorDiff(
            target, target_perturbed, source, source_big, inside_lower, edge_upper
        )[3],
    }
    # --8<-- [end:tolerances]

    return {
        "jacobian": jacobian_result,
        "twist": twist_result,
        "singularity": singularity,
        "manipulability": manipulability,
        "harmonize": harmonize,
        "is_valid": is_valid,
        "check_kinematics": check_kinematics,
        "limits": limits_result,
        "transform_error": transform_error,
        "apply_tolerances": v.tolist(),
        "error_diff": error_diff,
    }


def main() -> None:
    """Console entry point (`tesseract_kinematics_analysis_example`): run() and print the results."""
    result = run()
    jac = result["jacobian"]
    print(
        f"Jacobian vs numericalJacobian ({jac['fwd_kin_cases']} ForwardKinematics + {jac['group_cases']} "
        f"KinematicGroup cases): max error {jac['max_error']:.2e}, relative to an active base link "
        f"{jac['max_error_relative_base']:.2e} (tolerance {JACOBIAN_TOL:.2e})"
    )
    twist = result["twist"]
    print(
        f"Twist change == Jacobian change · q̇: residual {twist['residual']:.1e} <= {twist['bound']:.1e}"
    )
    for name, s in result["singularity"].items():
        print(
            f"q = {name}: rank {s['rank']}, σ_min {s['sigma_min']:.2e}, near singularity {s['near_singularity']}"
            + (", joint_a1/a5/a7 columns identical" if s["identical_columns"] else "")
        )
    manip = result["manipulability"]
    print(f"Manipulability at upstream q: {manip['repr']}")
    singular = manip["zero"]["m"]
    print(
        f"At q = 0: m.measure = m.condition = {singular['measure']:.3e} (float max), m.volume = {singular['volume']}"
    )
    harmonize = result["harmonize"]
    print(
        f"Harmonize q + 2π: toward zero off by {harmonize['toward_zero']:.1e}, "
        f"toward median by {harmonize['toward_median']:.1e} (<= {HARMONIZE_TOL:.1e})"
    )
    print(
        f"isValid (finite, with NaN): {result['is_valid']}; checkKinematics: {result['check_kinematics']}"
    )
    limits = result["limits"]
    print(
        f"Limits after resize: {limits['shapes_after_resize']}; equal to group limits {limits['equal_to_group_limits']}; "
        f"isWithinLimits (q, q + 2π): {limits['within']}; enforceLimits clamps q + 2π to the upper limits "
        f"{limits['enforced_is_upper_bound']}"
    )
    print(
        f"calcTransformError / calcRotationalError match upstream: {all(result['transform_error'].values())}"
    )
    print(
        f"applyTolerances([-2, -0.25, 0.25, 2], ±[1, 0.5, 0.5, 1]) = {result['apply_tolerances']}"
    )
    diff = result["error_diff"]
    print(
        f"calcJacobianTransformErrorDiff: raw x-rotation row {diff['raw'][3]:.3e}; inside the band "
        f"{diff['inside_band']:.1e}; across the band edge {diff['band_edge']:.6f} (dynamic target "
        f"{diff['dynamic_band_edge']:.6f})"
    )


if __name__ == "__main__":
    main()
