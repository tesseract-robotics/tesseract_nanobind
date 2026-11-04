"""CartAxisConeConstraint and CartAxisAlignConstraint (tesseract-robotics/trajopt#602).

The cone keeps the angle phi between a link axis and a target axis at most a half angle: one
one-sided row, phi itself. The alignment drives phi to zero: two equality rows, the tilt angle
times the unit tilt direction, so their norm is phi. Rotation about the axis is free in both.
"""

from dataclasses import dataclass

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinematicGroup

TOOL_Z = np.array([0.0, 0.0, 1.0])
BASE_X = np.array([1.0, 0.0, 0.0])
#: A bent arm, so the tool axis is not along a base axis
SEED = np.array([0.3, 0.6, -0.2, -1.2, 0.4, 0.8, 0.1])


@dataclass
class AxisFixture:
    """``env`` is held only to outlive ``manip`` (the kinematic group borrows from it)."""

    env: Environment
    manip: KinematicGroup
    nodes: ti.NodesVariables
    var: ti.Var

    def cone(self, target_axis, half_angle):
        return ti.CartAxisConeConstraint(
            self.var, self.manip, "tool0", TOOL_Z, "base_link", target_axis, half_angle
        )

    def align(self, target_axis):
        return ti.CartAxisAlignConstraint(
            self.var, self.manip, "tool0", TOOL_Z, "base_link", target_axis
        )


@pytest.fixture
def axes() -> AxisFixture:
    locator = GeneralResourceLocator()
    urdf = FilesystemPath(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
        ).getFilePath()
    )
    srdf = FilesystemPath(
        locator.locateResource(
            "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
        ).getFilePath()
    )
    env = Environment()
    assert env.init(urdf, srdf, locator)
    manip = env.getKinematicGroup("manipulator")
    joint_names = [jid.name() for jid in manip.getJointIds()]
    nodes = ti.createNodesVariables(
        "trajectory", joint_names, [SEED], ti.toBounds(manip.getLimits().joint_limits)
    )
    return AxisFixture(env, manip, nodes, nodes.getNodes()[0].getVar("joints"))


def test_cone_is_one_upper_bounded_row(axes: AxisFixture):
    cone = axes.cone(BASE_X, 0.2)
    assert cone.getRows() == 1
    (bound,) = cone.getBounds()
    assert bound.getType() == ti.BoundsType.UPPER_BOUND
    assert bound.getUpper() == pytest.approx(0.2)
    assert cone.getHalfAngle() == pytest.approx(0.2)


def test_align_is_two_equality_rows(axes: AxisFixture):
    align = axes.align(BASE_X)
    assert align.getRows() == 2
    assert [b.getType() for b in align.getBounds()] == [ti.BoundsType.EQUALITY] * 2


def test_align_rows_measure_the_cone_angle(axes: AxisFixture):
    """|r| is the tilt angle phi, which the cone row reports directly."""
    for target in (BASE_X, np.array([0.0, -1.0, 1.0]), -TOOL_Z):
        phi = axes.cone(target, 1.0).calcValues(SEED)[0]
        r = axes.align(target).calcValues(SEED)
        assert np.linalg.norm(r) == pytest.approx(phi, abs=1e-9)


def test_opposite_targets_split_a_half_turn(axes: AxisFixture):
    phi = axes.cone(BASE_X, 1.0).calcValues(SEED)[0]
    phi_opposite = axes.cone(-BASE_X, 1.0).calcValues(SEED)[0]
    assert phi + phi_opposite == pytest.approx(np.pi, abs=1e-9)


def test_target_axis_is_normalized_and_settable(axes: AxisFixture):
    cone = axes.cone(np.array([2.0, 0.0, 0.0]), 0.5)
    np.testing.assert_allclose(cone.getTargetAxis(), BASE_X)
    cone.setTargetAxis(np.array([0.0, 3.0, 0.0]))
    np.testing.assert_allclose(cone.getTargetAxis(), [0.0, 1.0, 0.0])


@pytest.mark.parametrize("half_angle", [0.0, np.pi])
def test_cone_refuses_a_half_angle_outside_the_open_range(axes: AxisFixture, half_angle):
    with pytest.raises(RuntimeError):
        axes.cone(BASE_X, half_angle)


def test_sqp_aligns_the_tool_axis(axes: AxisFixture):
    align = axes.align(BASE_X)
    assert np.linalg.norm(align.calcValues(SEED)) > 0.5
    problem = tsqp.TrajOptQPProblem(axes.nodes)
    problem.addConstraintSet(align)
    problem.setup()
    solver = tsqp.TrustRegionSQPSolver(tsqp.OSQPEigenSolver())

    solver.solve(problem)

    assert solver.getStatus() == tsqp.SQPStatus.NLP_CONVERGED
    assert np.linalg.norm(align.calcValues(axes.nodes.getValues())) < 1e-3
