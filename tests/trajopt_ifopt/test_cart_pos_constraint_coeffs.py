"""CartPosConstraint's per-axis coefficients + bounds overload.

The default constructor is an equality on all six axes at unit weight. That
cannot express a process constraint, which needs two things the equality form
has no room for:

* a **free axis** -- a rotation the task is indifferent to (about an
  axis-symmetric tool, say), which must not be pulled back to the target; and
* an **asymmetric bound** -- a standoff that may open but never close.

Both are needed to use the term as a COST (``IfoptProblem.addCostSet``) rather
than a hard pin. These tests pin the semantics that make that possible: a zero
coefficient DROPS the axis' row, and the bounds survive the trip into C++
unmangled.
"""

from dataclasses import dataclass

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinematicGroup

#: [x, y, z, rx, ry, rz]: position tight, standoff ASYMMETRIC (may open, never
#: close), tilt bounded, rotation about the tool axis free.
BAND_LOWER = np.array([-0.0005, -0.0005, -0.0015, -0.087, -0.087, -np.pi])
BAND_UPPER = np.array([0.0005, 0.0005, 0.0000, 0.087, 0.087, np.pi])


@dataclass
class CartPosFixture:
    """A kinematic group and the single Var a ``CartPosConstraint`` is built on.

    ``env`` is held only to outlive ``manip``: the kinematic group is
    plugin-backed and borrows from the environment, so dropping the env first
    is a use-after-free. Do not remove the field because no test reads it.
    """

    env: Environment
    manip: KinematicGroup
    var: ti.Var

    def constraint(
        self,
        coeffs,
        handling: ti.RangeBoundHandling = ti.RangeBoundHandling.KEEP_AS_IS,
    ) -> ti.CartPosConstraint:
        """Build the constraint under test over the fixture's BAND."""
        return ti.CartPosConstraint(
            self.var,
            np.asarray(coeffs, dtype=float),
            ti.toBounds(BAND_LOWER, BAND_UPPER),
            self.manip,
            "tool0",
            "base_link",
            Isometry3d.Identity(),
            Isometry3d.Identity(),
            "CartPos",
            handling,
        )


@pytest.fixture
def cart_pos() -> CartPosFixture:
    """KUKA IIWA kinematic group plus a single-node Var to constrain."""
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
    joint_names = list(manip.getJointNames())
    nodes = ti.createNodesVariables(
        "trajectory",
        joint_names,
        [np.zeros(len(joint_names))],
        ti.toBounds(manip.getLimits().joint_limits),
    )
    return CartPosFixture(env, manip, nodes.getNodes()[0].getVar("joints"))


def test_all_axes_weighted_keeps_six_rows(cart_pos: CartPosFixture):
    c = cart_pos.constraint(np.ones(6))
    assert c.getRows() == 6
    assert len(c.getBounds()) == 6
    np.testing.assert_allclose(np.asarray(c.getCoefficients()), np.ones(6))


def test_zero_coefficient_drops_that_axis(cart_pos: CartPosFixture):
    """A zero coefficient removes the row entirely -- the axis is FREE.

    This is the mechanism that lets rotation about an axis-symmetric tool go
    unconstrained while every other axis stays pinned.
    """
    c = cart_pos.constraint([1.0, 1.0, 1.0, 1.0, 1.0, 0.0])
    assert c.getRows() == 5, "the zero-weighted axis should contribute no row"
    assert len(c.getValues()) == 5
    # the surviving coefficients are the five non-zero ones
    np.testing.assert_allclose(np.asarray(c.getCoefficients()), np.ones(5))


def test_bounds_reach_cpp_unmangled(cart_pos: CartPosFixture):
    """The band survives the trip, asymmetry included.

    A silently symmetrised or dropped bound is the failure this guards: the
    standoff bound is the one that must never gain a positive upper limit.
    """
    c = cart_pos.constraint([1.0, 1.0, 1.0, 1.0, 1.0, 0.0])
    bounds = c.getBounds()
    # roll dropped, so the five remaining bounds are axes 0..4 in order
    for i, b in enumerate(bounds):
        assert b.getLower() == pytest.approx(BAND_LOWER[i])
        assert b.getUpper() == pytest.approx(BAND_UPPER[i])
        assert b.getType() == ti.BoundsType.RANGE_BOUND
    # the standoff axis is one-sided: it may open, never close
    assert bounds[2].getUpper() == pytest.approx(0.0)
    assert bounds[2].getLower() < 0.0


def test_split_handling_doubles_the_rows(cart_pos: CartPosFixture):
    """SPLIT_TO_TWO_INEQUALITIES turns each range into two one-sided rows."""
    c = cart_pos.constraint(
        [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
        # each range row [lb, ub] is emitted as two rows, g(x) >= lb and
        # g(x) <= ub, so five constrained axes report ten rows. KEEP_AS_IS
        # (the fixture default) would keep them as five [lb, ub] rows.
        ti.RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES,
    )
    assert c.getRows() == 10
    kinds = [b.getType() for b in c.getBounds()]
    assert kinds[0::2] == [ti.BoundsType.LOWER_BOUND] * 5
    assert kinds[1::2] == [ti.BoundsType.UPPER_BOUND] * 5


def test_default_ctor_still_constrains_all_six(cart_pos: CartPosFixture):
    """The pre-existing overload is untouched: equality on all six axes."""
    c = ti.CartPosConstraint(
        cart_pos.var,
        cart_pos.manip,
        "tool0",
        "base_link",
        Isometry3d.Identity(),
        Isometry3d.Identity(),
        "CartPos",
        ti.RangeBoundHandling.KEEP_AS_IS,
    )
    assert c.getRows() == 6
