"""Planning rules written in Python, solved by TrajOpt's SQP in C++.

``PyConstraint`` is a thin base on the ``ConstraintSet`` trampoline: a subclass writes
``f(q)``, the values of the rule at one waypoint's joint values ``q``, and gets bounds,
coefficients and a Jacobian for free. The default Jacobian is a forward difference over that
waypoint's joints only (``calcForwardNumJac``, n_dof + 1 calls to ``f``), placed in the
waypoint's columns of the problem; override ``jacobian(q)`` where an analytic one is cheap.

The rules below use only what Tesseract already computes: forward kinematics, the Jacobian,
manipulability and the discrete contact manager. Two of them have a C++ counterpart, which is
the one to use in production: ``CartAxisConeConstraint`` for ``PointsDown`` and
``CartAxisAlignConstraint`` for ``HoldAxis`` at zero tolerance. They stay here as the worked
examples of a rule with an analytic Jacobian, and the tests check them against the C++ ones.

Example::

    from tesseract_robotics.trajopt_ifopt.python_constraints import PointsDown

    for var in vars_list:
        problem.addConstraintSet(PointsDown(var, manip, "tool0", max_tilt=np.radians(20)))
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING, override

import numpy as np
import scipy.sparse

from tesseract_robotics.kinematics import calcManipulability
from tesseract_robotics.trajopt_ifopt._trajopt_ifopt import Bounds, ConstraintSet, calcForwardNumJac

if TYPE_CHECKING:
    from tesseract_robotics.common import Isometry3d
    from tesseract_robotics.environment import Environment
    from tesseract_robotics.kinematics import JointGroup
    from tesseract_robotics.trajopt_ifopt._trajopt_ifopt import Var

__all__ = [
    "PyConstraint",
    "PointsDown",
    "HoldAxis",
    "StayDexterous",
    "ElbowUp",
    "WatchTheTool",
    "Clearance",
    "WorldHold",
]

# forward-difference step: rounding error ~1e-16 / EPSILON, truncation error ~EPSILON
DEFAULT_EPSILON = 1e-6


class PyConstraint(ConstraintSet):
    """A constraint ``lower <= f(q) <= upper`` on one waypoint's joint values ``q``.

    A row bounded on both sides with lower < upper (a range) is split into two one-sided rows,
    as trajopt's ``RangeBoundHandling.SPLIT_TO_TWO_INEQUALITIES`` does: ``TrajOptQPProblem``
    takes equality and one-sided rows only, and ``addConstraintSet`` / ``addCostSet`` raise on
    a range row.

    Args:
        name: name of the constraint set (shown in the solver's output)
        var: the waypoint's ``Var`` (``node.getVar("joints")``)
        lower: lower bound per row; ``-np.inf`` for none
        upper: upper bound per row; ``np.inf`` for none
        epsilon: forward-difference step of the default Jacobian
        coeff: merit coefficient per row (scalar or one per row): how hard the SQP pushes on a
            violation relative to the other sets (the online example weights collision at 10)
    """

    def __init__(
        self,
        name: str,
        var: Var,
        lower: Sequence[float],
        upper: Sequence[float],
        epsilon: float = DEFAULT_EPSILON,
        coeff: float | Sequence[float] = 1.0,
    ):
        lows = np.asarray(lower, dtype=float).ravel()
        highs = np.asarray(upper, dtype=float).ravel()
        if lows.shape != highs.shape:
            raise ValueError(f"lower has {lows.size} rows, upper {highs.size}")
        if np.any(lows > highs):
            raise ValueError("lower bound above upper bound")
        # one-sided rows: a range [lo, hi] becomes [lo, inf] and [-inf, hi] on the same value
        row_map: list[int] = []
        bounds: list[Bounds] = []
        for i, (lo, hi) in enumerate(zip(lows, highs)):
            if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
                row_map += [i, i]
                bounds += [Bounds(lo, np.inf), Bounds(-np.inf, hi)]
            else:
                row_map.append(i)
                bounds.append(Bounds(lo, hi))
        super().__init__(name, len(row_map))
        self.var = var
        self.epsilon = epsilon
        self._row_map: np.ndarray = np.asarray(row_map, dtype=np.intp)
        self._coeff: np.ndarray = np.broadcast_to(np.asarray(coeff, dtype=float), lows.shape)[
            self._row_map
        ]
        self._bounds = bounds
        self._rows = len(row_map)

    # -- what a subclass writes ------------------------------------------------------------

    def f(self, q: np.ndarray) -> np.ndarray:
        """The rule's values at joint values ``q`` (one waypoint)."""
        raise NotImplementedError

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """d f / d q, shape (rows, n_dof). Default: forward difference over ``q``."""
        return calcForwardNumJac(
            lambda x: np.asarray(self.f(x), dtype=float), q, self.epsilon
        ).toarray()

    # -- the ConstraintSet interface -------------------------------------------------------

    @override
    def getValues(self) -> np.ndarray:
        return np.asarray(self.f(np.asarray(self.var.value())), dtype=float).ravel()[self._row_map]

    @override
    def getBounds(self) -> list[Bounds]:
        return self._bounds

    @override
    def getJacobian(self) -> scipy.sparse.csr_matrix:
        n_vars = self.getVariables().getRows()
        block = np.asarray(self.jacobian(np.asarray(self.var.value())), dtype=float)
        block = block.reshape(-1, block.shape[-1])[self._row_map]
        # a dense block in this waypoint's columns, built straight in CSR form
        n_dof = block.shape[1]
        indices = np.tile(np.arange(self.var.getIndex(), self.var.getIndex() + n_dof), self._rows)
        indptr = np.arange(0, self._rows * n_dof + 1, n_dof)
        return scipy.sparse.csr_matrix((block.ravel(), indices, indptr), shape=(self._rows, n_vars))

    @override
    def update(self) -> int:
        return self._rows

    @override
    def getCoefficients(self) -> np.ndarray:
        return np.array(self._coeff)


def _skew_cross_rows(a: np.ndarray, angular_jacobian: np.ndarray) -> np.ndarray:
    """Rows of d(R e)/dq for a body-fixed unit vector currently at ``a``: omega_i x a."""
    return np.cross(angular_jacobian.T, a).T


class PointsDown(PyConstraint):
    """The tool axis (``axis`` of ``tcp``) stays within ``max_tilt`` of ``down``.

    Value: cos of the angle between the tool axis and ``down``, at least cos(max_tilt). There
    is no upper bound: a cosine cannot pass 1, and a row that sits on its bound where its
    gradient vanishes (the axis aligned) only costs the QP its conditioning.
    Analytic Jacobian from Tesseract's: d(a . down)/dq = down . (omega x a).
    ``CartAxisConeConstraint`` is the C++ form of this rule, on the angle itself.
    """

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        tcp: str,
        max_tilt: float,
        down: Sequence[float] = (0.0, 0.0, -1.0),
        axis: int = 2,
        analytic: bool = True,
        name: str = "points_down",
        **kwargs,
    ):
        super().__init__(name, var, [np.cos(max_tilt)], [np.inf], **kwargs)
        self.manip, self.tcp, self.axis, self.analytic = manip, tcp, axis, analytic
        self.down = np.asarray(down, dtype=float) / np.linalg.norm(down)

    def f(self, q):
        return [self.manip.calcFwdKin(q)[self.tcp].rotation[:, self.axis] @ self.down]

    def jacobian(self, q):
        if not self.analytic:
            return super().jacobian(q)
        a = self.manip.calcFwdKin(q)[self.tcp].rotation[:, self.axis]
        omega = self.manip.calcJacobian(q, self.tcp)[3:6]
        return (self.down @ _skew_cross_rows(a, omega)).reshape(1, -1)


def _perpendicular_pair(direction: np.ndarray) -> np.ndarray:
    """Two unit vectors spanning the plane perpendicular to ``direction``, as rows."""
    seed = np.eye(3)[np.argmin(np.abs(direction))]
    u = np.cross(direction, seed)
    u /= np.linalg.norm(u)
    return np.vstack([u, np.cross(direction, u)])


class HoldAxis(PyConstraint):
    """The tool axis (``axis`` of ``tcp``) stays within ``tol`` of ``direction``, for small ``tol``.

    ``PointsDown`` bounds cos(angle), whose gradient vanishes at zero angle: fine for a tilt limit
    of tens of degrees, too flat to hold a fraction of a degree. This rule bounds the two
    components of the axis perpendicular to ``direction`` (each +-sin(tol), linear in the angle),
    plus axis . direction >= 0 so the opposite direction is not a solution. The cone is the
    square one: up to sqrt(2) * tol on the diagonal.
    Analytic Jacobian from Tesseract's: d(P a)/dq = P (omega x a).
    ``CartAxisAlignConstraint`` is the C++ rule for the equality (no tolerance band).
    """

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        tcp: str,
        direction: Sequence[float],
        tol: float,
        axis: int = 2,
        analytic: bool = True,
        name: str = "hold_axis",
        **kwargs,
    ):
        side = np.sin(tol)
        super().__init__(name, var, [-side, -side, 0.0], [side, side, np.inf], **kwargs)
        self.manip, self.tcp, self.axis, self.analytic = manip, tcp, axis, analytic
        direction = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
        self._basis = np.vstack([_perpendicular_pair(direction), direction])

    def f(self, q):
        return self._basis @ self.manip.calcFwdKin(q)[self.tcp].rotation[:, self.axis]

    def jacobian(self, q):
        if not self.analytic:
            return super().jacobian(q)
        a = self.manip.calcFwdKin(q)[self.tcp].rotation[:, self.axis]
        omega = self.manip.calcJacobian(q, self.tcp)[3:6]
        return self._basis @ _skew_cross_rows(a, omega)


class StayDexterous(PyConstraint):
    """Manipulability of ``tcp`` at least ``w_min``: keeps every waypoint off singularities.

    Value: Yoshikawa's measure sqrt(det(J J^T)), the volume of Tesseract's manipulability
    ellipsoid (``calcManipulability(J).m.volume``).
    """

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        tcp: str,
        w_min: float,
        name: str = "stay_dexterous",
        **kwargs,
    ):
        super().__init__(name, var, [w_min], [np.inf], **kwargs)
        self.manip, self.tcp = manip, tcp

    def f(self, q):
        return [calcManipulability(self.manip.calcJacobian(q, self.tcp)).m.volume]


class ElbowUp(PyConstraint):
    """``upper_link`` stays at least ``delta`` above ``lower_link`` (one IK branch)."""

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        upper_link: str,
        lower_link: str,
        delta: float = 0.0,
        axis: int = 2,
        name: str = "elbow_up",
        **kwargs,
    ):
        super().__init__(name, var, [delta], [np.inf], **kwargs)
        self.manip, self.upper_link, self.lower_link, self.axis = (
            manip,
            upper_link,
            lower_link,
            axis,
        )

    def f(self, q):
        tf = self.manip.calcFwdKin(q)
        return [
            tf[self.upper_link].translation[self.axis] - tf[self.lower_link].translation[self.axis]
        ]


class WatchTheTool(PyConstraint):
    """``tcp`` stays in ``camera``'s cone of view, half-angle ``alpha``, both links in one group.

    Value: cos of the angle between the optical axis and the line to ``tcp``, at least
    cos(alpha); no upper bound, as for ``PointsDown``.
    """

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        camera: str,
        tcp: str,
        alpha: float,
        optical_axis: int = 2,
        name: str = "watch_the_tool",
        **kwargs,
    ):
        super().__init__(name, var, [np.cos(alpha)], [np.inf], **kwargs)
        self.manip, self.camera, self.tcp, self.optical_axis = manip, camera, tcp, optical_axis

    def f(self, q):
        tf = self.manip.calcFwdKin(q)
        cam = tf[self.camera]
        v = tf[self.tcp].translation - cam.translation
        return [cam.rotation[:, self.optical_axis] @ v / np.linalg.norm(v)]


class Clearance(PyConstraint):
    """Closest distance from ``links`` to the rest of the scene stays in [d_min, d_max].

    Uses a clone of the environment's discrete contact manager, so the planner's collision
    engine answers. Distances beyond ``margin`` are reported as ``margin``; keep d_max below it.
    """

    def __init__(
        self,
        var: Var,
        env: Environment,
        joint_ids: Sequence[str],
        links: Sequence[str],
        d_min: float,
        d_max: float,
        margin: float | None = None,
        name: str = "clearance",
        **kwargs,
    ):
        from tesseract_robotics.collision import (
            ContactRequest,
            ContactResultMap,
            ContactResultVector,
            ContactTestType,
        )
        from tesseract_robotics.common import CollisionMarginData

        super().__init__(name, var, [d_min], [d_max], **kwargs)
        self.env, self.joint_ids = env, list(joint_ids)
        self.margin = margin if margin is not None else 2.0 * d_max
        if d_max >= self.margin:
            raise ValueError(f"d_max {d_max} must be below the margin {self.margin}")
        self.mgr = env.getDiscreteContactManager().clone()
        self.mgr.setActiveCollisionObjects(list(links))
        self.mgr.setCollisionMarginData(CollisionMarginData(self.margin))
        self._map, self._vec = ContactResultMap, ContactResultVector
        self._request = ContactRequest(ContactTestType.CLOSEST)

    def f(self, q):
        self.mgr.setCollisionObjectsTransform(self.env.getState(self.joint_ids, q).link_transforms)
        hits, flat = self._map(), self._vec()
        self.mgr.contactTest(hits, self._request)
        hits.flattenMoveResults(flat)
        return [min((c.distance for c in flat), default=self.margin)]


class WorldHold(PyConstraint):
    """``tcp`` holds ``target`` in the world while the base moves, within ``tol`` per axis.

    ``base_pose`` is the base's world pose for this waypoint: an ``Isometry3d``, or a callable
    returning one (e.g. a localisation reader evaluated at the waypoint's time).
    Analytic Jacobian: R_base @ J_linear.
    """

    def __init__(
        self,
        var: Var,
        manip: JointGroup,
        tcp: str,
        target: Sequence[float],
        base_pose: Callable[[], Isometry3d] | Isometry3d,
        tol: Sequence[float] | float = 1e-3,
        analytic: bool = True,
        name: str = "world_hold",
        **kwargs,
    ):
        band = np.broadcast_to(np.asarray(tol, dtype=float), (3,))
        super().__init__(name, var, -band, band, **kwargs)
        self.manip, self.tcp, self.analytic = manip, tcp, analytic
        self.target = np.asarray(target, dtype=float)
        self._base = base_pose if callable(base_pose) else (lambda: base_pose)

    def f(self, q):
        base = self._base()
        p = self.manip.calcFwdKin(q)[self.tcp].translation
        return base.rotation @ p + base.translation - self.target

    def jacobian(self, q):
        if not self.analytic:
            return super().jacobian(q)
        return self._base().rotation @ self.manip.calcJacobian(q, self.tcp)[0:3]
