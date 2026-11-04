"""Per-link world poses in viser's convention."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tesseract_robotics.tesseract_common import Isometry3d, Quaterniond
from tesseract_robotics.tesseract_environment import Environment

from .errors import UnknownJointError

Pose3 = tuple[NDArray[np.float64], NDArray[np.float64]]
"""`(xyz [m], wxyz)`: position and scalar-first unit quaternion, as viser takes them."""


def isometry_to_pose(iso: Isometry3d) -> Pose3:
    """Split an isometry into viser's position and `wxyz` quaternion.

    Args:
        iso: Rigid transform.

    Returns:
        `(xyz, wxyz)`.
    """
    q = Quaterniond(iso.rotation)
    return np.asarray(iso.translation, dtype=np.float64), np.array([q.w, q.x, q.y, q.z])


def link_poses(env: Environment, joint_names: Sequence[str], q: ArrayLike) -> dict[str, Pose3]:
    """World pose of every link with the named joints at `q`, without changing `env`.

    Joints not in `joint_names` keep the environment's current values.

    Args:
        env: Environment to evaluate.
        joint_names: Joints to set.
        q: Value per joint in `joint_names`.

    Returns:
        Pose per link name.

    Raises:
        UnknownJointError: a name is not a joint of `env`.
    """
    unknown = set(joint_names) - set(env.getJointNames())
    if unknown:
        raise UnknownJointError(f"not joints of the environment: {sorted(unknown)}")
    state = env.getState(list(joint_names), np.asarray(q, dtype=np.float64))
    return {link: isometry_to_pose(iso) for link, iso in state.link_transforms.items()}
