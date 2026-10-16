"""trajopt_ifopt test scenes: two overlapping boxes, and the KUKA IIWA.

Each scene keeps its ``Environment``: the kinematic and joint groups are plugin-backed and
borrow from it, so dropping the environment first is a use-after-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import JointGroup, KinematicGroup

# m: edge of each box; at slider x the boxes overlap by BOX_SIZE - |x| along x
BOX_SIZE = 0.2
# m: slider position of the moving box in the fixture's start state, 0.1 m of overlap
OVERLAP_X = 0.1
# m: collision margin; values are margin - distance, so overlap gives positive values
COLLISION_MARGIN = 0.025
# collision coefficient of every pair
COLLISION_COEFF = 20.0

TWO_BOXES_URDF = f"""
<robot name="two_boxes" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="base_link">
    <collision><geometry><box size="{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}"/></geometry></collision>
  </link>
  <link name="slider_link">
    <collision><geometry><box size="{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}"/></geometry></collision>
  </link>
  <joint name="slider_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="slider_link"/>
    <axis xyz="1 0 0"/>
    <limit effort="100" lower="-1.0" upper="1.0" velocity="1.0"/>
  </joint>
</robot>
"""

# The evaluators need the contact manager plugins, which only an SRDF configures
TWO_BOXES_SRDF = """
<robot name="two_boxes">
  <contact_managers_plugin_config filename="package://tesseract/support/urdf/contact_manager_plugins.yaml"/>
</robot>
"""


@dataclass
class TwoBoxes:
    """A fixed box at the origin and a box on a prismatic x joint, overlapping at OVERLAP_X."""

    env: Environment
    manip: JointGroup
    nodes: ti.NodesVariables
    var: ti.Var

    def config(self) -> ti.TrajOptCollisionConfig:
        return ti.TrajOptCollisionConfig(COLLISION_MARGIN, COLLISION_COEFF)

    def discrete_evaluator(self) -> ti.SingleTimestepCollisionEvaluator:
        return ti.SingleTimestepCollisionEvaluator(self.manip, self.env, self.config())

    def lvs_discrete_evaluator(self) -> ti.LVSDiscreteCollisionEvaluator:
        config = self.config()
        config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
        return ti.LVSDiscreteCollisionEvaluator(self.manip, self.env, config)


def two_boxes_at(*positions: float) -> TwoBoxes:
    """The two-box scene with one node per slider position."""
    env = Environment()
    assert env.init(TWO_BOXES_URDF, TWO_BOXES_SRDF, GeneralResourceLocator())
    manip = env.getJointGroup("slider", ["slider_joint"])
    nodes = ti.createNodesVariables(
        "trajectory",
        ["slider_joint"],
        [np.array([x]) for x in positions],
        ti.toBounds(manip.getLimits().joint_limits),
    )
    return TwoBoxes(env, manip, nodes, nodes.getNodes()[0].getVar("joints"))


@dataclass
class Iiwa:
    """KUKA IIWA environment and its kinematic group."""

    env: Environment
    manip: KinematicGroup
    joint_names: list[str]
    joint_limits: np.ndarray

    def nodes(self, *states: np.ndarray) -> ti.NodesVariables:
        return ti.createNodesVariables(
            "trajectory", self.joint_names, list(states), ti.toBounds(self.joint_limits)
        )


def load_iiwa() -> Iiwa:
    """The KUKA IIWA environment and its "manipulator" kinematic group."""
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
    assert urdf is not None and srdf is not None
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    manip = env.getKinematicGroup("manipulator")
    return Iiwa(env, manip, list(manip.getJointNames()), manip.getLimits().joint_limits)
