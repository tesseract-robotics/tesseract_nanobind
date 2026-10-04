"""ConstraintSet.getJacobian reaches Python whatever the set's sparse storage.

trajopt assembles some Jacobians with coeffRef: the collision constraints (analytic and
numerical, discrete and continuous) while a contact is active. That leaves Eigen's sparse
matrix uncompressed, and nanobind's Eigen caster only returns compressed matrices, so
getJacobian() raised for exactly the rows a caller needs ("unable to return an Eigen sparse
matrix that is not in a compressed format"). The binding returns a compressed copy.
"""

from __future__ import annotations

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import AddLinkCommand, Environment
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_scene_graph import Collision, Joint, JointType, Link

# The robot's pose and an obstacle cube centred on its tool: the collision row is active.
STATE = np.array([0.5, 0.3, 0.0, -1.2, 0.0, 0.5, 0.0])
OBSTACLE_EDGE = 0.1  # m
MARGIN = 0.05  # m, the collision config's safety margin
COEFF = 20.0


def test_collision_constraint_with_an_active_contact():
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
    srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
    env = Environment()
    assert env.init(FilesystemPath(urdf.getFilePath()), FilesystemPath(srdf.getFilePath()), locator)
    manip = env.getKinematicGroup("manipulator")

    obstacle = Link("obstacle")
    collision = Collision()
    collision.geometry = Box(OBSTACLE_EDGE, OBSTACLE_EDGE, OBSTACLE_EDGE)
    obstacle.addCollision(collision)
    joint = Joint("obstacle_joint")
    joint.type = JointType.FIXED
    joint.parent_link_id = "base_link"
    joint.child_link_id = "obstacle"
    joint.parent_to_joint_origin_transform = manip.calcFwdKin(STATE)["tool0"]
    assert env.applyCommand(AddLinkCommand(obstacle, joint))

    nodes = ti.createNodesVariables(
        "trajectory",
        list(manip.getJointIds()),
        [STATE],
        ti.toBounds(manip.getLimits().joint_limits),
    )
    var = nodes.getNodes()[0].getVar("joints")
    evaluator = ti.SingleTimestepCollisionEvaluator(
        manip, env, ti.TrajOptCollisionConfig(MARGIN, COEFF), True
    )
    constraint = ti.DiscreteCollisionConstraint(evaluator, var, 1, False, "collision")
    problem = tsqp.TrajOptQPProblem(nodes)
    problem.addConstraintSet(constraint)  # links the set to the variables
    problem.setup()
    assert constraint.getValues()[0] > 0.0  # inside the margin: the row is active

    jacobian = constraint.getJacobian()

    assert jacobian.shape == (1, len(STATE))
    assert jacobian.nnz > 0
