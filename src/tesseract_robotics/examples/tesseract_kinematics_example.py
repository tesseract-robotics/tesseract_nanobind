"""
Kinematics Example

Demonstrates forward and inverse kinematics using the high-level Robot API.

C++ Reference:
    tesseract_kinematics/examples/kinematics_example.cpp

**Overview**
This example shows the kinematics pipeline:
1. Load robot with kinematic solver (KDL by default)
2. Compute forward kinematics (FK): joint angles -> end effector pose
3. Compute inverse kinematics (IK): target pose -> joint angles

**Key Concepts**
**Forward Kinematics (FK)**:
    Given joint angles, compute the pose of any link in the kinematic chain.
    FK is always unique - one set of joint angles produces exactly one pose.
    Used for: visualization, collision checking, trajectory validation.

**Inverse Kinematics (IK)**:
    Given a target pose, find joint angles that achieve that pose.
    IK may have multiple solutions (redundant robots) or no solution (unreachable).
    Used for: motion planning, task-space control, pose targeting.

**Kinematic Groups**:
    SRDF defines named groups (e.g., "manipulator") specifying which joints
    belong together as a kinematic chain. The group also defines the base
    and tip links for the chain.

**Tip Link vs TCP Frame**:
    - tip_link: The end link of the kinematic chain (e.g., "tool0")
    - tcp_frame: Tool Center Point offset from tip_link (for attached tools)
    IK solves for the tip_link pose, then applies TCP offset if specified.

**Seed State**:
    IK solvers are iterative and require an initial guess (seed). Different
    seeds may find different solutions. For redundant robots, nearby seeds
    tend to find nearby solutions.

**Quaternion Convention**:
    Project canonical order is scalar-last `[qx, qy, qz, qw]` — used by
    every public Python API (`Pose.from_xyz_quat`, `Quaterniond.from_xyzw`,
    `q.coeffs()`, etc.). `Quaterniond([x, y, z, w])` takes the same order; there is no
    scalar-first constructor.

**Related Examples**
- basic_cartesian_example.py: Uses IK internally for Cartesian planning
- freespace_ompl_example.py: Uses FK for state validation
- pick_and_place_example.py: FK/IK for grasp pose computation
"""

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator, Quaterniond
from tesseract_robotics.planning import Robot

# Offset of every joint from the IK seed to the configuration the IK target is taken from [rad].
IK_TARGET_OFFSET_RAD = 0.1


# Pose for the forward kinematics [rad]: elbow bent -1.57 rad (-90 deg) on joint 4, wrist
# rotated 1.57 rad (90 deg) on joint 6; 1.57 is a rounded pi/2.
IK_JOINT_POS_RAD = (0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0)


def main():
    # --8<-- [start:fk_ik]
    # =========================================================================
    # STEP 1: Load Robot with Kinematic Solver
    # =========================================================================
    # KUKA LBR IIWA 14 R820 is a 7-DOF collaborative robot
    # 7 joints = redundant (more DOF than needed for 6D pose) -> multiple IK solutions
    # KDL (Kinematics and Dynamics Library) is the default solver
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )

    # =========================================================================
    # STEP 2: Forward Kinematics
    # =========================================================================
    # Joint configuration: 7 joint angles in radians
    # This pose bends the elbow (-1.57 rad = -90 deg on joint 4)
    # and rotates the wrist (1.57 rad = 90 deg on joint 6)
    joint_pos = np.array(IK_JOINT_POS_RAD)

    # robot.fk() computes pose of tip_link given joint angles
    # "manipulator" is the kinematic group defined in SRDF
    # "tool0" is the flange frame at end of arm
    tool0_pose = robot.fk("manipulator", joint_pos, tip_link="tool0")

    print(f"Tool0 transform at joint position {joint_pos}:")
    print(f"Translation: {tool0_pose.translation}")  # [x, y, z] in meters
    print(f"Rotation (quat): {Quaterniond(tool0_pose.linear).coeffs()}")  # [x, y, z, w] scalar-last

    # =========================================================================
    # STEP 3: Inverse Kinematics
    # =========================================================================
    # Target pose: the FK of a configuration IK_TARGET_OFFSET_RAD away from the seed, so a
    # solution exists and the solver, started at the seed, can be checked against it.
    target_pose = robot.fk("manipulator", joint_pos + IK_TARGET_OFFSET_RAD, tip_link="tool0")

    # robot.ik() solves for joint angles that achieve target pose
    # seed: initial guess for iterative solver (affects which solution is found)
    # tip_link: end of kinematic chain (must match FK tip_link for consistency)
    # IK raises IKNoSolutionError if no solution is found (pose unreachable or solver failed)
    ik_solutions = [robot.ik("manipulator", target_pose, seed=joint_pos, tip_link="tool0")]

    # Report results
    # For 7-DOF robot, infinite solutions exist for a 6D pose (self-motion)
    # The solver returns the solution closest to the seed
    print(f"\nFound {len(ik_solutions)} solution(s)")
    for i, sol in enumerate(ik_solutions):
        print(f"Solution {i}: {sol}")
    # --8<-- [end:fk_ik]


if __name__ == "__main__":
    main()
