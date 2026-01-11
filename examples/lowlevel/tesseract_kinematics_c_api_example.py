"""
Low-Level Kinematics Example (C-API Style)
===========================================

Demonstrates forward and inverse kinematics using Tesseract's low-level
C++ bindings without the high-level Robot wrapper class.

C++ Reference:
    tesseract_kinematics/examples/kinematics_example.cpp

High-Level Alternative:
    examples/tesseract_kinematics_example.py (uses Robot wrapper class)

Overview
--------
This example directly uses the C++ API bindings for kinematics operations:
1. Manually load URDF/SRDF and initialize Environment
2. Get KinematicGroup from Environment
3. Compute forward kinematics (FK): joint angles -> Isometry3d pose
4. Compute inverse kinematics (IK): target pose -> joint angles

Why Use Low-Level API?
----------------------
- Access to all IK solver parameters and options
- Multiple IK inputs for simultaneous multi-chain solving
- Direct control over working frames and tip links
- Performance-critical applications requiring minimal overhead

Key Concepts
------------
**Kinematic Groups (KinematicGroup)**:
    Defined in SRDF, groups specify which joints form a kinematic chain.
    Each group has a name (e.g., "manipulator"), base link, and tip link.
    The group determines which joints are included in FK/IK calculations.

**Forward Kinematics (calcFwdKin)**:
    Input: joint positions (numpy array)
    Output: TransformMap (dict: link_name -> Isometry3d)

    FK is deterministic - one joint configuration produces exactly one pose.
    Returns transforms for ALL links in the group, not just the tip.

**Inverse Kinematics (calcInvKin)**:
    Input: KinGroupIKInputs (list of poses), seed joint positions
    Output: list of joint configurations (may have 0, 1, or many solutions)

    IK is generally non-unique:
    - 6-DOF robots: up to 8 solutions for typical poses
    - 7-DOF robots: infinite solutions (1-DOF redundancy)
    - Some poses may have no solution (unreachable workspace)

**KinGroupIKInput Structure**:
    - pose: Target Isometry3d (position + orientation)
    - tip_link_name: End link of kinematic chain (e.g., "tool0")
    - working_frame: Reference frame for the pose (e.g., "base_link")

**Kinematics Solvers**:
    Configured via plugin YAML referenced in SRDF:

    OPW (Ortho-Parallel-Wrist): Analytical solver for 6-DOF industrial robots
    with spherical wrists (ABB, KUKA, Fanuc, etc.). Requires kinematic
    parameters in plugin config. Fast and returns all 8 solutions.

    KDL (Kinematics and Dynamics Library): Numerical iterative solver.
    Works with any robot geometry. Slower, returns single solution near seed.
    Used by KUKA IIWA, UR robots, custom mechanisms.

**Quaternion Convention**:
    Eigen::Quaterniond uses (w, x, y, z) order in constructor but Tesseract's
    Quaterniond follows Eigen. When printing: q.w(), q.x(), q.y(), q.z().
    Be careful with conversions to/from scipy, transforms3d, etc.

**Isometry3d**:
    Rigid transformation (rotation + translation, no scaling/shearing).
    - Isometry3d.Identity(): 4x4 identity matrix
    - Translation3d(x, y, z): Creates translation component
    - Multiply with * to compose transforms (right-to-left)

Pipeline
--------
1. Environment Setup:
   - GeneralResourceLocator finds URDF/SRDF via TESSERACT_RESOURCE_PATH
   - Environment.init() parses robot description and loads kinematic plugins
   - Plugin config specifies solver type and parameters (OPW, KDL, etc.)

2. Forward Kinematics:
   - getKinematicGroup("manipulator") returns solver for named group
   - calcFwdKin(joint_positions) returns TransformMap
   - Access specific link: transform_map["tool0"]

3. Inverse Kinematics:
   - Construct KinGroupIKInput with target pose, tip_link, working_frame
   - Wrap in KinGroupIKInputs vector
   - calcInvKin(inputs, seed_position) returns solution array
   - Iterate results (may be empty if no solution exists)

Related Examples
----------------
- tesseract_kinematics_example.py: Same workflow with Robot API
- basic_cartesian_example.py: Uses IK internally for Cartesian planning
- freespace_ompl_example.py: Uses FK for trajectory validation
- pick_and_place_example.py: FK/IK for grasp pose computation
"""

from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    Translation3d,
    Quaterniond,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs
import numpy as np

# Initialize Environment with a robot from URDF file
# The URDF and SRDF file must be configured. The kinematics solver also requires plugin configuration,
# which is specified in the SRDF file. For this example, the plugin configuration file is `abb_irb2400_plugins.yaml`
# and is located in the same directory as the SRDF file. This example uses the OPW kinematics solver, which is
# a solver for industrial 6-dof robots with spherical wrists. The kinematic parameters for the robot must
# be specified in the plugin configuration file in addition to the URDF file for the plugin to work.
# The other main solver is the KDL solver, which is used by the lbr_iiwa_14_r820 robot also included in the
# tesseract_support package. The KDL solver is a numerical solver and does not require additional configuration.

# This example uses the GeneralResourceLocator to find resources on the file system. The GeneralResourceLocator
# uses the TESSERACT_RESOURCE_PATH environmental variable.
#
# TESSERACT_RESOURCE_PATH must be set to the directory containing the `tesseract_support` package. This can be done
# by running:
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# export TESSERACT_RESOURCE_PATH="$(pwd)/tesseract/"
#
# or on Windows
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# set TESSERACT_RESOURCE_PATH=%cd%\tesseract\


def main():
    # =========================================================================
    # STEP 1: Environment Initialization
    # =========================================================================
    # GeneralResourceLocator resolves "package://" URIs using TESSERACT_RESOURCE_PATH
    locator = GeneralResourceLocator()
    env = Environment()

    # ABB IRB2400 uses OPW (analytical) kinematics solver
    # Solver is configured in abb_irb2400_plugins.yaml referenced by SRDF
    urdf_path_str = locator.locateResource(
        "package://tesseract_support/urdf/abb_irb2400.urdf"
    ).getFilePath()
    srdf_path_str = locator.locateResource(
        "package://tesseract_support/urdf/abb_irb2400.srdf"
    ).getFilePath()

    # FilesystemPath wraps std::filesystem::path for cross-platform compatibility
    urdf_path = FilesystemPath(urdf_path_str)
    srdf_path = FilesystemPath(srdf_path_str)

    # init() parses URDF/SRDF and loads configured kinematic plugins
    assert env.init(urdf_path, srdf_path, locator)

    # ABB IRB2400 joint naming convention: joint_1 through joint_6
    _robot_joint_names = [f"joint_{i + 1}" for i in range(6)]  # noqa: F841

    # =========================================================================
    # STEP 2: Get Kinematic Group
    # =========================================================================
    # "manipulator" is the group name defined in SRDF
    # The group specifies: base_link, tip_link (tool0), and included joints
    kin_group = env.getKinematicGroup("manipulator")

    # =========================================================================
    # STEP 3: Forward Kinematics
    # =========================================================================
    # Joint configuration: 6 angles in radians
    # np.deg2rad converts from human-readable degrees
    robot_joint_pos = np.deg2rad(np.array([10, 20, -5, 70, 30, 90], dtype=np.float64))

    # calcFwdKin returns TransformMap: dict mapping link names to Isometry3d
    # Contains transforms for ALL links in the kinematic chain
    fwdkin_result = kin_group.calcFwdKin(robot_joint_pos)

    # Extract the tool0 (flange) transform - this is typically what we care about
    tool0_transform = fwdkin_result["tool0"]

    # Extract position and orientation from Isometry3d
    # rotation() returns 3x3 rotation matrix, Quaterniond converts to quaternion
    print(f"Tool0 transform at joint position {robot_joint_pos} is: ")
    q = Quaterniond(tool0_transform.rotation())
    print(f"Translation: {tool0_transform.translation().flatten()}")
    # Quaternion printed as (w, x, y, z) - Eigen convention
    print(f"Rotation: {q.w()} {q.x()} {q.y()} {q.z()}")

    # =========================================================================
    # STEP 4: Inverse Kinematics
    # =========================================================================
    # Define target pose for end effector
    # Compose transform: Identity * Translation * Rotation
    # Note: Quaterniond constructor order is (w, x, y, z) - Eigen convention
    tool0_transform2 = (
        Isometry3d.Identity()
        * Translation3d(0.7, -0.1, 1)
        * Quaterniond(0.70711, 0, 0.7171, 0)
    )

    # KinGroupIKInput specifies a single IK request
    # Multiple inputs can be solved simultaneously for multi-chain robots
    ik = KinGroupIKInput()
    ik.pose = tool0_transform2  # Target transform (Isometry3d)
    ik.tip_link_name = "tool0"  # End effector link
    ik.working_frame = "base_link"  # Reference frame for the pose

    # KinGroupIKInputs is a vector - append all IK requests
    iks = KinGroupIKInputs()
    iks.append(ik)

    # Solve IK with seed position as initial guess
    # OPW solver returns up to 8 solutions for 6-DOF robots
    # KDL solver returns 1 solution (iterative, converges to nearest)
    ik_result = kin_group.calcInvKin(iks, robot_joint_pos)

    # Print all solutions found
    # Each solution is a numpy array of joint positions
    print(f"Found {len(ik_result)} solutions")
    for i in range(len(ik_result)):
        print(f"Solution {i}: {ik_result[i].flatten()}")

    # =========================================================================
    # CLEANUP: Prevent GC order segfaults
    # =========================================================================
    # Python's GC may destroy objects in arbitrary order at script exit.
    # This can cause segfaults if the locator is destroyed before objects
    # that depend on it. Explicit deletion in correct order prevents this.
    import gc

    del ik_result, ik, iks, fwdkin_result, tool0_transform
    del kin_group, env, locator
    gc.collect()


if __name__ == "__main__":
    main()
