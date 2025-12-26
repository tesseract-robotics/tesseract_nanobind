"""Tests for the Pythonic planning API."""

import numpy as np
import pytest

from tesseract_robotics.planning import (
    Robot,
    RobotState,
    Pose,
    translation,
    rotation_x,
    rotation_y,
    rotation_z,
    MotionProgram,
    CartesianTarget,
    JointTarget,
    StateTarget,
    MoveType,
    box,
    sphere,
    cylinder,
    create_obstacle,
)


class TestPose:
    """Test Pose class and helpers."""

    def test_identity(self):
        t = Pose.identity()
        np.testing.assert_array_almost_equal(t.position, [0, 0, 0])
        np.testing.assert_array_almost_equal(t.quaternion, [0, 0, 0, 1])

    def test_from_xyz(self):
        t = Pose.from_xyz(1, 2, 3)
        assert t.x == 1
        assert t.y == 2
        assert t.z == 3
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_from_position(self):
        t = Pose.from_position([1.5, 2.5, 3.5])
        np.testing.assert_array_almost_equal(t.position, [1.5, 2.5, 3.5])

    def test_from_xyz_quat(self):
        # 90 degree rotation around Z
        t = Pose.from_xyz_quat(1, 2, 3, 0, 0, 0.707, 0.707)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        np.testing.assert_array_almost_equal(
            t.quaternion, [0, 0, 0.707, 0.707], decimal=3
        )

    def test_from_xyz_rpy(self):
        # 90 degrees around Z
        t = Pose.from_xyz_rpy(1, 2, 3, 0, 0, np.pi / 2)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        roll, pitch, yaw = t.rpy
        np.testing.assert_almost_equal(yaw, np.pi / 2, decimal=5)

    def test_translation_helper(self):
        t = translation(1, 2, 3)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_rotation_x(self):
        t = rotation_x(np.pi / 2)
        # Should rotate Y to Z
        y_vec = np.array([0, 1, 0])
        rotated = t.rotation_matrix @ y_vec
        np.testing.assert_array_almost_equal(rotated, [0, 0, 1], decimal=5)

    def test_rotation_y(self):
        t = rotation_y(np.pi / 2)
        # Should rotate Z to X
        z_vec = np.array([0, 0, 1])
        rotated = t.rotation_matrix @ z_vec
        np.testing.assert_array_almost_equal(rotated, [1, 0, 0], decimal=5)

    def test_rotation_z(self):
        t = rotation_z(np.pi / 2)
        # Should rotate X to Y
        x_vec = np.array([1, 0, 0])
        rotated = t.rotation_matrix @ x_vec
        np.testing.assert_array_almost_equal(rotated, [0, 1, 0], decimal=5)

    def test_pose_chaining(self):
        t1 = translation(1, 0, 0)
        t2 = translation(0, 2, 0)
        combined = t1 @ t2
        np.testing.assert_array_almost_equal(combined.position, [1, 2, 0])

    def test_pose_inverse(self):
        t = Pose.from_xyz(1, 2, 3)
        inv = t.inverse()
        combined = t @ inv
        np.testing.assert_array_almost_equal(combined.position, [0, 0, 0], decimal=5)

    def test_to_isometry(self):
        t = Pose.from_xyz(1, 2, 3)
        iso = t.to_isometry()
        assert iso is not None
        # Round-trip
        t2 = Pose.from_isometry(iso)
        np.testing.assert_array_almost_equal(t.position, t2.position)


class TestRobot:
    """Test Robot loading and state management."""

    @pytest.fixture
    def robot(self):
        """Load test robot."""
        return Robot.from_tesseract_support("abb_irb2400")

    def test_load_robot(self, robot):
        assert robot is not None
        assert len(robot.get_link_names()) > 0

    def test_get_joint_names(self, robot):
        joints = robot.get_joint_names("manipulator")
        assert len(joints) == 6
        assert "joint_1" in joints

    def test_get_state(self, robot):
        state = robot.get_state()
        assert isinstance(state, RobotState)
        assert len(state.joint_names) > 0
        assert len(state.joint_positions) == len(state.joint_names)

    def test_set_joints_dict(self, robot):
        robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})
        state = robot.get_state(["joint_1", "joint_2"])
        np.testing.assert_almost_equal(state["joint_1"], 0.5)
        np.testing.assert_almost_equal(state["joint_2"], -0.3)

    def test_set_joints_array(self, robot):
        joints = robot.get_joint_names("manipulator")
        values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robot.set_joints(values, joint_names=joints)
        state = robot.get_state(joints)
        np.testing.assert_array_almost_equal(state.joint_positions, values)

    def test_fk(self, robot):
        pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0])
        assert isinstance(pose, Pose)
        # ABB IRB2400 tool0 at zeros should be around x=0.94, z=1.455
        assert pose.x > 0.9
        assert pose.z > 1.4

    def test_get_manipulator_info(self, robot):
        info = robot.get_manipulator_info("manipulator")
        assert info.manipulator == "manipulator"
        assert info.tcp_frame is not None

    def test_get_joint_limits(self, robot):
        """Test retrieving joint limits for manipulator group."""
        limits = robot.get_joint_limits("manipulator")

        # ABB IRB2400 has 6 joints
        assert len(limits) == 6

        # Check that all expected joints are present
        for i in range(1, 7):
            joint_name = f"joint_{i}"
            assert joint_name in limits

            # Each joint should have all limit types
            joint_limits = limits[joint_name]
            assert "lower" in joint_limits
            assert "upper" in joint_limits
            assert "velocity" in joint_limits
            assert "acceleration" in joint_limits

            # Lower should be less than upper
            assert joint_limits["lower"] < joint_limits["upper"]
            # Velocity and acceleration should be positive
            assert joint_limits["velocity"] > 0
            assert joint_limits["acceleration"] > 0

    def test_get_joint_limits_values(self, robot):
        """Test specific joint limit values for ABB IRB2400."""
        limits = robot.get_joint_limits("manipulator")

        # Joint 1 limits for ABB IRB2400 (from URDF)
        # These are typical values, may need adjustment based on actual URDF
        j1 = limits["joint_1"]
        assert j1["lower"] < 0  # negative lower bound
        assert j1["upper"] > 0  # positive upper bound
        # Typical range is about +/- 3 radians (~170 degrees)
        assert j1["lower"] > -4.0
        assert j1["upper"] < 4.0

    def test_ik_basic(self, robot):
        """Test basic inverse kinematics computation."""
        # First do FK to get a known reachable pose
        zero_joints = [0, 0, 0, 0, 0, 0]
        target_pose = robot.fk("manipulator", zero_joints)

        # Now solve IK for that pose
        result = robot.ik("manipulator", target_pose, seed=zero_joints)

        assert result is not None
        assert len(result) == 6

        # Verify FK of result matches target
        result_pose = robot.fk("manipulator", result)
        np.testing.assert_array_almost_equal(
            result_pose.position, target_pose.position, decimal=4
        )

    def test_ik_with_seed(self, robot):
        """Test IK with different seed configurations."""
        # Get pose at non-zero configuration
        seed_joints = [0.3, -0.5, 0.4, 0.0, 0.5, 0.0]
        target_pose = robot.fk("manipulator", seed_joints)

        # Solve IK starting from the known solution
        result = robot.ik("manipulator", target_pose, seed=seed_joints)

        assert result is not None

        # Result should produce same end-effector pose
        result_pose = robot.fk("manipulator", result)
        np.testing.assert_array_almost_equal(
            result_pose.position, target_pose.position, decimal=4
        )

    def test_ik_unreachable_pose(self, robot):
        """Test IK returns None for unreachable poses."""
        # Create a pose far outside robot workspace
        unreachable_pose = Pose.from_xyz(10.0, 10.0, 10.0)

        result = robot.ik("manipulator", unreachable_pose)

        # Should return None for unreachable pose
        assert result is None

    def test_ik_no_seed_uses_current_state(self, robot):
        """Test IK uses current robot state when no seed provided."""
        # Set robot to known configuration
        test_joints = [0.2, -0.3, 0.4, 0.1, 0.5, -0.2]
        joint_names = robot.get_joint_names("manipulator")
        robot.set_joints(test_joints, joint_names=joint_names)

        # Get pose at this configuration
        target_pose = robot.fk("manipulator", test_joints)

        # Solve IK without seed - should use current state
        result = robot.ik("manipulator", target_pose)

        assert result is not None

        # Verify solution is valid
        result_pose = robot.fk("manipulator", result)
        np.testing.assert_array_almost_equal(
            result_pose.position, target_pose.position, decimal=4
        )


class TestMotionProgram:
    """Test MotionProgram builder."""

    def test_create_empty(self):
        program = MotionProgram("manipulator")
        assert len(program) == 0

    def test_add_joint_target(self):
        program = MotionProgram("manipulator")
        program.move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        assert len(program) == 1

    def test_add_cartesian_target(self):
        program = MotionProgram("manipulator")
        program.move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
        assert len(program) == 1

    def test_fluent_api(self):
        program = (
            MotionProgram("manipulator")
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
            .linear_to(CartesianTarget(Pose.from_xyz(0.5, 0.2, 0.5)))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )
        assert len(program) == 4

    def test_to_composite_instruction(self):
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )
        composite = program.to_composite_instruction()
        assert composite is not None
        assert len(composite) == 2


class TestTargets:
    """Test target types."""

    def test_joint_target(self):
        target = JointTarget([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        assert target.move_type == MoveType.FREESPACE
        np.testing.assert_array_equal(target.positions, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_joint_target_with_names(self):
        names = ["j1", "j2", "j3"]
        target = JointTarget([0.1, 0.2, 0.3], names=names)
        wp = target.to_waypoint()
        assert wp is not None

    def test_cartesian_target_from_pose(self):
        pose = Pose.from_xyz(1, 2, 3)
        target = CartesianTarget(pose)
        assert target.pose.x == 1
        assert target.pose.y == 2
        assert target.pose.z == 3

    def test_cartesian_target_from_position(self):
        target = CartesianTarget(position=[1, 2, 3])
        assert target.pose.x == 1
        assert target.pose.y == 2
        assert target.pose.z == 3

    def test_cartesian_target_from_position_quaternion(self):
        target = CartesianTarget(
            position=[1, 2, 3],
            quaternion=[0, 0, 0.707, 0.707],
        )
        np.testing.assert_array_almost_equal(target.pose.position, [1, 2, 3])

    def test_state_target(self):
        target = StateTarget(
            positions=[0.1, 0.2, 0.3],
            names=["j1", "j2", "j3"],
            velocities=[1.0, 2.0, 3.0],
            accelerations=[0.5, 0.5, 0.5],
            time=0.5,
        )
        wp = target.to_waypoint()
        assert wp is not None
        # Check values are set correctly
        np.testing.assert_array_almost_equal(wp.getPosition(), [0.1, 0.2, 0.3])
        np.testing.assert_array_almost_equal(wp.getVelocity(), [1.0, 2.0, 3.0])
        np.testing.assert_array_almost_equal(wp.getAcceleration(), [0.5, 0.5, 0.5])
        assert wp.getTime() == 0.5


class TestGeometry:
    """Test geometry helpers."""

    def test_box(self):
        b = box(1, 2, 3)
        assert b is not None

    def test_sphere(self):
        s = sphere(0.5)
        assert s is not None

    def test_cylinder(self):
        c = cylinder(0.5, 1.0)
        assert c is not None


class TestCreateObstacle:
    """Test obstacle creation."""

    @pytest.fixture
    def robot(self):
        return Robot.from_tesseract_support("abb_irb2400")

    def test_create_box_obstacle(self, robot):
        result = create_obstacle(
            robot,
            name="test_box",
            geometry=box(0.5, 0.5, 0.5),
            transform=Pose.from_xyz(0.5, 0, 0.3),
        )
        assert result is True
        assert "test_box" in robot.get_link_names()

    def test_create_sphere_obstacle(self, robot):
        result = create_obstacle(
            robot,
            name="test_sphere",
            geometry=sphere(0.1),
            transform=Pose.from_xyz(0.3, 0.2, 0.6),
            color=(1.0, 0, 0, 1.0),
        )
        assert result is True
        assert "test_sphere" in robot.get_link_names()


class TestRobotLinkManagement:
    """Test Robot.add_link, remove_link, and allowed collision methods."""

    @pytest.fixture
    def robot(self):
        """Load test robot."""
        return Robot.from_tesseract_support("abb_irb2400")

    def test_add_link_with_geometry(self, robot):
        """Test adding a link with visual/collision geometry."""
        from tesseract_robotics.tesseract_scene_graph import (
            Link,
            Joint,
            JointType,
            Visual,
            Collision,
        )
        from tesseract_robotics.tesseract_geometry import Box

        # Create link with box geometry
        link = Link("test_obstacle")
        geometry = Box(0.1, 0.1, 0.1)
        visual = Visual()
        visual.geometry = geometry
        link.visual.append(visual)
        collision = Collision()
        collision.geometry = geometry
        link.collision.append(collision)

        # Create fixed joint
        joint = Joint("joint_test_obstacle")
        joint.type = JointType.FIXED
        joint.parent_link_name = "base_link"
        joint.child_link_name = "test_obstacle"

        # Verify not present initially
        assert "test_obstacle" not in robot.get_link_names()

        # Add link
        result = robot.add_link(link, joint)
        assert result is True
        assert "test_obstacle" in robot.get_link_names()

    def test_add_link_using_create_fixed_joint(self, robot):
        """Test adding link using create_fixed_joint helper."""
        from tesseract_robotics.tesseract_scene_graph import Link, Visual, Collision
        from tesseract_robotics.tesseract_geometry import Sphere
        from tesseract_robotics.planning import create_fixed_joint

        # Create sphere link
        link = Link("sphere_obstacle")
        geometry = Sphere(0.05)
        visual = Visual()
        visual.geometry = geometry
        link.visual.append(visual)
        collision = Collision()
        collision.geometry = geometry
        link.collision.append(collision)

        # Use helper for joint
        joint = create_fixed_joint(
            "joint_sphere_obstacle",
            parent_link="base_link",
            child_link="sphere_obstacle",
        )

        result = robot.add_link(link, joint)
        assert result is True
        assert "sphere_obstacle" in robot.get_link_names()

    def test_remove_link(self, robot):
        """Test removing a link from the environment."""
        # First add a link
        result = create_obstacle(
            robot,
            name="removable_box",
            geometry=box(0.1, 0.1, 0.1),
            transform=Pose.from_xyz(0.5, 0, 0.3),
        )
        assert result is True
        assert "removable_box" in robot.get_link_names()

        # Remove it
        result = robot.remove_link("removable_box")
        assert result is True
        assert "removable_box" not in robot.get_link_names()

    def test_remove_nonexistent_link(self, robot):
        """Test removing a link that doesn't exist returns False."""
        result = robot.remove_link("nonexistent_link_xyz")
        assert result is False

    def test_add_remove_multiple_links(self, robot):
        """Test adding and removing multiple links."""
        # Add multiple obstacles
        for i in range(3):
            result = create_obstacle(
                robot,
                name=f"multi_box_{i}",
                geometry=box(0.05, 0.05, 0.05),
                transform=Pose.from_xyz(0.3 + i * 0.1, 0, 0.5),
            )
            assert result is True

        # Verify all added
        for i in range(3):
            assert f"multi_box_{i}" in robot.get_link_names()

        # Remove in reverse order
        for i in range(2, -1, -1):
            result = robot.remove_link(f"multi_box_{i}")
            assert result is True
            assert f"multi_box_{i}" not in robot.get_link_names()

    def test_add_allowed_collision(self, robot):
        """Test adding an allowed collision pair."""
        # Add obstacle first
        create_obstacle(
            robot,
            name="collision_box",
            geometry=box(0.1, 0.1, 0.1),
            transform=Pose.from_xyz(0.5, 0, 0.3),
        )

        # Add allowed collision between link6 and the new obstacle
        result = robot.add_allowed_collision("link_6", "collision_box", "TestReason")
        assert result is True

    def test_add_allowed_collision_with_default_reason(self, robot):
        """Test add_allowed_collision uses default 'Adjacent' reason."""
        create_obstacle(
            robot,
            name="acm_test_box",
            geometry=box(0.05, 0.05, 0.05),
            transform=Pose.from_xyz(0.4, 0, 0.4),
        )

        # Should work with default reason
        result = robot.add_allowed_collision("link_5", "acm_test_box")
        assert result is True

    def test_add_allowed_collision_between_robot_links(self, robot):
        """Test adding allowed collision between existing robot links."""
        # Add allowed collision between two robot links
        result = robot.add_allowed_collision("link_3", "link_6", "UserDefined")
        assert result is True


class TestPlanningIntegration:
    """Integration tests for planning (require task composer)."""

    @pytest.fixture
    def robot(self):
        return Robot.from_tesseract_support("abb_irb2400")

    def test_plan_freespace(self, robot):
        """Test freespace planning through TaskComposer."""
        from tesseract_robotics.planning import plan_freespace

        joint_names = robot.get_joint_names("manipulator")
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )

        result = plan_freespace(robot, program)

        assert result.successful
        assert len(result) > 0
        assert result.trajectory[0].positions is not None
