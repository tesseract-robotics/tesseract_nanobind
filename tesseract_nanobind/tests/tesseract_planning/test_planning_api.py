"""Tests for the Pythonic planning API."""

import numpy as np
import pytest

from tesseract_robotics.planning import (
    CartesianTarget,
    JointTarget,
    MotionProgram,
    MoveType,
    Pose,
    Robot,
    RobotState,
    StateTarget,
    box,
    create_obstacle,
    cylinder,
    rotation_x,
    rotation_y,
    rotation_z,
    sphere,
    translation,
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
        np.testing.assert_array_almost_equal(t.quaternion, [0, 0, 0.707, 0.707], decimal=3)

    def test_from_xyz_rpy(self):
        # 90 degrees around Z
        t = Pose.from_xyz_rpy(1, 2, 3, 0, 0, np.pi / 2)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        roll, pitch, yaw = t.rpy
        np.testing.assert_almost_equal(yaw, np.pi / 2, decimal=5)

    def test_translation_helper(self):
        t = translation(1, 2, 3)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    @pytest.mark.parametrize(
        "rotation_func,input_vec,expected",
        [
            (rotation_x, [0, 1, 0], [0, 0, 1]),  # Y → Z
            (rotation_y, [0, 0, 1], [1, 0, 0]),  # Z → X
            (rotation_z, [1, 0, 0], [0, 1, 0]),  # X → Y
        ],
        ids=["rotation_x", "rotation_y", "rotation_z"],
    )
    def test_rotation_90deg(self, rotation_func, input_vec, expected):
        """Test 90-degree rotation around each axis."""
        t = rotation_func(np.pi / 2)
        rotated = t.rotation_matrix @ np.array(input_vec)
        np.testing.assert_array_almost_equal(rotated, expected, decimal=5)

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

    def test_pose_repr(self):
        """Test Pose string representation."""
        t = Pose.from_xyz(1.5, 2.5, 3.5)
        repr_str = repr(t)
        assert "Pose" in repr_str
        assert "1.5" in repr_str or "1.50" in repr_str

    def test_rotation_from_quaternion(self):
        """Test rotation_from_quaternion factory function."""
        from tesseract_robotics.planning import rotation_from_quaternion

        # 90 degree rotation around Z axis
        t = rotation_from_quaternion(0, 0, 0.707, 0.707)
        assert t is not None
        # Should rotate X to Y
        x_vec = np.array([1, 0, 0])
        rotated = t.rotation_matrix @ x_vec
        np.testing.assert_array_almost_equal(rotated, [0, 1, 0], decimal=2)

    def test_rotation_from_axis_angle(self):
        """Test rotation_from_axis_angle factory function."""
        from tesseract_robotics.planning import rotation_from_axis_angle

        # 90 degree rotation around Z axis
        t = rotation_from_axis_angle([0, 0, 1], np.pi / 2)
        assert t is not None
        # Should rotate X to Y
        x_vec = np.array([1, 0, 0])
        rotated = t.rotation_matrix @ x_vec
        np.testing.assert_array_almost_equal(rotated, [0, 1, 0], decimal=5)

    def test_from_position_quaternion(self):
        """Test Pose.from_position_quaternion factory method."""
        pos = [1, 2, 3]
        quat = [0, 0, 0.707, 0.707]  # 90 deg around Z
        t = Pose.from_position_quaternion(pos, quat)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        np.testing.assert_array_almost_equal(t.quaternion, quat, decimal=2)

    def test_from_matrix(self):
        """Test Pose.from_matrix factory method for 4x4 matrix."""
        # Identity matrix
        mat = np.eye(4)
        mat[0, 3] = 1.5  # Translation x
        mat[1, 3] = 2.5  # Translation y
        mat[2, 3] = 3.5  # Translation z
        t = Pose.from_matrix(mat)
        np.testing.assert_array_almost_equal(t.position, [1.5, 2.5, 3.5])

    def test_from_matrix_position(self):
        """Test Pose.from_matrix_position factory method."""
        # Identity rotation
        rot_mat = np.eye(3)
        pos = [1, 2, 3]
        t = Pose.from_matrix_position(rot_mat, pos)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        # Check rotation is identity
        np.testing.assert_array_almost_equal(t.rotation_matrix, np.eye(3))


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

    def test_robot_state_as_dict(self, robot):
        """Test RobotState.as_dict() conversion."""
        robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})
        state = robot.get_state(["joint_1", "joint_2"])

        d = state.as_dict()
        assert d["joint_1"] == pytest.approx(0.5)
        assert d["joint_2"] == pytest.approx(-0.3)

    def test_robot_state_getitem(self, robot):
        """Test RobotState bracket access."""
        robot.set_joints({"joint_1": 0.7})
        state = robot.get_state(["joint_1"])

        assert state["joint_1"] == pytest.approx(0.7)

    def test_robot_state_getitem_missing(self, robot):
        """Test RobotState bracket access raises KeyError for missing joint."""
        state = robot.get_state(["joint_1"])

        with pytest.raises(KeyError, match="not found"):
            _ = state["nonexistent_joint"]

    def test_robot_state_repr(self, robot):
        """Test RobotState string representation."""
        robot.set_joints({"joint_1": 0.1234})
        state = robot.get_state(["joint_1"])

        repr_str = repr(state)
        assert "RobotState" in repr_str
        assert "joint_1" in repr_str
        assert "0.1234" in repr_str

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
        np.testing.assert_array_almost_equal(result_pose.position, target_pose.position, decimal=4)

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
        np.testing.assert_array_almost_equal(result_pose.position, target_pose.position, decimal=4)

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
        np.testing.assert_array_almost_equal(result_pose.position, target_pose.position, decimal=4)

    def test_ik_all_solutions(self, robot):
        """Test IK returns multiple solutions for OPW solver (ABB IRB2400)."""
        # ABB IRB2400 uses OPW solver which can return up to 8 analytical solutions
        # Get a reachable pose in workspace interior
        test_joints = [0.0, 0.3, -0.3, 0.0, 0.5, 0.0]
        target_pose = robot.fk("manipulator", test_joints)

        # Get all IK solutions
        solutions = robot.ik("manipulator", target_pose, seed=test_joints, all_solutions=True)

        # OPW typically returns multiple solutions for 6-DOF industrial robots
        assert solutions is not None
        assert len(solutions) >= 1  # at least one solution

        # Verify each solution produces correct end-effector pose
        for sol in solutions:
            result_pose = robot.fk("manipulator", sol)
            np.testing.assert_array_almost_equal(
                result_pose.position, target_pose.position, decimal=3
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

    def test_add_targets_batch(self):
        """Test add_targets for adding multiple targets at once."""
        targets = [
            JointTarget([0, 0, 0, 0, 0, 0]),
            CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)),
            JointTarget([0.5, 0, 0, 0, 0, 0]),
        ]
        program = MotionProgram("manipulator").add_targets(targets)
        assert len(program) == 3
        assert program.targets == targets

    def test_repr(self):
        """Test MotionProgram string representation."""
        program = MotionProgram("my_group")
        program.move_to(JointTarget([0, 0, 0]))
        program.move_to(JointTarget([1, 1, 1]))

        repr_str = repr(program)
        assert "MotionProgram" in repr_str
        assert "my_group" in repr_str
        assert "2" in repr_str  # targets count

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

    def test_cone(self):
        from tesseract_robotics.planning.geometry import cone

        c = cone(0.5, 1.0)
        assert c is not None

    def test_create_link_with_geometry(self):
        """Test create_link_with_geometry helper."""
        from tesseract_robotics.planning.geometry import create_link_with_geometry

        link = create_link_with_geometry(
            name="test_link",
            geometry=box(0.1, 0.1, 0.1),
            origin=Pose.from_xyz(0.5, 0, 0.3),
            color=(1.0, 0, 0, 1.0),
        )
        assert link is not None
        assert link.getName() == "test_link"
        # Link should have visual and collision
        assert len(link.visual) > 0
        assert len(link.collision) > 0

    def test_mesh_from_file(self):
        """Test mesh_from_file loads STL mesh."""
        from tesseract_robotics.planning.geometry import mesh_from_file

        # Use mesh from tesseract_support package
        mesh = mesh_from_file(
            "package://tesseract_support/meshes/car_seat/visual/end_effector_open.stl"
        )
        assert mesh is not None

    def test_convex_mesh_from_file(self):
        """Test convex_mesh_from_file loads and converts mesh."""
        from tesseract_robotics.planning.geometry import convex_mesh_from_file

        # Use mesh from tesseract_support package
        mesh = convex_mesh_from_file(
            "package://tesseract_support/meshes/car_seat/visual/end_effector_open.stl"
        )
        assert mesh is not None


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
        from tesseract_robotics.tesseract_geometry import Box
        from tesseract_robotics.tesseract_scene_graph import (
            Collision,
            Joint,
            JointType,
            Link,
            Visual,
        )

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
        from tesseract_robotics.planning import create_fixed_joint
        from tesseract_robotics.tesseract_geometry import Sphere
        from tesseract_robotics.tesseract_scene_graph import Collision, Link, Visual

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

    def test_move_link(self, robot):
        """Test relocating a link to different parent."""
        from tesseract_robotics.tesseract_geometry import Box
        from tesseract_robotics.tesseract_scene_graph import (
            Collision,
            Joint,
            JointType,
            Link,
            Visual,
        )

        # First add a link attached to base_link
        link = Link("moveable_box")
        geometry = Box(0.1, 0.1, 0.1)
        visual = Visual()
        visual.geometry = geometry
        link.visual.append(visual)
        collision = Collision()
        collision.geometry = geometry
        link.collision.append(collision)

        joint = Joint("moveable_box_joint")
        joint.parent_link_name = "base_link"
        joint.child_link_name = "moveable_box"
        joint.type = JointType.FIXED

        robot.add_link(link, joint)
        assert "moveable_box" in robot.get_link_names()

        # Now move it to a different parent (link_3)
        new_joint = Joint("moveable_box_joint")
        new_joint.parent_link_name = "link_3"
        new_joint.child_link_name = "moveable_box"
        new_joint.type = JointType.FIXED

        result = robot.move_link(new_joint)
        assert result is True

    def test_scene_graph_property(self, robot):
        """Test direct scene graph access."""
        sg = robot.scene_graph
        assert sg is not None
        # Verify can query links via scene graph
        links = sg.getLinks()
        assert len(links) > 0

    def test_set_collision_margin(self, robot):
        """Test collision margin configuration."""
        result = robot.set_collision_margin(0.01)
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

    def test_plan_freespace_with_config(self, robot):
        """Test freespace planning with PlannerConfig."""
        from tesseract_robotics.planning import plan_freespace
        from tesseract_robotics.planning.planner import PlannerConfig

        config = PlannerConfig(
            pipeline="TrajOptPipeline",
            max_velocity_scaling=0.5,
            collision_safety_margin=0.03,
        )

        joint_names = robot.get_joint_names("manipulator")
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.3, 0, 0, 0, 0, 0]))
        )

        result = plan_freespace(robot, program, config=config)

        assert result.successful

    def test_assign_current_state_as_seed(self, robot):
        """Test assign_current_state_as_seed function."""
        from tesseract_robotics.planning import assign_current_state_as_seed

        joint_names = robot.get_joint_names("manipulator")

        # Set robot to a known state
        test_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robot.set_joints(test_joints, joint_names=joint_names)

        # Build program with Cartesian target (needs seed)
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(CartesianTarget(Pose.from_xyz(0.8, 0, 0.8)))
        )

        composite = program.to_composite_instruction(joint_names=joint_names)

        # Should not raise - just sets seed on Cartesian waypoints
        assign_current_state_as_seed(composite, robot)


class TestTaskComposer:
    """Tests for TaskComposer methods."""

    @pytest.fixture
    def robot(self):
        return Robot.from_tesseract_support("abb_irb2400")

    def test_get_available_pipelines(self):
        """Test get_available_pipelines returns list of pipeline names."""
        from tesseract_robotics.planning import TaskComposer

        composer = TaskComposer.from_config()
        pipelines = composer.get_available_pipelines()

        assert isinstance(pipelines, list)
        assert len(pipelines) > 0
        assert "TrajOptPipeline" in pipelines

    def test_plan_invalid_pipeline(self, robot):
        """Test plan returns failure for invalid pipeline."""
        from tesseract_robotics.planning import TaskComposer

        joint_names = robot.get_joint_names("manipulator")
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )

        composer = TaskComposer.from_config()
        result = composer.plan(robot, program, pipeline="NonexistentPipeline")

        assert not result.successful
        assert "not found" in result.message.lower() or "error" in result.message.lower()

    @pytest.mark.skip(reason="OMPLPipeline input key configuration differs from TrajOptPipeline")
    def test_plan_ompl(self, robot):
        """Test plan_ompl convenience method."""
        from tesseract_robotics.planning import plan_ompl

        joint_names = robot.get_joint_names("manipulator")
        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.3, 0, 0, 0, 0, 0]))
        )

        result = plan_ompl(robot, program)

        assert result.successful
        assert len(result) > 0

    @pytest.mark.skip(reason="DescartesPipeline not available in default config")
    def test_plan_cartesian(self, robot):
        """Test plan_cartesian convenience method."""
        from tesseract_robotics.planning import plan_cartesian

        joint_names = robot.get_joint_names("manipulator")

        # Get start pose for Cartesian planning
        start_joints = [0, 0.3, -0.3, 0, 0.5, 0]
        robot.set_joints(start_joints, joint_names=joint_names)
        start_pose = robot.fk("manipulator", start_joints)

        # Create small linear motion
        end_pose = Pose.from_xyz(start_pose.x, start_pose.y + 0.05, start_pose.z)

        program = (
            MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget(start_joints))
            .linear_to(CartesianTarget(end_pose))
        )

        result = plan_cartesian(robot, program)

        assert result.successful
        assert len(result) > 0


class TestProfileCreation:
    """Tests for profile factory functions."""

    def test_create_trajopt_default_profiles(self):
        """Test TrajOpt profile creation with defaults."""
        from tesseract_robotics.planning import create_trajopt_default_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_trajopt_default_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_trajopt_default_profiles_custom_names(self):
        """Test TrajOpt profile creation with custom names."""
        from tesseract_robotics.planning import create_trajopt_default_profiles

        profiles = create_trajopt_default_profiles(profile_names=["MY_PROFILE"])

        assert profiles is not None

    def test_create_trajopt_ifopt_default_profiles(self):
        """Test TrajOptIfopt profile creation with defaults."""
        from tesseract_robotics.planning import create_trajopt_ifopt_default_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_trajopt_ifopt_default_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_ompl_default_profiles(self):
        """Test OMPL profile creation with defaults."""
        from tesseract_robotics.planning import create_ompl_default_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_ompl_default_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_ompl_default_profiles_custom_params(self):
        """Test OMPL profile creation with custom parameters."""
        from tesseract_robotics.planning import create_ompl_default_profiles

        profiles = create_ompl_default_profiles(
            planning_time=10.0,
            max_solutions=5,
            optimize=False,
            num_planners=2,
        )

        assert profiles is not None

    def test_create_descartes_default_profiles(self):
        """Test Descartes profile creation with defaults."""
        from tesseract_robotics.planning import create_descartes_default_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_descartes_default_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_descartes_default_profiles_custom_params(self):
        """Test Descartes profile creation with edge collision enabled."""
        from tesseract_robotics.planning import create_descartes_default_profiles

        profiles = create_descartes_default_profiles(
            enable_edge_collision=True,
            num_threads=4,
        )

        assert profiles is not None

    def test_create_freespace_pipeline_profiles(self):
        """Test freespace pipeline profile creation (OMPL + TrajOpt)."""
        from tesseract_robotics.planning import create_freespace_pipeline_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_freespace_pipeline_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_cartesian_pipeline_profiles(self):
        """Test cartesian pipeline profile creation (Descartes + TrajOpt)."""
        from tesseract_robotics.planning import create_cartesian_pipeline_profiles
        from tesseract_robotics.tesseract_command_language import ProfileDictionary

        profiles = create_cartesian_pipeline_profiles()

        assert isinstance(profiles, ProfileDictionary)

    def test_create_ompl_planner_configurators_default(self):
        """Test OMPL configurator creation with defaults."""
        from tesseract_robotics.planning import create_ompl_planner_configurators

        configurators = create_ompl_planner_configurators()

        assert len(configurators) == 1  # default: 1 RRTConnect

    def test_create_ompl_planner_configurators_multiple(self):
        """Test OMPL configurator creation with multiple planners."""
        from tesseract_robotics.planning import create_ompl_planner_configurators

        configurators = create_ompl_planner_configurators(
            planners=["RRTConnect", "RRTstar"],
            num_planners=2,
        )

        assert len(configurators) == 4  # 2 types * 2 instances

    def test_create_ompl_planner_configurators_invalid_planner(self):
        """Test OMPL configurator raises ValueError for invalid planner."""
        from tesseract_robotics.planning import create_ompl_planner_configurators

        with pytest.raises(ValueError, match="Unknown planner"):
            create_ompl_planner_configurators(planners=["InvalidPlanner"])

    def test_create_time_optimal_parameterization(self):
        """Test TOTG parameterization creation."""
        from tesseract_robotics.planning import create_time_optimal_parameterization

        totg = create_time_optimal_parameterization(path_tolerance=0.05)

        assert totg is not None

    def test_create_iterative_spline_parameterization(self):
        """Test ISP parameterization creation."""
        from tesseract_robotics.planning import create_iterative_spline_parameterization

        isp = create_iterative_spline_parameterization(add_points=False)

        assert isp is not None


class TestProgramErrors:
    """Tests for error cases in program building."""

    def test_cartesian_target_requires_pose_or_position(self):
        """Test CartesianTarget raises ValueError when neither pose nor position given."""
        with pytest.raises(ValueError, match="requires 'pose' or 'position'"):
            CartesianTarget()

    def test_joint_target_requires_names(self):
        """Test JointTarget.to_waypoint raises ValueError without joint names."""
        target = JointTarget([0, 0, 0, 0, 0, 0])

        with pytest.raises(ValueError, match="requires joint names"):
            target.to_waypoint()

    def test_state_target_requires_names(self):
        """Test StateTarget.to_waypoint raises ValueError without joint names."""
        from tesseract_robotics.planning import StateTarget

        target = StateTarget([0, 0, 0, 0, 0, 0])

        with pytest.raises(ValueError, match="requires joint names"):
            target.to_waypoint()

    def test_motion_program_empty_targets_error(self):
        """Test MotionProgram.to_composite_instruction raises ValueError on empty."""
        program = MotionProgram("manipulator")

        with pytest.raises(ValueError, match="has no targets"):
            program.to_composite_instruction(joint_names=["j1", "j2"])


class TestTrajectoryPoint:
    """Tests for TrajectoryPoint dataclass."""

    def test_creation(self):
        """Test basic TrajectoryPoint creation."""
        from tesseract_robotics.planning import TrajectoryPoint

        point = TrajectoryPoint(
            joint_names=["j1", "j2", "j3"],
            positions=np.array([0.1, 0.2, 0.3]),
        )
        assert len(point.joint_names) == 3
        np.testing.assert_array_almost_equal(point.positions, [0.1, 0.2, 0.3])
        assert point.velocities is None
        assert point.accelerations is None
        assert point.time is None

    def test_with_velocities_and_time(self):
        """Test TrajectoryPoint with optional fields."""
        from tesseract_robotics.planning import TrajectoryPoint

        point = TrajectoryPoint(
            joint_names=["j1", "j2"],
            positions=np.array([0.5, 0.6]),
            velocities=np.array([0.1, 0.2]),
            accelerations=np.array([0.01, 0.02]),
            time=1.5,
        )
        np.testing.assert_array_almost_equal(point.velocities, [0.1, 0.2])
        np.testing.assert_array_almost_equal(point.accelerations, [0.01, 0.02])
        assert point.time == 1.5

    def test_as_dict(self):
        """Test TrajectoryPoint.as_dict() conversion."""
        from tesseract_robotics.planning import TrajectoryPoint

        point = TrajectoryPoint(
            joint_names=["joint_1", "joint_2"],
            positions=np.array([0.5, -0.3]),
        )
        d = point.as_dict()
        assert d["joint_1"] == pytest.approx(0.5)
        assert d["joint_2"] == pytest.approx(-0.3)

    def test_repr(self):
        """Test TrajectoryPoint string representation."""
        from tesseract_robotics.planning import TrajectoryPoint

        point = TrajectoryPoint(
            joint_names=["j1"],
            positions=np.array([0.1234]),
            time=2.5,
        )
        repr_str = repr(point)
        assert "0.1234" in repr_str
        assert "time=2.500" in repr_str


class TestPlanningResult:
    """Tests for PlanningResult dataclass."""

    def test_successful_result(self):
        """Test successful PlanningResult."""
        from tesseract_robotics.planning import TrajectoryPoint
        from tesseract_robotics.planning.composer import PlanningResult

        traj = [
            TrajectoryPoint(["j1"], np.array([0.0])),
            TrajectoryPoint(["j1"], np.array([0.5])),
            TrajectoryPoint(["j1"], np.array([1.0])),
        ]
        result = PlanningResult(
            successful=True,
            message="Planning successful",
            trajectory=traj,
        )

        assert result.successful
        assert bool(result) is True
        assert len(result) == 3
        assert result.message == "Planning successful"

    def test_failed_result(self):
        """Test failed PlanningResult."""
        from tesseract_robotics.planning.composer import PlanningResult

        result = PlanningResult(
            successful=False,
            message="No solution found",
        )

        assert not result.successful
        assert bool(result) is False
        assert len(result) == 0

    def test_iteration(self):
        """Test iterating over PlanningResult trajectory."""
        from tesseract_robotics.planning import TrajectoryPoint
        from tesseract_robotics.planning.composer import PlanningResult

        traj = [
            TrajectoryPoint(["j1"], np.array([0.1])),
            TrajectoryPoint(["j1"], np.array([0.2])),
        ]
        result = PlanningResult(successful=True, trajectory=traj)

        positions = [pt.positions[0] for pt in result]
        np.testing.assert_array_almost_equal(positions, [0.1, 0.2])

    def test_indexing(self):
        """Test indexing PlanningResult trajectory."""
        from tesseract_robotics.planning import TrajectoryPoint
        from tesseract_robotics.planning.composer import PlanningResult

        traj = [
            TrajectoryPoint(["j1"], np.array([0.0])),
            TrajectoryPoint(["j1"], np.array([0.5])),
        ]
        result = PlanningResult(successful=True, trajectory=traj)

        assert result[0].positions[0] == pytest.approx(0.0)
        assert result[1].positions[0] == pytest.approx(0.5)

    def test_to_numpy(self):
        """Test PlanningResult.to_numpy() conversion."""
        from tesseract_robotics.planning import TrajectoryPoint
        from tesseract_robotics.planning.composer import PlanningResult

        traj = [
            TrajectoryPoint(["j1", "j2"], np.array([0.0, 0.1])),
            TrajectoryPoint(["j1", "j2"], np.array([0.5, 0.6])),
            TrajectoryPoint(["j1", "j2"], np.array([1.0, 1.1])),
        ]
        result = PlanningResult(successful=True, trajectory=traj)

        arr = result.to_numpy()
        assert arr.shape == (3, 2)
        np.testing.assert_array_almost_equal(arr[0], [0.0, 0.1])
        np.testing.assert_array_almost_equal(arr[2], [1.0, 1.1])

    def test_to_numpy_empty(self):
        """Test to_numpy() on empty trajectory."""
        from tesseract_robotics.planning.composer import PlanningResult

        result = PlanningResult(successful=False)
        arr = result.to_numpy()
        assert len(arr) == 0

    def test_negative_indexing(self):
        """Test negative indexing on PlanningResult trajectory."""
        from tesseract_robotics.planning import TrajectoryPoint
        from tesseract_robotics.planning.composer import PlanningResult

        traj = [
            TrajectoryPoint(["j1"], np.array([0.0])),
            TrajectoryPoint(["j1"], np.array([0.5])),
            TrajectoryPoint(["j1"], np.array([1.0])),
        ]
        result = PlanningResult(successful=True, trajectory=traj)

        # Negative indexing should work
        assert result[-1].positions[0] == pytest.approx(1.0)
        assert result[-2].positions[0] == pytest.approx(0.5)
        assert result[-3].positions[0] == pytest.approx(0.0)
