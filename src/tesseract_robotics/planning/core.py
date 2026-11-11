"""
Core classes for robot environment management.

Provides high-level interfaces for loading and managing robot environments,
wrapping the lower-level Environment and related classes.

Example:
    ```python
    from tesseract_robotics import get_tesseract_resource_path
    from tesseract_robotics.common import GeneralResourceLocator
    from tesseract_robotics.planning import Robot

    # Load from the support package's URLs
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_urdf(
        "package://tesseract/support/urdf/abb_irb2400.urdf",
        "package://tesseract/support/urdf/abb_irb2400.srdf",
        locator,
    )

    # Access robot state
    state = robot.get_state()
    print(state.joint_positions)

    # Set joint positions
    robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})
    ```
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, overload

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.common import (
    AllowedCollisionMatrix,
    GeneralResourceLocator,
    Isometry3d,
    LinkId,
    ManipulatorInfo,
)
from tesseract_robotics.environment import (
    AddLinkCommand,
    ChangeCollisionMarginsCommand,
    Command,
    Environment,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    MoveLinkCommand,
    RemoveLinkCommand,
)
from tesseract_robotics.kinematics import KinGroupIKInput
from tesseract_robotics.planning.errors import (
    EnvironmentCommandError,
    EnvironmentInitError,
    IKNoSolutionError,
    InvalidEnvironmentError,
    JointCountMismatchError,
    JointNamesRequiredError,
    JointNotFoundError,
    ResourceNotFoundError,
    UninitializedEnvironmentError,
)
from tesseract_robotics.planning.transforms import Pose
from tesseract_robotics.scene_graph import (
    Joint,
    Link,
    SceneGraph,
)


@dataclass
class RobotState:
    """
    Snapshot of robot joint state.

    Attributes:
        joint_names: List of joint names
        joint_positions: Array of joint positions, one per name (rad revolute, m prismatic)
        joint_velocities: Array of joint velocities (rad/s revolute, m/s prismatic), optional
        joint_accelerations: Array of joint accelerations (rad/s^2 revolute, m/s^2 prismatic),
            optional

    Raises:
        JointCountMismatchError: `joint_positions` and `joint_names` differ in length.
    """

    joint_names: list[str]
    joint_positions: np.ndarray
    joint_velocities: np.ndarray | None = None
    joint_accelerations: np.ndarray | None = None

    def __post_init__(self) -> None:
        """Convert arrays to numpy float64 for C++ interop and check their lengths."""
        self.joint_positions = np.asarray(self.joint_positions, dtype=np.float64)
        if self.joint_positions.shape != (len(self.joint_names),):
            raise JointCountMismatchError(
                f"{len(self.joint_names)} joint names but joint_positions has shape "
                f"{self.joint_positions.shape}"
            )
        if self.joint_velocities is not None:
            self.joint_velocities = np.asarray(self.joint_velocities, dtype=np.float64)
        if self.joint_accelerations is not None:
            self.joint_accelerations = np.asarray(self.joint_accelerations, dtype=np.float64)

    def as_dict(self) -> dict[str, float]:
        """Return joint positions as {name: position} dictionary."""
        return dict(zip(self.joint_names, self.joint_positions))

    def __getitem__(self, joint_name: str) -> float:
        """Get position of a specific joint by name."""
        if joint_name not in self.joint_names:
            raise JointNotFoundError(f"Joint '{joint_name}' not found in {self.joint_names}")
        return float(self.joint_positions[self.joint_names.index(joint_name)])

    def __repr__(self) -> str:
        joints_str = ", ".join(
            f"{n}={p:.4f}" for n, p in zip(self.joint_names, self.joint_positions)
        )
        return f"RobotState({joints_str})"


class Robot:
    """
    High-level interface for robot environment management.

    Wraps the Tesseract Environment with a Pythonic interface for loading
    robots, managing state, and accessing kinematic information.

    Attributes:
        env: Underlying Tesseract Environment
        locator: Resource locator for resolving package:// URLs

    Example:
        robot = Robot.from_urdf(
            "package://my_robot/urdf/robot.urdf",
            "package://my_robot/urdf/robot.srdf"
        )

        # Get joint names for a manipulator
        joints = robot.get_joint_names("manipulator")

        # Compute forward kinematics
        pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0], "tool0")

        # Compute inverse kinematics
        solutions = robot.ik("manipulator", target_pose, "tool0")
    """

    def __init__(
        self,
        env: Environment,
        locator: GeneralResourceLocator | None = None,
    ):
        """
        Wrap an environment without checking it; use `Robot.build` to validate.

        Args:
            env: Initialized Tesseract Environment
            locator: Resource locator (created if not provided)
        """
        self.env = env
        self.locator = locator or GeneralResourceLocator()

    @classmethod
    def build(
        cls,
        env: Environment,
        locator: GeneralResourceLocator | None = None,
    ) -> Robot:
        """
        Wrap an environment after checking it is an initialised `Environment`.

        Args:
            env: Initialized Tesseract Environment
            locator: Resource locator (created if not provided)

        Returns:
            The wrapped robot.

        Raises:
            InvalidEnvironmentError: `env` is not an `Environment`.
            UninitializedEnvironmentError: `env` has not been initialised.
        """
        if not isinstance(env, Environment):
            raise InvalidEnvironmentError(
                f"Robot wraps a tesseract_robotics.environment.Environment, got {type(env).__name__}"
            )
        if not env.isInitialized():
            raise UninitializedEnvironmentError(
                "Environment is not initialised; call Environment.init(...) first, "
                "or load with Robot.from_urdf / Robot.from_files."
            )
        return cls(env, locator)

    @classmethod
    def from_urdf(
        cls,
        urdf_url: str,
        srdf_url: str,
        locator: GeneralResourceLocator | None = None,
    ) -> Robot:
        """
        Load robot from URDF and SRDF URLs.

        Supports both package:// URLs and file:// URLs.

        Args:
            urdf_url: URL to URDF file (e.g., "package://my_robot/urdf/robot.urdf")
            srdf_url: URL to SRDF file (e.g., "package://my_robot/urdf/robot.srdf")
            locator: Resource locator (uses GeneralResourceLocator if not provided)

        Returns:
            Initialized Robot instance

        Raises:
            ResourceNotFoundError: The locator resolves neither URL.
            EnvironmentInitError: The environment rejects the URDF / SRDF pair.
        """
        locator = locator or GeneralResourceLocator()

        urdf_resource = locator.locateResource(urdf_url)
        if urdf_resource is None:
            raise ResourceNotFoundError(f"Could not locate URDF resource: {urdf_url}")
        srdf_resource = locator.locateResource(srdf_url)
        if srdf_resource is None:
            raise ResourceNotFoundError(f"Could not locate SRDF resource: {srdf_url}")

        urdf_path = Path(urdf_resource.getFilePath())
        srdf_path = Path(srdf_resource.getFilePath())

        env = Environment()
        if not env.init(urdf_path, srdf_path, locator):
            raise EnvironmentInitError(f"Failed to initialize environment from {urdf_url}")

        return cls(env, locator)

    @classmethod
    def from_files(
        cls,
        urdf_path: str | Path,
        srdf_path: str | Path,
        locator: GeneralResourceLocator | None = None,
    ) -> Robot:
        """
        Load robot from local URDF and SRDF files.

        Args:
            urdf_path: Path to URDF file
            srdf_path: Path to SRDF file
            locator: Resource locator for resolving mesh references

        Returns:
            Initialized Robot instance

        Raises:
            ResourceNotFoundError: A file does not exist.
            EnvironmentInitError: The environment rejects the URDF / SRDF pair.
        """
        locator = locator or GeneralResourceLocator()

        urdf_path = Path(urdf_path)
        srdf_path = Path(srdf_path)
        for path in (urdf_path, srdf_path):
            if not path.is_file():
                raise ResourceNotFoundError(f"No such file: {path}")

        env = Environment()
        if not env.init(urdf_path, srdf_path, locator):
            raise EnvironmentInitError(f"Failed to initialize environment from {urdf_path}")

        return cls(env, locator)

    @classmethod
    def from_tesseract_support(
        cls,
        robot_name: str,
        locator: GeneralResourceLocator | None = None,
    ) -> Robot:
        """
        Load robot from tesseract's support package.

        Convenience method for loading standard test robots. `locator` must resolve
        `package://tesseract/support/...`, e.g. with the support data installed
        (`pip install 'tesseract-robotics-nanobind[support]'`):

        ```python
        locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
        ```

        Args:
            robot_name: Name of robot (e.g., "abb_irb2400", "lbr_iiwa_14_r820")
            locator: Resource locator (uses GeneralResourceLocator if not provided)

        Returns:
            Initialized Robot instance
        """
        urdf_url = f"package://tesseract/support/urdf/{robot_name}.urdf"
        srdf_url = f"package://tesseract/support/urdf/{robot_name}.srdf"
        return cls.from_urdf(urdf_url, srdf_url, locator)

    def get_state(self, joint_names: list[str] | None = None) -> RobotState:
        """
        Get current robot state.

        Args:
            joint_names: Specific joints to query (all if None)

        Returns:
            Current robot state
        """
        env_state = self.env.getState()

        if joint_names is None:
            # Get all joint names from the state's joints dict
            joint_names = [jid.name() for jid in env_state.joints]

        positions = env_state.getJointValues(joint_names)

        return RobotState(
            joint_names=joint_names,
            joint_positions=positions,
        )

    def set_joints(
        self,
        joint_values: dict[str, float] | ArrayLike,
        joint_names: list[str] | None = None,
    ) -> None:
        """
        Set robot joint positions.

        Args:
            joint_values: Joint positions as dict or array-like
            joint_names: Joint names (required if joint_values is sequence)

        Raises:
            JointNamesRequiredError: `joint_values` is a sequence and no `joint_names` given.
            JointCountMismatchError: `joint_values` and `joint_names` differ in length.
        """
        if isinstance(joint_values, dict):
            names = list(joint_values.keys())
            values = np.array(list(joint_values.values()), dtype=np.float64)
        else:
            if joint_names is None:
                raise JointNamesRequiredError(
                    "joint_names required when joint_values is a sequence"
                )
            names = joint_names
            values = np.asarray(joint_values, dtype=np.float64)
            if values.shape != (len(names),):
                raise JointCountMismatchError(
                    f"{len(names)} joint names but joint_values has shape {values.shape}"
                )

        self.env.setState(names, values)

    def get_joint_names(self, group_name: str) -> list[str]:
        """
        Get joint names for a kinematic group.

        Args:
            group_name: Name of the kinematic group (e.g., "manipulator")

        Returns:
            List of joint names
        """
        group = self.env.getJointGroup(group_name)
        return [jid.name() for jid in group.getJointIds()]

    def get_link_names(self) -> list[str]:
        """Get all link names in the robot."""
        return [lid.name() for lid in self.env.getLinkIds()]

    def get_joint_limits(self, group_name: str) -> dict[str, dict[str, float]]:
        """
        Get joint limits for a kinematic group.

        Args:
            group_name: Name of the kinematic group

        Returns:
            Dictionary mapping joint name to {lower, upper, velocity, acceleration}
        """
        group = self.env.getKinematicGroup(group_name)
        limits = group.getLimits()

        result = {}
        joint_names = [jid.name() for jid in group.getJointIds()]
        for i, name in enumerate(joint_names):
            result[name] = {
                "lower": float(limits.joint_limits[i, 0]),
                "upper": float(limits.joint_limits[i, 1]),
                "velocity": float(limits.velocity_limits[i, 1]),  # upper bound
                "acceleration": float(limits.acceleration_limits[i, 1]),  # upper bound
            }
        return result

    def fk(
        self,
        group_name: str,
        joint_positions: ArrayLike,
        tip_link: str,
    ) -> Pose:
        """
        Compute forward kinematics.

        Args:
            group_name: Kinematic group name
            joint_positions: Joint positions array
            tip_link: Link whose pose to return (e.g. "tool0"); a link, not a named TCP

        Returns:
            Pose of the tip link in the scene graph's root frame (what
            `KinematicGroup.calcFwdKin` reports, not relative to the group's base link; the
            two coincide when the base link is the root)
        """
        group = self.env.getKinematicGroup(group_name)
        joint_positions = np.asarray(joint_positions, dtype=np.float64)

        poses = group.calcFwdKin(joint_positions)

        return Pose(poses[LinkId(tip_link)])

    @overload
    def ik(
        self,
        group_name: str,
        target_pose: Pose | Isometry3d,
        tip_link: str,
        seed: ArrayLike | None = None,
        all_solutions: Literal[False] = False,
    ) -> np.ndarray: ...
    @overload
    def ik(
        self,
        group_name: str,
        target_pose: Pose | Isometry3d,
        tip_link: str,
        seed: ArrayLike | None = None,
        *,
        all_solutions: Literal[True],
    ) -> list[np.ndarray]: ...
    def ik(
        self,
        group_name: str,
        target_pose: Pose | Isometry3d,
        tip_link: str,
        seed: ArrayLike | None = None,
        all_solutions: bool = False,
    ) -> np.ndarray | list[np.ndarray]:
        """
        Compute inverse kinematics.

        Args:
            group_name: Kinematic group name
            target_pose: Desired pose of `tip_link`, relative to the group's base link
            tip_link: Link to solve for (e.g. "tool0"); a link, not a named TCP: strip a TCP
                offset first (`target * offset.inverse()`)
            seed: Initial joint configuration (current state if None)
            all_solutions: If True, return every solution; otherwise the first

        Returns:
            With `all_solutions=False`, the joint positions (rad) of the first solution; with
            `all_solutions=True`, the list of every solution's joint positions.

        Raises:
            IKNoSolutionError: The pose is unreachable from the group.
        """
        group = self.env.getKinematicGroup(group_name)

        if seed is None:
            seed = self.get_state([jid.name() for jid in group.getJointIds()]).joint_positions
        else:
            seed = np.asarray(seed, dtype=np.float64)

        # Get working frame from group
        working_frame = group.getBaseLinkId()

        # Create proper IK input
        ik_input = KinGroupIKInput(target_pose, working_frame, tip_link)

        result = group.calcInvKin(ik_input, seed)

        if len(result) == 0:
            raise IKNoSolutionError(
                f"No IK solution for tip link '{tip_link}' of group '{group_name}' at {target_pose}"
            )

        if all_solutions:
            return [np.asarray(sol) for sol in result]

        return np.asarray(result[0])

    def get_manipulator_info(
        self,
        group_name: str,
        tcp_frame: str,
        working_frame: str,
        *,
        tcp_offset: str | Isometry3d | None = None,
    ) -> ManipulatorInfo:
        """
        Build a ManipulatorInfo for a kinematic group.

        Every call returns a new object. The TCP is `tcp_frame * tcp_offset`; see
        how a named offset resolves: `Environment.findTCPOffset`.

        Args:
            group_name: Kinematic group name
            tcp_frame: Link the TCP is attached to (e.g. "tool0")
            working_frame: Link Cartesian waypoints are relative to (e.g. "base_link")
            tcp_offset: Offset from `tcp_frame`: a named TCP (SRDF `group_tcps` of this group,
                or a find-TCP callback), resolved by the environment at plan time, or an
                `Isometry3d`. None leaves upstream's default, identity.

        Returns:
            Configured ManipulatorInfo
        """
        info = ManipulatorInfo(group_name, working_frame, tcp_frame)
        if tcp_offset is not None:
            info.tcp_offset = tcp_offset
        return info

    def _apply(self, command: Command, what: str) -> None:
        """Apply an environment command; raise if the environment refuses it."""
        if not self.env.applyCommand(command):
            raise EnvironmentCommandError(f"Environment refused to {what}")

    def add_link(
        self,
        link: Link,
        joint: Joint,
    ) -> None:
        """
        Add a new link to the environment.

        Args:
            link: Link to add
            joint: Joint connecting link to parent

        Raises:
            EnvironmentCommandError: The environment refused the link (duplicate name, unknown
                parent link).
        """
        self._apply(AddLinkCommand(link, joint), f"add link '{link.getName()}'")

    def remove_link(self, link_name: str) -> None:
        """
        Remove a link from the environment.

        Args:
            link_name: Name of link to remove

        Raises:
            EnvironmentCommandError: The environment has no such link.
        """
        self._apply(RemoveLinkCommand(link_name), f"remove link '{link_name}'")

    def move_link(
        self,
        joint: Joint,
    ) -> None:
        """
        Move a link by providing a new joint configuration.

        This attaches the child link to a new parent link via the joint.

        Args:
            joint: Joint object specifying new parent-child relationship

        Raises:
            EnvironmentCommandError: The environment refused the move.
        """
        self._apply(MoveLinkCommand(joint), f"move link of joint '{joint.getName()}'")

    def add_allowed_collision(
        self,
        link1: str,
        link2: str,
        reason: str = "Adjacent",
    ) -> None:
        """
        Add an allowed collision pair.

        Args:
            link1: First link name
            link2: Second link name
            reason: Reason for allowing collision

        Raises:
            EnvironmentCommandError: The environment refused the pair.
        """
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision(link1, link2, reason)
        self._apply(
            ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD),
            f"allow collisions between '{link1}' and '{link2}'",
        )

    def set_collision_margin(self, margin: float) -> None:
        """
        Set default collision margin distance.

        Args:
            margin: Collision margin in meters

        Raises:
            EnvironmentCommandError: The environment refused the margin.
        """
        self._apply(ChangeCollisionMarginsCommand(margin), f"set collision margin {margin}")

    @property
    def scene_graph(self) -> SceneGraph:
        """Access the underlying scene graph."""
        return self.env.getSceneGraph()

    def __repr__(self) -> str:
        links = len(self.get_link_names())
        return f"Robot(links={links})"
