"""Named failure modes of `tesseract_robotics.viewer`."""


class EmptyTrajectoryError(ValueError):
    """A trajectory has no waypoint the viewer can animate."""


class TrajectoryShapeError(ValueError):
    """Joint names, positions and times disagree in length, or times decrease."""


class UnplannedWaypointError(TypeError):
    """A program move still has a Cartesian waypoint: it was not planned."""


class UnknownJointError(KeyError):
    """A joint name is not part of the environment."""


class UnsupportedGeometryError(TypeError):
    """A visual's geometry type has no viewer representation."""


class ViewerCloseTimeoutError(RuntimeError):
    """The playback thread did not exit within `PLAYER_JOIN_TIMEOUT_S`."""


class NoEnvironmentError(RuntimeError):
    """A viewer call needs an environment; `update_environment` has not been called."""


class NoTrajectoryError(RuntimeError):
    """A viewer call needs a trajectory; `update_trajectory` has not been called."""


class ViewerPortInUseError(OSError):
    """The requested port is taken; viser would have moved to the next free one."""


class MalformedOctreeStreamError(ValueError):
    """An OctoMap `.bt` stream has no header it needs, or its tree is cut short."""


class StaleSceneError(RuntimeError):
    """The environment gained or lost links since `update_environment`."""


class UnsupportedMoveTypeError(TypeError):
    """A program move has a type whose TCP path the viewer cannot draw exactly (`CIRCULAR`)."""


class MovingWorkingFrameError(ValueError):
    """A MoveL's working frame moves with the group, so its line is not straight in world."""


class NamedTcpOffsetError(TypeError):
    """`ManipulatorInfo.tcp_offset` names a link; the path needs a transform."""
