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
