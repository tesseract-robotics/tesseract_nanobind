"""Frame-tagged poses and the binding from tesseract frames to controller frames.

A tesseract Cartesian waypoint is the pose of `tcp_frame` (composed with
`tcp_offset`) expressed in `working_frame`, both taken from the move's
`ManipulatorInfo`. Dropping those frames and emitting the bare pose would make
the controller interpret it against whatever work object and tool happen to be
active: a different motion than the plan. So every move carries its
`MoveFrames`, Cartesian targets are `FramedPose`s, and each backend resolves the
frames through a `FrameBinding` the caller supplies: an explicit map from
tesseract frame names to the controller's own frame handles (RAPID `wobjdata`
names, KRL `BASE_DATA` indices, Fanuc user-frame numbers, URScript feature
poses, …). A frame with no binding raises `UnboundFrameError`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from tesseract_robotics.planning.transforms import Pose

from .errors import TcpOffsetError, UnboundFrameError


@dataclass(frozen=True)
class MoveFrames:
    """The frames a move is expressed in (from the move's combined `ManipulatorInfo`).

    Attributes:
        working_frame: Link the Cartesian target is expressed in.
        tcp_frame: Link whose (offset) origin the target positions.
        tcp_offset: Pose of the TCP in `tcp_frame` (identity when there is none).
    """

    working_frame: str
    tcp_frame: str
    tcp_offset: Pose

    @property
    def has_tcp_offset(self) -> bool:
        """Whether `tcp_offset` differs from the identity (exact comparison)."""
        return not np.array_equal(np.asarray(self.tcp_offset.matrix), np.eye(4))

    def canonical(self) -> str:
        """Deterministic text form for the content digest."""
        rows = np.asarray(self.tcp_offset.matrix).round(9).tolist()
        return f"MoveFrames({self.working_frame!r},{self.tcp_frame!r},{rows})"

    def require_no_offset(self, brand: str) -> None:
        """Raise unless `tcp_offset` is the identity.

        Controllers that select a stored tool by handle cannot add an extra
        offset on top of it, so the offset must already be part of the bound tool.

        Raises:
            TcpOffsetError: `tcp_offset` is not the identity.
        """
        if self.has_tcp_offset:
            raise TcpOffsetError(
                brand,
                f"tcp_frame {self.tcp_frame!r} carries a non-identity tcp_offset; "
                "bind a controller tool that already includes it and plan with the "
                "offset folded into a TCP link",
            )


@dataclass(frozen=True)
class FramedPose:
    """A Cartesian target: the pose of the move's TCP, expressed in its working frame.

    Attributes:
        pose: Pose of `frames.tcp_frame` composed with `frames.tcp_offset`, in
            `frames.working_frame` (metres, scalar-last quaternion).
        frames: The frames `pose` is expressed in.
    """

    pose: Pose
    frames: MoveFrames


@dataclass(frozen=True)
class FrameBinding[W, T]:
    """Map from tesseract frame names to one controller's frame handles.

    Build with `FrameBinding.build`; the brand packages validate the handle
    ranges before calling it.

    Attributes:
        work_frames: Working-frame link name to controller work-frame handle.
        tools: TCP link name to controller tool handle.
    """

    work_frames: Mapping[str, W]
    tools: Mapping[str, T]

    @classmethod
    def build(cls, *, work_frames: Mapping[str, W], tools: Mapping[str, T]) -> FrameBinding[W, T]:
        """Freeze the two maps (read-only copies) into a binding."""
        return cls(MappingProxyType(dict(work_frames)), MappingProxyType(dict(tools)))

    def work(self, frames: MoveFrames, brand: str) -> W:
        """The controller work frame for `frames.working_frame`.

        Raises:
            UnboundFrameError: The working frame is not in `work_frames`.
        """
        try:
            return self.work_frames[frames.working_frame]
        except KeyError:
            raise UnboundFrameError(
                frames.working_frame, "working", brand, list(self.work_frames)
            ) from None

    def tool(self, frames: MoveFrames, brand: str) -> T:
        """The controller tool for `frames.tcp_frame`.

        Raises:
            UnboundFrameError: The TCP frame is not in `tools`.
        """
        try:
            return self.tools[frames.tcp_frame]
        except KeyError:
            raise UnboundFrameError(frames.tcp_frame, "tcp", brand, list(self.tools)) from None
