"""drive(): exhaustive dispatch of an IR event stream onto a backend."""

from __future__ import annotations

import numpy as np

from tesseract_robotics.emitters.core.backend import ProgramBackend, drive
from tesseract_robotics.emitters.core.events import (
    Dwell,
    JointMove,
    Note,
    ProgramIR,
    SetDigital,
)


class _RecordingBackend:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def prog_start(self, name: str) -> None:
        self.calls.append(f"start:{name}")

    def move_joint(self, m: JointMove) -> None:
        self.calls.append("joint")

    def move_cartesian(self, m) -> None:
        self.calls.append("cart")

    def dwell(self, e: Dwell) -> None:
        self.calls.append(f"dwell:{e.seconds}")

    def wait_digital(self, e) -> None:
        self.calls.append("wait")

    def set_digital(self, e: SetDigital) -> None:
        self.calls.append(f"do:{e.key}{e.index}={e.value}")

    def set_analog(self, e) -> None:
        self.calls.append("ao")

    def tool_change(self, e) -> None:
        self.calls.append("tool")

    def note(self, e: Note) -> None:
        self.calls.append(f"note:{e.text}")

    def prog_finish(self) -> dict[str, str]:
        self.calls.append("finish")
        return {"OUT.src": "\n".join(self.calls)}


def test_drive_dispatches_in_order_and_returns_files() -> None:
    ir = ProgramIR(
        name="P",
        events=(
            Note("hi"),
            JointMove(joints=tuple(np.zeros(6)), profile="J"),
            Dwell(seconds=2.0),
            SetDigital(key="do", index=3, value=True),
        ),
    )
    backend = _RecordingBackend()
    files = drive(ir, backend)
    assert backend.calls == ["start:P", "note:hi", "joint", "dwell:2.0", "do:do3=True", "finish"]
    assert "OUT.src" in files


def test_recording_backend_satisfies_protocol() -> None:
    assert isinstance(_RecordingBackend(), ProgramBackend)
