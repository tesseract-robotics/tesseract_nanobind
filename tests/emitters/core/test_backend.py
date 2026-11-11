"""drive(): exhaustive dispatch of an IR event stream onto a backend, inside its session."""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager

import pytest

from tesseract_robotics.emitters.core.backend import ProgramBackend, drive
from tesseract_robotics.emitters.core.events import (
    Dwell,
    JointMove,
    Note,
    ProgramIR,
    SetDigital,
)
from tests.emitters.programs import joint_move


class _RecordingBackend(ProgramBackend):
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.open = False

    @contextmanager
    def _session(self):  # noqa: ANN202
        self.open = True
        try:
            yield
        finally:
            self.open = False

    def session(self) -> AbstractContextManager[object]:
        return self._session()

    def prog_start(self) -> None:
        self.calls.append("start")

    def move_joint(self, m: JointMove) -> None:
        self.calls.append("joint")

    def move_cartesian(self, m) -> None:  # noqa: ANN001
        self.calls.append("cart")

    def dwell(self, e: Dwell) -> None:
        if e.seconds < 0:
            raise RuntimeError("boom")
        self.calls.append(f"dwell:{e.seconds}")

    def wait_digital(self, e) -> None:  # noqa: ANN001
        self.calls.append("wait")

    def set_digital(self, e: SetDigital) -> None:
        self.calls.append(f"do:{e.key}{e.index}={e.value}")

    def set_analog(self, e) -> None:  # noqa: ANN001
        self.calls.append("ao")

    def tool_change(self, e) -> None:  # noqa: ANN001
        self.calls.append("tool")

    def note(self, e: Note) -> None:
        assert self.open
        self.calls.append(f"note:{e.text}")

    def prog_finish(self) -> dict[str, str]:
        self.calls.append("finish")
        return {"OUT.src": "\n".join(self.calls)}


def test_drive_dispatches_in_order_and_returns_files() -> None:
    ir = ProgramIR(
        name="P",
        events=(
            Note("hi"),
            joint_move([0, 0, 0, 0, 0, 0]),
            Dwell(seconds=2.0),
            SetDigital(key="do", index=3, value=True),
        ),
    )
    backend = _RecordingBackend()
    files = drive(ir, backend)
    assert backend.calls == ["start", "note:hi", "joint", "dwell:2.0", "do:do3=True", "finish"]
    assert "OUT.src" in files and not backend.open


def test_session_closes_when_a_handler_raises() -> None:
    backend = _RecordingBackend()
    with pytest.raises(RuntimeError, match="boom"):
        drive(ProgramIR(name="P", events=(Dwell(seconds=-1.0),)), backend)
    assert not backend.open
