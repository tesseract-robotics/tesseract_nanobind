"""``RapidBackend`` — consume the core IR, emit RAPID via the existing DSL.

Drives the ``rapid_writer`` ``MODULE``/``PROC`` DSL so output is byte-identical
to the pre-reseat inline dispatcher (locked by ``tests/emitters/rapid/
test_dispatcher.py``). It owns RAPID signal naming: a ``SetDigital`` with
``key is None`` renders ``do<index>`` (the Timer convention), a named key uses
``key`` (index 0) or ``key{index}`` (array subscript).
"""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from ..core.backend import ProgramBackend
from ..core.errors import MissingProfileError
from ..core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    Note,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)
from .rapid_writer import (
    Comment,
    JointTarget,
    Module,
    MoveAbsJ,
    MoveJ,
    MoveL,
    Proc,
    RapidProfile,
    RapidTarget,
    RapidWriter,
    SetAO,
    SetDO,
    WaitDI,
    WaitDO,
    WaitTime,
)


def _rapid_signal_name(key: str, index: int) -> str:
    """Named-signal RAPID rendering: bare ``key`` at index 0, else ``key{index}``."""
    return key if index == 0 else f"{key}{{{index}}}"


class RapidBackend(ProgramBackend):
    """Stateful RAPID backend; drives the RapidWriter singleton (clears on start)."""

    def __init__(
        self, *, profiles: Mapping[str, RapidProfile], module_name: str, proc_name: str
    ) -> None:
        self._profiles = profiles
        self._module_name = module_name
        self._proc_name = proc_name
        self._rapid = RapidWriter()
        self._module: Module | None = None
        self._proc: Proc | None = None

    def _profile(self, name: str) -> RapidProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self) -> None:
        self._rapid.clear()
        self._module = Module(self._module_name)
        self._module.__enter__()
        self._proc = Proc(self._proc_name)
        self._proc.__enter__()

    def move_joint(self, m: JointMove) -> None:
        MoveAbsJ(JointTarget(np.asarray(m.joints, dtype=np.float64)), self._profile(m.profile))

    def move_cartesian(self, m: CartesianMove) -> None:
        target = RapidTarget(pose=m.pose)
        profile = self._profile(m.profile)
        if m.kind is MoveKind.LINEAR:
            MoveL(target, profile)
        else:
            MoveJ(target, profile)

    def dwell(self, e: Dwell) -> None:
        WaitTime(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        signal = f"{'di' if e.is_input else 'do'}{e.index}"
        (WaitDI if e.is_input else WaitDO)(signal, value=e.value)

    def set_digital(self, e: SetDigital) -> None:
        signal = f"do{e.index}" if e.key is None else _rapid_signal_name(e.key, e.index)
        SetDO(signal, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        signal = f"ao{e.index}" if e.key is None else _rapid_signal_name(e.key, e.index)
        SetAO(signal, e.value)

    def tool_change(self, e: ToolChange) -> None:
        Comment(f"tool change to tool id {e.tool_id}")

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._proc is not None and self._module is not None
        self._proc.__exit__(None, None, None)
        self._module.__exit__(None, None, None)
        return {f"{self._module_name}.mod": self._rapid.getvalue()}
