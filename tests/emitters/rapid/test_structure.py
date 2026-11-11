"""The RAPID DSL cannot produce a structurally invalid module."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import (
    InvalidDeclarationError,
    ProgramStructureError,
    UndeclaredVariableError,
    UnemittableTextError,
)
from tesseract_robotics.emitters.rapid.rapid_writer import (
    AssignVariable,
    Comment,
    Module,
    Pos,
    Proc,
    RapidBool,
    RapidWriter,
    Stop,
    Tooldata,
    TPWrite,
    While,
)
from tesseract_robotics.emitters.rapid.syntax import RapidType
from tesseract_robotics.planning import Pose


def test_proc_inside_while_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"), While("TRUE"):
        with pytest.raises(ProgramStructureError, match="PROC"):
            Proc("inner")


def test_nested_module_raises() -> None:
    with RapidWriter(), Module("M"):
        with pytest.raises(ProgramStructureError, match="MODULE"):
            Module("N")


def test_while_outside_routine_raises() -> None:
    with RapidWriter(), Module("M"):
        with pytest.raises(ProgramStructureError, match="WHILE"):
            While("TRUE")


def test_statement_at_module_level_raises() -> None:
    with RapidWriter(), Module("M"):
        with pytest.raises(ProgramStructureError, match="Stop"):
            Stop()


def test_pers_inside_routine_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"):
        with pytest.raises(ProgramStructureError, match="PERS bool flag"):
            RapidBool("flag", True, RapidType.PERS)


def test_tooldata_inside_routine_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"):
        with pytest.raises(ProgramStructureError, match="tooldata"):
            Tooldata("gun", Pose(), 1.0, [0, 0, 0.1])


def test_declaration_after_statement_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"):
        Stop()
        with pytest.raises(ProgramStructureError, match="after the routine's first statement"):
            Pos("p", [0, 0, 0])


def test_declaration_inside_loop_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"), While("TRUE"):
        with pytest.raises(ProgramStructureError):
            RapidBool("flag", True)


def test_comment_before_declaration_is_fine() -> None:
    with RapidWriter() as w, Module("M"), Proc("main"):
        Comment("data")
        RapidBool("flag", True)
    assert "VAR bool flag := TRUE;" in w.getvalue()


def test_assign_const_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"):
        flag = RapidBool("flag", True, RapidType.CONST)
        with pytest.raises(UndeclaredVariableError, match="CONST"):
            AssignVariable(flag, "FALSE")


def test_assign_from_other_writer_raises() -> None:
    with RapidWriter(), Module("M"), Proc("main"):
        flag = RapidBool("flag", True)
    with RapidWriter(), Module("N"), Proc("main"):
        with pytest.raises(UndeclaredVariableError, match="not declared"):
            AssignVariable(flag, "FALSE")


def test_duplicate_declaration_raises() -> None:
    with RapidWriter(), Module("M"):
        Pos("p", [0, 0, 0])
        with pytest.raises(ProgramStructureError, match="second declaration"):
            Pos("P", [0, 0, 0])


def test_comment_with_newline_raises() -> None:
    with RapidWriter(), Module("M"):
        with pytest.raises(UnemittableTextError):
            Comment("ok\nStop;")


def test_tpwrite_escapes_and_limits() -> None:
    with RapidWriter() as w, Module("M"), Proc("main"):
        TPWrite('say "hi" \\')
        with pytest.raises(UnemittableTextError, match="maximum is 80"):
            TPWrite('"' * 41)
    assert 'TPWrite "say ""hi"" \\\\";' in w.getvalue()


def test_tooldata_rejects_massless_tool() -> None:
    with RapidWriter(), Module("M"):
        with pytest.raises(InvalidDeclarationError, match="mass_kg"):
            Tooldata("gun", Pose(), 0.0, [0, 0, 0.1])
