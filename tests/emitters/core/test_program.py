"""EmittedProgram: immutable filename-to-text map with identity and disk write."""

from __future__ import annotations

from pathlib import Path

import pytest

from tesseract_robotics.emitters.core.errors import ProgramFileCountError
from tesseract_robotics.emitters.core.events import ProgramIR
from tesseract_robotics.emitters.core.identity import EmitIdentity
from tesseract_robotics.emitters.core.program import EmittedProgram

_ID = EmitIdentity.build(ProgramIR(name="p"), profiles_repr="x")


def test_single_file_text_and_names() -> None:
    prog = EmittedProgram(identity=_ID, files={"SPIKE.src": "DEF SPIKE()\nEND\n"})
    assert prog.text == "DEF SPIKE()\nEND\n"
    assert prog.filenames == ["SPIKE.src"]
    assert prog.identity is _ID


def test_write_to_creates_files(tmp_path: Path) -> None:
    prog = EmittedProgram(identity=_ID, files={"A.src": "a", "B.dat": "b"})
    written = prog.write_to(tmp_path)
    assert (tmp_path / "A.src").read_text() == "a"
    assert (tmp_path / "B.dat").read_text() == "b"
    assert {p.name for p in written} == {"A.src", "B.dat"}


def test_text_requires_single_file() -> None:
    with pytest.raises(ProgramFileCountError, match="exactly one file"):
        _ = EmittedProgram(identity=_ID, files={"A.src": "a", "B.dat": "b"}).text
