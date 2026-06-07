"""EmitIdentity: same program in → same digest out (structural reproducibility)."""

from __future__ import annotations

from tesseract_robotics.emitters.core.events import Dwell, Note, ProgramIR
from tesseract_robotics.emitters.core.identity import EmitIdentity


def test_identical_inputs_give_identical_digests() -> None:
    ir = ProgramIR(name="p", events=(Note("x"), Dwell(seconds=1.0)))
    a = EmitIdentity.build(ir, profiles_repr="P")
    b = EmitIdentity.build(ir, profiles_repr="P")
    assert a.ir_digest == b.ir_digest and a.profiles_digest == b.profiles_digest


def test_program_change_changes_ir_digest() -> None:
    a = EmitIdentity.build(ProgramIR(name="p", events=(Dwell(seconds=1.0),)), profiles_repr="x")
    b = EmitIdentity.build(ProgramIR(name="p", events=(Dwell(seconds=2.0),)), profiles_repr="x")
    assert a.ir_digest != b.ir_digest


def test_digests_are_prefixed_sha256_hex() -> None:
    ident = EmitIdentity.build(ProgramIR(name="p"), profiles_repr="x")
    assert ident.ir_digest.startswith("sha256:")
    assert len(ident.ir_digest) == len("sha256:") + 64


def test_header_lines_have_version_and_digest() -> None:
    ident = EmitIdentity.build(ProgramIR(name="p"), profiles_repr="x")
    lines = ident.header_lines()
    assert any("tesseract_robotics.emitters" in ln for ln in lines)
    assert any(ident.ir_digest in ln for ln in lines)
