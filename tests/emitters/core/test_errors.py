"""Error taxonomy: every failure mode is a distinct, structured exception."""

from __future__ import annotations

from tesseract_robotics.emitters.core.errors import (
    EmitterError,
    EmptyProgramError,
    MissingProfileError,
    UnsupportedInstructionError,
)


def test_all_inherit_emitter_error() -> None:
    for cls in (EmptyProgramError, UnsupportedInstructionError, MissingProfileError):
        assert issubclass(cls, EmitterError)


def test_missing_profile_carries_structured_fields_and_name() -> None:
    err = MissingProfileError("WELD", ["RASTER", "DEFAULT"])
    assert err.profile == "WELD"
    assert err.available == ["DEFAULT", "RASTER"]  # sorted
    assert "'WELD'" in str(err)


def test_empty_program_message_contains_phrase() -> None:
    err = EmptyProgramError("my_prog")
    assert err.description == "my_prog"
    assert "no leaf instructions" in str(err)


def test_unsupported_keeps_message() -> None:
    err = UnsupportedInstructionError("CIRCULAR moves cannot be lowered")
    assert "CIRCULAR" in str(err)
