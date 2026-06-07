"""Identifier sanitization → legal target-language identifiers."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import IdentifierError
from tesseract_robotics.emitters.core.naming import safe_identifier


def test_passes_through_legal_name() -> None:
    assert safe_identifier("main_program", brand="KRL", max_len=24) == "main_program"


def test_replaces_illegal_chars() -> None:
    assert safe_identifier("weld path #2", brand="KRL", max_len=24) == "weld_path__2"


def test_prefixes_leading_digit() -> None:
    assert safe_identifier("2nd", brand="KRL", max_len=24) == "_2nd"


def test_truncates_to_max_len() -> None:
    assert safe_identifier("a" * 50, brand="KRL", max_len=24) == "a" * 24


def test_empty_after_sanitization_raises() -> None:
    with pytest.raises(IdentifierError):
        safe_identifier("", brand="KRL", max_len=24)
