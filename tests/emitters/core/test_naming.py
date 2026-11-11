"""Identifier rules: legal names pass unchanged, illegal names raise (never mangled)."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import IdentifierError
from tesseract_robotics.emitters.jbi.syntax import JBI_IDENTIFIERS
from tesseract_robotics.emitters.krl.syntax import KRL_IDENTIFIERS
from tesseract_robotics.emitters.ls.syntax import LS_IDENTIFIERS
from tesseract_robotics.emitters.rapid.syntax import RAPID_IDENTIFIERS
from tesseract_robotics.emitters.urscript.syntax import UR_IDENTIFIERS


@pytest.mark.parametrize(
    "rules", [RAPID_IDENTIFIERS, KRL_IDENTIFIERS, LS_IDENTIFIERS, JBI_IDENTIFIERS, UR_IDENTIFIERS]
)
def test_legal_name_passes_unchanged(rules) -> None:
    assert rules.validate("WELD_01") == "WELD_01"


@pytest.mark.parametrize("name", ["weld path", "2nd", "", "a-b"])
def test_rapid_rejects_illegal(name: str) -> None:
    with pytest.raises(IdentifierError):
        RAPID_IDENTIFIERS.validate(name)


def test_rapid_length_limit_is_32() -> None:
    assert RAPID_IDENTIFIERS.validate("a" * 32) == "a" * 32
    with pytest.raises(IdentifierError, match="maximum is 32"):
        RAPID_IDENTIFIERS.validate("a" * 33)


def test_rapid_reserved_word_case_insensitive() -> None:
    with pytest.raises(IdentifierError, match="reserved"):
        RAPID_IDENTIFIERS.validate("Module")


def test_rapid_rejects_leading_underscore() -> None:
    with pytest.raises(IdentifierError):
        RAPID_IDENTIFIERS.validate("_main")


def test_krl_length_and_keyword() -> None:
    with pytest.raises(IdentifierError, match="maximum is 24"):
        KRL_IDENTIFIERS.validate("a" * 25)
    with pytest.raises(IdentifierError, match="reserved"):
        KRL_IDENTIFIERS.validate("lin")


def test_ls_needs_letter_first_and_upper_case() -> None:
    with pytest.raises(IdentifierError):
        LS_IDENTIFIERS.validate("_SPIKE")
    with pytest.raises(IdentifierError):
        LS_IDENTIFIERS.validate("spike")


def test_urscript_keyword() -> None:
    with pytest.raises(IdentifierError, match="reserved"):
        UR_IDENTIFIERS.validate("def")
