"""Per-dialect identifier rules and the validator every backend shares.

An identifier the controller would reject, or one silently rewritten into a
different name, is a program that does not do what the plan says. So names are
validated, never mangled: an illegal name raises `IdentifierError` naming the
rule it breaks. Each dialect states its rules once as an `IdentifierRules`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .errors import IdentifierError


@dataclass(frozen=True)
class IdentifierRules:
    """Lexical rules for one dialect's identifiers.

    Attributes:
        brand: Dialect name used in error messages.
        pattern: Full-match regular expression for a legal identifier.
        pattern_text: Human-readable statement of `pattern` for error messages.
        max_len: Maximum length in characters, or `None` when the dialect sets none.
        reserved: Words the identifier must not equal (compared upper-cased when
            `case_insensitive`).
        case_insensitive: Whether the dialect folds case when comparing names.
    """

    brand: str
    pattern: re.Pattern[str]
    pattern_text: str
    max_len: int | None
    reserved: frozenset[str]
    case_insensitive: bool

    def validate(self, name: str) -> str:
        """Return `name` unchanged if it is a legal identifier, else raise.

        Args:
            name: Candidate identifier.

        Returns:
            `name`, unchanged.

        Raises:
            IdentifierError: `name` is empty, too long, breaks `pattern`, or is reserved.
        """
        if not name:
            raise IdentifierError(name, self.brand, "empty identifier")
        if self.max_len is not None and len(name) > self.max_len:
            raise IdentifierError(
                name, self.brand, f"{len(name)} characters, maximum is {self.max_len}"
            )
        if self.pattern.fullmatch(name) is None:
            raise IdentifierError(name, self.brand, f"must match {self.pattern_text}")
        key = name.upper() if self.case_insensitive else name
        if key in self.reserved:
            raise IdentifierError(name, self.brand, "reserved word")
        return name
