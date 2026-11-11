"""Free-text guards for comments and messages.

A comment that contains a line break ends early, and whatever follows the break
is parsed as code. Every dialect's comment statement routes its text through
`single_line` so that can never happen.
"""

from __future__ import annotations

from .errors import UnemittableTextError

#: Characters that end a source line in any of the supported dialects.
_LINE_BREAKS = ("\n", "\r", "\x0b", "\x0c", "\x85", "\u2028", "\u2029")


def single_line(text: str, *, brand: str, forbidden: str = "", max_len: int | None = None) -> str:
    """Return `text` unchanged if it fits on one line of a comment, else raise.

    Args:
        text: The comment or message body.
        brand: Dialect name for the error message.
        forbidden: Extra characters the dialect's syntax cannot carry in this position.
        max_len: Maximum length the dialect allows, or `None` for no limit.

    Returns:
        `text`, unchanged.

    Raises:
        UnemittableTextError: `text` contains a line break or a forbidden character,
            or exceeds `max_len`.
    """
    for brk in _LINE_BREAKS:
        if brk in text:
            raise UnemittableTextError(text, brand, f"contains line break {brk!r}")
    for ch in forbidden:
        if ch in text:
            raise UnemittableTextError(text, brand, f"contains {ch!r}")
    if max_len is not None and len(text) > max_len:
        raise UnemittableTextError(text, brand, f"{len(text)} characters, maximum is {max_len}")
    return text
