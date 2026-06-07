"""Identifier sanitization shared by all backends.

Controller languages restrict names (alphanumeric + underscore, no leading
digit, a length cap). Coerce deterministically, or raise ``IdentifierError``
when nothing legal remains.
"""

from __future__ import annotations

import re

from .errors import IdentifierError

_ILLEGAL = re.compile(r"[^A-Za-z0-9_]")


def safe_identifier(name: str, *, brand: str, max_len: int) -> str:
    """Coerce ``name`` to a legal identifier for ``brand`` (≤ ``max_len`` chars).

    Non-``[A-Za-z0-9_]`` → ``_``; leading digit gets ``_`` prefix; truncated to
    ``max_len``. Raises ``IdentifierError`` if the result is empty.
    """
    cleaned = _ILLEGAL.sub("_", name)
    if cleaned and cleaned[0].isdigit():
        cleaned = "_" + cleaned
    cleaned = cleaned[:max_len]
    if not cleaned:
        raise IdentifierError(name, brand, "no legal characters after sanitization")
    return cleaned
