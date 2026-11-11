"""Comment and message text can never break out of its syntax."""

from __future__ import annotations

import pytest

from tesseract_robotics.emitters.core.errors import UnemittableTextError
from tesseract_robotics.emitters.core.text import single_line


@pytest.mark.parametrize("text", ["a\nMoveL p1", "a\rb", "a b"])
def test_line_breaks_rejected(text: str) -> None:
    with pytest.raises(UnemittableTextError, match="line break"):
        single_line(text, brand="X")


def test_forbidden_and_length() -> None:
    with pytest.raises(UnemittableTextError, match="';'"):
        single_line("a;b", brand="X", forbidden=";")
    with pytest.raises(UnemittableTextError, match="maximum is 3"):
        single_line("abcd", brand="X", max_len=3)
    assert single_line("ok", brand="X", max_len=3) == "ok"
