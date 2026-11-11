"""trajopt_sqp.toString(SQPStatus): trajopt's own text for a solver status (#284)."""

from __future__ import annotations

import pytest

from tesseract_robotics import trajopt_sqp as tsqp


@pytest.mark.parametrize("status", list(tsqp.SQPStatus))
def test_every_status_has_text(status):
    text = tsqp.toString(status)
    assert isinstance(text, str)
    assert text


def test_texts_differ():
    texts = [tsqp.toString(status) for status in tsqp.SQPStatus]
    assert len(set(texts)) == len(texts)


def test_module_has_no_all():
    """The extension is the module itself: no __all__, every bound name is public."""
    assert not hasattr(tsqp, "__all__")
    from tesseract_robotics.trajopt_sqp import toString  # noqa: F401
