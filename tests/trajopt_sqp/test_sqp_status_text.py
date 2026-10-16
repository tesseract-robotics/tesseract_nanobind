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


def test_star_import_exports_every_bound_name():
    """No __all__: the package re-exports what the extension binds."""
    from tesseract_robotics.trajopt_sqp import _trajopt_sqp

    assert not hasattr(tsqp, "__all__")
    public = {name for name in dir(_trajopt_sqp) if not name.startswith("_")}
    assert public <= set(dir(tsqp))
    from tesseract_robotics.trajopt_sqp import toString  # noqa: F401
