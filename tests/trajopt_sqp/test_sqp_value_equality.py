"""SQPParameters compares by value and is unhashable (#261)."""

from __future__ import annotations

import pytest

from tesseract_robotics import trajopt_sqp as tsqp


def test_sqp_parameters():
    a, b = tsqp.SQPParameters(), tsqp.SQPParameters()
    assert a == b
    assert not (a != b)
    b.max_iterations = a.max_iterations + 1
    assert a != b


def test_unhashable():
    assert tsqp.SQPParameters.__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(tsqp.SQPParameters())
