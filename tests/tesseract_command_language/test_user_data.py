"""`CompositeInstruction` user data (#271)."""

import pytest

from tesseract_robotics.tesseract_command_language import CompositeInstruction

INT_MAX = 2**31 - 1  # C++ `int` on every supported platform


def test_round_trip():
    data = {"a": 1, "b": "x", "c": 2.5, "d": True, "e": False, "n": None}
    c = CompositeInstruction()
    c.setUserData(data)
    got = c.getUserData()
    assert got == data
    assert type(got["d"]) is bool
    assert type(got["a"]) is int


def test_double_precision_is_kept():
    c = CompositeInstruction()
    c.setUserData({"x": 0.1})
    assert c.getUserData()["x"] == 0.1


def test_large_int_round_trips():
    c = CompositeInstruction()
    c.setUserData({"big": INT_MAX + 1, "small": -INT_MAX - 1})
    assert c.getUserData() == {"big": INT_MAX + 1, "small": -INT_MAX - 1}


def test_getter_returns_a_copy():
    c = CompositeInstruction()
    c.setUserData({"a": 1})
    c.getUserData()["a"] = 2
    assert c.getUserData() == {"a": 1}
    c.setUserData({"b": 2})
    assert c.getUserData() == {"b": 2}


@pytest.mark.parametrize("value", [[1, 2], object(), 2**80, {"nested": 1}])
def test_unsupported_value_raises_type_error(value):
    c = CompositeInstruction()
    c.setUserData({"keep": 1})
    with pytest.raises(TypeError, match="bad"):
        c.setUserData({"bad": value})
    assert c.getUserData() == {"keep": 1}
