"""Value equality on the tesseract_collision value types (#169).

Each class binds its C++ operator==/operator!= as __eq__/__ne__ and sets __hash__ = None.
"""

import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionCheckConfig,
    ContactManagerConfig,
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactResultValidator,
)


def _changed(make, mutate):
    obj = make()
    mutate(obj)
    return obj


def _result_map_with_entry():
    result_map = ContactResultMap()
    result_map.addContactResult(("link_a", "link_b"), ContactResult())
    return result_map


# (make, make_other): two make() calls give equal objects; make_other() differs in one field.
CASES = {
    "ContactResult": (
        ContactResult,
        lambda: _changed(ContactResult, lambda r: setattr(r, "distance", 0.5)),
    ),
    "ContactResultMap": (ContactResultMap, _result_map_with_entry),
    "ContactRequest": (
        ContactRequest,
        lambda: _changed(ContactRequest, lambda r: setattr(r, "contact_limit", 7)),
    ),
    "ContactManagerConfig": (ContactManagerConfig, lambda: ContactManagerConfig(0.05)),
    "CollisionCheckConfig": (
        CollisionCheckConfig,
        lambda: _changed(
            CollisionCheckConfig, lambda c: setattr(c, "longest_valid_segment_length", 0.5)
        ),
    ),
}


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_equal_when_built_alike(make, make_other):
    a, b = make(), make()
    assert a is not b
    assert a == b
    assert not (a != b)


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_unequal_when_one_field_differs(make, make_other):
    assert make() != make_other()
    assert not (make() == make_other())


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_unhashable(make, make_other):
    obj = make()
    assert type(obj).__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(obj)
    with pytest.raises(TypeError, match="unhashable"):
        set().add(obj)


@pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())
def test_foreign_operand_is_unequal_without_raising(make, make_other):
    """nanobind returns NotImplemented for a foreign operand; Python falls back to identity."""
    obj = make()
    assert (obj == object()) is False
    assert (obj != object()) is True
    assert obj != 5


def test_contact_result_tolerance_comes_from_cpp():
    """ContactResult::operator== uses almostEqualRelativeAndAbs (types.cpp, 0.35.0): default max_diff 1e-6."""
    a, b = ContactResult(), ContactResult()
    a.distance, b.distance = 0.5, 0.5 + 1e-9
    assert a == b
    b.distance = 0.5 + 1e-3
    assert a != b


class _AcceptAll(ContactResultValidator):
    def __call__(self, result):
        return True


def test_contact_request_compares_validator_by_identity():
    """ContactRequest::operator== compares is_valid as a shared_ptr: the same validator object or none."""
    validator = _AcceptAll()
    a, b = ContactRequest(), ContactRequest()
    a.is_valid = validator
    b.is_valid = validator
    assert a == b
    b.is_valid = _AcceptAll()
    assert a != b
    b.is_valid = None
    assert a != b
