"""A ContactResult held from a ContactResultVector never outlives its storage (#395).

`v[i]` is a reference into the vector, so `v[i].distance = d` writes through. While Python
holds such a reference, every call that could reallocate or destroy the elements (`append`,
`clear`, `ContactResultMap.flattenMoveResults(v)`) raises `ContactResultHeldError` and leaves
the vector unchanged, as `bytearray` refuses to resize while a buffer is exported.
"""

import copy
import gc

import pytest

from tesseract_robotics.collision import (
    ContactResult,
    ContactResultHeldError,
    ContactResultMap,
    ContactResultVector,
)

KEY_AB = ("a", "b")
# Appends made while an element is held; the issue's reproducer reallocated within these.
APPEND_COUNT = 64
HELD_DISTANCE = 1.5
FILL_DISTANCE = 0.0
WRITTEN_DISTANCE = 7.5


def _result(distance):
    r = ContactResult()
    r.distance = distance
    return r


def _vector(*distances):
    v = ContactResultVector()
    for d in distances:
        v.append(_result(d))
    return v


def test_error_is_a_buffer_error():
    assert issubclass(ContactResultHeldError, BufferError)


def test_append_refused_while_element_held():
    v = _vector(HELD_DISTANCE)
    held = v[0]
    with pytest.raises(ContactResultHeldError):
        for _ in range(APPEND_COUNT):
            v.append(_result(FILL_DISTANCE))
    # Refused on the first append: nothing was added, and the held element is still v[0].
    assert len(v) == 1
    assert held.distance == HELD_DISTANCE
    held.distance = WRITTEN_DISTANCE
    assert v[0].distance == WRITTEN_DISTANCE


def test_append_allowed_once_element_released():
    v = _vector(HELD_DISTANCE)
    held = v[0]
    with pytest.raises(ContactResultHeldError):
        v.append(_result(FILL_DISTANCE))
    del held
    gc.collect()
    for _ in range(APPEND_COUNT):
        v.append(_result(FILL_DISTANCE))
    assert len(v) == APPEND_COUNT + 1
    assert v[0].distance == HELD_DISTANCE


def test_index_write_goes_through():
    v = _vector(HELD_DISTANCE, FILL_DISTANCE)
    v[0].distance = WRITTEN_DISTANCE
    assert v[0].distance == WRITTEN_DISTANCE
    assert v[1].distance == FILL_DISTANCE


def test_clear_refused_while_element_held():
    v = _vector(HELD_DISTANCE, FILL_DISTANCE)
    held = v[1]
    with pytest.raises(ContactResultHeldError):
        v.clear()
    assert len(v) == 2
    assert held.distance == FILL_DISTANCE
    del held
    v.clear()
    assert len(v) == 0


def test_flatten_move_refused_while_element_held():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result(WRITTEN_DISTANCE))
    v = _vector(HELD_DISTANCE)
    held = v[0]
    with pytest.raises(ContactResultHeldError):
        m.flattenMoveResults(v)
    # Checked before the move: the map keeps its results and v its elements.
    assert m.count() == 1
    assert len(v) == 1
    assert held.distance == HELD_DISTANCE
    del held
    m.flattenMoveResults(v)
    assert [r.distance for r in (v[i] for i in range(len(v)))] == [WRITTEN_DISTANCE]


def test_held_element_keeps_vector_alive():
    v = _vector(HELD_DISTANCE)
    held = v[0]
    del v
    gc.collect()
    assert held.distance == HELD_DISTANCE


def test_copy_does_not_pin_vector():
    v = _vector(HELD_DISTANCE)
    kept = copy.copy(v[0])
    v.append(_result(FILL_DISTANCE))
    v.clear()
    assert kept.distance == HELD_DISTANCE
