"""`ProfileDictionary`: multi-name `addProfile`, `getProfileEntry`, `getAllProfileEntries` (#272)."""

from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_simple import SimplePlannerLVSMoveProfile


def _dictionary():
    p = SimplePlannerLVSMoveProfile()
    d = ProfileDictionary()
    d.addProfile("ns", ["a", "b"], p)
    return d, p


def test_add_profile_under_several_names():
    d, p = _dictionary()
    assert d.hasProfile(p.getKey(), "ns", "a")
    assert d.hasProfile(p.getKey(), "ns", "b")


def test_get_profile_entry():
    d, p = _dictionary()
    entry = d.getProfileEntry(p.getKey(), "ns")
    assert set(entry) == {"a", "b"}
    assert all(type(v) is SimplePlannerLVSMoveProfile for v in entry.values())
    assert entry["a"] is p  # shared, not copied


def test_get_all_profile_entries():
    d, p = _dictionary()
    everything = d.getAllProfileEntries()
    assert set(everything) == {"ns"}
    assert set(everything["ns"]) == {p.getKey()}
    assert set(everything["ns"][p.getKey()]) == set(d.getProfileEntry(p.getKey(), "ns"))
