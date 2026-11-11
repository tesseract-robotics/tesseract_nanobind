"""`PluginDiscoveryInfo` (#373), the base of every `*PluginInfo`: search paths and libraries."""

from tesseract_robotics.common import (
    ContactManagersPluginInfo,
    KinematicsPluginInfo,
    PluginDiscoveryInfo,
    ProfilesPluginInfo,
    TaskComposerPluginInfo,
)


def test_fields_round_trip():
    info = PluginDiscoveryInfo()
    assert info.empty()
    info.search_paths = ["/a", "/b"]
    info.search_libraries = ["lib_one"]
    assert info.search_paths == ["/a", "/b"]
    assert info.search_libraries == ["lib_one"]
    assert not info.empty()


def test_insert_clear_and_equality():
    a = PluginDiscoveryInfo()
    a.search_paths = ["/a"]
    b = PluginDiscoveryInfo()
    b.search_libraries = ["lib_b"]
    a.insert(b)
    assert a.search_libraries == ["lib_b"]
    copy = PluginDiscoveryInfo()
    copy.search_paths = ["/a"]
    copy.search_libraries = ["lib_b"]
    assert a == copy
    copy.search_paths = ["/other"]
    assert a != copy
    a.clear()
    assert a.empty()


def test_plugin_infos_derive_from_it():
    for cls in (
        KinematicsPluginInfo,
        ContactManagersPluginInfo,
        TaskComposerPluginInfo,
        ProfilesPluginInfo,
    ):
        assert issubclass(cls, PluginDiscoveryInfo)
