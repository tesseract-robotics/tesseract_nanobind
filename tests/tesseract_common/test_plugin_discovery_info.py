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


def test_derived_insert_accepts_the_base():
    # A *PluginInfo is a PluginDiscoveryInfo, so its insert takes one too (the inherited
    # PluginDiscoveryInfo::insert: search paths and libraries only), as well as its own type.
    base = PluginDiscoveryInfo()
    base.search_paths = ["/base"]
    base.search_libraries = ["lib_base"]
    for cls in (
        KinematicsPluginInfo,
        ContactManagersPluginInfo,
        TaskComposerPluginInfo,
        ProfilesPluginInfo,
    ):
        info = cls()
        info.insert(base)
        assert info.search_paths == ["/base"], cls.__name__
        assert info.search_libraries == ["lib_base"], cls.__name__
        same = cls()
        same.insert(info)
        assert same == info, cls.__name__
