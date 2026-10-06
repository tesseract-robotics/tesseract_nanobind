import io
from inspect import currentframe, getframeinfo

import numpy as np
import numpy.testing as nptest
import pytest

from tesseract_robotics import tesseract_common


def test_bytes_resource():
    my_bytes = bytearray([10, 57, 92, 56, 92, 46, 92, 127])
    my_bytes_url = "file:///test_bytes.bin"
    bytes_resource = tesseract_common.BytesResource(my_bytes_url, my_bytes)
    my_bytes_ret = bytes_resource.getResourceContents()
    assert len(my_bytes_ret) == len(my_bytes)
    assert my_bytes == bytearray(my_bytes_ret)
    assert my_bytes_url == bytes_resource.getUrl()


def test_bytes_resource_content_stream():
    """getResourceContentStream returns a readable binary stream of the contents."""
    my_bytes = bytes([10, 57, 92, 56, 92, 46, 92, 127])
    bytes_resource = tesseract_common.BytesResource("file:///test_bytes.bin", my_bytes)
    stream = bytes_resource.getResourceContentStream()
    assert isinstance(stream, io.BytesIO)
    assert stream.read() == my_bytes


class _TestOutputHandler(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.last_text = None

    def log(self, text, level, filename, line):
        self.last_text = text


def test_console_bridge():
    tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

    frameinfo = getframeinfo(currentframe())
    tesseract_common.log(
        frameinfo.filename,
        frameinfo.lineno,
        tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG,
        "This is a test message",
    )

    output_handler = _TestOutputHandler()
    tesseract_common.useOutputHandler(output_handler)

    tesseract_common.log(
        frameinfo.filename,
        frameinfo.lineno,
        tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG,
        "This is a test message 2",
    )
    tesseract_common.restorePreviousOutputHandler()

    assert output_handler.last_text == "This is a test message 2"

    tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_ERROR)


def test_manipulator_info():
    info = tesseract_common.ManipulatorInfo()
    info.tcp_offset = "tool0"
    assert info.tcp_offset == "tool0"

    transform = tesseract_common.Isometry3d() * tesseract_common.Translation3d(1, 2, 3)
    info.tcp_offset = transform
    transform2 = info.tcp_offset
    nptest.assert_allclose(transform2.matrix, transform.matrix)


# satisfiesLimits (gh-158). Tolerances in joint units (rad); upstream scalar default max_diff is 1e-6.
SATISFIES_LIMITS_INSIDE_DEFAULT_TOL = 1e-7  # [rad] overshoot below the 1e-6 default max_diff
SATISFIES_LIMITS_OUTSIDE_DEFAULT_TOL = 1e-5  # [rad] overshoot above the 1e-6 default max_diff
SATISFIES_LIMITS_LOOSE_TOL = 1e-4  # [rad] max_diff that admits the 1e-5 overshoot
SATISFIES_LIMITS_NO_REL_TOL = 0.0  # disable the relative check so only max_diff decides

_LIMITS = np.array([[-1.0, 1.0], [-2.0, 2.0]])


def test_satisfies_limits_inside():
    assert tesseract_common.satisfiesLimits(np.array([0.5, -1.5]), _LIMITS)


def test_satisfies_limits_outside():
    assert not tesseract_common.satisfiesLimits(np.array([1.1, 0.0]), _LIMITS)


def test_satisfies_limits_default_max_diff():
    near = np.array([1.0 + SATISFIES_LIMITS_INSIDE_DEFAULT_TOL, 0.0])
    over = np.array([1.0 + SATISFIES_LIMITS_OUTSIDE_DEFAULT_TOL, 0.0])
    assert tesseract_common.satisfiesLimits(near, _LIMITS)
    assert not tesseract_common.satisfiesLimits(over, _LIMITS)


def test_satisfies_limits_scalar_tolerance_kwargs():
    over = np.array([1.0 + SATISFIES_LIMITS_OUTSIDE_DEFAULT_TOL, 0.0])
    assert tesseract_common.satisfiesLimits(
        over, _LIMITS, max_diff=SATISFIES_LIMITS_LOOSE_TOL, max_rel_diff=SATISFIES_LIMITS_NO_REL_TOL
    )


def test_satisfies_limits_per_axis_tolerance():
    # both joints overshoot by 1e-5; only axis 0 gets the loose tolerance
    over = np.array([1.0, 2.0]) + SATISFIES_LIMITS_OUTSIDE_DEFAULT_TOL
    no_rel = np.full(2, SATISFIES_LIMITS_NO_REL_TOL)
    loose_0 = np.array([SATISFIES_LIMITS_LOOSE_TOL, SATISFIES_LIMITS_INSIDE_DEFAULT_TOL])
    loose_both = np.full(2, SATISFIES_LIMITS_LOOSE_TOL)
    assert not tesseract_common.satisfiesLimits(over, _LIMITS, loose_0, no_rel)
    assert tesseract_common.satisfiesLimits(over, _LIMITS, loose_both, no_rel)


def _plugin_info(class_name):
    pi = tesseract_common.PluginInfo()
    pi.class_name = class_name
    return pi


def _container(name, class_name):
    container = tesseract_common.PluginInfoContainer()
    container.default_plugin = name
    container.plugins = {name: _plugin_info(class_name)}
    return container


# (class name, its two PluginInfoContainer fields)
_CONTAINER_PLUGIN_INFOS = [
    ("ContactManagersPluginInfo", "discrete_plugin_infos", "continuous_plugin_infos"),
    ("TaskComposerPluginInfo", "executor_plugin_infos", "task_plugin_infos"),
]


@pytest.mark.parametrize(("cls_name", "field_a", "field_b"), _CONTAINER_PLUGIN_INFOS)
def test_container_plugin_info_fields(cls_name, field_a, field_b):
    info = getattr(tesseract_common, cls_name)()
    assert info.empty()

    info.search_paths = ["/opt/plugins"]
    info.search_libraries = ["my_factories"]
    setattr(info, field_a, _container("A", "AFactory"))
    # The container field is a bound class: an in-place edit persists.
    getattr(info, field_b).default_plugin = "B"
    getattr(info, field_b).plugins = {"B": _plugin_info("BFactory")}

    assert info.search_paths == ["/opt/plugins"]
    assert info.search_libraries == ["my_factories"]
    assert getattr(info, field_a).default_plugin == "A"
    assert getattr(info, field_a).plugins["A"].class_name == "AFactory"
    assert getattr(info, field_b).default_plugin == "B"
    assert getattr(info, field_b).plugins["B"].class_name == "BFactory"
    assert not info.empty()

    info.clear()
    assert info.empty()


@pytest.mark.parametrize(("cls_name", "field_a", "field_b"), _CONTAINER_PLUGIN_INFOS)
def test_container_plugin_info_insert(cls_name, field_a, field_b):
    cls = getattr(tesseract_common, cls_name)
    a, b = cls(), cls()
    a.search_paths = ["/a"]
    setattr(a, field_a, _container("A", "AFactory"))
    b.search_paths = ["/b"]
    setattr(b, field_a, _container("B", "BFactory"))
    setattr(b, field_b, _container("C", "CFactory"))

    a.insert(b)

    assert a.search_paths == ["/a", "/b"]
    assert set(getattr(a, field_a).plugins) == {"A", "B"}
    assert set(getattr(a, field_b).plugins) == {"C"}


@pytest.mark.parametrize(("cls_name", "field_a", "field_b"), _CONTAINER_PLUGIN_INFOS)
def test_container_plugin_info_eq(cls_name, field_a, field_b):
    cls = getattr(tesseract_common, cls_name)
    a, b = cls(), cls()
    assert a == b
    assert not (a != b)
    setattr(a, field_a, _container("A", "AFactory"))
    assert a != b
    assert not (a == b)


def test_profiles_plugin_info_fields():
    info = tesseract_common.ProfilesPluginInfo()
    assert info.empty()

    info.search_paths = ["/opt/plugins"]
    info.search_libraries = ["my_profiles"]
    info.plugin_infos = {"ns": {"p": _plugin_info("PFactory")}}

    assert info.search_paths == ["/opt/plugins"]
    assert info.search_libraries == ["my_profiles"]
    assert info.plugin_infos["ns"]["p"].class_name == "PFactory"
    assert not info.empty()

    info.clear()
    assert info.empty()


def test_profiles_plugin_info_insert():
    a = tesseract_common.ProfilesPluginInfo()
    b = tesseract_common.ProfilesPluginInfo()
    a.plugin_infos = {"ns": {"p": _plugin_info("PFactory")}}
    b.plugin_infos = {
        "ns": {"q": _plugin_info("QFactory")},
        "other": {"r": _plugin_info("RFactory")},
    }

    a.insert(b)

    assert set(a.plugin_infos) == {"ns", "other"}
    assert set(a.plugin_infos["ns"]) == {"p", "q"}


def test_profiles_plugin_info_eq():
    a = tesseract_common.ProfilesPluginInfo()
    b = tesseract_common.ProfilesPluginInfo()
    assert a == b
    assert not (a != b)
    a.search_paths = ["/a"]
    assert a != b
    assert not (a == b)


@pytest.mark.parametrize(
    "cls_name", ["ContactManagersPluginInfo", "TaskComposerPluginInfo", "ProfilesPluginInfo"]
)
def test_plugin_info_unhashable(cls_name):
    """Value equality on a mutable type: hashing must raise, not fall back to identity."""
    with pytest.raises(TypeError):
        hash(getattr(tesseract_common, cls_name)())


def test_profiles_plugin_info_plugin_infos_is_copy():
    """plugin_infos converts to a fresh dict: assignment round-trips, in-place edits don't."""
    info = tesseract_common.ProfilesPluginInfo()
    info.plugin_infos = {"ns": {"p": _plugin_info("PFactory")}}

    info.plugin_infos["ns"] = {}

    assert set(info.plugin_infos["ns"]) == {"p"}


@pytest.mark.parametrize(
    ("cls_name", "key"),
    [
        ("KinematicsPluginInfo", "kinematic_plugins"),
        ("ContactManagersPluginInfo", "contact_manager_plugins"),
        ("ProfilesPluginInfo", "profile_plugins"),
        ("TaskComposerPluginInfo", "task_composer_plugins"),
    ],
)
def test_plugin_info_config_keys(cls_name, key):
    cls = getattr(tesseract_common, cls_name)
    assert cls.CONFIG_KEY == key
    with pytest.raises(AttributeError):
        cls().CONFIG_KEY = "x"


@pytest.mark.parametrize("name", ["FilesystemPath", "_FilesystemPath", "TransformMap"])
def test_no_filesystem_path_shims(name):
    """gh-165: `std::filesystem::path` is a caster (`pathlib.Path`), TransformMap a plain dict."""
    assert not hasattr(tesseract_common, name)


# ---------------------------------------------------------------------------
# gh-166: Resource is a ResourceLocator; BytesResource takes `parent`; GeneralResourceLocator API
# ---------------------------------------------------------------------------


class _RecordingLocator(tesseract_common.ResourceLocator):
    """Answers only `hits`; records every url it is asked for."""

    def __init__(self, hits):
        super().__init__()
        self.hits = hits
        self.asked = []

    def locateResource(self, url):
        self.asked.append(url)
        if url in self.hits:
            return tesseract_common.BytesResource(url, self.hits[url])
        return None


def test_resource_is_resource_locator():
    assert issubclass(tesseract_common.Resource, tesseract_common.ResourceLocator)
    assert isinstance(
        tesseract_common.BytesResource("file:///a.bin", b"x"), tesseract_common.ResourceLocator
    )


def test_located_resources_without_parent_locate_nothing():
    """Upstream: no parent -> locateResource returns nullptr (resource_locator.cpp, 0.35.0)."""
    br = tesseract_common.BytesResource("file:///dir/a.bin", b"x")
    slr = tesseract_common.SimpleLocatedResource("file:///dir/a.bin", "/dir/a.bin")
    assert br.locateResource("b.bin") is None
    assert slr.locateResource("b.bin") is None


def test_bytes_resource_parent_resolves_relative_url():
    """BytesResource asks its parent for the url as given, then for the sibling of its own url."""
    sibling = "package://pkg/meshes/b.stl"
    loc = _RecordingLocator({sibling: b"mesh"})
    br = tesseract_common.BytesResource("package://pkg/meshes/a.urdf", b"<robot/>", parent=loc)

    found = br.locateResource("b.stl")

    assert found is not None
    assert found.getUrl() == sibling
    assert found.getResourceContents() == b"mesh"
    assert loc.asked == ["b.stl", sibling]


def test_bytes_resource_list_ctor_takes_parent():
    loc = _RecordingLocator({"package://pkg/b.bin": b"y"})
    br = tesseract_common.BytesResource("package://pkg/a.bin", [1, 2, 3], parent=loc)
    assert br.getResourceContents() == bytes([1, 2, 3])
    assert br.locateResource("b.bin").getUrl() == "package://pkg/b.bin"


def test_simple_located_resource_delegates_to_parent():
    loc = _RecordingLocator({"package://pkg/meshes/b.stl": b"mesh"})
    slr = tesseract_common.SimpleLocatedResource("package://pkg/meshes/a.urdf", "/x/a.urdf", loc)
    assert slr.locateResource("b.stl").getUrl() == "package://pkg/meshes/b.stl"


def _make_package(root, name="my_pkg"):
    """Upstream package rule: a directory holding `package.xml` is a package named after the directory."""
    pkg = root / name
    pkg.mkdir()
    (pkg / "package.xml").write_text("<package/>")
    (pkg / "data.txt").write_text("hello")
    return pkg


def test_general_resource_locator_paths_ctor(tmp_path):
    pkg = _make_package(tmp_path)
    loc = tesseract_common.GeneralResourceLocator(paths=[tmp_path], environment_variables=[])
    res = loc.locateResource("package://my_pkg/data.txt")
    assert res is not None
    assert res.getFilePath() == str(pkg / "data.txt")
    assert res.getResourceContents() == b"hello"


def test_general_resource_locator_add_path(tmp_path):
    _make_package(tmp_path)
    loc = tesseract_common.GeneralResourceLocator(environment_variables=[])
    assert loc.locateResource("package://my_pkg/data.txt") is None
    assert loc.addPath(tmp_path) is True
    assert loc.locateResource("package://my_pkg/data.txt") is not None
    assert loc.addPath(tmp_path / "missing") is False


def test_general_resource_locator_environment_variable(tmp_path, monkeypatch):
    _make_package(tmp_path)
    monkeypatch.setenv("GH166_RESOURCE_PATH", str(tmp_path))
    monkeypatch.delenv("GH166_UNSET_VAR", raising=False)

    loc = tesseract_common.GeneralResourceLocator(environment_variables=["GH166_RESOURCE_PATH"])
    assert loc.locateResource("package://my_pkg/data.txt") is not None

    empty = tesseract_common.GeneralResourceLocator(environment_variables=[])
    assert empty.loadEnvironmentVariable("GH166_UNSET_VAR") is False
    assert empty.loadEnvironmentVariable("GH166_RESOURCE_PATH") is True
    assert empty.locateResource("package://my_pkg/data.txt") is not None


def test_general_resource_locator_positional_list_rejected():
    """A positional list could mean env-var names or paths; both ctors are keyword-only."""
    with pytest.raises(TypeError):
        tesseract_common.GeneralResourceLocator(["/some/dir"])


# ---------------------------------------------------------------------------
# gh-167: JointTrajectory
# ---------------------------------------------------------------------------

_NIL_UUID = "00000000-0000-0000-0000-000000000000"
_SOME_UUID = "123e4567-e89b-12d3-a456-426614174000"


def _joint_state(t, q=(0.0, 0.0)):
    js = tesseract_common.JointState(["j1", "j2"], np.array(q))
    js.time = t
    return js


def _trajectory(n=3):
    return tesseract_common.JointTrajectory([_joint_state(float(i)) for i in range(n)], "traj")


def test_joint_trajectory_construct():
    empty = tesseract_common.JointTrajectory()
    assert empty.description == ""
    assert len(empty) == 0
    assert tesseract_common.JointTrajectory("d").description == "d"

    traj = _trajectory()
    assert traj.description == "traj"
    assert [s.time for s in traj.states] == [0.0, 1.0, 2.0]
    assert traj.states[1].joint_names == ["j1", "j2"]

    traj.states = [_joint_state(9.0)]
    traj.description = "other"
    assert [s.time for s in traj.states] == [9.0]
    assert traj.description == "other"


def test_joint_trajectory_container_protocol():
    traj = _trajectory()
    assert len(traj) == 3
    assert traj[0].time == 0.0
    assert traj[-1].time == 2.0
    assert [s.time for s in traj] == [0.0, 1.0, 2.0]

    traj[1] = _joint_state(5.0)
    assert traj[1].time == 5.0
    assert traj.states[1].time == 5.0  # __setitem__ writes through to `states`
    traj[-1] = _joint_state(6.0)
    assert traj[2].time == 6.0

    traj.push_back(_joint_state(7.0))
    assert len(traj) == 4
    assert traj.back().time == 7.0
    assert traj.front().time == 0.0
    assert traj.at(3).time == 7.0
    traj.pop_back()
    assert len(traj) == 3
    assert not traj.empty()

    traj.reserve(10)
    assert traj.capacity() >= 10
    traj.shrink_to_fit()
    assert traj.max_size() >= len(traj)

    traj.clear()
    assert traj.empty()
    assert len(traj) == 0


def test_joint_trajectory_index_out_of_range_raises():
    traj = _trajectory()
    with pytest.raises(IndexError):
        traj[len(traj)]
    with pytest.raises(IndexError):
        traj[-len(traj) - 1]
    with pytest.raises(IndexError):
        traj[len(traj)] = _joint_state(0.0)
    with pytest.raises(IndexError):
        traj.at(99)
    with pytest.raises(IndexError):
        tesseract_common.JointTrajectory().front()
    with pytest.raises(IndexError):
        tesseract_common.JointTrajectory().back()
    with pytest.raises(IndexError):
        tesseract_common.JointTrajectory().pop_back()


def test_joint_trajectory_getitem_is_copy():
    """Element access returns a copy: an edit needs a write-back (`traj[i] = js`)."""
    traj = _trajectory()
    traj[0].time = 5.0
    assert traj[0].time == 0.0
    traj.front().time = 5.0
    traj.at(0).time = 5.0
    for js in traj:
        js.time = 5.0
    assert traj[0].time == 0.0

    js = traj[0]
    js.time = 5.0
    traj[0] = js
    assert traj[0].time == 5.0


def test_joint_trajectory_eq():
    """C++ operator== compares uuid, description and states (joint_state.cpp, 0.35.0)."""
    a, b = _trajectory(), _trajectory()
    assert a == b
    assert not (a != b)

    b[0] = _joint_state(0.0, (1.0, 0.0))
    assert a != b
    assert not (a == b)

    c = _trajectory()
    c.description = "other"
    assert a != c

    d = _trajectory()
    d.uuid = _SOME_UUID
    assert a != d


def test_joint_trajectory_unhashable():
    with pytest.raises(TypeError):
        hash(tesseract_common.JointTrajectory())


def test_joint_trajectory_uuid_roundtrip():
    traj = tesseract_common.JointTrajectory()
    assert traj.uuid == _NIL_UUID
    traj.uuid = _SOME_UUID
    assert traj.uuid == _SOME_UUID
    with pytest.raises(ValueError):
        traj.uuid = "not-a-uuid"
    assert traj.uuid == _SOME_UUID


# ---------------------------------------------------------------------------
# gh-180: AllowedCollisionMatrix entries ctor, single-link remove, reserve, __str__;
# makeOrderedLinkPair, getAllowedCollisions
# ---------------------------------------------------------------------------


def _acm():
    acm = tesseract_common.AllowedCollisionMatrix()
    acm.addAllowedCollision("link_a", "link_b", "adjacent")
    acm.addAllowedCollision("link_a", "link_c", "never")
    acm.addAllowedCollision("link_b", "link_d", "adjacent")
    acm.addAllowedCollision("link_c", "link_d", "never")
    return acm


def test_acm_entries_ctor_roundtrip():
    acm = _acm()
    copy = tesseract_common.AllowedCollisionMatrix(acm.getAllAllowedCollisions())
    assert copy.getAllAllowedCollisions() == acm.getAllAllowedCollisions()
    assert copy.isCollisionAllowed("link_b", "link_a")


def test_acm_entries_ctor_orders_keys():
    """Upstream orders each key (allowed_collision_matrix.cpp, 0.35.0), so ("b", "a") is stored as ("a", "b")."""
    acm = tesseract_common.AllowedCollisionMatrix({("link_b", "link_a"): "adjacent"})
    assert acm.getAllAllowedCollisions() == {("link_a", "link_b"): "adjacent"}
    assert acm.isCollisionAllowed("link_a", "link_b")


def test_acm_remove_allowed_collision_single_link():
    acm = _acm()
    acm.removeAllowedCollision("link_a")
    assert not acm.isCollisionAllowed("link_a", "link_b")
    assert not acm.isCollisionAllowed("link_a", "link_c")
    assert acm.getAllAllowedCollisions() == {
        ("link_b", "link_d"): "adjacent",
        ("link_c", "link_d"): "never",
    }


def test_acm_reserve_keeps_entries():
    acm = _acm()
    before = acm.getAllAllowedCollisions()
    acm.reserveAllowedCollisionMatrix(100)
    assert acm.getAllAllowedCollisions() == before


def test_acm_str():
    text = str(_acm())
    for (link1, link2), reason in _acm().getAllAllowedCollisions().items():
        assert f"link={link1} link={link2} reason={reason}" in text


def test_make_ordered_link_pair():
    assert tesseract_common.makeOrderedLinkPair("b", "a") == ("a", "b")
    assert tesseract_common.makeOrderedLinkPair("a", "b") == ("a", "b")


def test_get_allowed_collisions():
    entries = _acm().getAllAllowedCollisions()
    assert sorted(tesseract_common.getAllowedCollisions(["link_a"], entries)) == [
        "link_b",
        "link_c",
    ]
    # link_b and link_c share the partner link_d (and link_a): once by default, twice without dedup
    shared = tesseract_common.getAllowedCollisions(["link_b", "link_c"], entries)
    assert sorted(shared) == ["link_a", "link_d"]
    dup = tesseract_common.getAllowedCollisions(
        ["link_b", "link_c"], entries, remove_duplicates=False
    )
    assert sorted(dup) == ["link_a", "link_a", "link_d", "link_d"]
