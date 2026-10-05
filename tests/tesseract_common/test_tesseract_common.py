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
