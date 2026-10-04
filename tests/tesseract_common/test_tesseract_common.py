import io
from inspect import currentframe, getframeinfo

import numpy as np
import numpy.testing as nptest

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
