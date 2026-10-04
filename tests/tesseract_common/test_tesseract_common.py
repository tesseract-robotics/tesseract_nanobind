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


def test_manipulator_info():
    info = tesseract_common.ManipulatorInfo()
    info.tcp_offset = "tool0"
    assert info.tcp_offset == "tool0"

    transform = tesseract_common.Isometry3d() * tesseract_common.Translation3d(1, 2, 3)
    info.tcp_offset = transform
    transform2 = info.tcp_offset
    nptest.assert_allclose(transform2.matrix, transform.matrix)
