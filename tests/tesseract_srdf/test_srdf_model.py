"""`CalibrationInfo` and the `SRDFModel` fields that carry plugin and calibration config (gh-217)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

import tesseract_robotics.tesseract_common as tc
from tesseract_robotics.tesseract_common import (
    ContactManagersPluginInfo,
    GeneralResourceLocator,
    Isometry3d,
    Quaterniond,
)
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_urdf import parseURDFFile

IIWA_URDF = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
IIWA_SRDF = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
# lbr_iiwa_14_r820.srdf:50 <contact_managers_plugin_config filename=...>
IIWA_CONTACT_MANAGERS_YAML = "package://tesseract/support/urdf/contact_manager_plugins.yaml"

# CalibrationInfo::operator== (calibration_info.cpp:41-50) compares each transform with
# Isometry3d::isApprox(other, 1e-5): relative, on the 4x4 matrix (norm ~2 for these
# transforms). Offsets two orders of magnitude below and above that precision, in metres.
CALIBRATION_EQUAL_OFFSET_M = 1e-7
CALIBRATION_UNEQUAL_OFFSET_M = 1e-3


def _translation(x: float, y: float, z: float) -> Isometry3d:
    return Isometry3d(np.array([x, y, z]), Quaterniond.Identity())


def _info(joints: dict[str, Isometry3d]) -> tc.CalibrationInfo:
    info = tc.CalibrationInfo()
    info.joints = joints
    return info


@pytest.fixture
def locator() -> GeneralResourceLocator:
    return GeneralResourceLocator()


@pytest.fixture
def iiwa(locator):
    """(scene_graph, srdf_model) of the iiwa support robot."""
    scene_graph = parseURDFFile(locator.locateResource(IIWA_URDF).getFilePath(), locator)
    model = SRDFModel()
    model.initFile(scene_graph, locator.locateResource(IIWA_SRDF).getFilePath(), locator)
    return scene_graph, model


def test_srdf_model_calibration_info_round_trip():
    model = SRDFModel()
    assert model.calibration_info.empty()

    info = _info({"j": _translation(1.0, 2.0, 3.0)})
    model.calibration_info = info
    assert model.calibration_info == info


def test_calibration_info_insert_overwrites_by_joint_name():
    info = _info({"j": _translation(1.0, 0.0, 0.0)})
    other = _info({"j": _translation(2.0, 0.0, 0.0), "k": _translation(3.0, 0.0, 0.0)})

    info.insert(other)

    assert set(info.joints) == {"j", "k"}
    np.testing.assert_allclose(info.joints["j"].translation, [2.0, 0.0, 0.0])
    np.testing.assert_allclose(info.joints["k"].translation, [3.0, 0.0, 0.0])

    info.clear()
    assert info.empty()
    assert info.joints == {}


def test_calibration_info_equality_uses_upstream_tolerance():
    base = _info({"j": _translation(1.0, 2.0, 3.0)})
    near = _info({"j": _translation(1.0 + CALIBRATION_EQUAL_OFFSET_M, 2.0, 3.0)})
    far = _info({"j": _translation(1.0 + CALIBRATION_UNEQUAL_OFFSET_M, 2.0, 3.0)})

    assert base == near
    assert not base != near
    assert base != far
    assert not base == far


def test_calibration_info_is_unhashable():
    with pytest.raises(TypeError):
        hash(tc.CalibrationInfo())


def test_calibration_info_config_key():
    assert tc.CalibrationInfo.CONFIG_KEY == "calibration"


def test_srdf_model_contact_managers_plugin_info_from_iiwa(iiwa, locator):
    _, model = iiwa
    plugin_info = model.contact_managers_plugin_info

    assert isinstance(plugin_info, ContactManagersPluginInfo)
    assert not plugin_info.empty()

    yaml_path = Path(locator.locateResource(IIWA_CONTACT_MANAGERS_YAML).getFilePath())
    config = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))["contact_manager_plugins"]
    discrete = config["discrete_plugins"]
    assert set(plugin_info.discrete_plugin_infos.plugins) == set(discrete["plugins"])
    assert plugin_info.discrete_plugin_infos.default_plugin == discrete["default"]


def test_srdf_model_save_and_reload_round_trip(iiwa, locator, tmp_path):
    """saveToFile writes the calibration and contact-manager configs beside the SRDF and
    references them by bare file name (srdf_model.cpp:351-373)."""
    scene_graph, model = iiwa
    model.calibration_info = _info({"joint_a1": _translation(0.0, 0.0, 0.1)})

    srdf_path = tmp_path / "robot.srdf"
    assert model.saveToFile(str(srdf_path))
    assert (tmp_path / "calibration_config.yaml").is_file()
    assert (tmp_path / "contact_managers_plugin_config.yaml").is_file()

    reloaded = SRDFModel()
    reloaded.initFile(scene_graph, str(srdf_path), locator)

    assert reloaded.calibration_info == model.calibration_info
    assert reloaded.contact_managers_plugin_info == model.contact_managers_plugin_info
    assert reloaded == model
