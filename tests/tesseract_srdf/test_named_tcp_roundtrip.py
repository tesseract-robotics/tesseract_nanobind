"""Named TCPs parsed from an SRDF read back exactly, and resolve through `findTCPOffset` (gh-133).

The SRDF below gives one group three named TCPs (`rpy` and `wxyz` forms) and a second group
two, one of them sharing a name with the first group but not its transform. Expected transforms
are built here from the same attribute strings, independently of the C++ parser:
`rpy` as Rz(y) Ry(p) Rx(r) (group_tool_center_points.cpp:135-139), `wxyz` scalar-first
(:164-173).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_urdf import parseURDFFile

IIWA_URDF = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"

# Rotation entries are <= 1; the parser composes AngleAxis -> quaternion -> matrix, each step
# rounding to within a few ulp (2.2e-16), so independent constructions agree to ~1e-15.
ROTATION_ATOL = 1e-14

# group -> tcp name -> (xyz, ("rpy" | "wxyz", values)); the SRDF text is generated from this.
NAMED_TCPS: dict[str, dict[str, tuple[tuple[float, ...], tuple[str, tuple[float, ...]]]]] = {
    "manipulator": {
        "laser": ((0.01, -0.02, 0.25), ("rpy", (0.1, -0.2, 0.3))),
        "welder": ((0.0, 0.05, 0.18), ("wxyz", (0.5, 0.5, 0.5, 0.5))),
        "probe": ((0.1, 0.0, 0.0), ("rpy", (0.0, 0.0, 1.5707963267948966))),
    },
    "manipulator_joint_group": {
        "laser": ((0.0, 0.0, 0.4), ("rpy", (0.0, 1.57, 0.0))),
        "gripper": ((-0.03, 0.0, 0.12), ("wxyz", (0.0, 1.0, 0.0, 0.0))),
    },
}


def _srdf_string() -> str:
    blocks = []
    for group, tcps in NAMED_TCPS.items():
        lines = [f'  <group_tcps group="{group}">']
        for name, (xyz, (kind, values)) in tcps.items():
            lines.append(
                f'    <tcp name="{name}" xyz="{" ".join(map(repr, xyz))}" '
                f'{kind}="{" ".join(map(repr, values))}"/>'
            )
        lines.append("  </group_tcps>")
        blocks.append("\n".join(lines))
    joints = "\n".join(f'    <joint name="joint_a{i}"/>' for i in range(1, 8))
    return f"""<?xml version="1.0" ?>
<robot name="kuka_lbr_iiwa_14_r820">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <group name="manipulator_joint_group">
{joints}
  </group>
{chr(10).join(blocks)}
  <kinematics_plugin_config filename="package://tesseract/support/urdf/lbr_iiwa_14_r820_plugins.yaml"/>
  <contact_managers_plugin_config filename="package://tesseract/support/urdf/contact_manager_plugins.yaml"/>
</robot>
"""


def _rotation(kind: str, values: tuple[float, ...]) -> np.ndarray:
    if kind == "rpy":
        r, p, y = values
        rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return rz @ ry @ rx
    w, x, y, z = np.asarray(values) / np.linalg.norm(values)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _assert_tcp(
    actual: Isometry3d, xyz: tuple[float, ...], kind: str, values: tuple[float, ...]
) -> None:
    matrix = actual.matrix
    np.testing.assert_array_equal(matrix[:3, 3], xyz)
    np.testing.assert_allclose(matrix[:3, :3], _rotation(kind, values), rtol=0, atol=ROTATION_ATOL)
    np.testing.assert_array_equal(matrix[3], [0, 0, 0, 1])


@pytest.fixture(scope="module")
def locator() -> GeneralResourceLocator:
    return GeneralResourceLocator()


@pytest.fixture(scope="module")
def urdf_path(locator) -> Path:
    return Path(locator.locateResource(IIWA_URDF).getFilePath())


@pytest.fixture(scope="module")
def env(locator, urdf_path) -> Environment:
    environment = Environment()
    assert environment.init(urdf_path.read_text(), _srdf_string(), locator)
    return environment


def test_parsed_srdf_group_tcps_read_back_exactly(locator, urdf_path):
    scene_graph = parseURDFFile(str(urdf_path), locator)
    model = SRDFModel()
    model.initString(scene_graph, _srdf_string(), locator)

    group_tcps = model.kinematics_information.group_tcps

    assert {g: set(t) for g, t in group_tcps.items()} == {g: set(t) for g, t in NAMED_TCPS.items()}
    for group, tcps in NAMED_TCPS.items():
        for name, (xyz, (kind, values)) in tcps.items():
            _assert_tcp(group_tcps[group][name], xyz, kind, values)


def test_environment_keeps_parsed_group_tcps(env):
    group_tcps = env.getKinematicsInformation().group_tcps

    for group, tcps in NAMED_TCPS.items():
        for name, (xyz, (kind, values)) in tcps.items():
            _assert_tcp(group_tcps[group][name], xyz, kind, values)


@pytest.mark.parametrize(
    ("group", "name"), [(g, n) for g, tcps in NAMED_TCPS.items() for n in tcps]
)
def test_find_tcp_offset_by_name_is_per_group(env, group, name):
    """`laser` resolves to a different transform in each group: lookup is keyed by `manipulator`."""
    xyz, (kind, values) = NAMED_TCPS[group][name]

    offset = env.findTCPOffset(ManipulatorInfo(group, "base_link", "tool0", name))

    _assert_tcp(offset, xyz, kind, values)


def test_find_tcp_offset_returns_isometry_unchanged(env):
    matrix = np.eye(4)
    matrix[:3, 3] = (0.3, -0.1, 0.05)
    offset = env.findTCPOffset(
        ManipulatorInfo("manipulator", "base_link", "tool0", Isometry3d(matrix))
    )
    np.testing.assert_array_equal(offset.matrix, matrix)


def test_find_tcp_offset_rejects_link_name(env):
    """A scene link is a `tcp_frame`, never a named offset (environment.cpp:839-842)."""
    with pytest.raises(RuntimeError, match="should not be an existing link"):
        env.findTCPOffset(ManipulatorInfo("manipulator", "base_link", "tool0", "link_7"))


def test_find_tcp_offset_name_from_other_group_is_not_found(env):
    """`welder` exists only in `manipulator`; no fallback across groups (environment.cpp:845-861)."""
    with pytest.raises(RuntimeError, match="Could not find tcp by name"):
        env.findTCPOffset(
            ManipulatorInfo("manipulator_joint_group", "base_link", "tool0", "welder")
        )
