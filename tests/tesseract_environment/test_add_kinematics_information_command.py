"""Tests for AddKinematicsInformationCommand + the common KinematicsPluginInfo bindings.

Registers a kinematic group (with a KDL fwd/inv IK plugin) into a LIVE env at
runtime and verifies getKinematicGroup + calcInvKin then work, and that the
command insert-MERGES (a second add does not clobber the first). Also covers the
PluginInfo.config str round-trip (config is a YAML::Node in C++, exposed as a
str via getConfigString / YAML::Load).
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    KinematicsPluginInfo,
    PluginInfo,
    PluginInfoContainer,
)
from tesseract_robotics.tesseract_environment import (
    AddKinematicsInformationCommand,
    Command,
    Environment,
)
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs
from tesseract_robotics.tesseract_srdf import KinematicsInformation

# A self-contained 6-revolute chain (base_link -> link_1..6 -> tool0) — enough
# DOF for a full 6-DOF IK round trip, no meshes / no package:// so a plain
# GeneralResourceLocator resolves it.
_LINKS = (
    '<link name="base_link"/>'
    + "".join(f'<link name="link_{i}"/>' for i in range(1, 7))
    + '<link name="tool0"/>'
)


def _joints() -> str:
    out, parent = "", "base_link"
    for i in range(1, 7):
        out += (
            f'<joint name="joint_{i}" type="revolute"><parent link="{parent}"/>'
            f'<child link="link_{i}"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>'
            f'<limit lower="-3.14" upper="3.14" effort="10" velocity="2"/></joint>'
        )
        parent = f"link_{i}"
    out += f'<joint name="joint_tool" type="fixed"><parent link="{parent}"/><child link="tool0"/><origin xyz="0 0 0.1"/></joint>'
    return out


SIX_DOF_URDF = (
    '<robot name="mini6" xmlns:tesseract="http://ros.org/wiki/tesseract" '
    f'tesseract:make_convex="true">{_LINKS}{_joints()}</robot>'
)


@pytest.fixture
def env():
    environment = Environment()
    assert environment.init(SIX_DOF_URDF, GeneralResourceLocator())
    return environment


def _kdl_container(factory_class: str, name: str, base: str, tip: str) -> PluginInfoContainer:
    pi = PluginInfo()
    pi.class_name = factory_class
    pi.config = f"base_link: {base}\ntip_link: {tip}"
    container = PluginInfoContainer()
    container.default_plugin = name
    container.plugins = {name: pi}
    return container


def _kinematics_information(group: str, base: str, tip: str) -> KinematicsInformation:
    ki = KinematicsInformation()
    ki.addChainGroup(group, [(base, tip)])
    kpi = KinematicsPluginInfo()
    kpi.search_libraries = ["tesseract_kinematics_kdl_factories"]
    kpi.fwd_plugin_infos = {
        group: _kdl_container("KDLFwdKinChainFactory", "KDLFwdKinChain", base, tip)
    }
    kpi.inv_plugin_infos = {
        group: _kdl_container("KDLInvKinChainLMAFactory", "KDLInvKinChainLMA", base, tip)
    }
    ki.kinematics_plugin_info = kpi
    return ki


def test_plugin_info_config_str_round_trip():
    pi = PluginInfo()
    pi.class_name = "KDLFwdKinChainFactory"
    pi.config = "base_link: base_link\ntip_link: tool0"
    assert pi.class_name == "KDLFwdKinChainFactory"
    assert "tip_link: tool0" in pi.config
    assert pi.getConfigString() == pi.config


class TestAddKinematicsInformationCommand:
    def test_constructor_default(self):
        cmd = AddKinematicsInformationCommand()
        assert isinstance(cmd, Command)

    def test_constructor_with_kinematics_information(self):
        ki = _kinematics_information("manipulator", "base_link", "tool0")
        cmd = AddKinematicsInformationCommand(ki)
        assert isinstance(cmd, Command)
        assert cmd.getKinematicsInformation().hasChainGroup("manipulator")

    def test_apply_registers_group_and_ik_resolves(self, env):
        assert "manipulator" not in env.getGroupNames()

        ki = _kinematics_information("manipulator", "base_link", "tool0")
        assert env.applyCommand(AddKinematicsInformationCommand(ki))
        assert "manipulator" in env.getGroupNames()

        kg = env.getKinematicGroup("manipulator")
        assert kg is not None
        assert len(list(kg.getJointNames())) == 6

        # FK -> IK round trip actually solves through the registered KDL plugin.
        q = np.array([0.1, 0.2, -0.15, 0.05, 0.3, 0.12])
        target = kg.calcFwdKin(q)["tool0"]
        ik_in = KinGroupIKInput()
        ik_in.pose = target
        ik_in.tip_link_name = "tool0"
        ik_in.working_frame = "base_link"
        ik_ins = KinGroupIKInputs()
        ik_ins.append(ik_in)
        solutions = kg.calcInvKin(ik_ins, np.zeros(6))
        assert solutions is not None and len(solutions) > 0
        back = kg.calcFwdKin(np.asarray(solutions[0]))["tool0"]
        delta_mm = (
            float(np.linalg.norm(np.asarray(back.matrix)[:3, 3] - np.asarray(target.matrix)[:3, 3]))
            * 1000.0
        )
        assert delta_mm < 1e-2

    def test_apply_insert_merges(self, env):
        assert env.applyCommand(
            AddKinematicsInformationCommand(
                _kinematics_information("manipulator", "base_link", "tool0")
            )
        )
        assert env.applyCommand(
            AddKinematicsInformationCommand(_kinematics_information("manip2", "base_link", "tool0"))
        )
        # Second add must NOT clobber the first — insert-merges.
        assert "manipulator" in env.getGroupNames()
        assert "manip2" in env.getGroupNames()
        assert env.getKinematicGroup("manipulator") is not None
        assert env.getKinematicGroup("manip2") is not None
