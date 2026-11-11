"""Maps keyed by a link or joint come back as plain dicts keyed by name (`str`).

0.36 returns every `LinkId` / `JointId`-keyed C++ map as `dict[str, V]`, so `poses["tool0"]`
is exactly typed. The checks live in `name_keyed_maps_typed.py`, written as user code: this
module runs each one and asserts every key is a `str` (a `LinkId` compares equal to its
name, so `==` alone would not tell them apart), and type-checks that module with ty.
"""

import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.environment import Environment
from tesseract_robotics.urdf import parseURDFString

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
from . import name_keyed_maps_typed as typed
from .test_tesseract_environment import (
    _FLOATING_URDF,
    _IIWA_JOINTS,
    _fresh_env,
)

TYPED_MODULE = Path(typed.__file__)
JOINT = "joint_a1"
LINK = "tool0"
Q = np.linspace(-0.3, 0.3, len(_IIWA_JOINTS))


def _assert_str_keys(mapping: dict, expected: set[str] | None = None) -> None:
    assert type(mapping) is dict
    assert mapping, "an empty map proves nothing about its key type"
    for key in mapping:
        assert type(key) is str, f"key {key!r} is a {type(key).__name__}, expected str"
    if expected is not None:
        assert set(mapping) == expected


def _floating_env():
    floater = Environment()
    scene_graph = parseURDFString(_FLOATING_URDF, TesseractSupportResourceLocator())
    assert floater.init(scene_graph)
    return floater


def test_calc_fwd_kin_is_keyed_by_link_name():
    env = _fresh_env()
    for poses in (typed.kinematic_group_fwd_kin(env, Q), typed.joint_group_fwd_kin(env, Q)):
        _assert_str_keys(poses)
        assert LINK in poses


def test_scene_state_maps_are_keyed_by_name():
    env = _fresh_env()
    joints, _, links, joint_tfs = typed.scene_state_maps(env.getState(_IIWA_JOINTS, Q))
    _assert_str_keys(joints, set(_IIWA_JOINTS))
    _assert_str_keys(links, {str(link) for link in env.getLinkIds()})
    _assert_str_keys(joint_tfs, {str(joint) for joint in env.getJointIds()})


def test_floating_joint_maps_are_keyed_by_joint_name():
    floater = _floating_env()
    state = floater.getState()
    _assert_str_keys(state.floating_joints, {"float_joint"})
    _assert_str_keys(typed.floating_joint_values(state, "float_joint"), {"float_joint"})
    _assert_str_keys(floater.getCurrentFloatingJointValues(), {"float_joint"})
    _assert_str_keys(floater.getCurrentFloatingJointValues(["float_joint"]), {"float_joint"})


def test_environment_link_transforms_are_keyed_by_link_name():
    links, solver_links, floating = typed.environment_maps(_fresh_env(), Q)
    _assert_str_keys(links)
    _assert_str_keys(solver_links, set(links))
    assert floating == {}


def test_command_limits_are_keyed_by_joint_name():
    for limits in typed.command_limits(JOINT):
        _assert_str_keys(limits, {JOINT})


def test_field_maps_are_keyed_by_name():
    _assert_str_keys(typed.contact_manager_config(LINK), {LINK})
    for mapping in typed.kinematics_information(JOINT, LINK):
        _assert_str_keys(mapping)
    _assert_str_keys(typed.calibration_joints(JOINT), {JOINT})


def test_adjacency_map_keys_are_link_names():
    env = _fresh_env()
    adjacency = env.getSceneGraph().getAdjacencyMap(["link_5"])
    _assert_str_keys(adjacency, {"link_5", "link_6", "link_7", "tool0"})
    assert typed.adjacency_map(env.getSceneGraph(), "link_5") == list(adjacency)


def test_a_returned_map_round_trips_into_a_setter():
    env = _fresh_env()
    state = env.getState(_IIWA_JOINTS, Q)
    copy = type(state)()
    copy.link_transforms = state.link_transforms
    copy.joints = state.joints
    assert copy.joints == state.joints
    assert set(copy.link_transforms) == set(state.link_transforms)
    for name, transform in state.link_transforms.items():
        np.testing.assert_array_equal(copy.link_transforms[name].matrix, transform.matrix)


def test_a_setter_takes_names_and_ids_mixed():
    from tesseract_robotics.common import JointId

    state = type(_fresh_env().getState(_IIWA_JOINTS, Q))()
    state.joints = {"a": 1.0, JointId("b"): 2.0}
    assert state.joints == {"a": 1.0, "b": 2.0}
    assert all(type(key) is str for key in state.joints)


@pytest.mark.timeout(120)
def test_typed_reads_type_check_clean():
    ty = shutil.which("ty")
    assert ty is not None, "ty is not on PATH; run the tests in the pixi default environment"
    result = subprocess.run(
        [ty, "check", "--python", sys.prefix, "--output-format", "concise", str(TYPED_MODULE)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
