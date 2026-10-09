"""KinematicsPluginFactory: the full plugin API, and solvers tied to the factory (gh-211).

Solvers created by the factory run code from plugin libraries the factory's PluginLoader
loaded, so each solver keeps its factory alive (the gh-72 keep_alive<0, 1> rule).
"""

import gc
import subprocess
import sys

import numpy as np
import numpy.testing as nptest
import pytest
import yaml

from tesseract_robotics import tesseract_common, tesseract_kinematics, tesseract_state_solver
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory

from .test_kdl_kinematics import get_plugin_factory, get_scene_graph

GROUP = "manipulator"
FWD = "Fwd"
INV = "Inv"
KINDS = [FWD, INV]
# Configured in lbr_iiwa_14_r820_plugins.yaml: one fwd solver, two inv solvers.
CONFIGURED = {FWD: {"KDLFwdKinChain"}, INV: {"KDLInvKinChainLMA", "KDLInvKinChainNR"}}
DEFAULT = {FWD: "KDLFwdKinChain", INV: "KDLInvKinChainLMA"}
_IIWA_Q = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])


@pytest.fixture
def factory():
    factory, locator = get_plugin_factory()
    yield factory
    del factory
    del locator
    gc.collect()


def _method(factory, template, kind):
    return getattr(factory, template.format(kind=kind))


def _scene():
    scene_graph = get_scene_graph()
    scene_state = tesseract_state_solver.KDLStateSolver(scene_graph).getState(np.zeros((7,)))
    return scene_graph, scene_state


@pytest.mark.parametrize("kind", KINDS)
def test_get_plugins_keyed_by_group(factory, kind):
    plugins = _method(factory, "get{kind}KinPlugins", kind)()
    assert isinstance(plugins, dict)
    assert set(plugins) == {GROUP}
    container = plugins[GROUP]
    assert isinstance(container, tesseract_common.PluginInfoContainer)
    assert set(container.plugins) == CONFIGURED[kind]
    assert container.default_plugin == DEFAULT[kind]


@pytest.mark.parametrize("kind", KINDS)
def test_add_set_default_remove_plugin(factory, kind):
    info = _method(factory, "get{kind}KinPlugins", kind)()[GROUP].plugins[DEFAULT[kind]]

    _method(factory, "add{kind}KinPlugin", kind)(GROUP, "Copied", info)
    added = _method(factory, "get{kind}KinPlugins", kind)()[GROUP].plugins
    assert added["Copied"] == info

    _method(factory, "setDefault{kind}KinPlugin", kind)(GROUP, "Copied")
    assert _method(factory, "getDefault{kind}KinPlugin", kind)(GROUP) == "Copied"

    _method(factory, "remove{kind}KinPlugin", kind)(GROUP, "Copied")
    remaining = _method(factory, "get{kind}KinPlugins", kind)()[GROUP]
    assert "Copied" not in remaining.plugins
    # Removing the default clears it; upstream then answers the first solver by name
    # (kinematics_plugin_factory.cpp:179-180 @ 0.35.0).
    assert remaining.default_plugin == ""
    assert _method(factory, "getDefault{kind}KinPlugin", kind)(GROUP) == min(CONFIGURED[kind])


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("template", ["remove{kind}KinPlugin", "setDefault{kind}KinPlugin"])
@pytest.mark.parametrize(
    ("group", "solver"),
    [("no_such_group", "KDLFwdKinChain"), (GROUP, "NoSuchSolver")],
    ids=["unknown_group", "unknown_solver"],
)
def test_unknown_group_or_solver_raises_key_error(factory, kind, template, group, solver):
    before = _method(factory, "get{kind}KinPlugins", kind)()
    with pytest.raises(KeyError, match=f"'{solver}'.*'{group}'"):
        _method(factory, template, kind)(group, solver)
    assert _method(factory, "get{kind}KinPlugins", kind)() == before


@pytest.mark.parametrize("kind", KINDS)
def test_remove_last_solver_of_group_is_refused(factory, kind):
    """Upstream reads and writes through an erased iterator here (cpp:150-154 @ 0.35.0)."""
    for solver in sorted(CONFIGURED[kind])[1:]:
        _method(factory, "remove{kind}KinPlugin", kind)(GROUP, solver)
    before = _method(factory, "get{kind}KinPlugins", kind)()
    (last,) = before[GROUP].plugins

    with pytest.raises(
        tesseract_kinematics.KinematicsPluginRemovalError, match=f"last solver of group '{GROUP}'"
    ):
        _method(factory, "remove{kind}KinPlugin", kind)(GROUP, last)
    assert _method(factory, "get{kind}KinPlugins", kind)() == before


def test_removal_error_is_runtime_error():
    assert issubclass(tesseract_kinematics.KinematicsPluginRemovalError, RuntimeError)


def test_create_fwd_kin_from_plugin_info(factory):
    scene_graph, scene_state = _scene()
    info = factory.getFwdKinPlugins()[GROUP].plugins["KDLFwdKinChain"]
    from_info = factory.createFwdKin("from_info", info, scene_graph, scene_state)
    by_name = factory.createFwdKin(GROUP, "KDLFwdKinChain", scene_graph, scene_state)
    assert isinstance(from_info, tesseract_kinematics.ForwardKinematics)
    assert from_info.getSolverName() == "from_info"
    expected = by_name.calcFwdKin(_IIWA_Q)["tool0"].matrix
    nptest.assert_array_equal(from_info.calcFwdKin(_IIWA_Q)["tool0"].matrix, expected)


def test_create_inv_kin_from_plugin_info(factory):
    scene_graph, scene_state = _scene()
    info = factory.getInvKinPlugins()[GROUP].plugins["KDLInvKinChainLMA"]
    from_info = factory.createInvKin("from_info", info, scene_graph, scene_state)
    by_name = factory.createInvKin(GROUP, "KDLInvKinChainLMA", scene_graph, scene_state)
    assert isinstance(from_info, tesseract_kinematics.InverseKinematics)
    assert from_info.getSolverName() == "from_info"
    fwd = factory.createFwdKin(GROUP, "KDLFwdKinChain", scene_graph, scene_state)
    target = {"tool0": fwd.calcFwdKin(_IIWA_Q)["tool0"]}
    seed = np.zeros(7)
    expected = by_name.calcInvKin(target, seed)
    actual = from_info.calcInvKin(target, seed)
    assert len(expected) > 0
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        nptest.assert_array_equal(a, e)


def test_get_config_is_yaml_with_both_plugin_sections(factory):
    section = yaml.safe_load(factory.getConfig())["kinematic_plugins"]
    assert set(section["fwd_kin_plugins"][GROUP]["plugins"]) == CONFIGURED[FWD]
    assert set(section["inv_kin_plugins"][GROUP]["plugins"]) == CONFIGURED[INV]


def test_save_config_round_trips(factory, tmp_path):
    path = tmp_path / "kin.yaml"
    factory.saveConfig(path)
    _, locator = get_plugin_factory()
    reloaded = KinematicsPluginFactory(path, locator)
    assert reloaded.getFwdKinPlugins() == factory.getFwdKinPlugins()
    assert reloaded.getInvKinPlugins() == factory.getInvKinPlugins()


def test_save_config_missing_directory_raises(factory, tmp_path):
    with pytest.raises(FileNotFoundError):
        factory.saveConfig(tmp_path / "missing" / "kin.yaml")


_SOLVER_OUTLIVES_FACTORY_SCRIPT = """\
import gc
import os
from pathlib import Path

import numpy as np

import tesseract_robotics  # noqa: F401 - sets TESSERACT_SUPPORT_DIR
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory
from tesseract_robotics.tesseract_state_solver import KDLStateSolver
from tesseract_robotics.tesseract_urdf import parseURDFFile

support = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf"
locator = GeneralResourceLocator()
scene_graph = parseURDFFile(str(support / "lbr_iiwa_14_r820.urdf"), locator)
scene_state = KDLStateSolver(scene_graph).getState(np.zeros(7))
factory = KinematicsPluginFactory(support / "lbr_iiwa_14_r820_plugins.yaml", locator)
info = factory.get{kind}KinPlugins()["manipulator"].plugins["{solver}"]
solver = factory.create{kind}Kin("from_info", info, scene_graph, scene_state)
del factory, info
gc.collect()
{call}
print("OK:", solver.numJoints(), solver.getSolverName())
"""

_FWD_CALL = 'assert "tool0" in solver.calcFwdKin(np.zeros(7))'
_INV_CALL = (
    "from tesseract_robotics.tesseract_common import Isometry3d\n"
    "pose = np.eye(4)\n"
    "pose[2, 3] = 1.306\n"
    'assert len(solver.calcInvKin({"tool0": Isometry3d(pose)}, np.array([-0.785398, 0.785398] * 3 + [-0.785398]))) > 0'
)


@pytest.mark.parametrize(
    ("kind", "solver", "call"),
    [(FWD, "KDLFwdKinChain", _FWD_CALL), (INV, "KDLInvKinChainLMA", _INV_CALL)],
    ids=["fwd", "inv"],
)
def test_solver_from_plugin_info_outlives_factory(kind, solver, call):
    """gh-72 rule: `del factory` must not dlclose the plugin the solver runs from.

    Before keep_alive<0, 1> was added to the PluginInfo overloads, both cases already
    passed on macOS (2026-10-09, `.scratch/phaseD-b-211-lifetime-red.log`), as the gh-170
    by-name case did. The tie is kept anyway, as the gh-72 rule requires, since unmapping
    on dlclose is platform-dependent.
    """
    script = _SOLVER_OUTLIVES_FACTORY_SCRIPT.format(kind=kind, solver=solver, call=call)
    proc = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 0, f"rc={proc.returncode} (SIGSEGV is -11): {proc.stderr[-800:]}"
    assert "OK: 7 from_info" in proc.stdout
