"""
Kinematics Plugins Example
==========================

Builds IIWA forward and inverse kinematics from the kinematics plugin YAML,
switches the inverse solver, edits the factory and round-trips its config,
then solves IK through a `KinematicGroup` built without an `Environment`.

C++ Reference:
    kinematics/test/kinematics_test_utils.h  (runInvKinIIWATest, runFwdKinIIWATest)
    kinematics/test/kinematics_factory_unit.cpp  (PluginFactorAPIUnit)
    support/urdf/lbr_iiwa_14_r820_plugins.yaml

Overview
--------
1. `KinematicsPluginFactory(Path, locator)` from the plugin YAML the IIWA SRDF
   names; list its solvers per group.
2. Create forward and inverse kinematics by group name and by `PluginInfo`.
3. Switch the default inverse solver from KDL LMA to KDL NR and solve upstream's
   IK problem with each: target = FK at q = 0, seed = ±0.785398.
4. Add, set-default and remove a solver, as upstream's `PluginFactorAPIUnit`.
5. `saveConfig`, rebuild a factory from the file, compare `getConfig()`.
6. A `KinematicGroup` straight from the factory's solver, solved with a list of
   `KinGroupIKInput`.

Key Concepts
------------
**Convergence check, not a guessed tolerance**:
    Each IK solution is checked with the solver's own stopping rule, evaluated
    on the error twist KDL computes, `diff(FK(q), target)` in the base frame:

    - LMA (`KDL::ChainIkSolverPos_LMA`, orocos_kdl 1.5.3) stops when the
      task-weighted twist norm `‖L ∘ Δ‖₂ < eps`, on the q it returns.
    - NR (`KDL::ChainIkSolverPos_NR`) stops when every twist component is below
      `eps`, but tests the iterate *before* its last Newton update. The returned
      q is one step further, so the bound holds as long as that step does not
      increase the error, which is what a converged Newton step does.

**Default solver fallback**:
    Removing a group's default solver makes the first remaining solver (by name)
    the default; removing its last solver raises `KinematicsPluginRemovalError`.
"""

import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    PluginInfo,
    calcTransformError,
)
from tesseract_robotics.tesseract_kinematics import (
    ForwardKinematics,
    KinematicGroup,
    KinematicsPluginFactory,
    KinGroupIKInput,
)
from tesseract_robotics.tesseract_state_solver import KDLStateSolver
from tesseract_robotics.tesseract_urdf import parseURDFFile

URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
PLUGINS_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820_plugins.yaml"
GROUP = "manipulator"
BASE_LINK = "base_link"
TIP_LINK = "tool0"
JOINT_NAMES = [f"joint_a{i}" for i in range(1, 8)]

# Upstream runInvKinIIWATest seed (kinematics_test_utils.h:1038-1046), rad
SEED = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])

# KDLInvKinChainLMA::Config defaults (kdl_inv_kin_chain_lma.h:66-67): task weights
# [x, y, z, rx, ry, rz] (dimensionless) and eps, the bound on the weighted twist
# norm (m, or rad scaled by the 0.1 weight)
LMA_TASK_WEIGHTS = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
LMA_EPS = 1e-5
# KDLInvKinChainNR::Config pos_eps (kdl_inv_kin_chain_nr.h:69): per-component
# bound on the error twist, m for translation and rad for rotation
NR_POS_EPS = 1e-6


def kdl_error_twist(head: Isometry3d, goal: Isometry3d) -> np.ndarray:
    """KDL's `diff(head, goal)`: the error twist in the base frame.

    `calcTransformError(head, goal)` is that twist expressed in the `head` frame;
    rotating both halves by `head`'s rotation gives the base-frame twist.
    """
    error = calcTransformError(head, goal)
    rotation = head.rotation
    return np.concatenate([rotation @ error[:3], rotation @ error[3:]])


def convergence(solver_name: str, twist: np.ndarray) -> tuple[float, float]:
    """The solver's stopping quantity for `twist` and its bound."""
    if solver_name == "KDLInvKinChainLMA":
        return float(np.linalg.norm(LMA_TASK_WEIGHTS * twist)), LMA_EPS
    return float(np.abs(twist).max()), NR_POS_EPS


def check_solutions(solver_name, solutions, fwd_kin: ForwardKinematics, target: Isometry3d) -> dict:
    """Run each IK solution through FK and the solver's own convergence test."""
    criteria = [
        convergence(solver_name, kdl_error_twist(fwd_kin.calcFwdKin(q)[TIP_LINK], target))
        for q in solutions
    ]
    return {
        "num_solutions": len(solutions),
        "criterion": max(value for value, _ in criteria),
        "bound": criteria[0][1],
    }


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def plugin_names(plugins) -> dict[str, list[str]]:
    """Group name -> solver names of a `getFwdKinPlugins()` / `getInvKinPlugins()` map."""
    return {group: list(container.plugins) for group, container in plugins.items()}


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    # --8<-- [start:factory]
    locator = GeneralResourceLocator()
    factory = KinematicsPluginFactory(locate(locator, PLUGINS_URL), locator)

    fwd_kin_plugins = plugin_names(factory.getFwdKinPlugins())
    inv_kin_plugins = plugin_names(factory.getInvKinPlugins())
    default_inv_kin_plugin = factory.getDefaultInvKinPlugin(GROUP)
    # --8<-- [end:factory]

    scene_graph = parseURDFFile(str(locate(locator, URDF_URL)), locator)
    scene_state = KDLStateSolver(scene_graph).getState()

    # --8<-- [start:switch_solver]
    fwd_kin = factory.createFwdKin(
        GROUP, factory.getDefaultFwdKinPlugin(GROUP), scene_graph, scene_state
    )
    target = fwd_kin.calcFwdKin(np.zeros(len(JOINT_NAMES)))[TIP_LINK]

    ik = {}
    for solver_name in ["KDLInvKinChainLMA", "KDLInvKinChainNR"]:
        factory.setDefaultInvKinPlugin(GROUP, solver_name)
        inv_kin = factory.createInvKin(
            GROUP, factory.getDefaultInvKinPlugin(GROUP), scene_graph, scene_state
        )
        solutions = inv_kin.calcInvKin({TIP_LINK: target}, SEED)
        ik[solver_name] = {
            "solver_name": inv_kin.getSolverName(),
            **check_solutions(solver_name, solutions, fwd_kin, target),
        }
    # --8<-- [end:switch_solver]

    # --8<-- [start:plugin_info]
    fwd_info = factory.getFwdKinPlugins()[GROUP].plugins["KDLFwdKinChain"]
    inv_info = factory.getInvKinPlugins()[GROUP].plugins["KDLInvKinChainNR"]
    fwd_kin_from_info = factory.createFwdKin("KDLFwdKinChain", fwd_info, scene_graph, scene_state)
    inv_kin_from_info = factory.createInvKin("KDLInvKinChainNR", inv_info, scene_graph, scene_state)
    created_by_plugin_info = (fwd_kin_from_info.getSolverName(), inv_kin_from_info.getSolverName())
    # --8<-- [end:plugin_info]

    # --8<-- [start:edit_plugins]
    fwd_default = PluginInfo()
    fwd_default.class_name = fwd_info.class_name
    fwd_default.config = fwd_info.config
    factory.addFwdKinPlugin(GROUP, "default", fwd_default)
    fwd_after_add = list(factory.getFwdKinPlugins()[GROUP].plugins)
    factory.setDefaultFwdKinPlugin(GROUP, "default")
    fwd_default_after_set = factory.getDefaultFwdKinPlugin(GROUP)
    factory.removeFwdKinPlugin(GROUP, "default")

    inv_default = PluginInfo()
    inv_default.class_name = inv_info.class_name
    inv_default.config = inv_info.config
    factory.addInvKinPlugin(GROUP, "default", inv_default)
    inv_after_add = list(factory.getInvKinPlugins()[GROUP].plugins)
    factory.setDefaultInvKinPlugin(GROUP, "default")
    inv_default_after_set = factory.getDefaultInvKinPlugin(GROUP)
    factory.removeInvKinPlugin(GROUP, "default")
    # --8<-- [end:edit_plugins]

    fwd_plugin_edits = {
        "after_add": fwd_after_add,
        "default_after_set": fwd_default_after_set,
        "after_remove": list(factory.getFwdKinPlugins()[GROUP].plugins),
        "default_after_remove": factory.getDefaultFwdKinPlugin(GROUP),
    }
    inv_plugin_edits = {
        "after_add": inv_after_add,
        "default_after_set": inv_default_after_set,
        "after_remove": list(factory.getInvKinPlugins()[GROUP].plugins),
        "default_after_remove": factory.getDefaultInvKinPlugin(GROUP),
    }

    # --8<-- [start:save_config]
    with tempfile.TemporaryDirectory() as tmp:
        saved = Path(tmp) / "kinematics_plugins.yaml"
        factory.saveConfig(saved)
        config_round_trip_equal = (
            KinematicsPluginFactory(saved, locator).getConfig() == factory.getConfig()
        )
    # --8<-- [end:save_config]

    # --8<-- [start:kinematic_group]
    group = KinematicGroup(GROUP, JOINT_NAMES, inv_kin_from_info, scene_graph, scene_state)
    solutions = group.calcInvKin([KinGroupIKInput(target, BASE_LINK, TIP_LINK)], SEED)
    # --8<-- [end:kinematic_group]
    kinematic_group = check_solutions("KDLInvKinChainNR", solutions, fwd_kin, target)

    return {
        "fwd_kin_plugins": fwd_kin_plugins,
        "inv_kin_plugins": inv_kin_plugins,
        "default_inv_kin_plugin": default_inv_kin_plugin,
        "target_translation": target.translation,
        "ik": ik,
        "created_by_plugin_info": created_by_plugin_info,
        "fwd_plugin_edits": fwd_plugin_edits,
        "inv_plugin_edits": inv_plugin_edits,
        "config_round_trip_equal": config_round_trip_equal,
        "kinematic_group": kinematic_group,
    }


def main() -> None:
    """Console entry point (`tesseract_kinematics_plugins_example`): run() and print the results."""
    result = run()
    print(f"Forward solvers: {result['fwd_kin_plugins']}")
    print(
        f"Inverse solvers: {result['inv_kin_plugins']} (default {result['default_inv_kin_plugin']})"
    )
    print(f"IK target: tool0 at FK(q = 0), translation {result['target_translation']}")
    for solver_name, ik in result["ik"].items():
        print(
            f"{solver_name}: {ik['num_solutions']} solution(s), convergence {ik['criterion']:.2e} < {ik['bound']:.0e}"
        )
    print(f"Created from PluginInfo: {result['created_by_plugin_info']}")
    for kind in ["fwd", "inv"]:
        edits = result[f"{kind}_plugin_edits"]
        print(
            f"{kind} add 'default' -> {edits['after_add']}, default {edits['default_after_set']}; "
            f"remove -> {edits['after_remove']}, default {edits['default_after_remove']}"
        )
    print(f"saveConfig round trip equal: {result['config_round_trip_equal']}")
    group = result["kinematic_group"]
    print(
        f"KinematicGroup IK: {group['num_solutions']} solution(s), convergence {group['criterion']:.2e} < {group['bound']:.0e}"
    )


if __name__ == "__main__":
    main()
