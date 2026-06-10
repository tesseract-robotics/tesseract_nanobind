"""Lifetime of plugin-backed objects returned by Environment (gh-72).

Environment owns the plugin loader (boost_plugin_loader): destroying it
dlcloses the kinematics/collision plugin dylibs. Objects created BY those
plugins (OPWInvKin inside KinematicGroup, contact managers) carry vtables
that live inside the plugin dylibs — if the Environment dies first, their
destructors virtual-call into unmapped pages and the process dies with
SIGSEGV. Python destruction order (interpreter teardown, pytest GC) does not
honor C++ ownership contracts, so the bindings must: `nb::keep_alive<0, 1>`
ties every plugin-backed object to its Environment.
"""

import gc
import subprocess
import sys

import numpy as np

from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

# Byte-equivalent to the gh-72 forensic reproducer: module globals in this
# exact order made interpreter teardown destroy env (unmapping the OPW plugin
# dylib) before kg -> EXC_BAD_ACCESS in ~KinematicGroup. Deterministic SIGSEGV
# (exit 139) before the fix.
_TEARDOWN_SCRIPT = """\
import numpy as np
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
env = Environment()
assert env.init(FilesystemPath(urdf), FilesystemPath(srdf), locator)
kg = env.getKinematicGroup("manipulator")
J = kg.calcJacobian(np.array([0.0, 0.0, 0.5, 0.0, 0.5, 0.0]), "tool0")
print("OK:", J.shape)
"""


def _make_env():
    locator = GeneralResourceLocator()
    urdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    srdf = locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    env = Environment()
    assert env.init(FilesystemPath(urdf), FilesystemPath(srdf), locator)
    return env


def test_interpreter_teardown_survives_group_outliving_env():
    """The gh-72 crash: clean interpreter exit with env+group as module globals.

    On Linux this additionally depends on the wheel shipping no libstdc++ of
    its own (gh-119) — with a bundled copy present the child died earlier, in
    `env.init`, from dual-libstdc++ heap corruption.
    """
    proc = subprocess.run(
        [sys.executable, "-c", _TEARDOWN_SCRIPT],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, (
        f"interpreter teardown died (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[-500:]}"
    )
    assert "OK: (6, 6)" in proc.stdout


def test_kinematic_group_usable_after_env_release():
    """keep_alive contract: group keeps the Environment (and plugin libs) alive."""
    env = _make_env()
    kg = env.getKinematicGroup("manipulator")
    del env
    gc.collect()
    J = kg.calcJacobian(np.array([0.0, 0.0, 0.5, 0.0, 0.5, 0.0]), "tool0")
    assert J.shape == (6, 6)


def test_joint_group_usable_after_env_release():
    env = _make_env()
    jg = env.getJointGroup("manipulator")
    del env
    gc.collect()
    state = jg.calcFwdKin(np.zeros(6))
    assert "tool0" in state.keys()


def test_contact_manager_usable_after_env_release():
    """Contact managers are plugin-created (bullet/fcl factories) — same hazard."""
    env = _make_env()
    manager = env.getDiscreteContactManager()
    del env
    gc.collect()
    assert len(manager.getActiveCollisionObjects()) > 0
