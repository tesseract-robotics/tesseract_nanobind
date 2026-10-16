"""trajopt_ifopt names JointGroup, KinematicGroup and Environment as Python classes (#285).

nanobind renders a parameter whose type is not registered yet as its quoted C++ name. The
extension imports the modules that register these three types, so its signatures and the
generated stub name the classes, also when trajopt_ifopt is the first module imported.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import tesseract_robotics.trajopt_ifopt as trajopt_ifopt

# s: a child imports trajopt_ifopt and reads a few signatures
CHILD_TIMEOUT_S = 60.0

STUB = Path(trajopt_ifopt.__file__).parent / "_trajopt_ifopt.pyi"

_CHILD_SCRIPT = """\
import tesseract_robotics.trajopt_ifopt as ti

signatures = [
    ti.CartPosConstraint.__init__.__nb_signature__,
    ti.SingleTimestepCollisionEvaluator.__init__.__nb_signature__,
    ti.LVSDiscreteCollisionEvaluator.__init__.__nb_signature__,
    ti.LVSContinuousCollisionEvaluator.__init__.__nb_signature__,
    ti.CartLineInfo.manip.fget.__nb_signature__,
    ti.InverseKinematicsInfo.manip.fget.__nb_signature__,
]
print(repr(signatures))
"""


def test_first_import_names_the_classes():
    child = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT],
        capture_output=True,
        text=True,
        timeout=CHILD_TIMEOUT_S,
        check=True,
    )
    assert "tesseract::" not in child.stdout
    assert "tesseract_kinematics._tesseract_kinematics.JointGroup" in child.stdout
    assert "tesseract_kinematics._tesseract_kinematics.KinematicGroup" in child.stdout
    assert "tesseract_environment._tesseract_environment.Environment" in child.stdout


def test_stub_has_no_quoted_cpp_names():
    assert '"tesseract::' not in STUB.read_text()
