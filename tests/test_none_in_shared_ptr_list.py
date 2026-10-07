"""A `None` element of a `list[shared_ptr]` argument raises `TypeError` naming its index (gh-201).

nanobind rejects `None` for a direct `std::shared_ptr<T>` argument, but converts a `None`
element of a list to a null pointer, which upstream later dereferences. Each case runs in a
child interpreter: without the guard, the call (or the first use of the object) segfaults.
`Environment.applyCommands` / `init(commands)` are covered in tests/tesseract_environment.
"""

import subprocess
import sys

import pytest

_SHAPES = """
import os
from pathlib import Path
from tesseract_robotics.tesseract_collision import ContactManagersPluginFactory
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_geometry import Box
cfg = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf" / "contact_manager_plugins.yaml"
factory = ContactManagersPluginFactory(cfg, GeneralResourceLocator())
manager = factory.create{kind}ContactManager("{plugin}")
call = lambda: manager.addCollisionObject(
    "obj", 0, [Box(1, 1, 1), None], [Isometry3d.Identity(), Isometry3d.Identity()]
)
"""

_VARS = """
import numpy as np
from tesseract_robotics import trajopt_ifopt as ti
call = lambda: ti.{cls}(np.zeros(2), [None] * 6, np.ones(1), "c")  # jerk needs >= 6
"""

CASES = {
    "CombinedContactAllowedValidator": (
        "from tesseract_robotics.tesseract_common import (\n"
        "    CombinedContactAllowedValidator, CombinedContactAllowedValidatorType)\n"
        "call = lambda: CombinedContactAllowedValidator([None], CombinedContactAllowedValidatorType.AND)\n",
        "validators[0]",
    ),
    "CompoundMesh": (
        "from tesseract_robotics.tesseract_geometry import CompoundMesh\n"
        "call = lambda: CompoundMesh([None, None])  # upstream needs more than one mesh\n",
        "meshes[0]",
    ),
    "DiscreteContactManager.addCollisionObject": (
        _SHAPES.format(kind="Discrete", plugin="BulletDiscreteBVHManager"),
        "shapes[1]",
    ),
    "ContinuousContactManager.addCollisionObject": (
        _SHAPES.format(kind="Continuous", plugin="BulletCastBVHManager"),
        "shapes[1]",
    ),
    "JointVelConstraint": (_VARS.format(cls="JointVelConstraint"), "position_vars[0]"),
    "JointAccelConstraint": (_VARS.format(cls="JointAccelConstraint"), "position_vars[0]"),
    "JointJerkConstraint": (_VARS.format(cls="JointJerkConstraint"), "position_vars[0]"),
}

_RUN = """
try:
    call()
except TypeError as e:
    print("TypeError:", e)
else:
    print("no error")
"""


@pytest.mark.parametrize("binding", sorted(CASES))
def test_none_element_raises_type_error(binding):
    setup, index = CASES[binding]
    proc = subprocess.run(
        [sys.executable, "-c", setup + _RUN], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 0, f"rc={proc.returncode} (SIGSEGV is -11): {proc.stderr[-800:]}"
    assert proc.stdout.startswith("TypeError:"), proc.stdout
    assert f"{index} is None" in proc.stdout
