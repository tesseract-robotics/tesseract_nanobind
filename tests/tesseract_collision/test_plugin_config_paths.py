"""A plugin-factory config path the locator cannot resolve raises FileNotFoundError.

Upstream's loadYamlFile dereferences the resource the locator returns without a null check, so
a path the locator does not handle (a relative path, for GeneralResourceLocator) crashed the
interpreter. Each case runs in a child interpreter, so a regression is a failed test, not a
crashed run.
"""

import subprocess
import sys

import pytest

_SETUP = """
from pathlib import Path
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import ContactManagersPluginFactory
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.kinematics import KinematicsPluginFactory
from tesseract_robotics.task_composer import TaskComposerPluginFactory, TaskComposerServer
locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
"""

CALLS = {
    "contact_managers_relative": "ContactManagersPluginFactory(Path('cm_missing.yaml'), locator)",
    "contact_managers_absolute": "ContactManagersPluginFactory(Path('/nonexistent/cm.yaml'), locator)",
    "kinematics": "KinematicsPluginFactory(Path('kin_missing.yaml'), locator)",
    "task_composer_factory": "TaskComposerPluginFactory(Path('tc_missing.yaml'), locator)",
    "task_composer_factory_load": "TaskComposerPluginFactory().loadConfig(Path('tc_missing.yaml'), locator)",
    "task_composer_server": "TaskComposerServer().loadConfig(Path('tc_missing.yaml'), locator)",
}


@pytest.mark.parametrize("call", sorted(CALLS))
def test_unresolvable_config_path_raises_file_not_found(call, tmp_path):
    script = (
        _SETUP
        + f"""
try:
    {CALLS[call]}
except FileNotFoundError as error:
    print("RAISED", error.filename)
"""
    )
    proc = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, cwd=tmp_path, check=False
    )
    assert proc.returncode == 0, f"child died (rc={proc.returncode}): {proc.stderr[-1500:]}"
    assert "RAISED" in proc.stdout, proc.stdout
