"""Pytest configuration for tesseract_robotics tests.

Provides unified object lifetime management via TesseractContext fixture.
C++ bindings require objects to be deleted in correct order - locators
must outlive environments that use them, etc.

IMPORTANT - Test Isolation:
  Due to KinematicsPluginFactory global state in the C++ library, tests using
  different kinematics plugin types (KDL vs OPW) CANNOT run in the same process.
  See docs/KINEMATICS_PLUGIN_FACTORY_ISSUE.md for details.

  For full test suite: Use ./run_tests.sh which runs tests in isolated phases.
  For individual test categories: pytest -m <marker> tests/

Custom markers:
  - forked: Tests requiring process isolation (use with pytest-forked)
  - viewer: Viewer/visualization examples
  - planning: Motion planning examples (require TESSERACT_TASK_COMPOSER_CONFIG_FILE)
  - basic: Basic examples (collision, kinematics, scene_graph)
"""

import os
from pathlib import Path

# Force TESSERACT_HEADLESS=1 to prevent visualization server stalling tests
os.environ["TESSERACT_HEADLESS"] = "1"

# Set up env vars BEFORE importing tesseract_robotics
_tests_dir = Path(__file__).parent
_project_root = _tests_dir.parent.parent
_ws_src = _project_root / "ws" / "src"

# TESSERACT_SUPPORT_DIR
_tesseract_support = _ws_src / "tesseract" / "tesseract_support"
if _tesseract_support.exists() and not os.environ.get("TESSERACT_SUPPORT_DIR"):
    os.environ["TESSERACT_SUPPORT_DIR"] = str(_tesseract_support)

# TESSERACT_RESOURCE_PATH
_tesseract_resource = _ws_src / "tesseract"
if _tesseract_resource.exists() and not os.environ.get("TESSERACT_RESOURCE_PATH"):
    os.environ["TESSERACT_RESOURCE_PATH"] = str(_tesseract_resource)

# TESSERACT_TASK_COMPOSER_DIR
_task_composer = _ws_src / "tesseract_planning" / "tesseract_task_composer"
if _task_composer.exists() and not os.environ.get("TESSERACT_TASK_COMPOSER_DIR"):
    os.environ["TESSERACT_TASK_COMPOSER_DIR"] = str(_task_composer)

# TESSERACT_TASK_COMPOSER_CONFIG_FILE
_config_file = _task_composer / "config" / "task_composer_plugins.yaml"
if _config_file.exists() and not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
    os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(_config_file)

# Now import tesseract_robotics (its _configure_environment will be a no-op since we set vars)
import tesseract_robotics  # noqa: F401

import pytest
from .context import TesseractContext
from .tesseract_support_resource_locator import TesseractSupportResourceLocator
from tesseract_robotics.tesseract_common import FilesystemPath, ManipulatorInfo, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

TESSERACT_SUPPORT_DIR = os.environ.get("TESSERACT_SUPPORT_DIR", "")


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "forked: tests requiring process isolation (KinematicsPluginFactory)")
    config.addinivalue_line("markers", "viewer: marks tests as viewer examples")
    config.addinivalue_line("markers", "planning: marks tests as planning examples")
    config.addinivalue_line("markers", "basic: marks tests as basic examples")


@pytest.fixture
def ctx():
    """Provide TesseractContext for managing C++ object lifetimes.

    Use ctx.keep(obj) to ensure obj stays alive until test cleanup.
    Cleanup happens automatically in reverse order (LIFO).
    """
    context = TesseractContext()
    yield context
    context.cleanup()


@pytest.fixture
def abb_env(ctx):
    """ABB IRB2400 environment with OPW kinematics.

    Returns (env, manip_info, joint_names).
    Locator kept alive via ctx.
    """
    if not TESSERACT_SUPPORT_DIR:
        pytest.skip("TESSERACT_SUPPORT_DIR not set")

    locator = ctx.keep(TesseractSupportResourceLocator())
    env = Environment()
    urdf = os.path.join(TESSERACT_SUPPORT_DIR, "urdf/abb_irb2400.urdf")
    srdf = os.path.join(TESSERACT_SUPPORT_DIR, "urdf/abb_irb2400.srdf")
    if not env.init(urdf, srdf, locator):
        pytest.skip("Failed to init ABB IRB2400 environment")

    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names


@pytest.fixture
def iiwa_env(ctx):
    """IIWA LBR 14 R820 environment with KDL kinematics.

    Returns (env, manip_info, joint_names).
    Locator kept alive via ctx.
    """
    if not TESSERACT_SUPPORT_DIR:
        pytest.skip("TESSERACT_SUPPORT_DIR not set")

    locator = ctx.keep(TesseractSupportResourceLocator())
    env = Environment()
    urdf = os.path.join(TESSERACT_SUPPORT_DIR, "urdf/lbr_iiwa_14_r820.urdf")
    srdf = os.path.join(TESSERACT_SUPPORT_DIR, "urdf/lbr_iiwa_14_r820.srdf")
    if not env.init(urdf, srdf, locator):
        pytest.skip("Failed to init IIWA environment")

    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names


@pytest.fixture
def simple_env(ctx):
    """Simple 2-link robot environment for command tests.

    Returns env. Locator kept alive via ctx.
    """
    SIMPLE_URDF = """
<robot name="test_robot" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="false">
  <link name="world"/>
  <link name="link1">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="link2">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
</robot>
"""
    locator = ctx.keep(GeneralResourceLocator())
    env = Environment()
    env.init(SIMPLE_URDF, locator)
    return env
