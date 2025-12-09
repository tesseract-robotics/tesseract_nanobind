"""Pytest configuration for tesseract_robotics tests.

Handles cleanup to avoid segfaults from Python's non-deterministic
garbage collection order at interpreter shutdown.

Custom markers:
  - viewer: Viewer/visualization examples
  - planning: Motion planning examples (require TESSERACT_TASK_COMPOSER_CONFIG_FILE)
  - basic: Basic examples (collision, kinematics, scene_graph)
"""

import os
from pathlib import Path

# Set up env vars BEFORE importing tesseract_robotics
# For dev testing, use ws/src/ paths if bundled data doesn't exist
_tests_dir = Path(__file__).parent
_project_root = _tests_dir.parent.parent  # tesseract_nanobind/tests -> tesseract_nanobind -> project_root
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

import gc
import pytest


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "viewer: marks tests as viewer examples")
    config.addinivalue_line("markers", "planning: marks tests as planning examples")
    config.addinivalue_line("markers", "basic: marks tests as basic examples")


@pytest.fixture(autouse=True)
def cleanup_after_test():
    """Force garbage collection after each test to ensure proper cleanup order."""
    yield
    # Force garbage collection to clean up C++ objects before interpreter shutdown
    gc.collect()
    gc.collect()  # Run twice to handle cyclic references


def pytest_sessionfinish(session, exitstatus):
    """Clean up at end of test session."""
    gc.collect()
    gc.collect()
