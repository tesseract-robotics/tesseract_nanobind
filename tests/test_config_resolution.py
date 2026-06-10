"""Contract tests for get_task_composer_config_path resolution (gh-110).

The helper must return the same env-var-published, plugin-path-patched config
the runtime uses — in every install flavour (wheel, editable/conda) — and fail
loudly instead of returning a phantom path.
"""

import pytest

from tesseract_robotics import TaskComposerConfigNotFoundError, get_task_composer_config_path


def test_resolves_to_existing_yaml():
    """Helper returns an existing config in every supported environment."""
    config = get_task_composer_config_path()
    assert config.is_file()
    assert config.suffix == ".yaml"


def test_resolved_config_is_plugin_path_patched():
    """The returned config is loadable as-is — no unresolved plugin placeholders."""
    assert "@PLUGIN_PATH@" not in get_task_composer_config_path().read_text()


def test_stale_env_var_raises(monkeypatch):
    """A stale TESSERACT_TASK_COMPOSER_CONFIG_FILE raises, not a nonexistent Path."""
    monkeypatch.setenv(
        "TESSERACT_TASK_COMPOSER_CONFIG_FILE", "/nonexistent/task_composer_plugins.yaml"
    )
    with pytest.raises(TaskComposerConfigNotFoundError):
        get_task_composer_config_path()
