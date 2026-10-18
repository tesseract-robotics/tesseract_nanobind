"""TaskComposerPluginFactory configuration API (#321, #337) and TaskComposerServer (#323)."""

import gc

import pytest
import yaml

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    PluginInfo,
    TaskComposerPluginInfo,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerConfigError,
    TaskComposerPluginFactory,
    TaskComposerServer,
)

PIPELINE = "TrajOptPipeline"
EXECUTOR = "TaskflowExecutor"


class TestPathAndYamlConstructors:
    """`os.PathLike` is a config path, `str` is YAML text (#337)."""

    def test_path_and_text_configure_the_same_plugins(self, config_path):
        from_path = TaskComposerPluginFactory(config_path, GeneralResourceLocator())
        from_text = TaskComposerPluginFactory(config_path.read_text(), GeneralResourceLocator())
        assert set(from_path.getTaskComposerNodePlugins()) == set(
            from_text.getTaskComposerNodePlugins()
        )
        assert len(from_path.getTaskComposerNodePlugins()) == 36

    def test_str_path_is_parsed_as_yaml(self, config_path):
        """A path given as `str` is YAML text: it parses as a scalar, not the config mapping."""
        with pytest.raises(TaskComposerConfigError, match="is YAML text"):
            TaskComposerPluginFactory(str(config_path), GeneralResourceLocator())

    @pytest.mark.parametrize("text", ["", "~", "[1, 2]", "foo: bar"])
    @pytest.mark.parametrize("caller", ["init", "loadConfig", "server"])
    def test_text_without_plugins_key_raises(self, caller, text):
        """Empty, null, list and unrelated YAML loaded silently as an empty config (#337)."""
        load = {
            "init": lambda: TaskComposerPluginFactory(text, GeneralResourceLocator()),
            "loadConfig": lambda: TaskComposerPluginFactory().loadConfig(
                text, GeneralResourceLocator()
            ),
            "server": lambda: TaskComposerServer().loadConfig(text, GeneralResourceLocator()),
        }[caller]
        with pytest.raises(TaskComposerConfigError, match="task_composer_plugins"):
            load()

    def test_config_error_is_a_value_error(self):
        assert issubclass(TaskComposerConfigError, ValueError)

    def test_load_config_str_path_raises_config_error(self, config_path):
        with pytest.raises(TaskComposerConfigError, match="loadConfig"):
            TaskComposerPluginFactory().loadConfig(str(config_path), GeneralResourceLocator())

    def test_server_load_config_str_path_raises_config_error(self, config_path):
        with pytest.raises(TaskComposerConfigError, match="TaskComposerServer.loadConfig"):
            TaskComposerServer().loadConfig(str(config_path), GeneralResourceLocator())

    def test_default_then_load_config_path(self, config_path):
        factory = TaskComposerPluginFactory()
        assert not factory.hasTaskComposerNodePlugins()
        factory.loadConfig(config_path, GeneralResourceLocator())
        expected = TaskComposerPluginFactory(config_path, GeneralResourceLocator())
        assert set(factory.getTaskComposerNodePlugins()) == set(
            expected.getTaskComposerNodePlugins()
        )

    def test_default_then_load_config_text(self, config_path):
        factory = TaskComposerPluginFactory()
        factory.loadConfig(config_path.read_text(), GeneralResourceLocator())
        assert PIPELINE in factory.getTaskComposerNodePlugins()

    def test_plugin_info_constructor_and_load_config(self):
        info = TaskComposerPluginInfo()
        info.search_libraries = ["my_task_composer_factories"]
        factory = TaskComposerPluginFactory(info)
        assert "my_task_composer_factories" in factory.getSearchLibraries()

        loaded = TaskComposerPluginFactory()
        loaded.loadConfig(info)
        assert loaded.getSearchLibraries() == factory.getSearchLibraries()


class TestSearchPathsAndLibraries:
    """Search paths and libraries (#321)."""

    def test_search_paths(self):
        factory = TaskComposerPluginFactory()
        factory.addSearchPath("/some/path")
        assert "/some/path" in factory.getSearchPaths()
        factory.clearSearchPaths()
        assert factory.getSearchPaths() == []

    def test_search_libraries(self):
        factory = TaskComposerPluginFactory()
        factory.addSearchLibrary("some_library")
        assert "some_library" in factory.getSearchLibraries()
        factory.clearSearchLibraries()
        assert factory.getSearchLibraries() == []


class TestPluginRegistry:
    """Add, get, remove and default plugins; create from an explicit PluginInfo (#321)."""

    def test_node_plugin_add_create_remove(self, factory):
        info = factory.getTaskComposerNodePlugins()["TrajOptTask"]
        factory.addTaskComposerNodePlugin("X", info)
        assert "X" in factory.getTaskComposerNodePlugins()
        node = factory.createTaskComposerNode("X")
        assert node.getName() == "X"
        factory.setDefaultTaskComposerNodePlugin("X")
        assert factory.getDefaultTaskComposerNodePlugin() == "X"
        factory.removeTaskComposerNodePlugin("X")
        assert "X" not in factory.getTaskComposerNodePlugins()

    def test_executor_plugin_add_get_remove(self, factory):
        plugins = factory.getTaskComposerExecutorPlugins()
        assert EXECUTOR in plugins
        factory.addTaskComposerExecutorPlugin("Y", plugins[EXECUTOR])
        assert "Y" in factory.getTaskComposerExecutorPlugins()
        factory.setDefaultTaskComposerExecutorPlugin("Y")
        assert factory.getDefaultTaskComposerExecutorPlugin() == "Y"
        factory.removeTaskComposerExecutorPlugin("Y")
        assert "Y" not in factory.getTaskComposerExecutorPlugins()

    @pytest.mark.parametrize(
        "method",
        [
            "removeTaskComposerNodePlugin",
            "setDefaultTaskComposerNodePlugin",
            "removeTaskComposerExecutorPlugin",
            "setDefaultTaskComposerExecutorPlugin",
        ],
    )
    def test_unknown_name_is_key_error(self, factory, method):
        with pytest.raises(KeyError, match="NoSuchPlugin"):
            getattr(factory, method)("NoSuchPlugin")

    def test_create_node_from_plugin_info_outlives_factory(self, config_path):
        factory = TaskComposerPluginFactory(config_path, GeneralResourceLocator())
        info = factory.getTaskComposerNodePlugins()["TrajOptTask"]
        node = factory.createTaskComposerNode("Unconfigured", info)
        del factory
        gc.collect()
        assert node.getName() == "Unconfigured"

    def test_create_executor_from_plugin_info(self, factory):
        info = factory.getTaskComposerExecutorPlugins()[EXECUTOR]
        executor = factory.createTaskComposerExecutor("Unconfigured", info)
        assert executor.getName() == "Unconfigured"

    def test_create_node_from_bad_plugin_info_raises(self, factory):
        from tesseract_robotics.tesseract_task_composer import TaskComposerPluginError

        info = PluginInfo()
        info.class_name = "NoSuchFactory"
        with pytest.raises(TaskComposerPluginError, match="Unconfigured"):
            factory.createTaskComposerNode("Unconfigured", info)

    def test_available_executor_plugins(self, factory):
        assert "TaskflowTaskComposerExecutorFactory" in (
            factory.getAvailableTaskComposerExecutorPlugins()
        )


class TestConfig:
    """`getConfig` and `saveConfig` (#321)."""

    def test_get_config_round_trips_through_text_constructor(self, factory):
        text = factory.getConfig()
        assert "task_composer_plugins" in yaml.safe_load(text)
        rebuilt = TaskComposerPluginFactory(text, GeneralResourceLocator())
        assert set(rebuilt.getTaskComposerNodePlugins()) == set(
            factory.getTaskComposerNodePlugins()
        )

    def test_save_config(self, factory, tmp_path):
        out = tmp_path / "config.yaml"
        factory.saveConfig(out)
        assert yaml.safe_load(out.read_text()) == yaml.safe_load(factory.getConfig())

    def test_save_config_to_missing_directory_raises(self, factory, tmp_path):
        with pytest.raises(FileNotFoundError):
            factory.saveConfig(tmp_path / "missing" / "config.yaml")


class TestServer:
    """Upstream's top-level entry point (#323)."""

    @pytest.fixture(scope="class")
    def server(self, config_path):
        server = TaskComposerServer()
        server.loadConfig(config_path, GeneralResourceLocator())
        return server

    def test_loads_tasks_and_executors(self, server):
        assert PIPELINE in server.getAvailableTasks()
        assert server.hasTask(PIPELINE)
        assert server.getAvailableExecutors()
        assert server.hasExecutor(EXECUTOR)
        assert server.getExecutor(EXECUTOR).getName() == EXECUTOR
        assert server.getWorkerCount(EXECUTOR) > 0
        assert server.getTaskCount(EXECUTOR) >= 0

    def test_load_config_text(self, config_path):
        server = TaskComposerServer()
        server.loadConfig(config_path.read_text(), GeneralResourceLocator())
        assert PIPELINE in server.getAvailableTasks()

    @pytest.mark.parametrize("by", ["task_name", "node"])
    def test_run_on_unknown_executor_raises(self, server, factory, make_storage, by):
        """Upstream's error, unchanged: the bindings resolve the executor before running."""
        node = factory.createTaskComposerNode(PIPELINE)
        target = PIPELINE if by == "task_name" else node
        with pytest.raises(
            RuntimeError, match="^Executor with name 'NoSuchExecutor' does not exist!$"
        ):
            server.run(target, make_storage(node), False, "NoSuchExecutor")

    def test_run_unknown_task_raises(self, server, factory, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        with pytest.raises(RuntimeError, match="^Task with name 'NoSuchTask' does not exist!$"):
            server.run("NoSuchTask", make_storage(node), False, EXECUTOR)

    def test_run_unknown_task_and_executor_names_the_executor(self, server, factory, make_storage):
        """Upstream checks the executor first."""
        node = factory.createTaskComposerNode(PIPELINE)
        with pytest.raises(RuntimeError, match="Executor with name 'NoSuchExecutor'"):
            server.run("NoSuchTask", make_storage(node), False, "NoSuchExecutor")

    def test_run_by_name_matches_planning_task_composer(self, server, make_storage, config_path):
        storage = make_storage(server.getTask(PIPELINE))
        future = server.run(PIPELINE, storage, False, EXECUTOR)
        future.wait()
        assert future.context.isSuccessful()

        composer = TaskComposer.from_config(config_path)
        node = composer.factory.createTaskComposerNode(PIPELINE)
        reference = composer.executor.run(node, make_storage(node))
        reference.wait()
        assert reference.context.isSuccessful() == future.context.isSuccessful()

    def test_run_node(self, server, factory, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        future = server.run(node, make_storage(node), False, EXECUTOR)
        future.wait()
        assert future.context.isSuccessful()

    def test_add_task(self, server, factory):
        node = factory.createTaskComposerNode("TrajOptTask")
        node.setName("AddedTask")
        server.addTask(node)
        assert server.hasTask("AddedTask")

    def test_add_executor(self, server, factory):
        executor = factory.createTaskComposerExecutor(EXECUTOR)
        executor_name = executor.getName()
        server.addExecutor(executor)
        assert server.hasExecutor(executor_name)

    def test_task_outlives_server(self, config_path):
        server = TaskComposerServer()
        server.loadConfig(config_path, GeneralResourceLocator())
        task = server.getTask(PIPELINE)
        del server
        gc.collect()
        assert task.getName() == PIPELINE


def test_planning_task_composer_passes_a_path():
    """`planning.TaskComposer.from_config` builds its factory from a `Path` (#337): a `str`
    would now be parsed as YAML text."""
    composer = TaskComposer.from_config()
    assert PIPELINE in composer.get_available_pipelines()
