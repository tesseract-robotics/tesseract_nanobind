"""Tests for tesseract_task_composer bindings."""

import gc
import os
from pathlib import Path

import pytest

import tesseract_robotics
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerNodeInfoContainer,
    TaskComposerPluginFactory,
)


def _resolve_task_composer_config():
    """Resolve task composer plugin config via env var, package data, or workspace."""
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg and Path(env_cfg).is_file():
        return env_cfg

    pkg_config = tesseract_robotics.get_task_composer_config_path()
    if pkg_config.is_file():
        return str(pkg_config)

    test_dir = Path(__file__).parent.resolve()
    repo_root = test_dir.parent.parent
    ws_config = (
        repo_root
        / "ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml"
    )
    if ws_config.is_file():
        return str(ws_config)

    return None


class TestTaskComposerPluginFactory:
    """Test TaskComposerPluginFactory."""

    def test_create_factory_and_nodes(self):
        """Test factory creation and pipeline node creation."""
        from pathlib import Path

        # Try multiple fallbacks to find config
        config_file = None

        # 1. Try env var first (may be set by test runner)
        env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        if env_cfg and Path(env_cfg).is_file():
            config_file = env_cfg

        # 2. Try package bundled config (installed wheels)
        if not config_file:
            pkg_config = tesseract_robotics.get_task_composer_config_path()
            if pkg_config.is_file():
                config_file = str(pkg_config)

        # 3. Try workspace config (editable install)
        if not config_file:
            # Navigate from test file to repo root
            test_dir = Path(__file__).parent.resolve()
            repo_root = test_dir.parent.parent  # tests/tesseract_task_composer -> repo root
            ws_config = (
                repo_root
                / "ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml"
            )
            if ws_config.is_file():
                config_file = str(ws_config)

        assert config_file and Path(config_file).is_file(), (
            "No task composer config found. Tried env var, package config, and workspace"
        )
        config_path = FilesystemPath(config_file)
        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(config_path, locator)
        assert factory is not None

        # Test pipeline node creation - keep references to avoid GC issues
        task1 = factory.createTaskComposerNode("TrajOptPipeline")
        assert task1 is not None
        assert task1.getName() == "TrajOptPipeline"

        task2 = factory.createTaskComposerNode("FreespacePipeline")
        assert task2 is not None
        assert task2.getName() == "FreespacePipeline"

        # Test executor creation
        executor = factory.createTaskComposerExecutor("TaskflowExecutor")
        assert executor is not None

        # Cleanup - delete in reverse order to avoid use-after-free
        del executor
        del task2
        del task1
        del factory
        gc.collect()

    def test_all_pipelines_loadable(self):
        """Test all 36 expected pipelines are loadable via the factory."""
        from pathlib import Path

        # Explicit list of all 36 pipelines that must be loadable
        EXPECTED_PIPELINES = [
            # Core pipelines
            "CartesianPipeline",
            "CartesianTask",
            "FreespacePipeline",
            "FreespaceTask",
            "FreespaceIfoptPipeline",
            "FreespaceIfoptTask",
            "OMPLPipeline",
            "OMPLTask",
            "TrajOptPipeline",
            "TrajOptTask",
            "TrajOptIfoptPipeline",
            "TrajOptIfoptTask",
            # Descartes variants
            "DescartesDNPCPipeline",
            "DescartesDNPCTask",
            "DescartesDPipeline",
            "DescartesDTask",
            "DescartesFNPCPipeline",
            "DescartesFNPCTask",
            "DescartesFPipeline",
            "DescartesFTask",
            # Raster variants
            "RasterCtGlobalPipeline",
            "RasterCtGlobalTask",
            "RasterCtOnlyGlobalPipeline",
            "RasterCtOnlyGlobalTask",
            "RasterCtOnlyPipeline",
            "RasterCtOnlyTask",
            "RasterCtPipeline",
            "RasterCtTask",
            "RasterFtGlobalPipeline",
            "RasterFtGlobalTask",
            "RasterFtOnlyGlobalPipeline",
            "RasterFtOnlyGlobalTask",
            "RasterFtOnlyPipeline",
            "RasterFtOnlyTask",
            "RasterFtPipeline",
            "RasterFtTask",
        ]

        # Get config file (same fallback logic as test_create_factory_and_nodes)
        config_file = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        if not config_file or not Path(config_file).is_file():
            pkg_config = tesseract_robotics.get_task_composer_config_path()
            if pkg_config.is_file():
                config_file = str(pkg_config)

        if not config_file:
            test_dir = Path(__file__).parent.resolve()
            repo_root = test_dir.parent.parent  # tests/tesseract_task_composer -> repo root
            ws_config = (
                repo_root
                / "ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml"
            )
            if ws_config.is_file():
                config_file = str(ws_config)

        assert config_file and Path(config_file).is_file(), "No task composer config found"

        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(FilesystemPath(config_file), locator)

        # Try to load each pipeline
        loaded = []
        failed = []
        for name in EXPECTED_PIPELINES:
            try:
                node = factory.createTaskComposerNode(name)
                assert node is not None
                assert node.getName() == name
                loaded.append(name)
                del node
            except Exception as e:
                failed.append((name, str(e)))

        # Report failures explicitly
        assert not failed, f"Failed to load {len(failed)} pipelines: {failed}"
        assert len(loaded) == 36, f"Expected 36 pipelines, loaded {len(loaded)}"

        del factory
        gc.collect()


class TestDotgraph:
    """Test dotgraph generation on TaskComposerNode."""

    @pytest.fixture
    def pipeline_node(self):
        config_file = _resolve_task_composer_config()
        assert config_file, "No task composer config found"
        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(FilesystemPath(config_file), locator)
        node = factory.createTaskComposerNode("TrajOptPipeline")
        assert node is not None
        yield node, factory
        del node
        del factory
        gc.collect()

    def test_get_dotgraph_returns_valid_dot(self, pipeline_node):
        node, _ = pipeline_node
        dot = node.getDotgraph()
        assert isinstance(dot, str)
        assert dot.startswith("digraph TaskComposer {")
        assert dot.rstrip().endswith("}")
        assert "TrajOptPipeline" in dot

    def test_get_dotgraph_with_empty_results(self, pipeline_node):
        node, _ = pipeline_node
        results = TaskComposerNodeInfoContainer()
        dot = node.getDotgraph(results)
        assert isinstance(dot, str)
        assert dot.startswith("digraph TaskComposer {")
        assert "TrajOptPipeline" in dot

    def test_save_dotgraph_writes_file(self, pipeline_node, tmp_path):
        node, _ = pipeline_node
        out = tmp_path / "graph.dot"
        ok = node.saveDotgraph(str(out))
        assert ok is True
        assert out.is_file()
        contents = out.read_text()
        assert contents.startswith("digraph TaskComposer {")
        assert "TrajOptPipeline" in contents
        # File output should match in-memory output
        assert contents == node.getDotgraph()

    def test_save_dotgraph_with_results(self, pipeline_node, tmp_path):
        node, _ = pipeline_node
        out = tmp_path / "graph_results.dot"
        results = TaskComposerNodeInfoContainer()
        ok = node.saveDotgraph(str(out), results)
        assert ok is True
        assert out.is_file()
        assert out.read_text().startswith("digraph TaskComposer {")

    def test_save_dotgraph_invalid_path_returns_false(self, pipeline_node, tmp_path):
        node, _ = pipeline_node
        bad = tmp_path / "nonexistent_dir" / "graph.dot"
        ok = node.saveDotgraph(str(bad))
        assert ok is False
        assert not bad.exists()


class TestAnyPolyDataStorage:
    """AnyPoly_wrap/as_TaskComposerDataStorage — used to navigate sub-task storages
    that the framework nests under a parent's data_storage keyed by node UUID."""

    def test_anypoly_wrap_data_storage_not_null(self):
        from tesseract_robotics.tesseract_task_composer import (
            AnyPoly_wrap_TaskComposerDataStorage,
            createTaskComposerDataStorage,
        )

        ds = createTaskComposerDataStorage()
        any_poly = AnyPoly_wrap_TaskComposerDataStorage(ds)
        assert any_poly is not None
        assert not any_poly.isNull()

    def test_anypoly_roundtrip_data_storage(self):
        """Wrap a DataStorage in AnyPoly, unwrap, confirm we got the same instance back."""
        from tesseract_robotics.tesseract_task_composer import (
            AnyPoly_as_TaskComposerDataStorage,
            AnyPoly_wrap_TaskComposerDataStorage,
            createTaskComposerDataStorage,
        )

        ds = createTaskComposerDataStorage()
        ds.setName("sub_storage")

        recovered = AnyPoly_as_TaskComposerDataStorage(
            AnyPoly_wrap_TaskComposerDataStorage(ds)
        )
        assert recovered is not None
        assert recovered.getName() == "sub_storage"

        # Mutate via the recovered handle; original sees it -> shared_ptr semantics preserved.
        recovered.setName("mutated")
        assert ds.getName() == "mutated"

    def test_navigate_nested_sub_task_storage(self):
        """Mirror the real pattern: parent stores a sub-storage as AnyPoly under a
        UUID key; navigate parent.getData(uuid) -> as_TaskComposerDataStorage ->
        getData('input_data') to recover the mid-pipeline AnyPoly that the
        framework would have placed there (e.g. RasterSimple's densified output)."""
        from tesseract_robotics.tesseract_command_language import (
            CompositeInstruction,
        )
        from tesseract_robotics.tesseract_task_composer import (
            AnyPoly_as_CompositeInstruction,
            AnyPoly_as_TaskComposerDataStorage,
            AnyPoly_wrap_CompositeInstruction,
            AnyPoly_wrap_TaskComposerDataStorage,
            createTaskComposerDataStorage,
        )

        sub_storage = createTaskComposerDataStorage()
        program = CompositeInstruction("DENSIFIED")
        sub_storage.setData("input_data", AnyPoly_wrap_CompositeInstruction(program))

        parent = createTaskComposerDataStorage()
        fake_uuid = "deadbeef" * 4
        parent.setData(fake_uuid, AnyPoly_wrap_TaskComposerDataStorage(sub_storage))

        recovered_sub = AnyPoly_as_TaskComposerDataStorage(parent.getData(fake_uuid))
        recovered_program = AnyPoly_as_CompositeInstruction(
            recovered_sub.getData("input_data")
        )
        assert recovered_program.getProfile() == "DENSIFIED"

    def test_unwrap_null_anypoly_raises(self):
        """A null AnyPoly cannot be unwrapped — verify the binding surfaces the error
        rather than returning a silently-null shared_ptr."""
        from tesseract_robotics.tesseract_task_composer import (
            AnyPoly,
            AnyPoly_as_TaskComposerDataStorage,
        )

        with pytest.raises(Exception):
            AnyPoly_as_TaskComposerDataStorage(AnyPoly())
