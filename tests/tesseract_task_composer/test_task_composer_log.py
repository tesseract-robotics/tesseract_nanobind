"""TaskComposerLog: a run recorded for replay (#325) and its equality (#261)."""

import gc

import pytest

from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_serialization import (
    task_composer_log_from_binary,
    task_composer_log_from_file,
    task_composer_log_from_xml,
    task_composer_log_to_binary,
    task_composer_log_to_file,
    task_composer_log_to_xml,
)
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_TaskComposerDataStorage,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerContext,
    TaskComposerDataStorage,
    TaskComposerLog,
)

PIPELINE = "TrajOptPipeline"


def _assert_same_storage(a: TaskComposerDataStorage, b: TaskComposerDataStorage) -> None:
    """Same keys and payload types. Value payloads compare by value; upstream compares an
    AnyPoly holding a shared_ptr by pointer, so a deserialized one never equals the original:
    nested data storages are compared recursively, other pointers by type only."""
    assert a.getName() == b.getName()
    da, db = a.getData(), b.getData()
    assert da.keys() == db.keys()
    for key, value in da.items():
        type_name = value.getTypeName()
        assert db[key].getTypeName() == type_name
        if type_name.endswith("shared_ptr<tesseract::task_composer::TaskComposerDataStorage>"):
            _assert_same_storage(
                AnyPoly_as_TaskComposerDataStorage(value),
                AnyPoly_as_TaskComposerDataStorage(db[key]),
            )
        elif "shared_ptr<" not in type_name:
            assert db[key] == value, key


def _assert_same_log(a: TaskComposerLog, b: TaskComposerLog) -> None:
    assert (a.description, a.dotgraph) == (b.description, b.dotgraph)
    _assert_same_storage(a.initial_data, b.initial_data)
    assert (a.context.name, a.context.dotgraph) == (b.context.name, b.context.dotgraph)
    assert a.context.isAborted() == b.context.isAborted()
    assert a.context.task_infos == b.context.task_infos
    _assert_same_storage(a.context.data_storage, b.context.data_storage)


@pytest.fixture(scope="module")
def run_log(factory, make_storage):
    """A log filled from a real TrajOptPipeline run, as upstream records a run for replay.

    The run uses the planners' default profiles: an archive can only hold the profile types
    whose cereal registrations this module carries, and it carries none of the planners'.
    """
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")
    node = factory.createTaskComposerNode(PIPELINE)
    storage = make_storage(node)
    storage.setData("profiles", AnyPoly_wrap_ProfileDictionary(ProfileDictionary()))
    log = TaskComposerLog("trajopt run")
    log.initial_data = storage
    future = executor.run(node, storage, True)
    future.wait()
    assert future.context.isSuccessful()
    log.context = future.context
    log.dotgraph = node.getDotgraph(future.context.task_infos.getInfoMap())
    yield log
    del future
    gc.collect()


def test_fields_round_trip():
    log = TaskComposerLog("failing plan")
    assert log.description == "failing plan"
    assert log.context is None
    storage = TaskComposerDataStorage("initial")
    context = TaskComposerContext("ctx")
    log.initial_data = storage
    log.context = context
    log.dotgraph = "digraph {}"
    assert log.initial_data == storage
    assert log.context is context
    assert log.dotgraph == "digraph {}"


def test_default_description_is_empty():
    assert TaskComposerLog().description == ""


def test_equality():
    context = TaskComposerContext("ctx")
    a, b = TaskComposerLog("d"), TaskComposerLog("d")
    a.context = context
    b.context = context
    assert a == b
    b.dotgraph = "digraph {}"
    assert a != b
    with pytest.raises(TypeError):
        hash(a)


def test_equality_without_context():
    """Upstream dereferences `context` unconditionally; a log without one compares by the
    other fields and the context's presence."""
    assert TaskComposerLog("d") == TaskComposerLog("d")
    with_context = TaskComposerLog("d")
    with_context.context = TaskComposerContext("ctx")
    assert TaskComposerLog("d") != with_context


class TestSerialization:
    """`task_composer_log_{to,from}_{xml,file,binary}` in tesseract_serialization (#325)."""

    def test_xml_round_trip(self, run_log):
        _assert_same_log(task_composer_log_from_xml(task_composer_log_to_xml(run_log)), run_log)

    def test_binary_round_trip(self, run_log):
        _assert_same_log(
            task_composer_log_from_binary(task_composer_log_to_binary(run_log)), run_log
        )

    def test_file_round_trip(self, run_log, tmp_path):
        path = tmp_path / "run.tclx"
        assert task_composer_log_to_file(run_log, path) is None
        _assert_same_log(task_composer_log_from_file(path), run_log)

    def test_file_takes_str_path(self, run_log, tmp_path):
        path = tmp_path / "run.bin"
        task_composer_log_to_file(run_log, str(path))
        assert path.read_text().lstrip().startswith("<?xml")  # XML whatever the suffix
        _assert_same_log(task_composer_log_from_file(str(path)), run_log)

    def test_file_without_suffix_is_written_as_named(self, run_log, tmp_path):
        """Upstream appends `.tclx` to a path without a suffix; this writes the path given."""
        path = tmp_path / "run"
        task_composer_log_to_file(run_log, path)
        assert sorted(p.name for p in tmp_path.iterdir()) == ["run"]
        _assert_same_log(task_composer_log_from_file(path), run_log)

    def test_to_file_in_missing_directory_raises(self, run_log, tmp_path):
        with pytest.raises(FileNotFoundError):
            task_composer_log_to_file(run_log, tmp_path / "missing" / "run.xml")

    def test_to_file_in_read_only_directory_raises(self, run_log, tmp_path):
        read_only = tmp_path / "ro"
        read_only.mkdir()
        read_only.chmod(0o500)
        try:
            with pytest.raises(PermissionError):
                task_composer_log_to_file(run_log, read_only / "run.xml")
        finally:
            read_only.chmod(0o700)

    def test_from_missing_file_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            task_composer_log_from_file(tmp_path / "missing.xml")

    def test_unregistered_profile_type_raises(self, factory, make_storage):
        """Planner profiles are not registered with this module's cereal registry."""
        log = TaskComposerLog("with trajopt profiles")
        log.initial_data = make_storage(factory.createTaskComposerNode(PIPELINE))
        with pytest.raises(RuntimeError, match="unregistered polymorphic type"):
            task_composer_log_to_xml(log)
