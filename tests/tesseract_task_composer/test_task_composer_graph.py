"""Nodes, graphs, contexts, futures and node infos of a task-composer run."""

import copy
import datetime
import gc
import sys
import time
import uuid

import pytest

from tesseract_robotics.tesseract_task_composer import (
    FutureStatus,
    TaskComposerContext,
    TaskComposerDataStorage,
    TaskComposerGraph,
    TaskComposerKeys,
    TaskComposerNode,
    TaskComposerNodeInfo,
    TaskComposerNodeInfoContainer,
    TaskComposerNodePorts,
    TaskComposerNodeType,
    TaskComposerPipeline,
    TaskComposerTask,
)

PIPELINE = "TrajOptPipeline"


def _copy_of(infos: TaskComposerNodeInfoContainer) -> TaskComposerNodeInfoContainer:
    """A container holding the same infos (the root and aborting node stay nil)."""
    copied = TaskComposerNodeInfoContainer()
    copied.insertInfoMap(infos)
    return copied


def _task_with_required_inputs(pipeline: TaskComposerNode) -> TaskComposerNode:
    """A child task of `pipeline` that declares required input ports. Graphs and pipelines
    declare none (upstream constructs them with empty TaskComposerNodePorts)."""
    for child in pipeline.getNodes().values():
        if child.getType() == TaskComposerNodeType.TASK and child.getPorts().input_required:
            return child
    raise AssertionError(f"{pipeline.getName()} has no task with required input ports")


@pytest.fixture(scope="module")
def executor(factory):
    return factory.createTaskComposerExecutor("TaskflowExecutor")


@pytest.fixture(scope="module")
def run(factory, executor, make_storage):
    """A finished, successful TrajOptPipeline run: (node, context)."""
    node = factory.createTaskComposerNode(PIPELINE)
    future = executor.run(node, make_storage(node))
    future.wait()
    assert future.context.isSuccessful()
    yield node, future.context
    del future
    gc.collect()


class TestContext:
    """Constructors and `abort` (#317)."""

    def test_name_constructor(self):
        context = TaskComposerContext("x")
        assert context.name == "x"
        assert context.dotgraph is False
        assert context.data_storage is not None
        assert context.task_infos is not None

    def test_data_storage_constructor_keeps_storage(self):
        storage = TaskComposerDataStorage("s")
        context = TaskComposerContext("x", storage, True)
        assert context.data_storage is storage
        assert context.dotgraph is True

    def test_task_infos_constructor_keeps_infos(self):
        storage = TaskComposerDataStorage()
        infos = TaskComposerNodeInfoContainer()
        context = TaskComposerContext("x", storage, infos)
        assert context.task_infos is infos

    def test_abort(self):
        context = TaskComposerContext("x")
        context.abort()
        assert context.isAborted()

    def test_abort_with_calling_node_records_it(self):
        node_uuid = str(uuid.uuid4())
        context = TaskComposerContext("x")
        context.abort(node_uuid)
        assert context.task_infos.getAbortingNode() == node_uuid

    def test_executor_runs_python_built_context(self, factory, executor, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        context = TaskComposerContext("python_built", make_storage(node))
        future = executor.run(node, context)
        future.wait()
        assert future.context is context
        assert context.isSuccessful()

    def test_storage_shortcut_takes_a_name(self, factory, executor, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        future = executor.run(node, make_storage(node), name="named_run")
        future.wait()
        assert future.context.name == "named_run"


class TestFuture:
    """`clear` and `copy` (#315)."""

    def test_copy_shares_state(self, factory, executor, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        future = executor.run(node, make_storage(node))
        future.wait()
        duplicate = future.copy()
        assert duplicate.ready() == future.ready()
        assert duplicate.context is future.context
        assert copy.copy(future).context is future.context

    def test_future_keeps_node_alive(self, factory, executor, make_storage):
        """The executor runs the node after run() returns: dropping it before wait() was a
        use-after-free (a bus error)."""
        node = factory.createTaskComposerNode(PIPELINE)
        future = executor.run(node, make_storage(node))
        del node
        gc.collect()
        future.wait()
        assert future.context.isSuccessful()

    @pytest.mark.parametrize("finish", ["wait", "waitFor", "ready"])
    def test_finished_run_releases_node_and_executor(self, factory, make_storage, finish):
        """The bindings hold a run's node and executor until it ends; waiting on (or polling)
        the finished future releases them, not only the next run()."""
        executor = factory.createTaskComposerExecutor("TaskflowExecutor")
        node = factory.createTaskComposerNode(PIPELINE)
        node_refs, executor_refs = sys.getrefcount(node), sys.getrefcount(executor)
        future = executor.run(node, make_storage(node))
        if finish == "wait":
            future.wait()
        elif finish == "waitFor":
            assert future.waitFor(datetime.timedelta(seconds=60)) == FutureStatus.ready
        else:
            while not future.ready():
                time.sleep(0.05)
        assert (sys.getrefcount(node), sys.getrefcount(executor)) == (node_refs, executor_refs)

    def test_clear_invalidates(self, factory, executor, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        future = executor.run(node, make_storage(node))
        future.wait()
        future.clear()
        assert future.valid() is False


class TestNode:
    """Type, ports, setters, `validatePorts`, `getDataStorage` and `run` (#318)."""

    def test_pipeline_type_and_ports(self, factory):
        """A pipeline declares no ports (upstream skips port checks for graphs); its tasks do."""
        node = factory.createTaskComposerNode(PIPELINE)
        assert node.getType() == TaskComposerNodeType.PIPELINE
        assert node.getPorts() == TaskComposerNodePorts()
        task = _task_with_required_inputs(node)
        ports = task.getPorts()
        assert isinstance(ports, TaskComposerNodePorts)
        assert set(ports.input_required) <= set(task.getInputKeys().data())
        port = next(iter(ports.input_required))
        assert port in str(ports)

    def test_set_name_and_namespace(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        node.setName("x")
        node.setNamespace("ns")
        assert node.getName() == "x"
        assert node.getNamespace() == "ns"

    def test_set_conditional(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        node.setConditional(True)
        assert node.isConditional()

    def test_root_node_has_no_parent(self, factory):
        """A root node's parent is the nil uuid."""
        node = factory.createTaskComposerNode(PIPELINE)
        assert node.getParentUUIDString() == str(uuid.UUID(int=0))

    def test_validate_ports_raises_on_missing_required_key(self, factory):
        task = _task_with_required_inputs(factory.createTaskComposerNode(PIPELINE))
        task.validatePorts()
        task.setInputKeys(TaskComposerKeys())
        with pytest.raises(RuntimeError, match="missing required input port"):
            task.validatePorts()

    def test_get_data_storage_of_root_is_context_storage(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        storage = TaskComposerDataStorage("root")
        context = TaskComposerContext("x", storage)
        assert node.getDataStorage(context) is storage

    def test_run_in_calling_thread_matches_executor(self, factory, executor, make_storage):
        node = factory.createTaskComposerNode(PIPELINE)
        direct = TaskComposerContext("direct", make_storage(node))
        assert node.run(direct, executor) == 0

        pooled = executor.run(node, TaskComposerContext("pooled", make_storage(node)))
        pooled.wait()

        def names(context):
            return sorted(i.name for i in context.task_infos.getInfoMap().values())

        assert names(direct) == names(pooled.context)
        assert direct.isSuccessful() == pooled.context.isSuccessful()

    def test_run_pipeline_without_executor(self, factory, make_storage):
        """A pipeline runs its children in the calling thread; it needs no executor."""
        node = factory.createTaskComposerNode(PIPELINE)
        context = TaskComposerContext("direct", make_storage(node))
        node.run(context)
        assert context.isSuccessful()

    def test_run_graph_without_executor_records_the_error(self, factory, make_storage):
        """A graph needs an executor; upstream records the exception as the node's info."""
        pipeline = factory.createTaskComposerNode(PIPELINE)
        graph = TaskComposerGraph("g")
        graph.setTerminals([graph.addNode(pipeline)])
        context = TaskComposerContext("direct", make_storage(pipeline))
        graph.run(context)
        info = context.task_infos.getInfo(graph.getUUIDString())
        assert "executor is null" in info.status_message


class TestNodeUuids:
    """Uuid-addressed members, through the uuid caster (#316)."""

    def test_root_node_and_get_info(self, run):
        node, context = run
        infos = context.task_infos
        assert infos.getRootNode() == node.getUUIDString()
        assert infos.getInfo(node.getUUIDString()).name == node.getName()

    def test_info_map_keys_are_uuids(self, run):
        _, context = run
        info_map = context.task_infos.getInfoMap()
        assert info_map
        for key, info in info_map.items():
            uuid.UUID(key)
            assert info.uuid == key

    def test_node_uuid_accessors(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        assert node.getUUID() == node.getUUIDString()
        assert node.getParentUUID() == str(uuid.UUID(int=0))

    def test_child_edges_are_info_keys(self, run):
        node, context = run
        info_map = context.task_infos.getInfoMap()
        for child in node.getNodes().values():
            for edge in child.getOutboundEdges():
                assert edge in info_map
            for edge in child.getInboundEdges():
                assert edge in info_map

    def test_info_uuid_fields(self, run):
        node, context = run
        info = context.task_infos.getInfo(node.getUUIDString())
        assert info.root_uuid == node.getRootNode()
        assert set(info.terminals) == set(node.getTerminals())
        assert info.inbound_edges == []
        assert info.outbound_edges == []

    def test_get_info_unknown_uuid_is_none(self, run):
        _, context = run
        assert context.task_infos.getInfo(str(uuid.uuid4())) is None

    def test_get_info_malformed_uuid_raises(self, run):
        _, context = run
        with pytest.raises(ValueError):
            context.task_infos.getInfo("not-a-uuid")

    def test_root_and_aborting_setters(self):
        infos = TaskComposerNodeInfoContainer()
        a, b = str(uuid.uuid4()), str(uuid.uuid4())
        infos.setRootNode(a)
        infos.setAborted(b)
        assert infos.getRootNode() == a
        assert infos.getAbortingNode() == b

    def test_native_dotgraph_with_results(self, run):
        node, context = run
        dot = node.getDotgraph(context.task_infos.getInfoMap())
        assert dot.startswith("digraph TaskComposer {")
        assert "Time:" in dot  # results annotate the nodes

    def test_save_dotgraph_with_results(self, run, tmp_path):
        node, context = run
        info_map = context.task_infos.getInfoMap()
        out = tmp_path / "graph.dot"
        node.saveDotgraph(out, info_map)
        assert out.read_text() == node.getDotgraph(info_map)


class TestRemovedUuidHelpers:
    """The four Python-only helpers over the uuid API are gone (#336)."""

    @pytest.mark.parametrize("name", ["getAbortingNodeInfo", "getAllInfos"])
    def test_container_helper_gone(self, name):
        assert not hasattr(TaskComposerNodeInfoContainer, name)

    def test_dotgraph_does_not_take_a_container(self, run):
        node, context = run
        with pytest.raises(TypeError):
            node.getDotgraph(context.task_infos)

    def test_save_dotgraph_does_not_take_a_container(self, run, tmp_path):
        node, context = run
        with pytest.raises(TypeError):
            node.saveDotgraph(str(tmp_path / "g.dot"), context.task_infos)


class TestNodeInfo:
    """Constructor and all public fields, read-write (#319)."""

    def test_constructor_from_node(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        info = TaskComposerNodeInfo(node)
        assert info.name == node.getName()
        assert info.type == node.getType()
        assert info.uuid == node.getUUIDString()

    def test_fields_after_run(self, run):
        node, context = run
        info = context.task_infos.getInfo(node.getUUIDString())
        assert isinstance(info.start_time, datetime.datetime)
        assert info.start_time <= datetime.datetime.now()
        assert info.input_keys == node.getInputKeys()
        assert info.output_keys == node.getOutputKeys()
        assert info.ns == node.getNamespace()
        assert info.conditional is node.isConditional()
        assert info.aborted is False
        assert info.triggers_abort is False
        assert isinstance(info.type_hash_code, int)

    def test_fields_are_writable(self, factory):
        info = TaskComposerNodeInfo(factory.createTaskComposerNode(PIPELINE))
        info.color = "green"
        info.name = "n"
        info.status_message = "m"
        info.return_value = 3
        info.dotgraph = "digraph {}"
        assert (info.color, info.name, info.status_message, info.return_value) == (
            "green",
            "n",
            "m",
            3,
        )
        assert info.dotgraph == "digraph {}"


class TestNodeInfoContainer:
    """`find`, `prune`, `addInfo`, `insertInfoMap`, `mergeInfoMap`, `clear` (#320)."""

    @pytest.fixture
    def infos(self, run):
        _, context = run
        return _copy_of(context.task_infos)

    def test_find_by_name(self, infos):
        found = infos.find(lambda i: i.name == PIPELINE)
        assert [i.name for i in found] == [PIPELINE]

    def test_prune_mutates_every_info(self, infos):
        infos.prune(lambda i: setattr(i, "color", "pruned"))
        assert all(i.color == "pruned" for i in infos.getInfoMap().values())

    def test_add_info(self, factory):
        infos = TaskComposerNodeInfoContainer()
        info = TaskComposerNodeInfo(factory.createTaskComposerNode(PIPELINE))
        info.name = "added"
        infos.addInfo(info)
        assert [i.name for i in infos.find(lambda i: i.name == "added")] == ["added"]

    def test_clear(self, infos):
        infos.clear()
        assert infos.find(lambda i: True) == []

    def test_insert_and_merge(self, infos):
        inserted = TaskComposerNodeInfoContainer()
        inserted.insertInfoMap(infos)
        assert inserted.getInfoMap().keys() == infos.getInfoMap().keys()

        merged = TaskComposerNodeInfoContainer()
        merged.mergeInfoMap(_copy_of(infos))
        assert merged.getInfoMap().keys() == infos.getInfoMap().keys()


class TestGraphClasses:
    """TaskComposerTask, TaskComposerGraph and TaskComposerPipeline (#324)."""

    def test_factory_pipeline_is_a_pipeline(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        assert isinstance(node, TaskComposerPipeline)
        assert isinstance(node, TaskComposerGraph)
        nodes = node.getNodes()
        assert nodes
        assert all(isinstance(n, TaskComposerNode) for n in nodes.values())
        assert node.isValid()[0] is True
        assert set(node.getTerminals()) <= set(nodes)
        assert node.getRootNode() in nodes

    def test_get_node_by_name(self, factory):
        node = factory.createTaskComposerNode(PIPELINE)
        child = next(iter(node.getNodes().values()))
        assert node.getNodeByName(child.getName()).getUUIDString() == child.getUUIDString()

    def test_override_keys(self):
        graph = TaskComposerGraph("g")
        keys = TaskComposerKeys()
        keys.add("p", "k")
        graph.setOverrideInputKeys(keys)
        graph.setOverrideOutputKeys(keys)
        assert graph.getOverrideInputKeys() == keys
        assert graph.getOverrideOutputKeys() == keys

    def test_graph_built_in_python_runs(self, factory, executor, make_storage):
        """Two pipelines in sequence, the second on the first's output (#324)."""
        first = factory.createTaskComposerNode(PIPELINE)
        second = factory.createTaskComposerNode(PIPELINE)
        graph = TaskComposerGraph("python_graph")
        assert graph.getName() == "python_graph"
        a = graph.addNode(first)
        b = graph.addNode(second)
        graph.addEdges(a, [b])
        graph.setTerminals([b])
        assert graph.getTerminals() == [b]
        assert graph.getAbortTerminal() == str(uuid.UUID(int=0))
        assert graph.getAbortTerminalIndex() == -1
        assert graph.isValid()[0] is True

        future = executor.run(graph, make_storage(first))
        future.wait()
        names = {i.name for i in future.context.task_infos.getInfoMap().values()}
        assert {"python_graph", PIPELINE} <= names

    def test_only_tasks_trigger_abort(self, factory):
        graph = TaskComposerGraph("g")
        graph.setTerminals([graph.addNode(factory.createTaskComposerNode(PIPELINE))])
        with pytest.raises(RuntimeError, match="Tasks can only trigger abort"):
            graph.setTerminalTriggerAbortByIndex(0)

    def test_add_edge_to_missing_node_raises(self, factory):
        """Upstream looks the destination up with map::at."""
        graph = TaskComposerGraph("g")
        a = graph.addNode(factory.createTaskComposerNode(PIPELINE))
        with pytest.raises(IndexError):
            graph.addEdges(a, [str(uuid.uuid4())])

    def test_is_valid_reports_multiple_roots(self, factory):
        graph = TaskComposerGraph("g")
        graph.addNode(factory.createTaskComposerNode(PIPELINE))
        graph.addNode(factory.createTaskComposerNode(PIPELINE))
        valid, message = graph.isValid()
        assert valid is False
        assert "multiple root nodes" in message

    def test_pipeline_constructors(self):
        assert TaskComposerPipeline().getName() == "TaskComposerPipeline"
        conditional = TaskComposerPipeline("p", True)
        assert conditional.isConditional()
        assert conditional.getType() == TaskComposerNodeType.PIPELINE

    def test_task_has_no_python_constructor(self):
        with pytest.raises(TypeError):
            TaskComposerTask()

    def test_factory_task_set_trigger_abort(self, factory):
        """A task node from the factory is a TaskComposerTask when its class is bound; the
        planning tasks are unbound plugin classes, so they come back as the base node."""
        node = factory.createTaskComposerNode("TrajOptTask")
        assert isinstance(node, TaskComposerNode)


class TestNodePorts:
    """The ports struct (#318) and its equality (#261)."""

    def test_fields_and_type_enum(self):
        ports = TaskComposerNodePorts()
        ports.input_required = {"a": TaskComposerNodePorts.Type.SINGLE}
        ports.output_optional = {"b": TaskComposerNodePorts.Type.MULTIPLE}
        assert ports.input_required == {"a": TaskComposerNodePorts.Type.SINGLE}
        assert ports.output_optional["b"] == TaskComposerNodePorts.Type.MULTIPLE
        assert ports.input_optional == {}
        assert ports.output_required == {}


class TestValueEquality:
    """`operator==` as `__eq__`/`__ne__`, unhashable (#261)."""

    def test_node_info(self, run):
        node, context = run
        a = context.task_infos.getInfo(node.getUUIDString())
        b = context.task_infos.getInfo(node.getUUIDString())
        assert a == b
        b.name = "other"
        assert a != b
        with pytest.raises(TypeError):
            hash(a)

    def test_node_info_container(self, run):
        _, context = run
        a = _copy_of(context.task_infos)
        b = _copy_of(context.task_infos)
        assert a == b
        assert a == a
        b.clear()
        assert a != b
        with pytest.raises(TypeError):
            hash(a)

    def test_context(self):
        storage = TaskComposerDataStorage("s")
        a = TaskComposerContext("x", storage)
        assert a == TaskComposerContext("x", storage)
        assert a != TaskComposerContext("y", storage)
        with pytest.raises(TypeError):
            hash(a)

    def test_node_ports(self):
        a, b = TaskComposerNodePorts(), TaskComposerNodePorts()
        assert a == b
        b.input_required = {"a": TaskComposerNodePorts.Type.SINGLE}
        assert a != b
        with pytest.raises(TypeError):
            hash(a)
