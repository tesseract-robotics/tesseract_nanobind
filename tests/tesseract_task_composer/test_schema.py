"""Task-composer config schemas (#373): `schema()` on the node and executor types,
`TaskComposerNodePorts.inputSchema` / `outputSchema`, and `ValidationError.toString`.

A schema is a `tesseract_common.PropertyTree`; Python passes and reads its YAML as text.
"""

import pytest

from tesseract_robotics.tesseract_common import PropertyTree
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerGraph,
    TaskComposerNodePorts,
    TaskComposerPipeline,
    TaskComposerPortMap,
    TaskComposerTask,
    TaskflowTaskComposerExecutor,
)

Cardinality = TaskComposerNodePorts.Cardinality


def _ports() -> TaskComposerNodePorts:
    ports = TaskComposerNodePorts()
    ports.addRequiredInput("program")
    ports.addOptionalInput("environments", Cardinality.MULTIPLE)
    ports.addRequiredOutput("program")
    return ports


@pytest.mark.parametrize(
    "schema",
    [TaskComposerGraph.schema, TaskComposerPipeline.schema, TaskflowTaskComposerExecutor.schema],
    ids=["graph", "pipeline", "executor"],
)
def test_static_schemas_are_containers(schema):
    tree = schema()
    assert isinstance(tree, PropertyTree)
    assert tree.isContainer()
    assert not tree.empty()


def test_schema_names_config_keys():
    """A graph (the `TrajOptPipeline` node is one) declares its `inputs` / `outputs` and its
    node, edge and terminal lists."""
    keys = set(TaskComposerGraph.schema().keys())
    assert {"inputs", "outputs", "nodes", "edges", "terminals", "conditional"} <= keys
    assert "nodes" in TaskComposerGraph.schema().toYAML()


def test_task_schema_has_its_ports():
    keys = set(TaskComposerTask.schema(_ports()).keys())
    assert {"inputs", "outputs", "conditional", "trigger_abort"} <= keys
    assert set(TaskComposerTask.schema(TaskComposerNodePorts()).keys()).isdisjoint(
        {"inputs", "outputs"}
    )


def test_input_schema_lists_ports():
    ports = _ports()
    assert set(ports.inputSchema().keys()) == {"program", "environments"}
    assert set(ports.outputSchema().keys()) == {"program"}
    assert ports.inputSchema().at("program").isRequired()
    assert not ports.inputSchema().at("environments").isRequired()


def test_input_schema_of_a_factory_task(factory):
    graph = factory.createTaskComposerNode("DescartesFTask")
    ports = graph.getNodeByName("MinLengthTask").getPortContract()
    assert ports is not None and ports.inputs()
    assert set(ports.inputSchema().keys()) == set(ports.inputs())


def test_validation_error_names_bad_key():
    """Both error paths name the offending key: the port contract's ValidationError and the
    schema's applyConfig."""
    inputs = TaskComposerPortMap()
    inputs.set("program", "input_data")
    inputs.set("bogus_port", "x")
    outputs = TaskComposerPortMap()
    outputs.set("program", "output_data")
    (error,) = _ports().validate(inputs, outputs)
    assert error.port == "bogus_port"
    assert error.toString() == "inputs.bogus_port: undeclared port"
    assert str(error) == error.toString()

    schema = TaskComposerTask.schema(_ports())
    errors = schema.applyConfig("inputs: {program: in}\noutputs: {program: out}\nbogus_key: 1\n")
    assert any("bogus_key" in e for e in errors), errors
    assert not TaskComposerTask.schema(_ports()).applyConfig(
        "inputs: {program: in}\noutputs: {program: out}\nbogus_key: 1\n",
        allow_extra_properties=True,
    )


def test_apply_config_malformed_yaml_raises():
    with pytest.raises(ValueError):
        TaskComposerTask.schema(_ports()).applyConfig("inputs: [unclosed")


def test_property_tree_reads():
    schema = TaskComposerTask.schema(_ports())
    assert schema.applyConfig("inputs: {program: in}\noutputs: {program: out}\n") == []
    program = schema.at("inputs").at("program")
    assert program.getValue().strip() == "in"
    assert schema.find("no_such_key") is None
    assert schema.find("inputs") is not None
    assert program.hasAttribute("type")
    assert "type" in program.getAttributeKeys()
    assert program.getAttribute("type").strip() == "string"
    assert program.getAttribute("no_such_attribute") is None
    with pytest.raises(IndexError):
        schema.at("no_such_key")
    round_trip = PropertyTree.fromYAML(schema.toYAML(exclude_attributes=False))
    assert set(round_trip.keys()) == set(schema.keys())
    assert len(schema) == schema.size()
    assert "inputs" in str(schema)
