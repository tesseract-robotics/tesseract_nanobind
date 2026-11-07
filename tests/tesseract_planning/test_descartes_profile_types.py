"""Descartes profiles reach the nodes that read them, whatever their scalar type (#350).

Every shipped Cartesian and raster task runs `DescartesMotionPlannerF`, which looks its profiles
up under the `float` keys; only `DescartesDPipeline` / `DescartesDNPCPipeline` read the `double`
ones. A profile of the other type is never found, and the planner runs its defaults.
"""

import math

import numpy as np
import pytest
import yaml

import tesseract_robotics.tesseract_motion_planners_descartes as descartes
from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    MoveType,
    Pose,
    Robot,
    TaskComposer,
    create_descartes_default_profiles,
)
from tesseract_robotics.planning.profiles import (
    DESCARTES_DEFAULT_NAMESPACE,
    DESCARTES_PROFILE_NAMES,
)
from tesseract_robotics.tesseract_collision import ContactManagerConfig
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_common import PluginInfo
from tesseract_robotics.tesseract_motion_planners import ProfileOverrideNotDispatchedError

# m, the vertex contact margin of the infeasible profile. Every link of the IRB2400 lies within
# 3 m of every other (the arm reaches 1.55 m from its base), so at 10 m each link pair the
# allowed-collision matrix checks is in contact at every IK sample: no vertex survives.
EVERY_STATE_COLLIDES_MARGIN_M = 10.0

# The shipped pipelines that reach a Descartes node, with the scalar type that node reads:
# every Cartesian and raster task is `DescartesFMotionPlannerTaskFactory`.
DESCARTES_PIPELINES = {
    "CartesianPipeline": "F",
    "DescartesFPipeline": "F",
    "DescartesFNPCPipeline": "F",
    "DescartesDPipeline": "D",
    "DescartesDNPCPipeline": "D",
    "RasterFtPipeline": "F",
    "RasterCtPipeline": "F",
    "RasterFtOnlyPipeline": "F",
    "RasterCtOnlyPipeline": "F",
    "RasterFtGlobalPipeline": "F",
    "RasterCtGlobalPipeline": "F",
    "RasterFtOnlyGlobalPipeline": "F",
    "RasterCtOnlyGlobalPipeline": "F",
}

OTHER_SCALAR = {"F": "D", "D": "F"}


@pytest.fixture(scope="module")
def composer():
    """One composer for the module: each `from_config()` reloads the plugins (#137)."""
    return TaskComposer.from_config()


@pytest.fixture(scope="module")
def robot():
    return Robot.from_tesseract_support("abb_irb2400")


def _linear_program(robot):
    """Three tool-down linear waypoints, reachable for the IRB2400 (as in test_planning_api)."""
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(np.zeros(6), joint_names=joint_names)
    tool_down = [0.0, -1.0, 0.0, 0.0]  # scalar-last [qx, qy, qz, qw]
    program = MotionProgram(
        "manipulator", tcp_frame="tool0", working_frame="base_link"
    ).set_joint_names(joint_names)
    for y in (-0.2, 0.0, 0.2):
        pose = Pose.from_xyz_quat([0.8, y, 0.8], tool_down)
        program.move_to(CartesianTarget(pose, move_type=MoveType.LINEAR))
    return program


def _cls(name, scalar):
    """The bound Descartes class `name` + scalar suffix, looked up when the test runs."""
    return getattr(descartes, f"{name}{scalar}")


def _one_type_profiles(scalar, family):
    """A dictionary holding only `scalar`'s profile of `family` under every Descartes name."""
    concrete = {
        "move": "DescartesDefaultMoveProfile",
        "solver": "DescartesLadderGraphSolverProfile",
    }
    profiles = ProfileDictionary()
    for name in DESCARTES_PROFILE_NAMES:
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, _cls(concrete[family], scalar)())
    return profiles


def _registered(profiles, base, scalar, name):
    """The profile stored under `base`+`scalar`'s key for `name` in the Descartes namespace."""
    entries = profiles.getAllProfileEntries()[DESCARTES_DEFAULT_NAMESPACE]
    return entries[_cls(base, scalar).createKey()][name]


@pytest.mark.parametrize(
    "pipeline", ["CartesianPipeline", "DescartesFPipeline", "DescartesDPipeline"]
)
def test_infeasible_profile_fails_on_every_descartes_pipeline(robot, composer, pipeline):
    infeasible = descartes.DescartesDefaultMoveProfileD()
    infeasible.vertex_contact_manager_config = ContactManagerConfig(EVERY_STATE_COLLIDES_MARGIN_M)
    profiles = create_descartes_default_profiles(move_profile=infeasible)

    result = composer.plan(robot, _linear_program(robot), pipeline=pipeline, profiles=profiles)

    assert not result.successful, f"{pipeline} planned through a profile where every state collides"
    assert "Failed to build graph" in result.message, result.message


@pytest.mark.parametrize("family", ["move", "solver"])
@pytest.mark.parametrize("pipeline", sorted(DESCARTES_PIPELINES))
def test_mismatched_profile_type_raises(robot, composer, pipeline, family):
    from tesseract_robotics.planning import ProfileTypeMismatchError

    reads = DESCARTES_PIPELINES[pipeline]
    registered = OTHER_SCALAR[reads]
    base = {"move": "DescartesMoveProfile", "solver": "DescartesSolverProfile"}[family]
    profiles = _one_type_profiles(registered, family)

    with pytest.raises(ProfileTypeMismatchError) as raised:
        composer.plan(robot, _linear_program(robot), pipeline=pipeline, profiles=profiles)

    message = str(raised.value)
    for part in (
        f"'{pipeline}'",
        "DescartesMotionPlannerTask'",
        f"namespace '{DESCARTES_DEFAULT_NAMESPACE}'",
        f"'{base}{registered}'",
        f"reads '{base}{reads}'",
    ):
        assert part in message, f"{part!r} missing from: {message}"
    assert any(f"profile '{name}'" in message for name in DESCARTES_PROFILE_NAMES), message


@pytest.mark.parametrize("pipeline", ["CartesianPipeline", "DescartesDPipeline"])
def test_matching_profile_type_plans(robot, composer, pipeline):
    reads = DESCARTES_PIPELINES[pipeline]
    profiles = _one_type_profiles(reads, "move")
    for name in DESCARTES_PROFILE_NAMES:
        solver = _cls("DescartesLadderGraphSolverProfile", reads)()
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, solver)

    result = composer.plan(robot, _linear_program(robot), pipeline=pipeline, profiles=profiles)

    assert result.successful, f"{pipeline} failed: {result.message}"


@pytest.mark.parametrize("pipeline", sorted(DESCARTES_PIPELINES))
def test_helper_profiles_pass_the_type_check(composer, pipeline):
    """The helpers register both scalar types, so no Descartes pipeline refuses them."""
    from tesseract_robotics.planning.profile_types import check_descartes_profile_types

    node = composer.factory.createTaskComposerNode(pipeline)
    profiles = create_descartes_default_profiles()

    check_descartes_profile_types(composer.factory, pipeline, node, profiles)


# A Descartes task as a ForEach operation's sub-task, by task reference and by class. A
# ForEachTask builds its operation when it runs, so the walk builds it; no shipped config has one.
_DESCARTES_IO = {
    "inputs": {"program": "program", "environment": "environment", "profiles": "profiles"},
    "outputs": {"program": "program"},
}
FOR_EACH_OPERATIONS = {
    "by_task": (
        {"task": "DescartesDTask"},
        ("operation/DescartesMotionPlannerTask", DESCARTES_DEFAULT_NAMESPACE, "D"),
    ),
    "by_class_with_namespace": (
        {
            "class": "DescartesFMotionPlannerTaskFactory",
            "config": {"namespace": "ForEachDescartes", **_DESCARTES_IO},
        },
        ("operation", "ForEachDescartes", "F"),
    ),
    "by_class": (
        {"class": "DescartesFMotionPlannerTaskFactory", "config": _DESCARTES_IO},
        ("operation", "<named at run time>", "F"),
    ),
}


@pytest.fixture
def for_each_task(composer):
    """Adds a ForEachTask plugin with the given operation to the composer; removed afterwards."""
    added = []

    def add(operation):
        name = f"ForEachDescartesTest{len(added)}"
        info = PluginInfo()
        info.class_name = "ForEachTaskFactory"
        info.config = yaml.safe_dump(
            {
                "inputs": {"container": "programs"},
                "outputs": {"container": "programs"},
                "operation": {"input_port": "program", "output_port": "program", **operation},
            }
        )
        composer.factory.addTaskComposerNodePlugin(name, info)
        added.append(name)
        return name

    yield add
    for name in added:
        composer.factory.removeTaskComposerNodePlugin(name)


@pytest.mark.parametrize("shape", sorted(FOR_EACH_OPERATIONS))
def test_for_each_operation_is_walked(composer, for_each_task, shape):
    from tesseract_robotics.planning.profile_types import descartes_nodes

    operation, (path, namespace, scalar) = FOR_EACH_OPERATIONS[shape]
    name = for_each_task(operation)

    nodes = descartes_nodes(composer.factory, name, composer.factory.createTaskComposerNode(name))

    assert [(n.path, n.namespace, n.factory) for n in nodes] == [
        (f"{name}/{path}", namespace, f"Descartes{scalar}MotionPlannerTaskFactory")
    ]


def test_for_each_operation_mismatch_raises(composer, for_each_task):
    from tesseract_robotics.planning import ProfileTypeMismatchError
    from tesseract_robotics.planning.profile_types import check_descartes_profile_types

    name = for_each_task(FOR_EACH_OPERATIONS["by_task"][0])
    node = composer.factory.createTaskComposerNode(name)

    with pytest.raises(ProfileTypeMismatchError, match="reads 'DescartesMoveProfileD'"):
        check_descartes_profile_types(composer.factory, name, node, _one_type_profiles("F", "move"))


@pytest.mark.parametrize("family", ["move", "solver"])
def test_helpers_register_both_scalar_types(family):
    base = {"move": "DescartesMoveProfile", "solver": "DescartesSolverProfile"}[family]
    profiles = create_descartes_default_profiles(num_threads=3)

    for name in DESCARTES_PROFILE_NAMES:
        for scalar in ("F", "D"):
            assert isinstance(_registered(profiles, base, scalar, name), _cls(base, scalar))


def test_solver_twins_share_settings():
    profiles = create_descartes_default_profiles(num_threads=3)

    for scalar in ("F", "D"):
        assert _registered(profiles, "DescartesSolverProfile", scalar, "DEFAULT").num_threads == 3


def _configured_move_profile(scalar):
    """A default move profile with every field away from its default."""
    profile = _cls("DescartesDefaultMoveProfile", scalar)()
    profile.target_pose_fixed = False
    profile.target_pose_sample_axis = np.array([1.0, 0.0, 0.0])
    profile.target_pose_sample_resolution = math.radians(30)
    profile.target_pose_sample_min = -math.radians(90)
    profile.target_pose_sample_max = math.radians(60)
    profile.manipulator_ik_solver = "OPWInvKin"
    profile.allow_collision = True
    profile.enable_collision = False
    profile.vertex_contact_manager_config = ContactManagerConfig(0.05)
    profile.vertex_collision_check_config.longest_valid_segment_length = 0.2
    profile.enable_edge_collision = True
    profile.edge_contact_manager_config = ContactManagerConfig(0.01)
    profile.edge_collision_check_config.longest_valid_segment_length = 0.3
    profile.use_redundant_joint_solutions = True
    profile.debug = True
    return profile


def _fields(cls):
    return sorted(name for name, value in vars(cls).items() if isinstance(value, property))


@pytest.mark.parametrize(("source", "twin"), [("D", "F"), ("F", "D")])
def test_move_profile_override_gets_its_twin(source, twin):
    given = _configured_move_profile(source)
    profiles = create_descartes_default_profiles(move_profile=given)

    for name in DESCARTES_PROFILE_NAMES:
        assert _registered(profiles, "DescartesMoveProfile", source, name) is given
        made = _registered(profiles, "DescartesMoveProfile", twin, name)
        assert type(made) is _cls("DescartesDefaultMoveProfile", twin)
        fields = _fields(type(given))
        assert fields == _fields(type(made))
        for field in fields:
            want, got = getattr(given, field), getattr(made, field)
            if isinstance(want, (float, np.ndarray)):
                # The float twin holds the float32 rounding of a double; compare at that precision.
                np.testing.assert_array_equal(np.float32(got), np.float32(want), err_msg=field)
            else:
                assert got == want, field


@pytest.mark.parametrize("scalar", ["D", "F"])
def test_custom_move_profile_subclass_is_refused(scalar):
    class Custom(_cls("DescartesMoveProfile", scalar)):
        pass

    with pytest.raises(
        ProfileOverrideNotDispatchedError, match=f"subclass of DescartesMoveProfile{scalar}"
    ):
        create_descartes_default_profiles(move_profile=Custom())


def test_non_move_profile_override_is_refused():
    solver = descartes.DescartesLadderGraphSolverProfileD()

    with pytest.raises(ProfileOverrideNotDispatchedError, match="not a Descartes move profile"):
        create_descartes_default_profiles(move_profile=solver)
