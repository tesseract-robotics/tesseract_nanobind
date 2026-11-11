"""Error model and construction contracts of the planning API: named errors, overloads, profiles."""

import ast
import re
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import ProfileDictionary
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.environment import Environment
from tesseract_robotics.planning import (
    Pose,
    Robot,
    RobotState,
    TaskComposer,
    create_cartesian_pipeline_profiles,
    create_freespace_pipeline_profiles,
    create_ompl_default_profiles,
    create_trajopt_default_profiles,
    create_trajopt_ifopt_default_profiles,
    rotation_from_axis_angle,
    rotation_from_quaternion,
    translation,
)
from tesseract_robotics.planning.errors import (
    AmbiguousMeshError,
    ConflictingExecutorError,
    DegenerateRotationError,
    InvalidArrayShapeError,
    InvalidEnvironmentError,
    InvalidProfileArgumentError,
    InvalidThreadCountError,
    JointCountMismatchError,
    NoDefaultProfilesError,
    ResourceNotFoundError,
    UninitializedEnvironmentError,
)
from tesseract_robotics.planning.profiles import (
    OMPL_DEFAULT_NAMESPACE,
    STANDARD_PROFILE_NAMES,
    TRAJOPT_DEFAULT_NAMESPACE,
    TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
    default_profiles_for,
)
from tesseract_robotics.planning.profiles.pipelines import DEFAULT_PROFILE_FACTORIES

# Agreement of quantities computed from exact inputs: a few ulps of double arithmetic.
EXACT_DECIMAL = 12


@pytest.fixture(scope="module")
def locator():
    return GeneralResourceLocator(paths=[get_tesseract_resource_path()])


class TestPoseOverloads:
    """Every vector factory takes the array form and the scalar form."""

    def test_from_xyz_both_forms(self):
        np.testing.assert_array_equal(Pose.from_xyz([1, 2, 3]).translation, [1, 2, 3])
        np.testing.assert_array_equal(Pose.from_xyz(1, 2, 3).translation, [1, 2, 3])
        np.testing.assert_array_equal(translation([1, 2, 3]).translation, [1, 2, 3])
        np.testing.assert_array_equal(translation(1, 2, 3).translation, [1, 2, 3])

    def test_from_xyz_validates_shape(self):
        with pytest.raises(InvalidArrayShapeError, match="Position"):
            Pose.from_xyz([1, 2])
        with pytest.raises(TypeError, match="from_xyz"):
            Pose.from_xyz(1, 2)  # pyright: ignore[reportCallIssue]

    def test_quaternion_both_forms_agree(self):
        scalars = rotation_from_quaternion(0.0, 0.0, 1.0, 1.0)
        array = rotation_from_quaternion([0.0, 0.0, 1.0, 1.0])
        np.testing.assert_array_almost_equal(scalars.linear, array.linear, decimal=EXACT_DECIMAL)

    def test_axis_angle_both_forms_agree(self):
        scalars = rotation_from_axis_angle(0, 0, 2, 0.5)
        array = rotation_from_axis_angle([0, 0, 2], 0.5)
        np.testing.assert_array_almost_equal(scalars.linear, array.linear, decimal=EXACT_DECIMAL)

    def test_degenerate_rotations_raise_named_errors(self):
        with pytest.raises(DegenerateRotationError):
            rotation_from_axis_angle([0, 0, 0], 1.0)
        with pytest.raises(DegenerateRotationError):
            rotation_from_quaternion([0, 0, 0, 0])
        with pytest.raises(InvalidArrayShapeError):
            rotation_from_quaternion([0, 0, 1])

    def test_named_errors_are_value_errors(self):
        assert issubclass(DegenerateRotationError, ValueError)
        assert issubclass(InvalidArrayShapeError, ValueError)


class TestRobotConstruction:
    def test_build_rejects_a_non_environment(self):
        with pytest.raises(InvalidEnvironmentError):
            Robot.build("not an environment")  # pyright: ignore[reportArgumentType]

    def test_build_rejects_an_uninitialised_environment(self):
        with pytest.raises(UninitializedEnvironmentError):
            Robot.build(Environment())

    def test_build_accepts_an_initialised_environment(self, locator):
        robot = Robot.from_tesseract_support("abb_irb2400", locator)
        assert Robot.build(robot.env, locator).get_link_names() == robot.get_link_names()

    def test_missing_files_raise_resource_not_found(self, tmp_path, locator):
        with pytest.raises(ResourceNotFoundError):
            Robot.from_files(tmp_path / "no.urdf", tmp_path / "no.srdf")
        with pytest.raises(ResourceNotFoundError):
            Robot.from_urdf("package://tesseract/support/urdf/no.urdf", "package://x/y", locator)

    def test_robot_state_checks_lengths(self):
        with pytest.raises(JointCountMismatchError):
            RobotState(["a", "b"], [0.0])

    def test_set_joints_checks_lengths(self, locator):
        robot = Robot.from_tesseract_support("abb_irb2400", locator)
        with pytest.raises(JointCountMismatchError):
            robot.set_joints([0.0], joint_names=["joint_1", "joint_2"])


class TestGeometryLoading:
    MESH = "package://tesseract/support/meshes/car_seat/visual/end_effector_open.stl"

    def test_mesh_from_a_local_path(self, locator):
        from tesseract_robotics.planning import mesh_from_file

        path = Path(locator.locateResource(self.MESH).getFilePath())
        assert mesh_from_file(path, locator=locator) is not None
        assert mesh_from_file(path.as_uri(), locator=locator) is not None

    def test_a_local_mesh_does_not_go_through_the_locator(self, locator):
        """A local path or file:// URL becomes a resource from its filesystem path.

        The locator is not asked: tesseract's GeneralResourceLocator turns an RFC 8089 Windows URI
        (`file:///C:/...`) into `/C:/...` and cannot resolve it, so routing local files through it
        breaks every local mesh on Windows.
        """
        from tesseract_robotics.common import ResourceLocator
        from tesseract_robotics.planning import convex_mesh_from_file, mesh_from_file

        class ResolvesNothing(ResourceLocator):
            def locateResource(self, url):  # noqa: N802 - the C++ virtual's name
                return None

        path = Path(locator.locateResource(self.MESH).getFilePath())
        nothing = ResolvesNothing()
        assert mesh_from_file(path, locator=nothing) is not None
        assert mesh_from_file(path.as_uri(), locator=nothing) is not None
        assert convex_mesh_from_file(path, locator=nothing) is not None

    def test_mesh_from_a_relative_path(self, locator, monkeypatch):
        from tesseract_robotics.planning import convex_mesh_from_file, mesh_from_file

        path = Path(locator.locateResource(self.MESH).getFilePath())
        monkeypatch.chdir(path.parent)
        assert mesh_from_file(path.name, locator=locator) is not None
        assert convex_mesh_from_file(path.name, locator=locator) is not None

    def test_missing_mesh_raises(self, tmp_path, locator):
        from tesseract_robotics.planning import convex_mesh_from_file, mesh_from_file

        with pytest.raises(ResourceNotFoundError):
            mesh_from_file(tmp_path / "no.stl", locator=locator)
        with pytest.raises(ResourceNotFoundError):
            convex_mesh_from_file("package://tesseract/support/meshes/no.stl", locator=locator)

    def test_bad_scale_raises(self, locator):
        from tesseract_robotics.planning import mesh_from_file

        with pytest.raises(InvalidArrayShapeError, match="Scale"):
            mesh_from_file(self.MESH, scale=[1.0, 2.0], locator=locator)

    def test_several_meshes_are_not_truncated(self, monkeypatch, locator):
        """A file that yields several meshes returns all of them, or raises: never meshes[0]."""
        from tesseract_robotics.planning import geometry

        class Located:
            def getFilePath(self):
                return "x.stl"

        monkeypatch.setattr(geometry, "_locate", lambda filepath, locator: Located())
        monkeypatch.setattr(geometry, "createMeshFromResource", lambda *a: ["one", "two"])
        assert geometry.meshes_from_file("x.stl", locator=locator) == ["one", "two"]
        with pytest.raises(AmbiguousMeshError):
            geometry.mesh_from_file("x.stl", locator=locator)


class TestProfileHelpers:
    def test_each_name_has_its_own_profile_instances(self):
        """Editing the profile of one name leaves the others alone (no shared instance)."""
        from tesseract_robotics.motion_planners_trajopt import TrajOptDefaultCompositeProfile

        profiles = create_trajopt_default_profiles(["A", "B"])
        key = TrajOptDefaultCompositeProfile.createKey()
        a = profiles.getProfile(key, TRAJOPT_DEFAULT_NAMESPACE, "A")
        b = profiles.getProfile(key, TRAJOPT_DEFAULT_NAMESPACE, "B")
        a.collision_cost_config.contact_manager_config.default_margin = 0.123
        assert b.collision_cost_config.contact_manager_config.default_margin != 0.123

    def test_ifopt_profiles_are_not_shared_between_names(self):
        from tesseract_robotics.motion_planners_trajopt_ifopt import (
            TrajOptIfoptDefaultCompositeProfile,
        )

        profiles = create_trajopt_ifopt_default_profiles(["A", "B"])
        key = TrajOptIfoptDefaultCompositeProfile.createKey()
        a = profiles.getProfile(key, TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "A")
        b = profiles.getProfile(key, TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "B")
        a.smooth_velocities = True
        assert b.smooth_velocities is False

    def test_pipeline_helpers_register_every_requested_name(self):
        for build in (
            create_freespace_pipeline_profiles,
            create_cartesian_pipeline_profiles,
            create_trajopt_default_profiles,
        ):
            entries = build().getAllProfileEntries()[TRAJOPT_DEFAULT_NAMESPACE]
            for by_name in entries.values():
                assert set(by_name) == set(STANDARD_PROFILE_NAMES)

    def test_ompl_arguments_are_validated(self):
        for bad in (
            {"planning_time": 0.0},
            {"max_solutions": 0},
            {"num_planners": 0},
            {"planner_range": -1.0},
        ):
            with pytest.raises(InvalidProfileArgumentError):
                create_ompl_default_profiles(**bad)

    def test_ompl_planner_count_and_range(self):
        from tesseract_robotics.motion_planners_ompl import OMPLRealVectorMoveProfile

        profiles = create_ompl_default_profiles(["X"], num_planners=3, planner_range=0.25)
        profile = profiles.getProfile(
            OMPLRealVectorMoveProfile.createKey(), OMPL_DEFAULT_NAMESPACE, "X"
        )
        planners = profile.solver_config.planners
        assert len(planners) == 3
        assert all(p.range == pytest.approx(0.25) for p in planners)

    def test_default_profiles_exist_for_every_shipped_pipeline(self):
        composer = TaskComposer.from_config()
        for name in composer.get_available_pipelines():
            assert isinstance(default_profiles_for(name), ProfileDictionary), name

    def test_default_profile_table_names_only_shipped_pipelines(self):
        composer = TaskComposer.from_config()
        shipped = set(composer.factory.getTaskComposerNodePlugins())
        assert set(DEFAULT_PROFILE_FACTORIES) <= shipped

    def test_unknown_pipeline_has_no_default_profiles(self):
        with pytest.raises(NoDefaultProfilesError, match="MyPipeline"):
            default_profiles_for("MyPipeline")

    def test_raster_pipelines_get_profiles_of_every_planner(self):
        entries = default_profiles_for("RasterCtPipeline").getAllProfileEntries()
        assert {OMPL_DEFAULT_NAMESPACE, TRAJOPT_DEFAULT_NAMESPACE} <= set(entries)


class TestComposerConstruction:
    def test_threads_and_executor_conflict(self):
        composer = TaskComposer.from_config()
        with pytest.raises(ConflictingExecutorError):
            TaskComposer(composer.factory, num_threads=2, executor=composer.executor)

    def test_thread_count_must_be_positive(self):
        composer = TaskComposer.from_config()
        with pytest.raises(InvalidThreadCountError):
            TaskComposer(composer.factory, num_threads=0)

    def test_executor_is_created_once(self):
        composer = TaskComposer.from_config(num_threads=2)
        assert composer.executor is composer.executor

    def test_empty_result_array_is_two_dimensional(self):
        from tesseract_robotics.planning import PlanningResult

        assert PlanningResult(successful=False).to_numpy().shape == (0, 0)


def _python_blocks(docstring: str) -> list[str]:
    return re.findall(r"```python\n(.*?)```", docstring, flags=re.DOTALL)


# Modules whose docstring examples are complete programs: they must run as written.
EXAMPLE_MODULES = (
    "tesseract_robotics.planning",
    "tesseract_robotics.planning.core",
    "tesseract_robotics.planning.geometry",
    "tesseract_robotics.planning.planner",
    "tesseract_robotics.planning.program",
    "tesseract_robotics.planning.toolpaths",
    "tesseract_robotics.planning.transforms",
)


@pytest.mark.parametrize("module_name", EXAMPLE_MODULES)
def test_module_docstring_examples_run(module_name):
    import importlib
    import textwrap

    docstring = importlib.import_module(module_name).__doc__
    blocks = _python_blocks(textwrap.dedent(docstring or ""))
    assert blocks, f"{module_name} documents no fenced python example"
    for block in blocks:
        namespace: dict = {}
        exec(compile(textwrap.dedent(block), f"<{module_name} docstring>", "exec"), namespace)


def test_docstring_examples_are_valid_python():
    """Every fenced python block in the planning package's docstrings parses."""
    import inspect
    import textwrap

    import tesseract_robotics.planning as planning_package

    root = Path(planning_package.__file__).parent
    for path in sorted(root.rglob("*.py")):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef | ast.ClassDef | ast.Module):
                doc = ast.get_docstring(node)
                for block in _python_blocks(inspect.cleandoc(doc or "")):
                    try:
                        ast.parse(textwrap.dedent(block))
                    except SyntaxError as e:  # pragma: no cover - names the offending block
                        pytest.fail(f"{path.name}: docstring example does not parse: {e}")
