"""Tests for tesseract_motion_planners_ompl bindings: configurators, solver config, profile, planner."""

from pathlib import Path

import numpy as np
import pytest

import tesseract_robotics.tesseract_motion_planners_ompl as ompl
from tesseract_robotics.planning.profiles import (
    UnknownConfiguratorFieldError,
    UnknownPlannerTypeError,
    create_ompl_planner_configurators,
)
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import MotionPlanner, PlannerRequest
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLMoveProfile,
    OMPLPlannerType,
    OMPLRealVectorMoveProfile,
    OMPLSolverConfig,
    ProfileDictionary_addOMPLMoveProfile,
    RRTConnectConfigurator,
    RRTstarConfigurator,
    SBLConfigurator,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"

# Every OMPLPlannerConfigurator subclass with its public fields
# (include/tesseract/motion_planners/ompl/ompl_planner_configurator.h).
CONFIGURATORS = [
    ("SBLConfigurator", OMPLPlannerType.SBL, ["range"]),
    ("ESTConfigurator", OMPLPlannerType.EST, ["range", "goal_bias"]),
    (
        "LBKPIECE1Configurator",
        OMPLPlannerType.LBKPIECE1,
        ["range", "border_fraction", "min_valid_path_fraction"],
    ),
    (
        "BKPIECE1Configurator",
        OMPLPlannerType.BKPIECE1,
        ["range", "border_fraction", "failed_expansion_score_factor", "min_valid_path_fraction"],
    ),
    (
        "KPIECE1Configurator",
        OMPLPlannerType.KPIECE1,
        [
            "range",
            "goal_bias",
            "border_fraction",
            "failed_expansion_score_factor",
            "min_valid_path_fraction",
        ],
    ),
    (
        "BiTRRTConfigurator",
        OMPLPlannerType.BiTRRT,
        [
            "range",
            "temp_change_factor",
            "cost_threshold",
            "init_temperature",
            "frontier_threshold",
            "frontier_node_ratio",
        ],
    ),
    ("RRTConfigurator", OMPLPlannerType.RRT, ["range", "goal_bias"]),
    ("RRTConnectConfigurator", OMPLPlannerType.RRTConnect, ["range"]),
    (
        "RRTstarConfigurator",
        OMPLPlannerType.RRTstar,
        ["range", "goal_bias", "delay_collision_checking"],
    ),
    (
        "TRRTConfigurator",
        OMPLPlannerType.TRRT,
        [
            "range",
            "goal_bias",
            "temp_change_factor",
            "init_temperature",
            "frontier_threshold",
            "frontier_node_ratio",
        ],
    ),
    ("PRMConfigurator", OMPLPlannerType.PRM, ["max_nearest_neighbors"]),
    ("PRMstarConfigurator", OMPLPlannerType.PRMstar, []),
    ("LazyPRMstarConfigurator", OMPLPlannerType.LazyPRMstar, []),
    (
        "SPARSConfigurator",
        OMPLPlannerType.SPARS,
        ["max_failures", "dense_delta_fraction", "sparse_delta_fraction", "stretch_factor"],
    ),
]


@pytest.fixture
def abb_irb2400_request():
    """Freespace Cartesian request on the ABB IRB2400 (same scene as test_motion_planners)."""
    locator = GeneralResourceLocator()
    urdf_path = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )
    env = Environment()
    assert env.init(urdf_path, srdf_path, locator), "Failed to initialize ABB IRB2400"
    env.setState([f"joint_{i + 1}" for i in range(6)], np.ones(6) * 0.1)

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    def cartesian(y: float) -> MoveInstruction:
        wp = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, y, 1.455)
            * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
        )
        return MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(cartesian(-0.3)))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(cartesian(0.3)))

    request = PlannerRequest()
    request.instructions = program
    request.env = env
    return request


def _with_profile(request: PlannerRequest, profile: OMPLRealVectorMoveProfile) -> PlannerRequest:
    profiles = ProfileDictionary()
    ProfileDictionary_addOMPLMoveProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", profile)
    request.profiles = profiles
    return request


class TestPlannerConfigurators:
    """#301: all 14 OMPLPlannerType values have a bound configurator."""

    def test_every_planner_type_has_a_configurator(self):
        assert {t for _, t, _ in CONFIGURATORS} == set(OMPLPlannerType.__members__.values())

    @pytest.mark.parametrize(("cls_name", "planner_type", "fields"), CONFIGURATORS)
    def test_default_construct_and_type(self, cls_name, planner_type, fields):
        cfg = getattr(ompl, cls_name)()
        assert cfg.getType() == planner_type

    @pytest.mark.parametrize(("cls_name", "planner_type", "fields"), CONFIGURATORS)
    def test_fields_round_trip(self, cls_name, planner_type, fields):
        cfg = getattr(ompl, cls_name)()
        for i, name in enumerate(fields):
            default = getattr(cfg, name)
            value = (not default) if isinstance(default, bool) else type(default)(i + 7)
            setattr(cfg, name, value)
            assert getattr(cfg, name) == value

    @pytest.mark.parametrize("cls_name", ["PRMstarConfigurator", "RRTConfigurator"])
    def test_freespace_plan(self, abb_irb2400_request, cls_name):
        profile = OMPLRealVectorMoveProfile()
        profile.solver_config.planning_time = 2.0
        profile.solver_config.planners = [getattr(ompl, cls_name)()]
        response = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE).solve(
            _with_profile(abb_irb2400_request, profile)
        )
        assert response.successful, f"{cls_name} planning failed: {response.message}"

    @pytest.mark.parametrize("name", list(OMPLPlannerType.__members__))
    def test_create_ompl_planner_configurators_accepts_every_type(self, name):
        (cfg,) = create_ompl_planner_configurators(planners=[name])
        assert cfg.getType() == OMPLPlannerType.__members__[name]

    def test_create_ompl_planner_configurators_applies_shared_kwargs(self):
        cfgs = create_ompl_planner_configurators(
            planners=["RRTConnect", "RRTstar"], num_planners=2, range=0.1, goal_bias=0.2
        )
        assert [type(c) for c in cfgs] == [RRTConnectConfigurator] * 2 + [RRTstarConfigurator] * 2
        assert all(c.range == 0.1 for c in cfgs)
        assert all(c.goal_bias == 0.2 for c in cfgs[2:])

    def test_create_ompl_planner_configurators_rejects_unknown_planner(self):
        with pytest.raises(UnknownPlannerTypeError, match="Unknown planner"):
            create_ompl_planner_configurators(planners=["NotAPlanner"])

    def test_create_ompl_planner_configurators_rejects_unused_kwarg(self):
        with pytest.raises(UnknownConfiguratorFieldError, match="goal_bias"):
            create_ompl_planner_configurators(planners=["RRTConnect"], goal_bias=0.2)

    def test_create_ompl_planner_configurators_rejects_method_name(self):
        # A method is an attribute but not a field: only properties are settable.
        with pytest.raises(UnknownConfiguratorFieldError, match="getType"):
            create_ompl_planner_configurators(planners=["RRTConnect"], getType=1)


class TestSolverConfigPlanners:
    """#302: OMPLSolverConfig.planners is a readable list, copied on access; its configurators are shared."""

    def test_default_profile_planners(self):
        planners = OMPLRealVectorMoveProfile().solver_config.planners
        assert [type(p) for p in planners] == [RRTConnectConfigurator, RRTConnectConfigurator]

    def test_get_num_planners_removed(self):
        assert not hasattr(OMPLSolverConfig(), "getNumPlanners")

    def test_add_and_clear_planners(self):
        cfg = OMPLSolverConfig()
        cfg.addPlanner(RRTConnectConfigurator())
        cfg.addPlanner(SBLConfigurator())
        assert len(cfg.planners) == 2
        cfg.clearPlanners()
        assert len(cfg.planners) == 0

    def test_assign_replaces_list(self):
        cfg = OMPLRealVectorMoveProfile().solver_config
        cfg.planners = [RRTstarConfigurator()]
        assert [type(p) for p in cfg.planners] == [RRTstarConfigurator]

    def test_append_to_copy_does_not_change_config(self):
        cfg = OMPLSolverConfig()
        cfg.planners.append(RRTConnectConfigurator())
        assert len(cfg.planners) == 0

    def test_configurators_are_shared_with_config(self):
        # The list is a copy, but its elements are the config's own configurators.
        prof = OMPLRealVectorMoveProfile()
        prof.solver_config.planners[0].range = 0.77
        assert prof.solver_config.planners[0].range == 0.77


class TestMotionPlannerBase:
    """#299: OMPLMotionPlanner is a MotionPlanner."""

    def test_isinstance(self):
        assert isinstance(OMPLMotionPlanner("x"), MotionPlanner)

    def test_inherited_methods(self):
        planner = OMPLMotionPlanner("x")
        assert planner.getName() == "x"
        planner.terminate()
        planner.clear()
        cloned = planner.clone()
        assert type(cloned) is OMPLMotionPlanner
        assert cloned.getName() == "x"

    def test_base_solve_dispatches(self, abb_irb2400_request):
        request = _with_profile(abb_irb2400_request, OMPLRealVectorMoveProfile())
        planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = MotionPlanner.solve(planner, request)
        assert response.successful, response.message


class TestProfileHooks:
    """#339: OMPLMoveProfile.createSolverConfig."""

    def test_create_solver_config_copies_solver_config(self):
        profile = OMPLRealVectorMoveProfile()
        profile.solver_config.planning_time = 3.25
        created = profile.createSolverConfig()
        assert isinstance(created, OMPLSolverConfig)
        assert created.planning_time == profile.solver_config.planning_time
        assert created == profile.solver_config


class TestValueEquality:
    """#261: __eq__/__ne__ compare by value; the types are unhashable."""

    def test_solver_config_equality(self):
        a, b = OMPLSolverConfig(), OMPLSolverConfig()
        assert a == b
        b.planning_time = 1.0
        assert a != b

    def test_solver_config_compares_planner_count_only(self):
        # Upstream operator== compares planners by count (ompl_solver_config.cpp:40).
        a, b = OMPLSolverConfig(), OMPLSolverConfig()
        a.planners = [RRTConnectConfigurator()]
        b.planners = [SBLConfigurator()]
        assert a == b

    def test_move_profile_equality(self):
        a, b = OMPLRealVectorMoveProfile(), OMPLRealVectorMoveProfile()
        assert a == b
        b.collision_check_config.longest_valid_segment_length = 0.123
        assert a != b

    @pytest.mark.parametrize("cls", [OMPLSolverConfig, OMPLRealVectorMoveProfile])
    def test_unhashable(self, cls):
        with pytest.raises(TypeError):
            hash(cls())


class TestCreateKey:
    """#273: static createKey() matches the instance key."""

    def test_create_key_matches_get_key(self):
        assert OMPLRealVectorMoveProfile.createKey() == OMPLRealVectorMoveProfile().getKey()
        assert OMPLMoveProfile.createKey() == OMPLRealVectorMoveProfile().getKey()


class TestFromYaml:
    """#331: OMPLRealVectorMoveProfile.from_yaml reads the C++ YAML config."""

    def test_one_field(self):
        profile = OMPLRealVectorMoveProfile.from_yaml("solver_config:\n  planning_time: 3.5\n")
        # A YAML key replaces the whole sub-struct, starting from that struct's own defaults
        # (upstream `n.as<OMPLSolverConfig>()`): no planners, unlike the profile's default.
        expected = OMPLRealVectorMoveProfile()
        expected.solver_config = OMPLSolverConfig()
        expected.solver_config.planning_time = 3.5
        assert profile == expected
        assert profile != OMPLRealVectorMoveProfile()

    def test_malformed_yaml_raises_value_error(self):
        with pytest.raises(ValueError):
            OMPLRealVectorMoveProfile.from_yaml("solver_config: [")

    def test_wrong_value_type_raises(self):
        with pytest.raises(RuntimeError, match="Failed to parse yaml config"):
            OMPLRealVectorMoveProfile.from_yaml("solver_config:\n  planning_time: abc\n")
