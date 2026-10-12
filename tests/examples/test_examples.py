"""Tests that run the example scripts to verify API coverage.

Markers:
  - @pytest.mark.viewer: Viewer examples
  - @pytest.mark.planning: Motion planning examples
  - @pytest.mark.basic: Basic examples (collision, kinematics, scene_graph)
  - @pytest.mark.lowlevel: Low-level API examples
"""

import importlib
import sys

import numpy as np
import pytest

import tesseract_robotics.examples

# === Viewer Examples ===


@pytest.mark.viewer
def test_shapes_viewer():
    tesseract_robotics.examples.shapes_viewer()


@pytest.mark.viewer
def test_material_mesh_viewer():
    tesseract_robotics.examples.tesseract_material_mesh_viewer()


@pytest.mark.viewer
@pytest.mark.planning
def test_abb_irb2400_viewer():
    tesseract_robotics.examples.abb_irb2400_viewer()


# === High-Level API Examples ===


@pytest.mark.basic
def test_collision_example():
    tesseract_robotics.examples.tesseract_collision_example()


@pytest.mark.basic
def test_kinematics_example():
    tesseract_robotics.examples.tesseract_kinematics_example()


@pytest.mark.basic
def test_kinematics_plugins_example():
    result = tesseract_robotics.examples.kinematics_plugins_example()

    assert result["fwd_kin_plugins"] == {"manipulator": ["KDLFwdKinChain"]}
    assert result["inv_kin_plugins"] == {"manipulator": ["KDLInvKinChainLMA", "KDLInvKinChainNR"]}
    assert result["default_inv_kin_plugin"] == "KDLInvKinChainLMA"
    # upstream runFwdKinIIWATest: tool0 at q = 0 is the identity rotation at z = 1.306 m
    np.testing.assert_allclose(result["target_translation"], [0.0, 0.0, 1.306])

    # LMA -> NR switch: each solver reproduces the target under FK, by its own convergence test
    assert list(result["ik"]) == ["KDLInvKinChainLMA", "KDLInvKinChainNR"]
    for solver_name, ik in result["ik"].items():
        assert ik["solver_name"] == solver_name
        assert ik["num_solutions"] == 1
        assert ik["criterion"] < ik["bound"], solver_name

    assert result["created_by_plugin_info"] == ("KDLFwdKinChain", "KDLInvKinChainNR")

    # upstream PluginFactorAPIUnit: add "default", make it the default, remove it,
    # and the default falls back to the first remaining solver
    for kind, original in [
        ("fwd", ["KDLFwdKinChain"]),
        ("inv", ["KDLInvKinChainLMA", "KDLInvKinChainNR"]),
    ]:
        edits = result[f"{kind}_plugin_edits"]
        assert edits["after_add"] == sorted([*original, "default"])
        assert edits["default_after_set"] == "default"
        assert edits["after_remove"] == original
        assert edits["default_after_remove"] == original[0]

    assert result["config_round_trip_equal"]

    group = result["kinematic_group"]
    assert group["num_solutions"] == 1
    assert group["criterion"] < group["bound"]


@pytest.mark.basic
def test_kinematics_analysis_example():
    ex = importlib.import_module("tesseract_robotics.examples.kinematics_analysis_example")

    result = tesseract_robotics.examples.kinematics_analysis_example()

    # analytic vs numerical Jacobians, upstream runJacobianIIWATest / runKinGroupJacobianIIWATest cases
    jac = result["jacobian"]
    assert jac["fwd_kin_cases"] == 10
    assert jac["group_cases"] == 594
    assert jac["max_error"] < ex.JACOBIAN_TOL
    assert jac["max_error_relative_base"] < 2 * ex.JACOBIAN_TOL

    assert result["twist"]["residual"] <= result["twist"]["bound"]

    upstream, zero = result["singularity"]["upstream"], result["singularity"]["zero"]
    assert upstream["rank"] == 6
    assert upstream["sigma_min"] >= ex.SINGULARITY_THRESHOLD
    assert not upstream["near_singularity"]
    # joint_a1, a5, a7 collinear on the base z-axis at q = 0: three identical columns
    assert zero["identical_columns"]
    assert zero["rank"] == 5
    assert zero["sigma_min"] < ex.SINGULARITY_THRESHOLD
    assert zero["near_singularity"]

    for name, ellipsoid in result["manipulability"]["upstream"].items():
        assert (
            ellipsoid["condition"] == ellipsoid["max_eigen_value"] / ellipsoid["min_eigen_value"]
        ), name
        assert ellipsoid["measure"] == np.sqrt(ellipsoid["condition"]), name
        assert ellipsoid["volume_residual"] <= ex.VOLUME_RTOL, name
    singular = result["manipulability"]["zero"]["m"]
    assert singular["min_eigen_value"] == 0.0
    assert singular["measure"] == singular["condition"] == sys.float_info.max
    assert singular["volume"] == 0.0
    assert result["manipulability"]["repr"].startswith("Manipulability(m=ManipulabilityEllipsoid(")
    default_manip, default_ellipsoid = result["manipulability"]["default_repr"]
    assert default_manip.startswith("Manipulability(m=ManipulabilityEllipsoid(")
    assert default_ellipsoid.startswith("ManipulabilityEllipsoid(eigen_values=")

    for name, error in result["harmonize"].items():
        assert error <= ex.HARMONIZE_TOL, name

    assert result["is_valid"] == (True, False)
    assert result["check_kinematics"]

    limits = result["limits"]
    assert limits["shapes_after_resize"] == [(7, 2)] * 4
    assert limits["equal_to_group_limits"]
    assert limits["within"] == (True, False)
    assert limits["enforced_is_upper_bound"]
    assert limits["input_unchanged"]

    # upstream TesseractCommonUnit calcTransformError / applyTolerances / *_Toleranced cases
    assert all(result["transform_error"].values())
    assert result["apply_tolerances"] == [-1.0, 0.0, 0.0, 1.0]
    diff = result["error_diff"]
    assert diff["inside_band"] <= ex.ERROR_DIFF_ATOL
    assert diff["above_band_vs_raw"] <= ex.ERROR_DIFF_ATOL
    assert abs(diff["band_edge"] - 0.15) <= ex.ERROR_DIFF_ATOL
    assert diff["dynamic_inside_band"] <= ex.ERROR_DIFF_ATOL
    assert diff["dynamic_above_band_vs_raw"] <= ex.ERROR_DIFF_ATOL
    assert abs(diff["dynamic_band_edge"] - 0.15) <= ex.DYNAMIC_BAND_EDGE_ATOL


@pytest.mark.basic
def test_state_solver_example():
    result = tesseract_robotics.examples.state_solver_example()

    assert result["floating_joint_names"] == ["joint_a1"]
    assert result["scene_graph_edits_succeeded"]
    assert result["active_joint_names"] == [f"joint_a{i}" for i in range(2, 8)]

    # upstream runSetFloatingJointStateTest: x = 1.25, then y = 1.5, then z = 1.5
    steps = result["floating_steps"]
    expected = [[1.25, 0.0, 0.0], [1.25, 1.5, 0.0], [1.25, 1.5, 1.5]]
    assert [step["origin"] for step in steps] == expected
    for step in steps:
        assert step["agrees_with_kdl"], step["origin"]
        assert step["link_1_at_origin"], step["origin"]

    # every native overload, each checked against a KDL solver on the same scene graph
    assert result["state_overloads"] == {
        "setState(values, floating)": True,
        "setState(dict, floating)": True,
        "setState(names, values, floating)": True,
        "setState(floating)": True,
        "getState(values, floating)": True,
        "getState(dict, floating)": True,
        "getState(names, values, floating)": True,
        "getState(floating)": True,
        "getLinkTransforms()": True,
        "getLinkTransforms(names, values, floating)": True,
    }
    assert result["jacobian"] == {"overloads_identical": True, "agrees_with_kdl": True}

    insert = result["insert_scene_graph"]
    assert insert["inserted"] == {
        "attach_subgraph_joint": True,
        "prefix_attach_subgraph_joint": True,
    }
    assert insert["prefixed_links"] == ["prefix_subgraph_base_link", "prefix_subgraph_link_1"]
    assert insert["floating_joint_names"] == ["joint_a1", "prefix_attach_subgraph_joint"]
    assert insert["agrees_with_kdl"]
    assert insert["root_is_parent_times_origin"]
    assert insert["child_is_root_times_fixed_origin"]
    assert insert["floating_value_moves_root"]


@pytest.mark.basic
def test_contact_managers_example():
    ex = importlib.import_module("tesseract_robotics.examples.contact_managers_example")

    result = tesseract_robotics.examples.contact_managers_example()

    # contact_manager_plugins.yaml
    assert result["discrete_plugins"] == [
        "BulletDiscreteBVHManager",
        "BulletDiscreteSimpleManager",
        "FCLDiscreteBVHManager",
    ]
    assert result["continuous_plugins"] == ["BulletCastBVHManager", "BulletCastSimpleManager"]
    assert result["defaults"] == ("BulletDiscreteBVHManager", "BulletCastBVHManager")
    assert result["created_by_plugin_info"] == (
        "BulletDiscreteSimpleManager",
        "BulletCastSimpleManager",
    )

    # box_box_example on every discrete backend: the pair set agrees, depths need not
    assert result["hull_vertex_count"] == 8
    for name, box_box in result["box_box"].items():
        assert box_box["manager_name"] == name
        assert box_box["penetrating_pairs"] == [("box_link", "second_box_link")], name
        assert box_box["penetration_distance"] < 0.0, name
        # x = 1.6: the gap is 0.1 m plus one ulp, so margin 0.1 reports nothing
        assert box_box["gap_pairs_at_margin_0_1"] == [], name
        assert ex.BOX_GAP > ex.MARGIN
        assert box_box["gap_distance_at_margin_0_25"] > ex.MARGIN, name
        assert abs(box_box["gap_distance_at_margin_0_25"] - ex.BOX_GAP) <= ex.BOX_BOX_TOL, name

    # CollisionMarginData / CollisionMarginPairData arithmetic -> predicted contact at the gap pose
    margins = result["margins"]
    assert margins["unrelated_pair_1_7"] == {"max": 1.7, "max_box_link": 0.1, "contact": False}
    assert margins["box_pair_0_25"] == {"max": 0.25, "max_box_link": 0.25, "contact": True}
    assert margins["pair_only"] == {"max": 0.25, "max_box_link": 0.25, "contact": True}
    for case in ["incremented", "scaled", "modify_applied"]:
        assert margins[case]["contact"], case
        assert abs(margins[case]["box_pair_margin"] - 0.25) <= ex.MARGIN_ARITHMETIC_TOL, case
    assert margins["modify_applied"]["max"] == 1.7
    assert margins["replace_applied"] == {"max": 0.25, "max_box_link": 0.25, "contact": True}
    pair = margins["pair_data"]
    for case in ["incremented", "scaled"]:
        assert abs(pair[case] - 0.25) <= ex.MARGIN_ARITHMETIC_TOL, case
    assert pair["max_box_link"] == 0.1
    assert pair["max_unrelated_link"] is None
    assert pair["modify_applied_pairs"] == [
        ("also_not_box_link", "not_box_link"),
        ("box_link", "second_box_link"),
    ]
    assert pair["replace_applied_pairs"] == [("also_not_box_link", "not_box_link")]

    # upstream PluginFactorAPIUnit on the YAML factory
    edits = result["plugin_edits"]
    for kind in ["discrete", "continuous"]:
        assert edits[kind]["default_after_set"] == f"Test{kind.capitalize()}Manager", kind
        # a manager created by plugin name carries that name
        assert edits[kind]["created_name"] == edits[kind]["default_after_set"], kind
        assert edits[kind]["after_remove"] == result[f"{kind}_plugins"], kind
        # the removed default falls back to the first remaining plugin by name
        assert edits[kind]["default_after_remove"] == result[f"{kind}_plugins"][0], kind
    assert result["config_round_trip_equal"]
    assert result["cleared_search"] == {"paths": [], "libraries": []}

    # upstream TestGetAllowedCollisions
    acm = result["acm"]
    assert acm["from_dict_equal"]
    assert sorted(acm["allowed_deduplicated"]) == sorted(
        ["link1", "link2", "link3", "link4", "cause_duplicate"]
    )
    assert len(acm["allowed_with_duplicates"]) == 6
    assert acm["ordered_pair"] == ("link3", "link4")
    assert acm["after_remove_link1"] == [
        ("dummy", "dummy"),
        ("link10", "link11"),
        ("link3", "link4"),
    ]
    # one "link=<a> link=<b> reason=<r>" line per remaining entry
    acm_lines = acm["str_after_remove"].splitlines()
    assert len(acm_lines) == 3
    assert "link=link10 link=link11 reason=test" in acm_lines
    assert not any(line.startswith("link=link1 ") for line in acm_lines)

    # Python ContactResultValidator and ContactResultMap
    validator = result["validator"]
    assert validator["direct_call"] == {"dropped_pair": False, "kept_pair": True}
    assert validator["pairs_without"] == [
        ("box_link", "second_box_link"),
        ("box_link", "thin_box_link"),
        ("second_box_link", "thin_box_link"),
    ]
    assert validator["pairs_with"] == [
        ("box_link", "second_box_link"),
        ("second_box_link", "thin_box_link"),
    ]
    assert validator["seen"] == validator["pairs_without"]
    result_map = result["result_map"]
    assert result_map["iterated_keys"] == validator["pairs_without"]
    assert result_map["at_matches_container"]
    assert result_map["after_filter"] == [("box_link", "thin_box_link")]
    assert result_map["counts"] == {
        "start": 3,
        "add_result": 4,
        "add_vector": 6,
        "set_result": 1,
        "set_vector": 2,
    }

    # upstream collision_box_box_cast_unit, through each cast overload
    for overload, cast in result["continuous"].items():
        assert cast["stationary_contacts"] == 0, overload
        assert abs(cast["distance"] - (-0.2475)) <= ex.BOX_BOX_TOL, overload
        np.testing.assert_allclose(cast["cc_time"], [-1.0, 0.25], atol=ex.BOX_BOX_TOL)
        assert cast["cc_type"] == ["CCType_None", "CCType_Between"], overload
        np.testing.assert_allclose(
            cast["swept_start_point"], [-1.275, -0.625, 0.0], atol=ex.BOX_BOX_TOL
        )
        np.testing.assert_allclose(
            cast["swept_end_point"], [2.525, 3.175, 0.0], atol=ex.BOX_BOX_TOL
        )
        assert cast["subshape_id"] == result["continuous"]["names"]["subshape_id"], overload
    assert list(result["continuous"]) == ["name", "names", "maps"]

    env = result["environment"]
    assert env["validator_matches_scene_graph"]
    assert env["adjacent_allowed"] and not env["distant_allowed"]
    assert env["by_name"] == ("FCLDiscreteBVHManager", "BulletCastSimpleManager")
    assert env["commands"] == ("BulletDiscreteSimpleManager", "BulletCastSimpleManager")
    assert env["active_after_commands"] == env["commands"]
    assert env["applied"] == (True, True)


@pytest.mark.basic
def test_trajectory_collision_example():
    ex = importlib.import_module("tesseract_robotics.examples.trajectory_collision_example")

    result = tesseract_robotics.examples.trajectory_collision_example()

    # boxbot: unit boxes 0.1 m in collision (tesseract_environment_utils.cpp:112-175)
    state = result["state"]
    for kind in ["discrete", "continuous", "fcl"]:
        assert state[kind]["pairs"] == [("boxbot_link", "test_box_link")], kind
        assert state[kind]["distance"] < 0.0, kind
        assert state[kind]["pairs_with_boxbot_disabled"] == [], kind
    # the analytic depth on FCL; Bullet's is backend-defined on face-aligned boxes
    assert abs(state["fcl"]["distance"] - (-ex.PENETRATION)) <= ex.BOX_BOX_TOL
    assert state["segment_pairs"] == [("boxbot_link", "test_box_link")]

    # checkTrajectory LVS_CONTINUOUS rebuilt from checkTrajectorySegment + addInterpolatedCollisionResults
    lvs = result["lvs"]
    assert lvs["steps_equal"] == [True, True, True, True]
    # upstream checkTrajectoryUnit, LVS_CONTINUOUS / ALL (tesseract_environment_unit.cpp:5486-5501)
    assert lvs["pairs_per_step"] == [2, 2, 3, 2]
    assert lvs["rebuilt_pairs_per_step"] == lvs["pairs_per_step"]
    assert lvs["contact_count"] == 288
    assert lvs["rebuilt_contact_count"] == 288
    assert lvs["contacts_per_step"][1] == 80
    assert lvs["most_collisions_step"] == (2, 118)
    assert lvs["rebuilt_contacts_per_step"] == lvs["contacts_per_step"]

    traj = result["joint_trajectory"]
    assert traj["equal_by_constructor"]
    assert traj["description"] == ex.TRAJECTORY_DESCRIPTION
    assert traj["uuid_round_trip"]
    assert traj["len"] == 2
    assert traj["positions"] == [[0.0, 0.0], [1.0, 1.0]]
    assert traj["states_match_iteration"]
    assert traj["after_setitem"] == [0.5, 0.5]
    assert traj["getitem_is_copy"]
    assert traj["cleared"] == (0, True)

    # AddTrajectoryLinkCommand: collision count per Method (add_trajectory_link_command.h:52-73)
    links = result["trajectory_link"]
    assert links["active_links_with_collision"] == [f"link_{i}" for i in range(1, 8)]
    assert links["collisions"] == {
        "PER_STATE_OBJECTS": 14,
        "PER_STATE_CONVEX_HULL": 2,
        "GLOBAL_PER_LINK_CONVEX_HULL": 7,
        "GLOBAL_CONVEX_HULL": 1,
    }
    assert links["collisions"] == links["expected_collisions"]
    for method, getters in links["getters"].items():
        assert getters == {
            "link_name": "traj_link",
            "parent_link_name": "base_link",
            "trajectory_equal": True,
            "replace_allowed": method != "PER_STATE_OBJECTS",
            "method": method,
            "applied": True,
        }, method


@pytest.mark.basic
def test_convex_decomposition_example():
    ex = importlib.import_module("tesseract_robotics.examples.convex_decomposition_example")

    result = tesseract_robotics.examples.convex_decomposition_example()

    # the CLI defaults are VHACDParameters' defaults (convex_decomposition_vhacd.h:41-59)
    assert result["parameters"] == {
        "max_convex_hulls": 64,
        "resolution": 400000,
        "minimum_volume_percent_error_allowed": 1.0,
        "max_recursion_depth": 10,
        "shrinkwrap": True,
        "fill_mode": "FLOOD_FILL",
        "max_num_vertices_per_ch": 64,
        "async_ACD": True,
        "min_edge_length": 2,
        "find_best_plane": False,
    }
    # every option given on the command line reaches its field
    assert result["parameters_from_argv"] == {
        "max_convex_hulls": 8,
        "resolution": 1000,
        "minimum_volume_percent_error_allowed": 2.0,
        "max_recursion_depth": 4,
        "shrinkwrap": False,
        "fill_mode": "RAYCAST_FILL",
        "max_num_vertices_per_ch": 16,
        "async_ACD": True,
        "min_edge_length": 3,
        "find_best_plane": True,
    }
    # upstream's notifier ranges reject out-of-range values
    for argv in [["-n", "33"], ["-n", "0"], ["-e", "11"], ["-v", "3"], ["-r", "0"], ["-l", "0"]]:
        with pytest.raises(SystemExit):
            ex.parse_args(argv)

    seat = result["seat"]
    assert 0 < seat["num_hulls"] <= seat["max_convex_hulls"]
    assert seat["all_convex"]
    assert seat["files_written"] == seat["num_hulls"]
    assert seat["reloaded_matches"]
    assert result["default_constructor_equal"]

    # each FillMode gives its own decomposition, each convex
    modes = result["fill_modes"]
    assert list(modes) == ["FLOOD_FILL", "SURFACE_ONLY", "RAYCAST_FILL"]
    assert all(mode["all_convex"] for mode in modes.values())
    assert result["fill_modes_differ"]

    # reported, not asserted equal: the decomposition shipped with tesseract support
    assert result["shipped"]["num_hulls"] == 10


@pytest.mark.basic
def test_environment_commands_example():
    ex = importlib.import_module("tesseract_robotics.examples.environment_commands_example")

    result = tesseract_robotics.examples.environment_commands_example()

    # every GeneralResourceLocator constructor resolves the URDF to the same file
    assert result["locators"] == {
        "paths_same_file": True,
        "environment_variables_same_file": True,
        "unresolved_before_load": True,
        "load_set_variable": True,
        "load_unset_variable": False,
        "resolved_after_load": True,
        "add_path": True,
        "fixture_resolved": True,
    }

    # getInitCommands (environment.cpp:116-133): no calibration and no margins in abb_irb2400.srdf
    assert result["init_history"] == [
        "ADD_SCENE_GRAPH",
        "ADD_CONTACT_MANAGERS_PLUGIN_INFO",
        "ADD_KINEMATICS_INFORMATION",
    ]
    assert result["urdf_only_history"] == ["ADD_SCENE_GRAPH"]

    # upstream's calibration fixture (tesseract_srdf_unit.cpp:672-690): one ChangeJointOrigin per joint
    calibration = result["calibration"]
    assert calibration["joints"] == ex.CALIBRATED_POSITIONS
    assert calibration["rotations_identity"]
    assert calibration["history"] == result["init_history"] + ["CHANGE_JOINT_ORIGIN"] * 2
    assert calibration["calibrated_joints"] == ["joint_1", "joint_2"]
    assert calibration["origins_applied"]
    assert calibration["empty_before_insert"]
    assert calibration["insert_copies"]
    assert calibration["insert_overrides"] == {
        "joint_1": [1.0, 2.0, 3.0],
        "joint_2": [1.0, 2.0, 3.0],
    }
    assert calibration["empty_after_clear"]

    # SRDFModel.contact_managers_plugin_info read from contact_manager_plugins.yaml
    assert result["contact_managers_plugin_info"] == {
        "search_libraries": [
            "tesseract_collision_bullet_factories",
            "tesseract_collision_fcl_factories",
        ],
        "discrete_default": "BulletDiscreteBVHManager",
        "discrete_plugins": {
            "BulletDiscreteBVHManager": "BulletDiscreteBVHManagerFactory",
            "BulletDiscreteSimpleManager": "BulletDiscreteSimpleManagerFactory",
            "FCLDiscreteBVHManager": "FCLDiscreteBVHManagerFactory",
        },
        "continuous_default": "BulletCastBVHManager",
        "continuous_plugins": {
            "BulletCastBVHManager": "BulletCastBVHManagerFactory",
            "BulletCastSimpleManager": "BulletCastSimpleManagerFactory",
        },
        "equals_environment_info": True,
    }

    commands = result["commands"]
    assert commands["plugin_info_empty_before"]
    assert commands["manager_before"] is None
    assert commands["plugin_info_applied"]
    assert commands["command_round_trips"]
    assert commands["environment_info_equal"]
    assert commands["manager_after"] == "BulletDiscreteBVHManager"
    assert commands["batch_applied"]
    assert commands["batch_origins_applied"]
    # one COMMAND_APPLIED event per applyCommand / applyCommands call, carrying the whole history
    assert commands["events"] == [
        (2, ["ADD_SCENE_GRAPH", "ADD_CONTACT_MANAGERS_PLUGIN_INFO"]),
        (
            4,
            [
                "ADD_SCENE_GRAPH",
                "ADD_CONTACT_MANAGERS_PLUGIN_INFO",
                "CHANGE_JOINT_ORIGIN",
                "CHANGE_JOINT_ORIGIN",
            ],
        ),
    ]
    assert commands["events_match_history"]

    # init(commands): same history, but timestamps differ, so env != replay (environment.cpp:393-413)
    replay = result["replay"]
    assert replay == {
        "initialized": True,
        "histories_equal": True,
        "revisions_equal": True,
        "states_equal": True,
        "timestamps_differ": True,
        "replay_equals_env": False,
        "clone_equals_env": True,
        "clone_after_command_equals_env": False,
    }

    assert result["bytes_resource"] == {
        "url": ex.FIXTURE_URL,
        "is_file": False,
        "contents_equal": True,
        "constructors_equal": True,
        "contents_are_fixture": True,
    }


@pytest.mark.basic
def test_geometry_showcase_example():
    tesseract_robotics.examples.geometry_showcase_example()


@pytest.mark.basic
def test_pointcloud_octree_collision_example():
    # The shipped sample.ply lives in the source tree (docs/assets/) — it
    # isn't bundled with the wheel — so pass the repo-root path explicitly
    # rather than relying on the example's source-checkout default.
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    cloud = repo_root / "docs" / "assets" / "sample.ply"
    tesseract_robotics.examples.pointcloud_octree_collision_example(cloud)


@pytest.mark.planning
def test_freespace_ompl_example():
    tesseract_robotics.examples.freespace_ompl_example()


@pytest.mark.planning
def test_basic_cartesian_example():
    tesseract_robotics.examples.basic_cartesian_example()


@pytest.mark.planning
def test_glass_upright_example():
    tesseract_robotics.examples.glass_upright_example()


@pytest.mark.planning
def test_pick_and_place_example():
    tesseract_robotics.examples.pick_and_place_example()


@pytest.mark.planning
def test_car_seat_example():
    tesseract_robotics.examples.car_seat_example()


@pytest.mark.planning
def test_puzzle_piece_auxillary_axes_example():
    tesseract_robotics.examples.puzzle_piece_auxillary_axes_example()


@pytest.mark.planning
def test_raster_example():
    tesseract_robotics.examples.raster_example()


@pytest.mark.planning
def test_online_planning_example():
    tesseract_robotics.examples.online_planning_example()


@pytest.mark.planning
def test_online_planning_sqp_example():
    tesseract_robotics.examples.online_planning_sqp_example()


@pytest.mark.planning
def test_freespace_hybrid_example():
    tesseract_robotics.examples.freespace_hybrid_example()


@pytest.mark.planning
def test_chain_example():
    tesseract_robotics.examples.chain_example()


# === Low-Level API Examples ===


@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_collision_example():
    tesseract_robotics.examples.tesseract_collision_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_kinematics_example():
    tesseract_robotics.examples.tesseract_kinematics_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_scene_graph_example():
    tesseract_robotics.examples.scene_graph_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_freespace_ompl_example():
    tesseract_robotics.examples.freespace_ompl_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_basic_cartesian_example():
    tesseract_robotics.examples.basic_cartesian_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_glass_upright_example():
    tesseract_robotics.examples.glass_upright_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_puzzle_piece_example():
    tesseract_robotics.examples.puzzle_piece_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_pick_and_place_example():
    tesseract_robotics.examples.pick_and_place_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_car_seat_example():
    tesseract_robotics.examples.car_seat_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_puzzle_piece_auxillary_axes_example():
    tesseract_robotics.examples.puzzle_piece_auxillary_axes_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_pythonic_example():
    tesseract_robotics.examples.pythonic_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_planning_composer_example():
    tesseract_robotics.examples.tesseract_planning_composer_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_planning_lowlevel_example():
    tesseract_robotics.examples.tesseract_planning_lowlevel_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_descartes_raster_example():
    tesseract_robotics.examples.descartes_raster_c_api_example()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_trajopt_ifopt_example():
    tesseract_robotics.examples.tesseract_planning_lowlevel_trajopt_ifopt_example()


# OMPL seeds whose TrajOptIfopt trajectory TOTG could not parameterize ("Negative path
# velocity"); the example now uses ISP, which accepted all of seeds 1-200 (#103). Where the
# seed reaches OMPL (Linux, macOS) each case replays its old failure deterministically, so a
# return to a fragile parameterization fails here rather than as a Windows coin flip.
SEEDS_THAT_BROKE_TOTG = (1, 9, 16, 22, 23, 30, 38)


@pytest.mark.lowlevel
@pytest.mark.planning
@pytest.mark.parametrize("seed", SEEDS_THAT_BROKE_TOTG)
def test_lowlevel_trajopt_ifopt_example_on_seeds_that_broke_totg(seed):
    from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed

    RNG_setSeed(seed)
    tesseract_robotics.examples.tesseract_planning_lowlevel_trajopt_ifopt_example()


# OMPL seeds (of 1-100) whose TrajOpt-smoothed path touched the sphere and failed the
# pipeline's DiscreteContactCheckTask while the TrajOpt cost had no clearance margin; with
# the 5 mm cost margin each keeps 5.0-5.9 mm of clearance (#103). Each replays its old
# failure deterministically where the seed reaches OMPL (Linux, macOS).
SEEDS_THAT_GRAZED_THE_SPHERE = (39, 43, 96)


@pytest.mark.planning
@pytest.mark.parametrize("seed", SEEDS_THAT_GRAZED_THE_SPHERE)
def test_freespace_ompl_example_on_seeds_that_grazed_the_sphere(seed):
    from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed

    RNG_setSeed(seed)
    tesseract_robotics.examples.freespace_ompl_example()


# Joint-space resolution of the clearance scan below. A step moves link_7 a few millimetres;
# the path passes the sphere tangentially, so the sampled minimum sits a small fraction of a
# millimetre above the true one, far inside the 2.5 mm between the assertion and the
# clearances measured (the seed 1-100 figures in the test were taken at this resolution).
CLEARANCE_SCAN_STEP_RAD = 0.005
# Contact reporting distance for the scan: 20x the cost margin, so the clearance is measured
# rather than clipped at the margin.
CLEARANCE_REPORT_DISTANCE_M = 0.1
# TrajOpt collision cost margin of C++ car_seat_example.cpp, TrajOptCollisionConfig(0.005, 50),
# which the planning factories follow; test_trajopt_collision_margins_match_cpp_car_seat pins it.
CAR_SEAT_COST_MARGIN_M = 0.005


def _min_distance_to(obstacle, env, joint_names, positions):
    """Minimum signed distance between any robot link and `obstacle` along a joint path."""
    from tesseract_robotics.tesseract_collision import (
        ContactRequest,
        ContactResultMap,
        ContactResultVector,
        ContactTestType,
    )
    from tesseract_robotics.tesseract_common import CollisionMarginData
    from tesseract_robotics.tesseract_state_solver import OFKTStateSolver

    solver = OFKTStateSolver(env.getSceneGraph())
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkNames())
    manager.setCollisionMarginData(CollisionMarginData(CLEARANCE_REPORT_DISTANCE_M))
    closest = np.inf
    for start, end in zip(positions[:-1], positions[1:]):
        steps = max(1, int(np.ceil(np.max(np.abs(end - start)) / CLEARANCE_SCAN_STEP_RAD)))
        for fraction in np.linspace(0.0, 1.0, steps + 1):
            solver.setStateByNamesAndValues(joint_names, start + fraction * (end - start))
            manager.setCollisionObjectsTransform(solver.getState().link_transforms)
            contacts = ContactResultMap()
            manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
            flat = ContactResultVector()
            contacts.flattenMoveResults(flat)
            for index in range(len(flat)):
                if obstacle in list(flat[index].link_names):
                    closest = min(closest, flat[index].distance)
    return closest


@pytest.mark.planning
def test_freespace_ompl_example_keeps_half_the_cost_margin_from_the_sphere():
    """The TrajOpt collision cost holds the freespace path off the sphere, whatever path OMPL found.

    With no cost margin the optimizer pulled the path onto the sphere: median clearance 0.17 mm
    over OMPL seeds 1-100, and the pipeline's contact check failed whenever the solver's residual
    came out negative. With the 5 mm margin the least clearance over those seeds was 4.98 mm; the
    cost is soft, so it may settle slightly inside its margin, and half the margin allows for
    that. The assertion needs no seed, so it holds where the seed does not reach OMPL (#103).
    """
    outcome = tesseract_robotics.examples.freespace_ompl_example()
    positions = [np.asarray(point.positions, dtype=float) for point in outcome["result"].trajectory]
    clearance = _min_distance_to(
        "sphere_attached", outcome["robot"].env, outcome["joint_names"], positions
    )
    assert clearance >= CAR_SEAT_COST_MARGIN_M / 2, f"{clearance * 1000:.3f} mm from the sphere"
