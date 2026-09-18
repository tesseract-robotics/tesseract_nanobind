"""Tests that run the example scripts to verify API coverage.

Markers:
  - @pytest.mark.viewer: Viewer examples
  - @pytest.mark.planning: Motion planning examples
  - @pytest.mark.basic: Basic examples (collision, kinematics, scene_graph)
  - @pytest.mark.lowlevel: Low-level API examples
"""

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
        ContactTestType_ALL,
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
            manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))
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
