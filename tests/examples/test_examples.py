"""Tests that run the example scripts to verify API coverage.

Markers:
  - @pytest.mark.viewer: Viewer examples
  - @pytest.mark.planning: Motion planning examples
  - @pytest.mark.basic: Basic examples (collision, kinematics, scene_graph)
  - @pytest.mark.lowlevel: Low-level API examples
"""

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
def test_lowlevel_trajopt_ifopt_example():
    tesseract_robotics.examples.tesseract_planning_lowlevel_trajopt_ifopt_example()
