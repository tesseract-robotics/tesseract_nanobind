"""Floor under the scene: covers every visual's footprint, on whole grid sections."""

import numpy as np
import pytest
import trimesh

from tesseract_robotics.viewer.floor import (
    FLOOR_MARGIN_M,
    FLOOR_SECTION_M,
    MIN_FLOOR_SIZE_M,
    Floor,
)


def _box_at(extents, xyz):
    box = trimesh.creation.box(extents=extents)
    pose = np.eye(4)
    pose[:3, 3] = xyz
    return pose, box


def test_empty_scene_gets_the_smallest_floor_at_the_origin():
    floor = Floor.under([])
    assert floor.center_m == (0.0, 0.0)
    assert floor.size_m == MIN_FLOOR_SIZE_M


def test_small_scene_gets_the_smallest_floor():
    floor = Floor.under([_box_at((1.0, 1.0, 1.0), (0.0, 0.0, 0.5))])
    assert floor.size_m == MIN_FLOOR_SIZE_M


def test_large_scene_is_covered_with_a_margin():
    # 14 m long, 2 m wide, centred at x = 5.3 m.
    floor = Floor.under([_box_at((14.0, 2.0, 1.0), (5.3, 0.0, 0.5))])
    half = floor.size_m / 2
    cx, cy = floor.center_m
    assert cx - half <= 5.3 - 7.0 - FLOOR_MARGIN_M
    assert cx + half >= 5.3 + 7.0 + FLOOR_MARGIN_M
    assert cy - half <= -1.0 - FLOOR_MARGIN_M
    assert cy + half >= 1.0 + FLOOR_MARGIN_M


def test_grid_lines_stay_on_whole_metres():
    floor = Floor.under([_box_at((14.0, 2.0, 1.0), (5.3, -2.7, 0.5))])
    for value in (*floor.center_m, floor.size_m / 2):
        assert value / FLOOR_SECTION_M == pytest.approx(round(value / FLOOR_SECTION_M), abs=0.0)


def test_rotated_visual_footprint_is_covered():
    pose, box = _box_at((20.0, 0.2, 0.2), (0.0, 0.0, 0.0))
    pose[:3, :3] = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1])[:3, :3]
    floor = Floor.under([(pose, box)])
    # Along y now: 10 m each way, plus the margin.
    assert floor.size_m / 2 >= 10.0 + FLOOR_MARGIN_M
