"""Shared trajopt_ifopt fixtures (scenes in scenes.py)."""

from __future__ import annotations

import pytest

from .scenes import OVERLAP_X, Iiwa, TwoBoxes, load_iiwa, two_boxes_at


@pytest.fixture
def two_boxes() -> TwoBoxes:
    """The two-box scene with one node, the boxes overlapping by 0.1 m."""
    return two_boxes_at(OVERLAP_X)


@pytest.fixture
def iiwa() -> Iiwa:
    return load_iiwa()
