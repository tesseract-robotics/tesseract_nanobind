"""StateSolver on upstream's id API: str in, ids out, id-keyed transforms indexed by str."""

import os

import numpy as np

from tesseract_robotics import tesseract_common, tesseract_state_solver, tesseract_urdf

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

IIWA_JOINTS = [f"joint_a{i}" for i in range(1, 8)]


def _solver():
    path = os.path.join(os.environ["TESSERACT_SUPPORT_DIR"], "urdf/lbr_iiwa_14_r820.urdf")
    scene_graph = tesseract_urdf.parseURDFFile(path, TesseractSupportResourceLocator())
    return tesseract_state_solver.KDLStateSolver(scene_graph)


def test_set_state_from_str_names_and_index_transforms_by_str():
    solver = _solver()
    values = np.full(7, 0.1)
    solver.setStateByNamesAndValues(IIWA_JOINTS, values)
    state = solver.getState()
    assert state.joints["joint_a3"] == 0.1
    assert isinstance(state.link_transforms["tool0"], tesseract_common.Isometry3d)
    np.testing.assert_allclose(state.getJointValues(IIWA_JOINTS), values)


def test_id_getters_return_ids():
    solver = _solver()
    active = solver.getActiveJointIds()
    assert all(isinstance(j, tesseract_common.JointId) for j in active)
    assert sorted(j.name() for j in active) == IIWA_JOINTS
    assert solver.getBaseLinkId() == "base_link"
    assert solver.hasLinkId("tool0")
    assert not solver.hasLinkId("no_such_link")
