"""Tests for clone() bindings across the codebase."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
)
from tesseract_robotics.tesseract_environment import Environment


@pytest.fixture
def abb_environment():
    """Load ABB IRB2400 environment."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(
        locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource("package://tesseract_support/urdf/abb_irb2400.srdf").getFilePath()
    )
    env = Environment()
    assert env.init(urdf_path, srdf_path, locator)
    return env


class TestDiscreteContactManagerClone:
    def test_clone_returns_new_instance(self, abb_environment):
        mgr = abb_environment.getDiscreteContactManager()
        cloned = mgr.clone()
        assert cloned is not mgr

    def test_clone_preserves_name(self, abb_environment):
        mgr = abb_environment.getDiscreteContactManager()
        cloned = mgr.clone()
        assert cloned.getName() == mgr.getName()


class TestContinuousContactManagerClone:
    def test_clone_returns_new_instance(self, abb_environment):
        mgr = abb_environment.getContinuousContactManager()
        cloned = mgr.clone()
        assert cloned is not mgr

    def test_clone_preserves_name(self, abb_environment):
        mgr = abb_environment.getContinuousContactManager()
        cloned = mgr.clone()
        assert cloned.getName() == mgr.getName()


class TestEnvironmentClone:
    def test_clone_returns_new_instance(self, abb_environment):
        cloned = abb_environment.clone()
        assert cloned is not abb_environment

    def test_clone_preserves_state(self, abb_environment):
        joint_names = list(abb_environment.getStateSolver().getActiveJointNames())
        original_state = abb_environment.getState()
        cloned = abb_environment.clone()
        cloned_state = cloned.getState()
        for name in joint_names:
            assert original_state.joints[name] == pytest.approx(cloned_state.joints[name])

    def test_clone_is_independent(self, abb_environment):
        cloned = abb_environment.clone()
        joint_names = list(abb_environment.getStateSolver().getActiveJointNames())
        new_values = np.ones(len(joint_names)) * 0.5
        cloned.setState(joint_names, new_values)
        # original should be unaffected
        original_state = abb_environment.getState()
        for name in joint_names:
            assert original_state.joints[name] != pytest.approx(0.5)


class TestStateSolverClone:
    def test_polymorphic_clone(self, abb_environment):
        solver = abb_environment.getStateSolver()
        cloned = solver.clone()
        assert cloned is not solver
        assert list(cloned.getJointNames()) == list(solver.getJointNames())


class TestMotionPlannerClone:
    def test_trajopt_clone(self):
        from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptMotionPlanner

        planner = TrajOptMotionPlanner("test_trajopt")
        cloned = planner.clone()
        assert cloned is not planner
        assert cloned.getName() == "test_trajopt"

    def test_trajopt_ifopt_clone(self):
        from tesseract_robotics.tesseract_motion_planners_trajopt_ifopt import (
            TrajOptIfoptMotionPlanner,
        )

        planner = TrajOptIfoptMotionPlanner("test_ifopt")
        cloned = planner.clone()
        assert cloned is not planner
        assert cloned.getName() == "test_ifopt"

    def test_ompl_clone(self):
        from tesseract_robotics.tesseract_motion_planners_ompl import OMPLMotionPlanner

        planner = OMPLMotionPlanner("test_ompl")
        cloned = planner.clone()
        assert cloned is not planner
        assert cloned.getName() == "test_ompl"

    def test_simple_clone(self):
        from tesseract_robotics.tesseract_motion_planners_simple import SimpleMotionPlanner

        planner = SimpleMotionPlanner("test_simple")
        cloned = planner.clone()
        assert cloned is not planner
        assert cloned.getName() == "test_simple"

    def test_descartes_clone(self):
        from tesseract_robotics.tesseract_motion_planners_descartes import DescartesMotionPlannerD

        planner = DescartesMotionPlannerD("test_descartes")
        cloned = planner.clone()
        assert cloned is not planner
        assert cloned.getName() == "test_descartes"
