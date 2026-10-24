"""The GIL releases added for #124 (scripts/nogil_entry_points.yaml).

scripts/audit_gil.py checks statically that every listed binding releases the GIL; these tests
check it at run time for the entry points #124 added, and that each still returns the right
result with the GIL released.

The oracle: with a switch interval far above the test's run time, the main thread can only run
while the worker is still busy if the worker drops the GIL. The worker repeats the call until the
main thread has run, so a call too fast to observe once is caught on a later repetition. A call
that holds the GIL fails deterministically, never passes vacuously.
"""

import os
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics import tesseract_environment, tesseract_kinematics, tesseract_urdf
from tesseract_robotics.tesseract_common import Isometry3d, ResourceLocator

from .tesseract_support_resource_locator import TesseractSupportResourceLocator
from .tesseract_time_parameterization.test_time_parameterization import (
    _abb_program,
    abb_env,  # noqa: F401  (fixture)
)

LONG_SWITCH_INTERVAL = 100.0
ATTEMPTS = 200


def assert_releases_gil(call):
    """Call `call()` on a worker thread until the main thread has run; fail if it never could."""
    main_ran = threading.Event()
    state = {"done": False, "calls": 0, "error": None}

    def worker():
        try:
            for _ in range(ATTEMPTS):
                call()
                state["calls"] += 1
                if main_ran.is_set():
                    break
        except BaseException as e:  # reported on the main thread
            state["error"] = e
        finally:
            state["done"] = True

    previous = sys.getswitchinterval()
    sys.setswitchinterval(LONG_SWITCH_INTERVAL)
    try:
        thread = threading.Thread(target=worker)
        # start() waits for the worker to start; it returns only once the worker drops the GIL.
        thread.start()
        ran_during_worker = not state["done"]
        main_ran.set()
        thread.join()
    finally:
        sys.setswitchinterval(previous)

    if state["error"] is not None:
        raise state["error"]
    assert ran_during_worker, (
        f"main thread never ran during {state['calls']} calls: the GIL was held throughout"
    )


def _support_urdf(name):
    return Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf" / name


class _PythonLocator(ResourceLocator):
    """A Python locator: C++ calls it from inside the released region, through the trampoline,
    which has to re-acquire the GIL."""

    def __init__(self):
        super().__init__()
        self._inner = TesseractSupportResourceLocator()
        self.urls = []

    def locateResource(self, url):
        self.urls.append(url)
        return self._inner.locateResource(url)


@pytest.fixture(scope="module")
def iiwa_group():
    env = tesseract_environment.Environment()
    assert env.init(
        _support_urdf("lbr_iiwa_14_r820.urdf"),
        _support_urdf("lbr_iiwa_14_r820.srdf"),
        TesseractSupportResourceLocator(),
    )
    return env.getKinematicGroup("manipulator")


def test_kinematic_group_calc_inv_kin_releases_gil(iiwa_group):
    # Out of reach: the numerical solver (KDL LMA) runs to its iteration limit every call.
    pose = Isometry3d.Identity().translate([10.0, 0.0, 0.0])
    ik_input = tesseract_kinematics.KinGroupIKInput(pose, iiwa_group.getBaseLinkName(), "tool0")
    seed = np.zeros(7)

    assert_releases_gil(lambda: iiwa_group.calcInvKin(ik_input, seed))
    assert len(iiwa_group.calcInvKin(ik_input, seed)) == 0


def test_kinematic_group_calc_inv_kin_still_solves(iiwa_group):
    q = np.array([0.1, 0.2, -0.1, -0.5, 0.2, 0.3, 0.0])
    pose = iiwa_group.calcFwdKin(q)["tool0"]
    ik_input = tesseract_kinematics.KinGroupIKInput(pose, iiwa_group.getBaseLinkName(), "tool0")

    solutions = iiwa_group.calcInvKin([ik_input], q)

    assert len(solutions) > 0
    for solution in solutions:
        np.testing.assert_allclose(
            iiwa_group.calcFwdKin(solution)["tool0"].matrix, pose.matrix, atol=1e-4
        )


def test_parse_urdf_file_releases_gil():
    path = str(_support_urdf("lbr_iiwa_14_r820.urdf"))
    locator = _PythonLocator()

    assert_releases_gil(lambda: tesseract_urdf.parseURDFFile(path, locator))
    assert len(tesseract_urdf.parseURDFFile(path, locator).getLinks()) > 0
    # The mesh URLs went through Python while the GIL was released around the parse.
    assert any(url.startswith("package://") for url in locator.urls)


def test_parse_urdf_string_releases_gil():
    urdf = _support_urdf("lbr_iiwa_14_r820.urdf").read_text()
    locator = _PythonLocator()

    assert_releases_gil(lambda: tesseract_urdf.parseURDFString(urdf, locator))


def test_time_parameterization_compute_releases_gil(abb_env):  # noqa: F811
    from tesseract_robotics.tesseract_command_language import ProfileDictionary
    from tesseract_robotics.tesseract_time_parameterization import (
        TimeOptimalTrajectoryGeneration,
        TOTGCompositeProfile,
    )

    profiles = ProfileDictionary()
    profiles.addProfile("TOTG", "DEFAULT", TOTGCompositeProfile())
    totg = TimeOptimalTrajectoryGeneration()

    assert_releases_gil(lambda: totg.compute(_abb_program(num=200), abb_env, profiles))

    program = _abb_program(num=200)
    assert totg.compute(program, abb_env, profiles)


# --- Releases found by `pixi run gil-hotspots` -------------------------------------------------


def test_generate_interpolated_program_releases_gil(abb_env):  # noqa: F811
    from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

    program = _abb_program(num=20)
    # 0.001 rad segments: hundreds of interpolated states, each with its own state solve.
    interpolate = lambda: generateInterpolatedProgram(program, abb_env, 0.001, 1.0, 0.001, 1)  # noqa: E731

    assert_releases_gil(interpolate)
    assert len(interpolate().flatten()) > len(program.flatten())


def test_environment_serialization_releases_gil(abb_env, tmp_path):  # noqa: F811
    from tesseract_robotics import tesseract_serialization as ts

    xml = ts.environment_to_xml(abb_env)
    data = ts.environment_to_binary(abb_env)
    path = str(tmp_path / "env.xml")
    ts.environment_to_file(abb_env, path)

    for call in (
        lambda: ts.environment_to_xml(abb_env),
        lambda: ts.environment_from_xml(xml),
        lambda: ts.environment_to_binary(abb_env),
        lambda: ts.environment_from_binary(data),
        lambda: ts.environment_to_file(abb_env, path),
        lambda: ts.environment_from_file(path),
    ):
        assert_releases_gil(call)
    assert set(ts.environment_from_xml(xml).getJointNames()) == set(abb_env.getJointNames())


@pytest.mark.parametrize("kind", ["Mesh", "ConvexMesh", "SDFMesh"])
def test_create_mesh_releases_gil(kind):
    from tesseract_robotics import tesseract_geometry as tg

    path = str(Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "meshes" / "sphere_p25m.dae")
    from_path = getattr(tg, f"create{kind}FromPath")
    from_resource = getattr(tg, f"create{kind}FromResource")
    locator = _PythonLocator()
    resource = locator.locateResource("package://tesseract/support/meshes/sphere_p25m.dae")

    assert_releases_gil(lambda: from_path(path))
    assert_releases_gil(lambda: from_resource(resource))
    assert len(from_resource(resource)) == len(from_path(path)) > 0
