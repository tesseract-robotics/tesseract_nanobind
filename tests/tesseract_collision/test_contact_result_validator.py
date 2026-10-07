"""ContactResultValidator and ContactRequest.is_valid: Python approves or rejects contacts (gh-171).

contactTest releases the GIL; the trampoline takes it again before calling __call__.
"""

import gc
import subprocess
import sys
import threading

import pytest

import tesseract_robotics.tesseract_collision as tc
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactTestType,
)

from .test_contact_manager_config import _box, _get_discrete_factory, _two_overlapping_boxes

# Seconds a contactTest in a worker thread may take before the GIL hand-off counts as hung.
THREAD_JOIN_TIMEOUT_S = 30.0


def _validator_class():
    """Subclass lazily, so a missing binding fails per test instead of at collection."""

    class Recording(tc.ContactResultValidator):
        def __init__(self, accept):
            super().__init__()
            self.accept = accept
            self.seen = []

        def __call__(self, result):
            self.seen.append(tuple(result.link_names))
            return self.accept(result)

    return Recording


@pytest.fixture
def discrete_checker():
    factory, locator = _get_discrete_factory()
    checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    yield checker
    del checker
    del factory
    del locator
    gc.collect()


def _contacts(checker, request):
    result = ContactResultMap()
    checker.contactTest(result, request)
    return result


def test_reject_all_leaves_no_contacts(discrete_checker):
    _two_overlapping_boxes(discrete_checker)
    assert _contacts(discrete_checker, ContactRequest(ContactTestType.ALL)).count() == 1

    validator = _validator_class()(lambda r: False)
    request = ContactRequest(ContactTestType.ALL)
    request.is_valid = validator
    assert _contacts(discrete_checker, request).count() == 0
    assert validator.seen == [("box_a", "box_b")]


def test_reject_by_link_names_keeps_allowed_pairs(discrete_checker):
    _two_overlapping_boxes(discrete_checker)
    shapes_c, poses_c = _box()
    discrete_checker.addCollisionObject("box_c", 0, shapes_c, poses_c)
    discrete_checker.setActiveCollisionObjects(["box_a", "box_b", "box_c"])
    assert _contacts(discrete_checker, ContactRequest(ContactTestType.ALL)).count() == 3

    validator = _validator_class()(lambda r: "box_c" not in r.link_names)
    request = ContactRequest(ContactTestType.ALL)
    request.is_valid = validator
    result = _contacts(discrete_checker, request)
    assert [key for key, v in result.getContainer().items() if len(v)] == [("box_a", "box_b")]
    assert sorted(validator.seen) == [("box_a", "box_b"), ("box_a", "box_c"), ("box_b", "box_c")]


def test_is_valid_defaults_to_none_and_none_clears():
    request = ContactRequest()
    assert request.is_valid is None
    request.is_valid = _validator_class()(lambda r: True)
    assert request.is_valid is not None
    request.is_valid = None
    assert request.is_valid is None


def test_validator_is_directly_callable():
    validator = _validator_class()(lambda r: r.distance < 0.0)
    r = ContactResult()
    r.distance = -0.1
    assert validator(r) is True
    r.distance = 0.1
    assert validator(r) is False


def test_validator_survives_caller_reference(discrete_checker):
    _two_overlapping_boxes(discrete_checker)
    calls = []
    request = ContactRequest(ContactTestType.ALL)
    request.is_valid = _validator_class()(lambda r: calls.append(r) or False)
    gc.collect()
    assert _contacts(discrete_checker, request).count() == 0
    assert len(calls) == 1


def test_contact_test_in_thread_takes_gil_for_validator(discrete_checker):
    _two_overlapping_boxes(discrete_checker)
    validator = _validator_class()(lambda r: True)
    request = ContactRequest(ContactTestType.ALL)
    request.is_valid = validator
    counts = []

    worker = threading.Thread(
        target=lambda: counts.append(_contacts(discrete_checker, request).count())
    )
    worker.start()
    worker.join(THREAD_JOIN_TIMEOUT_S)
    assert not worker.is_alive()
    assert counts == [1]
    assert validator.seen == [("box_a", "box_b")]


_RAISING_VALIDATOR_SCRIPT = """\
import os
from pathlib import Path

import numpy as np

import tesseract_robotics  # noqa: F401 - sets TESSERACT_SUPPORT_DIR
from tesseract_robotics.tesseract_collision import (
    ContactManagersPluginFactory, ContactRequest, ContactResultMap, ContactResultValidator,
    ContactTestType,
)
from tesseract_robotics.tesseract_common import (
    CollisionMarginData, GeneralResourceLocator, Isometry3d, VectorIsometry3d,
)
from tesseract_robotics.tesseract_geometry import Box, GeometriesConst


class Raising(ContactResultValidator):
    def __call__(self, result):
        raise ValueError("validator rejected loudly")


cfg = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf" / "contact_manager_plugins.yaml"
factory = ContactManagersPluginFactory(cfg, GeneralResourceLocator())
checker = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
for name in ("box_a", "box_b"):
    shapes = GeometriesConst()
    shapes.append(Box(1.0, 1.0, 1.0))
    poses = VectorIsometry3d()
    poses.append(Isometry3d(np.eye(4)))
    checker.addCollisionObject(name, 0, shapes, poses)
checker.setActiveCollisionObjects(["box_a", "box_b"])
checker.setCollisionMarginData(CollisionMarginData(0.1))

request = ContactRequest(ContactTestType.ALL)
request.is_valid = Raising()
try:
    checker.contactTest(ContactResultMap(), request)
except ValueError as exc:
    print(f"RAISED: {exc}")
"""


def test_raising_validator_propagates_value_error():
    """Run in a subprocess: an exception crossing a noexcept backend frame would abort."""
    proc = subprocess.run(
        [sys.executable, "-c", _RAISING_VALIDATOR_SCRIPT],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode}: {proc.stderr[-800:]}"
    assert "RAISED: validator rejected loudly" in proc.stdout
