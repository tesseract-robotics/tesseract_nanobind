"""Subprocess body: instrumenting a trampoline base keeps a missing override's pure-virtual error.

`ResourceLocator.locateResource` is pure virtual (NB_OVERRIDE_PURE). A Python
subclass that does not override it must get nanobind's "tried to call a pure
virtual function" error, both for a subclass defined before the recorder is
installed and for one defined after.
"""

import sys
from pathlib import Path

from tesseract_robotics.tesseract_common import ResourceLocator


class DefinedBefore(ResourceLocator):
    pass


sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from tests.examples.api_recorder import Recorder  # noqa: E402

Recorder(Path(__file__).resolve().parent).install()


class DefinedAfter(ResourceLocator):
    pass


for cls in (DefinedBefore, DefinedAfter):
    try:
        cls().locateResource("package://fixture/none")
    except RuntimeError as exc:
        print(f"{cls.__name__}: {type(exc).__name__}: {str(exc).splitlines()[0]}")
