"""pytest plugin: record which nanobind API the shipped examples call, overload by overload.

Load with `-p tests.examples.api_recorder`. In `pytest_configure`, before
collection imports any example, it imports every `tesseract_robotics`
extension package and replaces, in place, each nanobind callable with a
recording wrapper: methods and dunders in the class that defines them, static
methods, property getters and setters, constructors, and module functions in
every namespace that re-exports them (so a later `from m import f` in an
example binds the wrapper). A call is recorded only when its immediate caller
frame is a file under the example root; library code an example calls into
does not count. Each recorded call carries the overload it dispatched to, as
decided by `tests.examples.overload_matcher`.

Each process (each xdist worker) writes `<record-dir>/<worker>.json` at
`pytest_unconfigure`; `scripts/example_api_coverage.py` merges them.

Constructors need more than a wrapper: nanobind's type vectorcall calls the
cached `__init__` function directly and never looks at the class dict. The
recorder clears `tp_vectorcall` on each class that defines `__init__`, which
is what nanobind itself does for Python subclasses (nb_type.cpp, `nb_type_init`):
construction then goes through `type_call` and the class dict's `__init__`.
The type object is reached through a ctypes view of the `PyTypeObject` head,
cross-checked against six attributes Python exposes before anything is written.
"""

from __future__ import annotations

import ctypes
import enum
import importlib
import importlib.util
import json
import os
import pkgutil
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from types import FrameType, ModuleType
from typing import Any, Callable

import pytest

from tests.examples.overload_matcher import match_call

PACKAGE = "tesseract_robotics"
EXAMPLES_PACKAGE = f"{PACKAGE}.examples"
NANOBIND_MODULE = "nanobind"
NB_FUNC, NB_METHOD = "nb_func", "nb_method"
DEFAULT_RECORD_DIR = Path("build/example-api-coverage/recordings")
SERIAL_WORKER = "main"
CTOR = "__init__"
WRAPPED_METADATA = ("__module__", "__name__", "__qualname__", "__doc__")


class TypeLayoutError(Exception):
    """The ctypes view of a type object disagrees with the attributes Python reports."""


class ExampleRootError(Exception):
    """The example root does not exist."""


_P, _S = ctypes.c_void_p, ctypes.c_ssize_t


class _TypeHead(ctypes.Structure):
    """`PyTypeObject` up to `tp_vectorcall` (unchanged from CPython 3.8 through 3.14)."""

    _fields_ = [
        ("ob_refcnt", _S), ("ob_type", _P), ("ob_size", _S), ("tp_name", ctypes.c_char_p),
        ("tp_basicsize", _S), ("tp_itemsize", _S), ("tp_dealloc", _P), ("tp_vectorcall_offset", _S),
        ("tp_getattr", _P), ("tp_setattr", _P), ("tp_as_async", _P), ("tp_repr", _P),
        ("tp_as_number", _P), ("tp_as_sequence", _P), ("tp_as_mapping", _P), ("tp_hash", _P),
        ("tp_call", _P), ("tp_str", _P), ("tp_getattro", _P), ("tp_setattro", _P),
        ("tp_as_buffer", _P), ("tp_flags", ctypes.c_ulong), ("tp_doc", _P), ("tp_traverse", _P),
        ("tp_clear", _P), ("tp_richcompare", _P), ("tp_weaklistoffset", _S), ("tp_iter", _P),
        ("tp_iternext", _P), ("tp_methods", _P), ("tp_members", _P), ("tp_getset", _P),
        ("tp_base", _P), ("tp_dict", _P), ("tp_descr_get", _P), ("tp_descr_set", _P),
        ("tp_dictoffset", _S), ("tp_init", _P), ("tp_alloc", _P), ("tp_new", _P), ("tp_free", _P),
        ("tp_is_gc", _P), ("tp_bases", _P), ("tp_mro", _P), ("tp_cache", _P), ("tp_subclasses", _P),
        ("tp_weaklist", _P), ("tp_del", _P), ("tp_version_tag", ctypes.c_uint), ("tp_finalize", _P),
        ("tp_vectorcall", _P),
    ]  # fmt: skip


def route_construction_through_init(cls: type) -> None:
    """Clear `cls`'s type vectorcall so `cls(...)` calls the `__init__` in its dict.

    Raises:
        TypeLayoutError: the ctypes view does not match `cls`.
    """
    head = _TypeHead.from_address(id(cls))
    observed = (
        head.ob_type,
        head.tp_basicsize,
        head.tp_itemsize,
        head.tp_flags,
        head.tp_dictoffset,
        head.tp_weaklistoffset,
    )
    expected = (
        id(type(cls)),
        cls.__basicsize__,
        cls.__itemsize__,
        cls.__flags__,
        cls.__dictoffset__,
        cls.__weakrefoffset__,
    )
    if observed != expected:
        raise TypeLayoutError(f"{cls.__qualname__}: PyTypeObject head {observed} != {expected}")
    head.tp_vectorcall = None


def _is_nb(obj: object, kind: str) -> bool:
    return type(obj).__module__ == NANOBIND_MODULE and type(obj).__name__ == kind


@dataclass(frozen=True)
class Call:
    """One distinct recorded call site and outcome.

    Attributes:
        module: Extension package, e.g. `tesseract_environment`.
        qualname: Class-qualified name of the callable or property.
        kind: `method`, `staticmethod`, `function`, `property` or `enum`.
        index: The dispatched overload, None if undecided (or a property / enum).
        candidates: Overloads that may have accepted the call.
        fully_bound: Every parameter of `index` got an argument.
        example: Example file, relative to the example root.
        line: Line of the call in `example`.
    """

    module: str
    qualname: str
    kind: str
    index: int | None
    candidates: tuple[int, ...]
    fully_bound: bool
    example: str
    line: int


@dataclass(frozen=True)
class Instrumented:
    """One wrapped callable or property, for the report's "not instrumented" check."""

    module: str
    qualname: str
    kind: str
    overloads: int


def default_example_root() -> Path:
    """The installed `tesseract_robotics/examples` directory, located without importing it."""
    spec = importlib.util.find_spec(EXAMPLES_PACKAGE)
    if spec is None or not spec.submodule_search_locations:
        raise ExampleRootError(f"{EXAMPLES_PACKAGE} is not installed")
    return Path(next(iter(spec.submodule_search_locations)))


def extension_modules() -> list[tuple[str, ModuleType]]:
    """(package name, extension module) for every `tesseract_robotics` extension, all imported.

    Everything is imported before anything is instrumented: an extension's
    module init can read attributes of types another extension defines, and
    finds a Python wrapper where it expects a nanobind function.
    """
    package = importlib.import_module(PACKAGE)
    out = []
    for info in pkgutil.iter_modules(package.__path__):
        if not info.ispkg or f"{PACKAGE}.{info.name}" == EXAMPLES_PACKAGE:
            continue
        ext = f"{PACKAGE}.{info.name}._{info.name}"
        if importlib.util.find_spec(ext) is None:
            continue
        importlib.import_module(f"{PACKAGE}.{info.name}")
        out.append((info.name, importlib.import_module(ext)))
    return out


class Recorder:
    """Wraps the nanobind API in place and collects the calls examples make."""

    def __init__(self, example_root: Path) -> None:
        if not example_root.is_dir():
            raise ExampleRootError(f"example root {example_root} is not a directory")
        self.root = os.path.realpath(example_root) + os.sep
        self.calls: set[Call] = set()
        self.instrumented: list[Instrumented] = []
        self._is_example: dict[str, str | None] = {}

    def _example_of(self, frame: FrameType) -> str | None:
        filename = frame.f_code.co_filename
        if filename not in self._is_example:
            real = os.path.realpath(filename)
            self._is_example[filename] = (
                os.path.relpath(real, self.root) if real.startswith(self.root) else None
            )
        return self._is_example[filename]

    def _record(
        self,
        module: str,
        qualname: str,
        kind: str,
        sigs: tuple[str, ...] | None,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        frame: FrameType,
    ) -> None:
        example = self._example_of(frame)
        if example is None:
            return
        line = frame.f_lineno
        if sigs is None:
            self.calls.add(Call(module, qualname, kind, None, (), False, example, line))
        else:
            m = match_call(sigs, args, kwargs)
            self.calls.add(
                Call(module, qualname, kind, m.index, m.candidates, m.fully_bound, example, line)
            )
        for value in (*args, *kwargs.values()):
            if isinstance(value, enum.Enum) and type(value).__module__.startswith(PACKAGE):
                enum_module = type(value).__module__.split(".")[1]
                self.calls.add(
                    Call(
                        enum_module,
                        type(value).__qualname__,
                        "enum",
                        None,
                        (),
                        False,
                        example,
                        line,
                    )
                )

    def _wrap(
        self,
        module: str,
        qualname: str,
        kind: str,
        orig: Any,
        with_overloads: bool = True,
    ) -> Callable[..., Any]:
        sigs = tuple(s[0] for s in orig.__nb_signature__) if with_overloads else None
        record = self._record

        def wrapper(*args: Any, **kwargs: Any) -> Any:
            record(module, qualname, kind, sigs, args, kwargs, sys._getframe(1))
            return orig(*args, **kwargs)

        # functools.wraps would fail: a property accessor's `__qualname__` is not a str.
        for attr in WRAPPED_METADATA:
            value = getattr(orig, attr, None)
            if isinstance(value, str):
                setattr(wrapper, attr, value)
        return wrapper

    def _instrument_class(self, module: str, cls: type) -> None:
        for name, attr in list(vars(cls).items()):
            qualname = f"{cls.__qualname__}.{name}"
            if _is_nb(attr, NB_METHOD):
                kind, new = "method", self._wrap(module, qualname, "method", attr)
                n = len(attr.__nb_signature__)
            elif _is_nb(attr, NB_FUNC) or (
                isinstance(attr, staticmethod) and _is_nb(attr.__func__, NB_FUNC)
            ):
                func = attr.__func__ if isinstance(attr, staticmethod) else attr
                kind, new = (
                    "staticmethod",
                    staticmethod(self._wrap(module, qualname, "staticmethod", func)),
                )
                n = len(func.__nb_signature__)
            elif isinstance(attr, property) and (
                _is_nb(attr.fget, NB_METHOD) or _is_nb(attr.fset, NB_METHOD)
            ):
                fget, fset = (
                    self._wrap(module, qualname, "property", f, with_overloads=False)
                    if _is_nb(f, NB_METHOD)
                    else f
                    for f in (attr.fget, attr.fset)
                )
                kind, new, n = "property", property(fget, fset, attr.fdel, attr.__doc__), 0
            else:
                continue
            setattr(cls, name, new)
            if name == CTOR:
                route_construction_through_init(cls)
            self.instrumented.append(Instrumented(module, qualname, kind, n))

    def install(self) -> None:
        """Wrap every nanobind callable and property of every extension module."""
        replaced: dict[int, Callable[..., Any]] = {}
        for package, ext in extension_modules():
            classes: list[type] = []
            pending = [
                v
                for v in vars(ext).values()
                if isinstance(v, type) and v.__module__ == ext.__name__
            ]
            while pending:
                cls = pending.pop()
                if cls in classes or issubclass(cls, enum.Enum):
                    continue
                classes.append(cls)
                pending += [
                    v
                    for v in vars(cls).values()
                    if isinstance(v, type) and v.__module__ == ext.__name__
                ]
            for cls in classes:
                self._instrument_class(package, cls)
            for name, obj in list(vars(ext).items()):
                if _is_nb(obj, NB_FUNC):
                    replaced[id(obj)] = self._wrap(package, name, "function", obj)
                    self.instrumented.append(
                        Instrumented(package, name, "function", len(obj.__nb_signature__))
                    )
        # Every namespace that re-exports a module function (`from ._x import *`) gets the wrapper.
        for mod_name, mod in list(sys.modules.items()):
            if (
                not mod_name.startswith(PACKAGE)
                or mod_name.startswith(EXAMPLES_PACKAGE)
                or mod is None
            ):
                continue
            for name, obj in list(vars(mod).items()):
                if id(obj) in replaced and _is_nb(obj, NB_FUNC):
                    setattr(mod, name, replaced[id(obj)])

    def dump(self, path: Path) -> None:
        """Write the calls and the instrumented manifest as JSON."""
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "example_root": self.root,
            "instrumented": [asdict(i) for i in self.instrumented],
            "calls": [
                asdict(c)
                for c in sorted(
                    self.calls,
                    key=lambda c: (c.module, c.qualname, c.example, c.line, str(c.index)),
                )
            ],
        }
        path.write_text(json.dumps(payload, indent=1) + "\n")


_RECORDER = pytest.StashKey[Recorder]()


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("api-recorder", "example API coverage recorder")
    group.addoption(
        "--api-record-dir",
        type=Path,
        default=DEFAULT_RECORD_DIR,
        help="directory for <worker>.json recordings",
    )
    group.addoption(
        "--api-record-example-root",
        type=Path,
        default=None,
        help="calls from files under this directory are recorded (default: tesseract_robotics/examples)",
    )


def pytest_configure(config: pytest.Config) -> None:
    root = config.getoption("--api-record-example-root") or default_example_root()
    recorder = Recorder(root)
    recorder.install()
    config.stash[_RECORDER] = recorder


def pytest_unconfigure(config: pytest.Config) -> None:
    recorder = config.stash.get(_RECORDER, None)
    if recorder is None:
        return
    worker = os.environ.get("PYTEST_XDIST_WORKER", SERIAL_WORKER)
    recorder.dump(config.getoption("--api-record-dir") / f"{worker}.json")
