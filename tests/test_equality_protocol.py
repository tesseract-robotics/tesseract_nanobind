"""Every bound `__eq__` / `__ne__` follows Python's comparison protocol.

`x == y` must not raise for an operand of another type: the operator returns NotImplemented, and
Python falls back to identity. The signature therefore accepts any object, as `object.__eq__`
does; a narrower one in the stub is an invalid override for a type checker.
"""

from __future__ import annotations

import importlib
import importlib.machinery
import importlib.util
import pkgutil
from collections.abc import Iterator

import pytest

import tesseract_robotics
from tesseract_robotics.common import JointId, LinkId, LinkIdPair
from tesseract_robotics.geometry import Box, Sphere
from tesseract_robotics.scene_graph import JointLimits, SceneGraph
from tesseract_robotics.srdf import SRDFModel

OPERATORS = ("__eq__", "__ne__")


def _extension_modules() -> Iterator[str]:
    for info in pkgutil.walk_packages(tesseract_robotics.__path__, "tesseract_robotics."):
        if info.ispkg:
            continue
        spec = importlib.util.find_spec(info.name)
        if spec is not None and isinstance(spec.loader, importlib.machinery.ExtensionFileLoader):
            yield info.name


def _bound_operators() -> list[tuple[str, str]]:
    found = []
    for name in sorted(_extension_modules()):
        module = importlib.import_module(name)
        for cls in vars(module).values():
            if not isinstance(cls, type) or cls.__module__ != name:
                continue
            for op in OPERATORS:
                fn = cls.__dict__.get(op)
                if fn is not None and hasattr(fn, "__nb_signature__"):
                    found.append((f"{cls.__qualname__}.{op}", fn))
    return found


BOUND = _bound_operators()


def test_bound_operators_found() -> None:
    assert len(BOUND) > 100, f"expected the 140+ bound comparison operators, found {len(BOUND)}"


@pytest.mark.parametrize(("name", "fn"), BOUND, ids=[name for name, _ in BOUND])
def test_operand_is_object(name: str, fn: object) -> None:
    op = name.rsplit(".", 1)[1]
    signatures = [sig for sig, _, _ in fn.__nb_signature__]  # type: ignore[attr-defined]
    assert signatures == [f"def {op}(self, other: object, /) -> bool"], name


@pytest.mark.parametrize(
    "value",
    [Box(1, 1, 1), Sphere(1), JointLimits(), SceneGraph(), SRDFModel(), LinkId("a"), JointId("a")],
    ids=lambda v: type(v).__name__,
)
def test_foreign_operand_is_unequal(value: object) -> None:
    assert (value == 5) is False
    assert (value != 5) is True
    assert value in [5, value]


def test_link_id_pair_refuses_tuple() -> None:
    assert (LinkIdPair("a", "b") == ("a", "b")) is False


@pytest.mark.parametrize("cls", [LinkId, JointId])
def test_id_compares_with_id_and_name(cls: type[LinkId] | type[JointId]) -> None:
    assert cls("a") == cls("a")
    assert cls("a") == "a"
    assert "a" == cls("a")
    assert cls("a") != cls("b")
    assert cls("a") != "b"
    assert (cls("a") != "a") is False
