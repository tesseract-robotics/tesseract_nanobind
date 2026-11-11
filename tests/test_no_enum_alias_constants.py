"""gh-176: no module attribute is a member of a bound enum (the SWIG-era `Enum_VALUE` copies).

The enum class is the only way to a value: `ContactTestType.ALL`, `LogLevel.debug`.
"""

import enum
import importlib

import pytest

# Modules with a Python package around the compiled `_<name>` extension
WRAPPED = ("common", "scene_graph")
DIRECT = ("collision", "environment", "command_language")


@pytest.mark.parametrize(
    "qualname",
    [f"tesseract_robotics.{m}{ext}" for m in WRAPPED for ext in ("", f"._{m}")]
    + [f"tesseract_robotics.{m}" for m in DIRECT],
)
def test_no_enum_alias_constants(qualname):
    module = importlib.import_module(qualname)
    aliases = sorted(name for name, value in vars(module).items() if isinstance(value, enum.Enum))
    assert aliases == []
