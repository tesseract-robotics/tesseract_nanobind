"""gh-176: no module attribute is a member of a bound enum (the SWIG-era `Enum_VALUE` copies).

The enum class is the only way to a value: `ContactTestType.ALL`, `LogLevel.CONSOLE_BRIDGE_LOG_DEBUG`.
"""

import enum
import importlib

import pytest

AUDITED_MODULES = (
    "tesseract_common",
    "tesseract_collision",
    "tesseract_environment",
    "tesseract_scene_graph",
)


@pytest.mark.parametrize(
    "qualname",
    [f"tesseract_robotics.{m}{ext}" for m in AUDITED_MODULES for ext in ("", f"._{m}")],
)
def test_no_enum_alias_constants(qualname):
    module = importlib.import_module(qualname)
    aliases = sorted(name for name, value in vars(module).items() if isinstance(value, enum.Enum))
    assert aliases == []
