"""`get_tesseract_resource_path`: the [support] extra is optional, a broken one is an error."""

import importlib
import importlib.util

import pytest

import tesseract_robotics
from tesseract_robotics import (
    SUPPORT_PACKAGE,
    SupportDataNotFoundError,
    get_tesseract_resource_path,
)


def test_without_the_extra_or_a_prefix_it_raises_by_name(monkeypatch):
    real_find_spec = importlib.util.find_spec
    monkeypatch.setattr(
        importlib.util,
        "find_spec",
        lambda name, *a: None if name == SUPPORT_PACKAGE else real_find_spec(name, *a),
    )
    monkeypatch.delenv("TESSERACT_CPP_PREFIX", raising=False)
    with pytest.raises(SupportDataNotFoundError, match=r"\[support\]"):
        get_tesseract_resource_path()


def test_a_broken_support_install_is_not_swallowed(monkeypatch):
    class BrokenInstallError(ImportError):
        pass

    real_find_spec = importlib.util.find_spec
    monkeypatch.setattr(
        importlib.util,
        "find_spec",
        lambda name, *a: object() if name == SUPPORT_PACKAGE else real_find_spec(name, *a),
    )
    real_import = importlib.import_module

    def import_module(name, *a):
        if name == SUPPORT_PACKAGE:
            raise BrokenInstallError(name)
        return real_import(name, *a)

    monkeypatch.setattr(importlib, "import_module", import_module)
    with pytest.raises(BrokenInstallError):
        tesseract_robotics.get_tesseract_resource_path()
