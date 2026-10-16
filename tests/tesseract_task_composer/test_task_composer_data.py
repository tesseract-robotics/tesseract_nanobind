"""TaskComposerKeys, TaskComposerDataStorage and AnyPoly: the data a task-composer run reads."""

import subprocess
import sys

import pytest

from tesseract_robotics import tesseract_task_composer
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly,
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    TaskComposerDataStorage,
    TaskComposerKeys,
)

# The wrappers whose signatures name types bound outside tesseract_common (#312).
_TYPED_WRAPPERS = (
    "AnyPoly_wrap_CompositeInstruction",
    "AnyPoly_wrap_ProfileDictionary",
    "AnyPoly_wrap_EnvironmentConst",
    "AnyPoly_as_CompositeInstruction",
    "AnyPoly_as_ContactResultMapVector",
)


def _ci(profile: str) -> AnyPoly:
    return AnyPoly_wrap_CompositeInstruction(CompositeInstruction(profile))


class TestTaskComposerKeys:
    """The list overload of `add`, `remove`, `rename`, `__str__`, and `get` of a list (#313)."""

    def test_add_list_then_get_returns_list(self):
        keys = TaskComposerKeys()
        keys.add("p", ["a", "b"])
        assert keys.get("p") == ["a", "b"]

    def test_add_str_then_get_returns_str(self):
        keys = TaskComposerKeys()
        keys.add("q", "k")
        assert keys.get("q") == "k"

    def test_remove(self):
        keys = TaskComposerKeys()
        keys.add("p", ["a", "b"])
        keys.remove("p")
        assert keys.has("p") is False

    def test_rename(self):
        """`rename` maps old *key* -> new key in every port; the ports stay (upstream
        task_composer_keys.cpp)."""
        keys = TaskComposerKeys()
        keys.add("q", "k")
        keys.add("p", ["k", "other"])
        keys.rename({"k": "r"})
        assert keys.get("q") == "r"
        assert keys.get("p") == ["r", "other"]

    def test_str_lists_every_port(self):
        keys = TaskComposerKeys()
        keys.add("single_port", "k")
        keys.add("list_port", ["a", "b"])
        text = str(keys)
        assert "single_port" in text
        assert "list_port" in text

    def test_add_keys_is_gone(self):
        """`addKeys` was the list overload of `add` under another name (#313)."""
        assert not hasattr(TaskComposerKeys, "addKeys")


class TestTaskComposerDataStorage:
    """`getData()`, `remapData`, `copyAs{Input,Output}Data` and the `name` argument (#314)."""

    def test_name_constructor(self):
        assert TaskComposerDataStorage("n").getName() == "n"

    def test_get_data_returns_every_key(self):
        storage = TaskComposerDataStorage()
        for key in ("a", "b", "c"):
            storage.setData(key, _ci(key.upper()))
        data = storage.getData()
        assert set(data) == {"a", "b", "c"}
        assert AnyPoly_as_CompositeInstruction(data["b"]).getProfile() == "B"

    def test_get_data_of_empty_storage(self):
        assert TaskComposerDataStorage().getData() == {}

    def test_remap_moves_entry(self):
        storage = TaskComposerDataStorage()
        storage.setData("a", _ci("A"))
        assert storage.remapData({"a": "b"}) is True
        assert not storage.hasKey("a")
        assert AnyPoly_as_CompositeInstruction(storage.getData("b")).getProfile() == "A"

    def test_remap_copy_keeps_both(self):
        storage = TaskComposerDataStorage()
        storage.setData("a", _ci("A"))
        assert storage.remapData({"a": "b"}, copy=True) is True
        assert storage.hasKey("a")
        assert storage.hasKey("b")

    def test_remap_missing_key_returns_false(self):
        assert TaskComposerDataStorage().remapData({"missing": "b"}) is False

    def test_copy_as_input_data_honours_override(self):
        """this[key] = parent[override key, else key]."""
        parent = TaskComposerDataStorage()
        parent.setData("plain", _ci("PLAIN"))
        parent.setData("parent_key", _ci("OVERRIDDEN"))
        keys = TaskComposerKeys()
        keys.add("p1", "plain")
        keys.add("p2", "local_key")
        override = TaskComposerKeys()
        override.add("p2", "parent_key")

        local = TaskComposerDataStorage()
        local.copyAsInputData(parent, keys, override)

        assert set(local.getData()) == {"plain", "local_key"}
        assert AnyPoly_as_CompositeInstruction(local.getData("local_key")).getProfile() == (
            "OVERRIDDEN"
        )

    def test_copy_as_output_data_honours_override(self):
        """this[override key, else key] = child[key]."""
        child = TaskComposerDataStorage()
        child.setData("plain", _ci("PLAIN"))
        child.setData("local_key", _ci("OUT"))
        keys = TaskComposerKeys()
        keys.add("p1", "plain")
        keys.add("p2", "local_key")
        override = TaskComposerKeys()
        override.add("p2", "parent_key")

        parent = TaskComposerDataStorage()
        parent.copyAsOutputData(child, keys, override)

        assert set(parent.getData()) == {"plain", "parent_key"}
        assert AnyPoly_as_CompositeInstruction(parent.getData("parent_key")).getProfile() == "OUT"

    def test_get_all_data_is_gone(self):
        """`getAllData` was the zero-argument `getData` under another name (#314)."""
        assert not hasattr(TaskComposerDataStorage, "getAllData")


class TestRemovedFactoryFunctions:
    """The two module functions that duplicated constructors are gone (#338)."""

    @pytest.mark.parametrize(
        "name", ["createTaskComposerPluginFactory", "createTaskComposerDataStorage"]
    )
    def test_not_a_module_attribute(self, name):
        assert not hasattr(tesseract_task_composer, name)

    def test_package_has_no_all(self):
        """The star import re-exports the extension module; no hand-kept `__all__` (#338)."""
        assert not hasattr(tesseract_task_composer, "__all__")


class TestAnyPoly:
    """Demangled type names (#322) and Python type names in the wrapper signatures (#312)."""

    def test_type_name_is_demangled(self):
        assert _ci("X").getTypeName() == "tesseract::command_language::CompositeInstruction"

    def test_null_type_name(self):
        assert AnyPoly().getTypeName() == "null"

    def test_signatures_name_python_types(self):
        """In a fresh process that imports only the task composer, the wrappers' signatures
        name the Python types, not quoted C++ names (#312)."""
        script = (
            "from tesseract_robotics.tesseract_task_composer import _tesseract_task_composer as m\n"
            "for name in " + repr(_TYPED_WRAPPERS) + ":\n"
            "    print(name, getattr(m, name).__doc__.splitlines()[0])\n"
        )
        proc = subprocess.run(
            [sys.executable, "-c", script], capture_output=True, text=True, check=True
        )
        lines = proc.stdout.splitlines()
        assert len(lines) == len(_TYPED_WRAPPERS)
        assert not [line for line in lines if "tesseract::" in line]


class TestValueEquality:
    """`operator==` bound as `__eq__`/`__ne__`; the types are mutable, so unhashable (#261)."""

    def test_keys(self):
        a, b = TaskComposerKeys(), TaskComposerKeys()
        a.add("p", "k")
        b.add("p", "k")
        assert a == b
        b.add("p", "other")
        assert a != b
        with pytest.raises(TypeError):
            hash(a)

    def test_data_storage(self):
        a, b = TaskComposerDataStorage("n"), TaskComposerDataStorage("n")
        assert a == b
        b.setName("m")
        assert a != b
        with pytest.raises(TypeError):
            hash(a)

    def test_any_poly(self):
        program = CompositeInstruction("X")
        a = AnyPoly_wrap_CompositeInstruction(program)
        assert a == AnyPoly_wrap_CompositeInstruction(program)
        assert a != _ci("X")  # a separately built instruction has its own uuid
        assert AnyPoly() == AnyPoly()
        with pytest.raises(TypeError):
            hash(a)
