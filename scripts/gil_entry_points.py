"""scripts/nogil_entry_points.yaml as data: the bindings that release the GIL, and why (#124).

scripts/audit_gil.py checks the list against the binding sources; scripts/gil_hotspots.py
leaves listed bindings out of its ranking.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml

ENTRY_POINTS = Path(__file__).resolve().parent / "nogil_entry_points.yaml"

# The scope of a module-level `m.def`.
MODULE_SCOPE = "module"


class EntryPointsError(ValueError):
    """nogil_entry_points.yaml is malformed."""


@dataclass(frozen=True, order=True)
class Release:
    """One bound name that releases the GIL: Python module, class (or `module`) and name."""

    module: str
    scope: str
    name: str

    def __str__(self) -> str:
        return f"{self.module}: {self.scope}.{self.name}"


def load_entry_points(path: Path) -> dict[Release, str]:
    """`{module: {scope: {name: reason}}}` -> `{Release: reason}`."""
    data = yaml.safe_load(path.read_text()) or {}
    out: dict[Release, str] = {}
    if not isinstance(data, Mapping):
        raise EntryPointsError(f"{path}: expected a mapping of modules")
    for module, scopes in data.items():
        if not isinstance(scopes, Mapping):
            raise EntryPointsError(f"{path}: {module}: expected a mapping of classes")
        for scope, names in scopes.items():
            if not isinstance(names, Mapping):
                raise EntryPointsError(f"{path}: {module}.{scope}: expected a mapping of names")
            for name, reason in names.items():
                if not isinstance(reason, str) or not reason.strip():
                    raise EntryPointsError(f"{path}: {module}: {scope}.{name}: needs a reason")
                out[Release(str(module), str(scope), str(name))] = reason
    return out
