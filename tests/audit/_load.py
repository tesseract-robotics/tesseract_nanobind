"""Load the audit scripts as modules for the contract tests, in import order."""

import importlib.util
import sys
from collections.abc import Iterable
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
# Import order: each script imports only the ones before it.
SCRIPTS = (
    "audit_types",
    "audit_ownership",
    "audit_parse",
    "audit_bound",
    "audit_stub",
    "audit_rules",
    "audit_match",
    "audit_report",
    "audit_bindings",
)


class DuplicateNameError(RuntimeError):
    """Two scripts define one name with different objects: a move left a stale copy."""


def load(name: str) -> ModuleType:
    """The script `scripts/<name>.py` as module `name`, loaded once per test session."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


def combine(modules: Iterable[ModuleType]) -> ModuleType:
    """One namespace holding every public and private name of `modules`.

    Raises:
        DuplicateNameError: two modules bind one name to different objects.
    """
    out = ModuleType("audit")
    for m in modules:
        for k, v in vars(m).items():
            if k.startswith("__"):
                continue
            old = vars(out).get(k, v)
            if old is not v and old != v:  # functions and classes compare by identity
                raise DuplicateNameError(f"{k!r} is defined in {m.__name__} and elsewhere")
            setattr(out, k, v)
    return out


audit = combine(load(n) for n in SCRIPTS)
