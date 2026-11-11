"""Load the audit scripts that need no libclang, for the contract tests that run in any env.

`_load` imports every script, libclang included; the tests of the cache, its key, its
telemetry and the module -> sources table import this module instead. Both register a script in `sys.modules` under its own
name, so a session that uses both (the audit env) holds one copy of each.
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

SCRIPTS_DIR = Path(__file__).resolve().parent.parent.parent / "scripts"
# Import order: each script imports only the ones before it.
FREE_SCRIPTS = (
    "audit_ownership",
    "audit_telemetry",
    "audit_toolchain",
    "audit_cache",
    "audit_symbols",
)
FREE_CLIS = ("audit_cache_prune", "audit_telemetry_report")


class NotLibclangFreeError(LookupError):
    """The script is not one of `FREE_SCRIPTS` or `FREE_CLIS`: load it through `_load`."""


def load_free(name: str) -> ModuleType:
    """The script `scripts/<name>.py` as module `name`, with the free scripts it imports.

    Raises:
        NotLibclangFreeError: `name` is not a libclang-free script.
    """
    if name not in (*FREE_SCRIPTS, *FREE_CLIS):
        raise NotLibclangFreeError(name)
    for script in FREE_SCRIPTS:
        _load(script)
    return _load(name)


def _load(name: str) -> ModuleType:
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SCRIPTS_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module
