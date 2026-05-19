"""tesseract — C++ binding namespace.

This file's sole job is to import `tesseract_robotics` so its top-of-module
side effects run before any sibling binding module is imported. Specifically:

- Windows: registers `os.add_dll_directory()` for the package root and
  `tesseract_robotics_nanobind.libs/` so the .pyd's transitive DLL deps
  (tesseract C++ libs, boost, etc.) resolve. Python 3.8+ doesn't honor
  PATH for .pyd loads.
- Linux/macOS: ensures plugin search paths and bundled-data env vars are
  configured before any binding is touched (also handles the editable-install
  ws/src layout fallback).

Without this, `from tesseract.tesseract_X import Y` would skip the env-setup
code that lives in `tesseract_robotics/__init__.py` and load .pyd files
with their DLL deps unresolved on Windows.

Functionally this makes `tesseract` a regular package rather than a PEP-420
namespace package — a cheap structural tradeoff for cross-platform binding
robustness.
"""

import tesseract_robotics  # noqa: F401
