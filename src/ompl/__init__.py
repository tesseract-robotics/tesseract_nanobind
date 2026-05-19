"""ompl — C++ binding namespace for OMPL state spaces.

Mirrors `tesseract/__init__.py`: imports `tesseract_robotics` so its
top-of-module side effects (Windows os.add_dll_directory, plugin paths,
TESSERACT_SUPPORT_DIR, etc.) run before any sibling binding is touched.

See `tesseract/__init__.py` for the rationale.
"""

import tesseract_robotics  # noqa: F401
