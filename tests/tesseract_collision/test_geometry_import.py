"""gh-168: the collision extension imports tesseract_geometry itself.

Stub generation imports each extension in its own process, so the geometry types
in collision signatures resolve only if the extension imports their module.
"""

import subprocess
import sys

_SCRIPT = """
import sys
import tesseract_robotics.tesseract_collision._tesseract_collision
print("GEOMETRY_LOADED" if "tesseract_robotics.tesseract_geometry._tesseract_geometry" in sys.modules else "GEOMETRY_MISSING")
"""


def test_collision_extension_imports_geometry():
    """Subprocess: nothing else in this process may have imported tesseract_geometry first."""
    proc = subprocess.run(
        [sys.executable, "-c", _SCRIPT],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode}: {proc.stderr[-800:]}"
    assert "GEOMETRY_LOADED" in proc.stdout, proc.stdout
