"""Robot models and meshes from tesseract's support package, for tesseract_robotics.

`data_dir()` holds `tesseract/package.xml` and `tesseract/support/`: the layout a
`GeneralResourceLocator` needs on `TESSERACT_RESOURCE_PATH` to resolve
`package://tesseract/support/...`.
"""

from importlib.resources import files
from pathlib import Path

__all__ = ["data_dir", "support_dir"]


def data_dir() -> Path:
    """Directory holding `tesseract/package.xml` and `tesseract/support/`.

    The C++ resource locator opens these files by path, so the package must be installed as a
    directory (as pip installs wheels), not imported from a zip archive.

    Raises:
        RuntimeError: the package's data is not on the filesystem.
    """
    data = files(__name__) / "data"
    if not isinstance(data, Path) or not data.is_dir():
        raise RuntimeError(
            f"tesseract_robotics_support data is not a directory on disk ({data!r}); "
            "install the wheel unzipped"
        )
    return data


def support_dir() -> Path:
    """tesseract's `support/` directory (urdf/, meshes/, sdf/): what `TESSERACT_SUPPORT_DIR` names."""
    return data_dir() / "tesseract" / "support"
