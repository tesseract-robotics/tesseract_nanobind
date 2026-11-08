"""Copy tesseract's support data from an installed prefix into the wheel (see pyproject.toml)."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py


def _tesseract_share() -> Path:
    """`<prefix>/share/tesseract` of the tesseract install the wheel is built against."""
    prefixes = [os.environ.get(var) for var in ("TESSERACT_CPP_PREFIX", "CONDA_PREFIX")]
    tried = []
    for prefix in filter(None, prefixes):
        # conda on Windows installs data under the Library/ prefix
        for share in (Path(prefix) / "share", Path(prefix) / "Library" / "share"):
            candidate = share / "tesseract"
            tried.append(str(candidate))
            if (candidate / "package.xml").is_file() and (candidate / "support").is_dir():
                return candidate
    raise RuntimeError(
        "tesseract support data not found: set TESSERACT_CPP_PREFIX (or activate a conda env) to a "
        "tesseract install with share/tesseract/package.xml and share/tesseract/support/"
        + (f"; tried {', '.join(tried)}" if tried else "")
    )


class BuildPyWithSupportData(build_py):
    """build_py that adds data/tesseract/{package.xml,support/} to the built package."""

    def run(self) -> None:
        super().run()
        share = _tesseract_share()
        dest = Path(self.build_lib) / "tesseract_robotics_support" / "data" / "tesseract"
        if dest.exists():
            shutil.rmtree(dest)
        dest.mkdir(parents=True)
        shutil.copy2(share / "package.xml", dest / "package.xml")
        shutil.copytree(share / "support", dest / "support")
        self.announce(f"copied tesseract support data from {share}", level=2)


setup(cmdclass={"build_py": BuildPyWithSupportData})
