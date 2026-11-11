"""Copy tesseract's support data from an installed prefix into the wheel (see pyproject.toml)."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py


def _tesseract_share() -> Path:
    """`<prefix>/share/tesseract` of the tesseract install the wheel is built against."""
    prefix = os.environ.get("TESSERACT_CPP_PREFIX")
    if not prefix:
        raise RuntimeError(
            "tesseract support data not found: set TESSERACT_CPP_PREFIX to the native stack's "
            "install (share/tesseract/package.xml and share/tesseract/support/)"
        )
    candidate = Path(prefix) / "share" / "tesseract"
    if not ((candidate / "package.xml").is_file() and (candidate / "support").is_dir()):
        raise RuntimeError(f"tesseract support data not found under {candidate}")
    return candidate


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
