"""Contract test on the license the installed distribution declares (gh-368).

pip and PyPI read the PEP 639 `License-Expression` and `License-File` fields of
the wheel's METADATA, not the trove classifier. The test reads them from the
installed distribution, so test-wheel checks the wheel it installed and the dev
env checks its editable install, with no build backend in either env.
"""

import tomllib
from importlib.metadata import distribution
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DISTRIBUTION = "tesseract-robotics-nanobind"
LICENSE_FILES = ("LICENSE", "NOTICE")


def test_metadata_declares_apache_2() -> None:
    dist = distribution(DISTRIBUTION)
    metadata = dist.metadata

    assert metadata["License-Expression"] == "Apache-2.0"
    assert sorted(metadata.get_all("License-File") or []) == list(LICENSE_FILES)
    shipped = {
        Path(str(f)).name: f for f in dist.files or [] if Path(str(f)).parent.name == "licenses"
    }
    assert sorted(shipped) == list(LICENSE_FILES)
    for name in LICENSE_FILES:
        assert shipped[name].read_text() == (REPO_ROOT / name).read_text()


SUPPORT_PACKAGE = REPO_ROOT / "packaging" / "tesseract-robotics-nanobind-support"
# Each SPDX id in the support wheel's expression ships as `licenses/LICENSE.<id>`, the
# names tesseract's own repository uses for the same texts.
SUPPORT_LICENSE_DIR = "licenses"


def test_support_wheel_ships_the_license_texts_it_declares() -> None:
    project = tomllib.loads((SUPPORT_PACKAGE / "pyproject.toml").read_text())["project"]

    declared = project["license"].split(" AND ")
    assert declared == ["Apache-2.0", "BSD-3-Clause"]
    expected = {f"{SUPPORT_LICENSE_DIR}/LICENSE.{spdx}" for spdx in declared}

    # PEP 639 `license-files` globs are relative to the project root
    listed = {
        path.relative_to(SUPPORT_PACKAGE).as_posix()
        for pattern in project.get("license-files", [])
        for path in SUPPORT_PACKAGE.glob(pattern)
    }
    assert listed == expected
    for relative in expected:
        assert (SUPPORT_PACKAGE / relative).read_text().strip()
