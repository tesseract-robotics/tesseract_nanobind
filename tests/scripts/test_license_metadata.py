"""Contract test on the license the installed distribution declares (gh-368).

pip and PyPI read the PEP 639 `License-Expression` and `License-File` fields of
the wheel's METADATA, not the trove classifier. The test reads them from the
installed distribution, so test-wheel checks the wheel it installed and the dev
env checks its editable install, with no build backend in either env.
"""

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
