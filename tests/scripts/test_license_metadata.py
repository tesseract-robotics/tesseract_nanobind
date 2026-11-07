"""Contract test on the license the built distribution declares (gh-368).

pip and PyPI read the PEP 639 `License-Expression` and `License-File` fields of
the wheel's METADATA, not the trove classifier. scikit-build-core writes that
METADATA without compiling, so the check runs everywhere the build backend is
installed.
"""

from email.parser import Parser
from pathlib import Path

import pytest
from scikit_build_core.build import prepare_metadata_for_build_wheel

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def test_metadata_declares_apache_2(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(REPO_ROOT)
    dist_info = tmp_path / prepare_metadata_for_build_wheel(str(tmp_path))
    metadata = Parser().parsestr((dist_info / "METADATA").read_text())

    assert metadata["License-Expression"] == "Apache-2.0"
    assert sorted(metadata.get_all("License-File") or []) == ["LICENSE", "NOTICE"]
    for name in ("LICENSE", "NOTICE"):
        shipped = (dist_info / "licenses" / name).read_text()
        assert shipped == (REPO_ROOT / name).read_text()
