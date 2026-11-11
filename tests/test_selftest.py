"""The installed-package selftest needs neither the test suite nor pytest."""

import subprocess
import sys

import pytest

from tesseract_robotics import SupportDataNotFoundError, get_tesseract_resource_path, selftest


def test_selftest_does_not_import_pytest_or_look_for_tests():
    source = selftest.__loader__.get_source("tesseract_robotics.selftest")
    assert "import pytest" not in source
    assert '"tests"' not in source


def test_selftest_runs_clean_without_pytest(tmp_path):
    """A subprocess with pytest unimportable passes every check and writes its report."""
    code = (
        "import sys; sys.modules['pytest'] = None; "
        "from tesseract_robotics.selftest import main; sys.exit(main())"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code], cwd=tmp_path, capture_output=True, text=True, timeout=600
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    report = (tmp_path / selftest.REPORT_NAME).read_text()
    assert report.count("PASS") == len(selftest.CHECKS)
    assert "FAIL" not in report


def test_failing_check_is_reported_with_its_traceback():
    def broken() -> str:
        raise selftest.SelftestCheckError("broken on purpose")

    result = selftest.run_check("broken", broken)
    assert not result.passed
    assert "broken on purpose" in result.detail
    report = selftest.build_report([result], {"k": "v"})
    assert "FAIL  broken" in report
    assert "GITHUB ISSUE TEMPLATE" in report


def test_support_data_path_raises_when_the_support_package_is_missing(monkeypatch):
    monkeypatch.setitem(sys.modules, "tesseract_robotics_support", None)
    with pytest.raises(SupportDataNotFoundError, match="support"):
        get_tesseract_resource_path()


def test_version_is_the_installed_distribution_version():
    from importlib.metadata import version

    import tesseract_robotics

    assert tesseract_robotics.__version__ == version("tesseract-robotics-nanobind")
    assert tesseract_robotics.__version__ != "0.0.0.dev"
