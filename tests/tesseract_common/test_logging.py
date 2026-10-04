"""Does the Python logging API control tesseract's own output?

Upstream tesseract (dc289659, #1367) logs through spdlog instead of console_bridge, so the
console_bridge bindings (`setLogLevel`, `useOutputHandler`, ...) no longer reach it. The
spdlog-backed API mirrors upstream: `getLogger(name)` for the level, `addLogRecordHandler`
for custom sinks.
"""

import subprocess
import sys

import pytest

from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_common import LoggerLevel

# GeneralResourceLocator logs TESSERACT_LOG_WARN("Resource not handled: ...") for a
# relative path and returns None.
UNHANDLED = "relative/not/handled.stl"


def _warn_once():
    assert tesseract_common.GeneralResourceLocator().locateResource(UNHANDLED) is None


@pytest.fixture
def logger():
    logger = tesseract_common.getLogger()
    previous = logger.level()
    yield logger
    logger.set_level(previous)


def test_log_level_silences_tesseract(capfd, logger):
    logger.set_level(LoggerLevel.off)
    _warn_once()
    _, err = capfd.readouterr()
    assert "Resource not handled" not in err


def test_default_logger_is_named_tesseract_and_round_trips_its_level(logger):
    assert logger.name() == "tesseract"
    logger.set_level(LoggerLevel.err)
    assert tesseract_common.getLogger().level() == LoggerLevel.err
    assert not tesseract_common.isLogLevelEnabled(LoggerLevel.warn)
    assert tesseract_common.isLogLevelEnabled(LoggerLevel.critical)


def test_record_handler_receives_a_tesseract_warning(logger):
    logger.set_level(LoggerLevel.warn)
    records = []
    handler_id = tesseract_common.addLogRecordHandler(records.append)
    try:
        _warn_once()
    finally:
        assert tesseract_common.removeLogRecordHandler(handler_id)

    (record,) = [r for r in records if UNHANDLED in r.message]
    # the record is a copy: still readable after the C++ call returned
    assert record.level == LoggerLevel.warn
    assert record.logger_name == "tesseract"
    assert record.message == f"Resource not handled: {UNHANDLED}"
    assert record.filename.endswith(".cpp")
    assert record.line > 0


def test_removed_handler_receives_nothing(logger):
    logger.set_level(LoggerLevel.warn)
    records = []
    handler_id = tesseract_common.addLogRecordHandler(records.append)
    assert tesseract_common.removeLogRecordHandler(handler_id)
    assert not tesseract_common.removeLogRecordHandler(handler_id)
    _warn_once()
    assert records == []


def test_handler_exception_reaches_unraisablehook(monkeypatch, logger):
    logger.set_level(LoggerLevel.warn)
    reported = []
    monkeypatch.setattr(sys, "unraisablehook", reported.append)

    def broken(record):
        raise RuntimeError("handler bug")

    handler_id = tesseract_common.addLogRecordHandler(broken)
    try:
        _warn_once()
    finally:
        tesseract_common.removeLogRecordHandler(handler_id)

    assert [type(u.exc_value) for u in reported] == [RuntimeError]
    assert str(reported[0].exc_value) == "handler bug"


def test_a_handler_left_registered_is_released_before_interpreter_exit():
    script = (
        "from tesseract_robotics import tesseract_common as tc\n"
        "records = []\n"
        "tc.addLogRecordHandler(records.append)\n"
        f"tc.GeneralResourceLocator().locateResource({UNHANDLED!r})\n"
        "assert len(records) == 1\n"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "leaked" not in result.stderr
