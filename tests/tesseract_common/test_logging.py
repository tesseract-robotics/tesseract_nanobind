"""Does the Python logging API control tesseract's own output?

Upstream tesseract (dc289659, #1367) logs through spdlog instead of console_bridge, so the
console_bridge bindings (`setLogLevel`, `useOutputHandler`, ...) were removed: they no longer
reached it. The spdlog-backed API mirrors upstream: `getLogger(name)` for the logger, its level
and its sinks (`StderrSink`, `PythonLoggingSink`), `addLogRecordHandler` for a callable.
"""

import subprocess
import sys

import pytest

from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_common import LogLevel

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
    logger.set_level(LogLevel.off)
    _warn_once()
    _, err = capfd.readouterr()
    assert "Resource not handled" not in err


@pytest.fixture
def sinks(logger):
    """The default logger's sinks, restored after the test."""
    previous = logger.sinks
    yield previous
    logger.sinks = previous


def test_default_logger_writes_to_one_stderr_sink(sinks):
    assert len(sinks) == 1
    assert isinstance(sinks[0], tesseract_common.StderrSink)


def test_python_logging_sink_replaces_stderr(capfd, caplog, logger, sinks):
    """With only a PythonLoggingSink, a tesseract warning reaches Python logging once and stderr not at all."""
    logger.set_level(LogLevel.warn)
    logger.sinks = [tesseract_common.PythonLoggingSink("tesseract.cpp")]
    with caplog.at_level("WARNING", logger="tesseract.cpp"):
        _warn_once()
    _, err = capfd.readouterr()
    assert UNHANDLED not in err
    (record,) = [r for r in caplog.records if UNHANDLED in r.getMessage()]
    assert record.name == "tesseract.cpp"
    assert record.levelname == "WARNING"
    assert record.pathname.endswith(".cpp") and record.lineno > 0
    assert record.spdlog_logger == "tesseract"


def test_python_logging_sink_pattern_and_levels(caplog, logger, sinks):
    """The sink's pattern makes the text; a sink level and the Python logger's level both filter."""
    logger.set_level(LogLevel.warn)
    sink = tesseract_common.PythonLoggingSink("tesseract.cpp")
    sink.set_pattern("%v (%s:%#)")
    logger.sinks = [sink]
    with caplog.at_level("WARNING", logger="tesseract.cpp"):
        _warn_once()
        (record,) = [r for r in caplog.records if UNHANDLED in r.getMessage()]
        assert record.getMessage().endswith(f"({record.filename}:{record.lineno})")

        caplog.clear()
        sink.set_level(LogLevel.err)
        _warn_once()
        assert not [r for r in caplog.records if UNHANDLED in r.getMessage()]
    sink.set_level(LogLevel.trace)
    with caplog.at_level("ERROR", logger="tesseract.cpp"):
        _warn_once()
        assert not [r for r in caplog.records if UNHANDLED in r.getMessage()]


def test_only_tesseract_sinks_can_be_assigned(logger, sinks):
    with pytest.raises(TypeError):
        logger.sinks = [object()]
    assert logger.sinks == sinks or len(logger.sinks) == len(sinks)


def test_stderr_sink_restores_the_default_output(capfd, logger, sinks):
    logger.set_level(LogLevel.warn)
    logger.sinks = [tesseract_common.StderrSink()]
    _warn_once()
    _, err = capfd.readouterr()
    assert UNHANDLED in err


def test_default_logger_is_named_tesseract_and_round_trips_its_level(logger):
    assert logger.name() == "tesseract"
    logger.set_level(LogLevel.err)
    assert tesseract_common.getLogger().level() == LogLevel.err
    assert not tesseract_common.isLogLevelEnabled(LogLevel.warn)
    assert tesseract_common.isLogLevelEnabled(LogLevel.critical)


def test_record_handler_receives_a_tesseract_warning(logger):
    logger.set_level(LogLevel.warn)
    records = []
    handler_id = tesseract_common.addLogRecordHandler(records.append)
    try:
        _warn_once()
    finally:
        assert tesseract_common.removeLogRecordHandler(handler_id)

    (record,) = [r for r in records if UNHANDLED in r.message]
    # the record is a copy: still readable after the C++ call returned
    assert record.level == LogLevel.warn
    assert record.logger_name == "tesseract"
    assert record.message == f"Resource not handled: {UNHANDLED}"
    assert record.filename.endswith(".cpp")
    assert record.line > 0


def test_removed_handler_receives_nothing(logger):
    logger.set_level(LogLevel.warn)
    records = []
    handler_id = tesseract_common.addLogRecordHandler(records.append)
    assert tesseract_common.removeLogRecordHandler(handler_id)
    assert not tesseract_common.removeLogRecordHandler(handler_id)
    _warn_once()
    assert records == []


def test_handler_exception_reaches_unraisablehook(monkeypatch, logger):
    logger.set_level(LogLevel.warn)
    reported = []
    # Keep the type and text, not the hook's arguments: their traceback's frames hold this test's
    # locals (reported among them), a cycle that keeps the record and logger alive past exit.
    monkeypatch.setattr(
        sys, "unraisablehook", lambda u: reported.append((type(u.exc_value), str(u.exc_value)))
    )

    def broken(record):
        raise RuntimeError("handler bug")

    handler_id = tesseract_common.addLogRecordHandler(broken)
    try:
        _warn_once()
    finally:
        tesseract_common.removeLogRecordHandler(handler_id)

    assert reported == [(RuntimeError, "handler bug")]


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


def test_a_python_sink_left_installed_is_released_before_interpreter_exit():
    """The default logger outlives Python, so the sink it holds must be let go at exit."""
    script = (
        "import logging\n"
        "from tesseract_robotics import tesseract_common as tc\n"
        "logging.basicConfig()\n"
        "tc.getLogger().sinks = [tc.PythonLoggingSink('tesseract.cpp')]\n"
        f"tc.GeneralResourceLocator().locateResource({UNHANDLED!r})\n"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "leaked" not in result.stderr
    # routed once, through Python logging, not also by spdlog's stderr sink
    assert result.stderr.count("Resource not handled") == 1
    assert "WARNING:tesseract.cpp:" in result.stderr


REMOVED_NAMES = [
    "setLogLevel",
    "getLogLevel",
    "log",
    "useOutputHandler",
    "restorePreviousOutputHandler",
    "OutputHandler",
    "LoggerLevel",
    "CONSOLE_BRIDGE_LOG_DEBUG",
    "CONSOLE_BRIDGE_LOG_INFO",
    "CONSOLE_BRIDGE_LOG_WARN",
    "CONSOLE_BRIDGE_LOG_ERROR",
    "CONSOLE_BRIDGE_LOG_NONE",
]


@pytest.mark.parametrize("name", REMOVED_NAMES)
def test_console_bridge_api_is_gone(name):
    # it silently did nothing against upstream; an AttributeError says so
    assert not hasattr(tesseract_common, name)


def test_log_level_is_the_spdlog_enum():
    assert [v.name for v in LogLevel] == [
        "trace",
        "debug",
        "info",
        "warn",
        "err",
        "critical",
        "off",
    ]
