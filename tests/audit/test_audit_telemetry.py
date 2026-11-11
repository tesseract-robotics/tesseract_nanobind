"""Contract tests of the audit's telemetry (`scripts/audit_telemetry.py`): one line per fetch
and per timed phase, whole lines under concurrent writers, nothing written when no log is set,
and never inside the cache directory. Needs no libclang.
"""

import json
import multiprocessing
from pathlib import Path

import pytest

from ._free import load_free

tel = load_free("audit_telemetry")
ac = load_free("audit_cache")
tc = load_free("audit_toolchain")

WRITERS = 3
EVENTS_PER_WRITER = 200


def _key(source: Path, kind: str = "decl") -> "ac.FactKey":
    toolchain = tc.Toolchain.build(
        args=("-x", "c++"),
        options=0,
        env=tc.digest_of("env"),
        search_env={},
        repo_dirs={},
        extractors=tc.digest_of("extractors"),
    )
    return ac.FactKey(kind=kind, source=source, toolchain=toolchain)


@pytest.fixture
def header(tmp_path):
    h = tmp_path / "a.h"
    h.write_text("int a;\n")
    return h


def test_no_log_is_set_without_the_variable(monkeypatch):
    monkeypatch.delenv(tel.EVENTS_ENV, raising=False)
    assert tel.EventLog.from_env() == tel.EventLog(None)


def test_the_variable_names_the_log(monkeypatch, tmp_path):
    monkeypatch.setenv(tel.EVENTS_ENV, str(tmp_path / "events.jsonl"))
    assert tel.EventLog.from_env() == tel.EventLog(tmp_path / "events.jsonl")


def test_an_unset_log_writes_nothing(tmp_path, header, monkeypatch):
    monkeypatch.chdir(tmp_path)
    cache = ac.FactCache(tmp_path / "cache", events=tel.EventLog(None))
    cache.fetch(_key(header), lambda: ("v", [header]))
    with tel.EventLog(None).timed("decl-parse"):
        pass
    written = {p.relative_to(tmp_path) for p in tmp_path.rglob("*") if p.is_file()}
    assert {p.suffix for p in written} == {".h", ".json", ".lock"}


def test_a_miss_and_a_hit_are_one_event_each(tmp_path, header):
    log = tmp_path / "events.jsonl"
    cache = ac.FactCache(tmp_path / "cache", events=tel.EventLog(log))
    cache.fetch(_key(header), lambda: ("v", [header]))
    cache.fetch(_key(header), lambda: pytest.fail("a hit computed"))
    miss, hit = tel.read_events(log)
    assert (miss["event"], miss["kind"], miss["hit"]) == ("fetch", "decl", False)
    assert (hit["event"], hit["kind"], hit["hit"]) == ("fetch", "decl", True)
    assert miss["compute_s"] > 0
    assert hit["compute_s"] == 0
    assert miss["wait_s"] >= 0 and hit["wait_s"] >= 0


def test_a_failed_compute_records_no_fetch(tmp_path, header):
    """A fetch that raised stored nothing: it is neither a hit nor a miss."""
    log = tmp_path / "events.jsonl"
    cache = ac.FactCache(tmp_path / "cache", events=tel.EventLog(log))

    def compute():
        raise RuntimeError("parse failed")

    with pytest.raises(RuntimeError, match="parse failed"):
        cache.fetch(_key(header), compute)
    assert not log.exists()


def test_the_log_may_not_live_in_the_cache(tmp_path):
    """CI saves the cache directory: an event log inside it would be restored into later runs."""
    root = tmp_path / "cache"
    with pytest.raises(ac.EventsInCacheError):
        ac.FactCache(root, events=tel.EventLog(root / "decl" / "events.jsonl"))


def test_a_timed_phase_is_recorded_when_its_body_raises(tmp_path):
    log = tel.EventLog(tmp_path / "events.jsonl")
    with pytest.raises(KeyError), log.timed("cpp-api"):
        raise KeyError("x")
    (event,) = tel.read_events(tmp_path / "events.jsonl")
    assert (event["event"], event["name"]) == ("phase", "cpp-api")
    assert event["seconds"] >= 0


def _write_many(path: str, count: int) -> None:
    log = load_free("audit_telemetry").EventLog(Path(path))
    for _ in range(count):
        log.fetched("bound", hit=True, compute_s=0.0, wait_s=0.0)


def test_concurrent_writers_leave_whole_lines(tmp_path):
    """xdist workers share one log: every line is one event, none torn or merged."""
    log = tmp_path / "events.jsonl"
    with multiprocessing.get_context("spawn").Pool(WRITERS) as pool:
        pool.starmap(_write_many, [(str(log), EVENTS_PER_WRITER)] * WRITERS)
    events = tel.read_events(log)
    assert len(events) == WRITERS * EVENTS_PER_WRITER
    assert len({e["pid"] for e in events}) == WRITERS


def test_a_line_that_is_not_an_event_is_loud(tmp_path):
    log = tmp_path / "events.jsonl"
    log.write_text('{"event": "fetch"\n', encoding="utf-8")
    with pytest.raises(tel.EventShapeError):
        tel.read_events(log)
    log.write_text(json.dumps({"event": "nap", "pid": 1}) + "\n", encoding="utf-8")
    with pytest.raises(tel.EventShapeError):
        tel.read_events(log)
    log.write_text(json.dumps({"event": "phase", "name": "x", "pid": 1}) + "\n", encoding="utf-8")
    with pytest.raises(tel.EventShapeError):
        tel.read_events(log)


def test_totals_per_kind_and_phase():
    events = [
        {"event": "fetch", "kind": "bound", "hit": False, "compute_s": 2.0, "wait_s": 0.5, "pid": 1},
        {"event": "fetch", "kind": "bound", "hit": True, "compute_s": 0.0, "wait_s": 1.5, "pid": 2},
        {"event": "fetch", "kind": "gil", "hit": True, "compute_s": 0.0, "wait_s": 0.0, "pid": 1},
        {"event": "phase", "name": "cpp-api", "seconds": 3.0, "pid": 1},
        {"event": "phase", "name": "cpp-api", "seconds": 4.0, "pid": 2},
    ]  # fmt: skip
    totals = tel.totals(events)
    assert totals.fetches == {
        "bound": tel.FetchTotal(hits=1, misses=1, compute_s=2.0, wait_s=2.0),
        "gil": tel.FetchTotal(hits=1, misses=0, compute_s=0.0, wait_s=0.0),
    }
    assert totals.phases == {"cpp-api": tel.PhaseTotal(calls=2, seconds=7.0)}
    assert totals.processes == 2


def test_the_rendered_table_names_every_kind_and_phase():
    events = [
        {"event": "fetch", "kind": "bound", "hit": False, "compute_s": 2.0, "wait_s": 0.5, "pid": 1},
        {"event": "phase", "name": "decl-parse", "seconds": 61.25, "pid": 1},
    ]  # fmt: skip
    text = tel.render("3-audit", events)
    lines = text.splitlines()
    assert lines[0] == "3-audit: 2 events from 1 process"
    assert any(line.split() == ["bound", "0", "1", "2.0", "0.5"] for line in lines)
    assert any(line.split() == ["decl-parse", "1", "61.2"] for line in lines)


def test_the_report_prints_one_table_per_step(tmp_path, capsys):
    events = tmp_path / "events"
    events.mkdir()
    tel.EventLog(events / "2-gil.jsonl").fetched("gil", hit=True, compute_s=0.0, wait_s=0.0)
    tel.EventLog(events / "1-tests.jsonl").fetched("decl", hit=False, compute_s=1.0, wait_s=0.0)
    assert load_free("audit_telemetry_report").main([str(events)]) == 0
    out = capsys.readouterr().out
    assert out.index("1-tests:") < out.index("2-gil:")


def test_the_report_needs_its_directory(tmp_path):
    with pytest.raises(tel.NoEventsDirError):
        load_free("audit_telemetry_report").main([str(tmp_path / "absent")])
