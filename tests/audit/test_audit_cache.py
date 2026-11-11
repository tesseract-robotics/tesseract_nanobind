"""Contract tests of the fact cache (`scripts/audit_cache.py`): a hit only when every file the
parse read is byte-identical, one computation per key across processes, loud on a bad entry.

Loads `audit_cache` alone: it needs no libclang, so these tests run in any env.
"""

import importlib.util
import multiprocessing
import os
import sys
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent.parent / "scripts" / "audit_cache.py"


def _load_cache_module():
    if "audit_cache" in sys.modules:
        return sys.modules["audit_cache"]
    spec = importlib.util.spec_from_file_location("audit_cache", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["audit_cache"] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


ac = _load_cache_module()


def _load_prune_cli():
    """The prune CLI; it imports `audit_cache`, which `_load_cache_module` registered."""
    path = SCRIPT.with_name("audit_cache_prune.py")
    spec = importlib.util.spec_from_file_location("audit_cache_prune", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Counter:
    """A `compute` that records its calls and reads `deps` as the parse would."""

    def __init__(self, value, deps):
        self.value = value
        self.deps = deps
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.value, self.deps


@pytest.fixture
def tree(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "a.h").write_text("int a;\n")
    (src / "b.h").write_text("int b;\n")
    return src


def key(source: Path, toolchain: str = "t0") -> "ac.FactKey":
    return ac.FactKey(kind="decl", source=source, toolchain=toolchain)


def test_miss_computes_and_a_hit_does_not(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    compute = Counter({"names": ["a", "b"]}, [tree / "a.h", tree / "b.h"])
    assert cache.fetch(key(tree / "a.h"), compute) == {"names": ["a", "b"]}
    assert cache.fetch(key(tree / "a.h"), compute) == {"names": ["a", "b"]}
    assert compute.calls == 1


def test_a_fresh_cache_object_reads_the_stored_entry(tmp_path, tree):
    compute = Counter([1, 2], [tree / "a.h"])
    ac.FactCache(tmp_path / "cache").fetch(key(tree / "a.h"), compute)
    assert ac.FactCache(tmp_path / "cache").fetch(key(tree / "a.h"), compute) == [1, 2]
    assert compute.calls == 1


def test_an_edited_dependency_recomputes(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    compute = Counter("v", [tree / "a.h", tree / "b.h"])
    cache.fetch(key(tree / "a.h"), compute)
    (tree / "b.h").write_text("int b2;\n")
    cache.fetch(key(tree / "a.h"), compute)
    assert compute.calls == 2


def test_an_edit_that_keeps_size_and_mtime_recomputes(tmp_path, tree):
    """Content, not stat: a checkout can restore an old mtime on new bytes."""
    cache = ac.FactCache(tmp_path / "cache")
    dep = tree / "b.h"
    compute = Counter("v", [dep])
    cache.fetch(key(tree / "a.h"), compute)
    st = dep.stat()
    dep.write_text("int c;\n")  # same length
    os.utime(dep, ns=(st.st_atime_ns, st.st_mtime_ns))
    ac.FactCache(tmp_path / "cache").fetch(key(tree / "a.h"), compute)
    assert compute.calls == 2


def test_a_removed_dependency_recomputes(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    compute = Counter("v", [tree / "a.h", tree / "b.h"])
    cache.fetch(key(tree / "a.h"), compute)
    (tree / "b.h").unlink()
    compute.deps = [tree / "a.h"]
    cache.fetch(key(tree / "a.h"), compute)
    assert compute.calls == 2


def test_another_toolchain_is_another_entry(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    compute = Counter("v", [tree / "a.h"])
    cache.fetch(key(tree / "a.h", "t0"), compute)
    cache.fetch(key(tree / "a.h", "t1"), compute)
    cache.fetch(key(tree / "a.h", "t0"), compute)
    assert compute.calls == 2


def test_kind_and_source_are_part_of_the_key(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    compute = Counter("v", [tree / "a.h"])
    cache.fetch(ac.FactKey("decl", tree / "a.h", "t"), compute)
    cache.fetch(ac.FactKey("bound", tree / "a.h", "t"), compute)
    cache.fetch(ac.FactKey("decl", tree / "b.h", "t"), compute)
    assert compute.calls == 3


def test_a_hit_marks_the_entry_used(tmp_path, tree):
    """CI prunes entries a run did not use by mtime: a hit must count as a use."""
    cache = ac.FactCache(tmp_path / "cache")
    k = key(tree / "a.h")
    cache.fetch(k, Counter("v", [tree / "a.h"]))
    entry = cache.entry_path(k)
    os.utime(entry, ns=(0, 0))
    cache.fetch(k, Counter("v", [tree / "a.h"]))
    assert entry.stat().st_mtime_ns > 0


def test_prune_removes_entries_unused_since(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    old, new = key(tree / "a.h"), key(tree / "b.h")
    cache.fetch(old, Counter("v", [tree / "a.h"]))
    cache.fetch(new, Counter("w", [tree / "b.h"]))
    os.utime(cache.entry_path(old), ns=(0, 0))
    removed = cache.prune(unused_since_ns=1)
    assert removed == [cache.entry_path(old)]
    assert not cache.entry_path(old).exists()
    assert cache.entry_path(new).exists()


def test_a_dependency_must_exist_when_stored(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    with pytest.raises(ac.MissingDependencyError):
        cache.fetch(key(tree / "a.h"), Counter("v", [tree / "nope.h"]))


def test_a_corrupt_entry_is_loud(tmp_path, tree):
    cache = ac.FactCache(tmp_path / "cache")
    k = key(tree / "a.h")
    cache.fetch(k, Counter("v", [tree / "a.h"]))
    cache.entry_path(k).write_text("{not json", encoding="utf-8")
    with pytest.raises(ac.CorruptEntryError):
        cache.fetch(k, Counter("v", [tree / "a.h"]))


def test_digest_is_stable_across_processes(tmp_path, tree):
    """The digest names the entry on disk: it must not depend on `hash()` randomisation."""
    k = key(tree / "a.h")
    with multiprocessing.get_context("spawn").Pool(1) as pool:
        other = pool.apply(_digest_in_child, (str(SCRIPT), str(tree / "a.h")))
    assert other == k.digest()


def _digest_in_child(script: str, source: str) -> str:
    spec = importlib.util.spec_from_file_location("audit_cache", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["audit_cache"] = module
    spec.loader.exec_module(module)
    return module.FactKey(kind="decl", source=Path(source), toolchain="t0").digest()


def _fetch_in_child(script: str, root: str, source: str, dep: str, log: str) -> object:
    spec = importlib.util.spec_from_file_location("audit_cache", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["audit_cache"] = module
    spec.loader.exec_module(module)

    def compute():
        with Path(log).open("a", encoding="utf-8") as f:
            f.write("x")
        time.sleep(0.5)  # long enough that an unlocked second worker would also compute
        return "v", [Path(dep)]

    k = module.FactKey(kind="decl", source=Path(source), toolchain="t0")
    return module.FactCache(Path(root)).fetch(k, compute)


def test_concurrent_workers_compute_once(tmp_path, tree):
    """xdist workers asking for one fact at once: one parses, the others wait and read."""
    log = tmp_path / "calls.log"
    args = (str(SCRIPT), str(tmp_path / "cache"), str(tree / "a.h"), str(tree / "a.h"), str(log))
    with multiprocessing.get_context("spawn").Pool(3) as pool:
        results = pool.starmap(_fetch_in_child, [args] * 3)
    assert results == ["v", "v", "v"]
    assert log.read_text(encoding="utf-8") == "x"


def test_prune_command_keeps_what_the_run_used(tmp_path, tree):
    """CI touches a stamp at the start of the run, then prunes what no step used after it."""
    root = tmp_path / "cache"
    cache = ac.FactCache(root)
    unused, used = key(tree / "a.h"), key(tree / "b.h")
    cache.fetch(unused, Counter("v", [tree / "a.h"]))
    cache.fetch(used, Counter("w", [tree / "b.h"]))
    stamp = tmp_path / "run-start"
    stamp.touch()
    os.utime(stamp, ns=(10**18, 10**18))
    os.utime(cache.entry_path(unused), ns=(1, 1))
    os.utime(cache.entry_path(used), ns=(2 * 10**18, 2 * 10**18))
    assert _load_prune_cli().main([str(root), str(stamp)]) == 0
    assert not cache.entry_path(unused).exists()
    assert cache.entry_path(used).exists()
