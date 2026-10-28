"""A Python OutputHandler never runs inside console_bridge's output lock.

console_bridge calls its handler with a global mutex held. A Python handler called there waits
for (or gives up) the GIL with the mutex held, so any thread that logs while it holds the GIL
deadlocks against it. Python handlers get their messages on a delivery thread instead, in order,
through a bounded queue. Each case runs in a subprocess: the failures are hangs and crashes.
"""

import subprocess
import sys

import pytest

# A case logs a few thousand messages; a deadlock never ends. Generous for a loaded machine.
TIMEOUT_S = 60

_PREAMBLE = """
import gc
import os
import sys
import threading
import time
from tesseract_robotics import tesseract_common

LEVEL = tesseract_common.LogLevel.CONSOLE_BRIDGE_LOG_INFO
tesseract_common.setLogLevel(LEVEL)

def log(text):
    tesseract_common.log("test.py", 1, LEVEL, text)

class Collect(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.texts = []
        self.threads = set()

    def log(self, text, level, filename, line):
        self.texts.append(text)
        self.threads.add(threading.get_ident())
"""


def _run_both(script: str, preamble: str = _PREAMBLE) -> "tuple[str, str]":
    """Run `script` after `preamble` in a fresh interpreter; return its stdout and stderr."""
    proc = subprocess.run(
        [sys.executable, "-c", preamble + script],
        capture_output=True,
        text=True,
        timeout=TIMEOUT_S,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode}: {proc.stderr[-1500:]}"
    return proc.stdout, proc.stderr


def _run(script: str, preamble: str = _PREAMBLE) -> str:
    return _run_both(script, preamble)[0]


def test_handler_that_releases_the_gil_with_two_logging_threads():
    """Both threads log with the GIL held. The handler gave the GIL away inside console_bridge's
    lock; the other thread then blocked on that lock holding the GIL: a deadlock."""
    out = _run("""
class Sleepy(Collect):
    def log(self, text, level, filename, line):
        time.sleep(0.0005)
        super().log(text, level, filename, line)

handler = Sleepy()
tesseract_common.useOutputHandler(handler)

def spam(tag):
    for i in range(300):
        log(f"{tag}{i}")

threads = [threading.Thread(target=spam, args=(tag,)) for tag in "ab"]
for t in threads:
    t.start()
for t in threads:
    t.join()
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("DELIVERED", len(handler.texts))
""")
    assert "DELIVERED 600" in out, out


def test_messages_arrive_in_order_on_the_delivery_thread():
    out = _run("""
handler = Collect()
tesseract_common.useOutputHandler(handler)
for i in range(2000):
    log(str(i))
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("ORDERED", handler.texts == [str(i) for i in range(2000)])
print("ON DELIVERY THREAD", threading.get_ident() not in handler.threads)
""")
    assert "ORDERED True" in out, out
    assert "ON DELIVERY THREAD True" in out, out


def test_installed_handler_is_kept_alive():
    """useOutputHandler kept only a raw pointer: a handler without another reference was freed
    and the next message called into freed memory."""
    out = _run("""
seen = []

class Record(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        seen.append(text)

tesseract_common.useOutputHandler(Record())
gc.collect()
log("kept")
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("SEEN", seen)
""")
    assert "SEEN ['kept']" in out, out


def test_handler_without_a_log_override_goes_to_unraisablehook():
    """The base OutputHandler.log is the C++ binding, which would queue the message again."""
    out = _run("""
caught = []
sys.unraisablehook = lambda u: caught.append((type(u.exc_value).__name__, u.object is handler))

class NoLog(tesseract_common.OutputHandler):
    pass

handler = NoLog()
tesseract_common.useOutputHandler(handler)
log("lost")
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("CAUGHT", caught)
""")
    assert "CAUGHT [('RuntimeError', True)]" in out, out


def test_handler_exception_goes_to_unraisablehook():
    out = _run("""
caught = []
sys.unraisablehook = lambda u: caught.append(type(u.exc_value).__name__)

class Fails(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        raise KeyError(text)

handler = Fails()
tesseract_common.useOutputHandler(handler)
log("boom")
log("again")
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("CAUGHT", caught)
""")
    assert "CAUGHT ['KeyError', 'KeyError']" in out, out


def test_each_message_reaches_the_handler_it_was_logged_to():
    """A swap does not wait for the queue: messages already queued still go to the handler that
    was installed when they were logged."""
    out = _run("""
first, second = Collect(), Collect()
tesseract_common.useOutputHandler(first)
log("one")
tesseract_common.useOutputHandler(second)
log("two")
tesseract_common.flushOutputHandlers()
print("FIRST", first.texts, "SECOND", second.texts)
""")
    assert "FIRST ['one'] SECOND ['two']" in out, out


def test_swap_while_the_handler_waits_for_the_swapper():
    """A swap that waited for the queue deadlocked with a handler waiting for the swapping
    thread (in a real program: a handler waiting on a lock the swapper holds)."""
    out = _run("""
in_handler = threading.Event()
release = threading.Event()

class Waits(Collect):
    def log(self, text, level, filename, line):
        in_handler.set()
        release.wait(30)
        super().log(text, level, filename, line)

handler = Waits()
tesseract_common.useOutputHandler(handler)
log("first")
assert in_handler.wait(30)
tesseract_common.useOutputHandler(Collect())  # must not wait for `handler`
release.set()
tesseract_common.flushOutputHandlers()
print("SWAPPED", handler.texts)
""")
    assert "SWAPPED ['first']" in out, out


def test_handler_swaps_the_handler_from_inside():
    out = _run("""
swapped = threading.Event()

class SwapOnce(Collect):
    def log(self, text, level, filename, line):
        super().log(text, level, filename, line)
        if text == "swap":
            tesseract_common.restorePreviousOutputHandler()
            tesseract_common.flushOutputHandlers()  # on the delivery thread: returns at once
            swapped.set()

inner = Collect()
tesseract_common.useOutputHandler(inner)
handler = SwapOnce()
tesseract_common.useOutputHandler(handler)
log("swap")
assert swapped.wait(30)
log("after")  # queued for the handler current when it is logged: `inner`
tesseract_common.flushOutputHandlers()
print("HANDLER", handler.texts, "INNER", inner.texts)
""")
    assert "HANDLER ['swap'] INNER ['after']" in out, out


def test_queue_is_bounded_and_drops_are_reported():
    """A slow handler and fast loggers: the queue grew without bound (5.5M messages, 1.7 GB,
    hours to drain at exit). Full, it drops and later reports how many, so every message is
    either delivered or counted."""
    out = _run("""
import re

class Slow(Collect):
    def log(self, text, level, filename, line):
        time.sleep(0.0001)
        super().log(text, level, filename, line)

handler = Slow()
tesseract_common.useOutputHandler(handler)
sent = [0]

def flood():
    end = time.monotonic() + 1.0
    n = 0
    while time.monotonic() < end:
        log("m")
        n += 1
    sent[0] += n

threads = [threading.Thread(target=flood) for _ in range(4)]
for t in threads:
    t.start()
for t in threads:
    t.join()
start = time.monotonic()
tesseract_common.flushOutputHandlers()
flush_s = time.monotonic() - start
tesseract_common.restorePreviousOutputHandler()
delivered = handler.texts.count("m")
dropped = sum(int(m.group(1)) for t in handler.texts for m in [re.search(r"dropped (\\d+) log messages", t)] if m)
print("ACCOUNTED", delivered + dropped == sent[0], "DROPPED", dropped > 0, "FLUSH FAST", flush_s < 10)
""")
    assert "ACCOUNTED True DROPPED True FLUSH FAST True" in out, out


@pytest.mark.skipif(sys.platform == "win32", reason="Windows has no os.fork")
def test_forked_child_delivers():
    """The child of a fork has no delivery thread: it starts one on its first message."""
    out = _run("""
handler = Collect()
tesseract_common.useOutputHandler(handler)
log("parent")
tesseract_common.flushOutputHandlers()
pid = os.fork()
if pid == 0:
    log("child")
    tesseract_common.flushOutputHandlers()
    os.write(1, ("CHILD %r\\n" % handler.texts).encode())
    os._exit(0)
_, status = os.waitpid(pid, 0)
print("STATUS", status)
""")
    assert "CHILD ['parent', 'child']" in out, out
    assert "STATUS 0" in out, out


def test_exit_while_threads_keep_logging():
    """A daemon thread logging forever must not keep interpreter exit waiting for the queue."""
    out = _run("""
handler = Collect()
tesseract_common.useOutputHandler(handler)

def forever():
    while True:
        log("x")

threading.Thread(target=forever, daemon=True).start()
time.sleep(0.2)
print("EXITING", flush=True)
""")
    assert "EXITING" in out, out


def test_exit_while_the_handler_never_returns():
    """Exit gives the delivery thread a bounded time, then leaves it: a stuck handler does not
    keep the interpreter from exiting."""
    out = _run("""
entered = threading.Event()

class Stuck(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        entered.set()
        threading.Event().wait()

handler = Stuck()
tesseract_common.useOutputHandler(handler)
log("stuck")
assert entered.wait(30)
log("never delivered")
print("EXITING", flush=True)
""")
    assert "EXITING" in out, out


def test_flush_waits_for_a_message_logged_while_drops_are_reported():
    """The delivery thread's drop notice was counted as queued only once its batch was delivered:
    a flush that started during that batch returned one message early."""
    out = _run("""
release_block = threading.Event()
at_last = threading.Event()
release_last = threading.Event()

class Gated(Collect):
    def log(self, text, level, filename, line):
        if text == "BLOCK":
            release_block.wait(30)
        if text == "m9999":
            at_last.set()
            release_last.wait(30)
        if text == "AFTER":
            time.sleep(0.5)  # the flush must wait for this
        super().log(text, level, filename, line)

handler = Gated()
tesseract_common.useOutputHandler(handler)
log("BLOCK")
time.sleep(0.3)  # the delivery thread waits inside BLOCK
for i in range(10500):  # the queue holds 10000: 500 are dropped
    log(f"m{i}")
release_block.set()
assert at_last.wait(30)  # inside the last batch, which carries the drop notice
log("AFTER")
seen = {}

def flusher():
    tesseract_common.flushOutputHandlers()
    seen["after"] = "AFTER" in handler.texts

t = threading.Thread(target=flusher)
t.start()
time.sleep(0.3)  # the flush reads its target while the notice's batch is being delivered
release_last.set()
t.join(30)
tesseract_common.restorePreviousOutputHandler()
print("FLUSHED AFTER", seen.get("after"))
""")
    assert "FLUSHED AFTER True" in out, out


def test_drops_are_reported_to_their_handler_after_it_is_uninstalled():
    """Drops counted while the handler fell behind were forgotten, without a notice or a stderr
    line, when no Python handler was installed by the time the queue ran empty."""
    out = _run("""
release = threading.Event()

class Blocks(Collect):
    def log(self, text, level, filename, line):
        if text == "BLOCK":
            release.wait(30)
        super().log(text, level, filename, line)

handler = Blocks()
tesseract_common.useOutputHandler(handler)
log("BLOCK")
time.sleep(0.3)
for i in range(10500):
    log(f"m{i}")
tesseract_common.restorePreviousOutputHandler()  # console_bridge's own handler again
release.set()
tesseract_common.flushOutputHandlers()
notices = [t for t in handler.texts if "dropped" in t]
print("NOTICES", notices)
""")
    assert "NOTICES ['tesseract_common: dropped 500 log messages" in out, out


def test_drops_from_two_handlers_are_reported_to_each():
    out = _run("""
release = threading.Event()

class Blocks(Collect):
    def log(self, text, level, filename, line):
        if text == "BLOCK":
            release.wait(30)
        super().log(text, level, filename, line)

first, second = Blocks(), Collect()
tesseract_common.useOutputHandler(first)
log("BLOCK")
time.sleep(0.3)
for i in range(10200):  # 10000 queued (BLOCK is taken): 200 dropped
    log(f"a{i}")
tesseract_common.useOutputHandler(second)
for i in range(30):  # all dropped
    log(f"b{i}")
release.set()
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("FIRST", [t for t in first.texts if "dropped" in t])
print("SECOND", [t for t in second.texts if "dropped" in t])
""")
    assert "FIRST ['tesseract_common: dropped 200 log messages" in out, out
    assert "SECOND ['tesseract_common: dropped 30 log messages" in out, out


@pytest.mark.skipif(sys.platform == "win32", reason="Windows has no os.fork")
def test_fork_from_inside_a_handler():
    """The child's only thread is a copy of the delivery thread. It went on delivering the
    parent's queue from the parent's state (each message twice, once per process) and the child's
    first message started a second delivery thread."""
    out = _run("""
r, w = os.pipe()

class Forks(Collect):
    child = None
    in_child = False

    def log(self, text, level, filename, line):
        super().log(text, level, filename, line)
        if text == "FORK":
            pid = os.fork()
            if pid == 0:
                self.in_child = True
                self.texts = []
                self.threads = set()
                threading.Thread(target=child_main).start()
            else:
                self.child = pid

def child_main():
    time.sleep(0.5)  # the parent's queued messages would arrive meanwhile
    log("CHILD")
    tesseract_common.flushOutputHandlers()
    os.write(w, ("CHILD %r THREADS %d\\n" % (handler.texts, len(handler.threads))).encode())
    os._exit(0)

handler = Forks()
tesseract_common.useOutputHandler(handler)
log("FORK")
for i in range(5):
    log(f"parent{i}")
tesseract_common.flushOutputHandlers()
_, status = os.waitpid(handler.child, 0)
os.close(w)
print(os.read(r, 100000).decode())
print("PARENT", handler.texts, "STATUS", status)
tesseract_common.restorePreviousOutputHandler()
""")
    assert "CHILD ['CHILD'] THREADS 1" in out, out
    assert (
        "PARENT ['FORK', 'parent0', 'parent1', 'parent2', 'parent3', 'parent4'] STATUS 0" in out
    ), out


def test_exit_prints_what_the_delivery_thread_has_not_delivered():
    """Exit printed only the queue: the rest of the batch the delivery thread had taken (up to
    LOG_DELIVERY_BATCH messages) was neither delivered nor printed, and neither were unreported
    drops."""
    out, err = _run_both("""
release = threading.Event()

class Spins(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        if text == "BLOCK":
            release.wait(30)
        if text == "SPIN":
            while True:
                pass

handler = Spins()
tesseract_common.useOutputHandler(handler)
log("BLOCK")
time.sleep(0.3)
log("SPIN")
for i in range(3):
    log(f"in-batch-{i}")  # queued behind SPIN, taken with it
release.set()
time.sleep(0.3)  # the delivery thread spins inside SPIN
for i in range(3):
    log(f"queued-{i}")
print("EXITING", flush=True)
""")
    assert "EXITING" in out, out
    lines = err.splitlines()
    for text in ["in-batch-0", "in-batch-1", "in-batch-2", "queued-0", "queued-1", "queued-2"]:
        assert text in lines, err
    assert "SPIN" not in lines, err  # the handler has it


def test_exit_prints_unreported_drops():
    out, err = _run_both("""
entered = threading.Event()

class Stuck(tesseract_common.OutputHandler):
    def log(self, text, level, filename, line):
        entered.set()
        threading.Event().wait()

handler = Stuck()
tesseract_common.useOutputHandler(handler)
log("stuck")
assert entered.wait(30)
for i in range(10007):
    log(f"m{i}")
print("EXITING", flush=True)
""")
    assert "EXITING" in out, out
    assert "tesseract_common: dropped 7 log messages" in err, err


@pytest.mark.skipif(
    sys.platform == "win32",
    reason="console_bridge::log is looked up by its Itanium-mangled name through the extension; "
    "a Windows .pyd does not resolve its dependencies' symbols",
)
def test_null_filename_reaches_the_handler_as_none():
    """C++ code may log without a file; the handler got "" instead of None. Python cannot pass a
    null filename, so this calls console_bridge::log through ctypes."""
    out = _run("""
import ctypes

# console_bridge::log(const char* file, int line, LogLevel level, const char* fmt, ...)
# Looked up through the extension, whose dependencies include console_bridge: CDLL(None) only
# searches the global namespace, which excludes console_bridge on Linux (RTLD_LOCAL).
from tesseract_robotics.tesseract_common import _tesseract_common
cb_log = ctypes.CDLL(_tesseract_common.__file__)["_ZN14console_bridge3logEPKciNS_8LogLevelES1_z"]
cb_log.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
cb_log.restype = None

class Files(tesseract_common.OutputHandler):
    def __init__(self):
        super().__init__()
        self.files = []

    def log(self, text, level, filename, line):
        self.files.append(filename)

handler = Files()
tesseract_common.useOutputHandler(handler)
cb_log(None, 1, LEVEL.value, b"x")
log("y")
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("FILES", handler.files)
""")
    assert "FILES [None, 'test.py']" in out, out


@pytest.mark.skipif(sys.platform == "win32", reason="Windows has no os.fork")
def test_child_forked_in_a_handler_exits_after_logging():
    """A child forked from a handler that logs, then returns from the handler, hung: the copied
    delivery thread (the child's only thread) ended, the child's own delivery thread waited
    forever."""
    out = _run("""
class Forks(tesseract_common.OutputHandler):
    child = None

    def log(self, text, level, filename, line):
        if text == "FORK" and Forks.child is None:
            pid = os.fork()
            if pid == 0:
                Forks.child = 0
                log("CHILD")
                # logs after the handler returned, then ends: the child must still end
                threading.Thread(target=lambda: (time.sleep(0.3), log("LATER"))).start()
                return  # no os._exit
            Forks.child = pid

handler = Forks()
tesseract_common.useOutputHandler(handler)
log("FORK")
tesseract_common.flushOutputHandlers()
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    pid, status = os.waitpid(Forks.child, os.WNOHANG)
    if pid:
        print("CHILD EXITED", status)
        break
    time.sleep(0.05)
else:
    os.kill(Forks.child, 9)
    os.waitpid(Forks.child, 0)
    print("CHILD HUNG")
tesseract_common.restorePreviousOutputHandler()
""")
    assert "CHILD EXITED 0" in out, out


def test_import_without_fork_support():
    """Windows has no os.register_at_fork; registering the fork reset unconditionally made the
    import fail there."""
    out = _run(
        """
handler = Collect()
tesseract_common.useOutputHandler(handler)
log("no fork")
tesseract_common.flushOutputHandlers()
tesseract_common.restorePreviousOutputHandler()
print("DELIVERED", handler.texts)
""",
        # as on Windows: no fork, so no fork hooks
        preamble="import os\ndel os.fork, os.register_at_fork\n" + _PREAMBLE,
    )
    assert "DELIVERED ['no fork']" in out, out
