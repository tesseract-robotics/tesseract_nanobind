"""A Python OutputHandler never runs inside console_bridge's output lock.

console_bridge calls its handler with a global mutex held. A Python handler called there waits
for (or gives up) the GIL with the mutex held, so any thread that logs while it holds the GIL
deadlocks against it. Python handlers get their messages on a delivery thread instead, in order,
through a bounded queue. Each case runs in a subprocess: the failures are hangs and crashes.
"""

import subprocess
import sys

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


def _run(script: str) -> str:
    proc = subprocess.run(
        [sys.executable, "-c", _PREAMBLE + script],
        capture_output=True,
        text=True,
        timeout=TIMEOUT_S,
        check=False,
    )
    assert proc.returncode == 0, f"rc={proc.returncode}: {proc.stderr[-1500:]}"
    return proc.stdout


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
