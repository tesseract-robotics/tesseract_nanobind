"""`child_stderr` keeps a dead child's traceback visible behind nanobind's exit leak report (#156)."""

from tests.trajopt_sqp.child_stderr import child_stderr

_TRACEBACK = (
    'Traceback (most recent call last):\n  File "<string>", line 3, in <module>\nIndexError: boom\n'
)
# What every child prints at exit: nanobind's leak report, about 1.4 kB.
_LEAK_REPORT = "nanobind: leaked 1 instances!\n" + " - leaked instance 0x0 of type 'X'\n" * 40


def test_keeps_the_traceback_ahead_of_the_leak_report():
    shown = child_stderr(_TRACEBACK + _LEAK_REPORT)
    assert shown.startswith("Traceback (most recent call last):")
    assert "IndexError: boom" in shown


def test_without_a_traceback_keeps_all_of_stderr():
    assert child_stderr(_LEAK_REPORT) == _LEAK_REPORT
