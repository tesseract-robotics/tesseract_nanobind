"""What a failed child-process assertion shows of the child's stderr."""


def child_stderr(stderr: str) -> str:
    """The child's stderr from its first traceback on, or all of it when there is none.

    Every child prints nanobind's leak report (about 1.4 kB) at exit, after any traceback, so a
    tail such as `stderr[-500:]` shows only the report.

    Args:
        stderr: The child's captured stderr.

    Returns:
        `stderr` from the first `Traceback`, or `stderr` unchanged.
    """
    start = stderr.find("Traceback")
    return stderr if start < 0 else stderr[start:]
