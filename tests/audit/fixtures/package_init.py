"""Fixture package __init__: one defined class, one fail-loud violation, one re-export, one
PEP 562 lazy re-export hook."""

try:
    from fixture import extra  # noqa: F401
except ImportError:
    pass

from fixture import Widget  # noqa: F401  (re-export: not a finding)


class FilesystemPath(str):
    pass


def __getattr__(name):
    if name == "Lazy":
        from fixture import Lazy

        return Lazy
    raise AttributeError(name)
