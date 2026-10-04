"""Fixture package __init__: one defined class, one fail-loud violation, one re-export."""

try:
    from fixture import extra  # noqa: F401
except ImportError:
    pass

from fixture import Widget  # noqa: F401  (re-export: not a finding)


class FilesystemPath(str):
    pass
