"""Fixture package __init__: `refuse_hook_overrides` installs `__init_subclass__` on a class
named directly, and on each class of a literal loop."""

from fixture import Lone, LoopA, LoopB, Unguarded, refuse_hook_overrides  # noqa: F401

refuse_hook_overrides(Lone, ("generate",), "Subclass the trampolined base.")

for _concrete in (LoopA, LoopB):
    refuse_hook_overrides(_concrete, ("generate",), "Subclass the trampolined base.")
del _concrete
