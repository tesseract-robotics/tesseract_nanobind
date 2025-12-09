"""Unified context holder for tesseract test object lifetimes.

C++ bindings via nanobind require objects to be deleted in a specific order -
parent objects must outlive child objects that reference them. Python's
non-deterministic GC causes segfaults when objects go out of scope arbitrarily.

This module provides TesseractContext to ensure proper cleanup order.
"""
import gc
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TesseractContext:
    """Hold C++ objects to prevent premature GC.

    Usage:
        ctx = TesseractContext()
        locator = ctx.keep(GeneralResourceLocator())
        env = Environment()
        env.init(urdf, srdf, locator)
        # use env...
        ctx.cleanup()  # explicit cleanup in reverse order

    Or as context manager:
        with TesseractContext() as ctx:
            locator = ctx.keep(GeneralResourceLocator())
            env = Environment()
            env.init(urdf, srdf, locator)
            # use env...
        # cleanup automatic on exit
    """
    _refs: list = field(default_factory=list)

    def keep(self, obj: Any) -> Any:
        """Keep object alive. Returns the object for chaining."""
        self._refs.append(obj)
        return obj

    def cleanup(self):
        """Cleanup in reverse order (LIFO)."""
        # Delete in reverse order to ensure proper cleanup
        while self._refs:
            self._refs.pop()
        gc.collect()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.cleanup()
