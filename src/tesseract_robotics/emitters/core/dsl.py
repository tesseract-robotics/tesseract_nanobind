"""The shared DSL skeleton — the domain every brand's hand-authoring DSL captures.

Three abstractions, specialized once per brand:

- ``Writer`` — an indented text-buffer **singleton per subclass**. Subclasses
  set the tab width and whether the buffer opens with a leading newline; they
  may extend per-instance state by overriding ``_setup``.
- ``Command`` — the base for every statement/scope class, bound to its brand's
  writer via the ``writer`` class attribute. A statement writes itself into the
  shared buffer on instantiation; this is what lets ``MoveL(target, profile)``
  or ``Ptp(axes)`` read like the target language.
- ``Block`` — a ``<opening> … <closing>`` indented scope driven by ``with``.

``rapid_writer`` and ``krl_writer`` are thin specializations of these. The
generic machinery (singleton lifetime, indentation, newline policy, the
write-on-construct convention) lives here exactly once.
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from io import StringIO
from typing import ClassVar


class Writer:
    """Indented code-buffer with one singleton instance *per subclass*.

    A plain ``ClassVar`` ``_instance`` would be shared across subclasses, so
    ``RapidWriter()`` and ``KrlWriter()`` would collide; instead instances are
    keyed by concrete class in ``_instances``. Every ``Command`` captures its
    brand's instance at import time, so nested ``with`` blocks all manipulate the
    same indent level and buffer.
    """

    #: Indentation unit (brand override: RAPID 4 spaces, KRL 2).
    _tab: ClassVar[str] = "  "
    #: Whether the buffer opens with a leading newline (RAPID does, KRL does not).
    _leading_newline: ClassVar[bool] = False
    _instances: ClassVar[dict[type, Writer]] = {}

    buffer: StringIO
    _indent: int

    def __new__(cls) -> Writer:
        instance = Writer._instances.get(cls)
        if instance is None:
            instance = super().__new__(cls)
            instance._setup()
            Writer._instances[cls] = instance
        return instance

    def _setup(self) -> None:
        """Initialize per-instance state. Override to add brand state, calling
        ``super()._setup()`` first; ``clear()`` re-runs this."""
        self.buffer = StringIO()
        self._indent = 0

    def indent(self) -> None:
        self._indent += 1

    def dedent(self) -> None:
        self._indent -= 1

    def write(self, cmd: str) -> None:
        prefix = "\n" if (self._leading_newline or self.buffer.tell() != 0) else ""
        self.buffer.write(prefix + self._tab * self._indent + cmd)

    def getvalue(self) -> str:
        return self.buffer.getvalue()

    def clear(self) -> None:
        """Reset the buffer + indent (and any subclass state) in place.

        ``Command`` captured this instance at import time, so mutate — never
        reassign the singleton, which would orphan the captured reference.
        """
        self._setup()


class Command:
    """Base for every DSL statement/scope class.

    A brand binds ``writer`` to its concrete ``Writer`` singleton; subclasses
    write through ``self.writer``. Re-annotate ``writer`` with the concrete
    type in the brand base so brand-specific writer methods type-check.
    """

    writer: ClassVar[Writer]


class Block(Command, AbstractContextManager["Block"]):
    """An indented ``<opening> … <closing>`` scope.

    ``__init__`` writes the opening line; the ``with`` body is indented; exit
    dedents and writes the closing line. Brand scope classes (RAPID ``Module`` /
    ``Proc``, KRL ``Def``) subclass this and pass their keywords.
    """

    _closing: str

    def __init__(self, opening: str, closing: str) -> None:
        self._closing = closing
        self.writer.write(opening)

    def __enter__(self) -> Block:
        self.writer.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.writer.dedent()
        self.writer.write(self._closing)
