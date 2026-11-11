"""The shared DSL skeleton: the machinery every brand's hand-authoring DSL specializes.

Three abstractions, specialized once per brand:

- `Writer`: an indented text buffer. A writer is a context manager; while its
  `with` block is open it is the active writer of its brand in the current
  context (a `contextvars.ContextVar` per writer class), so statements know where
  to write without a process-wide singleton. Every emit opens its own writer, so
  an emit never touches a DSL program the caller is building, and two threads
  (or asyncio tasks) never share a buffer.
- `Command`: the base for every statement/scope class. A statement writes itself
  into the active writer of its brand on instantiation; this is what lets
  `MoveL(target, profile)` or `Ptp(axes)` read like the target language.
- `Block`: a `<opening> ... <closing>` indented scope driven by `with`.

```python
with RapidWriter() as rapid:
    with Module("M"):
        with Proc("main"):
            MoveAbsJ(target, profile, tool0)
text = rapid.getvalue()
```
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from contextvars import ContextVar, Token
from io import StringIO
from typing import ClassVar, Self

from .errors import NoActiveWriterError


class Writer(AbstractContextManager["Writer"]):
    """Indented code buffer, active for its brand while its `with` block is open.

    Subclasses set the tab width and whether the buffer opens with a leading
    newline; each subclass gets its own context variable, so a RAPID writer and a
    KRL writer can be active at the same time.
    """

    #: Indentation unit (brand override: RAPID 4 spaces, KRL 2).
    _tab: ClassVar[str] = "  "
    #: Whether the buffer opens with a leading newline (RAPID does, KRL does not).
    _leading_newline: ClassVar[bool] = False
    _active: ClassVar[ContextVar[Writer]]

    def __init_subclass__(cls, **kwargs: object) -> None:
        super().__init_subclass__(**kwargs)
        cls._active = ContextVar(f"{cls.__name__}_active")

    def __init__(self) -> None:
        self.buffer = StringIO()
        self._indent = 0
        self._tokens: list[Token[Writer]] = []

    def __enter__(self) -> Self:
        self._tokens.append(type(self)._active.set(self))
        return self

    def __exit__(self, *_: object) -> None:
        type(self)._active.reset(self._tokens.pop())

    @classmethod
    def current(cls) -> Self:
        """The active writer of this class in the current context.

        Raises:
            NoActiveWriterError: no writer of this class is open.
        """
        writer = cls._active.get(None)
        if writer is None:
            raise NoActiveWriterError(cls.__name__)
        assert isinstance(writer, cls)
        return writer

    def indent(self) -> None:
        self._indent += 1

    def dedent(self) -> None:
        self._indent -= 1

    def write(self, cmd: str) -> None:
        """Append one line at the current indent."""
        prefix = "\n" if (self._leading_newline or self.buffer.tell() != 0) else ""
        self.buffer.write(prefix + self._tab * self._indent + cmd)

    def getvalue(self) -> str:
        return self.buffer.getvalue()


class Command:
    """Base for every DSL statement/scope class.

    A brand binds `writer_type` to its concrete `Writer` subclass; `writer`
    resolves the active instance at the moment the statement is written.
    """

    writer_type: ClassVar[type[Writer]]

    @property
    def writer(self) -> Writer:
        return self.writer_type.current()


class Block(Command, AbstractContextManager["Block"]):
    """An indented `<opening> ... <closing>` scope.

    `__init__` writes the opening line; the `with` body is indented; exit
    dedents and writes the closing line. Brand scope classes (RAPID `Module` /
    `Proc`, KRL `Def`) subclass this and pass their keywords.
    """

    _closing: str

    def __init__(self, opening: str, closing: str) -> None:
        self._closing = closing
        self.writer.write(opening)

    def __enter__(self) -> Self:
        self.writer.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.writer.dedent()
        self.writer.write(self._closing)
