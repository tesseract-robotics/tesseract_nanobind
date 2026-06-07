"""Fanuc LS (TP ASCII) code-generation DSL — class-per-``/MN``-line, write-on-construct.

A thin specialization of the shared ``core.dsl`` skeleton (the twin of
``emitters.krl.krl_writer``). The wrinkle versus KRL/RAPID is that a Fanuc ``.ls``
file has **two** sections: a logic section ``/MN`` (numbered instruction lines that
reference positions by index ``P[i]``) and a separate ``/POS`` position table.

This module owns only the ``/MN`` body buffer. Each statement class writes one
already-numbered ``/MN`` line on instantiation, so the backend reads like the
target language —

```python
JointMoveLine(1, index=1, speed_percent=100.0, termination="FINE")
LinearMoveLine(3, index=3, speed_mms=100.0, termination="FINE")
```

The ``/POS`` table is modelled separately as a list of :class:`PosEntry` records the
backend accumulates and renders at ``prog_finish`` (positions are indexed, not
nested, so they cannot be a ``with``-scope). ``LsBackend`` drives both halves from
the brand-neutral IR exactly as ``KrlBackend`` drives the KRL DSL.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from ..core.dsl import Command, Writer
from .targets import comment_line, instruction_line, joint_move_line, linear_move_line


class LsWriter(Writer):
    """Fanuc ``/MN`` body buffer: lines are pre-numbered, so no structural indent.

    ``_tab`` is empty (each statement embeds its own ``   <n>:`` prefix via the
    ``targets`` formatters) and there is no leading newline.
    """

    _tab: ClassVar[str] = ""
    _leading_newline: ClassVar[bool] = False


class LsCommand(Command):
    """Base for every Fanuc LS DSL class; bound to the ``LsWriter`` singleton."""

    writer: ClassVar[LsWriter] = LsWriter()


class CommentLine(LsCommand):
    """``<n>:! <text> ;`` — a numbered Fanuc remark line in ``/MN``."""

    def __init__(self, line_no: int, text: str) -> None:
        self.writer.write(comment_line(line_no, text))


class JointMoveLine(LsCommand):
    """``<n>:J P[i] <pct>% <term> ;`` — a numbered joint move in ``/MN``."""

    def __init__(self, line_no: int, *, index: int, speed_percent: float, termination: str) -> None:
        self.writer.write(joint_move_line(line_no, index, speed_percent, termination))


class LinearMoveLine(LsCommand):
    """``<n>:L P[i] <mm>mm/sec <term> ;`` — a numbered linear move in ``/MN``."""

    def __init__(self, line_no: int, *, index: int, speed_mms: float, termination: str) -> None:
        self.writer.write(linear_move_line(line_no, index, speed_mms, termination))


class InstructionLine(LsCommand):
    """``<n>:<body> ;`` — a numbered non-motion ``/MN`` line (dwell, I/O, tool).

    The ``body`` is the controller statement without the line-number prefix or
    trailing `` ;`` (e.g. ``WAIT 2.5(sec)``, ``DO[3]=ON``, ``UTOOL_NUM=1``); the
    DSL owns the numbering + termination framing so the backend never writes raw.
    """

    def __init__(self, line_no: int, body: str) -> None:
        self.writer.write(instruction_line(line_no, body))


@dataclass(frozen=True)
class PosEntry:
    """One rendered ``/POS`` entry: its index and the full literal block text.

    The block text is produced by ``targets.joint_pos_block`` /
    ``targets.cartesian_pos_block`` and spliced verbatim into the ``/POS`` section.
    Kept as a record (not a ``Command``) because positions form an indexed table,
    not a nested scope.
    """

    index: int
    block: str
