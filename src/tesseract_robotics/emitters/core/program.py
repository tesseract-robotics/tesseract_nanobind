"""`EmittedProgram`: the output of an emit, filename-to-text plus `write_to`.

A brand emit may produce one file (KRL `.src`) or several. `.text` is a
convenience for the single-file case and raises otherwise. `identity` is the
content-addressed identity of the emit, kept on the result for every dialect,
including those whose comment syntax cannot carry it in the file.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .errors import ProgramFileCountError
from .identity import EmitIdentity


@dataclass(frozen=True)
class EmittedProgram:
    """Immutable emit result: `files` maps filename to contents."""

    identity: EmitIdentity
    files: dict[str, str] = field(default_factory=dict)

    @property
    def filenames(self) -> list[str]:
        """The emitted filenames, in emit order."""
        return list(self.files.keys())

    @property
    def text(self) -> str:
        """The single file's text.

        Raises:
            ProgramFileCountError: the program does not hold exactly one file.
        """
        if len(self.files) != 1:
            raise ProgramFileCountError(self.filenames)
        return next(iter(self.files.values()))

    def write_to(self, directory: Path | str) -> list[Path]:
        """Write every file into `directory` (created if absent). Returns the paths."""
        out = Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        written: list[Path] = []
        for name, text in self.files.items():
            path = out / name
            # Explicit UTF-8: Windows defaults write_text to the locale codec
            # (cp1252), which mangles emitted programs. Robot code stays UTF-8.
            path.write_text(text, encoding="utf-8")
            written.append(path)
        return written
