"""``EmittedProgram`` — the output of an emit: filename→text + ``write_to``.

A brand emit may produce one file (KRL ``.src``) or several (later: KUKA
``.src``+``.dat``, Fanuc pages). ``.text`` is a convenience for the single-file
case and raises otherwise.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class EmittedProgram:
    """Immutable emit result: ``files`` maps filename → contents."""

    files: dict[str, str] = field(default_factory=dict)

    @property
    def filenames(self) -> list[str]:
        return list(self.files.keys())

    @property
    def text(self) -> str:
        """The single file's text. Raises ``ValueError`` if not exactly one."""
        if len(self.files) != 1:
            raise ValueError(
                f".text requires exactly one file; this has {len(self.files)}: "
                f"{self.filenames}. Use .files[<name>]."
            )
        return next(iter(self.files.values()))

    def write_to(self, directory: Path | str) -> list[Path]:
        """Write every file into ``directory`` (created if absent). Returns paths."""
        out = Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        written: list[Path] = []
        for name, text in self.files.items():
            path = out / name
            path.write_text(text)
            written.append(path)
        return written
