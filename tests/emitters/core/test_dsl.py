"""Writer contexts: per-emit buffers, no singleton, no cross-thread sharing."""

from __future__ import annotations

import threading

import pytest

from tesseract_robotics.emitters.core.errors import NoActiveWriterError
from tesseract_robotics.emitters.krl import KrlProfile, emit_krl, krl_frames
from tesseract_robotics.emitters.krl.krl_writer import Comment, Def, KrlWriter
from tests.emitters.programs import composite, joint_instruction

_FRAMES = krl_frames(bases={}, tools={"tool0": 1})


def test_statement_without_active_writer_raises() -> None:
    with pytest.raises(NoActiveWriterError, match="KrlWriter"):
        Comment("orphan")


def test_emit_does_not_touch_an_open_user_program() -> None:
    with KrlWriter() as mine:
        with Def("MINE"):
            Comment("before")
            emit_krl(
                composite(joint_instruction([0.1] * 6)),
                {"J": KrlProfile.build()},
                frames=_FRAMES,
                program_name="OTHER",
            )
            Comment("after")
    assert mine.getvalue() == "DEF MINE()\n  ; before\n  ; after\nEND"


def test_threads_write_to_their_own_buffers() -> None:
    results: dict[int, str] = {}
    barrier = threading.Barrier(2)

    def work(n: int) -> None:
        with KrlWriter() as w:
            barrier.wait()
            for _ in range(200):
                Comment(f"t{n}")
            results[n] = w.getvalue()

    threads = [threading.Thread(target=work, args=(n,)) for n in (1, 2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert set(results[1].split("\n")) == {"; t1"}
    assert set(results[2].split("\n")) == {"; t2"}
