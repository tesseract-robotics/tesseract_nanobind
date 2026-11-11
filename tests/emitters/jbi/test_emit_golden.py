"""End-to-end JBI emit golden (this emitter's output, hand-reviewed)."""

from __future__ import annotations

import difflib
from pathlib import Path

from tesseract_robotics.emitters.jbi import emit_jbi
from tests.emitters.jbi.spike import CALIBRATION, FRAMES, spike_composite, spike_profiles

GOLDEN = Path(__file__).parent / "golden" / "spike.jbi"


def emit_spike() -> str:
    return emit_jbi(
        spike_composite(), spike_profiles(), frames=FRAMES, calibration=CALIBRATION
    ).text


def test_emit_jbi_matches_golden() -> None:
    actual = emit_spike()
    expected = GOLDEN.read_text()
    diff = "\n".join(
        difflib.unified_diff(
            expected.splitlines(), actual.splitlines(), "golden", "emit_jbi", lineterm=""
        )
    )
    assert actual == expected, f"diverged from golden:\n{diff}"
