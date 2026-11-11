"""Parse-back contract: emitted JBI pulse rows invert to the input joint angles.

Each `MOVJ C#####` is resolved to its `C#####=` row; every entry must be an
integer pulse count, and `(pulse - zero) / pulses_per_degree` must give back the
input angle within half a pulse.
"""

from __future__ import annotations

import re

import numpy as np

from tesseract_robotics.emitters.jbi import emit_jbi
from tests.emitters.jbi.spike import (
    CALIBRATION,
    FRAMES,
    PULSES_PER_DEG,
    ROWS_DEG,
    ZERO_PULSES,
    spike_composite,
    spike_profiles,
)


def test_emitted_jbi_parses_back_to_inputs() -> None:
    jbi = emit_jbi(spike_composite(), spike_profiles(), frames=FRAMES, calibration=CALIBRATION).text
    table = {
        m.group(1): m.group(2).split(",")
        for m in re.finditer(r"^(C\d{5})=(.+)$", jbi, flags=re.MULTILINE)
    }
    refs = re.findall(r"MOVJ (C\d{5}) VJ=", jbi)
    assert len(refs) == len(ROWS_DEG)
    ppd = np.asarray(PULSES_PER_DEG)
    for ref, expected in zip(refs, ROWS_DEG, strict=True):
        assert all(re.fullmatch(r"-?\d+", tok) for tok in table[ref])
        pulses = np.array([int(tok) for tok in table[ref]])
        angles = (pulses - np.asarray(ZERO_PULSES)) / ppd
        assert np.all(np.abs(angles - expected) <= 0.5 / np.abs(ppd) + 1e-12)
    assert "///POSTYPE PULSE" in jbi and "///TOOL 0" in jbi
