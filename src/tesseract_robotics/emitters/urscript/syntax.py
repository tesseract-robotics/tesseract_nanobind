"""URScript lexical rules and the frame binding.

Source: Universal Robots Script Directory, PolyScope 5.26 e-Series:
`get_inverse_kin(x, qnear, ...)`: "If qnear is defined, the solution closest to
qnear is returned. Otherwise, the solution closest to the current joint
positions is returned"; `set_tcp(pose)`: "the transformation from the output
flange coordinate system to the TCP as a pose"; `set_standard_analog_out(n, f)`:
"n: integer [0:1]", "f: The relative signal level [0;1]".

URScript has no named work frames: a feature is a pose in the base frame, so the
binding maps each working frame to that pose and the emitter transforms targets
into the base frame exactly. Identifiers follow the Python-like URScript
grammar: a letter or `_`, then letters, digits or `_`, never a keyword.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from typing import Final

from tesseract_robotics.planning.transforms import Pose

from ..core.frames import FrameBinding
from ..core.naming import IdentifierRules
from ..core.text import single_line

BRAND: Final[str] = "URScript"

_KEYWORDS: Final[frozenset[str]] = frozenset(
    """
    and break continue def elif else end False for global halt if local not or
    pass return run sec thread True while xor
    """.split()
)

UR_IDENTIFIERS: Final[IdentifierRules] = IdentifierRules(
    brand=BRAND,
    pattern=re.compile(r"[A-Za-z_][A-Za-z0-9_]*"),
    pattern_text="a letter or '_' followed by letters, digits or '_'",
    max_len=None,
    reserved=_KEYWORDS,
    case_insensitive=False,
)

#: Standard analog outputs `set_standard_analog_out(n, f)`: n in [0:1].
ANALOG_PORTS: Final[range] = range(0, 2)
#: Relative analog signal level range [0; 1].
ANALOG_MIN: Final[float] = 0.0
ANALOG_MAX: Final[float] = 1.0


def ur_frames(
    *, features: Mapping[str, Pose], tcps: Mapping[str, Pose]
) -> FrameBinding[Pose, Pose]:
    """Bind tesseract frames to URScript poses.

    Args:
        features: Working-frame link name to that frame's pose in the robot
            base frame (identity for the base itself).
        tcps: TCP link name to the TCP pose in the output-flange frame
            (identity for the flange itself); emitted with `set_tcp`.
    """
    return FrameBinding.build(work_frames=features, tools=tcps)


def comment_text(text: str) -> str:
    """Validate `text` as a `#` comment body (no line breaks)."""
    return single_line(text, brand=BRAND)
