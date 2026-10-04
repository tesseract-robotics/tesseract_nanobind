"""Contract tests for scripts/widen_implicit_id_stubs.py.

Fixture lines are copied from stubgen output for the upstream id API (#141).
"""

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_SCRIPT = REPO_ROOT / "scripts" / "widen_implicit_id_stubs.py"

_spec = importlib.util.spec_from_file_location("widen_implicit_id_stubs", _SCRIPT)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
widen = _mod.widen

Q = "tesseract_robotics.tesseract_common._tesseract_common."


def test_qualified_parameter_ids_accept_str():
    line = f"    def __init__(self, id: {Q}LinkId) -> None: ...\n"
    assert widen(line) == f"    def __init__(self, id: {Q}LinkId | str) -> None: ...\n"


def test_return_type_stays_exact():
    line = f"    def getBaseLinkId(self, x: {Q}JointId) -> {Q}LinkId: ...\n"
    assert widen(line) == f"    def getBaseLinkId(self, x: {Q}JointId | str) -> {Q}LinkId: ...\n"


def test_sequences_and_pairs_widen_inside_brackets():
    line = "    def f(self, ids: Sequence[JointId], pair: LinkIdPair) -> dict[LinkId, int]: ...\n"
    assert widen(line) == (
        "    def f(self, ids: Sequence[JointId | str], "
        "pair: LinkIdPair | tuple[str, str]) -> dict[LinkId, int]: ...\n"
    )


def test_annotated_parentheses_do_not_end_the_parameter_list():
    line = (
        "def g(scale: Annotated[NDArray, dict(shape=(3), order='C')], tip: LinkId) -> LinkId: ...\n"
    )
    assert widen(line) == (
        "def g(scale: Annotated[NDArray, dict(shape=(3), order='C')], tip: LinkId | str) -> LinkId: ...\n"
    )


def test_id_classes_keep_their_own_signatures_and_widening_resumes_after():
    stub = (
        "class LinkId:\n"
        "    def __eq__(self, arg: LinkId, /) -> bool: ...\n"
        "\n"
        "def top(a: LinkId) -> None: ...\n"
        "class Link:\n"
        "    def __init__(self, id: LinkId) -> None: ...\n"
    )
    assert widen(stub) == (
        "class LinkId:\n"
        "    def __eq__(self, arg: LinkId, /) -> bool: ...\n"
        "\n"
        "def top(a: LinkId | str) -> None: ...\n"
        "class Link:\n"
        "    def __init__(self, id: LinkId | str) -> None: ...\n"
    )


def test_idempotent():
    line = "    def f(self, a: LinkId, p: LinkIdPair) -> None: ...\n"
    assert widen(widen(line)) == widen(line)


def test_a_multi_line_def_widens_across_its_lines():
    stub = "    def interpolate(\n        self, a: LinkId,\n        b: int\n    ) -> LinkId: ...\n"
    assert widen(stub) == (
        "    def interpolate(\n        self, a: LinkId | str,\n        b: int\n    ) -> LinkId: ...\n"
    )


def test_mapping_keys_stay_exact_so_an_id_keyed_dict_passes_back_in():
    # Mapping keys are invariant: widening them would reject SceneState.link_transforms.
    line = f"    def setTransforms(self, t: Mapping[{Q}LinkId, Isometry3d], tip: {Q}LinkId) -> None: ...\n"
    assert widen(line) == (
        f"    def setTransforms(self, t: Mapping[{Q}LinkId, Isometry3d], tip: {Q}LinkId | str) -> None: ...\n"
    )
