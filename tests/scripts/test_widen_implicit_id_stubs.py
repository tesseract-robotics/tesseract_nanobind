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

Q = "tesseract_robotics.common._common."


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


def test_id_classes_widen_their_own_parameters():
    # str -> LinkId and tuple -> LinkIdPair convert implicitly in the id classes' own methods
    # too: LinkIdPair("a", "b"), LinkId("a") < "b". Their __eq__ / __ne__ take `object`.
    stub = (
        "class LinkIdPair:\n"
        "    def __init__(self, link_id1: LinkId, link_id2: LinkId) -> None: ...\n"
        "    def __init__(self, pair: tuple[LinkId, LinkId]) -> None: ...\n"
        "    def __eq__(self, other: object, /) -> bool: ...\n"
        "class LinkId:\n"
        "    def __lt__(self, arg: LinkId, /) -> bool: ...\n"
    )
    assert widen(stub) == (
        "class LinkIdPair:\n"
        "    def __init__(self, link_id1: LinkId | str, link_id2: LinkId | str) -> None: ...\n"
        "    def __init__(self, pair: tuple[LinkId | str, LinkId | str]) -> None: ...\n"
        "    def __eq__(self, other: object, /) -> bool: ...\n"
        "class LinkId:\n"
        "    def __lt__(self, arg: LinkId | str, /) -> bool: ...\n"
    )


def test_idempotent():
    line = "    def f(self, a: LinkId, p: LinkIdPair) -> None: ...\n"
    assert widen(widen(line)) == widen(line)


def test_a_multi_line_def_widens_across_its_lines():
    stub = "    def interpolate(\n        self, a: LinkId,\n        b: int\n    ) -> LinkId: ...\n"
    assert widen(stub) == (
        "    def interpolate(\n        self, a: LinkId | str,\n        b: int\n    ) -> LinkId: ...\n"
    )


def test_mapping_params_accept_an_id_keyed_dict_and_a_str_keyed_one():
    # Mapping keys are invariant: `Mapping[LinkId | str, X]` would reject the dict[LinkId, X]
    # the bindings hand out (SceneState.link_transforms), so the parameter becomes a union.
    line = f"    def setTransforms(self, t: Mapping[{Q}LinkId, Isometry3d], tip: {Q}LinkId) -> None: ...\n"
    assert widen(line) == (
        f"    def setTransforms(self, t: Mapping[{Q}LinkId, Isometry3d] | Mapping[str, Isometry3d], "
        f"tip: {Q}LinkId | str) -> None: ...\n"
    )


def test_nested_and_pair_keyed_mappings_widen():
    line = (
        f"    def f(self, s: Mapping[str, Mapping[{Q}JointId, float]], "
        f"c: Mapping[{Q}LinkIdPair, float]) -> None: ...\n"
    )
    assert widen(line) == (
        f"    def f(self, s: Mapping[str, Mapping[{Q}JointId, float] | Mapping[str, float]], "
        f"c: Mapping[{Q}LinkIdPair, float] | Mapping[tuple[str, str], float]) -> None: ...\n"
    )


def test_widened_mapping_is_a_fixed_point():
    line = f"    def f(self, m: Mapping[{Q}JointId, Mapping[{Q}LinkId, float]]) -> None: ...\n"
    once = widen(line)
    assert widen(once) == once


def test_returned_mapping_stays_exact():
    line = f"    def get(self) -> dict[{Q}JointId, float]: ...\n"
    assert widen(line) == line
