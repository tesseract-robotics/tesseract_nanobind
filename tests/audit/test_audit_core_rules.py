"""Real-row tests of the accepted rules on core modules decided in #256 (C12).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

from tests.audit._load import audit

COLLISION = "tesseract_collision"
FLATTEN_WRAPPER = "ContactResultMap.flattenWrapperResults"
REFERENCE_WRAPPER_OUT_PARAM = "reference-wrapper-out-param"


def test_flatten_wrapper_results_accepted():
    """C12 (accept by explicit entry; phaseB won't-fix): both const overloads are one accepted
    symbol, never a gap."""
    report = audit.audit_module(COLLISION)
    rules = {a.rule for a in report.accepted if a.symbol == FLATTEN_WRAPPER}
    assert rules == {REFERENCE_WRAPPER_OUT_PARAM}
    assert FLATTEN_WRAPPER not in {g.symbol for g in report.gaps}
    assert (COLLISION, FLATTEN_WRAPPER) in audit.ACCEPTED_SYMBOLS
    assert REFERENCE_WRAPPER_OUT_PARAM in audit.ACCEPTED
