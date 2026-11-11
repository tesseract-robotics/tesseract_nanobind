"""The provenance the report tests render: one shape, the one `audit_report.provenance` returns.

`render_markdown` and `to_json` read exactly these keys (`AUDITED_UPSTREAM`, then `libclang` and
`stubs`); the values are fakes that each test can find in the rendered page.
"""

PROV = {
    "tesseract": "4cf644d7",
    "tesseract_planning": "4e27d94b",
    "trajopt": "98b85d55",
    "libclang": "clang version 23",
    "stubs": "abc1234",
}
