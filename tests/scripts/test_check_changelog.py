"""The changelog contract: release structure and the size of every searchable section."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "check_changelog.py"
spec = importlib.util.spec_from_file_location("check_changelog", SCRIPT)
assert spec is not None and spec.loader is not None
check_changelog = importlib.util.module_from_spec(spec)
sys.modules["check_changelog"] = check_changelog
spec.loader.exec_module(check_changelog)

FOOTER = (
    "\n[Unreleased]: https://github.com/o/r/compare/1.0.0...HEAD\n"
    "[1.0.0]: https://github.com/o/r/compare/0.9.0...1.0.0\n"
)


def changelog(body: str) -> str:
    return f"# Changelog\n\n## [Unreleased]\n\n## [1.0.0] - 2026-01-01\n\n{body}\n{FOOTER}"


def rules(text: str) -> set:
    return {violation.rule for violation in check_changelog.check(text)}


def test_small_sections_keep_the_contract() -> None:
    assert check_changelog.check(changelog("### Changes\n\n- one\n")) == []


def test_section_over_the_search_budget_is_a_violation() -> None:
    text = changelog("### Changes\n\n" + "- " + "x" * check_changelog.SECTION_BUDGET + "\n")
    violations = [
        v for v in check_changelog.check(text) if v.rule is check_changelog.Rule.SECTION_TOO_LARGE
    ]
    assert len(violations) == 1
    assert "Changes" in violations[0].detail
    assert str(check_changelog.SEARCH_FIELD_LIMIT) in violations[0].detail


def test_subheadings_split_a_large_section() -> None:
    half = "- " + "x" * (check_changelog.SECTION_BUDGET // 2) + "\n"
    text = changelog(f"### Changes\n\n#### A\n\n{half}\n#### B\n\n{half}")
    assert check_changelog.Rule.SECTION_TOO_LARGE not in rules(text)


def test_heading_lookalikes_in_code_fences_do_not_split_a_section() -> None:
    filler = "- " + "x" * check_changelog.SECTION_BUDGET + "\n"
    text = changelog(f"### Changes\n\n```bash\n# not a heading\n```\n\n{filler}")
    assert check_changelog.Rule.SECTION_TOO_LARGE in rules(text)


def test_the_repo_changelog_keeps_the_contract() -> None:
    text = check_changelog.DEFAULT_CHANGELOG.read_text(encoding="utf-8")
    assert check_changelog.check(text) == []
