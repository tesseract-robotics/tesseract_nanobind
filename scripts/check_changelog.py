"""Check CHANGELOG.md's release structure before the docs site publishes it.

The changelog follows [Keep a Changelog](https://keepachangelog.com/): a release
is cut by renaming `## [Unreleased]` to `## [<version>] - <YYYY-MM-DD>` and
pointing the compare links at it. 0.35.0.8 was cut as a second `## [0.35.0.7]`
heading; the docs build rendered that without complaint, and the published
changelog never named 0.35.0.8. The Docs workflow runs this check before it
builds, so a broken cut fails CI instead of publishing.

The contract, one `Rule` per clause:

1. `## [Unreleased]` is the first level-2 heading, and the only Unreleased one.
2. Every other level-2 heading is a release, `## [<version>] - <YYYY-MM-DD>`.
3. Release versions are unique and listed newest first.
4. Each link name is defined once.
5. Each release has a `[<version>]:` link, and each version link has a release.
6. `[Unreleased]:` compares the newest release with `HEAD`.
7. `[<version>]:` compares the next-older release with `<version>`; the oldest
   release compares from any older version (one that predates the changelog).

Usage:

```bash
python scripts/check_changelog.py                       # the repo's CHANGELOG.md
python scripts/check_changelog.py path/to/CHANGELOG.md
```
"""

from __future__ import annotations

import argparse
import datetime
import enum
import re
import sys
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

Version = tuple[int, ...]

DEFAULT_CHANGELOG = Path(__file__).resolve().parents[1] / "CHANGELOG.md"
UNRELEASED = "Unreleased"
HEAD = "HEAD"

LEVEL2_HEADING = re.compile(r"^## (?P<text>.*)$")
BRACKETED = re.compile(r"^\[(?P<name>[^\]]+)\](?P<rest>.*)$")
RELEASE_DATE = re.compile(r"^ - (?P<date>\d{4}-\d{2}-\d{2})(?=\s|$)")
# Markdown matches reference labels case-insensitively, so names are keyed lower-cased.
LINK_DEFINITION = re.compile(r"^\[(?P<name>[^\]]+)\]:\s+(?P<url>\S+)")
VERSION = re.compile(r"^\d+(?:\.\d+)+$")
COMPARE = re.compile(r"/compare/(?P<base>[^/]+?)\.\.\.(?P<head>[^/]+)$")


class Rule(enum.Enum):
    """One member per contract clause, so a failure names the clause it broke."""

    UNRELEASED_NOT_FIRST = "`## [Unreleased]` is the first level-2 heading"
    DUPLICATE_UNRELEASED = "there is one `## [Unreleased]` heading"
    RELEASE_WITHOUT_VERSION = "a release heading names its version, `## [<version>]`"
    RELEASE_WITHOUT_DATE = "a release heading carries its date, `## [<version>] - <YYYY-MM-DD>`"
    DUPLICATE_RELEASE = "each version is released once"
    RELEASES_NOT_DESCENDING = "releases are listed newest first"
    DUPLICATE_LINK = "each link name is defined once"
    RELEASE_WITHOUT_LINK = "each release has a `[<version>]:` compare link"
    LINK_WITHOUT_RELEASE = "each version link has a release heading"
    UNRELEASED_LINK_MISMATCH = "`[Unreleased]:` compares the newest release with HEAD"
    COMPARE_CHAIN_BROKEN = "`[<version>]:` compares the next-older release with <version>"


@dataclass(frozen=True)
class Violation:
    """A broken contract clause at a 1-based changelog line."""

    rule: Rule
    line: int
    detail: str


@dataclass(frozen=True)
class Heading:
    """A level-2 heading: its line and the text after `## `."""

    line: int
    text: str


@dataclass(frozen=True)
class Link:
    """A reference-link definition, `[name]: url`."""

    line: int
    name: str
    url: str


@dataclass(frozen=True)
class Release:
    """A level-2 heading that names a version; `date` is its ` - <date>` text, if any."""

    line: int
    name: str
    version: Version
    date: str | None


def parse_version(name: str) -> Version:
    """The integer parts of a dotted version, e.g. `0.35.0.8` -> `(0, 35, 0, 8)`."""
    return tuple(int(part) for part in name.split("."))


def heading_name(heading: Heading) -> str | None:
    """The bracketed name a heading opens with, or None if it has none."""
    match = BRACKETED.match(heading.text)
    return match["name"] if match else None


def compare_range(url: str) -> tuple[str, str] | None:
    """The `(base, head)` of a GitHub compare URL, or None if `url` is not one."""
    match = COMPARE.search(url)
    return (match["base"], match["head"]) if match else None


def describe_range(link: Link) -> str:
    """How a violation message shows what a link actually compares."""
    compared = compare_range(link.url)
    return f"{compared[0]}...{compared[1]}" if compared else f"no compare range ({link.url})"


def parse(text: str) -> tuple[list[Heading], list[Link]]:
    """Split a changelog into its level-2 headings and its link definitions.

    Args:
        text: The changelog's Markdown.

    Returns:
        The headings and the link definitions, each in line order.
    """
    headings: list[Heading] = []
    links: list[Link] = []
    for number, line in enumerate(text.splitlines(), start=1):
        heading = LEVEL2_HEADING.match(line)
        link = LINK_DEFINITION.match(line)
        if heading:
            headings.append(Heading(number, heading["text"]))
        elif link:
            links.append(Link(number, link["name"], link["url"]))
    return headings, links


def check_unreleased(headings: Sequence[Heading]) -> Iterator[Violation]:
    """Clause 1: `## [Unreleased]` comes first, once."""
    if not headings or heading_name(headings[0]) != UNRELEASED:
        line = headings[0].line if headings else 1
        yield Violation(
            Rule.UNRELEASED_NOT_FIRST, line, "the first level-2 heading is not `## [Unreleased]`"
        )
    unreleased = [heading for heading in headings if heading_name(heading) == UNRELEASED]
    for duplicate in unreleased[1:]:
        yield Violation(
            Rule.DUPLICATE_UNRELEASED,
            duplicate.line,
            f"another `## [Unreleased]` (the first is at line {unreleased[0].line})",
        )


def parse_releases(headings: Sequence[Heading]) -> tuple[list[Release], list[Violation]]:
    """Clause 2, part one: every heading but Unreleased names a version.

    Args:
        headings: The changelog's level-2 headings.

    Returns:
        The headings that name a version, as releases in line order, and one
        violation per heading that names none.
    """
    releases: list[Release] = []
    violations: list[Violation] = []
    for heading in headings:
        match = BRACKETED.match(heading.text)
        if match and match["name"] == UNRELEASED:
            continue
        if not match or not VERSION.match(match["name"]):
            violations.append(
                Violation(
                    Rule.RELEASE_WITHOUT_VERSION,
                    heading.line,
                    f"`## {heading.text}` names no version",
                )
            )
            continue
        date = RELEASE_DATE.match(match["rest"])
        releases.append(
            Release(
                heading.line,
                match["name"],
                parse_version(match["name"]),
                date["date"] if date else None,
            )
        )
    return releases, violations


def check_dates(releases: Sequence[Release]) -> Iterator[Violation]:
    """Clause 2, part two: every release carries a calendar date."""
    for release in releases:
        if release.date is None:
            yield Violation(
                Rule.RELEASE_WITHOUT_DATE,
                release.line,
                f"{release.name} has no ` - <YYYY-MM-DD>` date",
            )
            continue
        try:
            datetime.date.fromisoformat(release.date)
        except ValueError:
            yield Violation(
                Rule.RELEASE_WITHOUT_DATE,
                release.line,
                f"{release.name}'s date {release.date} is not a calendar date",
            )


def unique_releases(releases: Sequence[Release]) -> tuple[list[Release], list[Violation]]:
    """Clause 3, part one: keep each version's first heading and flag the others.

    Args:
        releases: Every release heading, in line order.

    Returns:
        Each version's first release, in line order, and one violation per repeat.
    """
    first: dict[Version, Release] = {}
    violations: list[Violation] = []
    for release in releases:
        if release.version in first:
            violations.append(
                Violation(
                    Rule.DUPLICATE_RELEASE,
                    release.line,
                    f"{release.name} is already released at line {first[release.version].line}",
                )
            )
        else:
            first[release.version] = release
    return list(first.values()), violations


def check_order(releases: Sequence[Release]) -> Iterator[Violation]:
    """Clause 3, part two, over unique releases: newest first."""
    for newer, older in zip(releases, releases[1:]):
        if older.version > newer.version:
            yield Violation(
                Rule.RELEASES_NOT_DESCENDING,
                older.line,
                f"{older.name} is listed below the older {newer.name}",
            )


def index_links(links: Sequence[Link]) -> tuple[dict[str, Link], list[Violation]]:
    """Clause 4: keep each name's first definition, the one Markdown uses, and flag the others.

    Args:
        links: Every link definition, in line order.

    Returns:
        Each name's first definition keyed by its lower-cased name, and one
        violation per repeat.
    """
    first: dict[str, Link] = {}
    violations: list[Violation] = []
    for link in links:
        key = link.name.lower()
        if key in first:
            violations.append(
                Violation(
                    Rule.DUPLICATE_LINK,
                    link.line,
                    f"[{link.name}] is already defined at line {first[key].line}",
                )
            )
        else:
            first[key] = link
    return first, violations


def check_release_links(
    releases: Sequence[Release], links: Mapping[str, Link]
) -> Iterator[Violation]:
    """Clause 5: releases and version links pair up."""
    released = {release.name.lower() for release in releases}
    for release in releases:
        if release.name.lower() not in links:
            yield Violation(Rule.RELEASE_WITHOUT_LINK, release.line, f"no `[{release.name}]:` link")
    for key, link in links.items():
        if VERSION.match(link.name) and key not in released:
            yield Violation(
                Rule.LINK_WITHOUT_RELEASE,
                link.line,
                f"`[{link.name}]:` has no `## [{link.name}]` heading",
            )


def check_unreleased_link(
    releases: Sequence[Release], links: Mapping[str, Link], unreleased_line: int
) -> Iterator[Violation]:
    """Clause 6: `[Unreleased]:` compares the newest release with HEAD."""
    link = links.get(UNRELEASED.lower())
    if link is None:
        yield Violation(Rule.UNRELEASED_LINK_MISMATCH, unreleased_line, "no `[Unreleased]:` link")
        return
    if not releases:
        return
    expected = (releases[0].name, HEAD)
    if compare_range(link.url) != expected:
        yield Violation(
            Rule.UNRELEASED_LINK_MISMATCH,
            link.line,
            f"`[Unreleased]:` compares {describe_range(link)}, expected {expected[0]}...{HEAD}",
        )


def check_compare_chain(
    releases: Sequence[Release], links: Mapping[str, Link]
) -> Iterator[Violation]:
    """Clause 7, over unique releases: each link compares the next-older release with its own."""
    for index, release in enumerate(releases):
        link = links.get(release.name.lower())
        if link is None:
            continue  # clause 5 reports it
        compared = compare_range(link.url)
        if index + 1 < len(releases):
            older = releases[index + 1].name
            if compared != (older, release.name):
                yield Violation(
                    Rule.COMPARE_CHAIN_BROKEN,
                    link.line,
                    f"`[{release.name}]:` compares {describe_range(link)}, "
                    f"expected {older}...{release.name}",
                )
        elif (
            compared is None
            or compared[1] != release.name
            or not VERSION.match(compared[0])
            or parse_version(compared[0]) >= release.version
        ):
            yield Violation(
                Rule.COMPARE_CHAIN_BROKEN,
                link.line,
                f"`[{release.name}]:` compares {describe_range(link)}, "
                f"expected <older version>...{release.name}",
            )


def check(text: str) -> list[Violation]:
    """Every contract violation in a changelog.

    Args:
        text: The changelog's Markdown.

    Returns:
        The violations in line order; empty when the changelog keeps the contract.
    """
    headings, links = parse(text)
    releases, violations = parse_releases(headings)
    unique, duplicate_releases = unique_releases(releases)
    link_index, duplicate_links = index_links(links)
    unreleased_line = next(
        (heading.line for heading in headings if heading_name(heading) == UNRELEASED), 1
    )
    violations += [
        *check_unreleased(headings),
        *check_dates(releases),
        *duplicate_releases,
        *check_order(unique),
        *duplicate_links,
        *check_release_links(unique, link_index),
        *check_unreleased_link(unique, link_index, unreleased_line),
        *check_compare_chain(unique, link_index),
    ]
    return sorted(violations, key=lambda violation: violation.line)


def main(argv: Sequence[str] | None = None) -> int:
    """Check a changelog, print each violation, and return the process exit code."""
    parser = argparse.ArgumentParser(description="Check CHANGELOG.md's release structure.")
    parser.add_argument("changelog", nargs="?", type=Path, default=DEFAULT_CHANGELOG)
    args = parser.parse_args(argv)
    violations = check(args.changelog.read_text(encoding="utf-8"))
    for violation in violations:
        print(
            f"{args.changelog}:{violation.line}: {violation.rule.name}: "
            f"{violation.detail} ({violation.rule.value})"
        )
    if violations:
        print(f"{args.changelog}: {len(violations)} contract violation(s)")
        return 1
    print(f"{args.changelog}: changelog contract holds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
