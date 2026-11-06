"""Player snippets: players only for what a person reviewed, and each on its example's page."""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import lock  # noqa: E402
import manifest  # noqa: E402
import players  # noqa: E402

CAMERA = manifest.Camera((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), 0.6)
SHOWN = manifest.Entry(
    "shown", "examples/p.md", "Shown", "It plays.", manifest.Status.PUBLISHED, None, CAMERA, None
)
IN_REVIEW = manifest.Entry(
    "in_review", "examples/p.md", "In review", "B.", manifest.Status.EXCLUDED, "Wait.", CAMERA, None
)
LEFT_OUT = manifest.Entry(
    "left_out", "examples/p.md", "Left out", "B.", manifest.Status.EXCLUDED, "Wrong.", None, None
)
HERO = manifest.Entry(
    players.HERO_MODULE,
    "examples/p.md",
    "Hero",
    "B.",
    manifest.Status.PUBLISHED,
    None,
    CAMERA,
    None,
)
ENTRIES = [SHOWN, IN_REVIEW, LEFT_OUT, HERO]
RECORDED = (SHOWN.module, IN_REVIEW.module, HERO.module)
LOCK = lock.DemoLock.build(
    "1.1.1",
    "client",
    {
        module: lock.RecordingLock(lock.BuildIdentity("0.35", "1.1.1", "abc"), "d", 2_000_000)
        for module in RECORDED
    },
    dict.fromkeys(RECORDED, "p"),
)


def test_a_published_entry_gets_a_click_to_load_player_with_its_identity():
    text = players.snippets(ENTRIES, LOCK)["shown.html"]
    assert 'data-src="../../demos/client/index.html?playbackPath=../recordings/shown.viser' in text
    assert "0.35" in text
    assert "viser 1.1.1" in text
    assert "<iframe" not in text


def test_a_player_shows_its_moving_poster_until_clicked():
    text = players.snippets(ENTRIES, LOCK)["shown.html"]
    # Source-relative: the docs builder rewrites an <img src> for the page's URL, unlike data-src.
    assert '<img src="../demos/posters/shown.webp"' in text
    assert 'loading="lazy"' in text


def test_the_hero_resolves_from_the_site_root():
    text = players.snippets(ENTRIES, LOCK)[players.HERO_SNIPPET]
    assert 'data-src="demos/client/index.html?' in text
    assert f'<img src="demos/posters/{players.HERO_MODULE}.webp"' in text


def test_an_excluded_entry_has_an_empty_snippet():
    snippets = players.snippets(ENTRIES, LOCK)
    assert snippets["in_review.html"] == ""
    assert snippets["left_out.html"] == ""


def test_an_unpublished_hero_has_an_empty_snippet():
    hero = manifest.Entry(
        players.HERO_MODULE, "p.md", "H", "B.", manifest.Status.EXCLUDED, "Wait.", CAMERA, None
    )
    assert players.snippets([hero], LOCK)[players.HERO_SNIPPET] == ""


def test_a_preview_gives_a_player_to_what_is_recorded_and_awaiting_review():
    snippets = players.snippets(ENTRIES, LOCK, preview=True)
    assert "recordings/in_review.viser" in snippets["in_review.html"]
    assert snippets["left_out.html"] == ""


def test_a_published_entry_without_a_recording_fails_loudly():
    empty = lock.DemoLock.build("1.1.1", "client", {}, {})
    with pytest.raises(players.MissingRecordingError, match="shown"):
        players.snippets(ENTRIES, empty)


def test_a_published_entry_without_a_poster_fails_loudly():
    no_poster = lock.DemoLock.build("1.1.1", "client", LOCK.recordings, {})
    with pytest.raises(players.MissingPosterError, match="shown"):
        players.snippets(ENTRIES, no_poster)


def test_a_preview_without_a_poster_fails_loudly_too():
    posters = {SHOWN.module: "p", HERO.module: "p"}
    partial = lock.DemoLock.build("1.1.1", "client", LOCK.recordings, posters)
    with pytest.raises(players.MissingPosterError, match="in_review"):
        players.snippets(ENTRIES, partial, preview=True)


def _page_text(page: str) -> str:
    return (REPO / "docs" / page).read_text("utf-8")


def test_a_published_entry_is_included_once_on_its_page():
    for entry in manifest.load_manifest():
        if entry.status is manifest.Status.PUBLISHED:
            assert players.included_modules(_page_text(entry.page)).count(entry.module) == 1


def test_a_page_only_includes_players_the_manifest_places_there():
    entries = {entry.module: entry for entry in manifest.load_manifest()}
    for path in (REPO / "docs").rglob("*.md"):
        if "superpowers" in path.parts or path == REPO / "docs/index.md":
            continue
        page = path.relative_to(REPO / "docs").as_posix()
        for module in players.included_modules(path.read_text("utf-8")):
            assert entries[module].page == page


def test_the_first_content_on_the_docs_home_is_the_hero():
    lines = [line for line in _page_text("index.md").splitlines() if line.strip()]
    assert lines[0].startswith("# ")
    assert lines[1] == f'--8<-- "docs/demos/players/{players.HERO_SNIPPET}"'
