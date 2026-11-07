"""Hosting contract: CI fetches what the lock pins, and the built site's demo links resolve."""

import hashlib
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
DEMOS = REPO / "scripts/demos"
sys.path.insert(0, str(DEMOS))

import check_site  # noqa: E402
import fetch  # noqa: E402
import lock  # noqa: E402
import manifest  # noqa: E402

CAMERA = manifest.Camera((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), 0.6)


def _entry(module, status):
    reason = None if status is manifest.Status.PUBLISHED else "Wait."
    return manifest.Entry(module, "examples/p.md", "T", "B.", status, reason, CAMERA, None)


SHOWN = _entry("shown", manifest.Status.PUBLISHED)
IN_REVIEW = _entry("in_review", manifest.Status.EXCLUDED)


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _lock(files: dict[str, bytes]) -> lock.DemoLock:
    recordings = {
        module: lock.RecordingLock(
            lock.BuildIdentity("0.35", "1.1.1", "abc"),
            _sha(files[f"recordings/{module}.viser"]),
            len(files[f"recordings/{module}.viser"]),
        )
        for module in ("shown", "in_review")
    }
    posters = {module: _sha(files[f"posters/{module}.png"]) for module in ("shown", "in_review")}
    return lock.DemoLock.build("1.1.1", _sha(files["client/index.html"]), recordings, posters)


FILES = {
    "client/index.html": b"client",
    "recordings/shown.viser": b"shown recording",
    "recordings/in_review.viser": b"unreviewed recording",
    "posters/shown.png": b"shown poster",
    "posters/in_review.png": b"unreviewed poster",
}


def _release(tmp_path, files=FILES) -> str:
    release = tmp_path / "release"
    release.mkdir()
    for data in files.values():
        (release / _sha(data)).write_bytes(data)
    return release.as_uri()


def test_only_the_client_and_published_entries_are_wanted():
    assert set(fetch.wanted([SHOWN, IN_REVIEW], _lock(FILES))) == {
        "client/index.html",
        "recordings/shown.viser",
        "posters/shown.png",
    }


def test_a_fetch_writes_what_the_lock_pins(tmp_path):
    out = tmp_path / "demos"
    fetch.fetch_all([SHOWN, IN_REVIEW], _lock(FILES), _release(tmp_path), out)
    assert (out / "recordings/shown.viser").read_bytes() == b"shown recording"
    assert not (out / "recordings/in_review.viser").exists()


def test_a_tampered_asset_fails_and_leaves_nothing_behind(tmp_path):
    base = _release(tmp_path)
    pinned = _sha(FILES["recordings/shown.viser"])
    (tmp_path / "release" / pinned).write_bytes(b"something else")
    out = tmp_path / "demos"
    with pytest.raises(fetch.AssetHashError):
        fetch.fetch_all([SHOWN], _lock(FILES), base, out)
    assert not (out / "recordings/shown.viser").exists()


def test_a_missing_asset_fails(tmp_path):
    base = _release(tmp_path)
    (tmp_path / "release" / _sha(FILES["posters/shown.png"])).unlink()
    with pytest.raises(fetch.AssetDownloadError):
        fetch.fetch_all([SHOWN], _lock(FILES), base, tmp_path / "demos")


def test_a_file_already_matching_the_lock_is_not_downloaded(tmp_path):
    out = tmp_path / "demos"
    for name, data in FILES.items():
        (out / name).parent.mkdir(parents=True, exist_ok=True)
        (out / name).write_bytes(data)
    # No release at this URL: reaching for one would raise.
    fetch.fetch_all([SHOWN], _lock(FILES), (tmp_path / "absent").as_uri(), out)


def test_staging_names_each_published_file_by_its_hash(tmp_path):
    demos = tmp_path / "demos"
    for name, data in FILES.items():
        (demos / name).parent.mkdir(parents=True, exist_ok=True)
        (demos / name).write_bytes(data)
    staged = fetch.stage([SHOWN, IN_REVIEW], _lock(FILES), demos, tmp_path / "staged")
    assert {path.name for path in staged} == {
        _sha(FILES["client/index.html"]),
        _sha(FILES["recordings/shown.viser"]),
        _sha(FILES["posters/shown.png"]),
    }


PLAYER = (
    '<button class="demo-player" type="button" data-src="{root}demos/client/index.html?'
    'playbackPath=../recordings/{module}.viser&amp;darkMode">'
    '<img src="{root}demos/posters/{module}.png" alt="" loading="lazy">'
    '<span class="demo-play">Play</span></button>'
)


def _site(tmp_path, *, drop=()):
    site = tmp_path / "site"
    page = site / "examples/p/index.html"
    page.parent.mkdir(parents=True)
    page.write_text(PLAYER.format(root="../../", module="shown"))
    (site / "index.html").write_text(PLAYER.format(root="", module="shown"))
    for name in (
        "demos/client/index.html",
        "demos/recordings/shown.viser",
        "demos/posters/shown.png",
    ):
        if name not in drop:
            (site / name).parent.mkdir(parents=True, exist_ok=True)
            (site / name).write_bytes(b"x")
    return site


def test_a_complete_site_passes(tmp_path):
    assert check_site.missing_files(_site(tmp_path), [SHOWN]) == []


@pytest.mark.parametrize(
    "name",
    ["demos/client/index.html", "demos/recordings/shown.viser", "demos/posters/shown.png"],
)
def test_a_referenced_file_missing_from_the_site_is_named(tmp_path, name):
    missing = check_site.missing_files(_site(tmp_path, drop=[name]), [SHOWN])
    assert any(name in problem for problem in missing), missing


def test_a_published_entry_without_a_player_on_its_page_is_named(tmp_path):
    site = _site(tmp_path)
    (site / "examples/p/index.html").write_text("no player")
    assert any("shown" in p for p in check_site.missing_files(site, [SHOWN]))


@pytest.mark.parametrize("script", ["fetch.py", "check_site.py", "players.py"])
def test_what_ci_runs_needs_only_the_standard_library(script):
    # docs.yml does not install this package; -S hides every site-package.
    result = subprocess.run(
        [sys.executable, "-S", str(DEMOS / script), "--help"], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_a_root_relative_link_is_named_though_a_local_server_would_find_it(tmp_path):
    # The poster 404 this project already hit once: fine at a local root, broken under Pages.
    site = _site(tmp_path)
    page = site / "examples/p/index.html"
    page.write_text(page.read_text().replace("../../demos/posters/", "/demos/posters/"))
    assert any("leaves the site" in p for p in check_site.missing_files(site, [SHOWN]))


def test_readme_poster_opens_the_docs_player():
    """The README's poster lands on the docs home, whose player `#play` starts: not the bare client."""
    readme = (REPO / "README.md").read_text("utf-8")
    site_url = next(
        line.split(":", 1)[1].strip()
        for line in (REPO / "mkdocs.yml").read_text("utf-8").splitlines()
        if line.startswith("site_url:")
    )
    poster = f"{site_url}demos/posters/{check_site.HERO_MODULE}.png"
    (line,) = [line for line in readme.splitlines() if poster in line]
    assert line.endswith(f"]({site_url}#play)")
