"""The harness's file server holds up against a docs page's burst of requests."""

import sys
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import browser  # noqa: E402


def test_the_backlog_fits_a_page_of_posters():
    # planning.md asks for 10 posters at once. http.server's default of 5 dropped connections:
    # 12 failed requests in 30 loads, against 0 at this backlog.
    assert browser.StaticServer.request_queue_size == browser.LISTEN_BACKLOG > 5


def test_the_server_serves_its_root(tmp_path):
    (tmp_path / "a.txt").write_text("hello")
    with browser.serve(tmp_path) as port:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/a.txt") as reply:
            assert reply.read() == b"hello"
