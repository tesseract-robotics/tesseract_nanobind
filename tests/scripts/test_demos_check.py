"""The player check's verdicts, on canned browser events."""

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/demos"))

import check  # noqa: E402


def _sent(request_id, url):
    return {
        "method": "Network.requestWillBeSent",
        "params": {"requestId": request_id, "request": {"url": url}},
    }


def _response(url, status):
    return {
        "method": "Network.responseReceived",
        "params": {"response": {"url": url, "status": status}},
    }


def test_a_page_url_drops_the_markdown_suffix():
    assert check.page_url("examples/planning.md") == "examples/planning/"


def test_a_404_on_a_demo_file_is_a_failure():
    url = "http://x/tesseract_nanobind/demos/recordings/a.viser"
    assert check.failed_demo_requests([_response(url, 404)]) == [f"404 {url}"]


def test_the_version_selector_404_is_not_a_demo_failure():
    # The theme's selector asks the server root for it; nothing of ours.
    assert check.failed_demo_requests([_response("http://x/versions.json", 404)]) == []


def test_a_demo_request_that_never_finished_is_a_failure():
    url = "http://x/demos/client/index.html"
    events = [
        _sent("7", url),
        {"method": "Network.loadingFailed", "params": {"requestId": "7", "errorText": "net::ERR"}},
    ]
    assert check.failed_demo_requests(events) == [f"net::ERR {url}"]


def test_changed_fraction_counts_pixels_that_moved():
    before = np.zeros((2, 2, 3), dtype=np.uint8)
    after = before.copy()
    after[0, 0] = 255
    assert check.changed_fraction(before, after) == 0.25


def test_noise_below_the_threshold_is_no_change():
    before = np.zeros((2, 2, 3), dtype=np.uint8)
    after = before + 1
    assert check.changed_fraction(before, after) == 0.0
