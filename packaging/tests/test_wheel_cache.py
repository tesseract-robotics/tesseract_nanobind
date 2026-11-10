"""Artifact reuse must respect source changes and producer trust boundaries."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def load_cache_module():
    spec = importlib.util.spec_from_file_location("wheel_cache", ROOT / "scripts/wheel_cache.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(event="push", repository="org/project", branch="main"):
    return {"event": event, "head_repository": {"full_name": repository}, "head_branch": branch}


@pytest.mark.parametrize("event", ["push", "schedule", "workflow_dispatch"])
def test_default_branch_producer_is_reusable(event):
    cache = load_cache_module()
    assert cache.trusted_run(run(event), "org/project", "main", "contributor/fork", "feature", True)


def test_only_same_pr_source_can_reuse_pr_artifacts():
    cache = load_cache_module()
    current = ("org/project", "main", "contributor/fork", "feature", True)
    assert cache.trusted_run(run("pull_request", "contributor/fork", "feature"), *current)
    assert not cache.trusted_run(run("pull_request", "other/fork", "feature"), *current)
    assert not cache.trusted_run(run("pull_request", "contributor/fork", "other"), *current)


def test_release_never_reuses_pull_request_artifacts():
    cache = load_cache_module()
    assert not cache.trusted_run(
        run("pull_request", "org/project", "main"),
        "org/project",
        "main",
        "org/project",
        "main",
        False,
    )


def test_binding_change_does_not_invalidate_native_stack():
    cache = load_cache_module()
    assert not cache.native_inputs_changed(["src/tesseract_common/bindings.cpp"])
    assert cache.native_inputs_changed(["dependencies.rosinstall"])
    assert cache.native_inputs_changed([".github/workflows/vcpkg_overlays/octomap/portfile.cmake"])
    assert cache.native_inputs_changed([".github/workflows/wheels.yml"])
    assert cache.native_inputs_changed(["scripts/wheel_cache.py"])


def test_missing_environment_is_an_error(monkeypatch):
    cache = load_cache_module()
    monkeypatch.delenv("ImageVersion", raising=False)
    with pytest.raises(cache.MissingBuildEnvironmentError, match="ImageVersion"):
        cache.build_environment("linux-x64", ROOT)


def prepare_discovery(tmp_path, monkeypatch):
    import json

    cache = load_cache_module()
    event = tmp_path / "event.json"
    event.write_text(json.dumps({"repository": {"default_branch": "main"}}))
    output = tmp_path / "outputs"
    for key, value in {
        "GITHUB_WORKSPACE": str(tmp_path),
        "GITHUB_EVENT_PATH": str(event),
        "GITHUB_REPOSITORY": "org/project",
        "GITHUB_EVENT_NAME": "push",
        "GITHUB_REF_NAME": "main",
        "GITHUB_OUTPUT": str(output),
    }.items():
        monkeypatch.setenv(key, value)
    environment = {
        "workspace": str(tmp_path),
        "platform": "linux-x64",
        "image": "2026",
        "compiler": "gcc-13",
        "homebrew": "",
    }
    monkeypatch.setattr(cache, "build_environment", lambda *_: environment)
    monkeypatch.setattr(cache, "producer_revision", lambda *_: "actual-merge-revision")
    return cache, output


def test_native_hit_skips_dependency_download_discovery(tmp_path, monkeypatch):
    cache, output = prepare_discovery(tmp_path, monkeypatch)
    producer = dict(run(), id=42, conclusion="success", head_sha="producer-revision")
    calls = []

    def api(endpoint):
        calls.append(endpoint)
        if "/artifacts?" in endpoint:
            return {"artifacts": [{"expired": False, "workflow_run": {"id": 42}}]}
        return producer

    monkeypatch.setattr(cache, "github_json", api)
    compared = []

    def unchanged(root, revision):
        compared.append(revision)
        return revision == "actual-merge-revision"

    monkeypatch.setattr(cache, "unchanged_native_inputs", unchanged)
    cache.discover(tmp_path, "linux-x64", False)
    assert "native-run=42\n" in output.read_text()
    assert compared == ["actual-merge-revision"]
    assert not any("vcpkg-binaries" in call for call in calls)


def test_native_source_change_reuses_only_vcpkg_packages(tmp_path, monkeypatch):
    cache, output = prepare_discovery(tmp_path, monkeypatch)
    producer = dict(run(), id=42, conclusion="success", head_sha="producer-revision")
    monkeypatch.setattr(
        cache,
        "github_json",
        lambda endpoint: (
            {"artifacts": [{"expired": False, "workflow_run": {"id": 42}}]}
            if "/artifacts?" in endpoint
            else producer
        ),
    )
    monkeypatch.setattr(cache, "unchanged_native_inputs", lambda *_: False)
    cache.discover(tmp_path, "linux-x64", False)
    assert "native-run=\n" in output.read_text()
    assert "vcpkg-run=42\n" in output.read_text()


def test_weekly_refresh_does_not_restore_native_install(tmp_path, monkeypatch):
    cache, output = prepare_discovery(tmp_path, monkeypatch)
    calls = []

    def api(endpoint):
        calls.append(endpoint)
        return {"artifacts": []}

    monkeypatch.setattr(cache, "github_json", api)
    cache.discover(tmp_path, "linux-x64", True)
    assert all("vcpkg-binaries" in call for call in calls)
    assert "native-run=\n" in output.read_text()


def test_api_failure_does_not_turn_into_a_cold_build(tmp_path, monkeypatch):
    import subprocess

    cache, output = prepare_discovery(tmp_path, monkeypatch)

    def api(endpoint):
        raise subprocess.CalledProcessError(1, ["gh", "api", endpoint])

    monkeypatch.setattr(cache, "github_json", api)
    with pytest.raises(subprocess.CalledProcessError):
        cache.discover(tmp_path, "linux-x64", False)
    assert not output.exists()


def test_failed_sibling_does_not_discard_successful_platform(monkeypatch):
    cache = load_cache_module()
    monkeypatch.setattr(
        cache,
        "github_json",
        lambda endpoint: {
            "jobs": [
                {"name": "native-linux-x64", "conclusion": "success"},
                {"name": "native-win-x64", "conclusion": "failure"},
                {"name": "native-macos-arm64", "conclusion": None},
            ]
        },
    )
    producer = {"id": 42, "conclusion": "failure"}
    assert cache.platform_succeeded("org/project", producer, "linux-x64")
    assert not cache.platform_succeeded("org/project", producer, "win-x64")
    assert not cache.platform_succeeded("org/project", producer, "macos-arm64")


def test_successful_workflow_needs_no_extra_job_lookup(monkeypatch):
    cache = load_cache_module()

    def unexpected(endpoint):
        pytest.fail(f"Unexpected API request: {endpoint}")

    monkeypatch.setattr(cache, "github_json", unexpected)
    assert cache.platform_succeeded("org/project", {"conclusion": "success"}, "linux-x64")
