"""Find reusable wheel-build artifacts from compatible, trusted workflow runs.

Native installs are reusable only at their original prefix, with the same runner
and toolchain, and unchanged native build inputs. vcpkg binary archives use
vcpkg's own compatibility checks. GitHub/API failures are errors, not cache misses.
"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

NATIVE_INPUTS = (
    "dependencies.rosinstall",
    ".github/workflows/wheels.yml",
    ".github/workflows/vcpkg_",
    ".github/actions/native-cache/",
    "scripts/wheel_cache",
    "upstream/",
)


class MissingBuildEnvironmentError(RuntimeError):
    """Required hosted-runner metadata is unavailable."""


def native_inputs_changed(paths: list[str]) -> bool:
    """Whether changed repository paths can affect the installed C++ stack."""
    return any(path.startswith(NATIVE_INPUTS) for path in paths)


def trusted_run(
    run: dict,
    repository: str,
    default_branch: str,
    source_repository: str,
    source_branch: str,
    is_pull_request: bool,
) -> bool:
    """Never promote another PR's artifacts into this PR or a trusted release."""
    producer = run["head_repository"]["full_name"]
    branch = run["head_branch"]
    if run["event"] in ("push", "schedule", "workflow_dispatch") and producer == repository:
        return branch == default_branch or (
            source_repository == repository and branch == source_branch
        )
    return (
        is_pull_request
        and run["event"] == "pull_request"
        and producer == source_repository
        and branch == source_branch
    )


def command(*args: str, cwd: Path | None = None) -> str:
    return subprocess.run(args, cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def build_environment(platform: str, workspace: Path) -> dict[str, str]:
    """Record actual compiler/runtime versions and the non-relocatable install prefix."""
    if not os.environ.get("ImageVersion"):
        raise MissingBuildEnvironmentError("Hosted runner ImageVersion is required")
    homebrew = ""
    if sys.platform == "win32":
        vswhere = "C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
        installation = Path(command(vswhere, "-latest", "-property", "installationPath"))
        compiler = (
            (installation / "VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt")
            .read_text()
            .strip()
        )
    else:
        compiler = command("c++", "--version").splitlines()[0]
        if sys.platform == "darwin":
            homebrew = command("brew", "list", "--versions", "libomp", "gcc")
    return {
        "workspace": str(workspace.resolve()),
        "platform": platform,
        "image": os.environ["ImageVersion"],
        "compiler": compiler,
        "homebrew": homebrew,
    }


def native_artifact_name(environment: dict[str, str]) -> str:
    """Use readable platform and toolchain versions as the artifact name."""
    versions = "-".join(environment[k] for k in ("platform", "image", "compiler", "homebrew"))
    return "native-stack-" + re.sub(r"[^a-zA-Z0-9_.-]+", "-", versions).strip("-")


def github_json(endpoint: str) -> dict:
    return json.loads(command("gh", "api", endpoint))


def unchanged_native_inputs(root: Path, revision: str) -> bool:
    """Compare the producer revision with current native build inputs."""
    present = subprocess.run(
        ["git", "cat-file", "-e", f"{revision}^{{commit}}"], cwd=root, capture_output=True
    )
    if present.returncode:
        command("git", "fetch", "--no-tags", "--depth=1", "origin", revision, cwd=root)
    paths = command("git", "diff", "--name-only", revision, "HEAD", cwd=root).splitlines()
    return not native_inputs_changed(paths)


def producer_revision(repository: str, run_id: int, name: str, work: Path) -> str:
    """Read the actual checkout revision, including GitHub's PR merge commit."""
    target = work / str(run_id)
    command(
        "gh",
        "run",
        "download",
        str(run_id),
        "--repo",
        repository,
        "--name",
        name + "-source",
        "--dir",
        str(target),
    )
    return (target / "revision.txt").read_text().strip()


def discover(root: Path, platform: str, refresh: bool) -> None:
    workspace = Path(os.environ["GITHUB_WORKSPACE"])
    environment = build_environment(platform, workspace)
    work = workspace / "cache-work"
    work.mkdir(exist_ok=True)
    (work / "current-environment.json").write_text(json.dumps(environment, indent=2) + "\n")
    native_name = native_artifact_name(environment)
    vcpkg_name = f"vcpkg-binaries-{platform}"
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text())
    repository = os.environ["GITHUB_REPOSITORY"]
    default_branch = event["repository"]["default_branch"]
    is_pr = os.environ["GITHUB_EVENT_NAME"] == "pull_request"
    source_repository = event["pull_request"]["head"]["repo"]["full_name"] if is_pr else repository
    source_branch = event["pull_request"]["head"]["ref"] if is_pr else os.environ["GITHUB_REF_NAME"]
    outputs = {
        "native-name": native_name,
        "vcpkg-name": vcpkg_name,
        "native-run": "",
        "vcpkg-run": "",
    }

    def find_producer(name: str, check_sources: bool) -> str:
        # Query matching artifacts directly so busy PR traffic cannot hide a weekly bundle.
        artifacts = github_json(f"repos/{repository}/actions/artifacts?name={name}&per_page=100")
        for artifact in artifacts["artifacts"]:
            if artifact["expired"]:
                continue
            run = github_json(f"repos/{repository}/actions/runs/{artifact['workflow_run']['id']}")
            if run["conclusion"] != "success":
                continue
            if not trusted_run(
                run, repository, default_branch, source_repository, source_branch, is_pr
            ):
                continue
            if check_sources:
                revision = producer_revision(repository, run["id"], name, work)
                if not unchanged_native_inputs(root, revision):
                    continue
            return str(run["id"])
        return ""

    if not refresh:
        outputs["native-run"] = find_producer(native_name, check_sources=True)
    if not outputs["native-run"]:
        outputs["vcpkg-run"] = find_producer(vcpkg_name, check_sources=False)
    with Path(os.environ["GITHUB_OUTPUT"]).open("a") as stream:
        for key, value in outputs.items():
            stream.write(f"{key}={value}\n")
    print(json.dumps(outputs, indent=2))
    if not outputs["native-run"]:
        print(
            "Native cache miss or requested refresh: building from the declared upstream sources."
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--refresh", action="store_true")
    args = parser.parse_args()
    discover(Path(__file__).resolve().parents[1], args.platform, args.refresh)
