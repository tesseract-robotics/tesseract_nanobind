"""Check wheel source revisions against the development checkout's upstream gitlinks."""

import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class WheelSourceDriftError(RuntimeError):
    """A wheel would build different upstream sources than the development checkout."""


def check_sources(root: Path) -> None:
    """Compare the three development stack gitlinks with dependencies.rosinstall."""
    sources = {
        entry["git"]["local-name"]: entry["git"]["version"]
        for entry in yaml.safe_load((root / "dependencies.rosinstall").read_text())
    }
    for name in ("tesseract", "tesseract_planning", "trajopt"):
        entry = subprocess.run(
            ["git", "ls-files", "--stage", f"upstream/{name}"],
            cwd=root,
            text=True,
            capture_output=True,
            check=True,
        ).stdout.split()
        if len(entry) != 4 or entry[0] != "160000":
            raise WheelSourceDriftError(f"Missing upstream/{name} gitlink")
        if sources[name] != entry[1]:
            raise WheelSourceDriftError(
                f"{name}: wheel source {sources[name]} differs from development {entry[1]}; "
                "update dependencies.rosinstall with the upstream submodule"
            )
        print(f"{name}: wheel and development source agree ({sources[name]})")


if __name__ == "__main__":
    check_sources(ROOT)
