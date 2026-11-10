"""Let macOS wheel consumers share their environment's OpenMP runtime (#136).

Run on an unpacked, delocated wheel. Only OpenMP load commands change; delocate
still owns dependency bundling. The normal dyld search order is the installed
Python environment's lib directory, then the wheel's bundled libomp.
"""

import argparse
import subprocess
from pathlib import Path

from delocate.tools import get_install_names, get_rpaths

OPENMP = "libomp.dylib"
# <env>/lib/python3.X/site-packages is two directories below <env>/lib.
SITE_PACKAGES_TO_LIB = 2


class WheelOpenMPError(RuntimeError):
    """The unpacked wheel does not contain the expected OpenMP library/users."""


def configure_openmp(root: Path) -> None:
    """Adjust OpenMP references and re-sign changed Mach-O images.

    Args:
        root: Unpacked wheel root, equivalent to the installed site-packages.

    Raises:
        WheelOpenMPError: The wheel lacks a unique bundled runtime or OpenMP users.
        subprocess.CalledProcessError: Mach-O editing or signing failed.
    """
    runtimes = list(root.rglob(OPENMP))
    if len(runtimes) != 1:
        raise WheelOpenMPError(f"Expected one bundled {OPENMP}, found {runtimes}")
    runtime = runtimes[0]
    users = 0
    for image in sorted(root.rglob("*")):
        if not image.is_file() or image.suffix not in (".so", ".dylib"):
            continue
        dependencies = [d for d in get_install_names(str(image)) if Path(d).name == OPENMP]
        edits: list[str] = []
        if dependencies:
            users += 1
            for dependency in dependencies:
                edits.extend(["-change", dependency, f"@rpath/{OPENMP}"])
            existing = tuple(dict.fromkeys(get_rpaths(str(image))))
            for rpath in existing:
                edits.extend(["-delete_rpath", rpath])
            depth = len(image.parent.relative_to(root).parts) + SITE_PACKAGES_TO_LIB
            env_lib = "/".join(["@loader_path", *([".."] * depth)])
            bundled = runtime.parent.relative_to(image.parent, walk_up=True).as_posix()
            bundle_rpath = "@loader_path" if bundled == "." else f"@loader_path/{bundled}"
            for rpath in dict.fromkeys((env_lib, bundle_rpath, *existing)):
                edits.extend(["-add_rpath", rpath])
        if image == runtime:
            edits.extend(["-id", f"@rpath/{OPENMP}"])
        if edits:
            image.chmod(image.stat().st_mode | 0o200)
            subprocess.run(["install_name_tool", *edits, str(image)], check=True)
            # Editing invalidates the original signature. Remove it, then sign the new image.
            subprocess.run(["codesign", "--remove-signature", str(image)], check=True)
            subprocess.run(["codesign", "--sign", "-", str(image)], check=True)
    if not users:
        raise WheelOpenMPError("No OpenMP users found in the wheel")
    print(f"Configured {users} OpenMP users and {runtime}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    configure_openmp(parser.parse_args().root.resolve())
