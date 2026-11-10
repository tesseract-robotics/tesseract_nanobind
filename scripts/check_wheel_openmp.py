"""Assert one loaded OpenMP runtime in a macOS wheel consumer (#136)."""

import argparse
import ctypes
import importlib
import sys
from pathlib import Path


class DuplicateOpenMPError(RuntimeError):
    """The wheel and its consumer loaded different OpenMP runtimes."""


def check_openmp(expected: str, order: str) -> None:
    """Load the consumer and bindings in either order, then inspect dyld's images."""
    env_runtime = Path(sys.prefix) / "lib" / "libomp.dylib"
    if expected == "environment" and not env_runtime.is_file():
        raise FileNotFoundError(env_runtime)
    if expected == "bundled" and env_runtime.exists():
        raise DuplicateOpenMPError(f"Bundled-only test has an environment runtime: {env_runtime}")

    def load_consumer() -> None:
        importlib.import_module("numpy")
        importlib.import_module("scipy.linalg")
        if expected == "environment":
            # Explicitly exercise llvm-openmp even if numpy's current BLAS is not an OMP user.
            runtime = ctypes.CDLL(str(env_runtime))
            runtime.omp_get_max_threads.restype = ctypes.c_int
            assert runtime.omp_get_max_threads() > 0

    if order == "consumer-first":
        load_consumer()
    importlib.import_module("tesseract_robotics.tesseract_motion_planners_descartes")
    if order == "bindings-first":
        load_consumer()
    dyld = ctypes.CDLL(None)
    dyld._dyld_image_count.restype = ctypes.c_uint32
    dyld._dyld_get_image_name.argtypes = [ctypes.c_uint32]
    dyld._dyld_get_image_name.restype = ctypes.c_char_p
    images = [dyld._dyld_get_image_name(i).decode() for i in range(dyld._dyld_image_count())]
    runtimes = [Path(p).resolve() for p in images if Path(p).name == "libomp.dylib"]
    print(f"{order}, {expected}: {runtimes}")
    if len(runtimes) != 1:
        raise DuplicateOpenMPError(f"Expected one OpenMP runtime, found {runtimes}")
    if expected == "environment" and runtimes[0] != env_runtime.resolve():
        raise DuplicateOpenMPError(f"Expected {env_runtime}, loaded {runtimes[0]}")
    if expected == "bundled" and runtimes[0].parent.name != ".dylibs":
        raise DuplicateOpenMPError(f"Expected bundled runtime, loaded {runtimes[0]}")
    runtime = ctypes.CDLL(str(runtimes[0]))
    runtime.omp_get_max_threads.restype = ctypes.c_int
    assert runtime.omp_get_max_threads() > 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected", choices=("environment", "bundled"), required=True)
    parser.add_argument("--order", choices=("consumer-first", "bindings-first"), required=True)
    args = parser.parse_args()
    check_openmp(args.expected, args.order)
