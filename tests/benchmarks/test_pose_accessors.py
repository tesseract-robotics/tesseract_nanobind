"""Benchmark Pose scalar accessors — measure the cost folklore-claimed in
the transforms.py "Perf note" section comment.

The class docstring asserts:
  - `pose.qx/qy/qz/qw` each allocates a fresh Quaterniond → multi-read
    overhead grows linearly in number of accesses.
  - `pose.roll/pitch/yaw` additionally runs `eulerAngles("ZYX")` per call,
    which is non-trivial trig.
  - Recommended pattern for multi-component reads: bind `q = pose.quaternion`
    once and access components on it.

This benchmark file quantifies each claim. Numbers are environment-specific;
the relative ordering between paths is the load-bearing signal.

Run with:
    pixi run pytest tests/benchmarks/test_pose_accessors.py --benchmark-only

Reference run (Apple M-series, Python 3.12, debug build), median ns:

    Path                                           Median   Relative
    pose.translation + [0]/[1]/[2]                 ~960   1.0× (baseline)
    q = pose.quaternion; q.x/y/z/w                 ~1,334   1.4×
    pose.quaternion.eulerAngles("ZYX")             ~2,458   2.6×
    pose.x / pose.y / pose.z (3 property reads)    ~2,500   2.6×
    pose.qx/qy/qz/qw (4 property reads)            ~4,500   4.7×
    pose.roll / pose.pitch / pose.yaw              ~5,917   6.2×

Two load-bearing conclusions:

1. The property paths are 2.5–3.5× slower than the equivalent "bind typed
   accessor once" pattern, exactly as the transforms.py perf note claims.
2. Absolute cost is sub-microsecond per call. "Negligible in typical use"
   is vindicated: even the slowest path (3× RPY reads, ~6 µs) supports
   ~170 kHz read rate — far above any non-realtime workload.

The perf note in `planning/transforms.py` is therefore accurate. The
recommendation to use `q = pose.quaternion` for multi-component reads
matters mostly inside tight numerical loops, not for typical logging /
formatting / one-off-read usage.
"""

import time
from collections.abc import Callable

import pytest

from tesseract_robotics.planning import Pose


@pytest.fixture(scope="module")
def sample_pose() -> Pose:
    """A non-trivial pose so eulerAngles has actual work to do."""
    return Pose.from_xyz_rpy(0.5, -0.2, 0.3, 0.1, 0.2, 0.3)


# ---------- Translation: properties vs underlying call ----------


def test_translation_via_x_y_z_properties(benchmark, sample_pose):
    """`pose.x, pose.y, pose.z` — 3 property reads, scalar each."""

    def read():
        return sample_pose.x, sample_pose.y, sample_pose.z

    benchmark(read)


def test_translation_via_translation_property_with_index(benchmark, sample_pose):
    """`pose.translation[i]` — 1 property read returning ndarray + 3 numpy indexes."""

    def read():
        t = sample_pose.translation
        return t[0], t[1], t[2]

    benchmark(read)


# ---------- Quaternion: 4 properties vs 1 typed accessor + reads ----------


def test_quaternion_via_qx_qy_qz_qw_properties(benchmark, sample_pose):
    """4× `pose.qx/qy/qz/qw` — 4 Quaterniond allocations from linear()."""

    def read():
        return sample_pose.qx, sample_pose.qy, sample_pose.qz, sample_pose.qw

    benchmark(read)


def test_quaternion_via_typed_accessor(benchmark, sample_pose):
    """`q = pose.quaternion; q.x, q.y, q.z, q.w` — 1 allocation."""

    def read():
        q = sample_pose.quaternion
        return q.x, q.y, q.z, q.w

    benchmark(read)


# ---------- RPY: 3 properties vs single eulerAngles call ----------


def test_rpy_via_roll_pitch_yaw_properties(benchmark, sample_pose):
    """3× `pose.roll/pitch/yaw` — 3 Quaterniond allocs + 3 eulerAngles."""

    def read():
        return sample_pose.roll, sample_pose.pitch, sample_pose.yaw

    benchmark(read)


def test_rpy_via_single_eulerangles_call(benchmark, sample_pose):
    """`pose.quaternion.eulerAngles("ZYX")` — 1 alloc + 1 decomposition."""

    def read():
        yaw, pitch, roll = sample_pose.quaternion.eulerAngles("ZYX")
        return yaw, pitch, roll

    benchmark(read)


# ---------- The documented claims, as ratios ----------

# Calls per timed batch, and batches per measurement: the minimum over batches discards
# scheduler noise, which only ever adds time.
CALLS_PER_BATCH = 2000
BATCHES = 15
# The module docstring's reference run puts `pose.qx/qy/qz/qw` at 3.4x the bound-quaternion read
# (4,500 ns vs 1,334 ns) and `pose.roll/pitch/yaw` at 2.4x the single `eulerAngles` call
# (5,917 ns vs 2,458 ns). A property path must stay at least half as far from its baseline as
# that run, so a different machine passes and a lost saving does not.
MIN_QUATERNION_RATIO = 1.7
MIN_RPY_RATIO = 1.2


def _best_ns_per_call(fn: Callable[[], object]) -> float:
    best = float("inf")
    for _ in range(BATCHES):
        start = time.perf_counter_ns()
        for _ in range(CALLS_PER_BATCH):
            fn()
        best = min(best, (time.perf_counter_ns() - start) / CALLS_PER_BATCH)
    return best


def test_quaternion_properties_cost_more_than_one_bound_quaternion(sample_pose):
    """transforms.py's perf note: bind `q = pose.quaternion` once for multi-component reads."""

    def properties():
        return sample_pose.qx, sample_pose.qy, sample_pose.qz, sample_pose.qw

    def bound():
        q = sample_pose.quaternion
        return q.x, q.y, q.z, q.w

    ratio = _best_ns_per_call(properties) / _best_ns_per_call(bound)
    assert ratio >= MIN_QUATERNION_RATIO, f"qx..qw cost {ratio:.2f}x the bound quaternion"


def test_rpy_properties_cost_more_than_one_euler_decomposition(sample_pose):
    """transforms.py's perf note: three `roll/pitch/yaw` reads decompose the rotation three times."""

    def properties():
        return sample_pose.roll, sample_pose.pitch, sample_pose.yaw

    def single():
        return sample_pose.quaternion.eulerAngles("ZYX")

    ratio = _best_ns_per_call(properties) / _best_ns_per_call(single)
    assert ratio >= MIN_RPY_RATIO, f"roll/pitch/yaw cost {ratio:.2f}x one eulerAngles call"
