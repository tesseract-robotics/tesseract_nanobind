"""OctoMap `.bt` decoder: the occupied voxels of a tesseract `Octree`."""

from __future__ import annotations

import tempfile
from collections.abc import Iterator
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from tesseract_robotics import tesseract_geometry

from .errors import MalformedOctreeStreamError

# OctoMap's fixed tree depth [-]: the root spans resolution * 2**16 and is centred at the origin.
OCTOMAP_TREE_DEPTH = 16

# A node's 16 bits hold 2 bits per child, in child order.
BITS_PER_CHILD = 2
CHILD_CODE_MASK = 0b11
CHILD_COUNT = 8
BYTES_PER_NODE = 2
CODE_ABSENT = 0b00
CODE_OCCUPIED_LEAF = 0b10
CODE_INNER = 0b11

_DATA_SEPARATOR = b"data\n"
_RESOLUTION_PREFIX = "res "


def _iter_octomap_bt_leaves(
    bt_bytes: bytes, tree_depth: int = OCTOMAP_TREE_DEPTH
) -> Iterator[tuple[float, float, float, float]]:
    """Yield `(cx, cy, cz, size)` [m] for every occupied leaf of an OctoMap `.bt` stream.

    Inner nodes are written depth-first in child order 0..7, each as 8 two-bit codes:
    absent, free leaf (`0b01`, ignored), occupied leaf, inner node. Child `i`'s octant is
    `(i & 1, i & 2, i & 4)` for `(x, y, z)`.

    Args:
        bt_bytes: Contents of a `.bt` file.
        tree_depth: OctoMap tree depth.

    Yields:
        Leaf centre and edge length.

    Raises:
        MalformedOctreeStreamError: no `data` or `res` header line, or the tree is cut short.
    """
    idx = bt_bytes.find(_DATA_SEPARATOR)
    if idx < 0:
        raise MalformedOctreeStreamError("malformed .bt stream: no 'data' line in header")
    header = bt_bytes[:idx].decode("ascii", errors="replace")
    binary = bt_bytes[idx + len(_DATA_SEPARATOR) :]

    resolution = None
    for line in header.splitlines():
        line = line.strip()
        if line.startswith(_RESOLUTION_PREFIX):
            resolution = float(line.split()[1])
            break
    if resolution is None:
        raise MalformedOctreeStreamError("malformed .bt stream: no 'res' line in header")

    pos = 0
    stack = [(0.0, 0.0, 0.0, resolution * (1 << tree_depth))]
    while stack:
        cx, cy, cz, size = stack.pop()
        if pos + BYTES_PER_NODE > len(binary):
            raise MalformedOctreeStreamError("truncated .bt stream while walking tree")
        bits = binary[pos] | (binary[pos + 1] << 8)
        pos += BYTES_PER_NODE
        child_size = size / 2
        offset = size / 4
        # Reverse: the stack then visits child 0 first, the stream's order.
        for i in reversed(range(CHILD_COUNT)):
            code = (bits >> (i * BITS_PER_CHILD)) & CHILD_CODE_MASK
            if code == CODE_ABSENT:
                continue
            ccx = cx + (offset if (i & 1) else -offset)
            ccy = cy + (offset if (i & 2) else -offset)
            ccz = cz + (offset if (i & 4) else -offset)
            if code == CODE_OCCUPIED_LEAF:
                yield (ccx, ccy, ccz, child_size)
            elif code == CODE_INNER:
                stack.append((ccx, ccy, ccz, child_size))


def octree_voxels(
    octree: tesseract_geometry.Octree,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Occupied voxels of `octree`.

    Args:
        octree: Tesseract octree geometry.

    Returns:
        `(N, 3)` centres [m] and `(N,)` edge lengths [m].
    """
    # octomap only serializes to a file path from Python.
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "octree.bt"
        octree.getOctree().writeBinary(str(path))
        data = path.read_bytes()
    leaves = list(_iter_octomap_bt_leaves(data))
    if not leaves:
        return np.empty((0, 3)), np.empty((0,))
    voxels = np.asarray(leaves, dtype=np.float64)
    return voxels[:, :3], voxels[:, 3]
