import enum
import os
from collections.abc import Callable, Mapping, Sequence
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.tesseract_common._tesseract_common

class Item:
    def __init__(self) -> None: ...

class Items:
    def __init__(self) -> None: ...

class Mode(enum.Enum):
    FAST = 0

    SAFE = 1

class Seq:
    def __init__(self) -> None: ...
    def at(self, i: int) -> int: ...
    def push(self, item: Item) -> None: ...

@overload
def pick(name: str) -> None: ...
@overload
def pick(flag: bool) -> None: ...
def take(items: Items) -> None: ...
def take_all(items: Sequence[Item]) -> None: ...
def put(value: int) -> None: ...
def guess(value: object) -> None: ...

class Command:
    def __init__(self) -> None: ...

class AddThing(Command):
    def __init__(self) -> None: ...

class RemoveThing(AddThing):
    def __init__(self) -> None: ...

@overload
def apply(command: AddThing) -> bool: ...
@overload
def apply(command: RemoveThing) -> bool: ...

class Shape:
    def __init__(self) -> None: ...

class Ball(Shape):
    def __init__(self) -> None: ...

def roll(balls: Sequence[Shape]) -> None: ...
def kinds(
    b: bool,
    i: int,
    n: int,
    x: float,
    s: str,
    c: str,
    p: str | os.PathLike,
    names: Sequence[str],
    values: Mapping[str, float],
    items: Mapping[str, Item],
    pair: tuple[str, str],
    shared: Item | None,
    opt: int | None,
    var: str | tesseract_robotics.tesseract_common._tesseract_common.Isometry3d,
    vec: Annotated[NDArray[numpy.float64], dict(shape=(None,), order="C")],
    ref: Annotated[NDArray[numpy.float64], dict(shape=(None,), writable=False)],
    ivec: Annotated[NDArray[numpy.int32], dict(shape=(None,), order="C")],
    iso: tesseract_robotics.tesseract_common._tesseract_common.Isometry3d,
    mode: Mode,
    fn: Callable[[int], None],
    id: str,
) -> None: ...
