# Audit contract fixture (#256 R15): binds the C++ enum LongPhaseName as Phase; its
# `kCamelCase` enumerators are bound in UPPER_SNAKE.
import enum

class Phase(enum.Enum):
    WARM_START = 0

    CONVERGED = 1
