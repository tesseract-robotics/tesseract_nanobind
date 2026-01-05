class SimpleMotionPlanner:
    def __init__(self, name: str = "SIMPLE") -> None: ...
    def getName(self) -> str: ...
    def solve(
        self, request: "tesseract_planning::PlannerRequest"
    ) -> "tesseract_planning::PlannerResponse": ...
    def terminate(self) -> bool: ...
    def clear(self) -> None: ...

def generateInterpolatedProgram(
    instructions: "tesseract_planning::CompositeInstruction",
    env: "tesseract_environment::Environment",
    state_longest_valid_segment_length: float = 0.08726646259971647,
    translation_longest_valid_segment_length: float = 0.15,
    rotation_longest_valid_segment_length: float = 0.08726646259971647,
    min_steps: int = 1,
) -> "tesseract_planning::CompositeInstruction":
    """Generate an interpolated program from a composite instruction"""
