"""tesseract_task_composer planning-node profile Python bindings"""

from collections.abc import Sequence
import enum
from typing import overload

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language


class ContactCheckProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, longest_valid_segment_length: float, contact_distance: float) -> None: ...

    @property
    def contact_manager_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig: ...

    @contact_manager_config.setter
    def contact_manager_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig, /) -> None: ...

    @property
    def collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig: ...

    @collision_check_config.setter
    def collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

class FixStateBoundsProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self, mode: FixStateBoundsProfile.Settings = FixStateBoundsProfile.Settings.ALL) -> None: ...

    class Settings(enum.Enum):
        START_ONLY = 0

        END_ONLY = 1

        ALL = 2

        DISABLED = 3

    @property
    def mode(self) -> FixStateBoundsProfile.Settings: ...

    @mode.setter
    def mode(self, arg: FixStateBoundsProfile.Settings, /) -> None: ...

    @property
    def max_deviation_global(self) -> float: ...

    @max_deviation_global.setter
    def max_deviation_global(self, arg: float, /) -> None: ...

    @property
    def upper_bounds_reduction(self) -> float: ...

    @upper_bounds_reduction.setter
    def upper_bounds_reduction(self, arg: float, /) -> None: ...

    @property
    def lower_bounds_reduction(self) -> float: ...

    @lower_bounds_reduction.setter
    def lower_bounds_reduction(self, arg: float, /) -> None: ...

class FixStateCollisionProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self, mode: FixStateCollisionProfile.Settings = FixStateCollisionProfile.Settings.ALL) -> None: ...

    class Settings(enum.Enum):
        START_ONLY = 0

        END_ONLY = 1

        INTERMEDIATE_ONLY = 2

        ALL = 3

        ALL_EXCEPT_START = 4

        ALL_EXCEPT_END = 5

        DISABLED = 6

    class CorrectionMethod(enum.Enum):
        NONE = 0

        TRAJOPT = 1

        RANDOM_SAMPLER = 2

    @property
    def mode(self) -> FixStateCollisionProfile.Settings: ...

    @mode.setter
    def mode(self, arg: FixStateCollisionProfile.Settings, /) -> None: ...

    @property
    def correction_workflow(self) -> list[FixStateCollisionProfile.CorrectionMethod]: ...

    @correction_workflow.setter
    def correction_workflow(self, arg: Sequence[FixStateCollisionProfile.CorrectionMethod], /) -> None: ...

    @property
    def jiggle_factor(self) -> float: ...

    @jiggle_factor.setter
    def jiggle_factor(self, arg: float, /) -> None: ...

    @property
    def contact_manager_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig: ...

    @contact_manager_config.setter
    def contact_manager_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.ContactManagerConfig, /) -> None: ...

    @property
    def collision_check_config(self) -> tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig: ...

    @collision_check_config.setter
    def collision_check_config(self, arg: tesseract_robotics.tesseract_collision._tesseract_collision.CollisionCheckConfig, /) -> None: ...

    @property
    def sampling_attempts(self) -> int: ...

    @sampling_attempts.setter
    def sampling_attempts(self, arg: int, /) -> None: ...

    @property
    def update_workspace(self) -> bool: ...

    @update_workspace.setter
    def update_workspace(self, arg: bool, /) -> None: ...

class KinematicLimitsCheckProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self, check_position: bool = True, check_velocity: bool = True, check_acceleration: bool = True) -> None: ...

    @property
    def check_position(self) -> bool: ...

    @check_position.setter
    def check_position(self, arg: bool, /) -> None: ...

    @property
    def check_velocity(self) -> bool: ...

    @check_velocity.setter
    def check_velocity(self, arg: bool, /) -> None: ...

    @property
    def check_acceleration(self) -> bool: ...

    @check_acceleration.setter
    def check_acceleration(self, arg: bool, /) -> None: ...

class MinLengthProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, min_length: int) -> None: ...

    @property
    def min_length(self) -> int: ...

    @min_length.setter
    def min_length(self, arg: int, /) -> None: ...

class ProfileSwitchProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    def __init__(self, return_value: int = 1) -> None: ...

    @property
    def return_value(self) -> int: ...

    @return_value.setter
    def return_value(self, arg: int, /) -> None: ...

class UpsampleTrajectoryProfile(tesseract_robotics.tesseract_command_language._tesseract_command_language.Profile):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, longest_valid_segment_length: float) -> None: ...

    @property
    def longest_valid_segment_length(self) -> float: ...

    @longest_valid_segment_length.setter
    def longest_valid_segment_length(self, arg: float, /) -> None: ...
