"""tesseract_collision Python bindings"""

from collections.abc import (
    Callable,
    Iterator,
    Mapping,
    Sequence,
    Set
)
import enum
import os
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray

import tesseract_robotics.common
import tesseract_robotics.geometry


class ContinuousCollisionType(enum.Enum):
    CCType_None = 0

    CCType_Time0 = 1

    CCType_Time1 = 2

    CCType_Between = 3

class ContactTestType(enum.Enum):
    FIRST = 0

    CLOSEST = 1

    ALL = 2

    LIMITED = 3

class CollisionEvaluatorType(enum.Enum):
    NONE = 0

    DISCRETE = 1

    LVS_DISCRETE = 2

    CONTINUOUS = 3

    LVS_CONTINUOUS = 4

class CollisionCheckProgramType(enum.Enum):
    ALL = 0

    ALL_EXCEPT_START = 1

    ALL_EXCEPT_END = 2

    START_ONLY = 3

    END_ONLY = 4

    INTERMEDIATE_ONLY = 5

class ACMOverrideType(enum.Enum):
    NONE = 0

    ASSIGN = 1

    AND = 2

    OR = 3

class CollisionCheckExitType(enum.Enum):
    FIRST = 0

    ONE_PER_STEP = 1

    ALL = 2

class ContactResult:
    def __init__(self) -> None: ...

    @property
    def distance(self) -> float: ...

    @distance.setter
    def distance(self, arg: float, /) -> None: ...

    @property
    def type_id(self) -> list[int]: ...

    @type_id.setter
    def type_id(self, arg: Sequence[int], /) -> None: ...

    @property
    def link_ids(self) -> list[tesseract_robotics.common.LinkId]: ...

    @link_ids.setter
    def link_ids(self, arg: Sequence[tesseract_robotics.common.LinkId | str], /) -> None: ...

    @property
    def shape_id(self) -> list[int]: ...

    @shape_id.setter
    def shape_id(self, arg: Sequence[int], /) -> None: ...

    @property
    def subshape_id(self) -> list[int]: ...

    @subshape_id.setter
    def subshape_id(self, arg: Sequence[int], /) -> None: ...

    @property
    def nearest_points(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]: ...

    @nearest_points.setter
    def nearest_points(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], /) -> None: ...

    @property
    def nearest_points_local(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]: ...

    @nearest_points_local.setter
    def nearest_points_local(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], /) -> None: ...

    @property
    def transform(self) -> list[tesseract_robotics.common.Isometry3d]: ...

    @transform.setter
    def transform(self, arg: Sequence[tesseract_robotics.common.Isometry3d], /) -> None: ...

    @property
    def normal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @normal.setter
    def normal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def cc_time(self) -> list[float]: ...

    @cc_time.setter
    def cc_time(self, arg: Sequence[float], /) -> None: ...

    @property
    def cc_type(self) -> list[ContinuousCollisionType]: ...

    @cc_type.setter
    def cc_type(self, arg: Sequence[ContinuousCollisionType], /) -> None: ...

    @property
    def cc_transform(self) -> list[tesseract_robotics.common.Isometry3d]: ...

    @cc_transform.setter
    def cc_transform(self, arg: Sequence[tesseract_robotics.common.Isometry3d], /) -> None: ...

    @property
    def single_contact_point(self) -> bool: ...

    @single_contact_point.setter
    def single_contact_point(self, arg: bool, /) -> None: ...

    def clear(self) -> None: ...

    def __eq__(self, arg: ContactResult, /) -> bool: ...

    def __ne__(self, arg: ContactResult, /) -> bool: ...

    __hash__: None = None

    def __copy__(self) -> ContactResult: ...

    def __deepcopy__(self, memo: dict) -> ContactResult: ...

class ContactResultHeldError(BufferError):
    """
    A ContactResultVector call would reallocate or destroy its elements while Python holds a reference to one (`x = v[i]`).
    """

class ContactResultVector:
    """
    List of ContactResult. `v[i]` is a reference into the vector: `v[i].distance = d` writes through. While Python holds such a reference, `append`, `clear` and `ContactResultMap.flattenMoveResults(v)` raise ContactResultHeldError and leave `v` unchanged; delete the reference, or hold a copy (`copy.copy(v[i])`), first.
    """

    def __init__(self) -> None: ...

    def __len__(self) -> int: ...

    def __getitem__(self, index: int) -> ContactResult:
        """
        Reference to the element at the index (write-through), kept valid by refusing reallocation while it is held.
        """

    def append(self, item: ContactResult) -> None:
        """
        Append a copy of `item`; raises ContactResultHeldError while an element is held.
        """

    def clear(self) -> None:
        """
        Remove every element; raises ContactResultHeldError while an element is held.
        """

class EmptyContactResultsError(ValueError):
    """A ContactResultMap vector overload got an empty ContactResultVector."""

class ContactResultMap:
    def __init__(self) -> None: ...

    def count(self) -> int: ...

    def size(self) -> int: ...

    def empty(self) -> bool: ...

    def clear(self) -> None: ...

    def release(self) -> None: ...

    def getSummary(self) -> str: ...

    def flattenCopyResults(self) -> ContactResultVector: ...

    def flattenMoveResults(self, results: ContactResultVector) -> None:
        """
        Move every result into `results` (cleared first); raises ContactResultHeldError, before moving anything, while an element of `results` is held.
        """

    def __len__(self) -> int: ...

    def at(self, key: tesseract_robotics.common.LinkIdPair | tuple[str, str]) -> ContactResultVector:
        """
        Copy of the results stored under `key`; raises KeyError when it is absent.
        """

    def getContainer(self) -> dict[tesseract_robotics.common.LinkIdPair, ContactResultVector]:
        """Copy of the underlying map, keyed by LinkIdPair."""

    def __iter__(self) -> Iterator[tuple[tesseract_robotics.common.LinkIdPair, ContactResultVector]]:
        """
        Iterate over (key, ContactResultVector) pairs, like C++ begin()/end(); a dict iterates keys only. Keys kept by clear() are included with empty vectors.
        """

    @overload
    def addContactResult(self, key: tesseract_robotics.common.LinkIdPair | tuple[str, str], result: ContactResult) -> ContactResult: ...

    @overload
    def addContactResult(self, key: tesseract_robotics.common.LinkIdPair | tuple[str, str], results: ContactResultVector) -> ContactResult: ...

    @overload
    def setContactResult(self, key: tesseract_robotics.common.LinkIdPair | tuple[str, str], result: ContactResult) -> ContactResult: ...

    @overload
    def setContactResult(self, key: tesseract_robotics.common.LinkIdPair | tuple[str, str], results: ContactResultVector) -> ContactResult: ...

    def shrinkToFit(self) -> None: ...

    def filter(self, fn: Callable[[tesseract_robotics.common.LinkIdPair | tuple[str, str], ContactResultVector], None]) -> None:
        """
        Call `fn(key, results)` for every pair; what `results` holds when `fn` returns replaces the pair's results (clear it to drop them). `results` is a copy: one kept past the call no longer edits the map.
        """

    def addInterpolatedCollisionResults(self, sub_segment_results: ContactResultMap, sub_segment_index: int, sub_segment_last_index: int, active_link_ids: Set[tesseract_robotics.common.LinkId | str], segment_dt: float, discrete: bool, filter: Callable[[tesseract_robotics.common.LinkIdPair | tuple[str, str], ContactResultVector], None] | None = None) -> None: ...

    def __eq__(self, arg: ContactResultMap, /) -> bool: ...

    def __ne__(self, arg: ContactResultMap, /) -> bool: ...

    __hash__: None = None

class ContactTrajectorySubstepResults:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, substep: int, start_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> None: ...

    @overload
    def __init__(self, substep: int, state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> None: ...

    def __bool__(self) -> bool: ...

    def addContact(self, substep_number: int, start_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], new_contacts: ContactResultMap) -> None: ...

    def numContacts(self) -> int: ...

    def worstCollision(self) -> ContactResultVector: ...

    @property
    def contacts(self) -> ContactResultMap: ...

    @contacts.setter
    def contacts(self, arg: ContactResultMap, /) -> None: ...

    @property
    def substep(self) -> int: ...

    @substep.setter
    def substep(self, arg: int, /) -> None: ...

    @property
    def state0(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @state0.setter
    def state0(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def state1(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @state1.setter
    def state1(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

class ContactTrajectoryStepResults:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, step_number: int, start_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], num_substeps: int) -> None: ...

    @overload
    def __init__(self, step_number: int, state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float]) -> None: ...

    def __bool__(self) -> bool: ...

    def addContact(self, step_number: int, substep_number: int, num_substeps: int, start_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], start_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], contacts: ContactResultMap) -> None: ...

    def resize(self, num_substeps: int) -> None: ...

    def numSubsteps(self) -> int: ...

    def numContacts(self) -> int: ...

    def worstSubstep(self) -> ContactTrajectorySubstepResults: ...

    def worstCollision(self) -> ContactResultVector: ...

    def mostCollisionsSubstep(self) -> ContactTrajectorySubstepResults: ...

    @property
    def substeps(self) -> list[ContactTrajectorySubstepResults]: ...

    @substeps.setter
    def substeps(self, arg: Sequence[ContactTrajectorySubstepResults], /) -> None: ...

    @property
    def step(self) -> int: ...

    @step.setter
    def step(self, arg: int, /) -> None: ...

    @property
    def state0(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @state0.setter
    def state0(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def state1(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]: ...

    @state1.setter
    def state1(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], /) -> None: ...

    @property
    def total_substeps(self) -> int: ...

    @total_substeps.setter
    def total_substeps(self, arg: int, /) -> None: ...

class ContactTrajectoryResults:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, j_ids: Sequence[tesseract_robotics.common.JointId | str]) -> None: ...

    @overload
    def __init__(self, j_ids: Sequence[tesseract_robotics.common.JointId | str], num_steps: int) -> None: ...

    def __bool__(self) -> bool: ...

    def addContact(self, step_number: int, substep_number: int, num_substeps: int, start_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_state: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], start_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], end_substate: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | Sequence[float], contacts: ContactResultMap) -> None: ...

    def resize(self, num_steps: int) -> None: ...

    def numSteps(self) -> int: ...

    def numContacts(self) -> int: ...

    def worstStep(self) -> ContactTrajectoryStepResults: ...

    def worstCollision(self) -> ContactResultVector: ...

    def mostCollisionsStep(self) -> ContactTrajectoryStepResults: ...

    def trajectoryCollisionResultsTable(self) -> str: ...

    def collisionFrequencyPerLink(self) -> str: ...

    def condensedSummary(self) -> str: ...

    @property
    def steps(self) -> list[ContactTrajectoryStepResults]: ...

    @steps.setter
    def steps(self, arg: Sequence[ContactTrajectoryStepResults], /) -> None: ...

    @property
    def joint_ids(self) -> list[tesseract_robotics.common.JointId]: ...

    @joint_ids.setter
    def joint_ids(self, arg: Sequence[tesseract_robotics.common.JointId | str], /) -> None: ...

    @property
    def total_steps(self) -> int: ...

    @total_steps.setter
    def total_steps(self, arg: int, /) -> None: ...

class ContactResultValidator:
    """
    Approves or rejects contact results: subclass and implement `__call__(result) -> bool`.
    """

    def __init__(self) -> None: ...

    def __call__(self, result: ContactResult) -> bool: ...

class ContactRequest:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, type: ContactTestType) -> None: ...

    @property
    def type(self) -> ContactTestType: ...

    @type.setter
    def type(self, arg: ContactTestType, /) -> None: ...

    @property
    def calculate_penetration(self) -> bool: ...

    @calculate_penetration.setter
    def calculate_penetration(self, arg: bool, /) -> None: ...

    @property
    def calculate_distance(self) -> bool: ...

    @calculate_distance.setter
    def calculate_distance(self, arg: bool, /) -> None: ...

    @property
    def contact_limit(self) -> int: ...

    @contact_limit.setter
    def contact_limit(self, arg: int, /) -> None: ...

    @property
    def is_valid(self) -> ContactResultValidator | None:
        """
        Validator called on each contact; return False to reject it. None disables validation.
        """

    @is_valid.setter
    def is_valid(self, value: ContactResultValidator | None) -> None: ...

    def __eq__(self, arg: ContactRequest, /) -> bool: ...

    def __ne__(self, arg: ContactRequest, /) -> bool: ...

    __hash__: None = None

class ContactManagerConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, default_margin: float) -> None: ...

    @property
    def default_margin(self) -> float | None: ...

    @default_margin.setter
    def default_margin(self, arg: float, /) -> None: ...

    @property
    def pair_margin_override_type(self) -> tesseract_robotics.common.CollisionMarginPairOverrideType: ...

    @pair_margin_override_type.setter
    def pair_margin_override_type(self, arg: tesseract_robotics.common.CollisionMarginPairOverrideType, /) -> None: ...

    @property
    def pair_margin_data(self) -> tesseract_robotics.common.CollisionMarginPairData: ...

    @pair_margin_data.setter
    def pair_margin_data(self, arg: tesseract_robotics.common.CollisionMarginPairData, /) -> None: ...

    @property
    def acm(self) -> tesseract_robotics.common.AllowedCollisionMatrix: ...

    @acm.setter
    def acm(self, arg: tesseract_robotics.common.AllowedCollisionMatrix, /) -> None: ...

    @property
    def acm_override_type(self) -> ACMOverrideType: ...

    @acm_override_type.setter
    def acm_override_type(self, arg: ACMOverrideType, /) -> None: ...

    @property
    def modify_object_enabled(self) -> dict[tesseract_robotics.common.LinkId, bool]: ...

    @modify_object_enabled.setter
    def modify_object_enabled(self, arg: Mapping[tesseract_robotics.common.LinkId, bool] | Mapping[str, bool], /) -> None: ...

    def incrementMargins(self, increment: float) -> None: ...

    def scaleMargins(self, scale: float) -> None: ...

    def validate(self) -> None: ...

    def __eq__(self, arg: ContactManagerConfig, /) -> bool: ...

    def __ne__(self, arg: ContactManagerConfig, /) -> bool: ...

    __hash__: None = None

class CollisionCheckConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, contact_request: ContactRequest = ..., type: CollisionEvaluatorType = CollisionEvaluatorType.DISCRETE, longest_valid_segment_length: float = 0.005, check_program_mode: CollisionCheckProgramType = CollisionCheckProgramType.ALL, exit_condition: CollisionCheckExitType = CollisionCheckExitType.FIRST) -> None: ...

    @property
    def contact_request(self) -> ContactRequest: ...

    @contact_request.setter
    def contact_request(self, arg: ContactRequest, /) -> None: ...

    @property
    def type(self) -> CollisionEvaluatorType: ...

    @type.setter
    def type(self, arg: CollisionEvaluatorType, /) -> None: ...

    @property
    def longest_valid_segment_length(self) -> float: ...

    @longest_valid_segment_length.setter
    def longest_valid_segment_length(self, arg: float, /) -> None: ...

    @property
    def check_program_mode(self) -> CollisionCheckProgramType: ...

    @check_program_mode.setter
    def check_program_mode(self, arg: CollisionCheckProgramType, /) -> None: ...

    @property
    def exit_condition(self) -> CollisionCheckExitType: ...

    @exit_condition.setter
    def exit_condition(self, arg: CollisionCheckExitType, /) -> None: ...

    def __eq__(self, arg: CollisionCheckConfig, /) -> bool: ...

    def __ne__(self, arg: CollisionCheckConfig, /) -> bool: ...

    __hash__: None = None

class CollisionObjectSpec:
    """A single collision object's definition, for addCollisionObjects."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, id: tesseract_robotics.common.LinkId | str, mask_id: int, shapes: Sequence[tesseract_robotics.geometry.Geometry], shape_poses: Sequence[tesseract_robotics.common.Isometry3d], enabled: bool = True) -> None: ...

    @property
    def id(self) -> tesseract_robotics.common.LinkId: ...

    @id.setter
    def id(self, arg: tesseract_robotics.common.LinkId | str, /) -> None: ...

    @property
    def mask_id(self) -> int: ...

    @mask_id.setter
    def mask_id(self, arg: int, /) -> None: ...

    @property
    def shapes(self) -> list[tesseract_robotics.geometry.Geometry]: ...

    @shapes.setter
    def shapes(self, arg: Sequence[tesseract_robotics.geometry.Geometry], /) -> None: ...

    @property
    def shape_poses(self) -> list[tesseract_robotics.common.Isometry3d]: ...

    @shape_poses.setter
    def shape_poses(self, arg: Sequence[tesseract_robotics.common.Isometry3d], /) -> None: ...

    @property
    def enabled(self) -> bool: ...

    @enabled.setter
    def enabled(self, arg: bool, /) -> None: ...

class DiscreteContactManager:
    def getName(self) -> str: ...

    def hasCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def removeCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def enableCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def disableCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def isCollisionObjectEnabled(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    @overload
    def setCollisionObjectsTransform(self, id: tesseract_robotics.common.LinkId | str, pose: tesseract_robotics.common.Isometry3d) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], poses: Sequence[tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, transforms: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Set each id's transform from `state`; raises IndexError if `state` lacks an id.
        """

    @overload
    def setCollisionObjectsTransform(self, ids: Set[tesseract_robotics.common.LinkId | str], state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Set each id's transform from `state`; raises IndexError if `state` lacks an id.
        """

    def getCollisionObjects(self) -> list[tesseract_robotics.common.LinkId]: ...

    def setActiveCollisionObjects(self, ids: Sequence[tesseract_robotics.common.LinkId | str]) -> None: ...

    def getActiveCollisionObjects(self) -> set[tesseract_robotics.common.LinkId]: ...

    def setDefaultCollisionMargin(self, default_collision_margin: float) -> None: ...

    def setCollisionMarginPair(self, name1: tesseract_robotics.common.LinkId | str, name2: tesseract_robotics.common.LinkId | str, collision_margin: float) -> None: ...

    def setCollisionMarginData(self, collision_margin_data: tesseract_robotics.common.CollisionMarginData) -> None: ...

    def setCollisionMarginPairData(self, pair_margin_data: tesseract_robotics.common.CollisionMarginPairData, override_type: tesseract_robotics.common.CollisionMarginPairOverrideType = tesseract_robotics.common.CollisionMarginPairOverrideType.REPLACE) -> None: ...

    def incrementCollisionMargin(self, increment: float) -> None: ...

    def setContactAllowedValidator(self, validator: tesseract_robotics.common.ContactAllowedValidator) -> None: ...

    def getContactAllowedValidator(self) -> tesseract_robotics.common.ContactAllowedValidator: ...

    def applyContactManagerConfig(self, config: ContactManagerConfig) -> None: ...

    def getCollisionMarginData(self) -> tesseract_robotics.common.CollisionMarginData: ...

    def addCollisionObject(self, name: tesseract_robotics.common.LinkId | str, mask_id: int, shapes: Sequence[tesseract_robotics.geometry.Geometry], shape_poses: Sequence[tesseract_robotics.common.Isometry3d], enabled: bool = True) -> bool: ...

    def getCollisionObjectGeometries(self, name: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.geometry.Geometry]: ...

    def getCollisionObjectGeometriesTransforms(self, name: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.common.Isometry3d]: ...

    def contactTest(self, collisions: ContactResultMap, request: ContactRequest) -> None: ...

    def clone(self) -> DiscreteContactManager:
        """An independent copy; it keeps this object alive (gh-72)."""

    def addCollisionObjects(self, objects: Sequence[CollisionObjectSpec]) -> bool:
        """
        Add the objects in order, as repeated addCollisionObject calls; a repeated id: the last entry decides. False if any failed; the rest are still added.
        """

    def removeCollisionObjects(self, ids: Sequence[tesseract_robotics.common.LinkId | str]) -> bool:
        """
        Remove the objects; False if any id was absent (the present ones are still removed).
        """

    @overload
    def setCollisionObjectsEnabled(self, enabled: Mapping[tesseract_robotics.common.LinkId, bool] | Mapping[str, bool]) -> bool:
        """Enable (True) or disable (False) each id; False if any id was absent."""

    @overload
    def setCollisionObjectsEnabled(self, ids: Sequence[tesseract_robotics.common.LinkId | str], enabled: bool) -> bool:
        """Enable or disable every id; False if any id was absent."""

    def getCollisionObjectsTransform(self, id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """
        The object's world transform; raises IndexError for an id the manager does not hold.
        """

class ContinuousContactManager:
    def getName(self) -> str: ...

    def hasCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def removeCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def enableCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def disableCollisionObject(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def isCollisionObjectEnabled(self, name: tesseract_robotics.common.LinkId | str) -> bool: ...

    def addCollisionObject(self, name: tesseract_robotics.common.LinkId | str, mask_id: int, shapes: Sequence[tesseract_robotics.geometry.Geometry], shape_poses: Sequence[tesseract_robotics.common.Isometry3d], enabled: bool = True) -> bool: ...

    def getCollisionObjectGeometries(self, name: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.geometry.Geometry]: ...

    def getCollisionObjectGeometriesTransforms(self, name: tesseract_robotics.common.LinkId | str) -> list[tesseract_robotics.common.Isometry3d]: ...

    @overload
    def setCollisionObjectsTransform(self, id: tesseract_robotics.common.LinkId | str, pose: tesseract_robotics.common.Isometry3d) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], poses: Sequence[tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, transforms: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, id: tesseract_robotics.common.LinkId | str, pose1: tesseract_robotics.common.Isometry3d, pose2: tesseract_robotics.common.Isometry3d) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], pose1: Sequence[tesseract_robotics.common.Isometry3d], pose2: Sequence[tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, pose1: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], pose2: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], state0: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], state1: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Cast each id from `state0` to `state1`; raises IndexError, moving nothing, if either map lacks an id. An id both maps have and the manager does not hold is skipped, with no error.
        """

    @overload
    def setCollisionObjectsTransform(self, ids: Set[tesseract_robotics.common.LinkId | str], state0: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], state1: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Cast each id from `state0` to `state1`; raises IndexError, moving nothing, if either map lacks an id. An id both maps have and the manager does not hold is skipped, with no error.
        """

    @overload
    def setCollisionObjectsTransform(self, ids: Sequence[tesseract_robotics.common.LinkId | str], state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Set each id's transform from `state`; raises IndexError if `state` lacks an id.
        """

    @overload
    def setCollisionObjectsTransform(self, ids: Set[tesseract_robotics.common.LinkId | str], state: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None:
        """
        Set each id's transform from `state`; raises IndexError if `state` lacks an id.
        """

    @overload
    def setCollisionObjectsTransformCast(self, id: tesseract_robotics.common.LinkId | str, pose1: tesseract_robotics.common.Isometry3d, pose2: tesseract_robotics.common.Isometry3d) -> None: ...

    @overload
    def setCollisionObjectsTransformCast(self, ids: Sequence[tesseract_robotics.common.LinkId | str], pose1: Sequence[tesseract_robotics.common.Isometry3d], pose2: Sequence[tesseract_robotics.common.Isometry3d]) -> None: ...

    @overload
    def setCollisionObjectsTransformCast(self, pose1: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d], pose2: Mapping[tesseract_robotics.common.LinkId, tesseract_robotics.common.Isometry3d] | Mapping[str, tesseract_robotics.common.Isometry3d]) -> None: ...

    def getCollisionObjects(self) -> list[tesseract_robotics.common.LinkId]: ...

    def setActiveCollisionObjects(self, ids: Sequence[tesseract_robotics.common.LinkId | str]) -> None: ...

    def getActiveCollisionObjects(self) -> set[tesseract_robotics.common.LinkId]: ...

    def setCollisionMarginData(self, collision_margin_data: tesseract_robotics.common.CollisionMarginData) -> None: ...

    def getCollisionMarginData(self) -> tesseract_robotics.common.CollisionMarginData: ...

    def setDefaultCollisionMargin(self, default_collision_margin: float) -> None: ...

    def setCollisionMarginPair(self, name1: tesseract_robotics.common.LinkId | str, name2: tesseract_robotics.common.LinkId | str, collision_margin: float) -> None: ...

    def setCollisionMarginPairData(self, pair_margin_data: tesseract_robotics.common.CollisionMarginPairData, override_type: tesseract_robotics.common.CollisionMarginPairOverrideType = tesseract_robotics.common.CollisionMarginPairOverrideType.REPLACE) -> None: ...

    def incrementCollisionMargin(self, increment: float) -> None: ...

    def setContactAllowedValidator(self, validator: tesseract_robotics.common.ContactAllowedValidator) -> None: ...

    def getContactAllowedValidator(self) -> tesseract_robotics.common.ContactAllowedValidator: ...

    def applyContactManagerConfig(self, config: ContactManagerConfig) -> None: ...

    def contactTest(self, collisions: ContactResultMap, request: ContactRequest) -> None: ...

    def clone(self) -> ContinuousContactManager:
        """An independent copy; it keeps this object alive (gh-72)."""

    def addCollisionObjects(self, objects: Sequence[CollisionObjectSpec]) -> bool:
        """
        Add the objects in order, as repeated addCollisionObject calls; a repeated id: the last entry decides. False if any failed; the rest are still added.
        """

    def removeCollisionObjects(self, ids: Sequence[tesseract_robotics.common.LinkId | str]) -> bool:
        """
        Remove the objects; False if any id was absent (the present ones are still removed).
        """

    @overload
    def setCollisionObjectsEnabled(self, enabled: Mapping[tesseract_robotics.common.LinkId, bool] | Mapping[str, bool]) -> bool:
        """Enable (True) or disable (False) each id; False if any id was absent."""

    @overload
    def setCollisionObjectsEnabled(self, ids: Sequence[tesseract_robotics.common.LinkId | str], enabled: bool) -> bool:
        """Enable or disable every id; False if any id was absent."""

    def getCollisionObjectsTransform(self, id: tesseract_robotics.common.LinkId | str) -> tesseract_robotics.common.Isometry3d:
        """
        The object's world transform; raises IndexError for an id the manager does not hold.
        """

class ImplicitSDFShape:
    """
    Callback representation of a signed-distance shape in world coordinates.
    """

    def __init__(self) -> None: ...

    @property
    def distance(self) -> Callable[[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], float]:
        """Signed distance at a world point (negative inside)"""

    @distance.setter
    def distance(self, arg: Callable[[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], float], /) -> None: ...

    @property
    def gradient(self) -> Callable[[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]:
        """Gradient of `distance` at a world point"""

    @gradient.setter
    def gradient(self, arg: Callable[[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], /) -> None: ...

    @property
    def aabb(self) -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]:
        """
        World axis-aligned bounds of the region searched for contacts, as (min, max)
        """

    @aabb.setter
    def aabb(self, arg: tuple[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], /) -> None: ...

    def isValid(self) -> bool:
        """Whether the shape has all data the solver needs"""

class ImplicitSDFCollisionConfig:
    """Settings of the multi-start implicit SDF collision solver."""

    def __init__(self) -> None: ...

    @property
    def initial_sample_count(self) -> int: ...

    @initial_sample_count.setter
    def initial_sample_count(self, arg: int, /) -> None: ...

    @property
    def max_iterations(self) -> int: ...

    @max_iterations.setter
    def max_iterations(self, arg: int, /) -> None: ...

    @property
    def max_contacts(self) -> int: ...

    @max_contacts.setter
    def max_contacts(self, arg: int, /) -> None: ...

    @property
    def contact_margin(self) -> float: ...

    @contact_margin.setter
    def contact_margin(self, arg: float, /) -> None: ...

    @property
    def minimum_step(self) -> float: ...

    @minimum_step.setter
    def minimum_step(self, arg: float, /) -> None: ...

    @property
    def duplicate_tolerance(self) -> float: ...

    @duplicate_tolerance.setter
    def duplicate_tolerance(self, arg: float, /) -> None: ...

class ImplicitSDFContact:
    """A contact between two implicit shapes."""

    def __init__(self) -> None: ...

    @property
    def distance(self) -> float: ...

    @distance.setter
    def distance(self, arg: float, /) -> None: ...

    @property
    def point(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @point.setter
    def point(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def normal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @normal.setter
    def normal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float], /) -> None: ...

    @property
    def nearest_points(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]: ...

    @nearest_points.setter
    def nearest_points(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], /) -> None: ...

def collideImplicitSDF(shape0: ImplicitSDFShape, shape1: ImplicitSDFShape, config: ImplicitSDFCollisionConfig = ...) -> list[ImplicitSDFContact]:
    """Contacts between two implicit shapes, within config.contact_margin."""

def makeImplicitSDFShape(geometry: tesseract_robotics.geometry.Geometry, pose: tesseract_robotics.common.Isometry3d) -> ImplicitSDFShape:
    """
    World-coordinate implicit shape of a box, sphere, cylinder, cone, capsule or SDF geometry; an invalid shape (isValid() False) for other geometry. The shape's callables keep the geometry alive.
    """

class ContactManagersPluginFactory:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, config_path: os.PathLike, locator: tesseract_robotics.common.ResourceLocator) -> None: ...

    @overload
    def __init__(self, config: str, locator: tesseract_robotics.common.ResourceLocator) -> None: ...

    def addSearchPath(self, path: str) -> None: ...

    def getSearchPaths(self) -> list[str]: ...

    def clearSearchPaths(self) -> None: ...

    def addSearchLibrary(self, library_name: str) -> None: ...

    def getSearchLibraries(self) -> list[str]: ...

    def clearSearchLibraries(self) -> None: ...

    def addDiscreteContactManagerPlugin(self, name: str, plugin_info: tesseract_robotics.common.PluginInfo) -> None: ...

    def getDiscreteContactManagerPlugins(self) -> dict[str, tesseract_robotics.common.PluginInfo]: ...

    def removeDiscreteContactManagerPlugin(self, name: str) -> None: ...

    def setDefaultDiscreteContactManagerPlugin(self, name: str) -> None: ...

    def hasDiscreteContactManagerPlugins(self) -> bool: ...

    def getDefaultDiscreteContactManagerPlugin(self) -> str: ...

    def addContinuousContactManagerPlugin(self, name: str, plugin_info: tesseract_robotics.common.PluginInfo) -> None: ...

    def getContinuousContactManagerPlugins(self) -> dict[str, tesseract_robotics.common.PluginInfo]: ...

    def removeContinuousContactManagerPlugin(self, name: str) -> None: ...

    def setDefaultContinuousContactManagerPlugin(self, name: str) -> None: ...

    def hasContinuousContactManagerPlugins(self) -> bool: ...

    def getDefaultContinuousContactManagerPlugin(self) -> str: ...

    @overload
    def createDiscreteContactManager(self, name: str) -> DiscreteContactManager: ...

    @overload
    def createDiscreteContactManager(self, name: str, plugin_info: tesseract_robotics.common.PluginInfo) -> DiscreteContactManager: ...

    @overload
    def createContinuousContactManager(self, name: str) -> ContinuousContactManager: ...

    @overload
    def createContinuousContactManager(self, name: str, plugin_info: tesseract_robotics.common.PluginInfo) -> ContinuousContactManager: ...

    def saveConfig(self, file_path: str | os.PathLike) -> None: ...

    def getConfig(self) -> str:
        """The factory configuration as a YAML document string."""

def makeConvexMesh(mesh: tesseract_robotics.geometry.Mesh) -> tesseract_robotics.geometry.ConvexMesh:
    """Create a ConvexMesh from a Mesh using bullet's convex hull algorithm."""

class ConvexHullError(RuntimeError):
    """createConvexHull failed: Bullet could not apply the requested shrink."""

def createConvexHull(input: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], shrink: float = -1.0, shrink_clamp: float = -1.0) -> tuple[int, list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]]:
    """
    Convex hull of a point set (Bullet). Returns (n_faces, vertices, faces); faces is a flat array of [count, i0, i1, ...] per face. shrink > 0 moves each face inwards by that amount; shrink_clamp > 0 caps it at shrink_clamp times the hull's inner radius. Raises ConvexHullError when the shrink cannot be applied.
    """

class MalformedFacesError(ValueError):
    """
    A faces array has a count running past its end or an index outside the vertices.
    """

class NonTriangleFaceError(ValueError):
    """
    ConvexDecomposition.compute got a face that is not a triangle (count != 3).
    """

class FillMode(enum.Enum):
    FLOOD_FILL = 0

    SURFACE_ONLY = 1

    RAYCAST_FILL = 2

class VHACDParameters:
    def __init__(self) -> None: ...

    @property
    def max_convex_hulls(self) -> int: ...

    @max_convex_hulls.setter
    def max_convex_hulls(self, arg: int, /) -> None: ...

    @property
    def resolution(self) -> int: ...

    @resolution.setter
    def resolution(self, arg: int, /) -> None: ...

    @property
    def minimum_volume_percent_error_allowed(self) -> float: ...

    @minimum_volume_percent_error_allowed.setter
    def minimum_volume_percent_error_allowed(self, arg: float, /) -> None: ...

    @property
    def max_recursion_depth(self) -> int: ...

    @max_recursion_depth.setter
    def max_recursion_depth(self, arg: int, /) -> None: ...

    @property
    def shrinkwrap(self) -> bool: ...

    @shrinkwrap.setter
    def shrinkwrap(self, arg: bool, /) -> None: ...

    @property
    def fill_mode(self) -> FillMode: ...

    @fill_mode.setter
    def fill_mode(self, arg: FillMode, /) -> None: ...

    @property
    def max_num_vertices_per_ch(self) -> int: ...

    @max_num_vertices_per_ch.setter
    def max_num_vertices_per_ch(self, arg: int, /) -> None: ...

    @property
    def async_ACD(self) -> bool: ...

    @async_ACD.setter
    def async_ACD(self, arg: bool, /) -> None: ...

    @property
    def min_edge_length(self) -> int: ...

    @min_edge_length.setter
    def min_edge_length(self, arg: int, /) -> None: ...

    @property
    def find_best_plane(self) -> bool: ...

    @find_best_plane.setter
    def find_best_plane(self, arg: bool, /) -> None: ...

    def print(self) -> None:
        """Print the parameters to stdout."""

class ConvexDecomposition:
    def compute(self, vertices: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] | Sequence[float]], faces: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')] | Sequence[int], verbose: bool = True) -> list[tesseract_robotics.geometry.ConvexMesh]:
        """
        Split a triangle mesh into convex hulls. faces is a flat array of [3, i0, i1, i2] per triangle. Raises NonTriangleFaceError for any other count, MalformedFacesError for a count running past the end or an index outside vertices.
        """

class ConvexDecompositionVHACD(ConvexDecomposition):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, params: VHACDParameters) -> None: ...
