"""tesseract_task_composer Python bindings"""

from collections.abc import Mapping, Sequence
import datetime
from typing import overload

import tesseract_robotics.tesseract_common._tesseract_common


class TaskComposerPortMap:
    def __init__(self) -> None: ...

    @overload
    def set(self, port: str, storage_key: str) -> None: ...

    @overload
    def set(self, port: str, storage_keys: Sequence[str]) -> None: ...

    def erase(self, port: str) -> None: ...

    def contains(self, port: str) -> bool: ...

    def at(self, port: str) -> str | list[str]: ...

    def single(self, port: str) -> str: ...

    def multiple(self, port: str) -> list[str]: ...

    def renameStorageKeys(self, storage_key_remapping: Mapping[str, str]) -> None: ...

    def data(self) -> dict[str, str | list[str]]: ...

    def size(self) -> int: ...

    def empty(self) -> bool: ...

    def __eq__(self, arg: TaskComposerPortMap, /) -> bool: ...

    def __ne__(self, arg: TaskComposerPortMap, /) -> bool: ...

class TaskComposerDataStorage:
    def __init__(self) -> None: ...

    def getName(self) -> str: ...

    def setName(self, name: str) -> None: ...

    def hasKey(self, key: str) -> bool: ...

    def setData(self, key: str, data: AnyPoly) -> None: ...

    def getData(self, key: str) -> AnyPoly: ...

    def getAllData(self) -> dict[str, AnyPoly]: ...

    def removeData(self, key: str) -> None: ...

def createTaskComposerDataStorage() -> TaskComposerDataStorage:
    """Create a TaskComposerDataStorage as shared_ptr"""

class TaskComposerNodeInfo:
    @property
    def uuid(self) -> str: ...

    @property
    def parent_uuid(self) -> str: ...

    @property
    def name(self) -> str: ...

    @property
    def return_value(self) -> int: ...

    @property
    def status_code(self) -> int: ...

    @property
    def status_message(self) -> str: ...

    @property
    def elapsed_time(self) -> float: ...

    @property
    def data_storage(self) -> TaskComposerDataStorage: ...

    @property
    def dotgraph(self) -> str: ...

class TaskComposerNodeInfoContainer:
    def __init__(self) -> None: ...

    def getAbortingNodeInfo(self) -> TaskComposerNodeInfo | None: ...

    def getAllInfos(self) -> list[TaskComposerNodeInfo]: ...

class TaskComposerContext:
    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def dotgraph(self) -> bool: ...

    @dotgraph.setter
    def dotgraph(self, arg: bool, /) -> None: ...

    @property
    def data_storage(self) -> TaskComposerDataStorage: ...

    @data_storage.setter
    def data_storage(self, arg: TaskComposerDataStorage, /) -> None: ...

    def isAborted(self) -> bool: ...

    def isSuccessful(self) -> bool: ...

    @property
    def task_infos(self) -> TaskComposerNodeInfoContainer: ...

class TaskComposerNode:
    def getName(self) -> str: ...

    def getNamespace(self) -> str: ...

    def getUUIDString(self) -> str: ...

    def isConditional(self) -> bool: ...

    def getInputPortMappings(self) -> TaskComposerPortMap: ...

    def getOutputPortMappings(self) -> TaskComposerPortMap: ...

    def setPortMappings(self, input_port_mappings: TaskComposerPortMap, output_port_mappings: TaskComposerPortMap) -> None: ...

    @overload
    def getDotgraph(self) -> str:
        """Generate a DOT graph string for this task"""

    @overload
    def getDotgraph(self, results: TaskComposerNodeInfoContainer) -> str:
        """Generate a DOT graph string with task results annotations"""

    @overload
    def saveDotgraph(self, filepath: str) -> bool:
        """Write the DOT graph for this task to the given file path"""

    @overload
    def saveDotgraph(self, filepath: str, results: TaskComposerNodeInfoContainer) -> bool:
        """
        Write the DOT graph (with task results annotations) for this task to the given file path
        """

class TaskComposerFuture:
    @property
    def context(self) -> TaskComposerContext: ...

    @context.setter
    def context(self, arg: TaskComposerContext, /) -> None: ...

    def valid(self) -> bool: ...

    def ready(self) -> bool: ...

    def wait(self) -> None: ...

    def waitFor(self, duration: datetime.timedelta | float) -> "std::__1::future_status": ...

class TaskComposerExecutor:
    def getName(self) -> str: ...

    @overload
    def run(self, node: TaskComposerNode, context: TaskComposerContext) -> TaskComposerFuture: ...

    @overload
    def run(self, node: TaskComposerNode, data_storage: TaskComposerDataStorage, dotgraph: bool = False) -> TaskComposerFuture: ...

    def getWorkerCount(self) -> int: ...

    def getTaskCount(self) -> int: ...

class TaskflowTaskComposerExecutor(TaskComposerExecutor):
    @overload
    def __init__(self, name: str = 'TaskflowExecutor', num_threads: int = 10) -> None: ...

    @overload
    def __init__(self, num_threads: int) -> None: ...

    def getName(self) -> str: ...

    @overload
    def run(self, node: TaskComposerNode, context: TaskComposerContext) -> TaskComposerFuture: ...

    @overload
    def run(self, node: TaskComposerNode, data_storage: TaskComposerDataStorage, dotgraph: bool = False) -> TaskComposerFuture: ...

    def getWorkerCount(self) -> int: ...

    def getTaskCount(self) -> int: ...

class TaskComposerPluginFactory:
    def __init__(self, config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None:
        """Create from config file path (string) and a ResourceLocator"""

    def createTaskComposerExecutor(self, name: str) -> TaskComposerExecutor:
        """Create a task composer executor by name"""

    def createTaskComposerNode(self, name: str) -> TaskComposerNode:
        """Create a task composer node by name"""

    def hasTaskComposerExecutorPlugins(self) -> bool: ...

    def hasTaskComposerNodePlugins(self) -> bool: ...

    def getDefaultTaskComposerExecutorPlugin(self) -> str: ...

    def getDefaultTaskComposerNodePlugin(self) -> str: ...

def createTaskComposerPluginFactory(config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> TaskComposerPluginFactory:
    """
    Create a TaskComposerPluginFactory from a config file path (string) and a ResourceLocator
    """

class AnyPoly:
    def __init__(self) -> None: ...

    def isNull(self) -> bool: ...

    def getTypeName(self) -> str: ...

def AnyPoly_wrap_CompositeInstruction(instruction: "tesseract::command_language::CompositeInstruction") -> AnyPoly:
    """Wrap a CompositeInstruction into an AnyPoly"""

def AnyPoly_wrap_ProfileDictionary(profiles: "tesseract::common::ProfileDictionary") -> AnyPoly:
    """Wrap a ProfileDictionary shared_ptr into an AnyPoly"""

def AnyPoly_wrap_EnvironmentConst(environment: "tesseract::environment::Environment") -> AnyPoly:
    """Wrap a const Environment shared_ptr into an AnyPoly"""

def AnyPoly_wrap_TaskComposerDataStorage(data_storage: TaskComposerDataStorage) -> AnyPoly:
    """Wrap a TaskComposerDataStorage shared_ptr into an AnyPoly"""

def AnyPoly_as_CompositeInstruction(any_poly: AnyPoly) -> "tesseract::command_language::CompositeInstruction":
    """Extract a CompositeInstruction from an AnyPoly"""

def AnyPoly_as_TaskComposerDataStorage(any_poly: AnyPoly) -> TaskComposerDataStorage:
    """Extract a TaskComposerDataStorage from an AnyPoly"""

def AnyPoly_as_ContactResultMapVector(any_poly: AnyPoly) -> list["tesseract::collision::ContactResultMap"]:
    """
    Extract a std::vector<ContactResultMap> from an AnyPoly. DiscreteContactCheckTask saves its per-step contact results under the 'contact_results' key on its node info's data_storage in this form.
    """
