"""tesseract_task_composer Python bindings"""

from collections.abc import Callable, Mapping, Sequence
import datetime
import enum
import os
from typing import overload

import tesseract_robotics.tesseract_collision._tesseract_collision
import tesseract_robotics.tesseract_command_language._tesseract_command_language
import tesseract_robotics.tesseract_common._tesseract_common
import tesseract_robotics.tesseract_environment._tesseract_environment


class TaskComposerKeys:
    def __init__(self) -> None: ...

    @overload
    def add(self, port: str, key: str) -> None: ...

    @overload
    def add(self, port: str, keys: Sequence[str]) -> None: ...

    def remove(self, port: str) -> None: ...

    def rename(self, keys: Mapping[str, str]) -> None: ...

    def get(self, port: str) -> str | list[str]:
        """
        The key, or list of keys, of `port`. Raises IndexError for an unknown port.
        """

    def has(self, port: str) -> bool: ...

    def empty(self) -> bool: ...

    def size(self) -> int: ...

    def data(self) -> dict[str, str | list[str]]: ...

    def __str__(self) -> str: ...

    def __eq__(self, arg: TaskComposerKeys, /) -> bool: ...

    def __ne__(self, arg: TaskComposerKeys, /) -> bool: ...

    __hash__: None = None

class TaskComposerDataStorage:
    def __init__(self, name: str = '') -> None: ...

    def getName(self) -> str: ...

    def setName(self, name: str) -> None: ...

    def hasKey(self, key: str) -> bool: ...

    def setData(self, key: str, data: AnyPoly) -> None: ...

    @overload
    def getData(self, key: str) -> AnyPoly: ...

    @overload
    def getData(self) -> dict[str, AnyPoly]:
        """Every entry, key -> AnyPoly (a copy)."""

    def removeData(self, key: str) -> None: ...

    def remapData(self, remapping: Mapping[str, str], copy: bool = False) -> bool:
        """
        Rename (or with copy=True, duplicate) entries old key -> new key. Returns False, with a console_bridge error, at the first missing key.
        """

    def copyAsInputData(self, data_storage: TaskComposerDataStorage, keys: TaskComposerKeys, override_keys: TaskComposerKeys) -> None:
        """
        For each port in `keys`: self[key] = data_storage[override key, else key].
        """

    def copyAsOutputData(self, data_storage: TaskComposerDataStorage, keys: TaskComposerKeys, override_keys: TaskComposerKeys) -> None:
        """
        For each port in `keys`: self[override key, else key] = data_storage[key].
        """

    def __eq__(self, arg: TaskComposerDataStorage, /) -> bool: ...

    def __ne__(self, arg: TaskComposerDataStorage, /) -> bool: ...

    __hash__: None = None

class TaskComposerNodeType(enum.Enum):
    NODE = 0

    TASK = 1

    PIPELINE = 2

    GRAPH = 3

class TaskComposerNodePorts:
    def __init__(self) -> None: ...

    class Type(enum.Enum):
        SINGLE = 0

        MULTIPLE = 1

    @property
    def input_required(self) -> dict[str, TaskComposerNodePorts.Type]: ...

    @input_required.setter
    def input_required(self, arg: Mapping[str, TaskComposerNodePorts.Type], /) -> None: ...

    @property
    def input_optional(self) -> dict[str, TaskComposerNodePorts.Type]: ...

    @input_optional.setter
    def input_optional(self, arg: Mapping[str, TaskComposerNodePorts.Type], /) -> None: ...

    @property
    def output_required(self) -> dict[str, TaskComposerNodePorts.Type]: ...

    @output_required.setter
    def output_required(self, arg: Mapping[str, TaskComposerNodePorts.Type], /) -> None: ...

    @property
    def output_optional(self) -> dict[str, TaskComposerNodePorts.Type]: ...

    @output_optional.setter
    def output_optional(self, arg: Mapping[str, TaskComposerNodePorts.Type], /) -> None: ...

    def __str__(self) -> str: ...

    def __eq__(self, arg: TaskComposerNodePorts, /) -> bool: ...

    def __ne__(self, arg: TaskComposerNodePorts, /) -> bool: ...

    __hash__: None = None

class TaskComposerNodeInfo:
    def __init__(self, node: TaskComposerNode) -> None: ...

    @property
    def uuid(self) -> str: ...

    @uuid.setter
    def uuid(self, arg: str, /) -> None: ...

    @property
    def root_uuid(self) -> str: ...

    @root_uuid.setter
    def root_uuid(self, arg: str, /) -> None: ...

    @property
    def parent_uuid(self) -> str: ...

    @parent_uuid.setter
    def parent_uuid(self, arg: str, /) -> None: ...

    @property
    def inbound_edges(self) -> list[str]: ...

    @inbound_edges.setter
    def inbound_edges(self, arg: Sequence[str], /) -> None: ...

    @property
    def outbound_edges(self) -> list[str]: ...

    @outbound_edges.setter
    def outbound_edges(self, arg: Sequence[str], /) -> None: ...

    @property
    def terminals(self) -> list[str]: ...

    @terminals.setter
    def terminals(self, arg: Sequence[str], /) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def ns(self) -> str: ...

    @ns.setter
    def ns(self, arg: str, /) -> None: ...

    @property
    def type(self) -> TaskComposerNodeType: ...

    @type.setter
    def type(self, arg: TaskComposerNodeType, /) -> None: ...

    @property
    def type_hash_code(self) -> int: ...

    @type_hash_code.setter
    def type_hash_code(self, arg: int, /) -> None: ...

    @property
    def conditional(self) -> bool: ...

    @conditional.setter
    def conditional(self, arg: bool, /) -> None: ...

    @property
    def input_keys(self) -> TaskComposerKeys: ...

    @input_keys.setter
    def input_keys(self, arg: TaskComposerKeys, /) -> None: ...

    @property
    def output_keys(self) -> TaskComposerKeys: ...

    @output_keys.setter
    def output_keys(self, arg: TaskComposerKeys, /) -> None: ...

    @property
    def triggers_abort(self) -> bool: ...

    @triggers_abort.setter
    def triggers_abort(self, arg: bool, /) -> None: ...

    @property
    def return_value(self) -> int: ...

    @return_value.setter
    def return_value(self, arg: int, /) -> None: ...

    @property
    def status_code(self) -> int: ...

    @status_code.setter
    def status_code(self, arg: int, /) -> None: ...

    @property
    def status_message(self) -> str: ...

    @status_message.setter
    def status_message(self, arg: str, /) -> None: ...

    @property
    def start_time(self) -> datetime.datetime: ...

    @start_time.setter
    def start_time(self, arg: datetime.datetime | datetime.date | datetime.time, /) -> None: ...

    @property
    def elapsed_time(self) -> float: ...

    @elapsed_time.setter
    def elapsed_time(self, arg: float, /) -> None: ...

    @property
    def color(self) -> str: ...

    @color.setter
    def color(self, arg: str, /) -> None: ...

    @property
    def dotgraph(self) -> str: ...

    @dotgraph.setter
    def dotgraph(self, arg: str, /) -> None: ...

    @property
    def data_storage(self) -> TaskComposerDataStorage: ...

    @data_storage.setter
    def data_storage(self, arg: TaskComposerDataStorage, /) -> None: ...

    @property
    def aborted(self) -> bool: ...

    @aborted.setter
    def aborted(self, arg: bool, /) -> None: ...

    def __eq__(self, arg: TaskComposerNodeInfo, /) -> bool: ...

    def __ne__(self, arg: TaskComposerNodeInfo, /) -> bool: ...

    __hash__: None = None

class TaskComposerServerBusyError(RuntimeError):
    """
    TaskComposerServer.loadConfig / addTask would replace a task that a run started by name is still using; wait() for the run's future first.
    """

class TaskComposerPruneConflictError(RuntimeError):
    """
    TaskComposerNodeInfoContainer.prune: an info changed or was removed while prune ran, so the edit of its stale copy was not stored.
    """

class TaskComposerNodeInfoContainer:
    def __init__(self) -> None: ...

    def addInfo(self, info: TaskComposerNodeInfo) -> None: ...

    def getInfo(self, key: str) -> TaskComposerNodeInfo | None:
        """A copy of the info of node `key`, or None."""

    def getInfoMap(self) -> dict[str, TaskComposerNodeInfo]:
        """Every info, node uuid -> info (a copy)."""

    def setRootNode(self, node_uuid: str) -> None: ...

    def getRootNode(self) -> str: ...

    def setAborted(self, node_uuid: str) -> None: ...

    def getAbortingNode(self) -> str:
        """The uuid of the node that aborted the run; the nil uuid if none did."""

    def find(self, search_fn: Callable[[TaskComposerNodeInfo], bool]) -> list[TaskComposerNodeInfo]:
        """
        Copies of the infos for which search_fn(info) is true. search_fn sees a snapshot taken before the first call, and may call back into the container.
        """

    def insertInfoMap(self, container: TaskComposerNodeInfoContainer) -> None: ...

    def mergeInfoMap(self, container: TaskComposerNodeInfoContainer) -> None: ...

    def clear(self) -> None: ...

    def prune(self, prune_fn: Callable[[TaskComposerNodeInfo], None]) -> None:
        """
        Call prune_fn(info) on a copy of every stored info, then store the copies it changed. prune_fn sees a snapshot taken before the first call, and may call back into the container. A changed copy is stored only if the stored info is still the one prune_fn was given; if any was changed or removed meanwhile, the other edits are stored and TaskComposerPruneConflictError names the conflicting uuids. Edits made to `info` after prune returns, or before prune_fn raises, are not stored.
        """

    def __eq__(self, arg: TaskComposerNodeInfoContainer, /) -> bool: ...

    def __ne__(self, arg: TaskComposerNodeInfoContainer, /) -> bool: ...

    __hash__: None = None

class TaskComposerContext:
    @overload
    def __init__(self, name: str, dotgraph: bool = False) -> None: ...

    @overload
    def __init__(self, name: str, data_storage: TaskComposerDataStorage, dotgraph: bool = False) -> None: ...

    @overload
    def __init__(self, name: str, data_storage: TaskComposerDataStorage, task_infos: TaskComposerNodeInfoContainer, dotgraph: bool = False) -> None: ...

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

    @overload
    def abort(self) -> None: ...

    @overload
    def abort(self, calling_node: str) -> None:
        """
        Abort the run and record `calling_node` as the aborting node in task_infos.
        """

    def isAborted(self) -> bool: ...

    def isSuccessful(self) -> bool: ...

    @property
    def task_infos(self) -> TaskComposerNodeInfoContainer: ...

    def __eq__(self, arg: TaskComposerContext, /) -> bool: ...

    def __ne__(self, arg: TaskComposerContext, /) -> bool: ...

    __hash__: None = None

class FutureStatus(enum.Enum):
    ready = 0

    timeout = 1

    deferred = 2

class TaskComposerFuture:
    @property
    def context(self) -> TaskComposerContext: ...

    @context.setter
    def context(self, arg: TaskComposerContext, /) -> None: ...

    def clear(self) -> None: ...

    def valid(self) -> bool: ...

    def ready(self) -> bool: ...

    def wait(self) -> None: ...

    def waitFor(self, duration: datetime.timedelta | float) -> FutureStatus: ...

    def copy(self) -> TaskComposerFuture: ...

    def __copy__(self) -> TaskComposerFuture: ...

class TaskComposerExecutor:
    def getName(self) -> str: ...

    @overload
    def run(self, node: TaskComposerNode, context: TaskComposerContext) -> TaskComposerFuture: ...

    @overload
    def run(self, node: TaskComposerNode, data_storage: TaskComposerDataStorage, dotgraph: bool = False, name: str = 'run') -> TaskComposerFuture: ...

    def getWorkerCount(self) -> int: ...

    def getTaskCount(self) -> int: ...

class TaskComposerNode:
    def run(self, context: TaskComposerContext, executor: TaskComposerExecutor | None = None) -> int: ...

    def setName(self, name: str) -> None: ...

    def getName(self) -> str: ...

    def setNamespace(self, ns: str) -> None: ...

    def getNamespace(self) -> str: ...

    def getType(self) -> TaskComposerNodeType: ...

    def getUUID(self) -> str: ...

    def getUUIDString(self) -> str: ...

    def getParentUUID(self) -> str: ...

    def getParentUUIDString(self) -> str: ...

    def getOutboundEdges(self) -> list[str]: ...

    def getInboundEdges(self) -> list[str]: ...

    def isConditional(self) -> bool: ...

    def setConditional(self, enable: bool) -> None: ...

    def validatePorts(self) -> None:
        """
        Raise RuntimeError if a required port has no key or a key names no port.
        """

    def getInputKeys(self) -> TaskComposerKeys: ...

    def getOutputKeys(self) -> TaskComposerKeys: ...

    def setInputKeys(self, input_keys: TaskComposerKeys) -> None: ...

    def setOutputKeys(self, output_keys: TaskComposerKeys) -> None: ...

    def getPorts(self) -> TaskComposerNodePorts: ...

    def getDataStorage(self, context: TaskComposerContext) -> TaskComposerDataStorage:
        """The context's data storage for a root node, else its parent graph's."""

    def getDotgraph(self, results_map: Mapping[str, TaskComposerNodeInfo] = {}) -> str:
        """
        The DOT graph of this node, annotated with the results in `results_map`.
        """

    def saveDotgraph(self, filepath: str | os.PathLike, results_map: Mapping[str, TaskComposerNodeInfo] = {}) -> None:
        """
        Write getDotgraph(results_map) to `filepath`. Raises OSError if it cannot be written.
        """

class TaskComposerTask(TaskComposerNode):
    def setTriggerAbort(self, enable: bool) -> None: ...

class TaskComposerGraph(TaskComposerNode):
    def __init__(self, name: str = 'TaskComposerGraph') -> None: ...

    def getRootNode(self) -> str: ...

    def addNode(self, task_node: TaskComposerNode) -> str:
        """Add a node to the graph; returns its uuid."""

    def addEdges(self, source: str, destinations: Sequence[str]) -> None: ...

    def getNodes(self) -> dict[str, TaskComposerNode]:
        """The child nodes, uuid -> node."""

    def getNodeByName(self, name: str) -> TaskComposerNode: ...

    def setTerminals(self, terminals: Sequence[str]) -> None: ...

    def getTerminals(self) -> list[str]: ...

    def setTerminalTriggerAbort(self, terminal: str) -> None: ...

    def setTerminalTriggerAbortByIndex(self, terminal_index: int) -> None: ...

    def getAbortTerminal(self) -> str: ...

    def getAbortTerminalIndex(self) -> int: ...

    def setOverrideInputKeys(self, override_input_keys: TaskComposerKeys) -> None: ...

    def setOverrideOutputKeys(self, override_output_keys: TaskComposerKeys) -> None: ...

    def getOverrideInputKeys(self) -> TaskComposerKeys: ...

    def getOverrideOutputKeys(self) -> TaskComposerKeys: ...

    def isValid(self) -> tuple[bool, str]:
        """(True, message) if valid, else (False, the reason)."""

class TaskComposerPipeline(TaskComposerGraph):
    @overload
    def __init__(self, name: str = 'TaskComposerPipeline') -> None: ...

    @overload
    def __init__(self, name: str, conditional: bool) -> None: ...

class TaskflowTaskComposerExecutor(TaskComposerExecutor):
    @overload
    def __init__(self, name: str = 'TaskflowExecutor', num_threads: int = ...) -> None: ...

    @overload
    def __init__(self, num_threads: int) -> None: ...

    def getName(self) -> str: ...

    @overload
    def run(self, node: TaskComposerNode, context: TaskComposerContext) -> TaskComposerFuture: ...

    @overload
    def run(self, node: TaskComposerNode, data_storage: TaskComposerDataStorage, dotgraph: bool = False, name: str = 'run') -> TaskComposerFuture: ...

    def getWorkerCount(self) -> int: ...

    def getTaskCount(self) -> int: ...

class TaskComposerPluginError(RuntimeError):
    """
    createTaskComposerNode / createTaskComposerExecutor could not create the named plugin: the name is not configured, or its plugin failed to load or build.
    """

class TaskComposerConfigError(ValueError):
    """
    A `str` config passed to TaskComposerPluginFactory or TaskComposerServer is not a loadable task composer config (a `str` is YAML text; pass a pathlib.Path to load a file).
    """

class TaskComposerPluginFactory:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, config: tesseract_robotics.tesseract_common._tesseract_common.TaskComposerPluginInfo) -> None: ...

    @overload
    def __init__(self, config_path: os.PathLike, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    @overload
    def __init__(self, config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    @overload
    def loadConfig(self, config: tesseract_robotics.tesseract_common._tesseract_common.TaskComposerPluginInfo) -> None: ...

    @overload
    def loadConfig(self, config_path: os.PathLike, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    @overload
    def loadConfig(self, config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    def addSearchPath(self, path: str) -> None: ...

    def getSearchPaths(self) -> list[str]: ...

    def clearSearchPaths(self) -> None: ...

    def addSearchLibrary(self, library_name: str) -> None: ...

    def getSearchLibraries(self) -> list[str]: ...

    def clearSearchLibraries(self) -> None: ...

    def addTaskComposerExecutorPlugin(self, name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> None: ...

    def getTaskComposerExecutorPlugins(self) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.PluginInfo]:
        """Configured executor plugins, name -> PluginInfo (a copy)."""

    def removeTaskComposerExecutorPlugin(self, name: str) -> None: ...

    def setDefaultTaskComposerExecutorPlugin(self, name: str) -> None: ...

    def hasTaskComposerExecutorPlugins(self) -> bool: ...

    def getDefaultTaskComposerExecutorPlugin(self) -> str: ...

    def addTaskComposerNodePlugin(self, name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> None: ...

    def getTaskComposerNodePlugins(self) -> dict[str, tesseract_robotics.tesseract_common._tesseract_common.PluginInfo]:
        """
        Configured task composer node plugins, name -> PluginInfo (a copy). The keys are the names createTaskComposerNode accepts.
        """

    def removeTaskComposerNodePlugin(self, name: str) -> None: ...

    def setDefaultTaskComposerNodePlugin(self, name: str) -> None: ...

    def hasTaskComposerNodePlugins(self) -> bool: ...

    def getDefaultTaskComposerNodePlugin(self) -> str: ...

    @overload
    def createTaskComposerExecutor(self, name: str) -> TaskComposerExecutor:
        """
        Create a task composer executor by name. Raises TaskComposerPluginError if it cannot be created.
        """

    @overload
    def createTaskComposerExecutor(self, name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> TaskComposerExecutor:
        """
        Create an executor named `name` from an explicit PluginInfo. Raises TaskComposerPluginError if it cannot be created.
        """

    @overload
    def createTaskComposerNode(self, name: str) -> TaskComposerNode:
        """
        Create a task composer node by name. Raises TaskComposerPluginError if it cannot be created.
        """

    @overload
    def createTaskComposerNode(self, name: str, plugin_info: tesseract_robotics.tesseract_common._tesseract_common.PluginInfo) -> TaskComposerNode:
        """
        Create a node named `name` from an explicit PluginInfo. Raises TaskComposerPluginError if it cannot be created.
        """

    def saveConfig(self, file_path: str | os.PathLike) -> None: ...

    def getConfig(self) -> str:
        """The factory configuration as a YAML document string."""

    def getAvailableTaskComposerNodePlugins(self) -> list[str]:
        """
        Node factory class names exported by the plugin search libraries (valid PluginInfo.class_name values), not the configured node names; see getTaskComposerNodePlugins. Loads the plugin libraries.
        """

    def getAvailableTaskComposerExecutorPlugins(self) -> list[str]:
        """
        Executor factory class names exported by the plugin search libraries (valid PluginInfo.class_name values). Loads the plugin libraries.
        """

class TaskComposerServer:
    def __init__(self) -> None: ...

    @overload
    def loadConfig(self, config_path: os.PathLike, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    @overload
    def loadConfig(self, config: str, locator: tesseract_robotics.tesseract_common._tesseract_common.ResourceLocator) -> None: ...

    def addExecutor(self, executor: TaskComposerExecutor) -> None: ...

    def getExecutor(self, name: str) -> TaskComposerExecutor: ...

    def hasExecutor(self, name: str) -> bool: ...

    def getAvailableExecutors(self) -> list[str]: ...

    def addTask(self, task: TaskComposerNode) -> None:
        """
        Add `task` under its name, replacing a task of that name. Raises TaskComposerServerBusyError if a run started by that name is still running.
        """

    def getTask(self, name: str) -> TaskComposerNode: ...

    def hasTask(self, name: str) -> bool: ...

    def getAvailableTasks(self) -> list[str]: ...

    @overload
    def run(self, task_name: str, data_storage: TaskComposerDataStorage, dotgraph: bool, executor_name: str) -> TaskComposerFuture: ...

    @overload
    def run(self, node: TaskComposerNode, data_storage: TaskComposerDataStorage, dotgraph: bool, executor_name: str) -> TaskComposerFuture: ...

    def getWorkerCount(self, name: str) -> int: ...

    def getTaskCount(self, name: str) -> int: ...

class TaskComposerLog:
    def __init__(self, desc: str = '') -> None: ...

    @property
    def description(self) -> str: ...

    @description.setter
    def description(self, arg: str, /) -> None: ...

    @property
    def initial_data(self) -> TaskComposerDataStorage: ...

    @initial_data.setter
    def initial_data(self, arg: TaskComposerDataStorage, /) -> None: ...

    @property
    def context(self) -> TaskComposerContext: ...

    @context.setter
    def context(self, value: TaskComposerContext | None) -> None: ...

    @property
    def dotgraph(self) -> str: ...

    @dotgraph.setter
    def dotgraph(self, arg: str, /) -> None: ...

    def __eq__(self, arg: TaskComposerLog, /) -> bool: ...

    def __ne__(self, arg: TaskComposerLog, /) -> bool: ...

    __hash__: None = None

class AnyPoly:
    def __init__(self) -> None: ...

    def isNull(self) -> bool: ...

    def getTypeName(self) -> str:
        """Demangled C++ type name of the stored value, or `null`."""

    def __eq__(self, arg: AnyPoly, /) -> bool: ...

    def __ne__(self, arg: AnyPoly, /) -> bool: ...

    __hash__: None = None

def AnyPoly_wrap_CompositeInstruction(instruction: tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction) -> AnyPoly:
    """Wrap a CompositeInstruction into an AnyPoly"""

def AnyPoly_wrap_ProfileDictionary(profiles: tesseract_robotics.tesseract_command_language._tesseract_command_language.ProfileDictionary) -> AnyPoly:
    """Wrap a ProfileDictionary shared_ptr into an AnyPoly"""

def AnyPoly_wrap_EnvironmentConst(environment: tesseract_robotics.tesseract_environment._tesseract_environment.Environment) -> AnyPoly:
    """Wrap a const Environment shared_ptr into an AnyPoly"""

def AnyPoly_wrap_TaskComposerDataStorage(data_storage: TaskComposerDataStorage) -> AnyPoly:
    """Wrap a TaskComposerDataStorage shared_ptr into an AnyPoly"""

def AnyPoly_as_CompositeInstruction(any_poly: AnyPoly) -> tesseract_robotics.tesseract_command_language._tesseract_command_language.CompositeInstruction:
    """Extract a CompositeInstruction from an AnyPoly"""

def AnyPoly_as_TaskComposerDataStorage(any_poly: AnyPoly) -> TaskComposerDataStorage:
    """Extract a TaskComposerDataStorage from an AnyPoly"""

def AnyPoly_as_ContactResultMapVector(any_poly: AnyPoly) -> list[tesseract_robotics.tesseract_collision._tesseract_collision.ContactResultMap]:
    """
    Extract a std::vector<ContactResultMap> from an AnyPoly. DiscreteContactCheckTask saves its per-step contact results under the 'contact_results' key on its node info's data_storage in this form.
    """
