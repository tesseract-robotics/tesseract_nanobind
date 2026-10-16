/**
 * @file tesseract_task_composer_bindings.cpp
 * @brief nanobind bindings for tesseract_task_composer
 */

#include "tesseract_nb.h"
#include <future>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/optional.h>

#include <boost/core/demangle.hpp>
#include <yaml-cpp/yaml.h>  // YAML::Emitter for getConfig; YAML::Exception from the text overloads

#include <fstream>
#include <sstream>
#include <stdexcept>  // std::runtime_error (TaskComposerPluginError)

// tesseract_task_composer core
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_graph.h>
#include <tesseract/task_composer/task_composer_keys.h>
#include <tesseract/task_composer/task_composer_log.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/task_composer_node_ports.h>
#include <tesseract/task_composer/task_composer_node_types.h>
#include <tesseract/task_composer/task_composer_pipeline.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/task_composer/task_composer_task.h>

// tesseract_task_composer taskflow
#include <tesseract/task_composer/taskflow/taskflow_task_composer_executor.h>

// tesseract_common
#include <tesseract/common/any_poly.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/common/resource_locator.h>
#include <filesystem>

// tesseract_environment for Environment wrapper
#include <tesseract/environment/environment.h>

// tesseract_command_language for CompositeInstruction
#include <tesseract/command_language/composite_instruction.h>
// Note: ProfileDictionary moved to tesseract_common in 0.33
#include <tesseract/common/profile_dictionary.h>

// tesseract_collision for ContactResultMap (DiscreteContactCheckTask saves
// std::vector<ContactResultMap> on its node info's data_storage)
#include <tesseract/collision/types.h>

// console_bridge for log output visible alongside the rest of tesseract's logging
#include <console_bridge/console.h>

// Plugin-dylib pinning: on macOS the TaskComposerPluginFactory dlclose's every
// plugin dylib it loaded when the factory is GC'd. Re-dlopen'ing those dylibs
// corrupts static state inside OMPL / OSQP / TrajOpt and crashes the next
// compute call (GH #48). The fix is to observe which dylibs get loaded during
// createTaskComposer{Node,Executor} and immediately re-open them with
// RTLD_NODELETE, pinning them for the process lifetime so dlclose becomes a
// no-op. Linux is unaffected (glibc / underlying deps tolerate the cycle) and
// Windows support will land when the Windows wheel does (#40).
#ifdef __APPLE__
#include <mach-o/dyld.h>
#include <dlfcn.h>
#include <cstdio>
#include <mutex>
#include <string>
#include <unordered_set>
#endif

namespace tp = tesseract::task_composer;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace tc = tesseract::common;

#ifdef __APPLE__
namespace {

std::unordered_set<std::string> snapshot_loaded_dylibs() {
    std::unordered_set<std::string> libs;
    const uint32_t n = _dyld_image_count();
    libs.reserve(n);
    for (uint32_t i = 0; i < n; ++i) {
        if (const char* name = _dyld_get_image_name(i)) {
            libs.emplace(name);
        }
    }
    return libs;
}

// Pin any dylib loaded since `before`. Only logs the first time a given path
// is pinned, so repeated createTaskComposer* calls stay quiet after warmup.
void pin_newly_loaded_dylibs(const std::unordered_set<std::string>& before) {
    static std::mutex mu;
    static std::unordered_set<std::string> already_pinned;

    auto after = snapshot_loaded_dylibs();
    std::lock_guard<std::mutex> guard(mu);
    for (const auto& path : after) {
        if (before.count(path) || already_pinned.count(path)) {
            continue;
        }
        // RTLD_NODELETE: any later dlclose leaves the library mapped.
        // Handle is intentionally leaked for the process lifetime.
        void* handle = ::dlopen(path.c_str(), RTLD_LAZY | RTLD_NODELETE);
        if (handle == nullptr) {
            CONSOLE_BRIDGE_logWarn("tesseract_nanobind: failed to pin plugin dylib '%s': %s",
                                   path.c_str(), dlerror());
            continue;
        }
        already_pinned.insert(path);
        const auto slash = path.find_last_of('/');
        const char* basename = slash == std::string::npos ? path.c_str() : path.c_str() + slash + 1;
        // stderr rather than console_bridge::logInform because tesseract's
        // default log threshold suppresses anything below ERROR.
        std::fprintf(stderr, "[tesseract_nanobind] pinned plugin dylib %s (GH #48)\n", basename);
    }
}

}  // anonymous namespace
#endif  // __APPLE__

namespace {

// Run `load`, which may dlopen plugin libraries, and on macOS pin every dylib it mapped (GH #48),
// also when it throws.
template <typename Load>
decltype(auto) with_pinned_plugins(Load&& load) {
#ifdef __APPLE__
    struct Pin {
        std::unordered_set<std::string> before = snapshot_loaded_dylibs();
        ~Pin() { pin_newly_loaded_dylibs(before); }
    } pin;
#endif
    return load();
}

// createTaskComposer{Node,Executor} never throw: an unknown name, a plugin symbol that fails to
// load and a plugin factory that throws all log a console_bridge warning and return nullptr
// (task_composer_plugin_factory.cpp, 0.35.0). The binding raises this instead of returning None.
struct TaskComposerPluginError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Throw TaskComposerPluginError for a null result, naming `name` and the configured plugins.
template <typename T>
std::unique_ptr<T> require_created(std::unique_ptr<T> created, const char* caller, const std::string& name,
                                   const tp::TaskComposerPluginFactory::PluginInfoMap& configured) {
    if (created) {
        return created;
    }
    std::string names;
    for (const auto& [configured_name, info] : configured) {
        names += (names.empty() ? "" : ", ") + configured_name;
    }
    if (configured.count(name) == 0) {
        throw TaskComposerPluginError(std::string(caller) + ": '" + name +
                                      "' not found among the configured plugins: [" + names + "]");
    }
    throw TaskComposerPluginError(std::string(caller) + ": plugin '" + name +
                                  "' is configured but failed to load or build; the console_bridge "
                                  "warning above gives the cause");
}

// The same for the overloads that take an explicit PluginInfo: the name need not be configured.
template <typename T>
std::unique_ptr<T> require_created(std::unique_ptr<T> created, const char* caller, const std::string& name,
                                   const tc::PluginInfo& info) {
    if (created) {
        return created;
    }
    throw TaskComposerPluginError(std::string(caller) + ": '" + name + "' (class '" + info.class_name +
                                  "') failed to load or build; the console_bridge warning above gives "
                                  "the cause");
}

// Upstream remove*/setDefault* throw std::runtime_error for an unknown name; a name lookup that
// misses is a KeyError here, as on ContactManagersPluginFactory (#170).
void require_plugin(const tp::TaskComposerPluginFactory::PluginInfoMap& plugins, const char* kind,
                    const std::string& name) {
    if (plugins.find(name) == plugins.end())
        throw nb::key_error(("no task composer " + std::string(kind) + " plugin '" + name + "'").c_str());
}

// The `str` overloads of the plugin factory and server take YAML text, the `os.PathLike` ones a
// file (#337). A path passed as `str` parses as a YAML scalar and fails inside upstream's loader;
// name the text, so that mistake reads as one.
template <typename Load>
void load_yaml_text(const char* caller, const std::string& config, Load&& load) {
    try {
        load();
    } catch (const YAML::Exception& e) {
        throw nb::value_error((std::string(caller) + ": `config` is YAML text (pass a pathlib.Path to "
                               "load a file) and did not load as a task composer config: " +
                               e.what() + "; config: '" + config.substr(0, 200) + "'")
                                  .c_str());
    }
}

}  // anonymous namespace

NB_MODULE(_tesseract_task_composer, m) {
    m.doc() = "tesseract_task_composer Python bindings";

    // Import the modules that bind the types this one takes and returns, so nanobind names
    // them in signatures (#312): ResourceLocator and AnyPoly's ProfileDictionary come from
    // common and command language, Environment and ContactResultMap from their own modules.
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");

    // ========== TaskComposerKeys ==========
    // A port maps to one key or a list of keys (EntryType = variant<string, vector<string>>).
    // nanobind never converts a str to vector<string>, so the two `add` overloads dispatch on
    // the argument's type.
    auto keys = nb::class_<tp::TaskComposerKeys>(m, "TaskComposerKeys")
        .def(nb::init<>())
        .def("add", nb::overload_cast<const std::string&, std::string>(&tp::TaskComposerKeys::add), "port"_a, "key"_a)
        .def("add", nb::overload_cast<const std::string&, std::vector<std::string>>(&tp::TaskComposerKeys::add),
             "port"_a, "keys"_a)
        .def("remove", &tp::TaskComposerKeys::remove, "port"_a)
        .def("rename", &tp::TaskComposerKeys::rename, "keys"_a)
        .def("get", [](const tp::TaskComposerKeys& self, const std::string& port) -> tp::TaskComposerKeys::EntryType {
            return self.data().at(port);
        }, "port"_a, "The key, or list of keys, of `port`. Raises IndexError for an unknown port.")
        .def("has", &tp::TaskComposerKeys::has, "port"_a)
        .def("empty", &tp::TaskComposerKeys::empty)
        .def("size", &tp::TaskComposerKeys::size)
        .def("data", &tp::TaskComposerKeys::data)
        .def("__str__", [](const tp::TaskComposerKeys& self) {
            std::ostringstream os;
            os << self;
            return os.str();
        });
    bind_value_equality(keys);

    // ========== TaskComposerDataStorage ==========
    auto data_storage = nb::class_<tp::TaskComposerDataStorage>(m, "TaskComposerDataStorage")
        .def(nb::init<std::string>(), "name"_a = "")
        .def("getName", &tp::TaskComposerDataStorage::getName)
        .def("setName", &tp::TaskComposerDataStorage::setName, "name"_a)
        .def("hasKey", &tp::TaskComposerDataStorage::hasKey, "key"_a)
        .def("setData", &tp::TaskComposerDataStorage::setData, "key"_a, "data"_a)
        .def("getData", nb::overload_cast<const std::string&>(&tp::TaskComposerDataStorage::getData, nb::const_), "key"_a)
        .def("getData", nb::overload_cast<>(&tp::TaskComposerDataStorage::getData, nb::const_),
             "Every entry, key -> AnyPoly (a copy).")
        .def("removeData", &tp::TaskComposerDataStorage::removeData, "key"_a)
        .def("remapData", &tp::TaskComposerDataStorage::remapData, "remapping"_a, "copy"_a = false,
             "Rename (or with copy=True, duplicate) entries old key -> new key. Returns False, with a "
             "console_bridge error, at the first missing key.")
        .def("copyAsInputData", &tp::TaskComposerDataStorage::copyAsInputData, "data_storage"_a, "keys"_a,
             "override_keys"_a,
             "For each port in `keys`: self[key] = data_storage[override key, else key].")
        .def("copyAsOutputData", &tp::TaskComposerDataStorage::copyAsOutputData, "data_storage"_a, "keys"_a,
             "override_keys"_a,
             "For each port in `keys`: self[override key, else key] = data_storage[key].");
    bind_value_equality(data_storage);

    // ========== TaskComposerNodeType / TaskComposerNodePorts ==========
    nb::enum_<tp::TaskComposerNodeType>(m, "TaskComposerNodeType")
        .value("NODE", tp::TaskComposerNodeType::NODE)
        .value("TASK", tp::TaskComposerNodeType::TASK)
        .value("PIPELINE", tp::TaskComposerNodeType::PIPELINE)
        .value("GRAPH", tp::TaskComposerNodeType::GRAPH);

    auto node_ports = nb::class_<tp::TaskComposerNodePorts>(m, "TaskComposerNodePorts");
    nb::enum_<tp::TaskComposerNodePorts::Type>(node_ports, "Type")
        .value("SINGLE", tp::TaskComposerNodePorts::SINGLE)
        .value("MULTIPLE", tp::TaskComposerNodePorts::MULTIPLE);
    node_ports.def(nb::init<>())
        .def_rw("input_required", &tp::TaskComposerNodePorts::input_required)
        .def_rw("input_optional", &tp::TaskComposerNodePorts::input_optional)
        .def_rw("output_required", &tp::TaskComposerNodePorts::output_required)
        .def_rw("output_optional", &tp::TaskComposerNodePorts::output_optional)
        .def("__str__", &tp::TaskComposerNodePorts::toString);
    bind_value_equality(node_ports);

    // ========== TaskComposerNodeInfo ==========
    // A value type with public fields: all read-write (prune callbacks mutate infos, #320).
    auto node_info = nb::class_<tp::TaskComposerNodeInfo>(m, "TaskComposerNodeInfo")
        .def(nb::init<const tp::TaskComposerNode&>(), "node"_a)
        .def_rw("uuid", &tp::TaskComposerNodeInfo::uuid)
        .def_rw("root_uuid", &tp::TaskComposerNodeInfo::root_uuid)
        .def_rw("parent_uuid", &tp::TaskComposerNodeInfo::parent_uuid)
        .def_rw("inbound_edges", &tp::TaskComposerNodeInfo::inbound_edges)
        .def_rw("outbound_edges", &tp::TaskComposerNodeInfo::outbound_edges)
        .def_rw("terminals", &tp::TaskComposerNodeInfo::terminals)
        .def_rw("name", &tp::TaskComposerNodeInfo::name)
        .def_rw("ns", &tp::TaskComposerNodeInfo::ns)
        .def_rw("type", &tp::TaskComposerNodeInfo::type)
        .def_rw("type_hash_code", &tp::TaskComposerNodeInfo::type_hash_code)
        .def_rw("conditional", &tp::TaskComposerNodeInfo::conditional)
        .def_rw("input_keys", &tp::TaskComposerNodeInfo::input_keys)
        .def_rw("output_keys", &tp::TaskComposerNodeInfo::output_keys)
        .def_rw("triggers_abort", &tp::TaskComposerNodeInfo::triggers_abort)
        .def_rw("return_value", &tp::TaskComposerNodeInfo::return_value)
        .def_rw("status_code", &tp::TaskComposerNodeInfo::status_code)
        .def_rw("status_message", &tp::TaskComposerNodeInfo::status_message)
        .def_rw("start_time", &tp::TaskComposerNodeInfo::start_time)
        .def_rw("elapsed_time", &tp::TaskComposerNodeInfo::elapsed_time)
        .def_rw("color", &tp::TaskComposerNodeInfo::color)
        .def_rw("dotgraph", &tp::TaskComposerNodeInfo::dotgraph)
        .def_rw("data_storage", &tp::TaskComposerNodeInfo::data_storage)
        .def_rw("aborted", &tp::TaskComposerNodeInfo::aborted);
    bind_value_equality(node_info);

    // ========== TaskComposerNodeInfoContainer ==========
    auto node_info_container = nb::class_<tp::TaskComposerNodeInfoContainer>(m, "TaskComposerNodeInfoContainer")
        .def(nb::init<>())
        .def("addInfo", &tp::TaskComposerNodeInfoContainer::addInfo, "info"_a)
        .def("getInfo", &tp::TaskComposerNodeInfoContainer::getInfo, "key"_a,
             "A copy of the info of node `key`, or None.")
        .def("getInfoMap", &tp::TaskComposerNodeInfoContainer::getInfoMap, "Every info, node uuid -> info (a copy).")
        .def("setRootNode", &tp::TaskComposerNodeInfoContainer::setRootNode, "node_uuid"_a)
        .def("getRootNode", &tp::TaskComposerNodeInfoContainer::getRootNode)
        .def("setAborted", &tp::TaskComposerNodeInfoContainer::setAborted, "node_uuid"_a)
        .def("getAbortingNode", &tp::TaskComposerNodeInfoContainer::getAbortingNode,
             "The uuid of the node that aborted the run; the nil uuid if none did.")
        .def("find", &tp::TaskComposerNodeInfoContainer::find, "search_fn"_a,
             "Copies of the infos for which search_fn(info) is true.")
        .def("insertInfoMap", &tp::TaskComposerNodeInfoContainer::insertInfoMap, "container"_a)
        // Upstream takes the container by rvalue reference: take a copy and move it in.
        .def("mergeInfoMap", [](tp::TaskComposerNodeInfoContainer& self, tp::TaskComposerNodeInfoContainer container) {
            self.mergeInfoMap(std::move(container));
        }, "container"_a)
        .def("clear", &tp::TaskComposerNodeInfoContainer::clear)
        // Upstream's callback takes TaskComposerNodeInfo&, which nanobind casts to Python as a
        // copy, so edits would be lost. Pass a pointer instead: the Python object refers to the
        // stored info, valid only during the call (the container holds its lock meanwhile).
        .def("prune", [](tp::TaskComposerNodeInfoContainer& self,
                         const std::function<void(tp::TaskComposerNodeInfo*)>& prune_fn) {
            self.prune([&](tp::TaskComposerNodeInfo& info) { prune_fn(&info); });
        }, "prune_fn"_a,
           "Call prune_fn(info) on every stored info, which it may modify in place. Do not keep `info` "
           "past the call.");
    // Upstream operator!= holds both shared locks and then calls operator==, which takes them
    // again: a recursive shared lock is undefined behaviour and deadlocks behind a waiting
    // writer. Negate operator== instead, and short-cut self-comparison for the same reason.
    node_info_container
        .def("__eq__", [](const tp::TaskComposerNodeInfoContainer& self, const tp::TaskComposerNodeInfoContainer& other) {
            return &self == &other || self == other;
        }, nb::is_operator())
        .def("__ne__", [](const tp::TaskComposerNodeInfoContainer& self, const tp::TaskComposerNodeInfoContainer& other) {
            return !(&self == &other || self == other);
        }, nb::is_operator());
    node_info_container.attr("__hash__") = nb::none();

    // ========== TaskComposerContext ==========
    // Copy and move are deleted and the class derives enable_shared_from_this: nanobind's
    // shared_ptr caster shares ownership with a Python-built context, so executor.run and
    // graph runs (which call shared_from_this) accept it.
    auto context = nb::class_<tp::TaskComposerContext>(m, "TaskComposerContext")
        .def(nb::init<std::string, bool>(), "name"_a, "dotgraph"_a = false)
        .def(nb::init<std::string, std::shared_ptr<tp::TaskComposerDataStorage>, bool>(), "name"_a,
             "data_storage"_a, "dotgraph"_a = false)
        .def(nb::init<std::string, std::shared_ptr<tp::TaskComposerDataStorage>,
                      std::shared_ptr<tp::TaskComposerNodeInfoContainer>, bool>(),
             "name"_a, "data_storage"_a, "task_infos"_a, "dotgraph"_a = false)
        .def_prop_rw("name",
            [](const tp::TaskComposerContext& self) { return self.name; },
            [](tp::TaskComposerContext& self, const std::string& v) { self.name = v; })
        .def_prop_rw("dotgraph",
            [](const tp::TaskComposerContext& self) { return self.dotgraph; },
            [](tp::TaskComposerContext& self, bool v) { self.dotgraph = v; })
        .def_prop_rw("data_storage",
            [](const tp::TaskComposerContext& self) { return self.data_storage; },
            [](tp::TaskComposerContext& self, std::shared_ptr<tp::TaskComposerDataStorage> v) { self.data_storage = v; })
        .def("abort", [](tp::TaskComposerContext& self) { self.abort(); })
        .def("abort", &tp::TaskComposerContext::abort, "calling_node"_a,
             "Abort the run and record `calling_node` as the aborting node in task_infos.")
        .def("isAborted", &tp::TaskComposerContext::isAborted)
        .def("isSuccessful", &tp::TaskComposerContext::isSuccessful)
        .def_prop_ro("task_infos",
            [](const tp::TaskComposerContext& self) { return self.task_infos; });
    bind_value_equality(context);

    // ========== TaskComposerFuture ==========
    nb::enum_<std::future_status>(m, "FutureStatus")
        .value("ready", std::future_status::ready)
        .value("timeout", std::future_status::timeout)
        .value("deferred", std::future_status::deferred);

    nb::class_<tp::TaskComposerFuture>(m, "TaskComposerFuture")
        .def_prop_rw("context",
            [](const tp::TaskComposerFuture& self) { return self.context; },
            [](tp::TaskComposerFuture& self, std::shared_ptr<tp::TaskComposerContext> v) { self.context = v; })
        .def("clear", &tp::TaskComposerFuture::clear)
        .def("valid", &tp::TaskComposerFuture::valid)
        .def("ready", &tp::TaskComposerFuture::ready)
        .def("wait", &tp::TaskComposerFuture::wait, nb::call_guard<nb::gil_scoped_release>())
        .def("waitFor", &tp::TaskComposerFuture::waitFor, "duration"_a, nb::call_guard<nb::gil_scoped_release>())
        // A second future on the same shared state and context; copy.copy() calls it too.
        .def("copy", &tp::TaskComposerFuture::copy)
        .def("__copy__", &tp::TaskComposerFuture::copy);

    // ========== TaskComposerExecutor (abstract base) ==========
    // Declared before TaskComposerNode, whose run() takes one.
    auto executor = nb::class_<tp::TaskComposerExecutor>(m, "TaskComposerExecutor");

    // ========== TaskComposerNode (abstract base) ==========
    nb::class_<tp::TaskComposerNode>(m, "TaskComposerNode")
        // Runs this node synchronously in the calling thread. Graphs and pipelines need an executor
        // for their children; a node without one records the error as its info. The context is
        // taken as a shared_ptr because graph runs call shared_from_this on it.
        .def("run", [](const tp::TaskComposerNode& self, std::shared_ptr<tp::TaskComposerContext> context,
                       tp::TaskComposerExecutor* executor) {
            nb::gil_scoped_release release;
            if (executor == nullptr)
                return self.run(*context);
            return self.run(*context, *executor);
        }, "context"_a, nb::arg("executor").none() = nb::none())
        .def("setName", &tp::TaskComposerNode::setName, "name"_a)
        .def("getName", &tp::TaskComposerNode::getName)
        .def("setNamespace", &tp::TaskComposerNode::setNamespace, "ns"_a)
        .def("getNamespace", &tp::TaskComposerNode::getNamespace)
        .def("getType", &tp::TaskComposerNode::getType)
        .def("getUUID", &tp::TaskComposerNode::getUUID)
        .def("getUUIDString", &tp::TaskComposerNode::getUUIDString)
        .def("getParentUUID", &tp::TaskComposerNode::getParentUUID)
        .def("getParentUUIDString", &tp::TaskComposerNode::getParentUUIDString)
        .def("getOutboundEdges", &tp::TaskComposerNode::getOutboundEdges)
        .def("getInboundEdges", &tp::TaskComposerNode::getInboundEdges)
        .def("isConditional", &tp::TaskComposerNode::isConditional)
        .def("setConditional", &tp::TaskComposerNode::setConditional, "enable"_a)
        .def("validatePorts", &tp::TaskComposerNode::validatePorts,
             "Raise RuntimeError if a required port has no key or a key names no port.")
        .def("getInputKeys", &tp::TaskComposerNode::getInputKeys, nb::rv_policy::reference)
        .def("getOutputKeys", &tp::TaskComposerNode::getOutputKeys, nb::rv_policy::reference)
        .def("setInputKeys", &tp::TaskComposerNode::setInputKeys, "input_keys"_a)
        .def("setOutputKeys", &tp::TaskComposerNode::setOutputKeys, "output_keys"_a)
        .def("getPorts", &tp::TaskComposerNode::getPorts)
        .def("getDataStorage", &tp::TaskComposerNode::getDataStorage, "context"_a,
             "The context's data storage for a root node, else its parent graph's.")
        // results_map: node uuid -> info, e.g. context.task_infos.getInfoMap(); annotates the graph
        // with each node's result.
        .def("getDotgraph", &tp::TaskComposerNode::getDotgraph,
             "results_map"_a = tp::TaskComposerNode::ResultsMap(),
             "The DOT graph of this node, annotated with the results in `results_map`.")
        // Upstream's saveDotgraph returns true when the file cannot be opened (an ofstream does not
        // throw); this returns False then, and writes the same text as getDotgraph.
        .def("saveDotgraph", [](const tp::TaskComposerNode& self, const std::string& filepath,
                                const tp::TaskComposerNode::ResultsMap& results_map) {
            std::ofstream out(filepath);
            if (!out.is_open())
                return false;
            out << self.getDotgraph(results_map);
            return out.good();
        }, "filepath"_a, "results_map"_a = tp::TaskComposerNode::ResultsMap(),
           "Write getDotgraph(results_map) to `filepath`. Returns False if it cannot be written.");

    // ========== TaskComposerExecutor (abstract base) ==========
    // run(node, data_storage, dotgraph, name) builds the context from a data storage, as
    // upstream's TaskComposerServer::run(node, data_storage, dotgraph, executor_name) does.
    // The future runs `node` in the executor's threads, so it keeps the node alive: a node
    // dropped before wait() was a use-after-free.
    executor
        .def("getName", &tp::TaskComposerExecutor::getName)
        .def("run", [](tp::TaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerContext> context) {
            nb::gil_scoped_release release;
            return self.run(node, context);
        }, "node"_a, "context"_a, nb::keep_alive<0, 2>())
        .def("run", [](tp::TaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph,
                       const std::string& name) {
            nb::gil_scoped_release release;
            auto context = std::make_shared<tp::TaskComposerContext>(name, data_storage, dotgraph);
            return self.run(node, context);
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false, "name"_a = "run", nb::keep_alive<0, 2>())
        .def("getWorkerCount", &tp::TaskComposerExecutor::getWorkerCount)
        .def("getTaskCount", &tp::TaskComposerExecutor::getTaskCount);

    // ========== TaskComposerTask / TaskComposerGraph / TaskComposerPipeline ==========
    // Factory nodes come back as their most-derived bound class: YAML pipelines are
    // TaskComposerPipeline, the concrete task classes are unbound plugins and stay
    // TaskComposerNode. TaskComposerTask has no Python constructor (runImpl is pure virtual).
    nb::class_<tp::TaskComposerTask, tp::TaskComposerNode>(m, "TaskComposerTask")
        .def("setTriggerAbort", &tp::TaskComposerTask::setTriggerAbort, "enable"_a);

    // getNodes / getNodeByName return nodes the graph shares (shared_ptr), so no keep_alive. A
    // node added with addNode is shared with the graph, which keeps the Python object alive.
    nb::class_<tp::TaskComposerGraph, tp::TaskComposerNode>(m, "TaskComposerGraph")
        .def("__init__", [](tp::TaskComposerGraph* self, std::string name) {
            new (self) tp::TaskComposerGraph(std::move(name));
        }, "name"_a = "TaskComposerGraph")
        .def("getRootNode", &tp::TaskComposerGraph::getRootNode)
        // Upstream's hook for Python wrappers; addNode(unique_ptr) has no Python form.
        .def("addNode", &tp::TaskComposerGraph::addNodePython, "task_node"_a,
             "Add a node to the graph; returns its uuid.")
        .def("addEdges", &tp::TaskComposerGraph::addEdges, "source"_a, "destinations"_a)
        .def("getNodes", &tp::TaskComposerGraph::getNodes, "The child nodes, uuid -> node.")
        .def("getNodeByName", &tp::TaskComposerGraph::getNodeByName, "name"_a)
        .def("setTerminals", &tp::TaskComposerGraph::setTerminals, "terminals"_a)
        .def("getTerminals", &tp::TaskComposerGraph::getTerminals)
        .def("setTerminalTriggerAbort", &tp::TaskComposerGraph::setTerminalTriggerAbort, "terminal"_a)
        .def("setTerminalTriggerAbortByIndex", &tp::TaskComposerGraph::setTerminalTriggerAbortByIndex,
             "terminal_index"_a)
        .def("getAbortTerminal", &tp::TaskComposerGraph::getAbortTerminal)
        .def("getAbortTerminalIndex", &tp::TaskComposerGraph::getAbortTerminalIndex)
        .def("setOverrideInputKeys", &tp::TaskComposerGraph::setOverrideInputKeys, "override_input_keys"_a)
        .def("setOverrideOutputKeys", &tp::TaskComposerGraph::setOverrideOutputKeys, "override_output_keys"_a)
        .def("getOverrideInputKeys", &tp::TaskComposerGraph::getOverrideInputKeys, nb::rv_policy::reference_internal)
        .def("getOverrideOutputKeys", &tp::TaskComposerGraph::getOverrideOutputKeys, nb::rv_policy::reference_internal)
        .def("isValid", &tp::TaskComposerGraph::isValid, "(True, message) if valid, else (False, the reason).");

    nb::class_<tp::TaskComposerPipeline, tp::TaskComposerGraph>(m, "TaskComposerPipeline")
        .def("__init__", [](tp::TaskComposerPipeline* self, std::string name) {
            new (self) tp::TaskComposerPipeline(std::move(name));
        }, "name"_a = "TaskComposerPipeline")
        .def(nb::init<std::string, bool>(), "name"_a, "conditional"_a);

    // ========== TaskflowTaskComposerExecutor ==========
    nb::class_<tp::TaskflowTaskComposerExecutor, tp::TaskComposerExecutor>(m, "TaskflowTaskComposerExecutor")
        .def(nb::init<std::string, size_t>(), "name"_a = "TaskflowExecutor",
             // sig("..."): the default is this machine's core count; keep it out of the stub
             nb::arg("num_threads").sig("...") = std::thread::hardware_concurrency())
        .def(nb::init<size_t>(), "num_threads"_a)
        // Re-expose base class methods (public run is from TaskComposerExecutor)
        .def("getName", &tp::TaskflowTaskComposerExecutor::getName)
        .def("run", [](tp::TaskflowTaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerContext> context) {
            nb::gil_scoped_release release;
            return static_cast<tp::TaskComposerExecutor&>(self).run(node, context);
        }, "node"_a, "context"_a, nb::keep_alive<0, 2>())
        .def("run", [](tp::TaskflowTaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph,
                       const std::string& name) {
            nb::gil_scoped_release release;
            auto context = std::make_shared<tp::TaskComposerContext>(name, data_storage, dotgraph);
            return static_cast<tp::TaskComposerExecutor&>(self).run(node, context);
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false, "name"_a = "run", nb::keep_alive<0, 2>())
        .def("getWorkerCount", &tp::TaskflowTaskComposerExecutor::getWorkerCount)
        .def("getTaskCount", &tp::TaskflowTaskComposerExecutor::getTaskCount);

    // ========== TaskComposerPluginFactory ==========
    // Move-only class (deleted copy, has move). Use unique_ptr internally to handle moves.
    // The locator binds to the ResourceLocator base (registered by _tesseract_common, imported
    // at module init), so GeneralResourceLocator and Python subclasses are both accepted (gh-150).
    // The locator is only consulted during construction; the factory keeps no reference to it.
    //
    // Every call that can dlopen plugin libraries (create*, getAvailable*) runs through
    // with_pinned_plugins: on macOS those dylibs are pinned with RTLD_NODELETE immediately
    // after the call so the factory's own dlclose (on destruction) cannot unload them. See the
    // comment block at the top of the file and GH #48.
    //
    // keep_alive<0, 1> is defense-in-depth lifetime hygiene: a returned node /
    // executor dispatches through vtables that live in the factory's plugin
    // dylibs, so it logically depends on the factory even though the pinning
    // already makes that safe.
    //
    // Upstream returns nullptr on every failure; every create* raises TaskComposerPluginError (gh-203).
    nb::exception<TaskComposerPluginError>(m, "TaskComposerPluginError", PyExc_RuntimeError)
        .attr("__doc__") =
        "createTaskComposerNode / createTaskComposerExecutor could not create the named plugin: the "
        "name is not configured, or its plugin failed to load or build.";

    using Factory = tp::TaskComposerPluginFactory;
    nb::class_<Factory>(m, "TaskComposerPluginFactory")
        .def(nb::init<>())
        .def(nb::init<const tc::TaskComposerPluginInfo&>(), "config"_a)
        // StrictPath (tesseract_nb.h): a `str` is always YAML text, an os.PathLike a file (#337).
        .def("__init__", [](Factory* self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            new (self) Factory(config_path.value, locator);
        }, "config_path"_a, "locator"_a)
        .def("__init__", [](Factory* self, const std::string& config, const tc::ResourceLocator& locator) {
            load_yaml_text("TaskComposerPluginFactory", config, [&] { new (self) Factory(config, locator); });
        }, "config"_a, "locator"_a)
        .def("loadConfig", nb::overload_cast<const tc::TaskComposerPluginInfo&>(&Factory::loadConfig), "config"_a)
        .def("loadConfig", [](Factory& self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            self.loadConfig(config_path.value, locator);
        }, "config_path"_a, "locator"_a)
        .def("loadConfig", [](Factory& self, const std::string& config, const tc::ResourceLocator& locator) {
            load_yaml_text("TaskComposerPluginFactory.loadConfig", config, [&] { self.loadConfig(config, locator); });
        }, "config"_a, "locator"_a)
        .def("addSearchPath", &Factory::addSearchPath, "path"_a)
        .def("getSearchPaths", &Factory::getSearchPaths)
        .def("clearSearchPaths", &Factory::clearSearchPaths)
        .def("addSearchLibrary", &Factory::addSearchLibrary, "library_name"_a)
        .def("getSearchLibraries", &Factory::getSearchLibraries)
        .def("clearSearchLibraries", &Factory::clearSearchLibraries)
        // remove* erases the map entry, then compares the default against `name`, a separate
        // string: no erase-then-use of the erased key (task_composer_plugin_factory.cpp, 0.35.0).
        .def("addTaskComposerExecutorPlugin", &Factory::addTaskComposerExecutorPlugin, "name"_a, "plugin_info"_a)
        .def("getTaskComposerExecutorPlugins", &Factory::getTaskComposerExecutorPlugins,
             "Configured executor plugins, name -> PluginInfo (a copy).")
        .def("removeTaskComposerExecutorPlugin", [](Factory& self, const std::string& name) {
            require_plugin(self.getTaskComposerExecutorPlugins(), "executor", name);
            self.removeTaskComposerExecutorPlugin(name);
        }, "name"_a)
        .def("setDefaultTaskComposerExecutorPlugin", [](Factory& self, const std::string& name) {
            require_plugin(self.getTaskComposerExecutorPlugins(), "executor", name);
            self.setDefaultTaskComposerExecutorPlugin(name);
        }, "name"_a)
        .def("hasTaskComposerExecutorPlugins", &Factory::hasTaskComposerExecutorPlugins)
        .def("getDefaultTaskComposerExecutorPlugin", &Factory::getDefaultTaskComposerExecutorPlugin)
        .def("addTaskComposerNodePlugin", &Factory::addTaskComposerNodePlugin, "name"_a, "plugin_info"_a)
        // Returns a copy of the factory's node plugin map, after `!include` expansion
        // (upstream loadConfig(path, locator) -> tesseract::common::loadYamlFile). A value,
        // not a plugin-backed object: no keep_alive, no dylib loading.
        .def("getTaskComposerNodePlugins", &Factory::getTaskComposerNodePlugins,
             "Configured task composer node plugins, name -> PluginInfo (a copy). The keys are "
             "the names createTaskComposerNode accepts.")
        .def("removeTaskComposerNodePlugin", [](Factory& self, const std::string& name) {
            require_plugin(self.getTaskComposerNodePlugins(), "node", name);
            self.removeTaskComposerNodePlugin(name);
        }, "name"_a)
        .def("setDefaultTaskComposerNodePlugin", [](Factory& self, const std::string& name) {
            require_plugin(self.getTaskComposerNodePlugins(), "node", name);
            self.setDefaultTaskComposerNodePlugin(name);
        }, "name"_a)
        .def("hasTaskComposerNodePlugins", &Factory::hasTaskComposerNodePlugins)
        .def("getDefaultTaskComposerNodePlugin", &Factory::getDefaultTaskComposerNodePlugin)
        .def("createTaskComposerExecutor", [](const Factory& self, const std::string& name) {
            auto executor = with_pinned_plugins([&] { return self.createTaskComposerExecutor(name); });
            return require_created(std::move(executor), "createTaskComposerExecutor", name,
                                   self.getTaskComposerExecutorPlugins());
        }, "name"_a, nb::rv_policy::move, nb::keep_alive<0, 1>(),
           "Create a task composer executor by name. Raises TaskComposerPluginError if it cannot be created.")
        .def("createTaskComposerExecutor", [](const Factory& self, const std::string& name, const tc::PluginInfo& plugin_info) {
            auto executor = with_pinned_plugins([&] { return self.createTaskComposerExecutor(name, plugin_info); });
            return require_created(std::move(executor), "createTaskComposerExecutor", name, plugin_info);
        }, "name"_a, "plugin_info"_a, nb::rv_policy::move, nb::keep_alive<0, 1>(),
           "Create an executor named `name` from an explicit PluginInfo. Raises TaskComposerPluginError if "
           "it cannot be created.")
        .def("createTaskComposerNode", [](const Factory& self, const std::string& name) {
            auto node = with_pinned_plugins([&] { return self.createTaskComposerNode(name); });
            return require_created(std::move(node), "createTaskComposerNode", name,
                                   self.getTaskComposerNodePlugins());
        }, "name"_a, nb::rv_policy::move, nb::keep_alive<0, 1>(),
           "Create a task composer node by name. Raises TaskComposerPluginError if it cannot be created.")
        .def("createTaskComposerNode", [](const Factory& self, const std::string& name, const tc::PluginInfo& plugin_info) {
            auto node = with_pinned_plugins([&] { return self.createTaskComposerNode(name, plugin_info); });
            return require_created(std::move(node), "createTaskComposerNode", name, plugin_info);
        }, "name"_a, "plugin_info"_a, nb::rv_policy::move, nb::keep_alive<0, 1>(),
           "Create a node named `name` from an explicit PluginInfo. Raises TaskComposerPluginError if it "
           "cannot be created.")
        // Same YAML as upstream saveConfig, which ignores a failed ofstream; here a failed open or
        // write raises OSError (FileNotFoundError for a missing directory), as on
        // ContactManagersPluginFactory.
        .def("saveConfig", [](const Factory& self, const std::filesystem::path& file_path) {
            errno = 0;
            std::ofstream fout(file_path);
            if (fout) fout << self.getConfig();
            if (fout) fout.close();
            if (!fout) {
                PyErr_SetFromErrnoWithFilename(PyExc_OSError, file_path.string().c_str());
                throw nb::python_error();
            }
        }, "file_path"_a)
        // YAML::Node has no caster: the YAML document as str, like PluginInfo.config.
        .def("getConfig", [](const Factory& self) {
            YAML::Emitter out;
            out << self.getConfig();
            return std::string(out.c_str());
        }, "The factory configuration as a YAML document string.")
        // Upstream returns plugin_loader.getAvailablePlugins(TaskComposerNodeFactory section):
        // the factory *class* names exported by the search libraries (the values
        // PluginInfo.class_name may take), not the configured node names. It dlopens the
        // search libraries into the factory's loader cache. A later createTaskComposerNode
        // then finds them already mapped, so its own snapshot/pin sees nothing new and the
        // libraries would stay unpinned, to be dlclose'd with the factory under a live node
        // (GH #48). Pin here too.
        .def("getAvailableTaskComposerNodePlugins", [](const Factory& self) {
            return with_pinned_plugins([&] { return self.getAvailableTaskComposerNodePlugins(); });
        }, "Node factory class names exported by the plugin search libraries (valid "
           "PluginInfo.class_name values), not the configured node names; see "
           "getTaskComposerNodePlugins. Loads the plugin libraries.")
        .def("getAvailableTaskComposerExecutorPlugins", [](const Factory& self) {
            return with_pinned_plugins([&] { return self.getAvailableTaskComposerExecutorPlugins(); });
        }, "Executor factory class names exported by the plugin search libraries (valid "
           "PluginInfo.class_name values). Loads the plugin libraries.");

    // ========== TaskComposerServer ==========
    // Upstream's top-level entry point: loads executors and tasks from one config and runs a task
    // by name. loadConfig creates every configured executor and task through the server's own
    // plugin factory, so it pins the libraries (GH #48), and returned executors, tasks and futures
    // keep the server alive: their code lives in those libraries and the tasks in the server.
    using Server = tp::TaskComposerServer;
    nb::class_<Server>(m, "TaskComposerServer")
        .def(nb::init<>())
        .def("loadConfig", [](Server& self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            with_pinned_plugins([&] { self.loadConfig(config_path.value, locator); });
        }, "config_path"_a, "locator"_a)
        .def("loadConfig", [](Server& self, const std::string& config, const tc::ResourceLocator& locator) {
            load_yaml_text("TaskComposerServer.loadConfig", config,
                           [&] { with_pinned_plugins([&] { self.loadConfig(config, locator); }); });
        }, "config"_a, "locator"_a)
        .def("addExecutor", &Server::addExecutor, "executor"_a)
        .def("getExecutor", &Server::getExecutor, "name"_a, nb::keep_alive<0, 1>())
        .def("hasExecutor", &Server::hasExecutor, "name"_a)
        .def("getAvailableExecutors", &Server::getAvailableExecutors)
        // Upstream's hook for Python wrappers; addTask(unique_ptr) has no Python form.
        .def("addTask", &Server::addTaskPython, "task"_a)
        .def("getTask", &Server::getTask, "name"_a, nb::rv_policy::reference_internal)
        .def("hasTask", &Server::hasTask, "name"_a)
        .def("getAvailableTasks", &Server::getAvailableTasks)
        .def("run", [](Server& self, const std::string& task_name, std::shared_ptr<tp::TaskComposerDataStorage> data_storage,
                       bool dotgraph, const std::string& executor_name) {
            nb::gil_scoped_release release;
            return self.run(task_name, std::move(data_storage), dotgraph, executor_name);
        }, "task_name"_a, "data_storage"_a, "dotgraph"_a, "executor_name"_a, nb::keep_alive<0, 1>())
        // The future runs `node` in the executor's threads: keep the node alive with it.
        .def("run", [](Server& self, const tp::TaskComposerNode& node, std::shared_ptr<tp::TaskComposerDataStorage> data_storage,
                       bool dotgraph, const std::string& executor_name) {
            nb::gil_scoped_release release;
            return self.run(node, std::move(data_storage), dotgraph, executor_name);
        }, "node"_a, "data_storage"_a, "dotgraph"_a, "executor_name"_a, nb::keep_alive<0, 1>(), nb::keep_alive<0, 2>())
        .def("getWorkerCount", &Server::getWorkerCount, "name"_a)
        .def("getTaskCount", &Server::getTaskCount, "name"_a);

    // ========== TaskComposerLog ==========
    // A run recorded for replay. Upstream operator== dereferences `context` unconditionally, so a
    // log without one (the constructor leaves it null) would crash; compare the pointers' presence
    // first.
    auto log_equal = [](const tp::TaskComposerLog& a, const tp::TaskComposerLog& b) {
        if (a.context == nullptr || b.context == nullptr)
            return a.context == b.context && a.description == b.description && a.initial_data == b.initial_data &&
                   a.dotgraph == b.dotgraph;
        return a == b;
    };
    auto log = nb::class_<tp::TaskComposerLog>(m, "TaskComposerLog")
        .def(nb::init<std::string>(), "desc"_a = "")
        .def_rw("description", &tp::TaskComposerLog::description)
        .def_rw("initial_data", &tp::TaskComposerLog::initial_data)
        .def_rw("context", &tp::TaskComposerLog::context, nb::for_setter(nb::arg("value").none()))
        .def_rw("dotgraph", &tp::TaskComposerLog::dotgraph)
        .def("__eq__", log_equal, nb::is_operator())
        .def("__ne__", [log_equal](const tp::TaskComposerLog& a, const tp::TaskComposerLog& b) {
            return !log_equal(a, b);
        }, nb::is_operator());
    log.attr("__hash__") = nb::none();

    // ========== AnyPoly ==========
    // Bind AnyPoly class for type-erased data storage
    auto any_poly = nb::class_<tc::AnyPoly>(m, "AnyPoly")
        .def(nb::init<>())
        .def("isNull", &tc::AnyPoly::isNull)
        .def("getTypeName", [](const tc::AnyPoly& self) {
            if (self.isNull())
                return std::string("null");
            return boost::core::demangle(self.getType().name());
        }, "Demangled C++ type name of the stored value, or `null`.");
    // operator== compares the held values (AnyWrapper<T>::equals); two null AnyPolys are equal.
    bind_value_equality(any_poly);

    // ========== AnyPoly wrapper functions for common types ==========
    // These wrap specific types into AnyPoly for use with TaskComposerDataStorage.setData()

    m.def("AnyPoly_wrap_CompositeInstruction", [](const tcl::CompositeInstruction& ci) {
        return tc::AnyPoly(ci);
    }, "instruction"_a, "Wrap a CompositeInstruction into an AnyPoly");

    // Note: ProfileDictionary moved to tesseract_common in 0.33
    m.def("AnyPoly_wrap_ProfileDictionary", [](std::shared_ptr<tc::ProfileDictionary> pd) {
        return tc::AnyPoly(pd);
    }, "profiles"_a, "Wrap a ProfileDictionary shared_ptr into an AnyPoly");

    m.def("AnyPoly_wrap_EnvironmentConst", [](std::shared_ptr<const te::Environment> env) {
        return tc::AnyPoly(env);
    }, "environment"_a, "Wrap a const Environment shared_ptr into an AnyPoly");

    m.def("AnyPoly_wrap_TaskComposerDataStorage", [](std::shared_ptr<tp::TaskComposerDataStorage> ds) {
        return tc::AnyPoly(ds);
    }, "data_storage"_a, "Wrap a TaskComposerDataStorage shared_ptr into an AnyPoly");

    // ========== AnyPoly unwrap functions ==========
    m.def("AnyPoly_as_CompositeInstruction", [](const tc::AnyPoly& ap) -> tcl::CompositeInstruction {
        return ap.as<tcl::CompositeInstruction>();
    }, "any_poly"_a, "Extract a CompositeInstruction from an AnyPoly");

    m.def("AnyPoly_as_TaskComposerDataStorage", [](const tc::AnyPoly& ap) {
        return ap.as<tp::TaskComposerDataStorage::Ptr>();
    }, "any_poly"_a, "Extract a TaskComposerDataStorage from an AnyPoly");

    m.def("AnyPoly_as_ContactResultMapVector",
          [](const tc::AnyPoly& ap) -> std::vector<tesseract::collision::ContactResultMap> {
        return ap.as<std::vector<tesseract::collision::ContactResultMap>>();
    }, "any_poly"_a,
       "Extract a std::vector<ContactResultMap> from an AnyPoly. "
       "DiscreteContactCheckTask saves its per-step contact results under "
       "the 'contact_results' key on its node info's data_storage in this form.");
}
