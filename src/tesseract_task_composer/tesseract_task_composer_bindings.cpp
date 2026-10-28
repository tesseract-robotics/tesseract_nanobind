/**
 * @file tesseract_task_composer_bindings.cpp
 * @brief nanobind bindings for tesseract_task_composer
 */

#include "tesseract_nb.h"
#include "tesseract_common/log_flush_barrier.h"
#include <future>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/optional.h>

#include <boost/core/demangle.hpp>
#include <boost/uuid/uuid_io.hpp>  // boost::uuids::to_string (prune conflicts)
#include <yaml-cpp/yaml.h>  // YAML::Emitter for getConfig; YAML::Exception from the text overloads

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstring>
#include <fstream>
#include <unordered_set>
#include <thread>
#include <typeindex>
#include <sstream>
#include <stdexcept>  // std::runtime_error, std::invalid_argument (the named errors)

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

// Plugin libraries stay loaded for the process lifetime (macOS, Windows): see plugin_pinning.h.
#include "tesseract_task_composer/plugin_pinning.h"

namespace tp = tesseract::task_composer;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace tc = tesseract::common;

namespace {

// Run `load`, which may load plugin libraries, and pin every library it loaded (plugin_pinning.h),
// also when it throws.
template <typename Load>
decltype(auto) with_pinned_plugins(Load&& load) {
    struct Pin {
        tesseract_nb::LoadedLibraries before = tesseract_nb::snapshot_loaded_libraries();
        ~Pin() { tesseract_nb::pin_libraries_loaded_since(before); }
    } pin;
    return load();
}

// The C++ name of `type`, the same on every compiler: GCC and Clang mangle typeid names
// (demangled here), MSVC writes them unmangled but with an elaborated-type keyword before each
// class name ("class tesseract::command_language::CompositeInstruction"), dropped here (#322).
std::string type_name(const std::type_index& type) {
    std::string name = boost::core::demangle(type.name());
    for (const char* keyword : { "class ", "struct ", "union ", "enum " }) {
        const std::size_t length = std::strlen(keyword);
        for (std::size_t at = name.find(keyword); at != std::string::npos; at = name.find(keyword, at)) {
            const bool starts_word = at == 0 || !(std::isalnum(static_cast<unsigned char>(name[at - 1])) ||
                                                  name[at - 1] == '_');
            if (starts_word)
                name.erase(at, length);
            else
                at += length;
        }
    }
    return name;
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

// A `str` config that is not a loadable task composer config (#337).
struct TaskComposerConfigError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// The `str` overloads of the plugin factory and server take YAML text, the `os.PathLike` ones a
// file (#337). A path passed as `str` parses as a YAML scalar and fails inside upstream's loader;
// name the text, so that mistake reads as one.
template <typename Load>
void load_yaml_text(const char* caller, const std::string& config, Load&& load) {
    try {
        // Upstream loads empty, null, list or unrelated YAML as an empty config, silently.
        const YAML::Node root = YAML::Load(config);
        if (!root.IsMap() || !root[tc::TaskComposerPluginInfo::CONFIG_KEY])
            throw TaskComposerConfigError(std::string(caller) + ": `config` is YAML text (pass a pathlib.Path "
                                          "to load a file) without a top-level `" +
                                          tc::TaskComposerPluginInfo::CONFIG_KEY + "` mapping; config: '" +
                                          config.substr(0, 200) + "'");
        load();
    } catch (const YAML::Exception& e) {
        throw TaskComposerConfigError(std::string(caller) + ": `config` is YAML text (pass a pathlib.Path "
                                      "to load a file) and did not load as a task composer config: " +
                                      e.what() + "; config: '" + config.substr(0, 200) + "'");
    }
}

// Upstream: a future "must not go out of scope until the process has finished". The executor's
// threads use the node, the data and the executor after run() returns; Python owns the node and
// the executor, and a dropped future, node or executor freed them under the run. Every run is
// held here instead, independent of the Python futures: its own copy of the future (which keeps
// the taskflow and the context alive; clear() on a Python future does not touch it) and
// references to the node and to the executor or server running it. Releasing a Python future
// never waits, so it is safe on an executor worker (a callback, or gc there). Finished runs are
// freed at the next run() and when a future is waited on or found ready, the rest at interpreter
// exit (wait_for_runs_at_exit).
// Freeing a run may destroy the executor it holds (one the server replaced): its Python
// references are dropped with the GIL held, the C++ executor without it (free_finished_runs).
// A run is registered with the GIL held, before run() releases it, so no other thread can see
// the node in use without its entry; its future is null until run() returns and counts as running.
struct InFlightRun {
    std::unique_ptr<tp::TaskComposerFuture> future;
    // The node being run. A server task has no Python owner to hold (upstream exposes it only as
    // a const&), so the server refuses to replace a task while an entry points at it
    // (require_tasks_replaceable).
    const tp::TaskComposerNode* node_ptr = nullptr;
    nb::object node;
    nb::object owner;
    // A server run's executor: the server owns it, and addExecutor may replace it mid-run.
    std::shared_ptr<tp::TaskComposerExecutor> executor;
    // The executor running it, held by `owner` or `executor`. The exit drain waits for it to go
    // idle: Taskflow sets the future before it counts the run off (Executor::_tear_down_topology).
    const tp::TaskComposerExecutor* runner = nullptr;
    // run() threw. Its thread erases the entry, but the exit drain may have moved it off the list
    // first: the drain and free_finished_runs drop flagged entries instead of waiting for them.
    // No test: the bound executors cannot be made to throw from run() through the Python API
    // (Taskflow builds the graph lazily; its one throw needs a node of type NODE, which no plugin
    // or Python constructor makes).
    bool start_failed = false;

    bool finished() const { return start_failed || (future != nullptr && future->ready()); }
};

// Accessed with the GIL held only. Never freed: it outlives module teardown.
std::vector<std::shared_ptr<InFlightRun>>& in_flight_runs() {
    static auto* runs = new std::vector<std::shared_ptr<InFlightRun>>();
    return *runs;
}

// Drop the runs that have finished, releasing their node and executor.
void free_finished_runs() {
    auto& runs = in_flight_runs();
    std::vector<std::shared_ptr<InFlightRun>> finished;
    auto running = runs.begin();
    for (auto& run : runs) {
        if (run->finished())
            finished.push_back(std::move(run));
        else
            *running++ = std::move(run);
    }
    runs.erase(running, runs.end());
    // Python references go with the GIL held, after the list is consistent again.
    for (auto& run : finished) {
        run->node = nb::object();
        run->owner = nb::object();
    }
    // The last reference to a C++-held executor joins its workers, which may wait for the GIL in
    // a Python callback: release it without the GIL.
    nb::gil_scoped_release release;
    finished.clear();
}

// How often the exit drain polls an executor whose last run set its future but is still being
// counted off: one step of Taskflow's teardown, microseconds unless the worker is preempted.
constexpr auto EXIT_DRAIN_IDLE_POLL = std::chrono::milliseconds(1);

// tesseract_common's flushOutputHandlers within the log delivery's exit budget, with the GIL held:
// false when the budget ran out. Set once at module init.
bool (*flush_output_handlers_at_exit)() = nullptr;

// Wait for every run still in flight with the GIL released (a worker's log handler or a run
// started meanwhile may need the GIL), before the interpreter tears down what they use. Other threads may start runs while it waits: repeat until none is left. A run is
// done when its executor is idle, not when its future is ready: Taskflow sets the future first,
// and the worker is still in the executor's code after it.
// A run still starting (null future) goes back on the list until its thread registers the
// future. The starter assigns `future` with the GIL held, so the entries are sorted with the GIL
// held, and only entries whose future was already set are touched without it.
void wait_for_in_flight_runs() {
    while (!in_flight_runs().empty()) {
        std::vector<std::shared_ptr<InFlightRun>> runs;
        runs.swap(in_flight_runs());
        std::vector<std::shared_ptr<InFlightRun>> started;
        std::vector<std::shared_ptr<InFlightRun>> starting;
        for (auto& run : runs) {
            if (run->future != nullptr)
                started.push_back(std::move(run));
            else if (!run->start_failed)
                starting.push_back(std::move(run));
        }
        runs.clear();
        {
            nb::gil_scoped_release release;
            for (auto& run : started) {
                run->future->wait();
                while (run->runner->getTaskCount() != 0)
                    std::this_thread::sleep_for(EXIT_DRAIN_IDLE_POLL);
                run->executor.reset();
            }
            if (started.empty())
                std::this_thread::yield();
        }
        // Python references go with the GIL held.
        started.clear();
        for (auto& run : starting)
            if (!run->start_failed)
                in_flight_runs().push_back(std::move(run));
    }
}

// Registered with atexit. Python log handlers get their messages on tesseract_common's delivery
// thread, after they were logged: a message delivered once the last run ended may start another.
// So the drain then waits for the handlers to receive everything logged so far, and starts over
// if they started a run. It waits for the handlers only within the log delivery's exit budget
// (shared with its own exit hook, which runs next): a stuck handler does not hold up exit. Once
// the budget is spent, the runs already started are waited for one last time, and the rest of the
// messages go to stderr.
void wait_for_runs_at_exit() {
    do {
        wait_for_in_flight_runs();
        if (!flush_output_handlers_at_exit()) {
            wait_for_in_flight_runs();
            return;
        }
    } while (!in_flight_runs().empty());
}

// The Python object of a C++ argument that Python passed in.
template <typename T>
nb::object python_object(const T& value) {
    nb::handle found = nb::find(&value);
    if (!found.is_valid())
        throw std::logic_error("task composer run: argument has no Python object");
    return nb::borrow(found);
}

// Register a run, then start it with the GIL released, and hold what it uses until it ends. The
// entry exists before the GIL is released, so a reload on another thread sees the node in use.
// Callers free finished runs (which releases the GIL) before resolving `node`, never between.
template <typename Run>
std::unique_ptr<tp::TaskComposerFuture> run_held(const tp::TaskComposerNode& node_ref, nb::object node,
                                                 nb::object owner, const tp::TaskComposerExecutor& runner,
                                                 Run&& run,
                                                 std::shared_ptr<tp::TaskComposerExecutor> executor = nullptr) {
    auto entry = std::make_shared<InFlightRun>();
    entry->node_ptr = &node_ref;
    entry->node = std::move(node);
    entry->owner = std::move(owner);
    entry->runner = &runner;
    entry->executor = std::move(executor);
    in_flight_runs().push_back(entry);
    std::unique_ptr<tp::TaskComposerFuture> future;
    try {
        nb::gil_scoped_release release;
        future = run();
    } catch (...) {
        entry->start_failed = true;
        auto& runs = in_flight_runs();
        runs.erase(std::remove(runs.begin(), runs.end(), entry), runs.end());
        throw;
    }
    entry->future = future->copy();
    return future;
}

// loadConfig or addTask would replace (and free) a server task that a run or Python can reach,
// or a call reached the server from inside one of its own loadConfig / addTask / addExecutor.
struct TaskComposerServerBusyError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// tesseract_common's flush barrier (log_flush_barrier.h). Set once at module init.
const tesseract_nb_common::LogFlushBarrierApi* log_flush_barrier = nullptr;

// Upstream's server has no lock. loadConfig and addTask (mutations) hold the GIL, but give it up
// inside Python callbacks they make (a ResourceLocator, a Python-owned task's deleter), where
// other threads run; addExecutor (a mutation too) runs without the GIL, because destroying an
// executor joins workers that may wait for the GIL. While a mutation runs, every other server
// call (enter_server) on another thread waits for it with the GIL released, a Python log handler
// on tesseract_common's delivery thread included, and raises only where waiting cannot end:
// - on the mutating thread itself (a Python ResourceLocator inside the load);
// - on a thread mutating another server: two loads whose ResourceLocators reach each other's
//   server would wait for each other.
// A mutation never waits for the delivery thread, so a handler waiting here always gets in: the
// mutating thread is inside tesseract_common's flush barrier, where flushOutputHandlers (from a
// Python ResourceLocator) raises instead of waiting for a handler that waits for this mutation.
// Lock order: the GIL, then `mutex`. `mutex` is taken with the GIL held only by the mutating
// thread (ServerMutation), and released before anything that can log or call Python; a waiter
// releases the GIL before taking it. Nothing acquires the GIL while holding `mutex`.
// `busy` and `thread` are written with both the GIL and `mutex` held, so either one reads them.
struct ServerGate {
    std::mutex mutex;
    std::condition_variable idle;
    bool busy = false;
    std::thread::id thread;
};

// Mutations (ServerMutation) running on this thread, on any server.
thread_local int server_mutations_on_this_thread = 0;

// One gate per server address. GIL-protected; never freed, so a waiter's reference stays valid
// with the GIL released. A later server at the same address reuses an idle gate.
ServerGate& server_gate(const tp::TaskComposerServer& server) {
    static auto* gates = new std::unordered_map<const tp::TaskComposerServer*, std::unique_ptr<ServerGate>>();
    auto& gate = (*gates)[&server];
    if (!gate)
        gate = std::make_unique<ServerGate>();
    return *gate;
}

// Enter the server: returns with the GIL held and no mutation running, so none starts until the
// caller releases the GIL. Callers free finished runs (which releases the GIL) before, never after.
void enter_server(const char* caller, const tp::TaskComposerServer& server) {
    ServerGate& gate = server_gate(server);
    while (gate.busy) {
        if (gate.thread == std::this_thread::get_id())
            throw TaskComposerServerBusyError(std::string(caller) +
                                              ": called from inside this server's loadConfig, addTask or addExecutor "
                                              "(a ResourceLocator)");
        if (server_mutations_on_this_thread != 0)
            throw TaskComposerServerBusyError(std::string(caller) +
                                              ": called from inside another server's loadConfig, addTask or "
                                              "addExecutor while this server is reloading: they could wait for "
                                              "each other");
        nb::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(gate.mutex);
        gate.idle.wait(lock, [&] { return !gate.busy; });
    }
}

// Marks a server busy for one mutation, also when it throws, and keeps the mutating thread in
// tesseract_common's flush barrier meanwhile (`caller`, a string literal, names it there). Enter
// the server (enter_server) first, with the GIL held since. A mutation that releases the GIL
// declares the release after this, so the gate is cleared with the GIL held.
struct ServerMutation {
    ServerGate& gate;
    ServerMutation(const char* caller, const tp::TaskComposerServer& server) : gate(server_gate(server)) {
        log_flush_barrier->enter(caller);
        std::lock_guard<std::mutex> lock(gate.mutex);
        gate.busy = true;
        gate.thread = std::this_thread::get_id();
        ++server_mutations_on_this_thread;
    }
    ServerMutation(const ServerMutation&) = delete;
    ServerMutation& operator=(const ServerMutation&) = delete;
    ~ServerMutation() {
        --server_mutations_on_this_thread;
        log_flush_barrier->leave();
        {
            std::lock_guard<std::mutex> lock(gate.mutex);
            gate.busy = false;
            gate.thread = std::thread::id();
        }
        gate.idle.notify_all();
    }
};

// Tasks a server got from Python (addTask), per server. Python owns such a task: the server's
// shared_ptr only holds its Python object, which owns it (or holds its own shared_ptr, e.g. a
// getNodes() child), so replacing it never frees it under a wrapper. A weak_ptr to the server's
// shared_ptr names the task exactly: it expires when the server drops it, so a later task at the
// same address never matches. Entries of a destroyed server name objects that server no longer
// holds; a later server at its address only matches them by adding the same object from Python.
// GIL-protected; never freed.
struct PythonAddedTask {
    const tp::TaskComposerServer* server;
    std::weak_ptr<tp::TaskComposerNode> task;
};
std::vector<PythonAddedTask>& python_added_tasks() {
    static auto* tasks = new std::vector<PythonAddedTask>();
    return *tasks;
}

void record_python_added_task(const tp::TaskComposerServer& server, const std::shared_ptr<tp::TaskComposerNode>& task) {
    auto& tasks = python_added_tasks();
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(), [](const PythonAddedTask& t) { return t.task.expired(); }),
                tasks.end());
    tasks.push_back({ &server, task });
}

bool is_python_added_task(const tp::TaskComposerServer& server, const tp::TaskComposerNode* task) {
    for (const auto& added : python_added_tasks())
        if (added.server == &server && added.task.lock().get() == task)
            return true;
    return false;
}

// The one invariant for server tasks: a task is not replaced while a run or Python can reach it.
// Raise TaskComposerServerBusyError if any of `names` (tasks of `server`) is the node of an
// unfinished run, on any executor or path, or a task the server owns has a live Python wrapper
// (a getTask() result, which does not own it). Call after enter_server, with the GIL held since;
// the caller then marks the server busy (ServerMutation), which keeps every other server call
// waiting until the replacement is done.
void require_tasks_replaceable(const char* caller, const tp::TaskComposerServer& server,
                               const std::vector<std::string>& names) {
    std::string running;
    std::string referenced;
    for (const auto& name : names) {
        const tp::TaskComposerNode* task = &const_cast<tp::TaskComposerServer&>(server).getTask(name);
        for (const auto& run : in_flight_runs()) {
            if (run->node_ptr == task && !run->finished()) {
                running += (running.empty() ? "'" : ", '") + name + "'";
                break;
            }
        }
        // A wrapper of a task added from Python is the object that owns it (the server's
        // shared_ptr keeps it alive, so it is always found): replacing the task only drops that
        // reference. Any wrapper of a task the server owns is a getTask() view and would dangle.
        if (!is_python_added_task(server, task) && nb::find(task).is_valid())
            referenced += (referenced.empty() ? "'" : ", '") + name + "'";
    }
    if (running.empty() && referenced.empty())
        return;
    std::string reason;
    if (!running.empty())
        reason += "task " + running + " is running (wait() for its runs' futures first)";
    if (!referenced.empty())
        reason += std::string(reason.empty() ? "" : "; ") + "task " + referenced +
                  " is referenced from Python (del the getTask() result first)";
    throw TaskComposerServerBusyError(std::string(caller) + ": " + reason +
                                      "; replacing it would free it under its user");
}

// prune found stored infos that changed (or were removed) while its callbacks ran.
struct TaskComposerPruneConflictError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Copies of the stored infos, taken under the container's lock without calling Python: the
// callbacks of find/prune run outside the lock, so they may re-enter the container (#320).
std::vector<tp::TaskComposerNodeInfo> snapshot(const tp::TaskComposerNodeInfoContainer& container) {
    std::vector<tp::TaskComposerNodeInfo> infos;
    container.find([&](const tp::TaskComposerNodeInfo& info) {
        infos.push_back(info);
        return false;
    });
    return infos;
}

}  // anonymous namespace

NB_MODULE(_tesseract_task_composer, m) {
    m.doc() = "tesseract_task_composer Python bindings";

    // Import the modules that bind the types this one takes and returns, so nanobind names
    // them in signatures (#312): ResourceLocator and AnyPoly's ProfileDictionary come from
    // common and command language, Environment and ContactResultMap from their own modules.
    {
        nb::module_ common = nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
        auto barrier = nb::cast<nb::capsule>(common.attr("_log_flush_barrier"));
        if (std::strcmp(barrier.name(), tesseract_nb_common::LOG_FLUSH_BARRIER_CAPSULE) != 0)
            throw std::logic_error("tesseract_common._log_flush_barrier: unexpected capsule");
        log_flush_barrier = static_cast<const tesseract_nb_common::LogFlushBarrierApi*>(barrier.data());
        auto flush = nb::cast<nb::capsule>(common.attr("_flush_output_handlers_at_exit"));
        if (std::strcmp(flush.name(), "tesseract_robotics.flush_output_handlers_at_exit") != 0)
            throw std::logic_error("tesseract_common._flush_output_handlers_at_exit: unexpected capsule");
        flush_output_handlers_at_exit = reinterpret_cast<bool (*)()>(flush.data());
    }
    // Runs still in flight at exit finish before the interpreter tears down (run_held). Registered
    // after tesseract_common registers its own exit hook, so it runs first (atexit is last in,
    // first out): while it waits, the Python log handlers still get their messages.
    nb::module_::import_("atexit").attr("register")(nb::cpp_function(&wait_for_runs_at_exit));
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

    nb::exception<TaskComposerServerBusyError>(m, "TaskComposerServerBusyError", PyExc_RuntimeError).attr("__doc__") =
        "TaskComposerServer.loadConfig / addTask would replace a task that a run or Python can reach "
        "(wait() for the runs, del the getTask() results first), or the server is inside loadConfig / "
        "addTask and getTask / run cannot reach its tasks.";

    nb::exception<TaskComposerPruneConflictError>(m, "TaskComposerPruneConflictError", PyExc_RuntimeError)
        .attr("__doc__") =
        "TaskComposerNodeInfoContainer.prune: an info changed or was removed while prune ran, so "
        "the edit of its stale copy was not stored.";

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
        .def("find", [](const tp::TaskComposerNodeInfoContainer& self,
                        const std::function<bool(const tp::TaskComposerNodeInfo&)>& search_fn) {
            std::vector<tp::TaskComposerNodeInfo> found;
            for (auto& info : snapshot(self))
                if (search_fn(info))
                    found.push_back(std::move(info));
            return found;
        }, "search_fn"_a,
           "Copies of the infos for which search_fn(info) is true. search_fn sees a snapshot taken "
           "before the first call, and may call back into the container.")
        .def("insertInfoMap", &tp::TaskComposerNodeInfoContainer::insertInfoMap, "container"_a)
        // Upstream takes the container by rvalue reference: take a copy and move it in.
        .def("mergeInfoMap", [](tp::TaskComposerNodeInfoContainer& self, tp::TaskComposerNodeInfoContainer container) {
            self.mergeInfoMap(std::move(container));
        }, "container"_a)
        .def("clear", &tp::TaskComposerNodeInfoContainer::clear)
        // prune_fn gets a Python-owned copy of each stored info (a reference into the container
        // dangled once the container dropped it). The changed copies are written back under the
        // lock when every call has returned, each only if the stored info still equals the copy
        // prune_fn was given: a stale copy never overwrites an info changed meanwhile.
        .def("prune", [](tp::TaskComposerNodeInfoContainer& self, const nb::callable& prune_fn) {
            struct Edit {
                tp::TaskComposerNodeInfo original;
                tp::TaskComposerNodeInfo edited;
                bool stored = false;
            };
            std::map<boost::uuids::uuid, Edit> edits;
            for (auto& info : snapshot(self)) {
                nb::object py_info = nb::cast(info, nb::rv_policy::copy);
                prune_fn(py_info);
                const auto& edited = nb::cast<const tp::TaskComposerNodeInfo&>(py_info);
                if (!(edited == info))
                    edits.emplace(info.uuid, Edit{ info, edited });
            }
            std::string conflicts;
            self.prune([&](tp::TaskComposerNodeInfo& stored) {
                auto it = edits.find(stored.uuid);
                if (it == edits.end())
                    return;
                it->second.stored = true;
                if (stored == it->second.original)
                    stored = it->second.edited;
                else
                    conflicts += (conflicts.empty() ? "" : ", ") + boost::uuids::to_string(stored.uuid) + " (changed)";
            });
            for (const auto& [uuid, edit] : edits)
                if (!edit.stored)
                    conflicts += (conflicts.empty() ? "" : ", ") + boost::uuids::to_string(uuid) + " (removed)";
            if (!conflicts.empty())
                throw TaskComposerPruneConflictError(
                    "prune: these infos changed or were removed while prune ran, so their edits were "
                    "not stored (the other edits were): " + conflicts);
        }, "prune_fn"_a,
           nb::sig("def prune(self, prune_fn: collections.abc.Callable[[TaskComposerNodeInfo], None]) -> None"),
           "Call prune_fn(info) on a copy of every stored info, then store the copies it changed. "
           "prune_fn sees a snapshot taken before the first call, and may call back into the "
           "container. A changed copy is stored only if the stored info is still the one prune_fn "
           "was given; if any was changed or removed meanwhile, the other edits are stored and "
           "TaskComposerPruneConflictError names the conflicting uuids. Edits made to `info` after "
           "prune returns, or before prune_fn raises, are not stored.");
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
        // A finished run's node and executor are released as soon as its future is seen done.
        .def("ready", [](const tp::TaskComposerFuture& self) {
            const bool ready = self.ready();
            if (ready)
                free_finished_runs();
            return ready;
        })
        .def("wait", [](const tp::TaskComposerFuture& self) {
            {
                nb::gil_scoped_release release;
                self.wait();
            }
            free_finished_runs();
        })
        .def("waitFor", [](const tp::TaskComposerFuture& self, const std::chrono::duration<double>& duration) {
            std::future_status status;
            {
                nb::gil_scoped_release release;
                status = self.waitFor(duration);
            }
            if (status == std::future_status::ready)
                free_finished_runs();
            return status;
        }, "duration"_a)
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
        // throw). Here a failed open or write raises OSError (FileNotFoundError for a missing
        // directory), as on saveConfig; the file holds the same text as getDotgraph.
        .def("saveDotgraph", [](const tp::TaskComposerNode& self, const std::filesystem::path& filepath,
                                const tp::TaskComposerNode::ResultsMap& results_map) {
            errno = 0;
            std::ofstream out(filepath);
            if (out) out << self.getDotgraph(results_map);
            if (out) out.close();
            if (!out) {
                PyErr_SetFromErrnoWithFilename(PyExc_OSError, filepath.string().c_str());
                throw nb::python_error();
            }
        }, "filepath"_a, "results_map"_a = tp::TaskComposerNode::ResultsMap(),
           "Write getDotgraph(results_map) to `filepath`. Raises OSError if it cannot be written.");

    // ========== TaskComposerExecutor (abstract base) ==========
    // run(node, data_storage, dotgraph, name) builds the context from a data storage, as
    // upstream's TaskComposerServer::run(node, data_storage, dotgraph, executor_name) does.
    // The run uses `node` and the executor in the executor's threads: run_held keeps both until
    // the run ends, whatever happens to the future.
    executor
        .def("getName", &tp::TaskComposerExecutor::getName)
        .def("run", [](tp::TaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerContext> context) {
            free_finished_runs();
            return run_held(node, python_object(node), python_object(self), self,
                            [&] { return self.run(node, context); });
        }, "node"_a, "context"_a)
        .def("run", [](tp::TaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph,
                       const std::string& name) {
            auto context = std::make_shared<tp::TaskComposerContext>(name, data_storage, dotgraph);
            free_finished_runs();
            return run_held(node, python_object(node), python_object(self), self,
                            [&] { return self.run(node, context); });
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false, "name"_a = "run")
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
            free_finished_runs();
            return run_held(node, python_object(node), python_object(self), self,
                            [&] { return static_cast<tp::TaskComposerExecutor&>(self).run(node, context); });
        }, "node"_a, "context"_a)
        .def("run", [](tp::TaskflowTaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph,
                       const std::string& name) {
            auto context = std::make_shared<tp::TaskComposerContext>(name, data_storage, dotgraph);
            free_finished_runs();
            return run_held(node, python_object(node), python_object(self), self,
                            [&] { return static_cast<tp::TaskComposerExecutor&>(self).run(node, context); });
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false, "name"_a = "run")
        .def("getWorkerCount", &tp::TaskflowTaskComposerExecutor::getWorkerCount)
        .def("getTaskCount", &tp::TaskflowTaskComposerExecutor::getTaskCount);

    // ========== TaskComposerPluginFactory ==========
    // Move-only class (deleted copy, has move). Use unique_ptr internally to handle moves.
    // The locator binds to the ResourceLocator base (registered by _tesseract_common, imported
    // at module init), so GeneralResourceLocator and Python subclasses are both accepted (gh-150).
    // The locator is only consulted during construction; the factory keeps no reference to it.
    //
    // Every call that can dlopen plugin libraries (create*, getAvailable*) runs through
    // with_pinned_plugins: on macOS and Windows those libraries are pinned immediately after the
    // call, so the factory's own unload (on destruction) cannot unload them (plugin_pinning.h,
    // GH #48).
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

    nb::exception<TaskComposerConfigError>(m, "TaskComposerConfigError", PyExc_ValueError).attr("__doc__") =
        "A `str` config passed to TaskComposerPluginFactory or TaskComposerServer is not a loadable "
        "task composer config (a `str` is YAML text; pass a pathlib.Path to load a file).";

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
    // plugin factory, so it pins the libraries (GH #48), and returned executors and tasks keep the
    // server alive: their code lives in those libraries and the tasks in the server. A run holds the
    // server (and a passed node) until it ends (run_held).
    using Server = tp::TaskComposerServer;
    nb::class_<Server>(m, "TaskComposerServer")
        .def(nb::init<>())
        .def("loadConfig", [](Server& self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            free_finished_runs();
            enter_server("TaskComposerServer.loadConfig", self);
            require_tasks_replaceable("TaskComposerServer.loadConfig", self, self.getAvailableTasks());
            ServerMutation mutation("TaskComposerServer.loadConfig", self);
            with_pinned_plugins([&] { self.loadConfig(config_path.value, locator); });
        }, "config_path"_a, "locator"_a)
        .def("loadConfig", [](Server& self, const std::string& config, const tc::ResourceLocator& locator) {
            free_finished_runs();
            enter_server("TaskComposerServer.loadConfig", self);
            require_tasks_replaceable("TaskComposerServer.loadConfig", self, self.getAvailableTasks());
            ServerMutation mutation("TaskComposerServer.loadConfig", self);
            load_yaml_text("TaskComposerServer.loadConfig", config,
                           [&] { with_pinned_plugins([&] { self.loadConfig(config, locator); }); });
        }, "config"_a, "locator"_a)
        // Replacing an executor destroys the old one when nothing else holds it, which joins its
        // workers: they may wait for the GIL in a Python callback, so it runs without the GIL, as a
        // mutation (ServerGate). A run holds its executor (run_held), so a replaced executor is
        // never destroyed mid-run.
        .def("addExecutor", [](Server& self, const std::shared_ptr<tp::TaskComposerExecutor>& executor) {
            enter_server("TaskComposerServer.addExecutor", self);
            ServerMutation mutation("TaskComposerServer.addExecutor", self);
            nb::gil_scoped_release release;  // see ServerGate
            self.addExecutor(executor);
        }, "executor"_a)
        .def("getExecutor", [](Server& self, const std::string& name) {
            enter_server("TaskComposerServer.getExecutor", self);
            return self.getExecutor(name);
        }, "name"_a, nb::keep_alive<0, 1>())
        .def("hasExecutor", [](const Server& self, const std::string& name) {
            enter_server("TaskComposerServer.hasExecutor", self);
            return self.hasExecutor(name);
        }, "name"_a)
        .def("getAvailableExecutors", [](const Server& self) {
            enter_server("TaskComposerServer.getAvailableExecutors", self);
            return self.getAvailableExecutors();
        })
        // Upstream's hook for Python wrappers; addTask(unique_ptr) has no Python form. Adding a task
        // under a name a by-name run is using would free that run's task.
        .def("addTask", [](Server& self, std::shared_ptr<tp::TaskComposerNode> task) {
            free_finished_runs();
            enter_server("TaskComposerServer.addTask", self);
            if (self.hasTask(task->getName()))
                require_tasks_replaceable("TaskComposerServer.addTask", self, { task->getName() });
            ServerMutation mutation("TaskComposerServer.addTask", self);
            record_python_added_task(self, task);
            self.addTaskPython(std::move(task));
        }, "task"_a,
           "Add `task` under its name, replacing a task of that name. Raises "
           "TaskComposerServerBusyError if that task is running or referenced from Python (a "
           "getTask() result).")
        .def("getTask", [](Server& self, const std::string& name) -> const tp::TaskComposerNode& {
            enter_server("TaskComposerServer.getTask", self);
            return self.getTask(name);
        }, "name"_a, nb::rv_policy::reference_internal,
           "The server's task `name`. While this reference lives, loadConfig and addTask of that "
           "name raise TaskComposerServerBusyError: `del` it first.")
        .def("hasTask", [](const Server& self, const std::string& name) {
            enter_server("TaskComposerServer.hasTask", self);
            return self.hasTask(name);
        }, "name"_a)
        .def("getAvailableTasks", [](const Server& self) {
            enter_server("TaskComposerServer.getAvailableTasks", self);
            return self.getAvailableTasks();
        })
        .def("run", [](Server& self, const std::string& task_name, std::shared_ptr<tp::TaskComposerDataStorage> data_storage,
                       bool dotgraph, const std::string& executor_name) {
            // Upstream's run(task_name, ...), on an executor resolved once: the executor held is the
            // one running, even if addExecutor replaces it meanwhile. getExecutor and getTask throw
            // upstream's errors, the executor's first. The server owns the task; the run is
            // registered (run_held) before the GIL is released, so loadConfig sees it.
            free_finished_runs();
            enter_server("TaskComposerServer.run", self);
            auto executor = self.getExecutor(executor_name);
            const tp::TaskComposerNode& task = self.getTask(task_name);
            data_storage->setName(task_name);
            auto context = std::make_shared<tp::TaskComposerContext>(task_name, std::move(data_storage), dotgraph);
            return run_held(task, nb::none(), python_object(self), *executor, [&] { return executor->run(task, context); },
                            executor);
        }, "task_name"_a, "data_storage"_a, "dotgraph"_a, "executor_name"_a)
        .def("run", [](Server& self, const tp::TaskComposerNode& node, std::shared_ptr<tp::TaskComposerDataStorage> data_storage,
                       bool dotgraph, const std::string& executor_name) {
            // Upstream's run(node, ...), on an executor resolved once (see above).
            free_finished_runs();
            enter_server("TaskComposerServer.run", self);
            auto executor = self.getExecutor(executor_name);
            data_storage->setName(node.getName());
            auto context = std::make_shared<tp::TaskComposerContext>(node.getName(), std::move(data_storage), dotgraph);
            return run_held(node, python_object(node), python_object(self), *executor,
                            [&] { return executor->run(node, context); },
                            executor);
        }, "node"_a, "data_storage"_a, "dotgraph"_a, "executor_name"_a)
        .def("getWorkerCount", [](const Server& self, const std::string& name) {
            enter_server("TaskComposerServer.getWorkerCount", self);
            return self.getWorkerCount(name);
        }, "name"_a)
        .def("getTaskCount", [](const Server& self, const std::string& name) {
            enter_server("TaskComposerServer.getTaskCount", self);
            return self.getTaskCount(name);
        }, "name"_a);

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
            return type_name(self.getType());
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
