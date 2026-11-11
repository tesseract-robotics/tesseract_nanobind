/**
 * @file task_composer_plugin_factory.cpp
 * @brief tesseract_task_composer: TaskComposerPluginFactory
 */

#include "task_composer_bindings.h"

#include <fstream>

#include <tesseract/common/resource_locator.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>

namespace tp = tesseract::task_composer;
namespace tc = tesseract::common;
using tesseract_nb::load_yaml_text;
using tesseract_nb::with_pinned_plugins;

namespace {

// createTaskComposer{Node,Executor} never throw: an unknown name, a plugin symbol that fails to
// load and a plugin factory that throws all log a tesseract warning and return nullptr
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
                                  "' is configured but failed to load or build; the tesseract "
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
                                  "') failed to load or build; the tesseract warning above gives "
                                  "the cause");
}

// Upstream remove*/setDefault* throw std::runtime_error for an unknown name; a name lookup that
// misses is a KeyError here, as on ContactManagersPluginFactory (#170).
void require_plugin(const tp::TaskComposerPluginFactory::PluginInfoMap& plugins, const char* kind,
                    const std::string& name) {
    if (plugins.find(name) == plugins.end())
        throw nb::key_error(("no task composer " + std::string(kind) + " plugin '" + name + "'").c_str());
}

}  // namespace

void bind_plugin_factory(nb::module_& m) {
    // ========== TaskComposerPluginFactory ==========
    // Move-only class (deleted copy, has move). Use unique_ptr internally to handle moves.
    // The locator binds to the ResourceLocator base (registered by common, imported
    // at module init), so GeneralResourceLocator and Python subclasses are both accepted (gh-150).
    // The locator is only consulted during construction; the factory keeps no reference to it.
    //
    // Every call that can dlopen plugin libraries (create*, getAvailable*) runs through
    // with_pinned_plugins: on macOS and Windows those libraries are pinned immediately after the
    // call, so the factory's own unload (on destruction) cannot unload them (task_composer_plugin_pinning.h,
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

    using Factory = tp::TaskComposerPluginFactory;
    nb::class_<Factory>(m, "TaskComposerPluginFactory")
        .def(nb::init<>())
        .def(nb::init<const tc::TaskComposerPluginInfo&>(), "config"_a)
        // StrictPath (tesseract_nb.h): a `str` is always YAML text, an os.PathLike a file (#337).
        .def("__init__", [](Factory* self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            require_located_file(config_path.value, locator);
            new (self) Factory(config_path.value, locator);
        }, "config_path"_a, "locator"_a)
        .def("__init__", [](Factory* self, const std::string& config, const tc::ResourceLocator& locator) {
            load_yaml_text("TaskComposerPluginFactory", config, [&] { new (self) Factory(config, locator); });
        }, "config"_a, "locator"_a)
        .def("loadConfig", nb::overload_cast<const tc::TaskComposerPluginInfo&>(&Factory::loadConfig), "config"_a)
        .def("loadConfig", [](Factory& self, const tesseract_nb::StrictPath& config_path, const tc::ResourceLocator& locator) {
            require_located_file(config_path.value, locator);
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

}
