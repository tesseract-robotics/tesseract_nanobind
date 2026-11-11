#pragma once

// The tesseract_task_composer module's translation units, one concern each, and what they
// share; NB_MODULE(task_composer) in task_composer.cpp calls them in registration order.

#include "tesseract_nb.h"
#include "task_composer_plugin_pinning.h"

#include <yaml-cpp/yaml.h>
#include <tesseract/common/plugin_info.h>

#include <stdexcept>
#include <string>

namespace tesseract_nb {

// Run `load`, which may load plugin libraries, and pin every library it loaded (task_composer_plugin_pinning.h),
// also when it throws.
template <typename Load>
decltype(auto) with_pinned_plugins(Load&& load) {
    struct Pin {
        tesseract_nb::LoadedLibraries before = tesseract_nb::snapshot_loaded_libraries();
        ~Pin() { tesseract_nb::pin_libraries_loaded_since(before); }
    } pin;
    return load();
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
        if (!root.IsMap() || !root[tesseract::common::TaskComposerPluginInfo::CONFIG_KEY])
            throw TaskComposerConfigError(std::string(caller) + ": `config` is YAML text (pass a pathlib.Path "
                                          "to load a file) without a top-level `" +
                                          tesseract::common::TaskComposerPluginInfo::CONFIG_KEY + "` mapping; config: '" +
                                          config.substr(0, 200) + "'");
        load();
    } catch (const YAML::Exception& e) {
        throw TaskComposerConfigError(std::string(caller) + ": `config` is YAML text (pass a pathlib.Path "
                                      "to load a file) and did not load as a task composer config: " +
                                      e.what() + "; config: '" + config.substr(0, 200) + "'");
    }
}

}  // namespace tesseract_nb

// TaskComposerPluginFactory (task_composer_plugin_factory.cpp).
void bind_plugin_factory(nb::module_& m);

// AnyPoly and its wrap / unwrap functions (task_composer_anypoly.cpp).
void bind_anypoly(nb::module_& m);
