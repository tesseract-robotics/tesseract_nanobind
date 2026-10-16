#pragma once

// Profile YAML helper. A separate header so only the profile TUs pull in yaml-cpp and
// tesseract/common/profile_plugin_factory.h.

#include "tesseract_nb.h"

#include <yaml-cpp/yaml.h>
#include <tesseract/common/profile_plugin_factory.h>

// Bind `T.from_yaml(config: str)` for a profile type T with the upstream constructor
// T(const YAML::Node&, const ProfilePluginFactory&). The factory is default-constructed:
// upstream's profile constructors ignore it, and no installed library defines a profile
// plugin (#331). Malformed YAML raises ValueError with yaml-cpp's message. The constructor's
// own errors propagate as is: upstream wraps a bad value (e.g. the wrong type) in a
// std::runtime_error naming the profile, which reaches Python as RuntimeError.
template <typename Class>
void bind_from_yaml(Class& cls) {
    using T = typename Class::Type;
    cls.def_static(
        "from_yaml",
        [](const std::string& config) {
            try {
                return T(YAML::Load(config), tesseract::common::ProfilePluginFactory{});
            } catch (const YAML::Exception& e) {
                throw nb::value_error(e.what());
            }
        },
        "config"_a, "Construct the profile from its YAML config text; malformed YAML raises ValueError.");
}
