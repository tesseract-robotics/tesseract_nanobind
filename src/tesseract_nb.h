#pragma once

// Standard library
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <functional>
#include <variant>
#include <optional>
#include <initializer_list>
#include <stdexcept>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// nanobind core
#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/eigen/sparse.h>

// Namespace aliases
namespace nb = nanobind;
using namespace nb::literals;

namespace tesseract_nb {

// Import a sibling extension module by its short name, resolving it against the
// importer's package (e.g. inside `tesseract.tesseract_environment`, calling
// `import_module_in_context(m, "tesseract_collision")` imports
// `tesseract.tesseract_collision`).
//
// Why the two branches exist:
//   - Runtime: the binding is imported as `tesseract.tesseract_<X>`. The
//     `__name__` attribute contains a dot, so we resolve siblings via the
//     package prefix. Missing siblings here are a real bug — propagate the
//     ImportError.
//   - Stub generation: `nanobind_add_stub` runs the binding in isolation via
//     `python -m nanobind.stubgen -m <X>`. The PYTHON_PATH is just the build
//     directory, the importer's `__name__` is the bare module name (no dot),
//     and sibling .so files may not be built yet (CMake doesn't know about
//     the implicit cross-module deps that `import_module_in_context` creates).
//     Tolerate ImportError here: the stub only needs the *importer's own*
//     symbols, and missing sibling references in type signatures degrade to
//     string-form names — acceptable until upstream `nanobind_add_stub` learns
//     about cross-module deps. The presence-of-dot heuristic distinguishes
//     these two regimes without a separate flag.
//
// Callers that use the return value (`.attr(...)`) at NB_MODULE top-level
// would crash on the empty module returned during stub generation. By
// convention, such usage is only valid inside lambdas (runtime), where the
// dot-prefixed branch always runs.
inline nb::module_ import_module_in_context(const nb::module_& current_module, const char* module_name)
{
    std::string importer_name = nb::cast<std::string>(current_module.attr("__name__"));
    auto dot = importer_name.rfind('.');
    if (dot != std::string::npos)
    {
        auto package_name = importer_name.substr(0, dot);
        auto fq_name = package_name + "." + module_name;
        return nb::module_::import_(fq_name.c_str());
    }
    // Stub-gen path: dependent sibling may not exist yet; swallow ImportError.
    try
    {
        return nb::module_::import_(module_name);
    }
    catch (const nb::python_error&)
    {
        PyErr_Clear();
        return nb::module_();
    }
}

inline void import_modules_in_context(const nb::module_& current_module,
                                      std::initializer_list<const char*> module_names)
{
    for (const char* module_name : module_names)
        import_module_in_context(current_module, module_name);
}

}  // namespace tesseract_nb

// Note: Eigen::Isometry3d is bound as an explicit class in src/tesseract/tesseract_common.cpp
// for SWIG API compatibility (tests expect .matrix() method etc.)
// No type caster is needed since we have explicit bindings.
