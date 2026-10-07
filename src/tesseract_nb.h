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
#include <stdexcept>
#include <filesystem>

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
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/eigen/sparse.h>
#include <nanobind/operators.h>

// nanobind's std::unordered_map caster (stl/unordered_map.h, nanobind 2.12) names
// only four template parameters, so it matches default-allocator maps only.
// Tesseract's Aligned* maps (e.g. tesseract::common::TransformMap) use
// Eigen::aligned_allocator and would otherwise have no Python conversion.
NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)
template <typename Key, typename T, typename Hash, typename KeyEqual>
struct type_caster<std::unordered_map<Key, T, Hash, KeyEqual, Eigen::aligned_allocator<std::pair<const Key, T>>>>
  : dict_caster<std::unordered_map<Key, T, Hash, KeyEqual, Eigen::aligned_allocator<std::pair<const Key, T>>>, Key, T>
{
};
NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)

namespace tesseract_nb
{
// A std::filesystem::path parameter that refuses `str` and `bytes`.
//
// nanobind's std::filesystem::path caster (stl/filesystem.h) accepts anything
// PyOS_FSPath accepts, `str` included, and ignores the convert flag. Where a
// native path overload sits beside a std::string *content* overload
// (Environment::init, the plugin factories' config ctors), a `str` would match
// both. Binding the path overload with StrictPath makes `str` always mean
// content and `os.PathLike` (pathlib.Path) always mean a path, whatever the
// registration order. Use it only for such path/string pairs; every other
// path parameter takes plain std::filesystem::path (`str | os.PathLike`).
struct StrictPath
{
  std::filesystem::path value;
};
}  // namespace tesseract_nb

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)
template <>
struct type_caster<tesseract_nb::StrictPath>
{
  NB_TYPE_CASTER(tesseract_nb::StrictPath, const_name("os.PathLike"))

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
  {
    if (PyUnicode_Check(src.ptr()) || PyBytes_Check(src.ptr()))
      return false;
    make_caster<std::filesystem::path> path_caster;
    if (!path_caster.from_python(src, flags, cleanup))
      return false;
    value.value = std::move(path_caster.value);
    return true;
  }
};
NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)

// Namespace aliases
namespace nb = nanobind;
using namespace nb::literals;

// Bind C++ operator==/operator!= as __eq__/__ne__. These types are mutable, so they must
// not hash: nanobind adds __eq__ after the type exists, which leaves object.__hash__
// (identity) in place unless it is cleared explicitly. `extra` goes to both operators
// (e.g. nb::call_guard<nb::gil_scoped_release>() for a comparison that takes a C++ lock).
template <typename Class, typename... Extra>
void bind_value_equality(Class& cls, const Extra&... extra) {
    cls.def(nb::self == nb::self, extra...).def(nb::self != nb::self, extra...);
    cls.attr("__hash__") = nb::none();
}

// nanobind rejects None for a direct std::shared_ptr<T> argument, but converts a None *element*
// of a list to a null std::shared_ptr<T>, which upstream later dereferences (gh-201). Every
// binding that takes a std::vector<std::shared_ptr<T>> passes it through this check first:
// TypeError "<caller>: <arg>[i] is None, expected <expected>".
template <typename T>
const std::vector<std::shared_ptr<T>>& require_non_null(const std::vector<std::shared_ptr<T>>& items,
                                                        const char* caller, const char* arg,
                                                        const char* expected) {
    for (std::size_t i = 0; i < items.size(); ++i)
        if (!items[i])
            throw nb::type_error((std::string(caller) + ": " + arg + "[" + std::to_string(i) +
                                  "] is None, expected " + expected)
                                     .c_str());
    return items;
}

// Note: Eigen::Isometry3d is bound as an explicit class in tesseract_common_bindings.cpp
// for SWIG API compatibility (tests expect .matrix() method etc.)
// No type caster is needed since we have explicit bindings.
