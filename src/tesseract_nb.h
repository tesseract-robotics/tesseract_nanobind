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

// Namespace aliases
namespace nb = nanobind;
using namespace nb::literals;

// Bind C++ operator==/operator!= as __eq__/__ne__. These types are mutable, so they must
// not hash: nanobind adds __eq__ after the type exists, which leaves object.__hash__
// (identity) in place unless it is cleared explicitly.
template <typename Class>
void bind_value_equality(Class& cls) {
    cls.def(nb::self == nb::self).def(nb::self != nb::self);
    cls.attr("__hash__") = nb::none();
}

// Note: Eigen::Isometry3d is bound as an explicit class in tesseract_common_bindings.cpp
// for SWIG API compatibility (tests expect .matrix() method etc.)
// No type caster is needed since we have explicit bindings.
