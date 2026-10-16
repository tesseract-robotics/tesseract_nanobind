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

// boost::uuids::uuid <-> str (caster below)
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>

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

// Dense Eigen arguments accept Python sequences.
//
// nanobind's dense Eigen casters (eigen/dense.h) all import their argument through
// type_caster<ndarray<Scalar, numpy, shape<...>, Order>>, and ndarray_import (nb_ndarray.cpp,
// nanobind 2.12) takes only DLPack or buffer-protocol objects: a list or tuple was a TypeError
// at every Vector3d / VectorXd / Matrix3d parameter. This specialization matches exactly the
// argument order eigen/dense.h spells (array_for_eigen_t) and delegates to nanobind's own
// caster; when that fails and implicit conversion is allowed it runs a sequence through
// numpy.asarray and asks nanobind again, so shape, dtype and order checks stay nanobind's.
//
// - Only when cast_flags::convert is set: noconvert() arguments, Eigen::Map and mutable
//   Eigen::Ref (whose casters clear the flag) stay strict.
// - Element kinds are checked before conversion: a str, object or complex array is refused
//   rather than parsed, and an integer target refuses floats rather than truncating them.
// - Every module sees this through tesseract_nb.h, the only include of nanobind/eigen and
//   nanobind/ndarray in src/, so all translation units instantiate the same caster.
NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)
template <typename Scalar, ssize_t... Is, typename Order>
struct type_caster<ndarray<Scalar, numpy, shape<Is...>, Order>>
{
  // The same array with the framework tag moved first: nanobind reads ndarray template
  // arguments in any order, and this spelling does not match this specialization,
  // so it names nanobind's own caster.
  using Array = ndarray<Scalar, numpy, shape<Is...>, Order>;
  using StockCaster = type_caster<ndarray<numpy, Scalar, shape<Is...>, Order>>;
  using Plain = std::remove_const_t<Scalar>;

  static constexpr bool IsFloat = std::is_floating_point_v<Plain>;
  static constexpr bool IsInteger = std::is_integral_v<Plain> && !std::is_same_v<Plain, bool>;
  static constexpr bool AcceptsSequence = (IsFloat || IsInteger) && (sizeof...(Is) == 1 || sizeof...(Is) == 2);

  static constexpr auto ItemName = const_name<IsFloat>("float", "int");
  static constexpr auto SequenceName =
      const_name<sizeof...(Is) == 1>(const_name("collections.abc.Sequence[") + ItemName + const_name("]"),
                                     const_name("collections.abc.Sequence[collections.abc.Sequence[") +
                                         ItemName + const_name("]]"));
  // Input: the array or a sequence; output: the array only (io_name encoding, nb_descr.h).
  static constexpr auto IOName =
      const_name<AcceptsSequence>(const_name("@") + StockCaster::Name + const_name(" | ") + SequenceName +
                                      const_name("@") + StockCaster::Name + const_name("@"),
                                  StockCaster::Name);

  NB_TYPE_CASTER(Array, IOName)

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
  {
    StockCaster stock;
    if (stock.from_python(src, flags, cleanup))
    {
      value = Value(stock.value);
      return true;
    }
    if constexpr (AcceptsSequence)
    {
      if (!(flags & (uint8_t)cast_flags::convert) || !PySequence_Check(src.ptr()) || PyUnicode_Check(src.ptr()) ||
          PyBytes_Check(src.ptr()))
        return false;
      object array;
      try
      {
        array = module_::import_("numpy").attr("asarray")(src);
        // numpy dtype.kind: b bool, i signed, u unsigned, f float. Strings (U, S), objects (O)
        // and complex (c) are refused; an integer target also refuses f.
        const char kind = borrow<str>(array.attr("dtype").attr("kind")).c_str()[0];
        const bool numeric = kind == 'b' || kind == 'i' || kind == 'u' || (IsFloat && kind == 'f');
        if (!numeric)
          return false;
      }
      catch (const python_error&)
      {
        // numpy.asarray raised (e.g. a ragged sequence): the argument does not match this
        // overload, and the dispatcher reports TypeError with the accepted signatures.
        return false;
      }
      if (stock.from_python(array, flags, cleanup))
      {
        value = Value(stock.value);
        return true;
      }
    }
    return false;
  }

  static handle from_cpp(const Value& tensor, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    return StockCaster::from_cpp(typename StockCaster::Value{tensor}, policy, cleanup);
  }
};
NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)

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

// boost::uuids::uuid <-> str, the canonical "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" spelling.
//
// from_python takes a `str` only (anything else is a TypeError from the dispatcher) and keeps
// the text; it is parsed when the bound function reads the argument, so a malformed string
// raises ValueError with the string instead of a TypeError listing the overloads. Every
// module sees this through tesseract_nb.h, so a binding takes or returns boost::uuids::uuid
// directly (`&tp::MoveInstruction::getUUID`) and its stub type is `str`.
NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)
template <>
struct type_caster<boost::uuids::uuid>
{
  using Value = boost::uuids::uuid;
  static constexpr auto Name = const_name("str");
  template <typename T_>
  using Cast = movable_cast_t<T_>;
  template <typename T_>
  static constexpr bool can_cast()
  {
    return true;
  }

  std::string text;
  Value value;

  bool from_python(handle src, uint8_t, cleanup_list*) noexcept
  {
    if (!PyUnicode_Check(src.ptr()))
      return false;
    Py_ssize_t size = 0;
    const char* data = PyUnicode_AsUTF8AndSize(src.ptr(), &size);
    if (!data)
    {
      PyErr_Clear();
      return false;
    }
    text.assign(data, static_cast<std::size_t>(size));
    return true;
  }

  static handle from_cpp(const Value& uuid, rv_policy, cleanup_list*) noexcept
  {
    const std::string s = boost::uuids::to_string(uuid);
    return PyUnicode_FromStringAndSize(s.data(), static_cast<Py_ssize_t>(s.size()));
  }

  template <typename T_, enable_if_t<std::is_same_v<std::remove_cv_t<T_>, Value>> = 0>
  static handle from_cpp(T_* p, rv_policy policy, cleanup_list* list)
  {
    if (!p)
      return none().release();
    return from_cpp(*p, policy, list);
  }

  explicit operator Value*() { return &parse(); }
  explicit operator Value&() { return parse(); }
  explicit operator Value&&() { return std::move(parse()); }

private:
  // Called from the bound function's argument unpacking, inside nanobind's dispatcher,
  // so std::invalid_argument reaches Python as ValueError.
  Value& parse()
  {
    try
    {
      value = boost::uuids::string_generator()(text);
    }
    catch (const std::runtime_error&)
    {
      throw std::invalid_argument("not a UUID string: '" + text + "'");
    }
    return value;
  }
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

namespace tesseract_nb
{
// An Eigen matrix parameter the binding writes into: the caller's numpy array, bound in place
// as a mutable Eigen::Ref (`.ref`).
//
// Converts exactly as Eigen::Ref<Matrix> does, so a list, a tuple or an array of another dtype
// or layout is refused (a write could not reach it). What it adds is the annotation: nanobind
// names a mutable Ref after the shared ndarray caster, whose input name above includes
// `Sequence`; this names the array alone. A second Eigen::Ref specialization cannot do that,
// since nanobind's own is SFINAE-constrained and clang reports the pair ambiguous. Use it for
// every in-place Eigen parameter.
template <typename Matrix>
struct InPlace
{
  Eigen::Ref<Matrix> ref;
};
}  // namespace tesseract_nb

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)
template <typename Matrix>
struct type_caster<tesseract_nb::InPlace<Matrix>>
{
  using RefCaster = make_caster<Eigen::Ref<Matrix>>;

  static constexpr auto Name = RefCaster::MapCaster::NDArrayCaster::StockCaster::Name;
  template <typename T_>
  using Cast = tesseract_nb::InPlace<Matrix>;
  template <typename T_>
  static constexpr bool can_cast()
  {
    return true;
  }

  RefCaster caster;

  bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
  {
    return caster.from_python(src, flags, cleanup);
  }

  // A parameter type only.
  static handle from_cpp(const tesseract_nb::InPlace<Matrix>&, rv_policy, cleanup_list*) noexcept = delete;

  operator tesseract_nb::InPlace<Matrix>() { return { caster.operator Eigen::Ref<Matrix>() }; }
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

// Bind the copy protocol from the C++ copy constructor: `copy.copy(x)` and `copy.deepcopy(x)`
// return a new object of the concrete type. Only for value types whose copy constructor
// copies everything they own (a Poly deep-copies its stored object through clone()), so
// __copy__ and __deepcopy__ give the same result and `memo` has nothing to record.
template <typename Class>
void bind_copy(Class& cls) {
    using T = typename Class::Type;
    cls.def("__copy__", [](const T& self) { return T(self); })
        .def("__deepcopy__", [](const T& self, nb::dict /*memo*/) { return T(self); }, "memo"_a);
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

// A Python override that raises reaches C++ as nb::python_error. Every module links nanobind
// statically (NB_STATIC), so each carries its own typeinfo for it, and on macOS libc++
// compares typeinfo by pointer: another module's dispatcher misses `catch (python_error&)`
// and its default translator turns the error into RuntimeError(what()) (#156). A translator
// registered here runs this module's code, so its catch matches the python_error this
// module's trampolines throw, and restores the original Python exception. Every module that
// defines a trampoline calls this in NB_MODULE. `static`, not `inline`: an inline function's
// lambda is a weak symbol, and dyld coalesces weak symbols across images, so every module
// would register the first-loaded module's copy, whose catch tests that module's typeinfo.
static void register_python_error_translator() {
    nb::register_exception_translator([](const std::exception_ptr& p, void*) {
        try {
            std::rethrow_exception(p);
        } catch (nb::python_error& e) {
            e.restore();
        }
    });
}

// Note: Eigen::Isometry3d is bound as an explicit class in tesseract_common_bindings.cpp
// for SWIG API compatibility (tests expect .matrix() method etc.)
// No type caster is needed since we have explicit bindings.
