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
#include <nanobind/eigen/dense.h>
#include <nanobind/eigen/sparse.h>

// Namespace aliases
namespace nb = nanobind;
using namespace nb::literals;

namespace tesseract_nb {

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
	return nb::module_::import_(module_name);
}

inline void import_modules_in_context(const nb::module_& current_module,
									  std::initializer_list<const char*> module_names)
{
	for (const char* module_name : module_names)
		import_module_in_context(current_module, module_name);
}

}  // namespace tesseract_nb

// Note: Eigen::Isometry3d is bound as an explicit class in tesseract_common_bindings.cpp
// for SWIG API compatibility (tests expect .matrix() method etc.)
// No type caster is needed since we have explicit bindings.
