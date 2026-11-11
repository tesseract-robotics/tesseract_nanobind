/**
 * @file task_composer_anypoly.cpp
 * @brief tesseract_task_composer: AnyPoly, the type-erased value in a TaskComposerDataStorage
 */

#include "task_composer_bindings.h"

#include <boost/core/demangle.hpp>
#include <cctype>
#include <cstring>
#include <typeindex>

#include <tesseract/collision/types.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/common/any_poly.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/environment/environment.h>
#include <tesseract/task_composer/task_composer_data_storage.h>

namespace tp = tesseract::task_composer;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace tc = tesseract::common;

namespace {

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

}  // namespace

void bind_anypoly(nb::module_& m) {
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
