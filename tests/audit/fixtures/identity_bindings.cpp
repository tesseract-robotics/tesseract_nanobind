// Audit contract fixture TU (#256 T3): `bound_classes` reads each nb::class_ / nb::enum_ /
// nb::exception below.
#include <nanobind/nanobind.h>
#include <tesseract/identity/widgets.h>

#include <stdexcept>

namespace nb = nanobind;
namespace ti = tesseract::identity;

namespace
{
// Declared in the binding TU, as the real bindings declare theirs; registered under a prefixed name.
struct WidgetError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};
}  // namespace

NB_MODULE(_identity, m)
{
  auto widget = nb::class_<ti::LongWidgetName>(m, "Gizmo");
  nb::class_<ti::LongWidgetName::Part>(widget, "Part");
  nb::class_<ti::Gadget<double>>(m, "GadgetD");
  nb::enum_<ti::Color>(m, "Color");
  nb::class_<ti::Derived, ti::LongWidgetName>(m, "Derived");
  nb::exception<WidgetError>(m, "GizmoWidgetError", PyExc_ValueError);
}
