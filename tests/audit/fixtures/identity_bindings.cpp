// Audit contract fixture TU (#256 T3): `bound_classes` reads each nb::class_ / nb::enum_ below.
#include <nanobind/nanobind.h>
#include <tesseract/identity/widgets.h>

namespace nb = nanobind;
namespace ti = tesseract::identity;

NB_MODULE(_identity, m)
{
  auto widget = nb::class_<ti::LongWidgetName>(m, "Widget");
  nb::class_<ti::LongWidgetName::Part>(widget, "Part");
  nb::class_<ti::Gadget<double>>(m, "GadgetD");
  nb::enum_<ti::Color>(m, "Color");
  nb::class_<ti::Derived, ti::LongWidgetName>(m, "Derived");
}
