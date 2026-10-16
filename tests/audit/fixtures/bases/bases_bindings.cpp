// Audit contract fixture TU (#256 T5): binds the template instance Planner<double> as PlannerD.
#include <nanobind/nanobind.h>
#include <tesseract/bases/runners.h>

namespace nb = nanobind;
namespace tb = tesseract::bases;

NB_MODULE(_bases, m) { nb::class_<tb::Planner<double>>(m, "PlannerD"); }
