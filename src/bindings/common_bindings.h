#pragma once

// The tesseract_common module's translation units, one concern each; NB_MODULE(_common) in
// common.cpp calls them in registration order.

#include "tesseract_nb.h"

// Isometry3d, Translation3d, Quaterniond, AngleAxisd, Hyperplane3d, ParametrizedLine3d and the
// batched rigid-transform readouts (common_eigen.cpp).
void bind_eigen_geometry(nb::module_& m);

// spdlog-backed tesseract logging: Logger, the sinks, log records and handlers (common_logging.cpp).
void bind_logging(nb::module_& m);
