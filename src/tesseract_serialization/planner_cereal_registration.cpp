/**
 * @file planner_cereal_registration.cpp
 * @brief Registers every planner profile type with this module's cereal registry
 *
 * Upstream registers these types by platform. On Linux they live in each planner's library, so a
 * type is registered once that library is in the process, and the module links them to make that
 * unconditional (each header's CEREAL_FORCE_DYNAMIC_INIT references the library). On macOS and
 * Windows the libraries carry no registrations and the header registers them in the including
 * translation unit; on Windows that registry is also per DLL, so this has to be in the module that
 * runs the archives.
 */

#include <tesseract/motion_planners/simple/cereal_serialization.h>
#include <tesseract/motion_planners/ompl/cereal_serialization.h>
#include <tesseract/motion_planners/descartes/cereal_serialization.h>
#include <tesseract/motion_planners/trajopt/cereal_serialization.h>
#include <tesseract/motion_planners/trajopt_ifopt/cereal_serialization.h>
#include <tesseract/task_composer/planning/cereal_serialization.h>
#include <tesseract/time_parameterization/isp/cereal_serialization.h>
#include <tesseract/time_parameterization/kdl/cereal_serialization.h>
#include <tesseract/time_parameterization/ruckig/cereal_serialization.h>
#include <tesseract/time_parameterization/totg/cereal_serialization.h>
