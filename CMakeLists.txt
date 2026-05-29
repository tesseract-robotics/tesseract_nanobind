cmake_minimum_required(VERSION 3.15)

# Work around jsoncpp CMake config requiring older CMake policy
cmake_policy(SET CMP0167 OLD)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version")

project(tesseract_nanobind LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Python and nanobind
find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

# Add tesseract install path to CMAKE_PREFIX_PATH
# Use absolute path derived from project source dir
get_filename_component(TESSERACT_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ws/install" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH "${TESSERACT_INSTALL_DIR}")
message(STATUS "Added tesseract install dir: ${TESSERACT_INSTALL_DIR}")

# Set RPATH to find tesseract libraries at runtime
# This embeds the full path to ws/install/lib in the built .so files
set(CMAKE_INSTALL_RPATH "${TESSERACT_INSTALL_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  # Also add conda lib for dependencies like boost, tinyxml2
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
  endif()
endif()

# Add conda prefix for dependencies (tinyxml2, console_bridge, etc.)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  link_directories("$ENV{CONDA_PREFIX}/lib")
  if(WIN32 AND IS_DIRECTORY "$ENV{CONDA_PREFIX}/Library")
    list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/Library")
    link_directories("$ENV{CONDA_PREFIX}/Library/lib")
    message(STATUS "Added conda Windows library prefix: $ENV{CONDA_PREFIX}/Library")
  endif()
  # Qt6 requires QT_HOST_PATH on macOS with conda
  set(QT_HOST_PATH "$ENV{CONDA_PREFIX}" CACHE PATH "Qt host path for cross-compilation")
  message(STATUS "Added conda prefix: $ENV{CONDA_PREFIX}")
endif()

# Find tesseract C++ libraries
find_package(tesseract REQUIRED COMPONENTS common)
find_package(tesseract REQUIRED COMPONENTS geometry)
find_package(tesseract REQUIRED COMPONENTS scene_graph)
find_package(tesseract REQUIRED COMPONENTS srdf)
find_package(tesseract REQUIRED COMPONENTS urdf)
find_package(tesseract REQUIRED COMPONENTS collision)
find_package(tesseract REQUIRED COMPONENTS state_solver)
find_package(tesseract REQUIRED COMPONENTS kinematics)
find_package(tesseract REQUIRED COMPONENTS environment)
find_package(tesseract_planning REQUIRED COMPONENTS command_language)
find_package(console_bridge REQUIRED)

# Define reusable function for nanobind extensions
function(add_tesseract_nanobind_extension module_name source_file)
  nanobind_add_module(
    _${module_name}
    STABLE_ABI
    NB_STATIC
    ${source_file}
  )

  # Apply precompiled headers
  target_precompile_headers(_${module_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tesseract_nb.h)

  # Include directories
  target_include_directories(_${module_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Link tesseract libraries (passed as remaining arguments)
  target_link_libraries(_${module_name} PRIVATE ${ARGN})

  # RTTI visibility fix: Export typeinfo symbols so typeid() comparison works
  # across library boundaries (needed for TaskComposer plugin compatibility)
  set_target_properties(_${module_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF)

  # Install the module
  install(TARGETS _${module_name}
          LIBRARY DESTINATION tesseract_robotics/${module_name})
endfunction()

# Add tesseract_common module
add_tesseract_nanobind_extension(
  tesseract_common
  src/tesseract_common/tesseract_common_bindings.cpp
  tesseract::common
  console_bridge::console_bridge
)

# Add tesseract_geometry module
add_tesseract_nanobind_extension(
  tesseract_geometry
  src/tesseract_geometry/tesseract_geometry_bindings.cpp
  tesseract::geometry
  tesseract::common
)

# Add tesseract_scene_graph module
add_tesseract_nanobind_extension(
  tesseract_scene_graph
  src/tesseract_scene_graph/tesseract_scene_graph_bindings.cpp
  tesseract::scene_graph
  tesseract::geometry
  tesseract::common
)

# Add tesseract_srdf module
add_tesseract_nanobind_extension(
  tesseract_srdf
  src/tesseract_srdf/tesseract_srdf_bindings.cpp
  tesseract::srdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_urdf module
add_tesseract_nanobind_extension(
  tesseract_urdf
  src/tesseract_urdf/tesseract_urdf_bindings.cpp
  tesseract::urdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_collision module
add_tesseract_nanobind_extension(
  tesseract_collision
  src/tesseract_collision/tesseract_collision_bindings.cpp
  tesseract::collision
  tesseract::collision_bullet
  tesseract::geometry
  tesseract::common
)

# Add tesseract_state_solver module
add_tesseract_nanobind_extension(
  tesseract_state_solver
  src/tesseract_state_solver/tesseract_state_solver_bindings.cpp
  tesseract::state_solver_kdl
  tesseract::state_solver_ofkt
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_kinematics module
add_tesseract_nanobind_extension(
  tesseract_kinematics
  src/tesseract_kinematics/tesseract_kinematics_bindings.cpp
  tesseract::kinematics
  tesseract::scene_graph
  tesseract::state_solver_kdl
  tesseract::common
)

# Add tesseract_environment module
add_tesseract_nanobind_extension(
  tesseract_environment
  src/tesseract_environment/tesseract_environment_bindings.cpp
  tesseract::environment
  tesseract::kinematics
  tesseract::collision
  tesseract::srdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_command_language module
add_tesseract_nanobind_extension(
  tesseract_command_language
  src/tesseract_command_language/tesseract_command_language_bindings.cpp
  tesseract::command_language
  tesseract::common
)

# Add tesseract_serialization module (XML/binary serialization for root types)
add_tesseract_nanobind_extension(
  tesseract_serialization
  src/tesseract_serialization/tesseract_serialization_bindings.cpp
  tesseract::command_language
  tesseract::environment
  tesseract::scene_graph
  tesseract::common
)

# Find motion planners
find_package(tesseract_planning REQUIRED COMPONENTS motion_planners motion_planners_simple motion_planners_ompl)

# Add tesseract_motion_planners module
add_tesseract_nanobind_extension(
  tesseract_motion_planners
  src/tesseract_motion_planners/tesseract_motion_planners_bindings.cpp
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add tesseract_motion_planners_simple module
add_tesseract_nanobind_extension(
  tesseract_motion_planners_simple
  src/tesseract_motion_planners_simple/tesseract_motion_planners_simple_bindings.cpp
  tesseract::motion_planners_simple
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add tesseract_motion_planners_ompl module
add_tesseract_nanobind_extension(
  tesseract_motion_planners_ompl
  src/tesseract_motion_planners_ompl/tesseract_motion_planners_ompl_bindings.cpp
  tesseract::motion_planners_ompl
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add ompl_base module (SE2, Reeds-Shepp, Dubins state spaces)
# Note: OMPL uses old-style CMake variables, not modern targets
find_package(ompl REQUIRED)
nanobind_add_module(
  _ompl_base
  STABLE_ABI
  NB_STATIC
  src/ompl_base/ompl_base_bindings.cpp
)
target_precompile_headers(_ompl_base PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/tesseract_nb.h)
target_include_directories(_ompl_base PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(_ompl_base PRIVATE ${OMPL_LIBRARIES})
set_target_properties(_ompl_base PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF)
install(TARGETS _ompl_base LIBRARY DESTINATION tesseract_robotics/ompl_base)

# Add tesseract_motion_planners_descartes module (optional - only if descartes component was built)
find_package(tesseract_planning QUIET COMPONENTS motion_planners_descartes)
find_package(descartes_light QUIET)
if(tesseract_planning_motion_planners_descartes_FOUND AND descartes_light_FOUND)
  message(STATUS "tesseract_motion_planners descartes component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_descartes
    src/tesseract_motion_planners_descartes/tesseract_motion_planners_descartes_bindings.cpp
    tesseract::motion_planners_descartes
    tesseract::motion_planners
    tesseract::environment
    tesseract::command_language
    tesseract::collision
    tesseract::common
  )
else()
  message(STATUS "tesseract_motion_planners descartes component not found - skipping bindings")
endif()

# Add tesseract_motion_planners_trajopt module (optional - only if trajopt component was built)
find_package(tesseract_planning QUIET COMPONENTS motion_planners_trajopt)
find_package(trajopt QUIET)
if(tesseract_planning_motion_planners_trajopt_FOUND AND trajopt_FOUND)
  message(STATUS "tesseract_motion_planners trajopt component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_trajopt
    src/tesseract_motion_planners_trajopt/tesseract_motion_planners_trajopt_bindings.cpp
    tesseract::motion_planners_trajopt
    tesseract::motion_planners
    tesseract::environment
    tesseract::command_language
    tesseract::common
    trajopt::trajopt
  )
else()
  message(STATUS "tesseract_motion_planners trajopt component not found - skipping bindings")
endif()

# Add tesseract_motion_planners_trajopt_ifopt module (optional - only if trajopt_ifopt component was built)
find_package(tesseract_planning QUIET COMPONENTS motion_planners_trajopt_ifopt)
find_package(trajopt_ifopt QUIET)
find_package(trajopt_sqp QUIET)
find_package(trajopt_common QUIET)
if(tesseract_planning_motion_planners_trajopt_ifopt_FOUND AND trajopt_ifopt_FOUND AND trajopt_sqp_FOUND)
  message(STATUS "tesseract_motion_planners trajopt_ifopt component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_trajopt_ifopt
    src/tesseract_motion_planners_trajopt_ifopt/tesseract_motion_planners_trajopt_ifopt_bindings.cpp
    tesseract::motion_planners_trajopt_ifopt
    tesseract::motion_planners
    tesseract::environment
    tesseract::command_language
    tesseract::common
    trajopt::trajopt_ifopt
    trajopt::trajopt_sqp
  )
else()
  message(STATUS "tesseract_motion_planners trajopt_ifopt component not found - skipping bindings")
endif()

# Add trajopt_ifopt module (variables, constraints, costs for IFOPT-based trajectory optimization)
find_package(trajopt_common QUIET)
if(trajopt_ifopt_FOUND AND trajopt_common_FOUND)
  message(STATUS "trajopt_ifopt found - building bindings for variables/constraints/costs")
  add_tesseract_nanobind_extension(
    trajopt_ifopt
    src/trajopt_ifopt/trajopt_ifopt_bindings.cpp
    trajopt::trajopt_ifopt
    trajopt::trajopt_common
    tesseract::kinematics
    tesseract::environment
    tesseract::collision
    tesseract::common
  )
else()
  message(STATUS "trajopt_ifopt/trajopt_common not found - skipping bindings")
endif()

# Add trajopt_sqp module (SQP solver with stepSQPSolver for real-time planning)
if(trajopt_sqp_FOUND)
  message(STATUS "trajopt_sqp found - building SQP solver bindings")
  add_tesseract_nanobind_extension(
    trajopt_sqp
    src/trajopt_sqp/trajopt_sqp_bindings.cpp
    trajopt::trajopt_sqp
    trajopt::trajopt_ifopt
    tesseract::common
  )
else()
  message(STATUS "trajopt_sqp not found - skipping bindings")
endif()

# Find and add tesseract_time_parameterization
find_package(tesseract_planning REQUIRED COMPONENTS time_parameterization time_parameterization_isp time_parameterization_totg time_parameterization_kdl)

add_tesseract_nanobind_extension(
  tesseract_time_parameterization
  src/tesseract_time_parameterization/tesseract_time_parameterization_bindings.cpp
  tesseract::time_parameterization_isp
  tesseract::time_parameterization_totg
  tesseract::time_parameterization_kdl
  tesseract::time_parameterization
  tesseract::command_language
  tesseract::common
)

# Find and add tesseract_task_composer
find_package(tesseract_planning REQUIRED COMPONENTS task_composer task_composer_taskflow task_composer_planning)

add_tesseract_nanobind_extension(
  tesseract_task_composer
  src/tesseract_task_composer/tesseract_task_composer_bindings.cpp
  tesseract::task_composer
  tesseract::task_composer_taskflow
  tesseract::environment
  tesseract::command_language
  tesseract::collision
  tesseract::common
)

add_tesseract_nanobind_extension(
  tesseract_task_composer_planning
  src/tesseract_task_composer_planning/tesseract_task_composer_planning_bindings.cpp
  tesseract::task_composer_planning_nodes
  tesseract::task_composer
  tesseract::motion_planners_trajopt
  tesseract::collision
  tesseract::command_language
  tesseract::common
)

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Python: ${Python_VERSION}")
message(STATUS "nanobind: ${nanobind_VERSION}")
message(STATUS "===============================================")

# ============== Install Bundled Data Files ==============
# These are installed into the wheel so env vars can use relative paths
# Support both:
#   - Local dev: ws/src/tesseract/... (flat repo layout)
#   - CI layout: ../tesseract/... (repo cloned to ws/src/tesseract_nanobind)

set(TESSERACT_SUPPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ws/src/tesseract/tesseract_support")
if(NOT EXISTS "${TESSERACT_SUPPORT_SRC}")
  # CI layout: repo at ws/src/tesseract_nanobind, deps at ws/src/tesseract
  set(TESSERACT_SUPPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../tesseract/tesseract_support")
endif()

set(TASK_COMPOSER_CFG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ws/src/tesseract_planning/tesseract_task_composer/config")
if(NOT EXISTS "${TASK_COMPOSER_CFG_SRC}")
  # CI layout
  set(TASK_COMPOSER_CFG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../tesseract_planning/tesseract_task_composer/config")
endif()

if(EXISTS "${TESSERACT_SUPPORT_SRC}")
  message(STATUS "Installing tesseract_support from: ${TESSERACT_SUPPORT_SRC}")
  install(DIRECTORY "${TESSERACT_SUPPORT_SRC}/"
    DESTINATION tesseract_robotics/data/tesseract_support
    FILES_MATCHING
    PATTERN "*.urdf" PATTERN "*.srdf" PATTERN "*.yaml" PATTERN "*.xacro"
    PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae" PATTERN "*.ply" PATTERN "*.obj" PATTERN "*.csv"
    PATTERN "*.xml" PATTERN "*.png" PATTERN "*.jpg" PATTERN "*.jpeg"
    PATTERN ".git" EXCLUDE PATTERN "__pycache__" EXCLUDE)
else()
  message(WARNING "tesseract_support not found at ${TESSERACT_SUPPORT_SRC}")
endif()

if(EXISTS "${TASK_COMPOSER_CFG_SRC}")
  message(STATUS "Installing task_composer config from: ${TASK_COMPOSER_CFG_SRC}")
  install(DIRECTORY "${TASK_COMPOSER_CFG_SRC}/"
    DESTINATION tesseract_robotics/data/task_composer_config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.schema")
else()
  message(WARNING "task_composer config not found at ${TASK_COMPOSER_CFG_SRC}")
endif()
