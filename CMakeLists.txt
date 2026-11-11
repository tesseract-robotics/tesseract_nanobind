cmake_minimum_required(VERSION 3.15)

# Work around jsoncpp CMake config requiring older CMake policy
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version")

project(tesseract_nanobind LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# VISIBILITY_INLINES_HIDDEN OFF (below) only stops CMake adding the flag; conda's clang
# activation still injects it through CXXFLAGS. Hidden inlines give this .so its own
# cereal registry singleton under Mach-O's two-level namespace, so polymorphic types
# registered inside libtesseract_* (upstream tesseract#1305) read as unregistered here.
if(APPLE)
  string(REPLACE "-fvisibility-inlines-hidden" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

# Find Python and nanobind
find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

# The tesseract C++ prefix: TESSERACT_CPP_PREFIX, the colcon install of the source-built stack
# (scripts/native_stack.py; the wheel workflow and `pixi run install` both point it there).
# find_package() resolves the stack via the CMAKE_PREFIX_PATH entry added below, and the data the
# wheel carries is copied from its share/.
if(NOT "$ENV{TESSERACT_CPP_PREFIX}" STREQUAL "")
  set(TESSERACT_CPP_PREFIX "$ENV{TESSERACT_CPP_PREFIX}")
endif()

# Embed an RPATH to <cpp-prefix>/lib so the built extensions locate the tesseract libs
# at runtime for an editable dev install (`pixi run install`). The portable wheel build
# later rewrites these RPATHs to $ORIGIN and bundles the libs into the wheel
# (see scripts/build_{linux,macos}_wheel.sh).
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
if(DEFINED TESSERACT_CPP_PREFIX)
  set(CMAKE_INSTALL_RPATH "${TESSERACT_CPP_PREFIX}/lib")
endif()
# Development builds (scripts/native_stack.py exec) also find the vcpkg libraries the stack links.
if(NOT "$ENV{TESSERACT_DEV_RPATH}" STREQUAL "")
  list(APPEND CMAKE_INSTALL_RPATH "$ENV{TESSERACT_DEV_RPATH}")
endif()
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

# Add the C++ prefix for dependencies (tinyxml2, etc.)
if(DEFINED TESSERACT_CPP_PREFIX)
  list(APPEND CMAKE_PREFIX_PATH "${TESSERACT_CPP_PREFIX}")
  link_directories("${TESSERACT_CPP_PREFIX}/lib")
  if(WIN32 AND IS_DIRECTORY "${TESSERACT_CPP_PREFIX}/Library")
    list(APPEND CMAKE_PREFIX_PATH "${TESSERACT_CPP_PREFIX}/Library")
    link_directories("${TESSERACT_CPP_PREFIX}/Library/lib")
    message(STATUS "Added conda Windows library prefix: ${TESSERACT_CPP_PREFIX}/Library")
  endif()
  # Qt6 requires QT_HOST_PATH on macOS with conda
  set(QT_HOST_PATH "${TESSERACT_CPP_PREFIX}" CACHE PATH "Qt host path for cross-compilation")
  message(STATUS "Added tesseract C++ prefix: ${TESSERACT_CPP_PREFIX}")
endif()

# Find tesseract C++ libraries
find_package(tesseract REQUIRED COMPONENTS common)
find_package(tesseract REQUIRED COMPONENTS geometry)
find_package(tesseract REQUIRED COMPONENTS scene_graph)
find_package(tesseract REQUIRED COMPONENTS srdf)
find_package(tesseract REQUIRED COMPONENTS urdf)
find_package(tesseract REQUIRED COMPONENTS collision)
find_package(tesseract REQUIRED COMPONENTS collision_vhacd)
find_package(tesseract REQUIRED COMPONENTS state_solver)
find_package(tesseract REQUIRED COMPONENTS kinematics)
find_package(tesseract REQUIRED COMPONENTS environment)
find_package(tesseract_planning REQUIRED COMPONENTS command_language)

# Plugin factory libraries are loaded at runtime by boost_plugin_loader and are mostly not exported
# as CMake targets. Create imported targets for them so the extension modules can link them: that
# makes them ordinary dependencies, so auditwheel/delocate/delvewheel bundle them, and the module
# init registers each one through its anchor symbol (addSymbolLibraryToSearchLibrariesEnv).
macro(tesseract_nb_find_plugin_target lib_name search_target)
  get_target_property(_plugin_search_location ${search_target} LOCATION)
  if(NOT WIN32)
    get_filename_component(_plugin_search_dir ${_plugin_search_location} DIRECTORY)
  else()
    # LOCATION is the .dll in bin/; the import library is in lib/
    get_filename_component(_plugin_search_dir1 ${_plugin_search_location} DIRECTORY)
    get_filename_component(_plugin_search_dir2 ${_plugin_search_dir1} DIRECTORY)
    set(_plugin_search_dir "${_plugin_search_dir2}/lib")
  endif()
  unset(_plugin_lib_location)
  find_library(_plugin_lib_location ${lib_name} HINTS ${_plugin_search_dir} NO_CACHE)
  if(NOT _plugin_lib_location)
    message(FATAL_ERROR "Could not find plugin library ${lib_name}")
  endif()
  add_library(tesseract::${lib_name} SHARED IMPORTED)
  if(WIN32)
    set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_IMPLIB ${_plugin_lib_location})
  else()
    set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_LOCATION ${_plugin_lib_location})
  endif()
endmacro()

tesseract_nb_find_plugin_target(tesseract_collision_bullet_factories tesseract::collision)
tesseract_nb_find_plugin_target(tesseract_collision_fcl_factories tesseract::collision)
tesseract_nb_find_plugin_target(tesseract_kinematics_kdl_factories tesseract::kinematics)
tesseract_nb_find_plugin_target(tesseract_kinematics_opw_factory tesseract::kinematics)
tesseract_nb_find_plugin_target(tesseract_kinematics_ur_factory tesseract::kinematics)

# Modules whose package keeps Python code around the compiled module (constants, lazy re-exports,
# profile-override guards): the extension is installed as tesseract_robotics/<name>/_<name> and the
# package __init__ re-exports it. Every other module is installed directly as tesseract_robotics/<name>.
set(TESSERACT_NB_WRAPPED_MODULES
  common
  scene_graph
  motion_planners
  motion_planners_simple
  motion_planners_descartes
  motion_planners_trajopt_ifopt
  trajopt_ifopt
)

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
  # PCH makes these calls uncacheable unless ccache's macro/time checks are relaxed.
  # Keep those checks and cache ordinary compilation; retain PCH without ccache.
  if(CMAKE_CXX_COMPILER_LAUNCHER STREQUAL "ccache")
    set_property(TARGET _${module_name} PROPERTY DISABLE_PRECOMPILE_HEADERS ON)
  endif()

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
  if(module_name IN_LIST TESSERACT_NB_WRAPPED_MODULES)
    # _<name> in the package directory: NB_MODULE(_<name>, m)
    install(TARGETS _${module_name}
            LIBRARY DESTINATION tesseract_robotics/${module_name})
  else()
    # <name> directly in tesseract_robotics: NB_MODULE(<name>, m)
    set_target_properties(_${module_name} PROPERTIES OUTPUT_NAME ${module_name})
    install(TARGETS _${module_name}
            LIBRARY DESTINATION tesseract_robotics)
  endif()
endfunction()

# Add tesseract_common module
add_tesseract_nanobind_extension(
  common
  src/bindings/common.cpp
  tesseract::common
)

# Add tesseract_geometry module
add_tesseract_nanobind_extension(
  geometry
  src/bindings/geometry.cpp
  tesseract::geometry
  tesseract::common
)

# Add tesseract_scene_graph module
add_tesseract_nanobind_extension(
  scene_graph
  src/bindings/scene_graph.cpp
  tesseract::scene_graph
  tesseract::geometry
  tesseract::common
)

# Add tesseract_srdf module
add_tesseract_nanobind_extension(
  srdf
  src/bindings/srdf.cpp
  tesseract::srdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_urdf module
add_tesseract_nanobind_extension(
  urdf
  src/bindings/urdf.cpp
  tesseract::urdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_collision module
add_tesseract_nanobind_extension(
  collision
  src/bindings/collision.cpp
  tesseract::collision
  tesseract::collision_bullet
  tesseract::collision_vhacd_convex_decomposition
  tesseract::geometry
  tesseract::common
  tesseract::tesseract_collision_bullet_factories
  tesseract::tesseract_collision_fcl_factories
)

# Add tesseract_state_solver module
add_tesseract_nanobind_extension(
  state_solver
  src/bindings/state_solver.cpp
  tesseract::state_solver_kdl
  tesseract::state_solver_ofkt
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_kinematics module
add_tesseract_nanobind_extension(
  kinematics
  src/bindings/kinematics.cpp
  tesseract::kinematics
  tesseract::scene_graph
  tesseract::state_solver_kdl
  tesseract::common
  tesseract::kinematics_factories
  tesseract::tesseract_kinematics_kdl_factories
  tesseract::tesseract_kinematics_opw_factory
  tesseract::tesseract_kinematics_ur_factory
)

# Add tesseract_environment module
add_tesseract_nanobind_extension(
  environment
  src/bindings/environment.cpp
  tesseract::environment
  tesseract::kinematics
  tesseract::collision
  tesseract::srdf
  tesseract::scene_graph
  tesseract::common
)

# Add tesseract_command_language module
add_tesseract_nanobind_extension(
  command_language
  src/bindings/command_language.cpp
  tesseract::command_language
  tesseract::common
)

# Add tesseract_serialization module (XML/binary serialization for root types)
add_tesseract_nanobind_extension(
  serialization
  src/bindings/serialization.cpp
  tesseract::command_language
  tesseract::environment
  tesseract::scene_graph
  tesseract::common
  # TaskComposerLog; the target is imported by find_package(tesseract_planning) below
  tesseract::task_composer
)

# Find motion planners
find_package(tesseract_planning REQUIRED COMPONENTS motion_planners motion_planners_simple motion_planners_ompl)

# Add tesseract_motion_planners module
add_tesseract_nanobind_extension(
  motion_planners
  src/bindings/motion_planners.cpp
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add tesseract_motion_planners_simple module
add_tesseract_nanobind_extension(
  motion_planners_simple
  src/bindings/motion_planners_simple.cpp
  tesseract::motion_planners_simple
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add tesseract_motion_planners_ompl module
add_tesseract_nanobind_extension(
  motion_planners_ompl
  src/bindings/motion_planners_ompl.cpp
  tesseract::motion_planners_ompl
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
)

# Add tesseract_motion_planners_descartes module
find_package(tesseract_planning REQUIRED COMPONENTS motion_planners_descartes)
find_package(descartes_light REQUIRED)
add_tesseract_nanobind_extension(
  motion_planners_descartes
  src/bindings/motion_planners_descartes.cpp
  tesseract::motion_planners_descartes
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::collision
  tesseract::common
)

# Add tesseract_motion_planners_trajopt module
find_package(tesseract_planning REQUIRED COMPONENTS motion_planners_trajopt)
find_package(trajopt REQUIRED)
add_tesseract_nanobind_extension(
  motion_planners_trajopt
  src/bindings/motion_planners_trajopt.cpp
  tesseract::motion_planners_trajopt
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
  trajopt::trajopt
)

# Add tesseract_motion_planners_trajopt_ifopt module
find_package(tesseract_planning REQUIRED COMPONENTS motion_planners_trajopt_ifopt)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_common REQUIRED)
add_tesseract_nanobind_extension(
  motion_planners_trajopt_ifopt
  src/bindings/motion_planners_trajopt_ifopt.cpp
  tesseract::motion_planners_trajopt_ifopt
  tesseract::motion_planners
  tesseract::environment
  tesseract::command_language
  tesseract::common
  trajopt::trajopt_ifopt
  trajopt::trajopt_sqp
)

# Add trajopt_ifopt module (variables, constraints, costs for IFOPT-based trajectory optimization)
add_tesseract_nanobind_extension(
  trajopt_ifopt
  src/bindings/trajopt_ifopt.cpp
  trajopt::trajopt_ifopt
  trajopt::trajopt_common
  tesseract::kinematics
  tesseract::environment
  tesseract::collision
  tesseract::common
)

# Add trajopt_sqp module (SQP solver with stepSQPSolver for real-time planning)
add_tesseract_nanobind_extension(
  trajopt_sqp
  src/bindings/trajopt_sqp.cpp
  trajopt::trajopt_sqp
  trajopt::trajopt_ifopt
  tesseract::common
)
# PIQPSettings' operator== lives in the header-only tesseract/motion_planners/piqp/settings_utils.h
# (#393): its include path only, no tesseract_planning library linked.
target_include_directories(_trajopt_sqp PRIVATE
  $<TARGET_PROPERTY:tesseract::motion_planners,INTERFACE_INCLUDE_DIRECTORIES>)

# Find and add tesseract_time_parameterization
find_package(tesseract_planning REQUIRED COMPONENTS time_parameterization time_parameterization_isp time_parameterization_totg time_parameterization_kdl time_parameterization_ruckig)

add_tesseract_nanobind_extension(
  time_parameterization
  src/bindings/time_parameterization.cpp
  tesseract::time_parameterization_isp
  tesseract::time_parameterization_totg
  tesseract::time_parameterization_kdl
  tesseract::time_parameterization_ruckig
  tesseract::time_parameterization
  tesseract::command_language
  tesseract::common
)

# Find and add tesseract_task_composer
find_package(tesseract_planning REQUIRED COMPONENTS task_composer task_composer_taskflow task_composer_planning)

tesseract_nb_find_plugin_target(tesseract_task_composer_factories tesseract::task_composer)
tesseract_nb_find_plugin_target(tesseract_task_composer_planning_factories tesseract::task_composer)
tesseract_nb_find_plugin_target(tesseract_task_composer_taskflow_factories tesseract::task_composer)

add_tesseract_nanobind_extension(
  task_composer
  src/bindings/task_composer.cpp
  tesseract::task_composer
  tesseract::task_composer_taskflow
  tesseract::environment
  tesseract::command_language
  tesseract::collision
  tesseract::common
  tesseract::tesseract_task_composer_factories
  tesseract::tesseract_task_composer_planning_factories
  tesseract::tesseract_task_composer_taskflow_factories
)
# Pins the plugin libraries the task composer loads (macOS, Windows).
target_sources(_task_composer PRIVATE src/bindings/task_composer_plugin_pinning.cpp)

add_tesseract_nanobind_extension(
  task_composer_planning
  src/bindings/task_composer_planning.cpp
  tesseract::task_composer_planning_nodes
  tesseract::task_composer
  tesseract::motion_planners_trajopt
  tesseract::collision
  tesseract::command_language
  tesseract::common
)

# Registers the planners' cereal types with the serialization module: it has to see every planner,
# and their targets are imported above.
target_sources(_serialization PRIVATE src/bindings/serialization_planner_cereal_registration.cpp)
target_link_libraries(_serialization PRIVATE
  tesseract::motion_planners_simple
  tesseract::motion_planners_ompl
  tesseract::motion_planners_descartes
  tesseract::motion_planners_trajopt
  tesseract::motion_planners_trajopt_ifopt
  tesseract::task_composer_planning_nodes
  tesseract::time_parameterization_isp
  tesseract::time_parameterization_kdl
  tesseract::time_parameterization_ruckig
  tesseract::time_parameterization_totg
)
# MSVC: the registrations and archives exceed the default COFF section limit (C1128).
target_compile_options(_serialization PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Python: ${Python_VERSION}")
message(STATUS "nanobind: ${nanobind_VERSION}")
message(STATUS "===============================================")

# ============== Install Bundled Data Files ==============
# The task composer config is installed into the wheel (get_task_composer_config_path()). It comes
# from <cpp-prefix>/share, where the tesseract install puts it:
#   task_composer config  -> share/tesseract_planning/task_composer/config
# tesseract's support data (robot models, meshes) is not: it is its own wheel,
# packaging/tesseract-robotics-nanobind-support (the `[support]` extra).
if(DEFINED TESSERACT_CPP_PREFIX)
  set(TASK_COMPOSER_CFG_SRC "${TESSERACT_CPP_PREFIX}/share/tesseract_planning/task_composer/config")
  if(WIN32 AND NOT EXISTS "${TASK_COMPOSER_CFG_SRC}")
    # conda on Windows installs data under the Library/ prefix
    set(TASK_COMPOSER_CFG_SRC "${TESSERACT_CPP_PREFIX}/Library/share/tesseract_planning/task_composer/config")
  endif()
endif()

if(EXISTS "${TASK_COMPOSER_CFG_SRC}")
  message(STATUS "Installing task_composer config from: ${TASK_COMPOSER_CFG_SRC}")
  install(DIRECTORY "${TASK_COMPOSER_CFG_SRC}/"
    DESTINATION tesseract_robotics/data/task_composer_config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.schema")
else()
  message(WARNING "task_composer config not found at ${TASK_COMPOSER_CFG_SRC}")
endif()
