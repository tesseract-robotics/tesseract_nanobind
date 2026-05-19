/**
 * @file tesseract_serialization_bindings.cpp
 * @brief Python bindings for tesseract serialization via Cereal (root types only)
 *
 * Exposes XML/binary serialization for high-level types. Cereal
 * recursively handles all nested types - only root entry points needed.
 */

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <tesseract_common/serialization.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/cereal_serialization.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/cereal_serialization.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/cereal_serialization.h>

// CEREAL_REGISTER_TYPE expansion needs all archive headers visible before the
// macro fires (so cereal can bind the type to each archive at TU init).
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

namespace nb = nanobind;
using namespace tesseract_common;
using namespace tesseract_planning;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;

// Cereal polymorphic-type registry on Windows MSVC is per-DLL — class-template
// statics (StaticObject<InputBindingMap<Archive>>, OutputBindingMap, ...) are
// instantiated independently in each DLL with no automatic dllexport/dllimport
// to share them. Upstream tesseract_command_language.dll registers all its
// polymorphic types into ITS OWN registry; our .pyd's registry stays empty
// and serialization fails at runtime with "Trying to save an unregistered
// polymorphic type". Linux and macOS skirt this because their linkers dedupe
// the template statics across shared libraries.
//
// CEREAL_FORCE_DYNAMIC_INIT alone is insufficient — it forces upstream's
// registration TU to survive in tesseract_command_language.dll, but doesn't
// bridge the registry gap on the consumer side. To populate our .pyd's
// registry we have to instantiate the registrar templates in OUR translation
// unit by re-issuing the registration macros here. These mirror upstream
// tesseract_command_language/src/cereal_serialization.cpp verbatim.
//
// Linux/macOS unaffected: cereal's polymorphic registry tolerates
// re-registration of the same {type, archive} pair (no-op on duplicates).
CEREAL_REGISTER_TYPE(tesseract_planning::CartesianWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::JointWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::StateWaypointPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::MoveInstructionPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::CartesianWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::JointWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::StateWaypoint)
CEREAL_REGISTER_TYPE(tesseract_planning::MoveInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::CompositeInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::CompositeInstructionAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_planning::SetAnalogInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::SetDigitalInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::SetToolInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::TimerInstruction)
CEREAL_REGISTER_TYPE(tesseract_planning::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::CartesianWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::JointWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::WaypointInterface, tesseract_planning::StateWaypointPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::MoveInstructionPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::CartesianWaypointInterface,
                                     tesseract_planning::CartesianWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::JointWaypointInterface, tesseract_planning::JointWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::StateWaypointInterface, tesseract_planning::StateWaypoint)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::MoveInstructionInterface, tesseract_planning::MoveInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::CompositeInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::SetAnalogInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface,
                                     tesseract_planning::SetDigitalInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::SetToolInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::TimerInstruction)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_planning::InstructionInterface, tesseract_planning::WaitInstruction)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_planning::CompositeInstructionAnyPoly)

// Belt-and-suspenders: also keep the upstream TU anchored. Harmless if the
// TU is already alive; the consumer-side registrations above are the actual
// fix.
CEREAL_FORCE_DYNAMIC_INIT(tesseract_command_language_cereal)

NB_MODULE(tesseract_serialization, m)
{
    m.doc() = "Tesseract serialization bindings (XML/binary via Cereal)";

    // ============================================================
    // CompositeInstruction - Motion Programs
    // ============================================================

    m.def("composite_instruction_to_xml",
        [](const CompositeInstruction& ci) {
            return Serialization::toArchiveStringXML(ci);
        },
        nb::arg("instruction"),
        "Serialize CompositeInstruction to XML string. Recursively captures all "
        "waypoints, instructions, profiles, and nested composites.");

    m.def("composite_instruction_from_xml",
        [](const std::string& xml) {
            return Serialization::fromArchiveStringXML<CompositeInstruction>(xml);
        },
        nb::arg("xml"),
        "Deserialize CompositeInstruction from XML string.");

    m.def("composite_instruction_to_file",
        [](const CompositeInstruction& ci, const std::string& path) {
            return Serialization::toArchiveFileXML(ci, path);
        },
        nb::arg("instruction"), nb::arg("path"),
        "Save CompositeInstruction to XML file.");

    m.def("composite_instruction_from_file",
        [](const std::string& path) {
            return Serialization::fromArchiveFileXML<CompositeInstruction>(path);
        },
        nb::arg("path"),
        "Load CompositeInstruction from XML file.");

    m.def("composite_instruction_to_binary",
        [](const CompositeInstruction& ci) {
            return Serialization::toArchiveBinaryData(ci);
        },
        nb::arg("instruction"),
        "Serialize CompositeInstruction to binary data (faster, smaller than XML).");

    m.def("composite_instruction_from_binary",
        [](const std::vector<uint8_t>& data) {
            return Serialization::fromArchiveBinaryData<CompositeInstruction>(data);
        },
        nb::arg("data"),
        "Deserialize CompositeInstruction from binary data.");

    // ============================================================
    // Environment - Full Scene State
    // ============================================================

    m.def("environment_to_xml",
        [](const Environment& env) {
            return Serialization::toArchiveStringXML(env);
        },
        nb::arg("environment"),
        "Serialize Environment to XML string. Includes command history - "
        "on deserialize, commands are replayed to rebuild scene graph.");

    m.def("environment_from_xml",
        [](const std::string& xml) {
            return Serialization::fromArchiveStringXML<std::shared_ptr<Environment>>(xml);
        },
        nb::arg("xml"),
        "Deserialize Environment from XML string. Returns shared_ptr.");

    m.def("environment_to_file",
        [](const Environment& env, const std::string& path) {
            return Serialization::toArchiveFileXML(env, path);
        },
        nb::arg("environment"), nb::arg("path"),
        "Save Environment to XML file.");

    m.def("environment_from_file",
        [](const std::string& path) {
            return Serialization::fromArchiveFileXML<std::shared_ptr<Environment>>(path);
        },
        nb::arg("path"),
        "Load Environment from XML file. Returns shared_ptr.");

    m.def("environment_to_binary",
        [](const Environment& env) {
            return Serialization::toArchiveBinaryData(env);
        },
        nb::arg("environment"),
        "Serialize Environment to binary data.");

    m.def("environment_from_binary",
        [](const std::vector<uint8_t>& data) {
            return Serialization::fromArchiveBinaryData<std::shared_ptr<Environment>>(data);
        },
        nb::arg("data"),
        "Deserialize Environment from binary data. Returns shared_ptr.");

    // ============================================================
    // SceneState - State Snapshot
    // ============================================================

    m.def("scene_state_to_xml",
        [](const SceneState& state) {
            return Serialization::toArchiveStringXML(state);
        },
        nb::arg("state"),
        "Serialize SceneState to XML string. Captures joint positions and link transforms.");

    m.def("scene_state_from_xml",
        [](const std::string& xml) {
            return Serialization::fromArchiveStringXML<SceneState>(xml);
        },
        nb::arg("xml"),
        "Deserialize SceneState from XML string.");

    m.def("scene_state_to_file",
        [](const SceneState& state, const std::string& path) {
            return Serialization::toArchiveFileXML(state, path);
        },
        nb::arg("state"), nb::arg("path"),
        "Save SceneState to XML file.");

    m.def("scene_state_from_file",
        [](const std::string& path) {
            return Serialization::fromArchiveFileXML<SceneState>(path);
        },
        nb::arg("path"),
        "Load SceneState from XML file.");

    m.def("scene_state_to_binary",
        [](const SceneState& state) {
            return Serialization::toArchiveBinaryData(state);
        },
        nb::arg("state"),
        "Serialize SceneState to binary data.");

    m.def("scene_state_from_binary",
        [](const std::vector<uint8_t>& data) {
            return Serialization::fromArchiveBinaryData<SceneState>(data);
        },
        nb::arg("data"),
        "Deserialize SceneState from binary data.");
}
