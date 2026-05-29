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
#include <tesseract/common/serialization.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/cereal_serialization.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/cereal_serialization.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/scene_graph/cereal_serialization.h>

// CEREAL_REGISTER_TYPE expansion needs all archive headers visible before the
// macro fires (so cereal can bind the type to each archive at TU init).
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

namespace nb = nanobind;
using namespace tesseract::common;
using namespace tesseract::command_language;
using namespace tesseract::environment;
using namespace tesseract::scene_graph;

// In tesseract 0.35, upstream's <tesseract/command_language/cereal_serialization.h>
// transitively includes cereal_serialization_impl.hpp, which issues all the
// CEREAL_REGISTER_TYPE / CEREAL_REGISTER_POLYMORPHIC_RELATION macros for the
// command_language polymorphic types. Every consumer TU that includes the .h
// (such as this binding) gets the registrations automatically — including
// Windows .pyd consumers where cereal's registry is per-DLL. The consumer-side
// hand-rolled mirror that 0.34 needed is now redundant; re-issuing the macros
// triggers "redefinition of binding_name<...>" compile errors.

NB_MODULE(_tesseract_serialization, m)
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
