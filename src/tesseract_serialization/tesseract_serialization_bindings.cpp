/**
 * @file tesseract_serialization_bindings.cpp
 * @brief Python bindings for tesseract Boost.Serialization (root types only)
 *
 * Exposes XML/binary serialization for high-level types. Boost.Serialization
 * recursively handles all nested types - only root entry points needed.
 */

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <boost/serialization/shared_ptr.hpp>

#include <tesseract_common/serialization.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_environment/environment.h>
#include <tesseract_scene_graph/scene_state.h>

namespace nb = nanobind;
using namespace tesseract_common;
using namespace tesseract_planning;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;

NB_MODULE(_tesseract_serialization, m)
{
    m.doc() = "Tesseract serialization bindings (XML/binary via Boost.Serialization)";

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
