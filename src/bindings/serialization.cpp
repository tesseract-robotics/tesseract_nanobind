/**
 * @file serialization.cpp
 * @brief Python bindings for tesseract serialization via Cereal (root types only)
 *
 * Exposes XML/binary serialization for high-level types. Cereal
 * recursively handles all nested types - only root entry points needed.
 */

#include <cerrno>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <tesseract/common/serialization.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/cereal_serialization.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/cereal_serialization.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/scene_graph/cereal_serialization.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_log.h>
#include <tesseract/task_composer/cereal_serialization.h>

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
using tesseract::task_composer::TaskComposerLog;

// In tesseract 0.35, upstream's <tesseract/command_language/cereal_serialization.h>
// transitively includes cereal_serialization_impl.hpp, which issues all the
// CEREAL_REGISTER_TYPE / CEREAL_REGISTER_POLYMORPHIC_RELATION macros for the
// command_language polymorphic types. Every consumer TU that includes the .h
// (such as this binding) gets the registrations automatically — including
// Windows .pyd consumers where cereal's registry is per-DLL. The consumer-side
// hand-rolled mirror that 0.34 needed is now redundant; re-issuing the macros
// triggers "redefinition of binding_name<...>" compile errors.

namespace {

// File I/O is the binding's own, for every type alike: upstream's toArchiveFileXML appends a
// suffix to a path without one and ignores a file that fails to open (an ofstream does not
// throw), and fromArchiveFileXML only asserts the open. These write and read exactly `path` and
// report a failure as OSError, with the errno of the failing call. They run without the GIL;
// the error is raised once it is back.

// 0, or the errno of the failure.
int write_text(const std::filesystem::path& path, const std::string& text) {
    errno = 0;
    std::ofstream out(path, std::ios::binary);
    if (out) out << text;
    if (out) out.close();
    return out ? 0 : (errno != 0 ? errno : EIO);
}

int read_text(const std::filesystem::path& path, std::string& text) {
    errno = 0;
    std::ifstream in(path, std::ios::binary);
    if (!in) return errno != 0 ? errno : EIO;
    std::ostringstream buffer;
    buffer << in.rdbuf();
    text = buffer.str();
    return 0;
}

// Raise OSError(err, strerror, path). Needs the GIL.
[[noreturn]] void raise_os_error(int err, const std::filesystem::path& path) {
    errno = err;
    PyErr_SetFromErrnoWithFilename(PyExc_OSError, path.string().c_str());
    throw nb::python_error();
}

// Write already serialized `xml` to `path` without the GIL.
void write_file(const std::string& xml, const std::filesystem::path& path) {
    int err = 0;
    {
        nb::gil_scoped_release release;
        err = write_text(path, xml);
    }
    if (err != 0) raise_os_error(err, path);
}

// Serialize `object` with the GIL held (another Python thread may be changing it), then write.
template <typename T>
void to_file(const T& object, const std::filesystem::path& path) {
    write_file(Serialization::toArchiveStringXML(object), path);
}

// Read and parse `path` without the GIL: the result is a new object no other thread sees.

template <typename T>
T from_file(const std::filesystem::path& path) {
    std::string xml;
    int err = 0;
    {
        nb::gil_scoped_release release;
        err = read_text(path, xml);
    }
    if (err != 0) raise_os_error(err, path);
    nb::gil_scoped_release release;
    return Serialization::fromArchiveStringXML<T>(xml);
}

// Binary archives are Python bytes.
nb::bytes to_bytes(const std::vector<std::uint8_t>& data) {
    return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
}

std::vector<std::uint8_t> from_bytes(const nb::bytes& data) {
    const auto* begin = static_cast<const std::uint8_t*>(data.data());
    return std::vector<std::uint8_t>(begin, begin + data.size());
}

}  // namespace

NB_MODULE(serialization, m)
{
    m.doc() = "Tesseract serialization bindings (XML/binary via Cereal)";

    // The serialized types' defining modules: CompositeInstruction, Environment, SceneState
    // (else stubs quote the C++ name)
    nb::module_::import_("tesseract_robotics.command_language");
    nb::module_::import_("tesseract_robotics.environment");
    nb::module_::import_("tesseract_robotics.state_solver");
    nb::module_::import_("tesseract_robotics.task_composer");

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

    m.def("composite_instruction_to_file", &to_file<CompositeInstruction>, nb::arg("instruction"), nb::arg("path"),
        "Save CompositeInstruction as XML to `path`, as named. Raises OSError if it cannot be written.");

    m.def("composite_instruction_from_file", &from_file<CompositeInstruction>, nb::arg("path"),
        "Load CompositeInstruction from the XML file `path`. Raises OSError if it cannot be read.");

    m.def("composite_instruction_to_binary",
        [](const CompositeInstruction& ci) { return to_bytes(Serialization::toArchiveBinaryData(ci)); },
        nb::arg("instruction"),
        "Serialize CompositeInstruction to binary data (bytes; faster, smaller than XML).");

    m.def("composite_instruction_from_binary",
        [](const nb::bytes& data) {
            return Serialization::fromArchiveBinaryData<CompositeInstruction>(from_bytes(data));
        },
        nb::arg("data"),
        "Deserialize CompositeInstruction from binary data (bytes).");

    // ============================================================
    // Environment - Full Scene State
    // ============================================================

    // NOTE: the Environment writers serialize a shared_ptr<Environment>, NOT a concrete
    // Environment. is_polymorphic<Environment> is true, so cereal's shared_ptr format
    // (with a polymorphic type header) differs from the concrete format. The readers below
    // deserialize shared_ptr<Environment>, so the writers MUST match or the reader misparses
    // the stream and throws "unregistered polymorphic cast" (tesseract's own C++ test
    // round-trips Environment::Ptr on both sides). The no-op deleter wraps the borrowed ref
    // without taking ownership.
    auto as_shared = [](const Environment& env) {
        return std::shared_ptr<Environment>(const_cast<Environment*>(&env), [](Environment*) {});
    };

    // GIL released (scripts/nogil_entry_points.yaml): serializing reads the environment under its
    // lock, which no thread may wait for with the GIL (docs/developer/threading.md); deserializing
    // replays the commands into a new one, which has no event callbacks.
    m.def("environment_to_xml",
        [as_shared](const Environment& env) {
            return Serialization::toArchiveStringXML(as_shared(env));
        },
        nb::arg("environment"), nb::call_guard<nb::gil_scoped_release>(),
        "Serialize Environment to XML string. Includes command history - "
        "on deserialize, commands are replayed to rebuild scene graph.");

    m.def("environment_from_xml",
        [](const std::string& xml) {
            return Serialization::fromArchiveStringXML<std::shared_ptr<Environment>>(xml);
        },
        nb::arg("xml"), nb::call_guard<nb::gil_scoped_release>(),
        "Deserialize Environment from XML string. Returns shared_ptr.");

    m.def("environment_to_file",
        [as_shared](const Environment& env, const std::filesystem::path& path) {
            std::string xml;
            {
                nb::gil_scoped_release release;
                xml = Serialization::toArchiveStringXML(as_shared(env));
            }
            write_file(xml, path);
        },
        nb::arg("environment"), nb::arg("path"),
        "Save Environment as XML to `path`, as named. Raises OSError if it cannot be written.");

    m.def("environment_from_file", &from_file<std::shared_ptr<Environment>>, nb::arg("path"),
        "Load Environment from the XML file `path`. Raises OSError if it cannot be read.");

    m.def("environment_to_binary",
        [as_shared](const Environment& env) {
            std::vector<std::uint8_t> data;
            {
                nb::gil_scoped_release release;
                data = Serialization::toArchiveBinaryData(as_shared(env));
            }
            return to_bytes(data);
        },
        nb::arg("environment"),
        "Serialize Environment to binary data (bytes).");

    m.def("environment_from_binary",
        [](const nb::bytes& data) {
            std::vector<std::uint8_t> buffer = from_bytes(data);
            nb::gil_scoped_release release;
            return Serialization::fromArchiveBinaryData<std::shared_ptr<Environment>>(buffer);
        },
        nb::arg("data"),
        "Deserialize Environment from binary data (bytes).");

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

    m.def("scene_state_to_file", &to_file<SceneState>, nb::arg("state"), nb::arg("path"),
        "Save SceneState as XML to `path`, as named. Raises OSError if it cannot be written.");

    m.def("scene_state_from_file", &from_file<SceneState>, nb::arg("path"),
        "Load SceneState from the XML file `path`. Raises OSError if it cannot be read.");

    m.def("scene_state_to_binary",
        [](const SceneState& state) { return to_bytes(Serialization::toArchiveBinaryData(state)); },
        nb::arg("state"),
        "Serialize SceneState to binary data (bytes).");

    m.def("scene_state_from_binary",
        [](const nb::bytes& data) { return Serialization::fromArchiveBinaryData<SceneState>(from_bytes(data)); },
        nb::arg("data"),
        "Deserialize SceneState from binary data (bytes).");

    // ============================================================
    // TaskComposerLog - a run recorded for replay
    // ============================================================

    m.def("task_composer_log_to_xml",
        [](const TaskComposerLog& log) {
            return Serialization::toArchiveStringXML(log);
        },
        nb::arg("log"),
        "Serialize TaskComposerLog to XML string: description, initial data, context and dot graph.");

    m.def("task_composer_log_from_xml",
        [](const std::string& xml) {
            return Serialization::fromArchiveStringXML<TaskComposerLog>(xml);
        },
        nb::arg("xml"),
        "Deserialize TaskComposerLog from XML string.");

    m.def("task_composer_log_to_file", &to_file<TaskComposerLog>, nb::arg("log"), nb::arg("path"),
        "Save TaskComposerLog as XML to `path`, as named. Raises OSError if it cannot be written.");

    m.def("task_composer_log_from_file", &from_file<TaskComposerLog>, nb::arg("path"),
        "Load TaskComposerLog from the XML file `path`. Raises OSError if it cannot be read.");

    m.def("task_composer_log_to_binary",
        [](const TaskComposerLog& log) { return to_bytes(Serialization::toArchiveBinaryData(log)); },
        nb::arg("log"),
        "Serialize TaskComposerLog to binary data (bytes).");

    m.def("task_composer_log_from_binary",
        [](const nb::bytes& data) { return Serialization::fromArchiveBinaryData<TaskComposerLog>(from_bytes(data)); },
        nb::arg("data"),
        "Deserialize TaskComposerLog from binary data (bytes).");
}
