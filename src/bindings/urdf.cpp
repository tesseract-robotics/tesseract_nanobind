/**
 * @file urdf.cpp
 * @brief nanobind bindings for tesseract_urdf
 */

#include "tesseract_nb.h"

#include <chrono>
#include <filesystem>

#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/urdf/utils.h>
#include <tesseract/geometry/impl/polygon_mesh.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/resource_locator.h>

namespace tu = tesseract::urdf;

namespace {

// writeURDFFile failed. Upstream nests the cause under "Could not write out urdf link"
// (std::throw_with_nested at every level); the message here carries the whole chain.
struct URDFWriteError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Upstream also calls throw_with_nested outside any handler, which nests nothing: rethrowing
// that empty nest would call std::terminate, so it is checked first.
// [s] file systems store modification times coarser than the clock: FAT 2 s, HFS+ 1 s, ext4
// and APFS 1 ns. A file written by this call is never older than the call's start by more.
constexpr auto FILE_TIME_SLACK = std::chrono::seconds(2);

std::string nested_what(const std::exception& e) {
    std::string out = e.what();
    const auto* nested = dynamic_cast<const std::nested_exception*>(&e);
    if (nested == nullptr || nested->nested_ptr() == nullptr)
        return out;
    try {
        nested->rethrow_nested();
    } catch (const std::exception& inner) {
        out += ": " + nested_what(inner);
    }
    return out;
}

}  // namespace

NB_MODULE(urdf, m) {
    m.doc() = "tesseract_urdf Python bindings";

    // Cross-module type registration: parseURDF* return SceneGraph (by value),
    // whose nb::class_ lives in scene_graph. Import it so the type is
    // registered even when the caller never imported tesseract_scene_graph —
    // otherwise the return-value cast fails with "Unable to convert function
    // return value". Latent since day one: the full test suite always imported
    // scene_graph during collection; the minimal-import wheel canary exposed it.
    nb::module_::import_("tesseract_robotics.scene_graph._scene_graph");
    nb::module_::import_("tesseract_robotics.common._common");
    // writeMeshToFile takes a PolygonMesh: import its defining module so the stub names it (#168).
    nb::module_::import_("tesseract_robotics.geometry");

    // parseURDF* release the GIL: XML parsing plus mesh loading. A Python ResourceLocator
    // subclass still works: its trampoline re-acquires the GIL (scripts/nogil_entry_points.yaml).
    // parseURDFString - returns unique_ptr, nanobind handles conversion
    m.def("parseURDFString", &tu::parseURDFString,
          "urdf_xml_string"_a, "locator"_a, nb::call_guard<nb::gil_scoped_release>(),
          "Parse a URDF string into a SceneGraph");

    // parseURDFFile
    m.def("parseURDFFile", &tu::parseURDFFile,
          "path"_a, "locator"_a, nb::call_guard<nb::gil_scoped_release>(),
          "Parse a URDF file into a SceneGraph");

    // writeURDFFile
    nb::exception<URDFWriteError>(m, "URDFWriteError", PyExc_RuntimeError)
        .attr("__doc__") = "writeURDFFile could not write the scene graph; the message names the cause.";
    // Upstream writes `package_path`/urdf/<urdf_name or the graph's name>.urdf and ignores
    // tinyxml2's SaveFile result, so an unwritable file failed silently: the binding checks that
    // the file was written by this call and returns its path.
    m.def("writeURDFFile", [](const std::shared_ptr<const tesseract::scene_graph::SceneGraph>& scene_graph,
                              const std::string& package_path, const std::string& urdf_name) {
        const std::filesystem::path written = std::filesystem::path(package_path) / "urdf" /
                                              ((urdf_name.empty() ? scene_graph->getName() : urdf_name) + ".urdf");
        const auto started = std::filesystem::file_time_type::clock::now();
        try {
            tu::writeURDFFile(scene_graph, package_path, urdf_name);
        } catch (const std::exception& e) {
            throw URDFWriteError("writeURDFFile: " + nested_what(e));
        }
        std::error_code ec;
        const auto modified = std::filesystem::last_write_time(written, ec);
        if (ec || modified < started - FILE_TIME_SLACK)
            throw URDFWriteError("writeURDFFile: could not write " + written.string());
        return written;
    }, "scene_graph"_a, "package_path"_a, "urdf_name"_a = "",
          "Write `scene_graph` to `package_path`/urdf/`urdf_name`.urdf (`urdf_name` is a stem; default: the "
          "graph's name), its meshes as PLY files under `package_path`/visual and /collision. Returns "
          "the URDF's path. Raises URDFWriteError naming the cause.");

    // writeMeshToFile (utils.h:30). `mesh` takes no None: upstream dereferences it unchecked
    // (urdf/src/utils.cpp:140-155); its "Could not export file" std::runtime_error stays RuntimeError.
    m.def("writeMeshToFile", &tu::writeMeshToFile, "mesh"_a, "filepath"_a,
          "Write the mesh as an ASCII PLY file (always PLY, whatever the extension of filepath).");
}
