/**
 * @file tesseract_geometry_bindings.cpp
 * @brief nanobind bindings for tesseract_geometry
 */

#include "tesseract_nb.h"

// tesseract_geometry
#include <tesseract/geometry/geometry.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/sphere.h>
#include <tesseract/geometry/impl/cylinder.h>
#include <tesseract/geometry/impl/capsule.h>
#include <tesseract/geometry/impl/cone.h>
#include <tesseract/geometry/impl/plane.h>
#include <tesseract/geometry/impl/polygon_mesh.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/impl/convex_mesh.h>
#include <tesseract/geometry/impl/sdf_mesh.h>
#include <tesseract/geometry/impl/compound_mesh.h>
#include <tesseract/geometry/impl/mesh_material.h>
#include <tesseract/geometry/impl/octree.h>
#include <tesseract/geometry/impl/octree_utils.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/geometry/utils.h>
#include <tesseract/geometry/conversions.h>

// octomap
#include <octomap/OcTree.h>

// tesseract_common
#include <tesseract/common/types.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/resource_locator.h>

// boost::uuids::uuid <-> str, the spelling JointTrajectory.uuid uses
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>

namespace tg = tesseract::geometry;
namespace tc = tesseract::common;

// Disable type caster for this specific vector type so we can bind it as a class
NB_MAKE_OPAQUE(std::vector<std::shared_ptr<const tg::Geometry>>);

// The face_count mesh constructors store the count they are given, unchecked
// (polygon_mesh.cpp:59-81 @ 0.35.0); the binding refuses one that disagrees with the faces.
struct MeshFaceCountError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

namespace {

using MeshTextures = std::vector<std::shared_ptr<tg::MeshTexture>>;

// Counts faces the way the counting PolygonMesh constructor does (polygon_mesh.cpp:51-56 @ 0.35.0).
int count_faces(const Eigen::VectorXi& faces) {
    int count = 0;
    for (Eigen::Index i = 0; i < faces.size(); ++i) {
        ++count;
        i += faces(i);
    }
    return count;
}

void check_face_count(const char* cls, const Eigen::VectorXi& faces, int face_count) {
    const int counted = count_faces(faces);
    if (counted != face_count)
        throw MeshFaceCountError(std::string(cls) + ": face_count " + std::to_string(face_count) +
                                 " disagrees with faces, which describes " + std::to_string(counted));
}

template <typename T>
std::shared_ptr<const T> share(const std::optional<T>& value) {
    return value ? std::make_shared<const T>(*value) : nullptr;
}

// Both native constructors of a PolygonMesh subclass, with the C++ defaults: (vertices, faces, ...)
// and (vertices, faces, face_count, ...). Every optional pointer argument takes None for null.
// PolygonMesh itself also takes a trailing `type` (polygon_mesh.h:79, :107): Tail names its C++
// parameter types, tail_args its Python arguments.
template <typename T, typename... Tail, typename Class, typename... TailArgs>
Class bind_mesh_ctors(Class cls, const char* name, TailArgs... tail_args) {
    cls.def("__init__",
            [](T* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces, tc::Resource::Ptr resource,
               const Eigen::Vector3d& scale, std::optional<tc::VectorVector3d> normals,
               std::optional<tc::VectorVector4d> vertex_colors, tg::MeshMaterial::Ptr mesh_material,
               std::optional<MeshTextures> mesh_textures, Tail... tail) {
                new (self) T(std::make_shared<const tc::VectorVector3d>(vertices),
                             std::make_shared<const Eigen::VectorXi>(faces), std::move(resource), scale,
                             share(normals), share(vertex_colors), std::move(mesh_material), share(mesh_textures),
                             tail...);
            },
            "vertices"_a, "faces"_a, nb::arg("resource").none() = nb::none(), "scale"_a = Eigen::Vector3d::Ones(),
            nb::arg("normals").none() = nb::none(), nb::arg("vertex_colors").none() = nb::none(),
            nb::arg("mesh_material").none() = nb::none(), nb::arg("mesh_textures").none() = nb::none(), tail_args...);
    cls.def("__init__",
            [name](T* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces, int face_count,
                   tc::Resource::Ptr resource, const Eigen::Vector3d& scale, std::optional<tc::VectorVector3d> normals,
                   std::optional<tc::VectorVector4d> vertex_colors, tg::MeshMaterial::Ptr mesh_material,
                   std::optional<MeshTextures> mesh_textures, Tail... tail) {
                check_face_count(name, faces, face_count);
                new (self) T(std::make_shared<const tc::VectorVector3d>(vertices),
                             std::make_shared<const Eigen::VectorXi>(faces), face_count, std::move(resource), scale,
                             share(normals), share(vertex_colors), std::move(mesh_material), share(mesh_textures),
                             tail...);
            },
            "vertices"_a, "faces"_a, "face_count"_a, nb::arg("resource").none() = nb::none(),
            "scale"_a = Eigen::Vector3d::Ones(), nb::arg("normals").none() = nb::none(),
            nb::arg("vertex_colors").none() = nb::none(), nb::arg("mesh_material").none() = nb::none(),
            nb::arg("mesh_textures").none() = nb::none(), tail_args...);
    return cls;
}

}  // namespace

NB_MODULE(_tesseract_geometry, m) {
    m.doc() = "tesseract_geometry Python bindings";

    // Import common module for the Isometry3d type (else stubs quote the C++ name, which is compiler-specific)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // GeometryType enum
    nb::enum_<tg::GeometryType>(m, "GeometryType")
        .value("UNINITIALIZED", tg::GeometryType::UNINITIALIZED)
        .value("SPHERE", tg::GeometryType::SPHERE)
        .value("CYLINDER", tg::GeometryType::CYLINDER)
        .value("CAPSULE", tg::GeometryType::CAPSULE)
        .value("CONE", tg::GeometryType::CONE)
        .value("BOX", tg::GeometryType::BOX)
        .value("PLANE", tg::GeometryType::PLANE)
        .value("MESH", tg::GeometryType::MESH)
        .value("CONVEX_MESH", tg::GeometryType::CONVEX_MESH)
        .value("SDF_MESH", tg::GeometryType::SDF_MESH)
        .value("OCTREE", tg::GeometryType::OCTREE)
        .value("POLYGON_MESH", tg::GeometryType::POLYGON_MESH)
        .value("COMPOUND_MESH", tg::GeometryType::COMPOUND_MESH);

    // Geometry base class (abstract)
    nb::class_<tg::Geometry>(m, "Geometry")
        .def("getType", &tg::Geometry::getType, "Get the geometry type")
        .def("clone", &tg::Geometry::clone, "Create a copy of this geometry")
        .def("getUUID", [](const tg::Geometry& self) { return boost::uuids::to_string(self.getUUID()); },
             "UUID as its canonical string")
        .def("setUUID", [](tg::Geometry& self, const std::string& s) {
                 try {
                     self.setUUID(boost::uuids::string_generator()(s));
                 } catch (const std::runtime_error&) {
                     throw std::invalid_argument("Geometry.setUUID: not a UUID string: '" + s + "'");
                 }
             }, "uuid"_a, "Set the UUID from its canonical string; a malformed string raises ValueError")
        .def("__eq__", &tg::Geometry::operator==)
        .def("__ne__", &tg::Geometry::operator!=);

    // GeometriesConst - vector of const geometry shared_ptr
    using GeometriesConst = std::vector<std::shared_ptr<const tg::Geometry>>;
    nb::class_<GeometriesConst>(m, "GeometriesConst")
        .def(nb::init<>())
        .def("__len__", [](const GeometriesConst& v) { return v.size(); })
        .def("__getitem__", [](const GeometriesConst& v, size_t i) {
            if (i >= v.size()) throw nb::index_error();
            return v[i];
        }, nb::rv_policy::reference_internal)
        .def("append", [](GeometriesConst& v, std::shared_ptr<const tg::Geometry> item) {
            v.push_back(item);
        })
        .def("clear", [](GeometriesConst& v) { v.clear(); });

    // Box
    nb::class_<tg::Box, tg::Geometry>(m, "Box")
        .def(nb::init<double, double, double>(), "x"_a, "y"_a, "z"_a,
             "Create a box with dimensions x, y, z")
        .def(nb::init<>(), "Create a default box")
        .def("getX", &tg::Box::getX, "Get X dimension")
        .def("getY", &tg::Box::getY, "Get Y dimension")
        .def("getZ", &tg::Box::getZ, "Get Z dimension")
        .def("__eq__", &tg::Box::operator==)
        .def("__ne__", &tg::Box::operator!=)
        .def("__repr__", [](const tg::Box& self) {
            return "Box(" + std::to_string(self.getX()) + ", " +
                   std::to_string(self.getY()) + ", " +
                   std::to_string(self.getZ()) + ")";
        });

    // Sphere
    nb::class_<tg::Sphere, tg::Geometry>(m, "Sphere")
        .def(nb::init<double>(), "r"_a, "Create a sphere with radius r")
        .def(nb::init<>(), "Create a default sphere")
        .def("getRadius", &tg::Sphere::getRadius, "Get the radius")
        .def("__eq__", &tg::Sphere::operator==)
        .def("__ne__", &tg::Sphere::operator!=)
        .def("__repr__", [](const tg::Sphere& self) {
            return "Sphere(" + std::to_string(self.getRadius()) + ")";
        });

    // Cylinder
    nb::class_<tg::Cylinder, tg::Geometry>(m, "Cylinder")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a cylinder with radius r and length l")
        .def(nb::init<>(), "Create a default cylinder")
        .def("getRadius", &tg::Cylinder::getRadius, "Get the radius")
        .def("getLength", &tg::Cylinder::getLength, "Get the length")
        .def("__eq__", &tg::Cylinder::operator==)
        .def("__ne__", &tg::Cylinder::operator!=)
        .def("__repr__", [](const tg::Cylinder& self) {
            return "Cylinder(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Capsule
    nb::class_<tg::Capsule, tg::Geometry>(m, "Capsule")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a capsule with radius r and length l")
        .def(nb::init<>(), "Create a default capsule")
        .def("getRadius", &tg::Capsule::getRadius, "Get the radius")
        .def("getLength", &tg::Capsule::getLength, "Get the length")
        .def("__eq__", &tg::Capsule::operator==)
        .def("__ne__", &tg::Capsule::operator!=)
        .def("__repr__", [](const tg::Capsule& self) {
            return "Capsule(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Cone
    nb::class_<tg::Cone, tg::Geometry>(m, "Cone")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a cone with radius r and length l")
        .def(nb::init<>(), "Create a default cone")
        .def("getRadius", &tg::Cone::getRadius, "Get the radius")
        .def("getLength", &tg::Cone::getLength, "Get the length")
        .def("__eq__", &tg::Cone::operator==)
        .def("__ne__", &tg::Cone::operator!=)
        .def("__repr__", [](const tg::Cone& self) {
            return "Cone(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Plane
    nb::class_<tg::Plane, tg::Geometry>(m, "Plane")
        .def(nb::init<double, double, double, double>(), "a"_a, "b"_a, "c"_a, "d"_a,
             "Create a plane with equation ax + by + cz + d = 0")
        .def(nb::init<>(), "Create a default plane")
        .def("getA", &tg::Plane::getA, "Get coefficient a")
        .def("getB", &tg::Plane::getB, "Get coefficient b")
        .def("getC", &tg::Plane::getC, "Get coefficient c")
        .def("getD", &tg::Plane::getD, "Get coefficient d")
        .def("__eq__", &tg::Plane::operator==)
        .def("__ne__", &tg::Plane::operator!=)
        .def("__repr__", [](const tg::Plane& self) {
            return "Plane(" + std::to_string(self.getA()) + ", " +
                   std::to_string(self.getB()) + ", " +
                   std::to_string(self.getC()) + ", " +
                   std::to_string(self.getD()) + ")";
        });

    // MeshMaterial - PBR material properties
    nb::class_<tg::MeshMaterial>(m, "MeshMaterial")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Vector4d&, double, double, const Eigen::Vector4d&>(),
             "base_color_factor"_a, "metallic_factor"_a, "roughness_factor"_a, "emissive_factor"_a)
        .def("getBaseColorFactor", &tg::MeshMaterial::getBaseColorFactor, "Get base color (RGBA)")
        .def("getMetallicFactor", &tg::MeshMaterial::getMetallicFactor, "Get metallic factor (0-1)")
        .def("getRoughnessFactor", &tg::MeshMaterial::getRoughnessFactor, "Get roughness factor (0-1)")
        .def("getEmissiveFactor", &tg::MeshMaterial::getEmissiveFactor, "Get emissive factor (RGBA)");

    // MeshTexture - texture with UV coordinates
    nb::class_<tg::MeshTexture>(m, "MeshTexture")
        // Neither argument takes None: upstream stores both unchecked (mesh_material.cpp:49-53), but a
        // texture without an image or UVs is never valid. texture_image must be a jpg or png (mesh_material.h:144).
        .def("__init__", [](tg::MeshTexture* self, tc::Resource::Ptr texture_image, const tc::VectorVector2d& uvs) {
            new (self) tg::MeshTexture(std::move(texture_image), std::make_shared<const tc::VectorVector2d>(uvs));
        }, "texture_image"_a, "uvs"_a, "Create a texture from a jpg or png image resource and per-vertex UVs")
        .def("getTextureImage", &tg::MeshTexture::getTextureImage, "Get the texture image resource")
        .def("getUVs", [](tg::MeshTexture& self) {
            auto uvs = self.getUVs();
            if (!uvs) return tc::VectorVector2d();
            return *uvs;
        }, "Get UV coordinates");

    nb::exception<MeshFaceCountError>(m, "MeshFaceCountError", PyExc_ValueError).attr("__doc__") =
        "A mesh constructor's face_count disagrees with the number of faces its faces array describes.";

    // PolygonMesh (base for Mesh, ConvexMesh, SDFMesh) - inherits shared_ptr holder from Geometry
    bind_mesh_ctors<tg::PolygonMesh, tg::GeometryType>(nb::class_<tg::PolygonMesh, tg::Geometry>(m, "PolygonMesh"),
                                                       "PolygonMesh", "type"_a = tg::GeometryType::POLYGON_MESH)
        .def("getVertexCount", &tg::PolygonMesh::getVertexCount, "Get number of vertices")
        .def("getFaceCount", &tg::PolygonMesh::getFaceCount, "Get number of faces")
        .def("getScale", &tg::PolygonMesh::getScale, "Get mesh scale")
        .def("getVertices", [](const tg::PolygonMesh& self) {
            auto verts = self.getVertices();
            if (!verts) return tc::VectorVector3d();
            return *verts;
        })
        .def("getFaces", [](const tg::PolygonMesh& self) -> Eigen::VectorXi {
            auto faces = self.getFaces();
            if (!faces) return Eigen::VectorXi();
            return *faces;
        })
        .def("getNormals", [](const tg::PolygonMesh& self) -> std::optional<tc::VectorVector3d> {
            auto normals = self.getNormals();
            if (!normals) return std::nullopt;
            return *normals;
        }, "Get vertex normals (optional)")
        .def("getVertexColors", [](const tg::PolygonMesh& self) -> std::optional<tc::VectorVector4d> {
            auto colors = self.getVertexColors();
            if (!colors) return std::nullopt;
            return *colors;
        }, "Get vertex colors (optional)")
        .def("getMaterial", &tg::PolygonMesh::getMaterial, "Get mesh material (optional)")
        .def("getTextures", [](const tg::PolygonMesh& self) -> std::optional<std::vector<std::shared_ptr<tg::MeshTexture>>> {
            auto textures = self.getTextures();
            if (!textures) return std::nullopt;
            return *textures;
        }, "Get mesh textures (optional)")
        .def("getResource", &tg::PolygonMesh::getResource, "Get mesh resource");

    // Mesh
    bind_mesh_ctors<tg::Mesh>(nb::class_<tg::Mesh, tg::PolygonMesh>(m, "Mesh"), "Mesh");

    // ConvexMesh
    nb::class_<tg::ConvexMesh, tg::PolygonMesh>(m, "ConvexMesh")
        .def("__init__", [](tg::ConvexMesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::ConvexMesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // SDFMesh
    bind_mesh_ctors<tg::SDFMesh>(nb::class_<tg::SDFMesh, tg::PolygonMesh>(m, "SDFMesh"), "SDFMesh");

    // CompoundMesh - container for multiple meshes from a single resource (e.g., .dae file)
    nb::class_<tg::CompoundMesh, tg::Geometry>(m, "CompoundMesh")
        .def("__init__", [](tg::CompoundMesh* self, const std::vector<std::shared_ptr<tg::PolygonMesh>>& meshes) {
            new (self) tg::CompoundMesh(require_non_null(meshes, "CompoundMesh", "meshes", "a PolygonMesh"));
        }, "meshes"_a)
        .def("getMeshes", &tg::CompoundMesh::getMeshes, nb::rv_policy::reference_internal,
             "Get the vector of meshes")
        .def("getResource", &tg::CompoundMesh::getResource, "Get the resource used to create this mesh")
        .def("getScale", &tg::CompoundMesh::getScale, "Get the scale applied to the mesh");

    // octomap::OcTree - minimal binding so callers can construct/load and
    // pass it to tesseract::geometry::Octree
    nb::class_<octomap::OcTree>(m, "OcTree")
        .def(nb::init<double>(), "resolution"_a,
             "Create an empty octomap OcTree with the given leaf resolution")
        .def(nb::init<std::string>(), "filename"_a,
             "Load an octomap OcTree from a .bt or .ot file")
        .def("getResolution", &octomap::OcTree::getResolution, "Get the leaf resolution")
        .def("size", &octomap::OcTree::size, "Get the total number of nodes")
        .def("getNumLeafNodes", &octomap::OcTree::getNumLeafNodes, "Get the number of leaf nodes")
        .def("updateNode", [](octomap::OcTree& self, double x, double y, double z, bool occupied, bool lazy_eval) {
            self.updateNode(x, y, z, occupied, lazy_eval);
        }, "x"_a, "y"_a, "z"_a, "occupied"_a, "lazy_eval"_a = false,
        "Insert/update a node at the given coordinate")
        .def("updateInnerOccupancy", &octomap::OcTree::updateInnerOccupancy,
             "Recompute inner occupancies after lazy updates")
        .def("toMaxLikelihood", &octomap::OcTree::toMaxLikelihood,
             "Convert occupancy probabilities to a binary maximum-likelihood representation")
        .def("writeBinary", [](octomap::OcTree& self, const std::string& filename) {
            return self.writeBinary(filename);
        }, "filename"_a, "Write the octree to a binary .bt file");

    // OctreeSubType enum
    nb::enum_<tg::OctreeSubType>(m, "OctreeSubType")
        .value("BOX", tg::OctreeSubType::BOX)
        .value("SPHERE_INSIDE", tg::OctreeSubType::SPHERE_INSIDE)
        .value("SPHERE_OUTSIDE", tg::OctreeSubType::SPHERE_OUTSIDE);

    // PointCloud::Point
    nb::class_<tg::PointCloud::Point>(m, "PointCloudPoint")
        .def(nb::init<>())
        .def(nb::init<double, double, double>(), "x"_a, "y"_a, "z"_a)
        .def_rw("x", &tg::PointCloud::Point::x)
        .def_rw("y", &tg::PointCloud::Point::y)
        .def_rw("z", &tg::PointCloud::Point::z);

    // PointCloud
    nb::class_<tg::PointCloud>(m, "PointCloud")
        .def(nb::init<>())
        .def_rw("points", &tg::PointCloud::points)
        .def("addPoint", &tg::PointCloud::addPoint, "x"_a, "y"_a, "z"_a,
             "Add a point to the cloud");

    // Octree
    nb::class_<tg::Octree, tg::Geometry>(m, "Octree")
        .def("__init__", [](tg::Octree* self,
                            std::shared_ptr<octomap::OcTree> octree,
                            tg::OctreeSubType sub_type,
                            bool pruned,
                            bool binary_octree) {
            new (self) tg::Octree(std::shared_ptr<const octomap::OcTree>(octree), sub_type, pruned, binary_octree);
        }, "octree"_a, "sub_type"_a, "pruned"_a = false, "binary_octree"_a = false,
        "Create an Octree geometry wrapping an octomap OcTree")
        .def("getOctree", &tg::Octree::getOctree, nb::rv_policy::reference_internal,
             "Get the underlying octomap OcTree")
        .def("getSubType", &tg::Octree::getSubType, "Get the sub-shape type")
        .def("getPruned", &tg::Octree::getPruned, "Whether the octree was pruned")
        .def("calcNumSubShapes", &tg::Octree::calcNumSubShapes,
             "Calculate the number of sub-shapes (expensive)")
        .def("__eq__", &tg::Octree::operator==)
        .def("__ne__", &tg::Octree::operator!=)
        .def_static("prune", &tg::Octree::prune, "octree"_a,
                    "Prune the octomap OcTree using tesseract's occupancy-threshold rule");

    // Octree utility: build an octomap::OcTree from a PointCloud
    m.def("createOctree", [](const tg::PointCloud& point_cloud,
                             double resolution,
                             bool prune,
                             bool binary) -> std::shared_ptr<octomap::OcTree> {
        return tg::createOctree(point_cloud, resolution, prune, binary);
    }, "point_cloud"_a, "resolution"_a, "prune"_a, "binary"_a = true,
    "Build an octomap OcTree from a PointCloud");

    // Mesh loading functions
    m.def("createMeshFromPath", [](const std::string& path,
                                   const Eigen::Vector3d& scale,
                                   bool triangulate,
                                   bool flatten) {
        return tg::createMeshFromPath<tg::Mesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of Mesh geometries");

    m.def("createConvexMeshFromPath", [](const std::string& path,
                                         const Eigen::Vector3d& scale,
                                         bool triangulate,
                                         bool flatten) {
        return tg::createMeshFromPath<tg::ConvexMesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of ConvexMesh geometries");

    m.def("createSDFMeshFromPath", [](const std::string& path,
                                      const Eigen::Vector3d& scale,
                                      bool triangulate,
                                      bool flatten) {
        return tg::createMeshFromPath<tg::SDFMesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of SDFMesh geometries");

    // Mesh loading from Resource (for package:// URLs)
    m.def("createMeshFromResource", [](tc::Resource::Ptr resource,
                                       const Eigen::Vector3d& scale,
                                       bool triangulate,
                                       bool flatten) {
        return tg::createMeshFromResource<tg::Mesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load Mesh from resource (e.g., package:// URL)");

    m.def("createConvexMeshFromResource", [](tc::Resource::Ptr resource,
                                              const Eigen::Vector3d& scale,
                                              bool triangulate,
                                              bool flatten) {
        return tg::createMeshFromResource<tg::ConvexMesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load ConvexMesh from resource (e.g., package:// URL)");

    m.def("createSDFMeshFromResource", [](tc::Resource::Ptr resource,
                                          const Eigen::Vector3d& scale,
                                          bool triangulate,
                                          bool flatten) {
        return tg::createMeshFromResource<tg::SDFMesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load SDFMesh from resource (e.g., package:// URL)");

    // Utilities
    m.def("isIdentical", &tg::isIdentical, "geom1"_a, "geom2"_a,
          "Check if two geometries are identical");

    m.def("extractVertices", &tg::extractVertices, "geom"_a, "origin"_a,
          "Extract vertices from a geometry, transforming primitives to a mesh first");

    // Conversions
    m.def("toTriangleMesh", [](const tg::Geometry& geom,
                               double tolerance,
                               const Eigen::Isometry3d& origin) {
        return tg::toTriangleMesh(geom, tolerance, origin);
    }, "geom"_a, "tolerance"_a, "origin"_a,
    "Convert a primitive geometry to a triangle Mesh");
}
