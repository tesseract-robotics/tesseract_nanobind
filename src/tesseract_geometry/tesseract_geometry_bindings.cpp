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
#include <tesseract/geometry/impl/signed_distance_field.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
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

namespace tg = tesseract::geometry;
namespace tc = tesseract::common;

// Disable type caster for this specific vector type so we can bind it as a class
NB_MAKE_OPAQUE(std::vector<std::shared_ptr<const tg::Geometry>>);

namespace {

/**
 * @brief The (N, 3) sample-point matrix handed to a batched Python sampler.
 *
 * Row-major so the numpy view nanobind hands to Python is C-contiguous.
 */
using SamplePoints = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

/** @brief Signature a batched Python sampler is cast to: (N, 3) float64 array -> (N,) float64 array */
using PyBatchedSampler = std::function<Eigen::VectorXd(const SamplePoints&)>;

/**
 * @brief Adapt a Python callable to tesseract's BatchedSignedDistanceFunction.
 *
 * Casting through a std::function lets nanobind own the GIL handling on both the call and the
 * (possibly off-main-thread) destruction of the captured callable, and gives us numpy arrays on
 * the Python side instead of a list of 3-vectors.
 */
tg::BatchedSignedDistanceFunction makeBatchedSampler(const nb::callable& fn) {
    auto py_fn = nb::cast<PyBatchedSampler>(fn);
    return [py_fn](const std::vector<Eigen::Vector3d>& points) -> std::vector<double> {
        const auto n = static_cast<Eigen::Index>(points.size());
        SamplePoints pts(n, 3);
        for (Eigen::Index i = 0; i < n; ++i)
            pts.row(i) = points[static_cast<std::size_t>(i)].transpose();

        const Eigen::VectorXd out = py_fn(pts);
        if (out.size() != n) {
            throw std::runtime_error("Batched signed distance function returned " + std::to_string(out.size()) +
                                     " distances for " + std::to_string(n) + " points");
        }
        return { out.data(), out.data() + n };
    };
}

/** @brief Adapt a per-point sampler to the batched signature so both paths share one code path */
tg::BatchedSignedDistanceFunction toBatched(tg::SignedDistanceFunction sdf) {
    return [sdf = std::move(sdf)](const std::vector<Eigen::Vector3d>& points) {
        std::vector<double> out;
        out.reserve(points.size());
        for (const Eigen::Vector3d& p : points)
            out.push_back(sdf(p));
        return out;
    };
}

/**
 * @brief Sample @p sampler onto a dense grid and return a field that holds only that grid.
 *
 * Upstream's createDiscreteSignedDistanceField pre-discretizes but still retains the sampler, which
 * from Python means the geometry keeps a reference to the user's callable forever. That reference is
 * a collection hazard: the callable reaches its __globals__, so a field stored in a module global is
 * part of an uncollectable cycle (nanobind instances have no tp_traverse), and it means collision
 * backends could re-enter Python. Rebuilding as a grid-backed field costs one copy of the samples and
 * leaves no Python in the geometry - which is what "discrete" should mean at this boundary.
 */
tg::SignedDistanceField::Ptr discretizeToGrid(const tg::BatchedSignedDistanceFunction& sampler,
                                              const Eigen::Vector3d& domain_min,
                                              const Eigen::Vector3d& domain_max,
                                              const Eigen::Vector3i& dimensions,
                                              const Eigen::Vector3d& scale) {
    const auto sampled = tg::createDiscreteSignedDistanceField(sampler, domain_min, domain_max, dimensions, scale);
    return std::make_shared<tg::SignedDistanceField>(
        Eigen::AlignedBox3d(domain_min, domain_max), dimensions, sampled->getDistances(), scale);
}

/**
 * @brief Run a VDB/NanoVDB writer with the GIL released and hand the result back as bytes.
 *
 * The release is scoped to the encode itself rather than applied with nb::call_guard, because the
 * guard would also cover construction of the returned nb::bytes - allocating a Python object with
 * the GIL released segfaults.
 *
 * The grid is materialized BEFORE the GIL is dropped, which is load-bearing rather than tidiness.
 * Both writers read getDistances(), and on a function-backed field that runs discretize(), which
 * takes a process-wide static mutex and then calls the Python sampler - re-entering the interpreter.
 * Doing that inside the released region is a lock-order inversion: this thread would hold the mutex
 * and block acquiring the GIL, while another Python thread holding the GIL blocks on the same mutex
 * (it is static, so *any* lazy field in the process contends for it) - a deadlock. Discretizing here
 * is idempotent and the encode needs the grid regardless, so the released region stays pure C++.
 */
template <typename Writer>
nb::bytes writeBytes(Writer&& writer, const tg::SignedDistanceField& sdf) {
    sdf.discretize();
    std::vector<std::uint8_t> data;
    {
        nb::gil_scoped_release nogil;
        data = writer(sdf);
    }
    return nb::bytes(data.data(), data.size());
}

}  // namespace

NB_MODULE(_tesseract_geometry, m) {
    m.doc() = "tesseract_geometry Python bindings";

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
        .value("SIGNED_DISTANCE_FIELD", tg::GeometryType::SIGNED_DISTANCE_FIELD)
        .value("OCTREE", tg::GeometryType::OCTREE)
        .value("POLYGON_MESH", tg::GeometryType::POLYGON_MESH)
        .value("COMPOUND_MESH", tg::GeometryType::COMPOUND_MESH);

    // Geometry base class (abstract)
    nb::class_<tg::Geometry>(m, "Geometry")
        .def("getType", &tg::Geometry::getType, "Get the geometry type")
        .def("clone", &tg::Geometry::clone, "Create a copy of this geometry")
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
        .def("getTextureImage", &tg::MeshTexture::getTextureImage, "Get the texture image resource")
        .def("getUVs", [](tg::MeshTexture& self) {
            auto uvs = self.getUVs();
            if (!uvs) return tc::VectorVector2d();
            return *uvs;
        }, "Get UV coordinates");

    // PolygonMesh (base for Mesh, ConvexMesh) - inherits shared_ptr holder from Geometry
    nb::class_<tg::PolygonMesh, tg::Geometry>(m, "PolygonMesh")
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
    nb::class_<tg::Mesh, tg::PolygonMesh>(m, "Mesh")
        .def("__init__", [](tg::Mesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::Mesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // ConvexMesh
    nb::class_<tg::ConvexMesh, tg::PolygonMesh>(m, "ConvexMesh")
        .def("__init__", [](tg::ConvexMesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::ConvexMesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // SignedDistanceField - volumetric signed distance field (negative inside the surface)
    nb::class_<tg::SignedDistanceField, tg::Geometry>(m, "SignedDistanceField")
        .def("__init__", [](tg::SignedDistanceField* self,
                            const Eigen::Vector3d& domain_min,
                            const Eigen::Vector3d& domain_max,
                            const Eigen::Vector3i& dimensions,
                            const Eigen::VectorXd& distances,
                            const Eigen::Vector3d& scale) {
            new (self) tg::SignedDistanceField(Eigen::AlignedBox3d(domain_min, domain_max),
                                               dimensions,
                                               std::vector<double>(distances.data(), distances.data() + distances.size()),
                                               scale);
        }, "domain_min"_a, "domain_max"_a, "dimensions"_a, "distances"_a, "scale"_a = Eigen::Vector3d::Ones(),
        "Create a field from signed distances sampled on a regular grid.\n\n"
        "domain_min/domain_max bound the sampled axis-aligned domain in the field's local frame,\n"
        "dimensions is the number of samples along each axis (>= 2 per axis), and distances holds\n"
        "dimensions.prod() values, negative inside the surface.\n\n"
        "distances is flat and ordered x-fastest: index = i + nx*(j + ny*k). From a numpy grid built\n"
        "with np.meshgrid(..., indexing='ij'), pass grid.ravel(order='F').")
        .def("getDomainMin", [](const tg::SignedDistanceField& self) -> Eigen::Vector3d {
            return self.getDomain().min();
        }, "Get the minimum corner of the sampled domain (local frame)")
        .def("getDomainMax", [](const tg::SignedDistanceField& self) -> Eigen::Vector3d {
            return self.getDomain().max();
        }, "Get the maximum corner of the sampled domain (local frame)")
        .def("getDimensions", &tg::SignedDistanceField::getDimensions,
             "Get the number of samples along each axis")
        .def("getDistances", [](const tg::SignedDistanceField& self) -> Eigen::VectorXd {
            const std::vector<double>& d = self.getDistances();
            return Eigen::Map<const Eigen::VectorXd>(d.data(), static_cast<Eigen::Index>(d.size()));
        }, "Get the sampled signed distances, flat and x-fastest (index = i + nx*(j + ny*k)).\n"
           "Reshape with .reshape(field.getDimensions(), order='F') for a 3D view.\n"
           "Discretizes a function-backed field on first call.")
        .def("getScale", &tg::SignedDistanceField::getScale, "Get the local scaling applied to the field")
        .def("getDistance", &tg::SignedDistanceField::getDistance, "point"_a,
             "Get the signed distance at a point in the field's local frame.\n"
             "A function-backed field evaluates its sampler directly; a grid-backed field\n"
             "trilinearly interpolates. The point is clamped to the domain.")
        .def("isDiscretized", &tg::SignedDistanceField::isDiscretized,
             "Whether the dense sample grid has been materialized")
        .def("discretize", &tg::SignedDistanceField::discretize,
             "Materialize the dense sample grid from the sampler (idempotent, no-op if already discretized).\n"
             "A function-backed field must be discretized before it can be serialized or compared; that\n"
             "happens automatically at those boundaries, so call this only to pin the snapshot up front.")
        .def("__eq__", &tg::SignedDistanceField::operator==)
        .def("__ne__", &tg::SignedDistanceField::operator!=)
        .def("__repr__", [](const tg::SignedDistanceField& self) {
            const Eigen::Vector3i& d = self.getDimensions();
            return "SignedDistanceField(dimensions=[" + std::to_string(d.x()) + ", " + std::to_string(d.y()) +
                   ", " + std::to_string(d.z()) + "], discretized=" +
                   (self.isDiscretized() ? "True" : "False") + ")";
        });

    // CompoundMesh - container for multiple meshes from a single resource (e.g., .dae file)
    nb::class_<tg::CompoundMesh, tg::Geometry>(m, "CompoundMesh")
        .def(nb::init<std::vector<std::shared_ptr<tg::PolygonMesh>>>(), "meshes"_a)
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

    // SignedDistanceField factories
    //
    // Both take a Python callable `sdf`. With batched=False it is called once per sample point with
    // a (3,) point and returns a float; with batched=True it is called with an (N, 3) float64 array
    // and must return N distances in the same order - far faster for a numpy/GPU evaluator, and the
    // reason the batched form exists upstream.
    m.def("createDiscreteSignedDistanceField", [](const nb::callable& sdf,
                                                  const Eigen::Vector3d& domain_min,
                                                  const Eigen::Vector3d& domain_max,
                                                  const Eigen::Vector3i& dimensions,
                                                  const Eigen::Vector3d& scale,
                                                  bool batched) {
        return discretizeToGrid(batched ? makeBatchedSampler(sdf) : toBatched(nb::cast<tg::SignedDistanceFunction>(sdf)),
                                domain_min, domain_max, dimensions, scale);
    }, "sdf"_a, "domain_min"_a, "domain_max"_a, "dimensions"_a, "scale"_a = Eigen::Vector3d::Ones(),
       "batched"_a = false,
    "Sample a signed distance function onto a dense grid up front and return the resulting field.\n\n"
    "sdf is negative inside the surface and evaluated in the field's local frame. The grid spans\n"
    "[domain_min, domain_max] inclusive with dimensions samples per axis (>= 2 per axis), so the\n"
    "callable is invoked dimensions.prod() times - pass batched=True to evaluate them in one call.\n\n"
    "Prefer this over createSignedDistanceField: sdf is called only during this call and is not\n"
    "retained, so the returned field is pure data - no GIL taken during collision checking and no\n"
    "reference back into your Python objects.");

    m.def("createSignedDistanceField", [](const nb::callable& sdf,
                                          const Eigen::Vector3d& domain_min,
                                          const Eigen::Vector3d& domain_max,
                                          const Eigen::Vector3i& dimensions,
                                          const Eigen::Vector3d& scale,
                                          bool batched) {
        if (batched)
            return tg::createSignedDistanceField(makeBatchedSampler(sdf), domain_min, domain_max, dimensions, scale);

        return tg::createSignedDistanceField(nb::cast<tg::SignedDistanceFunction>(sdf),
                                             domain_min, domain_max, dimensions, scale);
    }, "sdf"_a, "domain_min"_a, "domain_max"_a, "dimensions"_a, "scale"_a = Eigen::Vector3d::Ones(),
       "batched"_a = false,
    "Create a lazily-evaluated field that keeps the distance function as its source of truth.\n\n"
    "No grid is sampled up front: queries and collision backends call sdf directly (exact, no\n"
    "resampling), and the grid is materialized only for serialization or comparison. dimensions is\n"
    "the resolution used when that happens.\n\n"
    "Use createDiscreteSignedDistanceField unless you specifically need exact sampling. The field\n"
    "keeps sdf alive and re-enters it from C++, which costs you three things.\n\n"
    "1. sdf must be thread-safe.\n"
    "2. Every evaluation takes the GIL, and the collision backends call getDistance() once per\n"
    "   sample point. contactTest releases the GIL precisely so trajectory sweeps can run in\n"
    "   parallel; a lazy field re-acquires it per query, serializing that work back against the\n"
    "   interpreter. Call discretize() before handing the field to a contact manager - after that\n"
    "   the sampler is never invoked again and the field behaves like pure data.\n"
    "3. The field references sdf (and so its __globals__), so storing it in a module-level global\n"
    "   forms a reference cycle the garbage collector cannot see through - drop the field\n"
    "   explicitly or keep it out of module scope.\n\n"
    "Discretizing from a thread that does not hold the GIL is also a deadlock risk: discretize()\n"
    "holds a process-wide static mutex while calling the sampler, so it must not run inside a\n"
    "GIL-released region. The bindings never do that (the VDB writers discretize up front), but do\n"
    "not arrange it yourself by, say, discretizing from inside a C++ callback.");

    // SignedDistanceField serialization to/from the standard VDB grid formats. Both return/accept
    // the raw file bytes, so a .vdb on disk round-trips through Path.read_bytes()/write_bytes().
    // The field's local scale is not stored in the grid and must be supplied again on read.
    m.def("writeSignedDistanceFieldVDB", [](const tg::SignedDistanceField& sdf) {
        return writeBytes([](const tg::SignedDistanceField& f) { return tg::writeSignedDistanceFieldVDB(f); }, sdf);
    }, "sdf"_a,
    "Serialize a field as a standard OpenVDB FloatGrid (raises if the voxel spacing is non-uniform)");

    m.def("readSignedDistanceFieldVDB", [](const nb::bytes& data, const Eigen::Vector3d& scale) {
        return tg::readSignedDistanceFieldVDB(reinterpret_cast<const std::uint8_t*>(data.c_str()), data.size(), scale);
    }, "data"_a, "scale"_a = Eigen::Vector3d::Ones(),
    "Reconstruct a field from OpenVDB FloatGrid bytes (exactly one axis-aligned, uniformly scaled grid)");

    m.def("writeSignedDistanceFieldNVDB", [](const tg::SignedDistanceField& sdf) {
        return writeBytes([](const tg::SignedDistanceField& f) { return tg::writeSignedDistanceFieldNVDB(f); }, sdf);
    }, "sdf"_a,
    "Serialize a field as a standard NanoVDB FloatGrid file");

    m.def("readSignedDistanceFieldNVDB", [](const nb::bytes& data, const Eigen::Vector3d& scale) {
        return tg::readSignedDistanceFieldNVDB(reinterpret_cast<const std::uint8_t*>(data.c_str()), data.size(), scale);
    }, "data"_a, "scale"_a = Eigen::Vector3d::Ones(),
    "Reconstruct a field from NanoVDB FloatGrid bytes (exactly one axis-aligned, uniformly scaled grid)");

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
