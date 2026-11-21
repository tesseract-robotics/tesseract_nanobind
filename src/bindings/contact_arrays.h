#pragma once

// ContactArrays: every ContactResult of a ContactResultMap or ContactResultVector as one set of
// numpy arrays, row r for result r (collision.cpp binds `to_arrays()` on both). Each array owns
// its buffer (a capsule), so it outlives the results, the ContactArrays and any clear().

#include "tesseract_nb.h"
#include "transform_stack.h"

#include <tesseract/collision/types.h>

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace contact_arrays {

namespace tc = tesseract::collision;
namespace tcommon = tesseract::common;

// The int fields of ContactResult (type_id, shape_id, subshape_id) are copied without
// conversion: an int32 column is exactly as wide as the C++ field.
static_assert(sizeof(int) == sizeof(std::int32_t), "ContactResult's int fields are read as int32");
// cc_type is copied as its enum's underlying type, the narrowest dtype that holds every value.
using CCTypeInt = std::underlying_type_t<tc::ContinuousCollisionType>;
static_assert(std::is_same_v<CCTypeInt, std::uint8_t>, "cc_type column is documented as uint8");

template <class T, class Shape>
using Array = nb::ndarray<nb::numpy, T, Shape, nb::c_contig>;
using Rows = Array<double, nb::shape<-1>>;                   // (n,)
using RowsBool = Array<bool, nb::shape<-1>>;                 // (n,)
using Vectors = Array<double, nb::shape<-1, 3>>;             // (n, 3)
using PairDoubles = Array<double, nb::shape<-1, 2>>;         // (n, 2)
using PairInts = Array<std::int32_t, nb::shape<-1, 2>>;      // (n, 2)
using PairCCTypes = Array<CCTypeInt, nb::shape<-1, 2>>;      // (n, 2)
using PairPoints = Array<double, nb::shape<-1, 2, 3>>;       // (n, 2, 3)
using PairTransforms = Array<double, nb::shape<-1, 2, 4, 4>>;  // (n, 2, 4, 4)

struct ContactArrays {
    Rows distance;
    Vectors normal;
    PairPoints nearest_points;
    PairPoints nearest_points_local;
    PairTransforms transform;
    PairInts type_id;
    PairInts shape_id;
    PairInts subshape_id;
    RowsBool single_contact_point;
    PairDoubles cc_time;
    PairCCTypes cc_type;
    PairTransforms cc_transform;
    PairInts link_index;
    std::vector<tcommon::LinkId> links;
};

// A zero-initialized C-order array of `shape` that owns its buffer. Built with the GIL held: the
// capsule is a Python object.
template <class A>
A new_array(std::initializer_list<std::size_t> shape) {
    using T = typename A::Scalar;
    std::size_t count = 1;
    for (const std::size_t extent : shape) count *= extent;
    T* data = new T[count]();
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<T*>(p); });
    return A(data, shape, owner);
}

// One pass over `results` (any range whose elements bind to const ContactResult&), in its order.
// The GIL stays held: the arrays' owners are Python capsules, and a row is a copy of 93 scalars.
template <class Range>
ContactArrays to_arrays(const Range& results, std::size_t n) {
    ContactArrays out;
    out.distance = new_array<Rows>({n});
    out.normal = new_array<Vectors>({n, 3});
    out.nearest_points = new_array<PairPoints>({n, 2, 3});
    out.nearest_points_local = new_array<PairPoints>({n, 2, 3});
    out.transform = new_array<PairTransforms>({n, 2, 4, 4});
    out.type_id = new_array<PairInts>({n, 2});
    out.shape_id = new_array<PairInts>({n, 2});
    out.subshape_id = new_array<PairInts>({n, 2});
    out.single_contact_point = new_array<RowsBool>({n});
    out.cc_time = new_array<PairDoubles>({n, 2});
    out.cc_type = new_array<PairCCTypes>({n, 2});
    out.cc_transform = new_array<PairTransforms>({n, 2, 4, 4});
    out.link_index = new_array<PairInts>({n, 2});

    std::unordered_map<tcommon::LinkId, std::int32_t> index_of;  // link -> its index in out.links
    std::size_t r = 0;
    for (const tc::ContactResult& c : results) {
        out.distance.data()[r] = c.distance;
        Eigen::Map<Eigen::Vector3d>(out.normal.data() + 3 * r) = c.normal;
        out.single_contact_point.data()[r] = c.single_contact_point;
        for (std::size_t k = 0; k < 2; ++k) {
            const std::size_t rk = 2 * r + k;
            Eigen::Map<Eigen::Vector3d>(out.nearest_points.data() + 3 * rk) = c.nearest_points[k];
            Eigen::Map<Eigen::Vector3d>(out.nearest_points_local.data() + 3 * rk) = c.nearest_points_local[k];
            Eigen::Map<RowMajor4d>(out.transform.data() + 16 * rk) = c.transform[k].matrix();
            Eigen::Map<RowMajor4d>(out.cc_transform.data() + 16 * rk) = c.cc_transform[k].matrix();
            out.type_id.data()[rk] = c.type_id[k];
            out.shape_id.data()[rk] = c.shape_id[k];
            out.subshape_id.data()[rk] = c.subshape_id[k];
            out.cc_time.data()[rk] = c.cc_time[k];
            out.cc_type.data()[rk] = static_cast<CCTypeInt>(c.cc_type[k]);
            const auto [it, inserted] =
                index_of.try_emplace(c.link_ids[k], static_cast<std::int32_t>(out.links.size()));
            if (inserted) out.links.push_back(c.link_ids[k]);
            out.link_index.data()[rk] = it->second;
        }
        ++r;
    }
    return out;
}

inline ContactArrays to_arrays(const tc::ContactResultVector& results) { return to_arrays(results, results.size()); }

// Row order is flattenCopyResults()'s: upstream's flattenWrapperResults walks the map's buckets and
// each pair's vector in the same order, without copying the results.
inline ContactArrays to_arrays(const tc::ContactResultMap& results) {
    std::vector<std::reference_wrapper<const tc::ContactResult>> rows;
    results.flattenWrapperResults(rows);
    return to_arrays(rows, rows.size());
}

constexpr const char* TO_ARRAYS_DOC =
    "Every result as one ContactArrays: row r of each array is result r.\n\n"
    "Rows follow this container's order: ContactResultMap.flattenCopyResults() order for a map,\n"
    "element order for a vector. Each array is a new C-contiguous copy that owns its buffer: it\n"
    "stays valid after clear(), and writing to it does not change the results. For n results it\n"
    "costs one pass in C++ instead of one Python object per field read.";

inline void bind(nb::module_& m) {
    // A property getter defaults to reference_internal, which an ndarray that already has an owner
    // (its capsule) refuses; `automatic` returns a numpy view of that owned buffer, no copy.
    constexpr nb::rv_policy VIEW_POLICY = nb::rv_policy::automatic;
    nb::class_<ContactArrays>(
        m, "ContactArrays",
        "Every ContactResult of a ContactResultMap or ContactResultVector as arrays, from `to_arrays()`.\n\n"
        "n rows, row r for result r in the container's order. Each array owns its buffer (each read\n"
        "returns a view of that buffer), independent of the results it was read from. A field\n"
        "of the same name on ContactResult reads `results[r].<field>` as `arrays.<field>[r]`;\n"
        "`link_ids` becomes `links` plus `link_index`.\n\n"
        "Slots: column k (axis 1) of every two-slot field describes `link_ids[k]`, the same body\n"
        "for every field of a row. Which body is slot 0 is chosen by the contact manager, not by\n"
        "the binding (see ContactResult.link_ids); resolve a body by `links[link_index[r, k]]`,\n"
        "never by k. `normal[r]` points from slot 0 towards slot 1.")
        .def_prop_ro("distance", [](const ContactArrays& a) { return a.distance; }, VIEW_POLICY,
                     "(n,) float64, m: signed distance between the two nearest points; negative when the "
                     "bodies penetrate.")
        .def_prop_ro("normal", [](const ContactArrays& a) { return a.normal; }, VIEW_POLICY,
                     "(n, 3) float64, world frame, unit vector: points from link slot 0 towards slot 1, the "
                     "direction to move slot 1 to separate the pair.")
        .def_prop_ro("nearest_points", [](const ContactArrays& a) { return a.nearest_points; }, VIEW_POLICY,
                     "(n, 2, 3) float64, world frame, m: [r, k] is the point of link slot k nearest the other.")
        .def_prop_ro("nearest_points_local", [](const ContactArrays& a) { return a.nearest_points_local; }, VIEW_POLICY,
                     "(n, 2, 3) float64, m: nearest_points[r, k] in the frame of link slot k "
                     "(transform[r, k] is that frame's world pose).")
        .def_prop_ro("transform", [](const ContactArrays& a) { return a.transform; }, VIEW_POLICY,
                     "(n, 2, 4, 4) float64, homogeneous, translation in m: [r, k] is the world pose of link "
                     "slot k (for a cast, its start pose).")
        .def_prop_ro("type_id", [](const ContactArrays& a) { return a.type_id; }, VIEW_POLICY,
                     "(n, 2) int32: [r, k] is the type id (the mask id given to addCollisionObject) of link slot k.")
        .def_prop_ro("shape_id", [](const ContactArrays& a) { return a.shape_id; }, VIEW_POLICY,
                     "(n, 2) int32: [r, k] is the index of the shape of link slot k in contact.")
        .def_prop_ro("subshape_id", [](const ContactArrays& a) { return a.subshape_id; }, VIEW_POLICY,
                     "(n, 2) int32: [r, k] is the sub-shape (mesh triangle, octree box) of link slot k; -1 for none.")
        .def_prop_ro("single_contact_point", [](const ContactArrays& a) { return a.single_contact_point; }, VIEW_POLICY,
                     "(n,) bool: True when the checker gave one contact point, so nearest_points[r, 0] == "
                     "nearest_points[r, 1].")
        .def_prop_ro("cc_time", [](const ContactArrays& a) { return a.cc_time; }, VIEW_POLICY,
                     "(n, 2) float64, dimensionless: [r, k] is where in its cast link slot k is in contact, "
                     "0 at its start pose, 1 at its end pose; -1 when slot k was not cast.")
        .def_prop_ro("cc_type", [](const ContactArrays& a) { return a.cc_type; }, VIEW_POLICY,
                     "(n, 2) uint8: [r, k] is the ContinuousCollisionType value of link slot k "
                     "(0 CCType_None, 1 CCType_Time0, 2 CCType_Time1, 3 CCType_Between); "
                     "ContinuousCollisionType(int(v)) gives the enum.")
        .def_prop_ro("cc_transform", [](const ContactArrays& a) { return a.cc_transform; }, VIEW_POLICY,
                     "(n, 2, 4, 4) float64, homogeneous, translation in m: [r, k] is the world end pose of "
                     "the cast of link slot k (identity when slot k was not cast).")
        .def_prop_ro("link_index", [](const ContactArrays& a) { return a.link_index; }, VIEW_POLICY,
                     "(n, 2) int32: links[link_index[r, k]] is link_ids[k] of result r.")
        .def_prop_ro("links",
                     [](const ContactArrays& a) {
                         nb::list items;
                         for (const auto& link : a.links) items.append(nb::cast(link));
                         return nb::typed<nb::tuple, tcommon::LinkId, nb::ellipsis>(nb::tuple(items));
                     },
                     "The distinct links of the results, in order of first appearance (row by row, slot 0 "
                     "before slot 1). A new tuple per read.")
        .def("__repr__", [](const ContactArrays& a) {
            return "ContactArrays(n=" + std::to_string(a.distance.shape(0)) +
                   ", links=" + std::to_string(a.links.size()) + ")";
        });
}

}  // namespace contact_arrays
