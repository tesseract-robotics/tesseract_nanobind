/**
 * @file tesseract_collision_bindings.cpp
 * @brief nanobind bindings for tesseract_collision
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/tuple.h>

// tesseract_collision core
#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/contact_managers_plugin_factory.h>
#include <tesseract/collision/contact_result_validator.h>

// tesseract_common for types
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/contact_allowed_validator.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/common/resource_locator.h>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <tesseract/common/types.h>
#include <tesseract/common/plugin_info.h>
#include <yaml-cpp/yaml.h>

// tesseract_geometry for collision objects
#include <tesseract/geometry/geometry.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/impl/convex_mesh.h>

// bullet convex hull utils
#include <tesseract/collision/bullet/convex_hull_utils.h>

// V-HACD convex decomposition. convex_decomposition_vhacd.h defines ENABLE_VHACD_IMPLEMENTATION
// before including VHACD.h, which compiles the whole single-header V-HACD implementation into this
// TU. Including VHACD.h first (implementation disabled) makes the guarded second include a no-op, so
// VHACD:: resolves to the one copy in libtesseract_collision_vhacd_convex_decomposition.
#include <tesseract/collision/vhacd/VHACD.h>
#include <tesseract/collision/convex_decomposition.h>
#include <tesseract/collision/vhacd/convex_decomposition_vhacd.h>
// VHACD_GOOGOL_SIZE is defined only inside VHACD.h's implementation section: if it is set here, this
// TU compiled a second V-HACD copy (an ODR and size hazard, not a link error). Fails the build on
// every platform, Windows included, where the nm test cannot look.
#ifdef VHACD_GOOGOL_SIZE
#error "V-HACD implementation compiled into the binding: include VHACD.h before convex_decomposition_vhacd.h"
#endif

namespace tc = tesseract::collision;
namespace tcommon = tesseract::common;
namespace tg = tesseract::geometry;

// Disable type caster for ContactResultVector so we can bind it as a class
NB_MAKE_OPAQUE(tc::ContactResultVector);

namespace {

using CRM = tc::ContactResultMap;

// ContactResultMap mutator preconditions that upstream only assert()s (types.cpp), so a
// release build would store an unreachable key or call back() on an empty vector.
struct UnorderedLinkPairError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};
struct EmptyContactResultsError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// createConvexHull returns n < 0 when Bullet cannot apply the requested shrink
// (btConvexHullInternal::shrink -> shiftFace); upstream only logs it.
struct ConvexHullError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// ConvexDecompositionVHACD::compute walks `faces` without bounds checks (an out-of-bounds read on a
// count that runs past the end or an index past `vertices`) and throws a bare runtime_error for a
// non-triangle face. The binding checks both before calling it.
struct MalformedFacesError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};
struct NonTriangleFaceError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

void check_triangle_faces(const tcommon::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
    const Eigen::Index n = faces.size();
    const auto n_vertices = static_cast<long long>(vertices.size());
    for (Eigen::Index i = 0; i < n;) {
        const int count = faces(i);
        if (count != 3)
            throw NonTriangleFaceError("faces[" + std::to_string(i) + "] = " + std::to_string(count) +
                                       ": V-HACD decomposes triangle meshes only (count 3)");
        if (i + count >= n)
            throw MalformedFacesError("faces[" + std::to_string(i) + "] = " + std::to_string(count) +
                                      " runs past the end of faces (size " + std::to_string(n) + ")");
        for (Eigen::Index k = i + 1; k <= i + count; ++k)
            if (faces(k) < 0 || faces(k) >= n_vertices)
                throw MalformedFacesError("faces[" + std::to_string(k) + "] = " + std::to_string(faces(k)) +
                                          " is not a vertex index (" + std::to_string(n_vertices) + " vertices)");
        i += count + 1;
    }
}

void check_ordered_key(const CRM::KeyType& key) {
    if (tcommon::makeOrderedLinkPair(key.first, key.second) != key)
        throw UnorderedLinkPairError("ContactResultMap key ('" + key.first + "', '" + key.second +
                                     "') is not ordered: use ('" + key.second + "', '" + key.first + "')");
}

void check_nonempty(const CRM::MappedType& results) {
    if (results.empty())
        throw EmptyContactResultsError("ContactResultMap: results must hold at least one ContactResult");
}

// Wrap a Python callable as FilterFn. The vector is passed by reference so the callback can
// clear or append; it is valid only for the duration of the call (same contract as
// PySQPCallback's problem argument, src/trajopt_sqp/trajopt_sqp_bindings.cpp).
using PyFilterFn = nb::typed<nb::callable, void(CRM::KeyType, tc::ContactResultVector)>;

CRM::FilterFn to_filter_fn(const std::optional<PyFilterFn>& maybe_fn) {
    if (!maybe_fn) return nullptr;
    nb::callable fn = *maybe_fn;
    return [fn](CRM::PairType& pair) {
        fn(nb::cast(pair.first), nb::cast(pair.second, nb::rv_policy::reference));
    };
}

// Python subclasses implement __call__(result) -> bool. The ticket inside NB_OVERRIDE_PURE_NAME
// takes the GIL, which contactTest releases. The ContactResult argument is copied (nanobind's
// default for const& arguments), so a validator that keeps a reference cannot dangle.
class PyContactResultValidator : public tc::ContactResultValidator {
public:
    NB_TRAMPOLINE(tc::ContactResultValidator, 1);

    bool operator()(const tc::ContactResult& result) const override {
        NB_OVERRIDE_PURE_NAME("__call__", operator(), result);
    }
};

}  // namespace

NB_MODULE(_tesseract_collision, m) {
    // Import geometry module for tesseract::geometry::{Geometry,Mesh,ConvexMesh} (else stubs quote the
    // C++ name). It imports tesseract_common in turn, which covers Isometry3d, ACM and CollisionMarginData.
    nb::module_::import_("tesseract_robotics.tesseract_geometry._tesseract_geometry");
    m.doc() = "tesseract_collision Python bindings";

    // ========== Enums ==========
    nb::enum_<tc::ContinuousCollisionType>(m, "ContinuousCollisionType")
        .value("CCType_None", tc::ContinuousCollisionType::CCType_None)
        .value("CCType_Time0", tc::ContinuousCollisionType::CCType_Time0)
        .value("CCType_Time1", tc::ContinuousCollisionType::CCType_Time1)
        .value("CCType_Between", tc::ContinuousCollisionType::CCType_Between);

    nb::enum_<tc::ContactTestType>(m, "ContactTestType")
        .value("FIRST", tc::ContactTestType::FIRST)
        .value("CLOSEST", tc::ContactTestType::CLOSEST)
        .value("ALL", tc::ContactTestType::ALL)
        .value("LIMITED", tc::ContactTestType::LIMITED);

    nb::enum_<tc::CollisionEvaluatorType>(m, "CollisionEvaluatorType")
        .value("NONE", tc::CollisionEvaluatorType::NONE)
        .value("DISCRETE", tc::CollisionEvaluatorType::DISCRETE)
        .value("LVS_DISCRETE", tc::CollisionEvaluatorType::LVS_DISCRETE)
        .value("CONTINUOUS", tc::CollisionEvaluatorType::CONTINUOUS)
        .value("LVS_CONTINUOUS", tc::CollisionEvaluatorType::LVS_CONTINUOUS);

    nb::enum_<tc::CollisionCheckProgramType>(m, "CollisionCheckProgramType")
        .value("ALL", tc::CollisionCheckProgramType::ALL)
        .value("ALL_EXCEPT_START", tc::CollisionCheckProgramType::ALL_EXCEPT_START)
        .value("ALL_EXCEPT_END", tc::CollisionCheckProgramType::ALL_EXCEPT_END)
        .value("START_ONLY", tc::CollisionCheckProgramType::START_ONLY)
        .value("END_ONLY", tc::CollisionCheckProgramType::END_ONLY)
        .value("INTERMEDIATE_ONLY", tc::CollisionCheckProgramType::INTERMEDIATE_ONLY);

    nb::enum_<tc::ACMOverrideType>(m, "ACMOverrideType")
        .value("NONE", tc::ACMOverrideType::NONE)
        .value("ASSIGN", tc::ACMOverrideType::ASSIGN)
        .value("AND", tc::ACMOverrideType::AND)
        .value("OR", tc::ACMOverrideType::OR);

    nb::enum_<tc::CollisionCheckExitType>(m, "CollisionCheckExitType")
        .value("FIRST", tc::CollisionCheckExitType::FIRST)
        .value("ONE_PER_STEP", tc::CollisionCheckExitType::ONE_PER_STEP)
        .value("ALL", tc::CollisionCheckExitType::ALL);

    // ========== ContactResult ==========
    // Pair fields are std::array<T, 2>: nanobind's array caster raises TypeError on any other length.
    nb::class_<tc::ContactResult>(m, "ContactResult")
        .def(nb::init<>())
        .def_rw("distance", &tc::ContactResult::distance)
        .def_rw("type_id", &tc::ContactResult::type_id)
        .def_rw("link_names", &tc::ContactResult::link_names)
        .def_rw("shape_id", &tc::ContactResult::shape_id)
        .def_rw("subshape_id", &tc::ContactResult::subshape_id)
        .def_rw("nearest_points", &tc::ContactResult::nearest_points)
        .def_rw("nearest_points_local", &tc::ContactResult::nearest_points_local)
        .def_rw("transform", &tc::ContactResult::transform)
        .def_rw("normal", &tc::ContactResult::normal)
        .def_rw("cc_time", &tc::ContactResult::cc_time)
        .def_rw("cc_type", &tc::ContactResult::cc_type)
        .def_rw("cc_transform", &tc::ContactResult::cc_transform)
        .def_rw("single_contact_point", &tc::ContactResult::single_contact_point)
        .def("clear", &tc::ContactResult::clear);

    // ========== ContactResultVector ==========
    // ContactResultVector uses Eigen aligned_allocator, so we need to bind manually
    nb::class_<tc::ContactResultVector>(m, "ContactResultVector")
        .def(nb::init<>())
        .def("__len__", [](const tc::ContactResultVector& v) { return v.size(); })
        .def("__getitem__", [](const tc::ContactResultVector& v, size_t i) -> const tc::ContactResult& {
            if (i >= v.size()) throw nb::index_error();
            return v[i];
        }, nb::rv_policy::reference_internal)
        .def("append", [](tc::ContactResultVector& v, const tc::ContactResult& item) {
            v.push_back(item);
        })
        .def("clear", [](tc::ContactResultVector& v) { v.clear(); });

    // ========== ContactResultMap ==========
    nb::exception<UnorderedLinkPairError>(m, "UnorderedLinkPairError", PyExc_ValueError)
        .attr("__doc__") = "A ContactResultMap key is not ordered: the first link name must not sort after the second.";
    nb::exception<EmptyContactResultsError>(m, "EmptyContactResultsError", PyExc_ValueError)
        .attr("__doc__") = "A ContactResultMap vector overload got an empty ContactResultVector.";

    // Every value handed to Python is a copy (the filter callback's vector excepted), so Python
    // never holds a reference into the map that a later insert or shrinkToFit invalidates.
    nb::class_<CRM>(m, "ContactResultMap")
        .def(nb::init<>())
        .def("count", &CRM::count)
        .def("size", &CRM::size)
        .def("empty", &CRM::empty)
        .def("clear", &CRM::clear)
        .def("release", &CRM::release)
        .def("getSummary", &CRM::getSummary)
        .def("flattenCopyResults", [](const CRM& self) {
            tc::ContactResultVector v;
            self.flattenCopyResults(v);
            return v;
        })
        .def("flattenMoveResults", [](CRM& self, tc::ContactResultVector& v) {
            self.flattenMoveResults(v);
        }, "results"_a)
        .def("__len__", &CRM::size)
        .def("at", [](const CRM& self, const CRM::KeyType& key) -> tc::ContactResultVector {
            const auto& container = self.getContainer();
            auto it = container.find(key);
            if (it == container.end())
                throw nb::key_error(("ContactResultMap has no key ('" + key.first + "', '" + key.second + "')").c_str());
            return it->second;
        }, "key"_a, "Copy of the results stored under `key`; raises KeyError when it is absent.")
        .def("getContainer", [](const CRM& self) { return self.getContainer(); },
             "Copy of the underlying map, keyed by (link_name1, link_name2).")
        .def("__iter__", [](const CRM& self) {
            // Snapshot: iterating a copy keeps a shrinkToFit()/release() inside the loop safe.
            nb::list items;
            for (const auto& pair : self)
                items.append(nb::make_tuple(pair.first, nb::cast(pair.second, nb::rv_policy::copy)));
            return nb::typed<nb::iterator, std::pair<CRM::KeyType, tc::ContactResultVector>>(nb::iter(items));
        }, "Iterate over (key, ContactResultVector) pairs, like C++ begin()/end(); a dict iterates keys only. "
           "Keys kept by clear() are included with empty vectors.")
        // Mutators. The returned ContactResult is a copy: the C++ reference points into a vector
        // that the next insert may reallocate.
        .def("addContactResult", [](CRM& self, const CRM::KeyType& key, tc::ContactResult result) {
            check_ordered_key(key);
            return tc::ContactResult(self.addContactResult(key, std::move(result)));
        }, "key"_a, "result"_a)
        .def("addContactResult", [](CRM& self, const CRM::KeyType& key, const CRM::MappedType& results) {
            check_ordered_key(key);
            check_nonempty(results);
            return tc::ContactResult(self.addContactResult(key, results));
        }, "key"_a, "results"_a)
        .def("setContactResult", [](CRM& self, const CRM::KeyType& key, tc::ContactResult result) {
            check_ordered_key(key);
            return tc::ContactResult(self.setContactResult(key, std::move(result)));
        }, "key"_a, "result"_a)
        .def("setContactResult", [](CRM& self, const CRM::KeyType& key, const CRM::MappedType& results) {
            check_ordered_key(key);
            check_nonempty(results);
            return tc::ContactResult(self.setContactResult(key, results));
        }, "key"_a, "results"_a)
        .def("shrinkToFit", &CRM::shrinkToFit)
        .def("filter", [](CRM& self, const PyFilterFn& fn) { self.filter(to_filter_fn(fn)); }, "fn"_a,
             "Call `fn(key, results)` for every pair; clearing or appending to `results` edits the map. "
             "`results` is valid only during the call.")
        .def("addInterpolatedCollisionResults",
             [](CRM& self, CRM& sub_segment_results, long sub_segment_index, long sub_segment_last_index,
                const std::vector<std::string>& active_link_names, double segment_dt, bool discrete,
                const std::optional<PyFilterFn>& filter) {
                 self.addInterpolatedCollisionResults(sub_segment_results, sub_segment_index, sub_segment_last_index,
                                                      active_link_names, segment_dt, discrete, to_filter_fn(filter));
             },
             "sub_segment_results"_a, "sub_segment_index"_a, "sub_segment_last_index"_a, "active_link_names"_a,
             "segment_dt"_a, "discrete"_a, "filter"_a = nb::none());

    // ========== ContactTrajectory{Substep,Step,}Results ==========
    // Returned by tesseract_environment.checkTrajectory. The std::stringstream summaries are
    // exposed as str.
    nb::class_<tc::ContactTrajectorySubstepResults>(m, "ContactTrajectorySubstepResults")
        .def(nb::init<>())
        .def(nb::init<int, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
             "substep"_a, "start_state"_a, "end_state"_a)
        .def(nb::init<int, const Eigen::VectorXd&>(), "substep"_a, "state"_a)
        .def("__bool__", [](const tc::ContactTrajectorySubstepResults& self) { return static_cast<bool>(self); })
        .def("addContact", &tc::ContactTrajectorySubstepResults::addContact,
             "substep_number"_a, "start_substate"_a, "end_substate"_a, "new_contacts"_a)
        .def("numContacts", &tc::ContactTrajectorySubstepResults::numContacts)
        .def("worstCollision", &tc::ContactTrajectorySubstepResults::worstCollision)
        .def_rw("contacts", &tc::ContactTrajectorySubstepResults::contacts)
        .def_rw("substep", &tc::ContactTrajectorySubstepResults::substep)
        .def_rw("state0", &tc::ContactTrajectorySubstepResults::state0)
        .def_rw("state1", &tc::ContactTrajectorySubstepResults::state1);

    nb::class_<tc::ContactTrajectoryStepResults>(m, "ContactTrajectoryStepResults")
        .def(nb::init<>())
        .def(nb::init<int, const Eigen::VectorXd&, const Eigen::VectorXd&, int>(),
             "step_number"_a, "start_state"_a, "end_state"_a, "num_substeps"_a)
        .def(nb::init<int, const Eigen::VectorXd&>(), "step_number"_a, "state"_a)
        .def("__bool__", [](const tc::ContactTrajectoryStepResults& self) { return static_cast<bool>(self); })
        .def("addContact", &tc::ContactTrajectoryStepResults::addContact,
             "step_number"_a, "substep_number"_a, "num_substeps"_a, "start_state"_a, "end_state"_a,
             "start_substate"_a, "end_substate"_a, "contacts"_a)
        .def("resize", &tc::ContactTrajectoryStepResults::resize, "num_substeps"_a)
        .def("numSubsteps", &tc::ContactTrajectoryStepResults::numSubsteps)
        .def("numContacts", &tc::ContactTrajectoryStepResults::numContacts)
        .def("worstSubstep", &tc::ContactTrajectoryStepResults::worstSubstep)
        .def("worstCollision", &tc::ContactTrajectoryStepResults::worstCollision)
        .def("mostCollisionsSubstep", &tc::ContactTrajectoryStepResults::mostCollisionsSubstep)
        .def_rw("substeps", &tc::ContactTrajectoryStepResults::substeps)
        .def_rw("step", &tc::ContactTrajectoryStepResults::step)
        .def_rw("state0", &tc::ContactTrajectoryStepResults::state0)
        .def_rw("state1", &tc::ContactTrajectoryStepResults::state1)
        .def_rw("total_substeps", &tc::ContactTrajectoryStepResults::total_substeps);

    nb::class_<tc::ContactTrajectoryResults>(m, "ContactTrajectoryResults")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::string>>(), "j_names"_a)
        .def(nb::init<std::vector<std::string>, int>(), "j_names"_a, "num_steps"_a)
        .def("__bool__", [](const tc::ContactTrajectoryResults& self) { return static_cast<bool>(self); })
        .def("addContact", &tc::ContactTrajectoryResults::addContact,
             "step_number"_a, "substep_number"_a, "num_substeps"_a, "start_state"_a, "end_state"_a,
             "start_substate"_a, "end_substate"_a, "contacts"_a)
        .def("resize", &tc::ContactTrajectoryResults::resize, "num_steps"_a)
        .def("numSteps", &tc::ContactTrajectoryResults::numSteps)
        .def("numContacts", &tc::ContactTrajectoryResults::numContacts)
        .def("worstStep", &tc::ContactTrajectoryResults::worstStep)
        .def("worstCollision", &tc::ContactTrajectoryResults::worstCollision)
        .def("mostCollisionsStep", &tc::ContactTrajectoryResults::mostCollisionsStep)
        .def("trajectoryCollisionResultsTable",
             [](const tc::ContactTrajectoryResults& self) { return self.trajectoryCollisionResultsTable().str(); })
        .def("collisionFrequencyPerLink",
             [](const tc::ContactTrajectoryResults& self) { return self.collisionFrequencyPerLink().str(); })
        .def("condensedSummary",
             [](const tc::ContactTrajectoryResults& self) { return self.condensedSummary().str(); })
        .def_rw("steps", &tc::ContactTrajectoryResults::steps)
        .def_rw("joint_names", &tc::ContactTrajectoryResults::joint_names)
        .def_rw("total_steps", &tc::ContactTrajectoryResults::total_steps);

    // ========== ContactResultValidator ==========
    nb::class_<tc::ContactResultValidator, PyContactResultValidator>(
        m, "ContactResultValidator",
        "Approves or rejects contact results: subclass and implement `__call__(result) -> bool`.")
        .def(nb::init<>())
        .def("__call__", &tc::ContactResultValidator::operator(), "result"_a);

    // ========== ContactRequest ==========
    nb::class_<tc::ContactRequest>(m, "ContactRequest")
        .def(nb::init<>())
        .def(nb::init<tc::ContactTestType>(), "type"_a)
        .def_rw("type", &tc::ContactRequest::type)
        .def_rw("calculate_penetration", &tc::ContactRequest::calculate_penetration)
        .def_rw("calculate_distance", &tc::ContactRequest::calculate_distance)
        .def_rw("contact_limit", &tc::ContactRequest::contact_limit)
        .def_prop_rw("is_valid",
            [](const tc::ContactRequest& self) -> std::optional<std::shared_ptr<const tc::ContactResultValidator>> {
                if (!self.is_valid) return std::nullopt;
                return self.is_valid;
            },
            [](tc::ContactRequest& self, std::shared_ptr<const tc::ContactResultValidator> v) {
                self.is_valid = std::move(v);
            },
            nb::for_setter(nb::arg("value").none()),
            "Validator called on each contact; return False to reject it. None disables validation.");

    // ========== ContactManagerConfig ==========
    // Note: 0.33 renamed margin_data_override_type → pair_margin_override_type
    nb::class_<tc::ContactManagerConfig>(m, "ContactManagerConfig")
        .def(nb::init<>())
        .def(nb::init<double>(), "default_margin"_a)
        .def_prop_rw("default_margin",
            [](const tc::ContactManagerConfig& c) -> std::optional<double> { return c.default_margin; },
            [](tc::ContactManagerConfig& c, double v) { c.default_margin = v; })
        .def_rw("pair_margin_override_type", &tc::ContactManagerConfig::pair_margin_override_type)
        .def_rw("pair_margin_data", &tc::ContactManagerConfig::pair_margin_data)
        .def_rw("acm", &tc::ContactManagerConfig::acm)
        .def_rw("acm_override_type", &tc::ContactManagerConfig::acm_override_type)
        .def_rw("modify_object_enabled", &tc::ContactManagerConfig::modify_object_enabled)
        .def("incrementMargins", &tc::ContactManagerConfig::incrementMargins, "increment"_a)
        .def("scaleMargins", &tc::ContactManagerConfig::scaleMargins, "scale"_a)
        .def("validate", &tc::ContactManagerConfig::validate)
        // Backwards compatibility alias
        .def_prop_rw("margin_data_override_type",
            [](const tc::ContactManagerConfig& c) { return c.pair_margin_override_type; },
            [](tc::ContactManagerConfig& c, tcommon::CollisionMarginPairOverrideType v) { c.pair_margin_override_type = v; });

    // ========== CollisionCheckConfig ==========
    nb::class_<tc::CollisionCheckConfig>(m, "CollisionCheckConfig")
        .def(nb::init<>())
        .def(nb::init<tc::ContactRequest, tc::CollisionEvaluatorType, double, tc::CollisionCheckProgramType,
                      tc::CollisionCheckExitType>(),
             "contact_request"_a = tc::ContactRequest(),
             "type"_a = tc::CollisionEvaluatorType::DISCRETE,
             "longest_valid_segment_length"_a = 0.005,
             "check_program_mode"_a = tc::CollisionCheckProgramType::ALL,
             "exit_condition"_a = tc::CollisionCheckExitType::FIRST)
        .def_rw("contact_request", &tc::CollisionCheckConfig::contact_request)
        .def_rw("type", &tc::CollisionCheckConfig::type)
        .def_rw("longest_valid_segment_length", &tc::CollisionCheckConfig::longest_valid_segment_length)
        .def_rw("check_program_mode", &tc::CollisionCheckConfig::check_program_mode)
        .def_rw("exit_condition", &tc::CollisionCheckConfig::exit_condition);

    // ========== DiscreteContactManager (abstract, expose key methods) ==========
    nb::class_<tc::DiscreteContactManager>(m, "DiscreteContactManager")
        .def("getName", &tc::DiscreteContactManager::getName)
        .def("hasCollisionObject", &tc::DiscreteContactManager::hasCollisionObject, "name"_a)
        .def("removeCollisionObject", &tc::DiscreteContactManager::removeCollisionObject, "name"_a)
        .def("enableCollisionObject", &tc::DiscreteContactManager::enableCollisionObject, "name"_a)
        .def("disableCollisionObject", &tc::DiscreteContactManager::disableCollisionObject, "name"_a)
        .def("isCollisionObjectEnabled", &tc::DiscreteContactManager::isCollisionObjectEnabled, "name"_a)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const std::string& name, const Eigen::Isometry3d& pose) {
                 self.setCollisionObjectsTransform(name, pose);
             }, "name"_a, "pose"_a)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const std::vector<std::string>& names,
                const tcommon::VectorIsometry3d& poses) {
                 self.setCollisionObjectsTransform(names, poses);
             }, "names"_a, "poses"_a)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const tcommon::TransformMap& transforms) {
                 self.setCollisionObjectsTransform(transforms);
             }, "transforms"_a)
        // Overload accepting std::map (from Python dict / link_transforms property)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const std::map<std::string, Eigen::Isometry3d>& transforms) {
                 tcommon::TransformMap tm;
                 for (const auto& p : transforms) {
                     tm[p.first] = p.second;
                 }
                 self.setCollisionObjectsTransform(tm);
             }, "transforms"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("getCollisionObjects", &tc::DiscreteContactManager::getCollisionObjects)
        .def("setActiveCollisionObjects", &tc::DiscreteContactManager::setActiveCollisionObjects, "names"_a)
        .def("getActiveCollisionObjects", &tc::DiscreteContactManager::getActiveCollisionObjects)
        // Note: 0.33 renamed setDefaultCollisionMarginData → setDefaultCollisionMargin
        .def("setDefaultCollisionMargin", &tc::DiscreteContactManager::setDefaultCollisionMargin,
             "default_collision_margin"_a)
        // Note: 0.33 renamed setPairCollisionMarginData → setCollisionMarginPair
        .def("setCollisionMarginPair", &tc::DiscreteContactManager::setCollisionMarginPair,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("setCollisionMarginData", &tc::DiscreteContactManager::setCollisionMarginData,
             "collision_margin_data"_a)
        .def("setCollisionMarginPairData", &tc::DiscreteContactManager::setCollisionMarginPairData,
             "pair_margin_data"_a,
             "override_type"_a = tcommon::CollisionMarginPairOverrideType::REPLACE)
        .def("incrementCollisionMargin", &tc::DiscreteContactManager::incrementCollisionMargin, "increment"_a)
        .def("setContactAllowedValidator", &tc::DiscreteContactManager::setContactAllowedValidator, "validator"_a)
        .def("getContactAllowedValidator", &tc::DiscreteContactManager::getContactAllowedValidator)
        .def("applyContactManagerConfig", &tc::DiscreteContactManager::applyContactManagerConfig, "config"_a)
        // Backwards compatibility aliases
        .def("setDefaultCollisionMarginData", &tc::DiscreteContactManager::setDefaultCollisionMargin,
             "default_collision_margin"_a)
        .def("setPairCollisionMarginData", &tc::DiscreteContactManager::setCollisionMarginPair,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("getCollisionMarginData", &tc::DiscreteContactManager::getCollisionMarginData)
        .def("addCollisionObject",
             [](tc::DiscreteContactManager& self, const std::string& name, int mask_id,
                const std::vector<std::shared_ptr<const tg::Geometry>>& shapes,
                const tcommon::VectorIsometry3d& shape_poses, bool enabled) {
                 return self.addCollisionObject(name, mask_id, shapes, shape_poses, enabled);
             }, "name"_a, "mask_id"_a, "shapes"_a, "shape_poses"_a, "enabled"_a = true)
        .def("getCollisionObjectGeometries", &tc::DiscreteContactManager::getCollisionObjectGeometries, "name"_a)
        .def("getCollisionObjectGeometriesTransforms", &tc::DiscreteContactManager::getCollisionObjectGeometriesTransforms, "name"_a)
        // Release the GIL during the (broad+narrowphase) collision query so a
        // background worker thread can sweep a whole trajectory without blocking
        // the Python UI thread — mirrors the motion planners' solve() guards.
        .def("contactTest", &tc::DiscreteContactManager::contactTest, "collisions"_a, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("clone", [](const tc::DiscreteContactManager& self) { return self.clone(); });

    // ========== ContinuousContactManager (abstract, expose key methods) ==========
    // The three cast (start+end pose) overloads, each registered under two names below.
    auto cast_name = [](tc::ContinuousContactManager& self, const std::string& name,
                        const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2) {
        self.setCollisionObjectsTransform(name, pose1, pose2);
    };
    auto cast_names = [](tc::ContinuousContactManager& self, const std::vector<std::string>& names,
                         const tcommon::VectorIsometry3d& pose1, const tcommon::VectorIsometry3d& pose2) {
        self.setCollisionObjectsTransform(names, pose1, pose2);
    };
    auto cast_maps = [](tc::ContinuousContactManager& self, const std::map<std::string, Eigen::Isometry3d>& pose1,
                        const std::map<std::string, Eigen::Isometry3d>& pose2) {
        tcommon::TransformMap tm1, tm2;
        for (const auto& p : pose1) { tm1[p.first] = p.second; }
        for (const auto& p : pose2) { tm2[p.first] = p.second; }
        self.setCollisionObjectsTransform(tm1, tm2);
    };
    nb::class_<tc::ContinuousContactManager>(m, "ContinuousContactManager")
        .def("getName", &tc::ContinuousContactManager::getName)
        .def("hasCollisionObject", &tc::ContinuousContactManager::hasCollisionObject, "name"_a)
        .def("removeCollisionObject", &tc::ContinuousContactManager::removeCollisionObject, "name"_a)
        .def("enableCollisionObject", &tc::ContinuousContactManager::enableCollisionObject, "name"_a)
        .def("disableCollisionObject", &tc::ContinuousContactManager::disableCollisionObject, "name"_a)
        .def("isCollisionObjectEnabled", &tc::ContinuousContactManager::isCollisionObjectEnabled, "name"_a)
        .def("addCollisionObject",
             [](tc::ContinuousContactManager& self, const std::string& name, int mask_id,
                const std::vector<std::shared_ptr<const tg::Geometry>>& shapes,
                const tcommon::VectorIsometry3d& shape_poses, bool enabled) {
                 return self.addCollisionObject(name, mask_id, shapes, shape_poses, enabled);
             }, "name"_a, "mask_id"_a, "shapes"_a, "shape_poses"_a, "enabled"_a = true)
        .def("getCollisionObjectGeometries", &tc::ContinuousContactManager::getCollisionObjectGeometries, "name"_a)
        .def("getCollisionObjectGeometriesTransforms",
             &tc::ContinuousContactManager::getCollisionObjectGeometriesTransforms, "name"_a)
        // Static (single-pose) object transforms
        .def("setCollisionObjectsTransform",
             [](tc::ContinuousContactManager& self, const std::string& name, const Eigen::Isometry3d& pose) {
                 self.setCollisionObjectsTransform(name, pose);
             }, "name"_a, "pose"_a)
        .def("setCollisionObjectsTransform",
             [](tc::ContinuousContactManager& self, const std::vector<std::string>& names,
                const tcommon::VectorIsometry3d& poses) {
                 self.setCollisionObjectsTransform(names, poses);
             }, "names"_a, "poses"_a)
        .def("setCollisionObjectsTransform",
             [](tc::ContinuousContactManager& self, const std::map<std::string, Eigen::Isometry3d>& transforms) {
                 tcommon::TransformMap tm;
                 for (const auto& p : transforms) { tm[p.first] = p.second; }
                 self.setCollisionObjectsTransform(tm);
             }, "transforms"_a)
        // Cast (moving, start+end pose) overloads, under the native name and under the Python-only
        // setCollisionObjectsTransformCast, which stays until its removal is approved separately.
        .def("setCollisionObjectsTransform", cast_name, "name"_a, "pose1"_a, "pose2"_a)
        .def("setCollisionObjectsTransform", cast_names, "names"_a, "pose1"_a, "pose2"_a)
        .def("setCollisionObjectsTransform", cast_maps, "pose1"_a, "pose2"_a)
        .def("setCollisionObjectsTransformCast", cast_name, "name"_a, "pose1"_a, "pose2"_a)
        .def("setCollisionObjectsTransformCast", cast_names, "names"_a, "pose1"_a, "pose2"_a)
        .def("setCollisionObjectsTransformCast", cast_maps, "pose1"_a, "pose2"_a)
        .def("getCollisionObjects", &tc::ContinuousContactManager::getCollisionObjects)
        .def("setActiveCollisionObjects", &tc::ContinuousContactManager::setActiveCollisionObjects, "names"_a)
        .def("getActiveCollisionObjects", &tc::ContinuousContactManager::getActiveCollisionObjects)
        .def("setCollisionMarginData", &tc::ContinuousContactManager::setCollisionMarginData,
             "collision_margin_data"_a)
        .def("getCollisionMarginData", &tc::ContinuousContactManager::getCollisionMarginData)
        // Note: 0.33 renamed setDefaultCollisionMarginData → setDefaultCollisionMargin
        .def("setDefaultCollisionMargin", &tc::ContinuousContactManager::setDefaultCollisionMargin,
             "default_collision_margin"_a)
        // Note: 0.33 renamed setPairCollisionMarginData → setCollisionMarginPair
        .def("setCollisionMarginPair", &tc::ContinuousContactManager::setCollisionMarginPair,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("setCollisionMarginPairData", &tc::ContinuousContactManager::setCollisionMarginPairData,
             "pair_margin_data"_a,
             "override_type"_a = tcommon::CollisionMarginPairOverrideType::REPLACE)
        .def("incrementCollisionMargin", &tc::ContinuousContactManager::incrementCollisionMargin, "increment"_a)
        .def("setContactAllowedValidator", &tc::ContinuousContactManager::setContactAllowedValidator, "validator"_a)
        .def("getContactAllowedValidator", &tc::ContinuousContactManager::getContactAllowedValidator)
        .def("applyContactManagerConfig", &tc::ContinuousContactManager::applyContactManagerConfig, "config"_a)
        // Backwards compatibility aliases
        .def("setDefaultCollisionMarginData", &tc::ContinuousContactManager::setDefaultCollisionMargin,
             "default_collision_margin"_a)
        .def("setPairCollisionMarginData", &tc::ContinuousContactManager::setCollisionMarginPair,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("contactTest", &tc::ContinuousContactManager::contactTest, "collisions"_a, "request"_a, nb::call_guard<nb::gil_scoped_release>())  // GIL released (see discrete contactTest above)
        .def("clone", [](const tc::ContinuousContactManager& self) { return self.clone(); });

    // ========== ContactManagersPluginFactory ==========
    // Note: PluginLoader copy/move ctors aren't exported from library - cannot use nb::class_.
    // Use a wrapper struct with shared_ptr to avoid needing copy/move semantics.
    struct ContactManagersPluginFactoryWrapper {
        std::shared_ptr<tc::ContactManagersPluginFactory> ptr;

        ContactManagersPluginFactoryWrapper() : ptr(std::make_shared<tc::ContactManagersPluginFactory>()) {}
        // StrictPath (tesseract_nb.h): a `str` is always YAML content, never a path
        ContactManagersPluginFactoryWrapper(const tesseract_nb::StrictPath& config_path, const tcommon::ResourceLocator& locator)
            : ptr(std::make_shared<tc::ContactManagersPluginFactory>(config_path.value, locator)) {}
        ContactManagersPluginFactoryWrapper(const std::string& config, const tcommon::ResourceLocator& locator)
            : ptr(std::make_shared<tc::ContactManagersPluginFactory>(config, locator)) {}

        void addSearchPath(const std::string& path) { ptr->addSearchPath(path); }
        std::vector<std::string> getSearchPaths() const { return ptr->getSearchPaths(); }
        void clearSearchPaths() { ptr->clearSearchPaths(); }
        void addSearchLibrary(const std::string& lib) { ptr->addSearchLibrary(lib); }
        std::vector<std::string> getSearchLibraries() const { return ptr->getSearchLibraries(); }
        void clearSearchLibraries() { ptr->clearSearchLibraries(); }

        void addDiscreteContactManagerPlugin(const std::string& name, tcommon::PluginInfo info) {
            ptr->addDiscreteContactManagerPlugin(name, std::move(info));
        }
        tcommon::PluginInfoMap getDiscreteContactManagerPlugins() const { return ptr->getDiscreteContactManagerPlugins(); }
        // Upstream throws std::runtime_error for an unknown name; a name lookup that misses is a KeyError here.
        void removeDiscreteContactManagerPlugin(const std::string& name) {
            require_plugin(ptr->getDiscreteContactManagerPlugins(), "discrete", name);
            ptr->removeDiscreteContactManagerPlugin(name);
        }
        void setDefaultDiscreteContactManagerPlugin(const std::string& name) {
            require_plugin(ptr->getDiscreteContactManagerPlugins(), "discrete", name);
            ptr->setDefaultDiscreteContactManagerPlugin(name);
        }
        bool hasDiscreteContactManagerPlugins() const { return ptr->hasDiscreteContactManagerPlugins(); }
        std::string getDefaultDiscreteContactManagerPlugin() const { return ptr->getDefaultDiscreteContactManagerPlugin(); }

        void addContinuousContactManagerPlugin(const std::string& name, tcommon::PluginInfo info) {
            ptr->addContinuousContactManagerPlugin(name, std::move(info));
        }
        tcommon::PluginInfoMap getContinuousContactManagerPlugins() const { return ptr->getContinuousContactManagerPlugins(); }
        void removeContinuousContactManagerPlugin(const std::string& name) {
            require_plugin(ptr->getContinuousContactManagerPlugins(), "continuous", name);
            ptr->removeContinuousContactManagerPlugin(name);
        }
        void setDefaultContinuousContactManagerPlugin(const std::string& name) {
            require_plugin(ptr->getContinuousContactManagerPlugins(), "continuous", name);
            ptr->setDefaultContinuousContactManagerPlugin(name);
        }
        bool hasContinuousContactManagerPlugins() const { return ptr->hasContinuousContactManagerPlugins(); }
        std::string getDefaultContinuousContactManagerPlugin() const { return ptr->getDefaultContinuousContactManagerPlugin(); }

        std::unique_ptr<tc::DiscreteContactManager> createDiscreteContactManager(const std::string& name) const {
            return ptr->createDiscreteContactManager(name);
        }
        std::unique_ptr<tc::DiscreteContactManager>
        createDiscreteContactManager(const std::string& name, const tcommon::PluginInfo& info) const {
            return ptr->createDiscreteContactManager(name, info);
        }
        std::unique_ptr<tc::ContinuousContactManager> createContinuousContactManager(const std::string& name) const {
            return ptr->createContinuousContactManager(name);
        }
        std::unique_ptr<tc::ContinuousContactManager>
        createContinuousContactManager(const std::string& name, const tcommon::PluginInfo& info) const {
            return ptr->createContinuousContactManager(name, info);
        }

        // Same YAML as upstream saveConfig, which ignores a failed ofstream; here a failed open or
        // write raises OSError (FileNotFoundError for a missing directory).
        void saveConfig(const std::filesystem::path& file_path) const {
            errno = 0;
            std::ofstream fout(file_path);
            if (fout) fout << ptr->getConfig();
            if (fout) fout.close();
            if (!fout) {
                PyErr_SetFromErrnoWithFilename(PyExc_OSError, file_path.string().c_str());
                throw nb::python_error();
            }
        }
        // YAML::Node has no caster: the YAML document as str, like PluginInfo.config.
        std::string getConfig() const {
            YAML::Emitter out;
            out << ptr->getConfig();
            return out.c_str();
        }

    private:
        static void require_plugin(const tcommon::PluginInfoMap& plugins, const char* kind, const std::string& name) {
            if (plugins.find(name) == plugins.end())
                throw nb::key_error(("no " + std::string(kind) + " contact manager plugin '" + name + "'").c_str());
        }
    };

    using W = ContactManagersPluginFactoryWrapper;
    nb::class_<ContactManagersPluginFactoryWrapper>(m, "ContactManagersPluginFactory")
        .def(nb::init<>())
        .def(nb::init<const tesseract_nb::StrictPath&, const tcommon::ResourceLocator&>(), "config_path"_a, "locator"_a)
        .def(nb::init<const std::string&, const tcommon::ResourceLocator&>(), "config"_a, "locator"_a)
        .def("addSearchPath", &W::addSearchPath, "path"_a)
        .def("getSearchPaths", &W::getSearchPaths)
        .def("clearSearchPaths", &W::clearSearchPaths)
        .def("addSearchLibrary", &W::addSearchLibrary, "library_name"_a)
        .def("getSearchLibraries", &W::getSearchLibraries)
        .def("clearSearchLibraries", &W::clearSearchLibraries)
        .def("addDiscreteContactManagerPlugin", &W::addDiscreteContactManagerPlugin, "name"_a, "plugin_info"_a)
        .def("getDiscreteContactManagerPlugins", &W::getDiscreteContactManagerPlugins)
        .def("removeDiscreteContactManagerPlugin", &W::removeDiscreteContactManagerPlugin, "name"_a)
        .def("setDefaultDiscreteContactManagerPlugin", &W::setDefaultDiscreteContactManagerPlugin, "name"_a)
        .def("hasDiscreteContactManagerPlugins", &W::hasDiscreteContactManagerPlugins)
        .def("getDefaultDiscreteContactManagerPlugin", &W::getDefaultDiscreteContactManagerPlugin)
        .def("addContinuousContactManagerPlugin", &W::addContinuousContactManagerPlugin, "name"_a, "plugin_info"_a)
        .def("getContinuousContactManagerPlugins", &W::getContinuousContactManagerPlugins)
        .def("removeContinuousContactManagerPlugin", &W::removeContinuousContactManagerPlugin, "name"_a)
        .def("setDefaultContinuousContactManagerPlugin", &W::setDefaultContinuousContactManagerPlugin, "name"_a)
        .def("hasContinuousContactManagerPlugins", &W::hasContinuousContactManagerPlugins)
        .def("getDefaultContinuousContactManagerPlugin", &W::getDefaultContinuousContactManagerPlugin)
        // Managers run code from plugin libraries the factory's PluginLoader owns: keep the
        // factory alive as long as a manager lives (gh-72).
        .def("createDiscreteContactManager",
             nb::overload_cast<const std::string&>(&W::createDiscreteContactManager, nb::const_),
             "name"_a, nb::keep_alive<0, 1>())
        .def("createDiscreteContactManager",
             nb::overload_cast<const std::string&, const tcommon::PluginInfo&>(&W::createDiscreteContactManager,
                                                                              nb::const_),
             "name"_a, "plugin_info"_a, nb::keep_alive<0, 1>())
        .def("createContinuousContactManager",
             nb::overload_cast<const std::string&>(&W::createContinuousContactManager, nb::const_),
             "name"_a, nb::keep_alive<0, 1>())
        .def("createContinuousContactManager",
             nb::overload_cast<const std::string&, const tcommon::PluginInfo&>(&W::createContinuousContactManager,
                                                                              nb::const_),
             "name"_a, "plugin_info"_a, nb::keep_alive<0, 1>())
        .def("saveConfig", &W::saveConfig, "file_path"_a)
        .def("getConfig", &W::getConfig, "The factory configuration as a YAML document string.");

    // ========== Convex hulls ==========
    m.def("makeConvexMesh", &tc::makeConvexMesh, "mesh"_a,
          "Create a ConvexMesh from a Mesh using bullet's convex hull algorithm");

    nb::exception<ConvexHullError>(m, "ConvexHullError", PyExc_RuntimeError)
        .attr("__doc__") = "createConvexHull failed: Bullet could not apply the requested shrink.";

    // Out-param rule: (vertices, faces) are returned with the result n (face count).
    m.def(
        "createConvexHull",
        [](const tcommon::VectorVector3d& input, double shrink, double shrink_clamp) {
            tcommon::VectorVector3d vertices;
            Eigen::VectorXi faces;
            const int n = tc::createConvexHull(vertices, faces, input, shrink, shrink_clamp);
            if (n < 0)
                throw ConvexHullError("createConvexHull: Bullet convex hull computation failed (returned " +
                                      std::to_string(n) + ") for " + std::to_string(input.size()) +
                                      " input points with shrink=" + std::to_string(shrink) +
                                      ", shrink_clamp=" + std::to_string(shrink_clamp));
            return std::make_tuple(n, std::move(vertices), std::move(faces));
        },
        "input"_a, "shrink"_a = -1.0, "shrink_clamp"_a = -1.0,
        "Convex hull of a point set (Bullet). Returns (n_faces, vertices, faces); faces is a flat array of "
        "[count, i0, i1, ...] per face. shrink > 0 moves each face inwards by that amount; shrink_clamp > 0 "
        "caps it at shrink_clamp times the hull's inner radius. Raises ConvexHullError when the shrink "
        "cannot be applied.");

    // ========== Convex decomposition (V-HACD) ==========
    nb::exception<MalformedFacesError>(m, "MalformedFacesError", PyExc_ValueError)
        .attr("__doc__") = "A faces array has a count running past its end or an index outside the vertices.";
    nb::exception<NonTriangleFaceError>(m, "NonTriangleFaceError", PyExc_ValueError)
        .attr("__doc__") = "ConvexDecomposition.compute got a face that is not a triangle (count != 3).";

    nb::enum_<VHACD::FillMode>(m, "FillMode")
        .value("FLOOD_FILL", VHACD::FillMode::FLOOD_FILL)
        .value("SURFACE_ONLY", VHACD::FillMode::SURFACE_ONLY)
        .value("RAYCAST_FILL", VHACD::FillMode::RAYCAST_FILL);

    nb::class_<tc::VHACDParameters>(m, "VHACDParameters")
        .def(nb::init<>())
        .def_rw("max_convex_hulls", &tc::VHACDParameters::max_convex_hulls)
        .def_rw("resolution", &tc::VHACDParameters::resolution)
        .def_rw("minimum_volume_percent_error_allowed", &tc::VHACDParameters::minimum_volume_percent_error_allowed)
        .def_rw("max_recursion_depth", &tc::VHACDParameters::max_recursion_depth)
        .def_rw("shrinkwrap", &tc::VHACDParameters::shrinkwrap)
        .def_rw("fill_mode", &tc::VHACDParameters::fill_mode)
        .def_rw("max_num_vertices_per_ch", &tc::VHACDParameters::max_num_vertices_per_ch)
        .def_rw("async_ACD", &tc::VHACDParameters::async_ACD)
        .def_rw("min_edge_length", &tc::VHACDParameters::min_edge_length)
        .def_rw("find_best_plane", &tc::VHACDParameters::find_best_plane)
        .def("print", &tc::VHACDParameters::print, "Print the parameters to stdout.");

    // Abstract: no constructor. compute dispatches virtually to the concrete class. The GIL is
    // released only around the native call: arguments are converted and checked before it, the
    // ConvexMesh results converted after it.
    nb::class_<tc::ConvexDecomposition>(m, "ConvexDecomposition")
        .def(
            "compute",
            [](const tc::ConvexDecomposition& self, const tcommon::VectorVector3d& vertices,
               const Eigen::VectorXi& faces, bool verbose) {
                check_triangle_faces(vertices, faces);
                nb::gil_scoped_release release;
                return self.compute(vertices, faces, verbose);
            },
            "vertices"_a, "faces"_a, "verbose"_a = true,
            "Split a triangle mesh into convex hulls. faces is a flat array of [3, i0, i1, i2] per triangle. "
            "Raises NonTriangleFaceError for any other count, MalformedFacesError for a count running past "
            "the end or an index outside vertices.");

    nb::class_<tc::ConvexDecompositionVHACD, tc::ConvexDecomposition>(m, "ConvexDecompositionVHACD")
        .def(nb::init<>())
        .def(nb::init<const tc::VHACDParameters&>(), "params"_a);
}
