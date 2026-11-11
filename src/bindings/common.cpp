#include "tesseract_nb.h"
#include "common_bindings.h"
#include <nanobind/stl/string_view.h>  // PropertyTree.at / operator[] keys
#include <algorithm>
#include <iterator>
#include <limits>

// tesseract_common headers (need eigen_types.h before opaque declarations)
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>


// [joint units: rad or m] satisfiesLimits' scalar max_diff default (kinematic_limits.h): a default
// argument, which C++ exposes as no symbol, so it is restated here for the keyword.
constexpr double SATISFIES_LIMITS_DEFAULT_MAX_DIFF = 1e-6;
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/calibration_info.h>
#include <tesseract/common/contact_allowed_validator.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/utils.h>
#include <yaml-cpp/yaml.h>  // YAML::Load / YAML::Node for PluginInfo.config <-> str
#include <cmath>
#include <filesystem>
#include <sstream>
#include <utility>

#include <algorithm>

// boost::uuids::uuid <-> str, the spelling TaskComposerNodeInfo.uuid uses
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>

// GeneralResourceLocator's default `environment_variables`, copied from resource_locator.h:93/:103
static const std::vector<std::string> GENERAL_RESOURCE_LOCATOR_DEFAULT_ENV_VARS = {
    "TESSERACT_RESOURCE_PATH", "ROS_PACKAGE_PATH", "AMENT_PREFIX_PATH"};

// isWithinLimits / enforceLimits never check values.size() == limits.rows(), and the release
// build compiles out Eigen's size assertion, so a mismatch reads past the shorter operand.
struct LimitsSizeMismatchError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

static void check_limits_size(const Eigen::Ref<const Eigen::VectorXd>& values,
                              const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>& limits) {
    if (values.size() != limits.rows())
        throw LimitsSizeMismatchError("values has size " + std::to_string(values.size()) + " but limits has " +
                                      std::to_string(limits.rows()) + " rows");
}

// [components] a twist / transform error: 3 linear + 3 angular
constexpr Eigen::Index TWIST_SIZE = 6;

// applyTolerances throws a bare std::runtime_error on a size mismatch; the tolerance overloads
// of calcJacobianTransformErrorDiff state a size-6 contract (utils.h:150) without saying what a
// breach does. The bindings pre-check both, so each fails the same, named way.
struct ToleranceSizeMismatchError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// Passes when both tolerances are empty, or both have size n.
static void check_tolerance_size(Eigen::Index n, const Eigen::Ref<const Eigen::VectorXd>& lower,
                                 const Eigen::Ref<const Eigen::VectorXd>& upper) {
    const bool both_empty = lower.size() == 0 && upper.size() == 0;
    if (!both_empty && (lower.size() != n || upper.size() != n))
        throw ToleranceSizeMismatchError("lower_tolerance and upper_tolerance must both be empty or both have size " +
                                         std::to_string(n) + "; got " + std::to_string(lower.size()) + " and " +
                                         std::to_string(upper.size()));
}

// list.insert index: negative counts from the end; out of range clamps to either end.
static std::size_t insert_index(Py_ssize_t i, std::size_t size) {
    const auto n = static_cast<Py_ssize_t>(size);
    const Py_ssize_t j = i < 0 ? i + n : i;
    return static_cast<std::size_t>(std::clamp<Py_ssize_t>(j, 0, n));
}

// Python index (negative counts from the end) -> container index; std::out_of_range -> IndexError.
static std::size_t normalize_index(Py_ssize_t i, std::size_t size) {
    const auto n = static_cast<Py_ssize_t>(size);
    const Py_ssize_t j = i < 0 ? i + n : i;
    if (j < 0 || j >= n)
        throw std::out_of_range("index " + std::to_string(i) + " out of range for size " + std::to_string(size));
    return static_cast<std::size_t>(j);
}

// Trampoline class for ResourceLocator
class PyResourceLocator : public tesseract::common::ResourceLocator {
public:
    NB_TRAMPOLINE(tesseract::common::ResourceLocator, 1);

    std::shared_ptr<tesseract::common::Resource> locateResource(const std::string& url) const override {
        NB_OVERRIDE_PURE(locateResource, url);
    }
};

// `a == b` for a NameId and any Python object: an id of the same kind compares by identity
// (operator==), a str by name, anything else is NotImplemented (Python then tries b.__eq__, and
// falls back to identity).
template <typename Id>
nb::object id_equals(const Id& a, nb::handle b) {
    if (nb::isinstance<Id>(b))
        return nb::bool_(a == nb::cast<const Id&>(b));
    if (nb::isinstance<nb::str>(b))
        return nb::bool_(a.name() == nb::cast<std::string>(b));
    return nb::borrow(Py_NotImplemented);
}

// The negation of an id_equals result; NotImplemented passes through.
inline nb::object negate(nb::object equal) {
    if (equal.is(Py_NotImplemented))
        return equal;
    return nb::bool_(!nb::cast<bool>(equal));
}

// LinkId / JointId: upstream's hash-of-name identity (tesseract IDENTITY_DESIGN.md).
// Python sees the name: equality with str and hash(name) let an id-keyed dict answer d["tool0"].
// The C++ order (by hash) is not exposed; __lt__ sorts by name so Python sorting is deterministic.
template <typename Tag>
void bindNameId(nb::module_& m, const char* py_name) {
    using Id = tesseract::common::NameId<Tag>;
    nb::class_<Id>(m, py_name)
        .def(nb::init<>())
        .def(nb::init<std::string>(), "name"_a)
        .def("name", &Id::name)
        .def("value", &Id::value)
        .def("isValid", &Id::isValid)
        // One function per operator: two overloads (Id, str) would each render `other: object`.
        .def("__eq__", [](const Id& a, nb::handle b) { return id_equals(a, b); }, TNB_EQ_ATTRS)
        .def("__ne__", [](const Id& a, nb::handle b) { return negate(id_equals(a, b)); }, TNB_NE_ATTRS)
        .def("__lt__", [](const Id& a, const Id& b) { return a.name() < b.name(); }, nb::is_operator())
        .def("__hash__", [](const Id& a) { return nb::hash(nb::str(a.name().c_str(), a.name().size())); })
        .def("__str__", &Id::name)
        .def("__repr__", [py_name](const Id& a) { return std::string(py_name) + "('" + a.name() + "')"; })
        .def("__getstate__", &Id::name)
        .def("__setstate__", [](Id& self, const std::string& name) { new (&self) Id(name); });
    nb::implicitly_convertible<std::string, Id>();
}

namespace {

YAML::Node load_yaml_text(const std::string& text) {
    try {
        return YAML::Load(text);
    } catch (const YAML::Exception& e) {
        throw nb::value_error(e.what());
    }
}

std::string dump_yaml(const YAML::Node& node) {
    YAML::Emitter out;
    out << node;
    return out.c_str();
}

void bind_property_tree(nb::module_& m) {
    using tesseract::common::PropertyTree;
    nb::class_<PropertyTree>(m, "PropertyTree",
        "A node of a configuration schema: a YAML value, metadata attributes and ordered children.")
        .def(nb::init<>())
        .def("applyConfig",
             [](PropertyTree& self, const std::string& config, bool allow_extra_properties) {
                 return self.applyConfig(load_yaml_text(config), allow_extra_properties);
             },
             "config"_a, "allow_extra_properties"_a = false,
             "Merge the YAML config text into the schema and validate it; returns every error, empty "
             "when valid. A key the schema does not declare is an error unless allow_extra_properties. "
             "Malformed YAML raises ValueError.")
        .def("validate", &PropertyTree::validate, "allow_extra_properties"_a = false,
             "Every validation error of the tree, empty when valid.")
        .def("at", nb::overload_cast<std::string_view>(&PropertyTree::at, nb::const_), "key"_a,
             nb::rv_policy::reference_internal, "The child at key; raises IndexError when absent.")
        .def("__getitem__",
             [](const PropertyTree& self, std::string_view key) -> const PropertyTree& {
                 const PropertyTree* child = self.find(key);
                 if (child == nullptr)
                     throw nb::key_error(std::string(key).c_str());
                 return *child;
             },
             "key"_a, nb::rv_policy::reference_internal, "The child at key; raises KeyError when absent.")
        .def("find", nb::overload_cast<std::string_view>(&PropertyTree::find, nb::const_), "key"_a,
             nb::rv_policy::reference_internal, "The child at key, or None.")
        .def("__contains__", [](const PropertyTree& self, std::string_view key) { return self.find(key) != nullptr; },
             "key"_a)
        .def("setValue", [](PropertyTree& self, const std::string& value) { self.setValue(load_yaml_text(value)); },
             "value"_a, "Set the value from YAML text.")
        .def("getValue", [](const PropertyTree& self) { return dump_yaml(self.getValue()); },
             "The value as YAML text.")
        .def("isNull", &PropertyTree::isNull)
        .def("isContainer", &PropertyTree::isContainer)
        .def("keys", &PropertyTree::keys, "The child keys, in insertion order.")
        .def("setAttribute",
             [](PropertyTree& self, std::string_view name, const std::string& attr) {
                 self.setAttribute(name, std::string_view(attr));
             },
             "name"_a, "attr"_a)
        .def("setAttribute", nb::overload_cast<std::string_view, bool>(&PropertyTree::setAttribute), "name"_a,
             "attr"_a)
        .def("setAttribute", nb::overload_cast<std::string_view, int>(&PropertyTree::setAttribute), "name"_a,
             "attr"_a)
        .def("setAttribute", nb::overload_cast<std::string_view, double>(&PropertyTree::setAttribute), "name"_a,
             "attr"_a)
        .def("setAttribute",
             nb::overload_cast<std::string_view, const std::vector<std::string>&>(&PropertyTree::setAttribute),
             "name"_a, "attr"_a)
        .def("hasAttribute", &PropertyTree::hasAttribute, "name"_a)
        .def("getAttribute",
             [](const PropertyTree& self, std::string_view name) -> std::optional<std::string> {
                 const auto attr = self.getAttribute(name);
                 if (!attr)
                     return std::nullopt;
                 return dump_yaml(*attr);
             },
             "name"_a, "The attribute as YAML text, or None.")
        .def("getAttributeKeys", &PropertyTree::getAttributeKeys)
        .def("isRequired", &PropertyTree::isRequired)
        .def_static("fromYAML", [](const std::string& text) { return PropertyTree::fromYAML(load_yaml_text(text)); },
                    "text"_a, "A tree from the YAML that toYAML(exclude_attributes=False) writes.")
        .def("toYAML", [](const PropertyTree& self, bool exclude_attributes) {
                 return dump_yaml(self.toYAML(exclude_attributes));
             },
             "exclude_attributes"_a = true, "The tree as YAML text.")
        .def("size", &PropertyTree::size)
        .def("__len__", &PropertyTree::size)
        .def("empty", &PropertyTree::empty)
        .def("__bool__", [](const PropertyTree& self) { return static_cast<bool>(self); })
        .def("__str__", [](const PropertyTree& self) {
            std::ostringstream os;
            os << self;
            return os.str();
        });
}

}  // namespace

NB_MODULE(_common, m) {
    m.doc() = "tesseract_common Python bindings (nanobind)";
    register_python_error_translator();  // ResourceLocator trampoline

    // ========== Identity types (bound first: every link/joint-addressing API below takes them) ==========
    bindNameId<tesseract::common::LinkTag>(m, "LinkId");
    bindNameId<tesseract::common::JointTag>(m, "JointId");

    using tesseract::common::LinkId;
    using tesseract::common::LinkIdPair;
    nb::class_<LinkIdPair>(m, "LinkIdPair")
        .def(nb::init<const LinkId&, const LinkId&>(), "link_id1"_a, "link_id2"_a)
        .def("__init__", [](LinkIdPair* self, const std::pair<LinkId, LinkId>& p) {
            new (self) LinkIdPair(p.first, p.second);
        }, "pair"_a)
        // first()/second() are in canonical (hash) order, not alphabetical; orderedNameView() is alphabetical.
        .def("first", &LinkIdPair::first, nb::rv_policy::copy)
        .def("second", &LinkIdPair::second, nb::rv_policy::copy)
        .def("orderedNameView", [](const LinkIdPair& self) {
            const auto view = self.orderedNameView();
            return std::make_pair(std::string(view.first), std::string(view.second));
        })
        // noconvert: a tuple's hash is order-dependent, so a tuple must never compare equal to an unordered pair
        .def("__eq__", [](const LinkIdPair& a, const LinkIdPair& b) { return a == b; },
             nb::arg().noconvert(), TNB_EQ_ATTRS)
        .def("__ne__", [](const LinkIdPair& a, const LinkIdPair& b) { return !(a == b); },
             nb::arg().noconvert(), TNB_NE_ATTRS)
        .def("__hash__", [](const LinkIdPair& self) {
            const auto view = self.orderedNameView();
            return nb::hash(nb::make_tuple(view.first, view.second));
        })
        .def("__repr__", [](const LinkIdPair& self) {
            const auto view = self.orderedNameView();
            return "LinkIdPair('" + view.first + "', '" + view.second + "')";
        });
    nb::implicitly_convertible<std::pair<LinkId, LinkId>, LinkIdPair>();

    bind_eigen_geometry(m);  // common_eigen.cpp

    // ========== ResourceLocator Hierarchy ==========
    // Registered before Resource, which derives from it (resource_locator.h:146).
    nb::class_<tesseract::common::ResourceLocator, PyResourceLocator>(m, "ResourceLocator")
        .def(nb::init<>())
        // nullptr (None) when the url is not found
        .def("locateResource", &tesseract::common::ResourceLocator::locateResource, "url"_a,
             nb::sig("def locateResource(self, url: str) -> Resource | None"));

    // Both non-default ctors are keyword-only: nanobind's std::filesystem::path caster accepts
    // a plain `str`, so a positional list[str] would match both vector<string> (env-var names)
    // and vector<path> (directories), and registration order would silently pick one.
    // operator== compares the package paths found so far.
    auto general_resource_locator = nb::class_<tesseract::common::GeneralResourceLocator, tesseract::common::ResourceLocator>(m, "GeneralResourceLocator")
        .def(nb::init<>())
        .def(nb::init<const std::vector<std::string>&>(), nb::kw_only(), "environment_variables"_a)
        .def(nb::init<const std::vector<std::filesystem::path>&, const std::vector<std::string>&>(),
             nb::kw_only(), "paths"_a, "environment_variables"_a = GENERAL_RESOURCE_LOCATOR_DEFAULT_ENV_VARS)
        .def("addPath", &tesseract::common::GeneralResourceLocator::addPath, "path"_a)
        .def("loadEnvironmentVariable", &tesseract::common::GeneralResourceLocator::loadEnvironmentVariable,
             "environment_variable"_a);
    bind_value_equality(general_resource_locator);

    // ========== Resource Types ==========
    // Note: In nanobind 2.x, shared_ptr holder is automatic - don't specify it
    nb::class_<tesseract::common::Resource, tesseract::common::ResourceLocator>(m, "Resource")
        .def("isFile", &tesseract::common::Resource::isFile)
        .def("getUrl", &tesseract::common::Resource::getUrl)
        .def("getFilePath", &tesseract::common::Resource::getFilePath)
        .def("getResourceContents", [](tesseract::common::Resource& self) {
            std::vector<uint8_t> data = self.getResourceContents();
            return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
        })
        // std::istream has no Python counterpart; hand back the contents as io.BytesIO
        .def("getResourceContentStream", [](tesseract::common::Resource& self) {
            std::shared_ptr<std::istream> stream = self.getResourceContentStream();
            std::string data{std::istreambuf_iterator<char>(*stream), std::istreambuf_iterator<char>()};
            return nb::module_::import_("io").attr("BytesIO")(nb::bytes(data.data(), data.size()));
        }, nb::sig("def getResourceContentStream(self) -> io.BytesIO"));

    // `parent` resolves relative urls: BytesResource.locateResource asks it for the url as
    // given, then for the sibling of its own url. The raw-pointer ctor (h:251) stays unbound;
    // the `bytes` overload covers it.
    // operator== (resource_locator.cpp): url, bytes, and whether a parent is set. Two parents
    // always compare equal, because ResourceLocator::operator== is `return true`.
    auto bytes_resource = nb::class_<tesseract::common::BytesResource, tesseract::common::Resource>(m, "BytesResource")
        .def(nb::init<std::string, std::vector<uint8_t>, std::shared_ptr<tesseract::common::ResourceLocator>>(),
             "url"_a, "bytes"_a, "parent"_a.none() = nb::none())
        .def("__init__", [](tesseract::common::BytesResource* self, std::string url, nb::bytes data,
                            std::shared_ptr<tesseract::common::ResourceLocator> parent) {
            std::vector<uint8_t> vec(data.size());
            std::memcpy(vec.data(), data.c_str(), data.size());
            new (self) tesseract::common::BytesResource(std::move(url), std::move(vec), std::move(parent));
        }, "url"_a, "bytes"_a, "parent"_a.none() = nb::none());
    bind_value_equality(bytes_resource);

    // operator==: url, filename, and whether a parent is set (see BytesResource).
    auto simple_located_resource = nb::class_<tesseract::common::SimpleLocatedResource, tesseract::common::Resource>(m, "SimpleLocatedResource")
        .def(nb::init<const std::string&, const std::string&>(), "url"_a, "filename"_a)
        .def(nb::init<const std::string&, const std::string&, std::shared_ptr<tesseract::common::ResourceLocator>>(),
             "url"_a, "filename"_a, "parent"_a);
    bind_value_equality(simple_located_resource);

    // ========== ManipulatorInfo ==========
    // tcp_offset is std::variant<std::string, Eigen::Isometry3d>: a link name or a pose. The ctor
    // and the property share nanobind's variant caster, so any other type raises TypeError. The
    // getter returns a copy, not def_rw's reference_internal: a Python Isometry3d pointing into
    // the variant would dangle once the field is set to a str.
    using TcpOffset = std::variant<LinkId, Eigen::Isometry3d>;
    auto manipulator_info = nb::class_<tesseract::common::ManipulatorInfo>(m, "ManipulatorInfo")
        .def(nb::init<>())
        .def(nb::init<std::string, LinkId, LinkId, TcpOffset>(),
             "manipulator"_a, "working_frame"_a, "tcp_frame"_a,
             "tcp_offset"_a = Eigen::Isometry3d::Identity())
        .def_rw("manipulator", &tesseract::common::ManipulatorInfo::manipulator)
        .def_rw("manipulator_ik_solver", &tesseract::common::ManipulatorInfo::manipulator_ik_solver)
        .def_rw("working_frame", &tesseract::common::ManipulatorInfo::working_frame, nb::rv_policy::copy)
        .def_rw("tcp_frame", &tesseract::common::ManipulatorInfo::tcp_frame, nb::rv_policy::copy)
        .def_prop_rw("tcp_offset",
            [](const tesseract::common::ManipulatorInfo& self) -> TcpOffset { return self.tcp_offset; },
            [](tesseract::common::ManipulatorInfo& self, TcpOffset value) { self.tcp_offset = std::move(value); },
            "TCP offset: a link (LinkId, or its name as str) or a pose (Isometry3d). Reading returns a copy.")
        // Copy of self with every non-empty field of the override; an overriding tcp_frame
        // brings its tcp_offset along (manipulator_info.cpp).
        .def("getCombined", &tesseract::common::ManipulatorInfo::getCombined, "manip_info_override"_a)
        // True unless manipulator, working_frame and tcp_frame are all set.
        .def("empty", &tesseract::common::ManipulatorInfo::empty)
        .def("__repr__", [](const tesseract::common::ManipulatorInfo& self) {
            return "<ManipulatorInfo manipulator='" + self.manipulator + "'>";
        });
    bind_value_equality(manipulator_info);

    // ========== JointState ==========
    // operator== (joint_state.cpp): names exactly, vectors by size then isApprox(1e-5), time
    // with almostEqualRelativeAndAbs(1e-5).
    auto joint_state = nb::class_<tesseract::common::JointState>(m, "JointState")
        .def(nb::init<>())
        .def(nb::init<std::vector<tesseract::common::JointId>, const Eigen::VectorXd&>(), "joint_ids"_a, "position"_a)
        .def_rw("joint_ids", &tesseract::common::JointState::joint_ids, nb::rv_policy::copy)
        .def_rw("position", &tesseract::common::JointState::position)
        .def_rw("velocity", &tesseract::common::JointState::velocity)
        .def_rw("acceleration", &tesseract::common::JointState::acceleration)
        .def_rw("effort", &tesseract::common::JointState::effort)
        .def_rw("time", &tesseract::common::JointState::time);
    bind_value_equality(joint_state);

    // ========== JointTrajectory ==========
    // Element access returns a copy: a reference into `states` would dangle once push_back
    // reallocates. Edit an element with a write-back, `traj[i] = js`. `insert` and `erase` take
    // Python's list forms, `insert(index, state)` and `del traj[i]` / `del traj[a:b]`. The other
    // iterator-taking vector members (emplace, the range ctor, reverse iterators, data, swap)
    // have no natural Python signature and stay unbound.
    using tesseract::common::JointState;
    using tesseract::common::JointTrajectory;
    auto joint_trajectory = nb::class_<JointTrajectory>(m, "JointTrajectory")
        .def(nb::init<std::string>(), "description"_a = "")
        .def(nb::init<std::vector<JointState>, std::string>(), "states"_a, "description"_a = "")
        .def_rw("states", &JointTrajectory::states)
        .def_rw("description", &JointTrajectory::description)
        .def_prop_rw("uuid",
            [](const JointTrajectory& self) { return boost::uuids::to_string(self.uuid); },
            [](JointTrajectory& self, const std::string& s) {
                try {
                    self.uuid = boost::uuids::string_generator()(s);
                } catch (const std::runtime_error&) {
                    throw std::invalid_argument("JointTrajectory.uuid: not a UUID string: '" + s + "'");
                }
            },
            "UUID as its canonical string; a malformed string raises ValueError.")
        .def("__len__", &JointTrajectory::size)
        .def("__getitem__", [](const JointTrajectory& self, Py_ssize_t i) {
            return self[normalize_index(i, self.size())];
        }, "index"_a, nb::rv_policy::copy, "A copy of the state at `index`; write back with `traj[index] = js`.")
        .def("__setitem__", [](JointTrajectory& self, Py_ssize_t i, const JointState& state) {
            self[normalize_index(i, self.size())] = state;
        }, "index"_a, "state"_a)
        .def("__iter__", [](const JointTrajectory& self) {
            return nb::make_iterator<nb::rv_policy::copy>(nb::type<JointTrajectory>(), "JointTrajectoryIterator",
                                                          self.begin(), self.end());
        }, nb::keep_alive<0, 1>(), "Iterate over copies of the states.")
        .def("empty", &JointTrajectory::empty)
        .def("max_size", &JointTrajectory::max_size)
        .def("reserve", &JointTrajectory::reserve, "n"_a)
        .def("capacity", &JointTrajectory::capacity)
        .def("shrink_to_fit", &JointTrajectory::shrink_to_fit)
        // front/back/pop_back on an empty std::vector are undefined behaviour: check first.
        .def("front", [](const JointTrajectory& self) {
            if (self.empty()) throw std::out_of_range("JointTrajectory.front: empty trajectory");
            return self.front();
        }, nb::rv_policy::copy)
        .def("back", [](const JointTrajectory& self) {
            if (self.empty()) throw std::out_of_range("JointTrajectory.back: empty trajectory");
            return self.back();
        }, nb::rv_policy::copy)
        .def("at", nb::overload_cast<JointTrajectory::size_type>(&JointTrajectory::at, nb::const_), "n"_a,
             nb::rv_policy::copy)
        .def("clear", &JointTrajectory::clear)
        // A lambda, not overload_cast: GCC cannot pick the const& overload beside push_back(const T&&).
        .def("push_back", [](JointTrajectory& self, const JointState& x) { self.push_back(x); }, "x"_a)
        .def("pop_back", [](JointTrajectory& self) {
            if (self.empty()) throw std::out_of_range("JointTrajectory.pop_back: empty trajectory");
            self.pop_back();
        })
        .def("insert", [](JointTrajectory& self, Py_ssize_t index, const JointState& state) {
            self.insert(self.begin() + static_cast<std::ptrdiff_t>(insert_index(index, self.size())), state);
        }, "index"_a, "state"_a)
        .def("__delitem__", [](JointTrajectory& self, Py_ssize_t i) {
            self.erase(self.begin() + static_cast<std::ptrdiff_t>(normalize_index(i, self.size())));
        }, "index"_a)
        .def("__delitem__", [](JointTrajectory& self, const nb::slice& slice) {
            const auto [start, stop, step, length] = slice.compute(self.size());
            if (step == 1) {
                self.erase(self.begin() + start, self.begin() + start + static_cast<Py_ssize_t>(length));
                return;
            }
            // Erase from the back, so the indices still to erase stay valid.
            std::vector<Py_ssize_t> indices;
            for (std::size_t k = 0; k < length; ++k)
                indices.push_back(start + static_cast<Py_ssize_t>(k) * step);
            std::sort(indices.rbegin(), indices.rend());
            for (const Py_ssize_t j : indices)
                self.erase(self.begin() + j);
        }, "slice"_a);
    bind_value_equality(joint_trajectory);

    // ========== AllowedCollisionMatrix ==========
    // The entries ctor orders each key (allowed_collision_matrix.cpp), so ("b", "a") is stored as ("a", "b").
    auto allowed_collision_matrix = nb::class_<tesseract::common::AllowedCollisionMatrix>(m, "AllowedCollisionMatrix")
        .def(nb::init<>())
        .def(nb::init<const tesseract::common::AllowedCollisionEntries&>(), "entries"_a)
        .def("addAllowedCollision",
             nb::overload_cast<const LinkId&, const LinkId&, const std::string&>(
                 &tesseract::common::AllowedCollisionMatrix::addAllowedCollision),
             "link_id1"_a, "link_id2"_a, "reason"_a)
        .def("addAllowedCollision",
             nb::overload_cast<const LinkIdPair&, const std::string&>(
                 &tesseract::common::AllowedCollisionMatrix::addAllowedCollision),
             "pair"_a, "reason"_a)
        .def("removeAllowedCollision",
             nb::overload_cast<const LinkId&, const LinkId&>(
                 &tesseract::common::AllowedCollisionMatrix::removeAllowedCollision),
             "link_id1"_a, "link_id2"_a)
        .def("removeAllowedCollision",
             nb::overload_cast<const LinkIdPair&>(&tesseract::common::AllowedCollisionMatrix::removeAllowedCollision),
             "pair"_a)
        // Removes every entry that involves `link_id`.
        .def("removeAllowedCollision",
             nb::overload_cast<const LinkId&>(&tesseract::common::AllowedCollisionMatrix::removeAllowedCollision),
             "link_id"_a)
        .def("isCollisionAllowed", &tesseract::common::AllowedCollisionMatrix::isCollisionAllowed, "pair"_a)
        .def("clearAllowedCollisions", &tesseract::common::AllowedCollisionMatrix::clearAllowedCollisions)
        .def("getAllAllowedCollisions", &tesseract::common::AllowedCollisionMatrix::getAllAllowedCollisions)
        .def("insertAllowedCollisionMatrix", &tesseract::common::AllowedCollisionMatrix::insertAllowedCollisionMatrix)
        .def("reserveAllowedCollisionMatrix", &tesseract::common::AllowedCollisionMatrix::reserveAllowedCollisionMatrix,
             "size"_a)
        // operator<< (h:108): one "link=<a> link=<b> reason=<r>" line per entry
        .def("__str__", [](const tesseract::common::AllowedCollisionMatrix& self) {
            std::ostringstream os;
            os << self;
            return os.str();
        });
    bind_value_equality(allowed_collision_matrix);

    // Links allowed to collide with any of `link_ids`, in acm_entries' (unordered) iteration order.
    m.def("getAllowedCollisions", &tesseract::common::getAllowedCollisions,
          "link_ids"_a, "acm_entries"_a, "remove_duplicates"_a = true);

    // ========== ContactAllowedValidator ==========
    // Abstract base. Determines whether two links are allowed to be in collision.
    nb::class_<tesseract::common::ContactAllowedValidator>(m, "ContactAllowedValidator")
        .def("__call__", &tesseract::common::ContactAllowedValidator::operator(), "pair"_a);

    // Validator backed by an AllowedCollisionMatrix
    nb::class_<tesseract::common::ACMContactAllowedValidator, tesseract::common::ContactAllowedValidator>(
        m, "ACMContactAllowedValidator")
        .def(nb::init<>())
        .def(nb::init<tesseract::common::AllowedCollisionMatrix>(), "acm"_a);

    nb::enum_<tesseract::common::CombinedContactAllowedValidatorType>(m, "CombinedContactAllowedValidatorType")
        .value("AND", tesseract::common::CombinedContactAllowedValidatorType::AND)
        .value("OR", tesseract::common::CombinedContactAllowedValidatorType::OR);

    // Validator combining multiple validators with an AND/OR operator
    nb::class_<tesseract::common::CombinedContactAllowedValidator, tesseract::common::ContactAllowedValidator>(
        m, "CombinedContactAllowedValidator")
        .def(nb::init<>())
        .def("__init__",
             [](tesseract::common::CombinedContactAllowedValidator* self,
                const std::vector<std::shared_ptr<const tesseract::common::ContactAllowedValidator>>& validators,
                tesseract::common::CombinedContactAllowedValidatorType type) {
                 new (self) tesseract::common::CombinedContactAllowedValidator(
                     require_non_null(validators, "CombinedContactAllowedValidator", "validators",
                                      "a ContactAllowedValidator"),
                     type);
             },
             "validators"_a, "type"_a);

    // ========== CollisionMarginData ==========
    nb::enum_<tesseract::common::CollisionMarginPairOverrideType>(m, "CollisionMarginPairOverrideType")
        .value("NONE", tesseract::common::CollisionMarginPairOverrideType::NONE)
        .value("REPLACE", tesseract::common::CollisionMarginPairOverrideType::REPLACE)
        .value("MODIFY", tesseract::common::CollisionMarginPairOverrideType::MODIFY);

    // operator==: margins with almostEqualRelativeAndAbs(1e-5).
    auto collision_margin_pair_data = nb::class_<tesseract::common::CollisionMarginPairData>(m, "CollisionMarginPairData")
        .def(nb::init<>())
        .def(nb::init<const tesseract::common::PairsCollisionMarginData&>(), "pair_margins"_a)
        .def("setCollisionMargin", &tesseract::common::CollisionMarginPairData::setCollisionMargin,
             "link_id1"_a, "link_id2"_a, "margin"_a)
        .def("getCollisionMargin", &tesseract::common::CollisionMarginPairData::getCollisionMargin, "pair"_a)
        .def("getCollisionMargins", &tesseract::common::CollisionMarginPairData::getCollisionMargins)
        .def("getMaxCollisionMargin",
             nb::overload_cast<>(&tesseract::common::CollisionMarginPairData::getMaxCollisionMargin, nb::const_),
             "Largest pair margin, or None when no pair margin is set.")
        .def("getMaxCollisionMargin",
             nb::overload_cast<const LinkId&>(&tesseract::common::CollisionMarginPairData::getMaxCollisionMargin,
                                              nb::const_),
             "obj_id"_a, "Largest pair margin involving `obj_id`, or None when no pair involves it.")
        .def("incrementMargins", &tesseract::common::CollisionMarginPairData::incrementMargins, "increment"_a)
        .def("scaleMargins", &tesseract::common::CollisionMarginPairData::scaleMargins, "scale"_a)
        .def("apply", &tesseract::common::CollisionMarginPairData::apply, "pair_margin_data"_a, "override_type"_a)
        .def("empty", &tesseract::common::CollisionMarginPairData::empty)
        .def("clear", &tesseract::common::CollisionMarginPairData::clear);
    bind_value_equality(collision_margin_pair_data);

    auto collision_margin_data = nb::class_<tesseract::common::CollisionMarginData>(m, "CollisionMarginData")
        .def(nb::init<>())
        .def(nb::init<double>())
        .def(nb::init<double, tesseract::common::CollisionMarginPairData>(), "default_collision_margin"_a,
             "pair_collision_margins"_a)
        .def(nb::init<tesseract::common::CollisionMarginPairData>(), "pair_collision_margins"_a)
        .def("getDefaultCollisionMargin", &tesseract::common::CollisionMarginData::getDefaultCollisionMargin)
        .def("setDefaultCollisionMargin", &tesseract::common::CollisionMarginData::setDefaultCollisionMargin)
        .def("getCollisionMargin",
             nb::overload_cast<const LinkId&, const LinkId&>(&tesseract::common::CollisionMarginData::getCollisionMargin, nb::const_),
             "link_id1"_a, "link_id2"_a)
        .def("getCollisionMargin",
             nb::overload_cast<const LinkIdPair&>(&tesseract::common::CollisionMarginData::getCollisionMargin, nb::const_),
             "pair"_a)
        .def("setCollisionMargin", &tesseract::common::CollisionMarginData::setCollisionMargin,
             "link_id1"_a, "link_id2"_a, "collision_margin"_a)
        .def("getCollisionMarginPairData", &tesseract::common::CollisionMarginData::getCollisionMarginPairData)
        .def("getMaxCollisionMargin", nb::overload_cast<>(&tesseract::common::CollisionMarginData::getMaxCollisionMargin, nb::const_))
        .def("getMaxCollisionMargin",
             nb::overload_cast<const LinkId&>(&tesseract::common::CollisionMarginData::getMaxCollisionMargin,
                                                   nb::const_),
             "obj_id"_a, "Largest margin involving `obj_id`: its largest pair margin or the default, whichever is larger.")
        .def("incrementMargins", &tesseract::common::CollisionMarginData::incrementMargins, "increment"_a)
        .def("scaleMargins", &tesseract::common::CollisionMarginData::scaleMargins, "scale"_a)
        .def("apply", &tesseract::common::CollisionMarginData::apply, "pair_margin_data"_a, "override_type"_a);
    bind_value_equality(collision_margin_data);

    // ========== KinematicLimits ==========
    using tesseract::common::KinematicLimits;
    auto kinematic_limits = nb::class_<KinematicLimits>(m, "KinematicLimits")
        .def(nb::init<>())
        .def_rw("joint_limits", &tesseract::common::KinematicLimits::joint_limits)
        .def_rw("velocity_limits", &tesseract::common::KinematicLimits::velocity_limits)
        .def_rw("acceleration_limits", &tesseract::common::KinematicLimits::acceleration_limits)
        .def_rw("jerk_limits", &tesseract::common::KinematicLimits::jerk_limits)
        // Resizes all four limit matrices to (size, 2) (Eigen resize: values unset after a size change).
        .def("resize", &tesseract::common::KinematicLimits::resize, "size"_a);
    // operator== (kinematic_limits.cpp) calls isApprox on the four limit matrices without comparing
    // their shapes, and Eigen's isApprox on mismatched shapes reads out of bounds when eigen_assert
    // is compiled out. Limits of different shapes are unequal, so compare the shapes first.
    // Otherwise as bind_value_equality: NotImplemented for a non-KinematicLimits operand, unhashable.
    const auto kinematic_limits_equal = [](const KinematicLimits& a, const KinematicLimits& b) {
        const auto same_shape = [](const auto& x, const auto& y) {
            return x.rows() == y.rows() && x.cols() == y.cols();
        };
        return same_shape(a.joint_limits, b.joint_limits) && same_shape(a.velocity_limits, b.velocity_limits) &&
               same_shape(a.acceleration_limits, b.acceleration_limits) &&
               same_shape(a.jerk_limits, b.jerk_limits) && a == b;
    };
    kinematic_limits
        .def("__eq__", kinematic_limits_equal, TNB_EQ_ATTRS)
        .def("__ne__", [kinematic_limits_equal](const KinematicLimits& a, const KinematicLimits& b) {
            return !kinematic_limits_equal(a, b);
        }, TNB_NE_ATTRS);
    kinematic_limits.attr("__hash__") = nb::none();

    // satisfiesLimits<double>: scalar-tolerance overload (with upstream's defaults) and
    // per-axis-tolerance overload. Defaults mirror kinematic_limits.h.
    using RefVectorXd = Eigen::Ref<const Eigen::VectorXd>;
    using RefLimits = Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 2>>;
    m.def("satisfiesLimits",
        [](const RefVectorXd& values, const RefLimits& limits, double max_diff, double max_rel_diff) {
            return tesseract::common::satisfiesLimits<double>(values, limits, max_diff, max_rel_diff);
        },
        "values"_a, "limits"_a,
        "max_diff"_a = SATISFIES_LIMITS_DEFAULT_MAX_DIFF,
        "max_rel_diff"_a = std::numeric_limits<double>::epsilon());
    m.def("satisfiesLimits",
        [](const RefVectorXd& values, const RefLimits& limits,
           const RefVectorXd& max_diff, const RefVectorXd& max_rel_diff) {
            return tesseract::common::satisfiesLimits<double>(values, limits, max_diff, max_rel_diff);
        },
        "values"_a, "limits"_a, "max_diff"_a, "max_rel_diff"_a);

    // isWithinLimits<double> / enforceLimits<double>: sizes checked first (LimitsSizeMismatchError).
    nb::exception<LimitsSizeMismatchError>(m, "LimitsSizeMismatchError", PyExc_ValueError)
        .attr("__doc__") = "isWithinLimits / enforceLimits got `values` and `limits` of different lengths.";
    m.def("isWithinLimits",
        [](const RefVectorXd& values, const RefLimits& limits) {
            check_limits_size(values, limits);
            return tesseract::common::isWithinLimits<double>(values, limits);
        },
        "values"_a, "limits"_a, "True if every value lies in its [lower, upper] row; no tolerance.");
    // Out-param rule: C++ clamps `values` in place; Python gets the clamped copy back.
    m.def("enforceLimits",
        [](const RefVectorXd& values, const RefLimits& limits) -> Eigen::VectorXd {
            check_limits_size(values, limits);
            Eigen::VectorXd clamped = values;
            tesseract::common::enforceLimits<double>(clamped, limits);
            return clamped;
        },
        "values"_a, "limits"_a, "`values` clamped into `limits`, as a new array; the input is unchanged.");

    // ========== Frame and error math (utils.h) ==========
    // In place, not out-param: like C++, the twist/jacobian/applyTolerances bindings write into
    // the array passed in and return None. They take writable refs, so nanobind refuses any array
    // it would have to convert (float32, read-only, non-contiguous, C-order (6, n > 1)) with
    // TypeError, instead of writing into a temporary copy. The 6-row shapes live in the type, so
    // a wrong-length twist fails at the call boundary; upstream does not check sizes and the
    // release build compiles out Eigen's assertions.
    using RefTwist = tesseract_nb::InPlace<Eigen::Matrix<double, 6, 1>>;
    using RefJacobian = tesseract_nb::InPlace<Eigen::Matrix<double, 6, Eigen::Dynamic>>;  // column-major
    using RefPoint = Eigen::Ref<const Eigen::Vector3d>;
    nb::exception<ToleranceSizeMismatchError>(m, "ToleranceSizeMismatchError", PyExc_ValueError)
        .attr("__doc__") = "lower_tolerance / upper_tolerance are not both empty or both the expected size.";

    m.def("twistChangeRefPoint",
        [](RefTwist twist, const RefPoint& ref_point) { tesseract::common::twistChangeRefPoint(twist.ref, ref_point); },
        "twist"_a, "ref_point"_a,
        "Move the twist's reference point by `ref_point` (v += ω × ref_point), in place.");
    m.def("twistChangeBase",
        [](RefTwist twist, const Eigen::Isometry3d& change_base) {
            tesseract::common::twistChangeBase(twist.ref, change_base);
        },
        "twist"_a, "change_base"_a, "Rotate the twist into the frame `change_base`, in place.");
    m.def("jacobianChangeBase",
        [](RefJacobian jacobian, const Eigen::Isometry3d& change_base) {
            tesseract::common::jacobianChangeBase(jacobian.ref, change_base);
        },
        "jacobian"_a, "change_base"_a,
        "Rotate every column of a (6, n) Fortran-order jacobian into `change_base`, in place.");
    m.def("jacobianChangeRefPoint",
        [](RefJacobian jacobian, const RefPoint& ref_point) {
            tesseract::common::jacobianChangeRefPoint(jacobian.ref, ref_point);
        },
        "jacobian"_a, "ref_point"_a,
        "Move the reference point of a (6, n) Fortran-order jacobian by `ref_point`, in place.");
    m.def("calcRotationalError", &tesseract::common::calcRotationalError, "R"_a,
          "Angle-axis vector θ·a of the rotation `R`, with θ in [-π, π].");
    m.def("calcTransformError", &tesseract::common::calcTransformError, "t1"_a, "t2"_a,
          "Error of `t1.inverse() * t2` as [translation, angle-axis rotation].");

    // calcJacobianTransformErrorDiff: four overloads of arities 3/4/5/6, so the order does not
    // matter. All handle the angle-axis ±π discontinuity that subtracting two
    // calcTransformError results does not.
    using tesseract::common::calcJacobianTransformErrorDiff;
    using RefTolerance = Eigen::Ref<const Eigen::VectorXd>;
    m.def("calcJacobianTransformErrorDiff",
          nb::overload_cast<const Eigen::Isometry3d&, const Eigen::Isometry3d&, const Eigen::Isometry3d&>(
              &calcJacobianTransformErrorDiff),
          "target"_a, "source"_a, "source_perturbed"_a);
    m.def("calcJacobianTransformErrorDiff",
          nb::overload_cast<const Eigen::Isometry3d&, const Eigen::Isometry3d&, const Eigen::Isometry3d&,
                            const Eigen::Isometry3d&>(&calcJacobianTransformErrorDiff),
          "target"_a, "target_perturbed"_a, "source"_a, "source_perturbed"_a);
    m.def("calcJacobianTransformErrorDiff",
        [](const Eigen::Isometry3d& target, const Eigen::Isometry3d& source, const Eigen::Isometry3d& source_perturbed,
           const RefTolerance& lower_tolerance, const RefTolerance& upper_tolerance) {
            check_tolerance_size(TWIST_SIZE, lower_tolerance, upper_tolerance);
            return calcJacobianTransformErrorDiff(target, source, source_perturbed, lower_tolerance, upper_tolerance);
        },
        "target"_a, "source"_a, "source_perturbed"_a, "lower_tolerance"_a, "upper_tolerance"_a);
    m.def("calcJacobianTransformErrorDiff",
        [](const Eigen::Isometry3d& target, const Eigen::Isometry3d& target_perturbed, const Eigen::Isometry3d& source,
           const Eigen::Isometry3d& source_perturbed, const RefTolerance& lower_tolerance,
           const RefTolerance& upper_tolerance) {
            check_tolerance_size(TWIST_SIZE, lower_tolerance, upper_tolerance);
            return calcJacobianTransformErrorDiff(target, target_perturbed, source, source_perturbed, lower_tolerance,
                                                  upper_tolerance);
        },
        "target"_a, "target_perturbed"_a, "source"_a, "source_perturbed"_a, "lower_tolerance"_a,
        "upper_tolerance"_a);

    m.def("applyTolerances",
        [](tesseract_nb::InPlace<Eigen::VectorXd> v, const RefTolerance& lower_tolerance,
           const RefTolerance& upper_tolerance) {
            check_tolerance_size(v.ref.size(), lower_tolerance, upper_tolerance);
            tesseract::common::applyTolerances(v.ref, lower_tolerance, upper_tolerance);
        },
        "v"_a, "lower_tolerance"_a, "upper_tolerance"_a,
        "In place: 0 inside [lower, upper], v - lower below it, v - upper above it; no-op when both are empty.");

    // ========== PluginInfo ==========
    // `config` is a YAML::Node in C++, which nanobind has no caster for. Expose it as a
    // Python `str`: the getter serialises via getConfigString(); the setter parses the
    // string with YAML::Load. Callers pass a YAML document string (e.g.
    // "base_link: base_link\ntip_link: tool0").
    // operator== compares class_name and the config YAML trees (compareYAML).
    auto plugin_info = nb::class_<tesseract::common::PluginInfo>(m, "PluginInfo")
        .def(nb::init<>())
        .def_rw("class_name", &tesseract::common::PluginInfo::class_name)
        .def_prop_rw(
            "config",
            [](const tesseract::common::PluginInfo& self) { return self.getConfigString(); },
            [](tesseract::common::PluginInfo& self, const std::string& value) {
                self.config = YAML::Load(value);
            },
            "Plugin config as a YAML document string (a YAML::Node in C++).")
        .def("getConfigString", &tesseract::common::PluginInfo::getConfigString);
    bind_value_equality(plugin_info);

    // ========== PluginInfoContainer ==========
    // plugins is PluginInfoMap = std::map<std::string, PluginInfo> -> dict[str, PluginInfo].
    auto plugin_info_container = nb::class_<tesseract::common::PluginInfoContainer>(m, "PluginInfoContainer")
        .def(nb::init<>())
        .def_rw("default_plugin", &tesseract::common::PluginInfoContainer::default_plugin)
        .def_rw("plugins", &tesseract::common::PluginInfoContainer::plugins)
        .def("clear", &tesseract::common::PluginInfoContainer::clear);
    bind_value_equality(plugin_info_container);

    // ========== PluginDiscoveryInfo ==========
    // The base of every *PluginInfo: where to find plugin libraries, before any plugin is loaded.
    auto plugin_discovery_info = nb::class_<tesseract::common::PluginDiscoveryInfo>(m, "PluginDiscoveryInfo",
        "Plugin library discovery: search paths and library names (without prefix or suffix).")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::PluginDiscoveryInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::PluginDiscoveryInfo::search_libraries)
        .def("insert", &tesseract::common::PluginDiscoveryInfo::insert, "other"_a)
        .def("clear", &tesseract::common::PluginDiscoveryInfo::clear)
        .def("empty", &tesseract::common::PluginDiscoveryInfo::empty);
    bind_value_equality(plugin_discovery_info);

    // ========== PropertyTree ==========
    // A config schema (property_tree.h): YAML crosses as text, as PluginInfo.config does. Read
    // access, applyConfig and validate; the validator and builder API stays C++ (a schema is
    // built by the C++ type that owns it, e.g. TaskComposerTask.schema).
    bind_property_tree(m);

    // ========== KinematicsPluginInfo ==========
    // fwd/inv_plugin_infos are std::map<std::string, PluginInfoContainer> keyed on group
    // name -> dict[str, PluginInfoContainer]. search_paths / search_libraries are
    // std::vector<std::string> -> list[str].
    auto kinematics_plugin_info = nb::class_<tesseract::common::KinematicsPluginInfo, tesseract::common::PluginDiscoveryInfo>(m, "KinematicsPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::KinematicsPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::KinematicsPluginInfo::search_libraries)
        .def_rw("fwd_plugin_infos", &tesseract::common::KinematicsPluginInfo::fwd_plugin_infos)
        .def_rw("inv_plugin_infos", &tesseract::common::KinematicsPluginInfo::inv_plugin_infos)
        .def("insert", &tesseract::common::KinematicsPluginInfo::insert, "other"_a)
        // The inherited PluginDiscoveryInfo::insert (search paths and libraries), which the
        // C++ overload hides: a KinematicsPluginInfo is a PluginDiscoveryInfo, so insert takes one.
        .def("insert", [](tesseract::common::KinematicsPluginInfo& self, const tesseract::common::PluginDiscoveryInfo& other) {
            self.tesseract::common::PluginDiscoveryInfo::insert(other);
        }, "other"_a)
        .def("clear", &tesseract::common::KinematicsPluginInfo::clear)
        .def("empty", &tesseract::common::KinematicsPluginInfo::empty);
    // A class constant, not a static property: stubgen renders its value, not a getter signature.
    kinematics_plugin_info.attr("CONFIG_KEY") = std::string(tesseract::common::KinematicsPluginInfo::CONFIG_KEY);
    bind_value_equality(kinematics_plugin_info);

    // ========== ContactManagersPluginInfo ==========
    // discrete/continuous_plugin_infos are bound PluginInfoContainers: def_rw returns a
    // reference, so in-place edits persist.
    auto contact_managers_plugin_info = nb::class_<tesseract::common::ContactManagersPluginInfo, tesseract::common::PluginDiscoveryInfo>(m, "ContactManagersPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::ContactManagersPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::ContactManagersPluginInfo::search_libraries)
        .def_rw("discrete_plugin_infos", &tesseract::common::ContactManagersPluginInfo::discrete_plugin_infos)
        .def_rw("continuous_plugin_infos", &tesseract::common::ContactManagersPluginInfo::continuous_plugin_infos)
        .def("insert", &tesseract::common::ContactManagersPluginInfo::insert, "other"_a)
        // The inherited PluginDiscoveryInfo::insert (search paths and libraries), which the
        // C++ overload hides: a ContactManagersPluginInfo is a PluginDiscoveryInfo, so insert takes one.
        .def("insert", [](tesseract::common::ContactManagersPluginInfo& self, const tesseract::common::PluginDiscoveryInfo& other) {
            self.tesseract::common::PluginDiscoveryInfo::insert(other);
        }, "other"_a)
        .def("clear", &tesseract::common::ContactManagersPluginInfo::clear)
        .def("empty", &tesseract::common::ContactManagersPluginInfo::empty);
    // A class constant, not a static property: stubgen renders its value, not a getter signature.
    contact_managers_plugin_info.attr("CONFIG_KEY") = std::string(tesseract::common::ContactManagersPluginInfo::CONFIG_KEY);
    bind_value_equality(contact_managers_plugin_info);

    // ========== CalibrationInfo ==========
    // joints is a JointIdTransformMap -> dict[JointId, Isometry3d], copied on every access
    // (rv_policy::copy: the default reference_internal would hand out ids that live in the map's
    // nodes): assign the whole field or use insert(); an in-place edit of the returned dict
    // changes a copy.
    auto calibration_info = nb::class_<tesseract::common::CalibrationInfo>(m, "CalibrationInfo")
        .def(nb::init<>())
        .def_rw("joints", &tesseract::common::CalibrationInfo::joints, nb::rv_policy::copy,
                "Joint name -> calibrated joint origin. A copy on read: assign the whole dict, or use insert.")
        .def("insert", &tesseract::common::CalibrationInfo::insert, "other"_a)
        .def("clear", &tesseract::common::CalibrationInfo::clear)
        .def("empty", &tesseract::common::CalibrationInfo::empty);
    // A class constant, not a static property: stubgen renders its value, not a getter signature.
    calibration_info.attr("CONFIG_KEY") = std::string(tesseract::common::CalibrationInfo::CONFIG_KEY);
    // operator== (calibration_info.cpp:41-50): transforms compared with isApprox(1e-5), upstream's tolerance
    bind_value_equality(calibration_info);

    // ========== TaskComposerPluginInfo ==========
    auto task_composer_plugin_info = nb::class_<tesseract::common::TaskComposerPluginInfo, tesseract::common::PluginDiscoveryInfo>(m, "TaskComposerPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::TaskComposerPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::TaskComposerPluginInfo::search_libraries)
        .def_rw("executor_plugin_infos", &tesseract::common::TaskComposerPluginInfo::executor_plugin_infos)
        .def_rw("task_plugin_infos", &tesseract::common::TaskComposerPluginInfo::task_plugin_infos)
        .def("insert", &tesseract::common::TaskComposerPluginInfo::insert, "other"_a)
        // The inherited PluginDiscoveryInfo::insert (search paths and libraries), which the
        // C++ overload hides: a TaskComposerPluginInfo is a PluginDiscoveryInfo, so insert takes one.
        .def("insert", [](tesseract::common::TaskComposerPluginInfo& self, const tesseract::common::PluginDiscoveryInfo& other) {
            self.tesseract::common::PluginDiscoveryInfo::insert(other);
        }, "other"_a)
        .def("clear", &tesseract::common::TaskComposerPluginInfo::clear)
        .def("empty", &tesseract::common::TaskComposerPluginInfo::empty);
    // A class constant, not a static property: stubgen renders its value, not a getter signature.
    task_composer_plugin_info.attr("CONFIG_KEY") = std::string(tesseract::common::TaskComposerPluginInfo::CONFIG_KEY);
    bind_value_equality(task_composer_plugin_info);

    // ========== ProfilesPluginInfo ==========
    // plugin_infos is std::map<std::string, PluginInfoMap> -> dict[str, dict[str, PluginInfo]],
    // converted on every access (like KinematicsPluginInfo.fwd_plugin_infos): assign the
    // whole field; an in-place edit of the returned dict changes a copy.
    auto profiles_plugin_info = nb::class_<tesseract::common::ProfilesPluginInfo, tesseract::common::PluginDiscoveryInfo>(m, "ProfilesPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::ProfilesPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::ProfilesPluginInfo::search_libraries)
        .def_rw("plugin_infos", &tesseract::common::ProfilesPluginInfo::plugin_infos)
        .def("insert", &tesseract::common::ProfilesPluginInfo::insert, "other"_a)
        // The inherited PluginDiscoveryInfo::insert (search paths and libraries), which the
        // C++ overload hides: a ProfilesPluginInfo is a PluginDiscoveryInfo, so insert takes one.
        .def("insert", [](tesseract::common::ProfilesPluginInfo& self, const tesseract::common::PluginDiscoveryInfo& other) {
            self.tesseract::common::PluginDiscoveryInfo::insert(other);
        }, "other"_a)
        .def("clear", &tesseract::common::ProfilesPluginInfo::clear)
        .def("empty", &tesseract::common::ProfilesPluginInfo::empty);
    // A class constant, not a static property: stubgen renders its value, not a getter signature.
    profiles_plugin_info.attr("CONFIG_KEY") = std::string(tesseract::common::ProfilesPluginInfo::CONFIG_KEY);
    bind_value_equality(profiles_plugin_info);

    bind_logging(m);  // common_logging.cpp

    // ========== STL containers ==========
    // VectorVector3d and VectorIsometry3d (aligned std::vectors) convert to and from Python lists
    // of arrays / Isometry3d through nanobind's stl/vector caster, in every module alike.
    // Note: VectorLong and Eigen::VectorXi use numpy arrays (automatic conversion)
}
