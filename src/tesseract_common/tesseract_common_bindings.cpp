#include "tesseract_nb.h"
#include <iterator>

// tesseract_common headers (need eigen_types.h before opaque declarations)
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>

// Opaque declarations for vector types we want to bind as classes
using VectorVector3d = tesseract::common::VectorVector3d;  // std::vector<Eigen::Vector3d>
using VectorIsometry3d = tesseract::common::VectorIsometry3d;  // std::vector<Eigen::Isometry3d>

// [joint units: rad or m] satisfiesLimits' scalar max_diff default, copied from kinematic_limits.h
constexpr double SATISFIES_LIMITS_DEFAULT_MAX_DIFF = 1e-6;
NB_MAKE_OPAQUE(VectorVector3d)
NB_MAKE_OPAQUE(VectorIsometry3d)
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/joint_state.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/contact_allowed_validator.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/common/utils.h>
#include <yaml-cpp/yaml.h>  // YAML::Load / YAML::Node for PluginInfo.config <-> str
#include <cmath>
#include <filesystem>
#include <sstream>

// console_bridge
#include <console_bridge/console.h>

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

// Trampoline class for OutputHandler
class PyOutputHandler : public console_bridge::OutputHandler {
public:
    NB_TRAMPOLINE(console_bridge::OutputHandler, 1);

    void log(const std::string& text, console_bridge::LogLevel level, const char* filename, int line) override {
        NB_OVERRIDE_PURE(log, text, level, filename, line);
    }
};

NB_MODULE(_tesseract_common, m) {
    m.doc() = "tesseract_common Python bindings (nanobind)";

    // Single source of truth for the float64 default precision used across
    // the project. Eigen exposes this as `NumTraits<double>::dummy_precision()`
    // (constexpr, returns 1e-12); we re-export so Python tests + helpers
    // can reference one value instead of duplicating `1e-12` literals.
    // Used by: planning/transforms.py (_NORMALISE_DENOM_FLOOR),
    // tests/tesseract/common/test_eigen_geometry.py (DEFAULT_PREC),
    // tests/tesseract_planning/test_planning_api.py (EIGEN_DEFAULT_PREC),
    // and the C++ `kDegenerateGeometryEps` constexpr further down.
    m.attr("EIGEN_DEFAULT_PREC") = Eigen::NumTraits<double>::dummy_precision();

    // ========== Eigen Type Aliases ==========
    // Note: Vector3d, VectorXd, MatrixXd are handled automatically by nanobind/eigen/dense.h
    // Isometry3d needs explicit binding for SWIG compatibility (tests expect .matrix() method)

    // Isometry3d class binding for SWIG API compatibility
    nb::class_<Eigen::Isometry3d>(m, "Isometry3d")
        // Default ctor — explicit Identity. Eigen's `Transform()` initialises
        // only the bottom row of the augmented matrix; the linear and
        // translation blocks are uninitialised. Without this override the
        // bare `Isometry3d()` from Python would expose uninit memory.
        .def("__init__", [](Eigen::Isometry3d* self) {
            new (self) Eigen::Isometry3d(Eigen::Isometry3d::Identity());
        })
        // Defensive copy ctor — lets value-type wrappers (e.g. `Pose`) take
        // an `Isometry3d` argument without aliasing the caller's instance.
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Isometry3d& other) {
            new (self) Eigen::Isometry3d(other);
        }, "other"_a)
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Matrix4d& mat) {
            new (self) Eigen::Isometry3d();
            self->matrix() = mat;
        }, "matrix"_a)
        // Construct directly from rotation primitives so callers don't have
        // to assemble a 4x4 matrix by hand.
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Quaterniond& q) {
            new (self) Eigen::Isometry3d(q);
        }, "rotation_quaternion"_a)
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::AngleAxisd& aa) {
            new (self) Eigen::Isometry3d(aa);
        }, "rotation_angle_axis"_a)
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Translation3d& t) {
            new (self) Eigen::Isometry3d(t);
        }, "translation"_a)
        // Canonical robotics ctor: position + orientation in one call instead
        // of `Identity() * Translation3d(...) * Quaterniond(...)` chain.
        .def("__init__", [](Eigen::Isometry3d* self,
                            const Eigen::Vector3d& t,
                            const Eigen::Quaterniond& q) {
            new (self) Eigen::Isometry3d();
            self->setIdentity();
            self->linear() = q.toRotationMatrix();
            self->translation() = t;
        }, "translation"_a, "rotation"_a)
        .def("__init__", [](Eigen::Isometry3d* self,
                            const Eigen::Translation3d& t,
                            const Eigen::Quaterniond& q) {
            new (self) Eigen::Isometry3d();
            self->setIdentity();
            self->linear() = q.toRotationMatrix();
            self->translation() = t.vector();
        }, "translation"_a, "rotation"_a)
        .def_static("Identity", []() { return Eigen::Isometry3d::Identity(); })
        .def("setIdentity", [](Eigen::Isometry3d& self) { self.setIdentity(); })
        // Stored-data accessors as properties (matches the project
        // convention: scalar component accessors on Quaterniond /
        // Translation3d / AngleAxisd are properties; this completes the
        // pattern for the rigid-transform getters). `matrix` returns the
        // full 4x4 homogeneous matrix; `translation` the 3-vector;
        // `linear` and `rotation` the upper-left 3x3 block (identical for
        // Isometry3d since it guarantees no scale/shear, but both names
        // are kept for API familiarity).
        .def_prop_ro("matrix", [](const Eigen::Isometry3d& self) -> Eigen::Matrix4d {
            return self.matrix();
        })
        .def_prop_ro("translation", [](const Eigen::Isometry3d& self) -> Eigen::Vector3d {
            return self.translation();
        })
        .def_prop_ro("rotation", [](const Eigen::Isometry3d& self) -> Eigen::Matrix3d {
            return self.rotation();
        })
        .def_prop_ro("linear", [](const Eigen::Isometry3d& self) -> Eigen::Matrix3d {
            return self.linear();
        })
        .def("inverse", [](const Eigen::Isometry3d& self) {
            return self.inverse();
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Isometry3d& other) {
            return self * other;
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Translation3d& t) {
            return Eigen::Isometry3d(self * t);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Quaterniond& q) {
            return Eigen::Isometry3d(self * q);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) {
            return Eigen::Isometry3d(self * aa);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Vector3d& v) {
            return self * v;
        })
        // In-place composition mutators. Return self so callers can chain
        // (`iso.translate(v).rotate(q)`), matching Eigen's fluent C++ API.
        //
        // IMPORTANT — Python aliasing: chaining returns the SAME object,
        // so `iso2 = iso.translate(v)` gives `iso2 is iso == True` and any
        // later mutation on `iso2` mutates `iso`. Use `Isometry3d(iso)` to
        // defensively copy first if independence is required.
        .def("translate", [](Eigen::Isometry3d& self, const Eigen::Vector3d& v) -> Eigen::Isometry3d& {
            self.translate(v);
            return self;
        }, "vec"_a, nb::rv_policy::reference_internal)
        .def("pretranslate", [](Eigen::Isometry3d& self, const Eigen::Vector3d& v) -> Eigen::Isometry3d& {
            self.pretranslate(v);
            return self;
        }, "vec"_a, nb::rv_policy::reference_internal)
        .def("rotate", [](Eigen::Isometry3d& self, const Eigen::Quaterniond& q) -> Eigen::Isometry3d& {
            self.rotate(q);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("rotate", [](Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) -> Eigen::Isometry3d& {
            self.rotate(aa);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("rotate", [](Eigen::Isometry3d& self, const Eigen::Matrix3d& R) -> Eigen::Isometry3d& {
            self.rotate(R);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::Quaterniond& q) -> Eigen::Isometry3d& {
            self.prerotate(q);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) -> Eigen::Isometry3d& {
            self.prerotate(aa);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::Matrix3d& R) -> Eigen::Isometry3d& {
            self.prerotate(R);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        // Float-safe equality. Eigen default precision for double is ~1e-12.
        .def("isApprox", [](const Eigen::Isometry3d& self,
                            const Eigen::Isometry3d& other,
                            double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        .def("__repr__", [](const Eigen::Isometry3d& self) {
            Eigen::Quaterniond q(self.linear());
            const auto& t = self.translation();
            std::ostringstream ss;
            // Project canonical: scalar-last [qx, qy, qz, qw].
            ss << "Isometry3d(translation=[" << t.x() << ", " << t.y() << ", " << t.z()
               << "], quaternion=[x=" << q.x() << ", y=" << q.y()
               << ", z=" << q.z() << ", w=" << q.w() << "])";
            return ss.str();
        });

    nb::class_<Eigen::Translation3d>(m, "Translation3d")
        .def(nb::init<double, double, double>())
        // Construct from a numpy Vector3d.
        .def("__init__", [](Eigen::Translation3d* self, const Eigen::Vector3d& v) {
            new (self) Eigen::Translation3d(v);
        }, "vector"_a)
        // Component accessors — without these the class is effectively
        // write-only (you can construct one but not read components back).
        // Properties (def_prop_ro) match the Quaterniond.x/y/z/w convention:
        // stored-data scalar reads have no trailing parens in Python.
        .def_prop_ro("x", [](const Eigen::Translation3d& self) { return self.x(); })
        .def_prop_ro("y", [](const Eigen::Translation3d& self) { return self.y(); })
        .def_prop_ro("z", [](const Eigen::Translation3d& self) { return self.z(); })
        .def("translation", [](const Eigen::Translation3d& self) -> Eigen::Vector3d {
            return self.translation();
        })
        .def("inverse", [](const Eigen::Translation3d& self) {
            return Eigen::Translation3d(self.inverse());
        })
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Isometry3d& other) {
            return Eigen::Isometry3d(self * other);
        })
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Translation3d& other) {
            return Eigen::Translation3d(self.vector() + other.vector());
        })
        // Translate a point: T * v = v + translation.
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Vector3d& v) -> Eigen::Vector3d {
            return self * v;
        })
        .def("isApprox", [](const Eigen::Translation3d& self,
                            const Eigen::Translation3d& other,
                            double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        .def("__repr__", [](const Eigen::Translation3d& self) {
            std::ostringstream ss;
            ss << "Translation3d(" << self.x() << ", " << self.y() << ", " << self.z() << ")";
            return ss.str();
        });

    nb::class_<Eigen::Quaterniond>(m, "Quaterniond")
        .def(nb::init<double, double, double, double>())  // w, x, y, z
        // Constructor from rotation matrix — validates orthonormality so
        // callers cannot smuggle scaling or shear into a "rotation."
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::Matrix3d& rot) {
            if (!rot.isUnitary()) {
                const double dev = (rot.transpose() * rot
                                    - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
                std::ostringstream ss;
                ss << "Quaterniond(rotation_matrix): input is not orthonormal "
                   << "(max |Rᵀ·R − I| = " << dev << ")";
                throw std::invalid_argument(ss.str());
            }
            new (self) Eigen::Quaterniond(rot);
        }, "rotation_matrix"_a)
        // Construct from AngleAxisd
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::AngleAxisd& aa) {
            new (self) Eigen::Quaterniond(aa);
        }, "angle_axis"_a)
        // Construct from 4-vector in Eigen-internal (x, y, z, w) coeff order;
        // round-trips with `coeffs()` and any external array of that layout.
        // Uses Eigen's pointer ctor so there is no uninitialised intermediate.
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::Vector4d& coeffs) {
            new (self) Eigen::Quaterniond(coeffs.data());
        }, "coeffs"_a)
        .def_static("Identity", []() { return Eigen::Quaterniond::Identity(); })
        // Minimal-rotation quaternion taking `a` to `b` (Eigen handles the
        // antipodal case and gives an orthogonal-axis 180-deg rotation).
        .def_static("FromTwoVectors", [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return Eigen::Quaterniond().setFromTwoVectors(a, b);
        }, "a"_a, "b"_a)
        // Project-canonical scalar-last factory. The 4-double ctor above is
        // Eigen's scalar-first (w, x, y, z) signature; prefer this in Python
        // code so the convention is uniform.
        .def_static("from_xyzw",
                    [](double qx, double qy, double qz, double qw) {
                        return Eigen::Quaterniond(qw, qx, qy, qz);
                    },
                    "qx"_a, "qy"_a, "qz"_a, "qw"_a)
        // Intrinsic ZYX Tait-Bryan factory — `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
        // This is the ROS / `tf2::Quaternion::setRPY` convention, matching
        // `tf.transformations.quaternion_from_euler(r, p, y, axes='sxyz')`.
        // Eigen has no single `fromRPY` — the idiom is to compose three
        // AngleAxis rotations; centralising it here avoids the duplication
        // that previously lived in `Pose.from_xyz_rpy`. Round-trips with
        // `eulerAngles("ZYX")` (which returns `(yaw, pitch, roll)`).
        //
        // Two overloads: scalar-positional for `geometry_msgs/Vector3 rpy`
        // (`from_rpy(rpy.x, rpy.y, rpy.z)`) and arraylike for the numpy /
        // tf_transformations shape (`from_rpy(np.asarray([r, p, y]))`).
        // Both shapes show up at real ROS-interop call sites; pick one and
        // half the callers grow ugly `*` / `.x .y .z` plumbing.
        .def_static("from_rpy",
                    [](double roll, double pitch, double yaw) {
                        return Eigen::Quaterniond(
                            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
                    },
                    "roll"_a, "pitch"_a, "yaw"_a)
        .def_static("from_rpy",
                    [](const Eigen::Vector3d& rpy) {
                        return Eigen::Quaterniond(
                            Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
                    },
                    "rpy"_a)
        // Scalar component accessors — properties, not methods. Matches the
        // Pose scalar accessors (x/y/z/qx/qy/qz/qw) and keeps the read-side
        // free of trailing-parens noise. Eigen's C++ API exposes these as
        // methods; we deliberately don't mirror that in Python.
        .def_prop_ro("w", [](const Eigen::Quaterniond& q) { return q.w(); })
        .def_prop_ro("x", [](const Eigen::Quaterniond& q) { return q.x(); })
        .def_prop_ro("y", [](const Eigen::Quaterniond& q) { return q.y(); })
        .def_prop_ro("z", [](const Eigen::Quaterniond& q) { return q.z(); })
        // Eigen stores coeffs internally as (x, y, z, w); expose for direct
        // numpy interop without per-component getter calls.
        .def("coeffs", [](const Eigen::Quaterniond& q) -> Eigen::Vector4d {
            return q.coeffs();
        })
        // The vector (imaginary / xyz) part of the quaternion.
        .def("vec", [](const Eigen::Quaterniond& q) -> Eigen::Vector3d {
            return q.vec();
        })
        .def("toRotationMatrix", [](const Eigen::Quaterniond& q) -> Eigen::Matrix3d {
            return q.toRotationMatrix();
        })
        // Spherical linear interpolation: q1.slerp(t, q2) -> Quaterniond.
        // Eigen handles the short-arc sign flip and the near-parallel
        // numerically-stable lerp fallback internally.
        .def("slerp", [](const Eigen::Quaterniond& self, double t, const Eigen::Quaterniond& other) {
            return Eigen::Quaterniond(self.slerp(t, other));
        }, "t"_a, "other"_a)
        // Hamilton product as Python __mul__ so q1 * q2 composes rotations.
        .def("__mul__", [](const Eigen::Quaterniond& self, const Eigen::Quaterniond& other) {
            return Eigen::Quaterniond(self * other);
        })
        // q * v rotates a 3-vector by the quaternion (== R(q) @ v).
        .def("__mul__", [](const Eigen::Quaterniond& self, const Eigen::Vector3d& v) -> Eigen::Vector3d {
            return self * v;
        })
        // Conjugate / inverse for unit quaternions.
        .def("conjugate", [](const Eigen::Quaterniond& self) {
            return Eigen::Quaterniond(self.conjugate());
        })
        .def("inverse", [](const Eigen::Quaterniond& self) {
            return Eigen::Quaterniond(self.inverse());
        })
        // Dot product on the 4-vector representation.
        .def("dot", [](const Eigen::Quaterniond& self, const Eigen::Quaterniond& other) {
            return self.dot(other);
        })
        // Geodesic angle between two unit quaternions (== acos(2 dot^2 - 1)).
        .def("angularDistance", [](const Eigen::Quaterniond& self, const Eigen::Quaterniond& other) {
            return self.angularDistance(other);
        })
        .def("norm", [](const Eigen::Quaterniond& self) { return self.norm(); })
        .def("squaredNorm", [](const Eigen::Quaterniond& self) { return self.squaredNorm(); })
        .def("normalize", [](Eigen::Quaterniond& self) { self.normalize(); })
        .def("normalized", [](const Eigen::Quaterniond& self) {
            return Eigen::Quaterniond(self.normalized());
        })
        .def("setIdentity", [](Eigen::Quaterniond& self) { self.setIdentity(); })
        // Component-wise approx-equality on (w, x, y, z). NOTE: q and -q
        // represent the same rotation but isApprox returns false for them;
        // use angularDistance() to test rotational equality instead.
        .def("isApprox", [](const Eigen::Quaterniond& self,
                            const Eigen::Quaterniond& other,
                            double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        // Decompose into (roll, pitch, yaw) — the inverse of `from_rpy`.
        // Same intrinsic ZYX Tait-Bryan convention as ROS / `tf2`.
        //
        // Canonical ranges (matching `tf2::Matrix3x3::getRPY`):
        //   roll  ∈ [-π, π]
        //   pitch ∈ [-π/2, π/2]
        //   yaw   ∈ [-π, π]
        //
        // Implementation note: deliberately NOT `eulerAngles("ZYX")[::-1]`.
        // Eigen's `eulerAngles` is internally consistent for *rotation*
        // preservation but its wrap conditions can return a (roll, pitch,
        // yaw) representation outside the canonical ranges above — fine
        // for math, wrong for ROS interop where users expect tf2-style
        // values. The textbook extraction below picks the *canonical*
        // branch every time, so `from_rpy(*q.to_rpy())` returns exactly
        // the (r, p, y) values the user would expect.
        //
        // Gimbal lock at cos(pitch) ≈ 0: yaw becomes free and the (roll,
        // yaw) split is arbitrary. Convention here matches tf2: yaw = 0,
        // roll absorbs the residual. The rotation roundtrips; the
        // individual values stop being meaningful — for near-vertical
        // tool axes use the quaternion / rotation-matrix surface.
        .def("to_rpy", [](const Eigen::Quaterniond& self) -> Eigen::Vector3d {
            // Threshold on cos(pitch) below which the (roll, yaw) split is
            // taken as gimbal-locked and yaw is pinned to the tf2 convention.
            // Bounded BELOW by the FP residual of "exact" gimbal lock: a unit
            // quaternion at pitch = ±π/2 reconstructs a rotation matrix whose
            // -R(2,0) lands within ~1 ulp of ±1, so the rotation's true
            // cos(pitch) bottoms out around √(2·eps) ≈ 2.1e-8, not 0. The
            // threshold MUST sit above that floor or the convention never
            // triggers at the singularity — the bug that surfaced on aarch64,
            // whose rounding differs from x86. 1e-6 clears it with ~50x margin
            // while staying within ~1e-6 rad of ±π/2, far tighter than any real
            // tool-orientation tolerance. (NB: this governs only the gimbal
            // *branch* decision — pitch itself is resolved exactly below.)
            constexpr double kGimbalLockCosPitchThreshold = 1e-6;

            const Eigen::Matrix3d R = self.toRotationMatrix();
            // Resolve pitch and cos(pitch) from the matrix entries directly, NOT
            // via `cos(asin(-R(2,0)))`. That composition is `√(1 - sin²)`
            // evaluated the cancellation-prone way: near ±π/2, sin(pitch) rounds
            // to within 1 ulp of 1, flooring cos(pitch) at √(2·eps) ≈ 2.1e-8 and
            // discarding the low bits of pitch. The off-axis entries carry
            // cos(pitch) without cancellation — R(0,0) = cos(pitch)·cos(yaw),
            // R(1,0) = cos(pitch)·sin(yaw), so hypot(R00, R10) = |cos(pitch)| —
            // and atan2 needs no |sin| ≤ 1 clamp (it never overflows). The
            // result lands in the canonical pitch ∈ [-π/2, π/2] by construction
            // (hypot ≥ 0).
            const double cos_pitch = std::hypot(R(0, 0), R(1, 0));
            const double pitch = std::atan2(-R(2, 0), cos_pitch);

            double roll, yaw;
            if (cos_pitch > kGimbalLockCosPitchThreshold) {
                roll = std::atan2(R(2, 1), R(2, 2));
                yaw  = std::atan2(R(1, 0), R(0, 0));
            } else {
                // Gimbal lock: pitch ≈ ±π/2, the (roll, yaw) DOF pair is
                // degenerate. tf2 convention pins yaw = 0; roll absorbs the sum.
                roll = std::atan2(-R(1, 2), R(1, 1));
                yaw  = 0.0;
            }
            return Eigen::Vector3d(roll, pitch, yaw);
        })
        // Decompose into intrinsic Euler angles given a 3-character axis
        // order (case-insensitive), e.g. "ZYX" — first axis rotated about
        // is Z, then Y, then X. The returned triple matches that order:
        // `q.eulerAngles("ZYX")` gives `(yaw, pitch, roll)` for
        // `R = Rz(yaw) · Ry(pitch) · Rx(roll)`. For the RPY-natural
        // (roll, pitch, yaw) order, use `to_rpy()` instead.
        //
        // Tait-Bryan orders (3 distinct axes): ranges ([-π,π], [-π/2,π/2], [-π,π]).
        // Proper Euler (repeating first axis): ranges ([0,π], [-π,π], [-π,π]).
        .def("eulerAngles", [](const Eigen::Quaterniond& self,
                               const std::string& order) -> Eigen::Vector3d {
            if (order.size() != 3) {
                throw std::invalid_argument(
                    "eulerAngles order must be 3 characters from {X, Y, Z}, e.g. 'ZYX'; got \""
                    + order + "\"");
            }
            auto axis_index = [&order](char c) -> int {
                switch (c) {
                    case 'X': case 'x': return 0;
                    case 'Y': case 'y': return 1;
                    case 'Z': case 'z': return 2;
                    default:
                        throw std::invalid_argument(
                            "eulerAngles order: invalid axis '" + std::string(1, c)
                            + "' in \"" + order + "\"; expected X, Y, or Z");
                }
            };
            const int a0 = axis_index(order[0]);
            const int a1 = axis_index(order[1]);
            const int a2 = axis_index(order[2]);
            // Adjacent axes must differ: Tait-Bryan (i,j,k all distinct) and
            // proper Euler (i==k, i!=j) are the only well-defined orderings.
            if (a0 == a1 || a1 == a2) {
                throw std::invalid_argument(
                    "eulerAngles order requires adjacent axes to differ; got \""
                    + order + "\"");
            }
            return self.toRotationMatrix().eulerAngles(a0, a1, a2);
        }, "order"_a)
        // Project canonical: scalar-last [qx, qy, qz, qw] in repr.
        .def("__repr__", [](const Eigen::Quaterniond& self) {
            std::ostringstream ss;
            ss << "Quaterniond(x=" << self.x() << ", y=" << self.y()
               << ", z=" << self.z() << ", w=" << self.w() << ")";
            return ss.str();
        });

    nb::class_<Eigen::AngleAxisd>(m, "AngleAxisd")
        .def(nb::init<double, const Eigen::Vector3d&>())
        // Construct from a Quaterniond.
        .def("__init__", [](Eigen::AngleAxisd* self, const Eigen::Quaterniond& q) {
            new (self) Eigen::AngleAxisd(q);
        }, "quaternion"_a)
        // Construct from a 3x3 rotation matrix.
        .def("__init__", [](Eigen::AngleAxisd* self, const Eigen::Matrix3d& R) {
            new (self) Eigen::AngleAxisd(R);
        }, "rotation_matrix"_a)
        // Stored-data accessors as properties (matches Quaterniond / Translation3d).
        .def_prop_ro("angle", [](const Eigen::AngleAxisd& self) { return self.angle(); })
        .def_prop_ro("axis", [](const Eigen::AngleAxisd& self) -> Eigen::Vector3d { return self.axis(); })
        .def("inverse", [](const Eigen::AngleAxisd& self) {
            return Eigen::AngleAxisd(self.inverse());
        })
        .def("toRotationMatrix", [](const Eigen::AngleAxisd& self) -> Eigen::Matrix3d {
            return self.toRotationMatrix();
        })
        .def("isApprox", [](const Eigen::AngleAxisd& self,
                            const Eigen::AngleAxisd& other,
                            double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        .def("__repr__", [](const Eigen::AngleAxisd& self) {
            const auto& ax = self.axis();
            std::ostringstream ss;
            ss << "AngleAxisd(angle=" << self.angle()
               << ", axis=[" << ax.x() << ", " << ax.y() << ", " << ax.z() << "])";
            return ss.str();
        });

    // ========== Hyperplane3d ==========
    // Plane in 3D as {x : normal · x + offset = 0}. Bound before
    // ParametrizedLine3d because the latter's intersection methods take a
    // Hyperplane3d argument; nanobind needs the type registered first.
    //
    // IMPORTANT: `signedDistance`, `absDistance`, and `projection` compute
    // `normal · p + offset` directly — they return Euclidean distance ONLY
    // when the normal is unit. Call `.normalize()` first (it rescales BOTH
    // normal and offset, preserving the plane geometry) if you constructed
    // with an un-normalised normal.
    using Hyperplane3d = Eigen::Hyperplane<double, 3>;
    // Shared threshold for rejecting degenerate geometric inputs. Sourced
    // from Eigen's float64 default precision so the entire stack (the
    // Python `_NORMALISE_DENOM_FLOOR`, Python test `DEFAULT_PREC`, the
    // re-exported `EIGEN_DEFAULT_PREC` attribute above, and this constant)
    // all share one anchor — tightening the float64 precision regime
    // changes only Eigen's value, never our duplicates. Below this
    // magnitude the implied normal / direction is indistinguishable from
    // FP noise. `static` so the lambdas below can capture it implicitly
    // under MSVC (clang/gcc tolerate constexpr capture without it; MSVC
    // does not).
    static constexpr double kDegenerateGeometryEps =
        Eigen::NumTraits<double>::dummy_precision();
    nb::class_<Hyperplane3d>(m, "Hyperplane3d")
        // Normal + signed offset: plane is {x : normal · x + offset = 0}.
        .def("__init__", [](Hyperplane3d* self,
                            const Eigen::Vector3d& normal,
                            double offset) {
            const double n_norm = normal.norm();
            if (n_norm < kDegenerateGeometryEps) {
                std::ostringstream ss;
                ss << "Hyperplane3d(normal, offset): normal is zero-magnitude (|normal|="
                   << n_norm << ")";
                throw std::invalid_argument(ss.str());
            }
            new (self) Hyperplane3d(normal, offset);
        }, "normal"_a, "offset"_a)
        // Normal + point-on-plane: computes offset = -normal · point.
        .def("__init__", [](Hyperplane3d* self,
                            const Eigen::Vector3d& normal,
                            const Eigen::Vector3d& point) {
            const double n_norm = normal.norm();
            if (n_norm < kDegenerateGeometryEps) {
                std::ostringstream ss;
                ss << "Hyperplane3d(normal, point): normal is zero-magnitude (|normal|="
                   << n_norm << ")";
                throw std::invalid_argument(ss.str());
            }
            new (self) Hyperplane3d(normal, point);
        }, "normal"_a, "point"_a)
        // Plane through three non-collinear points, normal direction by the
        // right-hand rule: `normal = (p1 − p0) × (p2 − p0)`, normalised.
        // (Upstream Eigen's `Through` computes the cross with the operands
        //  swapped, giving the wrong sign for the natural reading. We pass
        //  `(p0, p2, p1)` to undo the swap so the Python API stays intuitive.)
        // Collinear inputs are rejected — Eigen silently falls back to an
        // SVD-derived perpendicular, but that plane is mathematically arbitrary
        // and almost never what the caller meant.
        .def_static("Through", [](const Eigen::Vector3d& p0,
                                  const Eigen::Vector3d& p1,
                                  const Eigen::Vector3d& p2) {
            const Eigen::Vector3d v1 = p1 - p0;
            const Eigen::Vector3d v2 = p2 - p0;
            const double cross_norm = v1.cross(v2).norm();
            // Relative criterion: matches Eigen's own SVD-fallback trigger
            // (norm <= ||v1|| · ||v2|| · eps), tightened to our shared epsilon.
            const double scale = v1.norm() * v2.norm();
            if (scale < kDegenerateGeometryEps
                || cross_norm < scale * kDegenerateGeometryEps) {
                std::ostringstream ss;
                ss << "Hyperplane3d.Through: points are collinear "
                   << "(|cross| = " << cross_norm
                   << ", ||p1-p0|| · ||p2-p0|| = " << scale << ")";
                throw std::invalid_argument(ss.str());
            }
            return Hyperplane3d(Hyperplane3d::Through(p0, p2, p1));
        }, "p0"_a, "p1"_a, "p2"_a)
        // Stored-data accessors as properties (matches the project convention
        // for getter-style methods that return stored values).
        .def_prop_ro("normal", [](const Hyperplane3d& self) -> Eigen::Vector3d {
            return self.normal();
        })
        .def_prop_ro("offset", [](const Hyperplane3d& self) { return self.offset(); })
        // Plane coefficients (n.x, n.y, n.z, offset) — `coeffs · [x,y,z,1] = 0`.
        .def("coeffs", [](const Hyperplane3d& self) -> Eigen::Vector4d {
            return self.coeffs();
        })
        // Signed distance: positive on the half-space the normal points into.
        .def("signedDistance", [](const Hyperplane3d& self, const Eigen::Vector3d& p) {
            return self.signedDistance(p);
        }, "point"_a)
        .def("absDistance", [](const Hyperplane3d& self, const Eigen::Vector3d& p) {
            return self.absDistance(p);
        }, "point"_a)
        // Closest point on the plane (perpendicular foot).
        .def("projection", [](const Hyperplane3d& self, const Eigen::Vector3d& p) -> Eigen::Vector3d {
            return self.projection(p);
        }, "point"_a)
        // In-place normalisation of the normal vector (and corresponding
        // rescaling of the offset). Returns self for chaining.
        .def("normalize", [](Hyperplane3d& self) -> Hyperplane3d& {
            self.normalize();
            return self;
        }, nb::rv_policy::reference_internal)
        // Apply a rigid-body transform to the plane in-place. Returns self
        // for chaining. Replicates Eigen's `Hyperplane::transform(Transform&)`
        // math manually because that overload is templated on `Affine`-mode
        // Transforms and won't bind to our `Isometry3d` (`Transform<…, Isometry>`).
        .def("transform", [](Hyperplane3d& self, const Eigen::Isometry3d& tf) -> Hyperplane3d& {
            self.transform(tf.linear(), Eigen::Isometry);
            self.offset() -= self.normal().dot(tf.translation());
            return self;
        }, "transform"_a, nb::rv_policy::reference_internal)
        .def("isApprox", [](const Hyperplane3d& self, const Hyperplane3d& other, double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        .def("__repr__", [](const Hyperplane3d& self) {
            const auto& n = self.normal();
            std::ostringstream ss;
            ss << "Hyperplane3d(normal=[" << n.x() << ", " << n.y() << ", " << n.z()
               << "], offset=" << self.offset() << ")";
            return ss.str();
        });

    // ========== ParametrizedLine3d ==========
    // Line as `origin + t · direction`. The direct ctor does NOT normalise the
    // direction; `Through(p0, p1)` DOES (it stores `(p1 − p0).normalized()`).
    // Methods like `distance()`, `projection()`, and `intersectionParameter()`
    // only return Euclidean / signed-length values when the direction is unit.
    using ParametrizedLine3d = Eigen::ParametrizedLine<double, 3>;
    nb::class_<ParametrizedLine3d>(m, "ParametrizedLine3d")
        // Direct ctor — direction must be non-zero (it parameterises the line;
        // a zero direction collapses every method to NaN).
        .def("__init__", [](ParametrizedLine3d* self,
                            const Eigen::Vector3d& origin,
                            const Eigen::Vector3d& direction) {
            const double d_norm = direction.norm();
            if (d_norm < kDegenerateGeometryEps) {
                std::ostringstream ss;
                ss << "ParametrizedLine3d(origin, direction): direction is zero-magnitude "
                   << "(|direction| = " << d_norm << ")";
                throw std::invalid_argument(ss.str());
            }
            new (self) ParametrizedLine3d(origin, direction);
        }, "origin"_a, "direction"_a)
        // Line through two distinct points: origin = p0, direction = (p1 − p0).normalized().
        .def_static("Through", [](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
            const double sep = (p1 - p0).norm();
            if (sep < kDegenerateGeometryEps) {
                std::ostringstream ss;
                ss << "ParametrizedLine3d.Through: p0 and p1 are coincident "
                   << "(||p1 - p0|| = " << sep << ")";
                throw std::invalid_argument(ss.str());
            }
            return ParametrizedLine3d(ParametrizedLine3d::Through(p0, p1));
        }, "p0"_a, "p1"_a)
        // Stored-data accessors as properties (project convention).
        .def_prop_ro("origin", [](const ParametrizedLine3d& self) -> Eigen::Vector3d {
            return self.origin();
        })
        .def_prop_ro("direction", [](const ParametrizedLine3d& self) -> Eigen::Vector3d {
            return self.direction();
        })
        // Euclidean distance from `point` to the line (assumes unit direction).
        .def("distance", [](const ParametrizedLine3d& self, const Eigen::Vector3d& p) {
            return self.distance(p);
        }, "point"_a)
        .def("squaredDistance", [](const ParametrizedLine3d& self, const Eigen::Vector3d& p) {
            return self.squaredDistance(p);
        }, "point"_a)
        // Closest point on the line to `point` (assumes unit direction).
        .def("projection", [](const ParametrizedLine3d& self, const Eigen::Vector3d& p) -> Eigen::Vector3d {
            return self.projection(p);
        }, "point"_a)
        // Point at parameter t: origin + t · direction.
        .def("pointAt", [](const ParametrizedLine3d& self, double t) -> Eigen::Vector3d {
            return self.pointAt(t);
        }, "t"_a)
        // Intersection with a plane. `intersectionParameter` returns the line
        // parameter t (in direction-vector units); `intersectionPoint` returns
        // the 3D point. Both return ±inf or NaN when the line is parallel to
        // the plane — Eigen does not check; the caller must.
        .def("intersectionParameter", [](const ParametrizedLine3d& self, const Hyperplane3d& plane) {
            return self.intersectionParameter(plane);
        }, "plane"_a)
        .def("intersectionPoint", [](const ParametrizedLine3d& self, const Hyperplane3d& plane) -> Eigen::Vector3d {
            return self.intersectionPoint(plane);
        }, "plane"_a)
        // Apply a rigid-body transform to the line in-place. Returns self for
        // chaining. Same `Affine`-vs-`Isometry` template-mismatch as the
        // Hyperplane3d.transform binding above; we call the Matrix overload
        // on `tf.linear()` then add `tf.translation()` to the origin.
        .def("transform", [](ParametrizedLine3d& self, const Eigen::Isometry3d& tf) -> ParametrizedLine3d& {
            self.transform(tf.linear(), Eigen::Isometry);
            self.origin() += tf.translation();
            return self;
        }, "transform"_a, nb::rv_policy::reference_internal)
        .def("isApprox", [](const ParametrizedLine3d& self, const ParametrizedLine3d& other, double prec) {
            return self.isApprox(other, prec);
        }, "other"_a, "prec"_a = Eigen::NumTraits<double>::dummy_precision())
        .def("__repr__", [](const ParametrizedLine3d& self) {
            const auto& o = self.origin();
            const auto& d = self.direction();
            std::ostringstream ss;
            ss << "ParametrizedLine3d(origin=[" << o.x() << ", " << o.y() << ", " << o.z()
               << "], direction=[" << d.x() << ", " << d.y() << ", " << d.z() << "])";
            return ss.str();
        });

    // Note: TransformMap (std::map<string, Isometry3d>) is handled automatically by nanobind's
    // stl/map type caster - Python dict with Isometry3d values will convert automatically

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
    nb::class_<tesseract::common::GeneralResourceLocator, tesseract::common::ResourceLocator>(m, "GeneralResourceLocator")
        .def(nb::init<>())
        .def(nb::init<const std::vector<std::string>&>(), nb::kw_only(), "environment_variables"_a)
        .def(nb::init<const std::vector<std::filesystem::path>&, const std::vector<std::string>&>(),
             nb::kw_only(), "paths"_a, "environment_variables"_a = GENERAL_RESOURCE_LOCATOR_DEFAULT_ENV_VARS)
        .def("addPath", &tesseract::common::GeneralResourceLocator::addPath, "path"_a)
        .def("loadEnvironmentVariable", &tesseract::common::GeneralResourceLocator::loadEnvironmentVariable,
             "environment_variable"_a);

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
    nb::class_<tesseract::common::BytesResource, tesseract::common::Resource>(m, "BytesResource")
        .def(nb::init<std::string, std::vector<uint8_t>, std::shared_ptr<tesseract::common::ResourceLocator>>(),
             "url"_a, "bytes"_a, "parent"_a.none() = nb::none())
        .def("__init__", [](tesseract::common::BytesResource* self, std::string url, nb::bytes data,
                            std::shared_ptr<tesseract::common::ResourceLocator> parent) {
            std::vector<uint8_t> vec(data.size());
            std::memcpy(vec.data(), data.c_str(), data.size());
            new (self) tesseract::common::BytesResource(std::move(url), std::move(vec), std::move(parent));
        }, "url"_a, "bytes"_a, "parent"_a.none() = nb::none());

    nb::class_<tesseract::common::SimpleLocatedResource, tesseract::common::Resource>(m, "SimpleLocatedResource")
        .def(nb::init<const std::string&, const std::string&>(), "url"_a, "filename"_a)
        .def(nb::init<const std::string&, const std::string&, std::shared_ptr<tesseract::common::ResourceLocator>>(),
             "url"_a, "filename"_a, "parent"_a);

    // ========== ManipulatorInfo ==========
    // tcp_offset is std::variant<std::string, Eigen::Isometry3d>: a link name or a pose. The ctor
    // and the property share nanobind's variant caster, so any other type raises TypeError. The
    // getter returns a copy, not def_rw's reference_internal: a Python Isometry3d pointing into
    // the variant would dangle once the field is set to a str.
    using TcpOffset = std::variant<std::string, Eigen::Isometry3d>;
    nb::class_<tesseract::common::ManipulatorInfo>(m, "ManipulatorInfo")
        .def(nb::init<>())
        .def(nb::init<std::string, std::string, std::string, TcpOffset>(),
             "manipulator"_a, "working_frame"_a, "tcp_frame"_a,
             "tcp_offset"_a = Eigen::Isometry3d::Identity())
        .def_rw("manipulator", &tesseract::common::ManipulatorInfo::manipulator)
        .def_rw("manipulator_ik_solver", &tesseract::common::ManipulatorInfo::manipulator_ik_solver)
        .def_rw("working_frame", &tesseract::common::ManipulatorInfo::working_frame)
        .def_rw("tcp_frame", &tesseract::common::ManipulatorInfo::tcp_frame)
        .def_prop_rw("tcp_offset",
            [](const tesseract::common::ManipulatorInfo& self) -> TcpOffset { return self.tcp_offset; },
            [](tesseract::common::ManipulatorInfo& self, TcpOffset value) { self.tcp_offset = std::move(value); },
            "TCP offset: a link name (str) or a pose (Isometry3d). Reading returns a copy.")
        // Copy of self with every non-empty field of the override; an overriding tcp_frame
        // brings its tcp_offset along (manipulator_info.cpp).
        .def("getCombined", &tesseract::common::ManipulatorInfo::getCombined, "manip_info_override"_a)
        // True unless manipulator, working_frame and tcp_frame are all set.
        .def("empty", &tesseract::common::ManipulatorInfo::empty)
        .def("__repr__", [](const tesseract::common::ManipulatorInfo& self) {
            return "<ManipulatorInfo manipulator='" + self.manipulator + "'>";
        });

    // ========== JointState ==========
    nb::class_<tesseract::common::JointState>(m, "JointState")
        .def(nb::init<>())
        .def(nb::init<const std::vector<std::string>&, const Eigen::VectorXd&>())
        .def_rw("joint_names", &tesseract::common::JointState::joint_names)
        .def_rw("position", &tesseract::common::JointState::position)
        .def_rw("velocity", &tesseract::common::JointState::velocity)
        .def_rw("acceleration", &tesseract::common::JointState::acceleration)
        .def_rw("effort", &tesseract::common::JointState::effort)
        .def_rw("time", &tesseract::common::JointState::time);

    // ========== JointTrajectory ==========
    // Element access returns a copy: a reference into `states` would dangle once push_back
    // reallocates. Edit an element with a write-back, `traj[i] = js`. The iterator-taking
    // vector members (insert/emplace/erase, the range ctor, reverse iterators, data, swap)
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
        });
    bind_value_equality(joint_trajectory);

    // ========== AllowedCollisionMatrix ==========
    // The entries ctor orders each key (allowed_collision_matrix.cpp), so ("b", "a") is stored as ("a", "b").
    nb::class_<tesseract::common::AllowedCollisionMatrix>(m, "AllowedCollisionMatrix")
        .def(nb::init<>())
        .def(nb::init<const tesseract::common::AllowedCollisionEntries&>(), "entries"_a)
        .def("addAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&, const std::string&>(
                 &tesseract::common::AllowedCollisionMatrix::addAllowedCollision))
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&>(
                 &tesseract::common::AllowedCollisionMatrix::removeAllowedCollision))
        // Removes every entry that involves `link_name`.
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&>(&tesseract::common::AllowedCollisionMatrix::removeAllowedCollision),
             "link_name"_a)
        .def("isCollisionAllowed", &tesseract::common::AllowedCollisionMatrix::isCollisionAllowed)
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

    // The void out-param overload (types.h:59) has the same Python signature; this covers both.
    m.def("makeOrderedLinkPair",
          nb::overload_cast<const std::string&, const std::string&>(&tesseract::common::makeOrderedLinkPair),
          "link_name1"_a, "link_name2"_a, "The pair with the lexicographically smaller link name first.");
    // Links allowed to collide with any of `link_names`, in acm_entries' (unordered) iteration order.
    m.def("getAllowedCollisions", &tesseract::common::getAllowedCollisions,
          "link_names"_a, "acm_entries"_a, "remove_duplicates"_a = true);

    // ========== ContactAllowedValidator ==========
    // Abstract base. Determines whether two links are allowed to be in collision.
    nb::class_<tesseract::common::ContactAllowedValidator>(m, "ContactAllowedValidator")
        .def("__call__", &tesseract::common::ContactAllowedValidator::operator(), "link_name1"_a, "link_name2"_a);

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
        .def(nb::init<std::vector<std::shared_ptr<const tesseract::common::ContactAllowedValidator>>,
                      tesseract::common::CombinedContactAllowedValidatorType>(),
             "validators"_a, "type"_a);

    // ========== CollisionMarginData ==========
    nb::enum_<tesseract::common::CollisionMarginPairOverrideType>(m, "CollisionMarginPairOverrideType")
        .value("NONE", tesseract::common::CollisionMarginPairOverrideType::NONE)
        .value("REPLACE", tesseract::common::CollisionMarginPairOverrideType::REPLACE)
        .value("MODIFY", tesseract::common::CollisionMarginPairOverrideType::MODIFY);

    // CollisionMarginPairData - new in 0.33
    nb::class_<tesseract::common::CollisionMarginPairData>(m, "CollisionMarginPairData")
        .def(nb::init<>())
        .def(nb::init<const tesseract::common::PairsCollisionMarginData&>(), "pair_margins"_a)
        .def("setCollisionMargin", &tesseract::common::CollisionMarginPairData::setCollisionMargin)
        .def("getCollisionMargin", &tesseract::common::CollisionMarginPairData::getCollisionMargin)
        .def("getCollisionMargins", &tesseract::common::CollisionMarginPairData::getCollisionMargins)
        .def("getMaxCollisionMargin",
             nb::overload_cast<>(&tesseract::common::CollisionMarginPairData::getMaxCollisionMargin, nb::const_),
             "Largest pair margin, or None when no pair margin is set.")
        .def("getMaxCollisionMargin",
             nb::overload_cast<const std::string&>(&tesseract::common::CollisionMarginPairData::getMaxCollisionMargin,
                                                   nb::const_),
             "obj"_a, "Largest pair margin involving `obj`, or None when no pair involves it.")
        .def("incrementMargins", &tesseract::common::CollisionMarginPairData::incrementMargins, "increment"_a)
        .def("scaleMargins", &tesseract::common::CollisionMarginPairData::scaleMargins, "scale"_a)
        .def("apply", &tesseract::common::CollisionMarginPairData::apply, "pair_margin_data"_a, "override_type"_a)
        .def("empty", &tesseract::common::CollisionMarginPairData::empty)
        .def("clear", &tesseract::common::CollisionMarginPairData::clear);

    nb::class_<tesseract::common::CollisionMarginData>(m, "CollisionMarginData")
        .def(nb::init<>())
        .def(nb::init<double>())
        .def(nb::init<double, tesseract::common::CollisionMarginPairData>(), "default_collision_margin"_a,
             "pair_collision_margins"_a)
        .def(nb::init<tesseract::common::CollisionMarginPairData>(), "pair_collision_margins"_a)
        .def("getDefaultCollisionMargin", &tesseract::common::CollisionMarginData::getDefaultCollisionMargin)
        .def("setDefaultCollisionMargin", &tesseract::common::CollisionMarginData::setDefaultCollisionMargin)
        .def("getCollisionMargin", &tesseract::common::CollisionMarginData::getCollisionMargin)
        .def("setCollisionMargin", &tesseract::common::CollisionMarginData::setCollisionMargin)
        .def("getCollisionMarginPairData", &tesseract::common::CollisionMarginData::getCollisionMarginPairData)
        .def("getMaxCollisionMargin", nb::overload_cast<>(&tesseract::common::CollisionMarginData::getMaxCollisionMargin, nb::const_))
        .def("getMaxCollisionMargin",
             nb::overload_cast<const std::string&>(&tesseract::common::CollisionMarginData::getMaxCollisionMargin,
                                                   nb::const_),
             "obj"_a, "Largest margin involving `obj`: its largest pair margin or the default, whichever is larger.")
        .def("incrementMargins", &tesseract::common::CollisionMarginData::incrementMargins, "increment"_a)
        .def("scaleMargins", &tesseract::common::CollisionMarginData::scaleMargins, "scale"_a)
        .def("apply", &tesseract::common::CollisionMarginData::apply, "pair_margin_data"_a, "override_type"_a);

    // ========== KinematicLimits ==========
    nb::class_<tesseract::common::KinematicLimits>(m, "KinematicLimits")
        .def(nb::init<>())
        .def_rw("joint_limits", &tesseract::common::KinematicLimits::joint_limits)
        .def_rw("velocity_limits", &tesseract::common::KinematicLimits::velocity_limits)
        .def_rw("acceleration_limits", &tesseract::common::KinematicLimits::acceleration_limits)
        .def_rw("jerk_limits", &tesseract::common::KinematicLimits::jerk_limits)
        // Resizes all four limit matrices to (size, 2) (Eigen resize: values unset after a size change).
        .def("resize", &tesseract::common::KinematicLimits::resize, "size"_a);

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
    using RefTwist = Eigen::Ref<Eigen::Matrix<double, 6, 1>>;
    using RefJacobian = Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>>;  // column-major
    using RefPoint = Eigen::Ref<const Eigen::Vector3d>;
    nb::exception<ToleranceSizeMismatchError>(m, "ToleranceSizeMismatchError", PyExc_ValueError)
        .attr("__doc__") = "lower_tolerance / upper_tolerance are not both empty or both the expected size.";

    m.def("twistChangeRefPoint",
        [](RefTwist twist, const RefPoint& ref_point) { tesseract::common::twistChangeRefPoint(twist, ref_point); },
        "twist"_a, "ref_point"_a,
        "Move the twist's reference point by `ref_point` (v += ω × ref_point), in place.");
    m.def("twistChangeBase",
        [](RefTwist twist, const Eigen::Isometry3d& change_base) {
            tesseract::common::twistChangeBase(twist, change_base);
        },
        "twist"_a, "change_base"_a, "Rotate the twist into the frame `change_base`, in place.");
    m.def("jacobianChangeBase",
        [](RefJacobian jacobian, const Eigen::Isometry3d& change_base) {
            tesseract::common::jacobianChangeBase(jacobian, change_base);
        },
        "jacobian"_a, "change_base"_a,
        "Rotate every column of a (6, n) Fortran-order jacobian into `change_base`, in place.");
    m.def("jacobianChangeRefPoint",
        [](RefJacobian jacobian, const RefPoint& ref_point) {
            tesseract::common::jacobianChangeRefPoint(jacobian, ref_point);
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
        [](Eigen::Ref<Eigen::VectorXd> v, const RefTolerance& lower_tolerance, const RefTolerance& upper_tolerance) {
            check_tolerance_size(v.size(), lower_tolerance, upper_tolerance);
            tesseract::common::applyTolerances(v, lower_tolerance, upper_tolerance);
        },
        "v"_a, "lower_tolerance"_a, "upper_tolerance"_a,
        "In place: 0 inside [lower, upper], v - lower below it, v - upper above it; no-op when both are empty.");

    // ========== PluginInfo ==========
    // `config` is a YAML::Node in C++, which nanobind has no caster for. Expose it as a
    // Python `str`: the getter serialises via getConfigString(); the setter parses the
    // string with YAML::Load. Callers pass a YAML document string (e.g.
    // "base_link: base_link\ntip_link: tool0").
    nb::class_<tesseract::common::PluginInfo>(m, "PluginInfo")
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

    // ========== PluginInfoContainer ==========
    // plugins is PluginInfoMap = std::map<std::string, PluginInfo> -> dict[str, PluginInfo].
    nb::class_<tesseract::common::PluginInfoContainer>(m, "PluginInfoContainer")
        .def(nb::init<>())
        .def_rw("default_plugin", &tesseract::common::PluginInfoContainer::default_plugin)
        .def_rw("plugins", &tesseract::common::PluginInfoContainer::plugins)
        .def("clear", &tesseract::common::PluginInfoContainer::clear);

    // ========== KinematicsPluginInfo ==========
    // fwd/inv_plugin_infos are std::map<std::string, PluginInfoContainer> keyed on group
    // name -> dict[str, PluginInfoContainer]. search_paths / search_libraries are
    // std::vector<std::string> -> list[str].
    nb::class_<tesseract::common::KinematicsPluginInfo>(m, "KinematicsPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::KinematicsPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::KinematicsPluginInfo::search_libraries)
        .def_rw("fwd_plugin_infos", &tesseract::common::KinematicsPluginInfo::fwd_plugin_infos)
        .def_rw("inv_plugin_infos", &tesseract::common::KinematicsPluginInfo::inv_plugin_infos)
        .def("insert", &tesseract::common::KinematicsPluginInfo::insert, "other"_a)
        .def("clear", &tesseract::common::KinematicsPluginInfo::clear)
        .def("empty", &tesseract::common::KinematicsPluginInfo::empty)
        .def_ro_static("CONFIG_KEY", &tesseract::common::KinematicsPluginInfo::CONFIG_KEY);

    // ========== ContactManagersPluginInfo ==========
    // discrete/continuous_plugin_infos are bound PluginInfoContainers: def_rw returns a
    // reference, so in-place edits persist.
    auto contact_managers_plugin_info = nb::class_<tesseract::common::ContactManagersPluginInfo>(m, "ContactManagersPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::ContactManagersPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::ContactManagersPluginInfo::search_libraries)
        .def_rw("discrete_plugin_infos", &tesseract::common::ContactManagersPluginInfo::discrete_plugin_infos)
        .def_rw("continuous_plugin_infos", &tesseract::common::ContactManagersPluginInfo::continuous_plugin_infos)
        .def("insert", &tesseract::common::ContactManagersPluginInfo::insert, "other"_a)
        .def("clear", &tesseract::common::ContactManagersPluginInfo::clear)
        .def("empty", &tesseract::common::ContactManagersPluginInfo::empty)
        .def_ro_static("CONFIG_KEY", &tesseract::common::ContactManagersPluginInfo::CONFIG_KEY);
    bind_value_equality(contact_managers_plugin_info);

    // ========== TaskComposerPluginInfo ==========
    auto task_composer_plugin_info = nb::class_<tesseract::common::TaskComposerPluginInfo>(m, "TaskComposerPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::TaskComposerPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::TaskComposerPluginInfo::search_libraries)
        .def_rw("executor_plugin_infos", &tesseract::common::TaskComposerPluginInfo::executor_plugin_infos)
        .def_rw("task_plugin_infos", &tesseract::common::TaskComposerPluginInfo::task_plugin_infos)
        .def("insert", &tesseract::common::TaskComposerPluginInfo::insert, "other"_a)
        .def("clear", &tesseract::common::TaskComposerPluginInfo::clear)
        .def("empty", &tesseract::common::TaskComposerPluginInfo::empty)
        .def_ro_static("CONFIG_KEY", &tesseract::common::TaskComposerPluginInfo::CONFIG_KEY);
    bind_value_equality(task_composer_plugin_info);

    // ========== ProfilesPluginInfo ==========
    // plugin_infos is std::map<std::string, PluginInfoMap> -> dict[str, dict[str, PluginInfo]],
    // converted on every access (like KinematicsPluginInfo.fwd_plugin_infos): assign the
    // whole field; an in-place edit of the returned dict changes a copy.
    auto profiles_plugin_info = nb::class_<tesseract::common::ProfilesPluginInfo>(m, "ProfilesPluginInfo")
        .def(nb::init<>())
        .def_rw("search_paths", &tesseract::common::ProfilesPluginInfo::search_paths)
        .def_rw("search_libraries", &tesseract::common::ProfilesPluginInfo::search_libraries)
        .def_rw("plugin_infos", &tesseract::common::ProfilesPluginInfo::plugin_infos)
        .def("insert", &tesseract::common::ProfilesPluginInfo::insert, "other"_a)
        .def("clear", &tesseract::common::ProfilesPluginInfo::clear)
        .def("empty", &tesseract::common::ProfilesPluginInfo::empty)
        .def_ro_static("CONFIG_KEY", &tesseract::common::ProfilesPluginInfo::CONFIG_KEY);
    bind_value_equality(profiles_plugin_info);

    // ========== Console Bridge ==========
    nb::enum_<console_bridge::LogLevel>(m, "LogLevel")
        .value("CONSOLE_BRIDGE_LOG_DEBUG", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
        .value("CONSOLE_BRIDGE_LOG_INFO", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        .value("CONSOLE_BRIDGE_LOG_WARN", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN)
        .value("CONSOLE_BRIDGE_LOG_ERROR", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR)
        .value("CONSOLE_BRIDGE_LOG_NONE", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    nb::class_<console_bridge::OutputHandler, PyOutputHandler>(m, "OutputHandler")
        .def(nb::init<>())
        .def("log", &console_bridge::OutputHandler::log);

    m.def("setLogLevel", &console_bridge::setLogLevel, "level"_a);
    m.def("getLogLevel", &console_bridge::getLogLevel);
    // Wrapper for console_bridge::log (variadic function)
    m.def("log", [](const std::string& filename, int line, console_bridge::LogLevel level, const std::string& msg) {
        console_bridge::log(filename.c_str(), line, level, "%s", msg.c_str());
    }, "filename"_a, "line"_a, "level"_a, "msg"_a);
    m.def("useOutputHandler", &console_bridge::useOutputHandler, "handler"_a);
    m.def("restorePreviousOutputHandler", &console_bridge::restorePreviousOutputHandler);

    // ========== STL Container Bindings ==========
    // VectorVector3d - explicit binding for aligned Eigen vectors (NB_MAKE_OPAQUE at top)
    nb::class_<VectorVector3d>(m, "VectorVector3d")
        .def(nb::init<>())
        .def("__len__", [](const VectorVector3d& self) { return self.size(); })
        .def("__getitem__", [](const VectorVector3d& self, size_t i) -> Eigen::Vector3d {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            return self[i];
        })
        .def("__setitem__", [](VectorVector3d& self, size_t i, const Eigen::Vector3d& v) {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            self[i] = v;
        })
        .def("append", [](VectorVector3d& self, const Eigen::Vector3d& v) { self.push_back(v); })
        .def("clear", [](VectorVector3d& self) { self.clear(); });

    // VectorIsometry3d - for transform arrays (NB_MAKE_OPAQUE at top)
    nb::class_<VectorIsometry3d>(m, "VectorIsometry3d")
        .def(nb::init<>())
        .def("__len__", [](const VectorIsometry3d& self) { return self.size(); })
        .def("__getitem__", [](const VectorIsometry3d& self, size_t i) {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            return self[i];
        })
        .def("append", [](VectorIsometry3d& self, const Eigen::Isometry3d& v) { self.push_back(v); })
        .def("clear", [](VectorIsometry3d& self) { self.clear(); });

    // Note: VectorLong and Eigen::VectorXi use numpy arrays (automatic conversion)
}
