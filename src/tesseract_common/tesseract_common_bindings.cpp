#include "tesseract_nb.h"

// tesseract_common headers (need eigen_types.h before opaque declarations)
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/types.h>

// Opaque declarations for vector types we want to bind as classes
using VectorVector3d = tesseract_common::VectorVector3d;  // std::vector<Eigen::Vector3d>
using VectorIsometry3d = tesseract_common::VectorIsometry3d;  // std::vector<Eigen::Isometry3d>
NB_MAKE_OPAQUE(VectorVector3d)
NB_MAKE_OPAQUE(VectorIsometry3d)
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/plugin_info.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

// console_bridge
#include <console_bridge/console.h>

// Trampoline class for ResourceLocator
class PyResourceLocator : public tesseract_common::ResourceLocator {
public:
    NB_TRAMPOLINE(tesseract_common::ResourceLocator, 1);

    std::shared_ptr<tesseract_common::Resource> locateResource(const std::string& url) const override {
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
    // tests/tesseract_common/test_eigen_geometry.py (DEFAULT_PREC),
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
            // Threshold on cos(pitch) below which (roll, yaw) cannot be
            // separated stably. 1e-9 in cos(pitch) corresponds to pitch
            // within ~4.5e-5 rad (≈ 2.5 millidegrees) of ±π/2 — well
            // below typical joint-angle precision and below Eigen's own
            // `NumTraits<double>::dummy_precision()` floor.
            constexpr double kGimbalLockCosPitchThreshold = 1e-9;

            const Eigen::Matrix3d R = self.toRotationMatrix();
            // Clamp before asin to guard against |sin(pitch)| > 1 from
            // accumulated FP error on the input quaternion.
            const double sin_pitch = std::clamp(-R(2, 0), -1.0, 1.0);
            const double pitch = std::asin(sin_pitch);
            const double cos_pitch = std::cos(pitch);

            double roll, yaw;
            if (std::abs(cos_pitch) > kGimbalLockCosPitchThreshold) {
                roll = std::atan2(R(2, 1), R(2, 2));
                yaw  = std::atan2(R(1, 0), R(0, 0));
            } else {
                // Gimbal lock: pitch ≈ ±π/2. Match tf2 convention.
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

    // ========== FilesystemPath ==========
    // Wrapper for std::filesystem::path (SWIG compatibility)
    nb::class_<std::filesystem::path>(m, "FilesystemPath")
        .def(nb::init<>())
        .def(nb::init<const std::string&>(), "path"_a)
        .def("string", [](const std::filesystem::path& p) { return p.string(); })
        .def("__str__", [](const std::filesystem::path& p) { return p.string(); })
        .def("__repr__", [](const std::filesystem::path& p) {
            return "FilesystemPath('" + p.string() + "')";
        });

    // Note: TransformMap (std::map<string, Isometry3d>) is handled automatically by nanobind's
    // stl/map type caster - Python dict with Isometry3d values will convert automatically

    // ========== Resource Types ==========
    // Note: In nanobind 2.x, shared_ptr holder is automatic - don't specify it
    nb::class_<tesseract_common::Resource>(m, "Resource")
        .def("isFile", &tesseract_common::Resource::isFile)
        .def("getUrl", &tesseract_common::Resource::getUrl)
        .def("getFilePath", &tesseract_common::Resource::getFilePath)
        .def("getResourceContents", [](tesseract_common::Resource& self) {
            std::vector<uint8_t> data = self.getResourceContents();
            return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
        })
        .def("getResourceContentStream", &tesseract_common::Resource::getResourceContentStream);

    nb::class_<tesseract_common::BytesResource, tesseract_common::Resource>(m, "BytesResource")
        .def(nb::init<const std::string&, const std::vector<uint8_t>&>())
        .def("__init__", [](tesseract_common::BytesResource* self, const std::string& url, nb::bytes data) {
            std::vector<uint8_t> vec(data.size());
            std::memcpy(vec.data(), data.c_str(), data.size());
            new (self) tesseract_common::BytesResource(url, vec);
        });

    nb::class_<tesseract_common::SimpleLocatedResource, tesseract_common::Resource>(m, "SimpleLocatedResource")
        .def(nb::init<const std::string&, const std::string&>(), "url"_a, "filename"_a)
        .def(nb::init<const std::string&, const std::string&, std::shared_ptr<tesseract_common::ResourceLocator>>(),
             "url"_a, "filename"_a, "parent"_a);

    // ========== ResourceLocator Hierarchy ==========
    nb::class_<tesseract_common::ResourceLocator, PyResourceLocator>(m, "ResourceLocator")
        .def(nb::init<>())
        .def("locateResource", &tesseract_common::ResourceLocator::locateResource);

    nb::class_<tesseract_common::GeneralResourceLocator, tesseract_common::ResourceLocator>(m, "GeneralResourceLocator")
        .def(nb::init<>());

    // ========== ManipulatorInfo ==========
    nb::class_<tesseract_common::ManipulatorInfo>(m, "ManipulatorInfo")
        .def(nb::init<>())
        .def_rw("manipulator", &tesseract_common::ManipulatorInfo::manipulator)
        .def_rw("manipulator_ik_solver", &tesseract_common::ManipulatorInfo::manipulator_ik_solver)
        .def_rw("working_frame", &tesseract_common::ManipulatorInfo::working_frame)
        .def_rw("tcp_frame", &tesseract_common::ManipulatorInfo::tcp_frame)
        .def_prop_rw("tcp_offset",
            [](const tesseract_common::ManipulatorInfo& self) -> nb::object {
                if (self.tcp_offset.index() == 0) {
                    return nb::cast(std::get<std::string>(self.tcp_offset));
                } else {
                    return nb::cast(std::get<Eigen::Isometry3d>(self.tcp_offset));
                }
            },
            [](tesseract_common::ManipulatorInfo& self, nb::object value) {
                if (nb::isinstance<nb::str>(value)) {
                    self.tcp_offset = nb::cast<std::string>(value);
                } else {
                    self.tcp_offset = nb::cast<Eigen::Isometry3d>(value);
                }
            })
        .def("__repr__", [](const tesseract_common::ManipulatorInfo& self) {
            return "<ManipulatorInfo manipulator='" + self.manipulator + "'>";
        });

    // ========== JointState ==========
    nb::class_<tesseract_common::JointState>(m, "JointState")
        .def(nb::init<>())
        .def(nb::init<const std::vector<std::string>&, const Eigen::VectorXd&>())
        .def_rw("joint_names", &tesseract_common::JointState::joint_names)
        .def_rw("position", &tesseract_common::JointState::position)
        .def_rw("velocity", &tesseract_common::JointState::velocity)
        .def_rw("acceleration", &tesseract_common::JointState::acceleration)
        .def_rw("effort", &tesseract_common::JointState::effort)
        .def_rw("time", &tesseract_common::JointState::time);

    // ========== AllowedCollisionMatrix ==========
    nb::class_<tesseract_common::AllowedCollisionMatrix>(m, "AllowedCollisionMatrix")
        .def(nb::init<>())
        .def("addAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&, const std::string&>(
                 &tesseract_common::AllowedCollisionMatrix::addAllowedCollision))
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&>(
                 &tesseract_common::AllowedCollisionMatrix::removeAllowedCollision))
        .def("isCollisionAllowed", &tesseract_common::AllowedCollisionMatrix::isCollisionAllowed)
        .def("clearAllowedCollisions", &tesseract_common::AllowedCollisionMatrix::clearAllowedCollisions)
        .def("getAllAllowedCollisions", &tesseract_common::AllowedCollisionMatrix::getAllAllowedCollisions)
        .def("insertAllowedCollisionMatrix", &tesseract_common::AllowedCollisionMatrix::insertAllowedCollisionMatrix);

    // ========== CollisionMarginData ==========
    // Note: CollisionMarginOverrideType was renamed to CollisionMarginPairOverrideType in 0.33
    // and reduced to 3 values (NONE, REPLACE, MODIFY)
    nb::enum_<tesseract_common::CollisionMarginPairOverrideType>(m, "CollisionMarginPairOverrideType")
        .value("NONE", tesseract_common::CollisionMarginPairOverrideType::NONE)
        .value("REPLACE", tesseract_common::CollisionMarginPairOverrideType::REPLACE)
        .value("MODIFY", tesseract_common::CollisionMarginPairOverrideType::MODIFY);

    // Backwards-compatible alias (old name)
    m.attr("CollisionMarginOverrideType") = m.attr("CollisionMarginPairOverrideType");

    // CollisionMarginPairData - new in 0.33
    nb::class_<tesseract_common::CollisionMarginPairData>(m, "CollisionMarginPairData")
        .def(nb::init<>())
        .def("setCollisionMargin", &tesseract_common::CollisionMarginPairData::setCollisionMargin)
        .def("getCollisionMargin", &tesseract_common::CollisionMarginPairData::getCollisionMargin)
        .def("getCollisionMargins", &tesseract_common::CollisionMarginPairData::getCollisionMargins)
        .def("empty", &tesseract_common::CollisionMarginPairData::empty)
        .def("clear", &tesseract_common::CollisionMarginPairData::clear);

    nb::class_<tesseract_common::CollisionMarginData>(m, "CollisionMarginData")
        .def(nb::init<>())
        .def(nb::init<double>())
        .def("getDefaultCollisionMargin", &tesseract_common::CollisionMarginData::getDefaultCollisionMargin)
        .def("setDefaultCollisionMargin", &tesseract_common::CollisionMarginData::setDefaultCollisionMargin)
        .def("getCollisionMargin", &tesseract_common::CollisionMarginData::getCollisionMargin)
        .def("setCollisionMargin", &tesseract_common::CollisionMarginData::setCollisionMargin)
        .def("getCollisionMarginPairData", &tesseract_common::CollisionMarginData::getCollisionMarginPairData)
        .def("getMaxCollisionMargin", nb::overload_cast<>(&tesseract_common::CollisionMarginData::getMaxCollisionMargin, nb::const_))
        // Backwards compatibility aliases
        .def("getPairCollisionMargin", &tesseract_common::CollisionMarginData::getCollisionMargin)
        .def("setPairCollisionMargin", &tesseract_common::CollisionMarginData::setCollisionMargin);

    // ========== KinematicLimits ==========
    nb::class_<tesseract_common::KinematicLimits>(m, "KinematicLimits")
        .def(nb::init<>())
        .def_rw("joint_limits", &tesseract_common::KinematicLimits::joint_limits)
        .def_rw("velocity_limits", &tesseract_common::KinematicLimits::velocity_limits)
        .def_rw("acceleration_limits", &tesseract_common::KinematicLimits::acceleration_limits);

    // ========== PluginInfo ==========
    nb::class_<tesseract_common::PluginInfo>(m, "PluginInfo")
        .def(nb::init<>())
        .def_rw("class_name", &tesseract_common::PluginInfo::class_name)
        .def_rw("config", &tesseract_common::PluginInfo::config);

    // ========== Console Bridge ==========
    nb::enum_<console_bridge::LogLevel>(m, "LogLevel")
        .value("CONSOLE_BRIDGE_LOG_DEBUG", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
        .value("CONSOLE_BRIDGE_LOG_INFO", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        .value("CONSOLE_BRIDGE_LOG_WARN", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN)
        .value("CONSOLE_BRIDGE_LOG_ERROR", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR)
        .value("CONSOLE_BRIDGE_LOG_NONE", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // Export log level constants at module level
    m.attr("CONSOLE_BRIDGE_LOG_DEBUG") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG;
    m.attr("CONSOLE_BRIDGE_LOG_INFO") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;
    m.attr("CONSOLE_BRIDGE_LOG_WARN") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN;
    m.attr("CONSOLE_BRIDGE_LOG_ERROR") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR;
    m.attr("CONSOLE_BRIDGE_LOG_NONE") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE;

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
