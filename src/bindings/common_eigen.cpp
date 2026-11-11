/**
 * @file common_eigen.cpp
 * @brief tesseract_common: the Eigen rigid-transform types and their batched readouts
 */

#include "common_bindings.h"

#include <cmath>
#include <limits>
#include <sstream>

// [dimensionless, cos(pitch)] to_rpy's gimbal-lock branch point, sqrt(eps) ≈ 1.49e-8. The matrix
// entries carry an absolute rounding error δ ≈ eps. Above the branch point, roll and yaw come from
// atan2 of entries of magnitude cos(pitch), so their error is δ / cos(pitch); in the gimbal branch,
// pinning yaw = 0 misplaces the entries R(0,0) = cos(pitch)·cos(yaw) and R(1,0) by up to
// cos(pitch). The round-trip error max(δ / c, c) is least at c = sqrt(δ): both branches then
// reproduce the rotation to about sqrt(eps) rad. pitch itself is exact in both branches.
static const double GIMBAL_LOCK_COS_PITCH = std::sqrt(std::numeric_limits<double>::epsilon());
// [dimensionless] Eigen's float64 dummy_precision (1e-12): exported as EIGEN_DEFAULT_PREC, and the
// rigidity bound — a 3x3 is a rotation iff orthonormality_residual <= this.
constexpr double EIGEN_DEFAULT_PREC = Eigen::NumTraits<double>::dummy_precision();

// (N, 3) points or directions as numpy holds them: row-major, one point per row.
using PointsRowMajor = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

// (M, 4, 4) homogeneous matrices as numpy holds a stack of them: C order, row-major 4 x 4 per
// transform. An input that is strided, Fortran-ordered or not float64 is converted by nanobind
// first; an output owns a new buffer.
using TransformStackIn = nb::ndarray<nb::numpy, const double, nb::shape<-1, 4, 4>, nb::c_contig>;
using TransformStackOut = nb::ndarray<nb::numpy, double, nb::shape<-1, 4, 4>, nb::c_contig>;
using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

// Transform i of a stack, built as `Isometry3d(matrix)` builds one: assigned through matrix().
static Eigen::Isometry3d stack_isometry(const TransformStackIn& stack, std::size_t i) {
    Eigen::Isometry3d iso;
    iso.matrix() = Eigen::Map<const RowMajor4d>(stack.data() + 16 * i);
    return iso;
}

static TransformStackOut new_transform_stack(std::size_t m) {
    double* data = new double[16 * m];
    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
    return TransformStackOut(data, {m, 4, 4}, owner);
}

// nearest_rotation got a NaN or an infinity: the SVD of such a matrix is undefined.
struct NonFiniteMatrixError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// The rotation closest to `m` in the Frobenius norm (orthogonal Procrustes; the polar factor
// when det(m) > 0): U·diag(1, 1, det(U·Vᵀ))·Vᵀ from m = U·Σ·Vᵀ. Σ is sorted descending, so a
// reflection flips the smallest singular direction. JacobiSVD, not BDCSVD: BDCSVD hands
// matrices under 16 columns to JacobiSVD anyway, and two-sided Jacobi on a fixed-size 3x3
// runs without heap allocation and with high relative accuracy.
static Eigen::Matrix3d nearest_rotation(const Eigen::Matrix3d& m) {
    if (!m.allFinite())
        throw NonFiniteMatrixError("nearest_rotation: matrix has a non-finite entry");
    const Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(m);
    const double d = svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0 ? -1.0 : 1.0;
    return svd.matrixU() * Eigen::Vector3d(1.0, 1.0, d).asDiagonal() * svd.matrixV().transpose();
}

// max|RᵀR − I| of a 3x3, dimensionless: how far it is from orthonormal. A consumer bounds it to
// accept a matrix as a rotation; computed here so that check is Eigen's, as the rotation is.
// A NaN entry gives NaN: Eigen's default maxCoeff may skip a NaN coefficient, which read a NaN
// matrix as rigid. The trace carries it: diagonal j is |column j|² − 1, NaN iff column j has a NaN
// (an inf column makes it inf, never NaN). maxCoeff<PropagateNaN> instead costs ~1.7x at 200k.
static double orthonormality_residual(const Eigen::Matrix3d& r) {
    const Eigen::Matrix3d deviation = (r.transpose() * r - Eigen::Matrix3d::Identity()).cwiseAbs();
    const double trace = deviation.trace();
    return std::isnan(trace) ? trace : deviation.maxCoeff();
}

// A matrix offered as a rotation is not one: orthonormality_residual > EIGEN_DEFAULT_PREC, a
// non-finite entry, or (for an Isometry3d) a bottom row that is not [0, 0, 0, 1]. The rule is
// the one a consumer builds with, written as !(dev <= ...) so a NaN residual refuses.
struct NotRotationError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// A Hyperplane3d / ParametrizedLine3d input with no direction: a zero (or non-finite) normal or
// direction, coincident points, collinear points.
struct DegenerateGeometryError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

// [dimensionless, sin of the angle at p0] Hyperplane3d.Through's collinearity bound: Eigen's
// float64 dummy_precision. The cross product's rounding error is ≈ eps·|v1|·|v2|, so the plane's
// normal direction is off by ≈ eps / sine rad, at most ≈ 2.2e-4 rad at this bound.
constexpr double COLLINEAR_SINE = Eigen::NumTraits<double>::dummy_precision();

static void require_direction(const Eigen::Vector3d& v, const char* what) {
    if (!v.allFinite() || v.isZero(0.0)) {
        std::ostringstream ss;
        ss << what << " = [" << v.x() << ", " << v.y() << ", " << v.z() << "] has no direction (zero or not finite)";
        throw DegenerateGeometryError(ss.str());
    }
}

// A quaternion or axis with no direction: zero, or not finite. Every other quaternion names a
// rotation exactly (q and q/|q| rotate alike), as does every other axis.
struct DegenerateRotationError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

static const Eigen::Matrix3d& require_rotation(const Eigen::Matrix3d& r, const char* caller) {
    const double dev = orthonormality_residual(r);
    if (!(dev <= EIGEN_DEFAULT_PREC)) {
        std::ostringstream ss;
        ss << caller << ": the rotation is not orthonormal (max |Rᵀ·R − I| = " << dev << ", bound "
           << EIGEN_DEFAULT_PREC << ")";
        throw NotRotationError(ss.str());
    }
    return r;
}

// q/|q|: the unit quaternion of q's rotation. Eigen's toRotationMatrix and Transform(q) assume a
// unit quaternion and scale or shear otherwise.
static Eigen::Quaterniond unit_quaternion(const Eigen::Quaterniond& q, const char* caller) {
    const double norm = q.norm();
    if (!q.coeffs().allFinite() || !(norm > 0.0)) {
        std::ostringstream ss;
        ss << caller << ": the quaternion [x=" << q.x() << ", y=" << q.y() << ", z=" << q.z() << ", w=" << q.w()
           << "] has no rotation (zero or not finite)";
        throw DegenerateRotationError(ss.str());
    }
    return Eigen::Quaterniond(q.coeffs() / norm);
}

static Eigen::Vector3d unit_axis(const Eigen::Vector3d& axis, const char* caller) {
    const double norm = axis.norm();
    if (!axis.allFinite() || !(norm > 0.0)) {
        std::ostringstream ss;
        ss << caller << ": the axis [" << axis.x() << ", " << axis.y() << ", " << axis.z()
           << "] has no direction (zero or not finite)";
        throw DegenerateRotationError(ss.str());
    }
    return axis / norm;
}

// The 3x3 rotation block of transform i of a stack, as a fixed-size matrix: the scalar readouts
// on it give row i of the _each readouts bit for bit.
static Eigen::Matrix3d stack_rotation(const TransformStackIn& stack, std::size_t i) {
    return Eigen::Map<const RowMajor4d>(stack.data() + 16 * i).topLeftCorner<3, 3>();
}

// An Isometry3d of the caller's own Python type. `Pose` subclasses Isometry3d in Python, and a
// frame-tagged pose subclasses `Pose`; inverse and compose hand each its own type back, built here
// with no Python-level re-wrap (a `type(self)(...)` override in Python measured ~3x the plain call).
// No subclass `__init__` runs: the value is copied in place, as `Isometry3d(iso)` would.
static nb::object isometry_like(nb::handle self, const Eigen::Isometry3d& value) {
    nb::object result = nb::inst_alloc(self.type());
    new (nb::inst_ptr<Eigen::Isometry3d>(result)) Eigen::Isometry3d(value);
    nb::inst_mark_ready(result);
    return result;
}


void bind_eigen_geometry(nb::module_& m) {
    // Single source of truth for the float64 default precision used across
    // the project. Eigen exposes this as `NumTraits<double>::dummy_precision()`
    // (constexpr, returns 1e-12); we re-export so Python tests + helpers
    // can reference one value instead of duplicating `1e-12` literals.
    // Used by: planning/transforms.py (_NORMALISE_DENOM_FLOOR),
    // tests/tesseract/common/test_eigen_geometry.py (DEFAULT_PREC),
    // tests/tesseract_planning/test_planning_api.py (EIGEN_DEFAULT_PREC),
    // and the rotation and collinearity bounds in this module.
    m.attr("EIGEN_DEFAULT_PREC") = EIGEN_DEFAULT_PREC;

    // ========== Eigen Type Aliases ==========
    // Vector3d, VectorXd, MatrixXd are numpy arrays (nanobind/eigen/dense.h). The geometry types
    // are classes: a rigid transform or rotation keeps its invariant (see each constructor).
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
        // A rigid transform: bottom row exactly [0, 0, 0, 1] and an orthonormal rotation block
        // (the Quaterniond(rotation_matrix) rule); NotRotationError otherwise.
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Matrix4d& mat) {
            if (mat.row(3) != Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0) || !mat.allFinite()) {
                std::ostringstream ss;
                ss << "Isometry3d(matrix): the bottom row is [" << mat.row(3) << "], not [0 0 0 1]"
                   << (mat.allFinite() ? "" : ", or an entry is not finite");
                throw NotRotationError(ss.str());
            }
            require_rotation(mat.topLeftCorner<3, 3>(), "Isometry3d(matrix)");
            new (self) Eigen::Isometry3d();
            self->matrix() = mat;
        }, "matrix"_a)
        // Construct directly from rotation primitives so callers don't have
        // to assemble a 4x4 matrix by hand.
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Quaterniond& q) {
            new (self) Eigen::Isometry3d(unit_quaternion(q, "Isometry3d(rotation_quaternion)"));
        }, "rotation_quaternion"_a)
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::AngleAxisd& aa) {
            new (self) Eigen::Isometry3d(aa);  // AngleAxisd keeps a unit axis (its constructors)
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
            self->linear() = unit_quaternion(q, "Isometry3d(translation, rotation)").toRotationMatrix();
            self->translation() = t;
        }, "translation"_a, "rotation"_a)
        .def("__init__", [](Eigen::Isometry3d* self,
                            const Eigen::Translation3d& t,
                            const Eigen::Quaterniond& q) {
            new (self) Eigen::Isometry3d();
            self->setIdentity();
            self->linear() = unit_quaternion(q, "Isometry3d(translation, rotation)").toRotationMatrix();
            self->translation() = t.vector();
        }, "translation"_a, "rotation"_a)
        .def_static("Identity", []() { return Eigen::Isometry3d::Identity(); })
        .def("setIdentity", [](Eigen::Isometry3d& self) { self.setIdentity(); })
        // Stored-data accessors as properties (matches the project
        // convention: scalar component accessors on Quaterniond /
        // Translation3d / AngleAxisd are properties; this completes the
        // pattern for the rigid-transform getters). `matrix` returns the
        // full 4x4 homogeneous matrix; `translation` the 3-vector;
        // `linear` and `rotation` the upper-left 3x3 block. They agree because every
        // constructor and mutator above admits only rotations (NotRotationError,
        // DegenerateRotationError); `rotation` re-extracts it by SVD, `linear` reads it.
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
        .def("inverse", [](nb::pointer_and_handle<Eigen::Isometry3d> self) {
            return isometry_like(self.h, self.p->inverse());
        }, nb::sig("def inverse(self) -> typing.Self"))
        .def("__mul__", [](nb::pointer_and_handle<Eigen::Isometry3d> self, const Eigen::Isometry3d& other) {
            return isometry_like(self.h, *self.p * other);
        }, nb::sig("def __mul__(self, arg: Isometry3d, /) -> typing.Self"))
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Translation3d& t) {
            return Eigen::Isometry3d(self * t);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Quaterniond& q) {
            return Eigen::Isometry3d(self * unit_quaternion(q, "Isometry3d * Quaterniond"));
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) {
            return Eigen::Isometry3d(self * aa);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Vector3d& v) {
            return self * v;
        })
        // Batched apply over an (N, 3) array: one call instead of a Python loop
        // over `iso * p`. Each column runs the fixed-size `iso * v` itself, so
        // row i is bitwise `iso * points[i]`; a dynamic 3 x N product (GEMM or
        // lazy) vectorises and fuses differently, up to ~4e-16 apart (measured).
        // A row-major (N, 3) block is the same memory as a column-major 3 x N
        // one with the same outer stride, so a strided view (`a[:, :3]` of an
        // (N, 4)) is read in place; a Fortran or non-float64 array is converted
        // by nanobind first. The result is a new array. A (3,) input raises
        // TypeError: the single point is `iso * p`.
        .def("apply_points", [](const Eigen::Isometry3d& self,
                                const Eigen::Ref<const PointsRowMajor>& points) -> PointsRowMajor {
            const Eigen::Map<const Eigen::Matrix3Xd, 0, Eigen::OuterStride<>> columns(
                points.data(), 3, points.rows(), Eigen::OuterStride<>(points.outerStride()));
            PointsRowMajor mapped(points.rows(), 3);
            Eigen::Map<Eigen::Matrix3Xd> out(mapped.data(), 3, mapped.rows());
            for (Eigen::Index i = 0; i < out.cols(); ++i) {
                out.col(i) = self * Eigen::Vector3d(columns.col(i));
            }
            return mapped;
        }, "points"_a)
        // Rotation only: the same isometry with its translation zeroed, so a
        // direction runs the scalar sequence a point does (Eigen fuses the
        // translation into the product; `linear() * v` alone rounds differently).
        .def("apply_directions", [](const Eigen::Isometry3d& self,
                                    const Eigen::Ref<const PointsRowMajor>& vectors) -> PointsRowMajor {
            const Eigen::Map<const Eigen::Matrix3Xd, 0, Eigen::OuterStride<>> columns(
                vectors.data(), 3, vectors.rows(), Eigen::OuterStride<>(vectors.outerStride()));
            PointsRowMajor mapped(vectors.rows(), 3);
            Eigen::Isometry3d rotation = self;
            rotation.translation().setZero();
            Eigen::Map<Eigen::Matrix3Xd> out(mapped.data(), 3, mapped.rows());
            for (Eigen::Index i = 0; i < out.cols(); ++i) {
                out.col(i) = rotation * Eigen::Vector3d(columns.col(i));
            }
            return mapped;
        }, "vectors"_a)
        // `self * Isometry3d(stack[i])` for every transform of an (M, 4, 4) stack, in one call:
        // row i is bitwise the single product.
        .def("compose_each", [](const Eigen::Isometry3d& self, const TransformStackIn& stack) {
            const std::size_t m = stack.shape(0);
            TransformStackOut out = new_transform_stack(m);
            for (std::size_t i = 0; i < m; ++i) {
                Eigen::Map<RowMajor4d>(out.data() + 16 * i) = (self * stack_isometry(stack, i)).matrix();
            }
            return out;
        }, "transforms"_a)
        // `Isometry3d(stack[i]) * self` for every transform of an (M, 4, 4) stack: `compose_each`
        // with self on the right ("pre" as in `pretranslate`); row i is bitwise the single product.
        .def("precompose_each", [](const Eigen::Isometry3d& self, const TransformStackIn& stack) {
            const std::size_t m = stack.shape(0);
            TransformStackOut out = new_transform_stack(m);
            for (std::size_t i = 0; i < m; ++i) {
                Eigen::Map<RowMajor4d>(out.data() + 16 * i) = (stack_isometry(stack, i) * self).matrix();
            }
            return out;
        }, "transforms"_a)
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
            self.rotate(unit_quaternion(q, "Isometry3d.rotate"));
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("rotate", [](Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) -> Eigen::Isometry3d& {
            self.rotate(aa);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("rotate", [](Eigen::Isometry3d& self, const Eigen::Matrix3d& R) -> Eigen::Isometry3d& {
            self.rotate(require_rotation(R, "Isometry3d.rotate"));
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::Quaterniond& q) -> Eigen::Isometry3d& {
            self.prerotate(unit_quaternion(q, "Isometry3d.prerotate"));
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) -> Eigen::Isometry3d& {
            self.prerotate(aa);
            return self;
        }, "rotation"_a, nb::rv_policy::reference_internal)
        .def("prerotate", [](Eigen::Isometry3d& self, const Eigen::Matrix3d& R) -> Eigen::Isometry3d& {
            self.prerotate(require_rotation(R, "Isometry3d.prerotate"));
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
        .def(nb::init<double, double, double>(), "x"_a, "y"_a, "z"_a)
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

    // Scalar-last everywhere: no positional four-scalar constructor, which in Eigen is scalar-first
    // (w, x, y, z) and next to the (x, y, z, w) array form read the same numbers as two rotations.
    // Scalars go through from_xyzw(qx, qy, qz, qw).
    nb::class_<Eigen::Quaterniond>(m, "Quaterniond")
        // Constructor from rotation matrix — validates orthonormality so
        // callers cannot smuggle scaling or shear into a "rotation."
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::Matrix3d& rot) {
            new (self) Eigen::Quaterniond(require_rotation(rot, "Quaterniond(rotation_matrix)"));
        }, "rotation_matrix"_a)
        // Construct from AngleAxisd
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::AngleAxisd& aa) {
            new (self) Eigen::Quaterniond(aa);
        }, "angle_axis"_a)
        // Construct from a scalar-last [x, y, z, w] array; round-trips with `coeffs()`.
        // Uses Eigen's pointer ctor so there is no uninitialised intermediate.
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::Vector4d& xyzw) {
            new (self) Eigen::Quaterniond(xyzw.data());
        }, "xyzw"_a)
        .def_static("Identity", []() { return Eigen::Quaterniond::Identity(); })
        // Minimal-rotation quaternion taking `a` to `b` (Eigen handles the
        // antipodal case and gives an orthogonal-axis 180-deg rotation).
        .def_static("FromTwoVectors", [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return Eigen::Quaterniond().setFromTwoVectors(a, b);
        }, "a"_a, "b"_a)
        // Project-canonical scalar-last factory for four scalars.
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
            if (cos_pitch > GIMBAL_LOCK_COS_PITCH) {
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

    // Eigen's AngleAxis assumes a unit axis; the constructors give it one, so `axis` is unit.
    nb::class_<Eigen::AngleAxisd>(m, "AngleAxisd")
        .def("__init__", [](Eigen::AngleAxisd* self, double angle, const Eigen::Vector3d& axis) {
            new (self) Eigen::AngleAxisd(angle, unit_axis(axis, "AngleAxisd(angle, axis)"));
        }, "angle"_a, "axis"_a, "Rotation by `angle` [rad] about `axis` (normalized; zero raises).")
        // Construct from a Quaterniond.
        .def("__init__", [](Eigen::AngleAxisd* self, const Eigen::Quaterniond& q) {
            new (self) Eigen::AngleAxisd(unit_quaternion(q, "AngleAxisd(quaternion)"));
        }, "quaternion"_a)
        // Construct from a 3x3 rotation matrix.
        .def("__init__", [](Eigen::AngleAxisd* self, const Eigen::Matrix3d& R) {
            new (self) Eigen::AngleAxisd(require_rotation(R, "AngleAxisd(rotation_matrix)"));
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

    // ========== apply_directions_each ==========
    // Row i of an (M, 3) array rotated by transform i of an (M, 4, 4) stack: bitwise
    // `Isometry3d(stack[i]).apply_directions(vectors[i:i + 1])[0]`, without M Python calls.
    m.def("apply_directions_each", [](const TransformStackIn& stack,
                                      const Eigen::Ref<const PointsRowMajor>& vectors) -> PointsRowMajor {
        const std::size_t count = stack.shape(0);
        if (static_cast<Eigen::Index>(count) != vectors.rows())
            throw std::invalid_argument("apply_directions_each: " + std::to_string(count) + " transforms but " +
                                        std::to_string(vectors.rows()) + " vector rows");
        PointsRowMajor rotated(vectors.rows(), 3);
        for (std::size_t i = 0; i < count; ++i) {
            Eigen::Isometry3d rotation = stack_isometry(stack, i);
            rotation.translation().setZero();
            rotated.row(static_cast<Eigen::Index>(i)) =
                (rotation * Eigen::Vector3d(vectors.row(static_cast<Eigen::Index>(i)).transpose())).transpose();
        }
        return rotated;
    }, "transforms"_a, "vectors"_a);

    // ========== nearest_rotation ==========
    nb::exception<NotRotationError>(m, "NotRotationError", PyExc_ValueError)
        .attr("__doc__") = "A matrix offered as a rotation (or rigid transform) is not one: max|RᵀR − I| above "
                           "EIGEN_DEFAULT_PREC, a non-finite entry, or a bottom row other than [0, 0, 0, 1].";
    nb::exception<DegenerateRotationError>(m, "DegenerateRotationError", PyExc_ValueError)
        .attr("__doc__") = "A quaternion or rotation axis that is zero or not finite: it names no rotation.";
    nb::exception<DegenerateGeometryError>(m, "DegenerateGeometryError", PyExc_ValueError)
        .attr("__doc__") = "A Hyperplane3d / ParametrizedLine3d input with no direction: a zero normal or "
                           "direction, coincident points, or collinear points.";
    nb::exception<NonFiniteMatrixError>(m, "NonFiniteMatrixError", PyExc_ValueError)
        .attr("__doc__") = "nearest_rotation got a matrix with a NaN or infinite entry.";
    m.def("nearest_rotation", &nearest_rotation, "matrix"_a,
          "The rotation closest to a 3x3 matrix in the Frobenius norm, U·diag(1, 1, det(U·Vᵀ))·Vᵀ "
          "from its SVD; det is +1 also for a reflection. A new array; the input is unchanged.");

    // ========== rigidity readouts ==========
    m.def("orthonormality_residual", &orthonormality_residual, "matrix"_a,
          "max|RᵀR − I| of a 3x3 matrix, dimensionless: 0 for an exact rotation.");
    m.def("determinant", [](const Eigen::Matrix3d& r) { return r.determinant(); }, "matrix"_a,
          "The determinant of a 3x3 matrix: +1 for a rotation, -1 for a reflection.");
    m.def("orthonormality_residual_each", [](const TransformStackIn& stack) {
        const std::size_t count = stack.shape(0);
        Eigen::VectorXd residual(static_cast<Eigen::Index>(count));
        for (std::size_t i = 0; i < count; ++i)
            residual(static_cast<Eigen::Index>(i)) = orthonormality_residual(stack_rotation(stack, i));
        return residual;
    }, "transforms"_a,
          "max|RᵀR − I| of the rotation block of each transform of an (M, 4, 4) stack, (M,); row i is "
          "bitwise orthonormality_residual(stack[i, :3, :3]).");
    m.def("rotation_determinant_each", [](const TransformStackIn& stack) {
        const std::size_t count = stack.shape(0);
        Eigen::VectorXd det(static_cast<Eigen::Index>(count));
        for (std::size_t i = 0; i < count; ++i)
            det(static_cast<Eigen::Index>(i)) = stack_rotation(stack, i).determinant();
        return det;
    }, "transforms"_a,
          "The determinant of the rotation block of each transform of an (M, 4, 4) stack, (M,); row i "
          "is bitwise determinant(stack[i, :3, :3]).");
    m.def("norm_each", [](const Eigen::Ref<const PointsRowMajor>& vectors) -> Eigen::VectorXd {
        return vectors.rowwise().norm();
    }, "vectors"_a, "The Euclidean norm of each row of an (N, 3) array, (N,).");

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
    // Degenerate inputs are refused scale-free: a normal or direction is degenerate only when it
    // is exactly zero (or not finite), two points only when they are equal; a nonzero vector has
    // a direction at any length, and points 1e-7 m apart are not coincident. Collinearity is a
    // dimensionless test on sin of the angle between p1 − p0 and p2 − p0 (COLLINEAR_SINE).
    nb::class_<Hyperplane3d>(m, "Hyperplane3d")
        // Normal + signed offset: plane is {x : normal · x + offset = 0}.
        .def("__init__", [](Hyperplane3d* self,
                            const Eigen::Vector3d& normal,
                            double offset) {
            require_direction(normal, "Hyperplane3d(normal, offset): normal");
            new (self) Hyperplane3d(normal, offset);
        }, "normal"_a, "offset"_a)
        // Normal + point-on-plane: computes offset = -normal · point.
        .def("__init__", [](Hyperplane3d* self,
                            const Eigen::Vector3d& normal,
                            const Eigen::Vector3d& point) {
            require_direction(normal, "Hyperplane3d(normal, point): normal");
            new (self) Hyperplane3d(normal, point);
        }, "normal"_a, "point"_a)
        // Plane through three non-collinear points, as Eigen's Through: normal =
        // ((p2 − p0) × (p1 − p0)).normalized(), offset = −normal · p0. Points listed
        // counter-clockwise seen from the side the normal points to are p0, p2, p1.
        // Collinear inputs are rejected — Eigen silently falls back to an
        // SVD-derived perpendicular, but that plane is mathematically arbitrary
        // and almost never what the caller meant.
        .def_static("Through", [](const Eigen::Vector3d& p0,
                                  const Eigen::Vector3d& p1,
                                  const Eigen::Vector3d& p2) {
            const Eigen::Vector3d v1 = p1 - p0;
            const Eigen::Vector3d v2 = p2 - p0;
            require_direction(v1, "Hyperplane3d.Through: p1 - p0");
            require_direction(v2, "Hyperplane3d.Through: p2 - p0");
            const double sine = v1.cross(v2).norm() / (v1.norm() * v2.norm());
            if (!(sine > COLLINEAR_SINE)) {
                std::ostringstream ss;
                ss << "Hyperplane3d.Through: points are collinear (sin of the angle at p0 = " << sine
                   << ", bound " << COLLINEAR_SINE << ")";
                throw DegenerateGeometryError(ss.str());
            }
            return Hyperplane3d(Hyperplane3d::Through(p0, p1, p2));
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
            require_direction(direction, "ParametrizedLine3d(origin, direction): direction");
            new (self) ParametrizedLine3d(origin, direction);
        }, "origin"_a, "direction"_a)
        // Line through two distinct points: origin = p0, direction = (p1 − p0).normalized().
        .def_static("Through", [](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
            require_direction(p1 - p0, "ParametrizedLine3d.Through: p1 - p0");
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

    // Note: LinkIdTransformMap (unordered_map<LinkId, Isometry3d>) is handled by nanobind's
    // stl/unordered_map caster: a dict keyed by LinkId (or str, implicitly) with Isometry3d values

}
