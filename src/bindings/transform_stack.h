#pragma once

// (M, 4, 4) stacks of rigid transforms and the rule a matrix must meet to be one. Shared by the
// tesseract_common readouts (common_eigen.cpp: Isometry3d(matrix), compose_each, ...) and the
// contact managers' setCollisionObjectsTransform(ids, transforms) (collision.cpp), so both refuse
// exactly the same matrices with exactly the same message.

#include "tesseract_nb.h"

#include <cmath>
#include <optional>
#include <sstream>
#include <string>

// [dimensionless] Eigen's float64 dummy_precision (1e-12): exported as EIGEN_DEFAULT_PREC, and the
// rigidity bound — a 3x3 is a rotation iff orthonormality_residual <= this.
constexpr double EIGEN_DEFAULT_PREC = Eigen::NumTraits<double>::dummy_precision();

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

// Why `r` is not a rotation (orthonormality_residual above EIGEN_DEFAULT_PREC, or NaN), or nothing
// if it is one. Written as !(dev <= ...) so a NaN residual refuses.
static std::optional<std::string> rotation_defect(const Eigen::Matrix3d& r) {
    const double dev = orthonormality_residual(r);
    if (dev <= EIGEN_DEFAULT_PREC)
        return std::nullopt;
    std::ostringstream ss;
    ss << "the rotation is not orthonormal (max |Rᵀ·R − I| = " << dev << ", bound " << EIGEN_DEFAULT_PREC << ")";
    return ss.str();
}

// Why `mat` is not a rigid transform, or nothing if it is one: the bottom row is exactly
// [0, 0, 0, 1], every entry is finite and the rotation block passes rotation_defect. The rule
// Isometry3d(matrix) enforces; a caller prefixes its own name and raises NotRotationError.
static std::optional<std::string> rigid_transform_defect(const Eigen::Matrix4d& mat) {
    if (mat.row(3) != Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0) || !mat.allFinite()) {
        std::ostringstream ss;
        ss << "the bottom row is [" << mat.row(3) << "], not [0 0 0 1]"
           << (mat.allFinite() ? "" : ", or an entry is not finite");
        return ss.str();
    }
    return rotation_defect(mat.topLeftCorner<3, 3>());
}
