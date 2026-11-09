# Audit contract fixture: binds C++ `ns::Settings` as `RenamedSettings` (trajopt_sqp binds
# `piqp::Settings<double>` as `PIQPSettings`); another module takes it as a parameter.
class RenamedSettings:
    def __init__(self) -> None: ...

class Unrelated:
    def __init__(self) -> None: ...

# Bound as `Eigen::Transform` (tesseract_common's `Isometry3d`): a C++ parameter of it is keyed
# by this Python name already (`audit_types.EIGEN_CLASSES`), never by the leaf `Transform`.
class OwnerPose:
    def __init__(self) -> None: ...

# Bound as a wrapper the binding declares itself, outside any namespace (trajopt_ifopt's
# constraints as `VarCheckedConstraint<T>`): no header parameter names the wrapper.
class OwnerWrapped:
    def __init__(self) -> None: ...
