# Binding conventions

The rules every nanobind binding in `src/bindings/` follows, so a user can predict a name, an
error or a lifetime without reading the C++. Each rule names the code that implements it; a new
binding follows them from its first commit.

## Names

One rule, applied per name:

1. **A member that wraps a C++ symbol keeps the C++ name and the C++ semantics.**
   `Environment.getState`, `PropertyTree.fromYAML`, `Quaterniond.FromTwoVectors`,
   `Hyperplane3d.Through` (Eigen's normal sign, not a right-hand-rule variant),
   `parseURDFFile`. A port of C++ code reads the same in Python.
2. **A Python-only addition is `snake_case`.** `Environment.link_transforms_batch`,
   `Isometry3d.apply_points`, `Quaterniond.from_xyzw` / `from_rpy`, `Profile.from_yaml`,
   the `serialization` functions (`environment_to_file`, ...), `nearest_rotation`.
3. **A Python-only helper named after a C++ type keeps the type's spelling** after a
   `snake_case` verb: `cast_CommandAppliedEvent`, `AnyPoly_wrap_CompositeInstruction`.
4. **A Python-only member that completes a C++ class's own interface takes the class's
   convention**, so one class reads in one style: `Link.addVisual` beside upstream's
   `Link.getName`. The same holds for a function that returns part of a C++ function's result:
   `calcManipulabilityVolume` beside upstream's `calcManipulability`.

No name has an alias. A C++ overload set is one Python name with `@overload`s; a Python-only
alias of a native overload (`getStateByMap`, `calcInvKinMultiple`, ...) does not exist.

Properties are used for stored scalar components of value types (`Quaterniond.x`,
`Isometry3d.translation`); methods for everything that mirrors a C++ method.

## Units and frames

Units are SI throughout (metres, radians, seconds) and live in the parameter and attribute names
(`radius_m`, `z_m`, `joint_positions` documented in rad) and in the docstrings; frames live in the
names of the frames themselves (`working_frame`, `tcp_frame`) and in `ManipulatorInfo`. They are not
encoded as `NewType`s or frame-tagged `Pose` generics.

Decision (0.36): a `Radians` / `Metres` `NewType` over `float` makes every literal argument a type
error at the call site (`box(0.5, 0.5, 0.5)` would need `box(Metres(0.5), ...)`), arrays cannot
carry a `NewType` at all, and every value crossing the C++ boundary would need wrapping on the way
back. That is a large ergonomic cost for users, and it would not catch the errors that matter in
practice, which are frame mix-ups, not unit mix-ups. A frame-tagged `Pose` would catch those, but
it needs every Tesseract API that takes a transform to name its frame in the type, which the C++
does not; the bindings would have to invent it. Where a frame matters at a boundary we own, the
code carries it explicitly instead: the emitters' `FramedPose` pairs a pose with its
`MoveFrames`, and an unbound frame raises `UnboundFrameError`.

## Errors

- One named exception per failure mode, subclassing the builtin a caller would catch:
  `PolyTypeError(TypeError)`, `InstructionHeldError(BufferError)`,
  `NotRotationError(ValueError)`, `DegenerateAxisError(ValueError)`,
  `URDFWriteError(RuntimeError)`. The message names the actual value and what was expected.
- A lookup by one id or name that misses is a `KeyError`: `Environment.getLink("x")`,
  `getJointGroup("x")`, `getKinematicGroup("g", "solver")`, `getDiscreteContactManager("x")`.
  A list argument holding unknown ids is a `ValueError` naming every unknown id
  (`setState`, `getActiveLinkIds(joint_ids)`).
- A file that cannot be read or written is an `OSError` (`FileNotFoundError` for a missing
  one) naming the path: the `serialization` functions, every plugin factory's config path
  (upstream dereferences an unresolved locator result: `require_located_file`,
  `src/tesseract_nb.h`).
- Upstream's nested exceptions (`std::throw_with_nested`) reach Python with the whole chain in
  the message (`urdf.writeURDFFile`).
- A precondition upstream only `assert()`s is checked by the binding before the call.

## Values, references and lifetimes

- Scalar-last quaternions everywhere: `[qx, qy, qz, qw]`. `Quaterniond` has no positional
  four-scalar constructor (Eigen's is scalar-first); use `Quaterniond.from_xyzw(...)` or
  `Quaterniond([x, y, z, w])`.
- A rigid-transform type holds a rigid transform: `Isometry3d(matrix)` and
  `Quaterniond(rotation_matrix)` refuse a non-orthonormal rotation block (`NotRotationError`);
  a non-unit quaternion or axis names the rotation of its direction and is normalized; a zero
  one raises `DegenerateRotationError`.
- A getter returns a copy unless its docstring says "by reference". A reference into a C++
  container is kept valid by refusing the calls that would move or free it while Python holds
  it (`ContactResultVector`, `CompositeInstruction`: `nb::find` on the element addresses), not
  by documentation.
- An object handed to a Python callback is valid after the callback returns: environment events
  carry their own copy of the history or state; `ContactResultMap.filter` passes a copy that is
  copied back. Where a copy is impossible (`QPProblem` in an `SQPCallback`), the object expires
  and raises when used later.
- An object whose code lives in a plugin library keeps the object that owns that library alive
  (`keep_alive<0, 1>` on `clone()`, the contact-manager and kinematics getters): gh-72.
- Paths are `str | os.PathLike` where the argument can only be a path; where a `str` could also
  be content (URDF text, YAML text), the path overload takes `os.PathLike` only (`StrictPath`).
- Binary archives are `bytes`.
- Containers of Eigen values (`VectorVector3d`, `VectorIsometry3d`) are Python lists.

## Threads

[Threading and the GIL](threading.md) states when a binding releases the GIL; every
`Environment` method does, so an event callback and a writer on another thread cannot deadlock.

## Docstrings and stubs

Every public class and method carries an `nb::doc` string (or a docstring argument), so the
committed `.pyi` stubs (`pixi run stubs`) carry it. Defaults restated from upstream are named
constants with units and source; where an upstream object exposes the default
(`CollisionCheckConfig().longest_valid_segment_length`), it is read from there.
