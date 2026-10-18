"""The audit's rule tables: what each accepted deviation means and where it applies."""

from __future__ import annotations

import re

from audit_parse import UNBOUND_DEPENDENCY

# C++ members a Python protocol dunder covers (container-protocol, iterator-pair rules): a
# class that binds the dunder needs no binding under the C++ name.
PROTOCOL_MEMBERS = {
    "size": "__len__",
    "begin": "__iter__",
    "end": "__iter__",
    "cbegin": "__iter__",
    "cend": "__iter__",
    "clone": "__deepcopy__",  # clone-as-copy (R9)
    "erase": "__delitem__",  # container-protocol: `del c[i]`, `del c[i:j]`
}
# Python protocol dunders accepted by rule: dunder -> (ACCEPTED key, C++ members, keyed as in
# `cpp_api`, the owner must declare). An owner with no C++ class (a bound std container
# typedef) is checked against the TU's declaration names instead.
PROTOCOL_RULES = {
    "__len__": ("container-protocol", ("size",)),
    "__setitem__": ("container-protocol", ("__getitem__",)),
    "__delitem__": ("container-protocol", ("erase",)),
    "__iter__": ("iterator-pair", ("begin", "end")),
    "__repr__": ("presentation-dunder", ()),
    "__str__": ("presentation-dunder", ()),
    # Bind the C++ copy constructor: no member to declare (R9).
    "__copy__": ("clone-as-copy", ()),
    "__deepcopy__": ("clone-as-copy", ()),
}
# Deviations accepted by rule; each is printed with its reason in the report.
ACCEPTED = {
    "out-param": "A non-const lvalue-reference out-param (or a by-value `Eigen::Ref` of a "
    "non-const matrix) is returned in a tuple with the result (Phase A precedent: "
    "checkTrajectory), or alone when the C++ returns `void`.",
    "stringstream": "A `std::stringstream&` parameter the C++ writes into is returned as `str`.",
    "byte-buffer": "A `(const uint8_t* bytes, size_t len)` pair is one Python `bytes` argument: "
    "the binding passes the buffer and its size (`createMeshFromBytes`, `BytesResource`).",
    "scalar-last-quaternion": "Quaterniond takes (x, y, z, w), the project-wide scalar-last "
    "order; Eigen's constructor is (w, x, y, z).",
    "container-protocol": "`size()` is bound as `__len__`, `operator[]` as "
    "`__getitem__`/`__setitem__` and `erase` (one element, or a range) as `__delitem__` (of an "
    "index, or a slice), the Python container protocol.",
    "iterator-pair": "A `begin()`/`end()` pair (and `cbegin`/`cend`) is bound as `__iter__`.",
    "stream-insertion": "A free `operator<<(std::ostream&, const T&)` is bound as `T.__str__` "
    "(`T` a class or an enum).",
    "presentation-dunder": "`__repr__` and `__str__` are Python presentation and need no C++ "
    "counterpart.",
    "serialization-default-ctor": "The default constructor of a class that befriends cereal "
    "`serialize` and declares another public constructor exists for deserialization only.",
    "value-equality-unhashable": "`__hash__ = None` next to a bound `__eq__` makes a mutable "
    "value type unhashable, as Python does for any class that defines `__eq__`; it needs no "
    "C++ counterpart.",
    "eigen-template-instance": "`Eigen::Hyperplane<double, 3>` and "
    "`Eigen::ParametrizedLine<double, 3>` have no Eigen typedef; the class takes Eigen's own "
    "`…3d` naming (`Vector3d`, `Quaterniond`).",
    "quaternion-rpy": "`Quaterniond.from_rpy`/`to_rpy` convert roll-pitch-yaw in the ROS/tf2 "
    "convention, which Eigen spells as three composed `AngleAxisd` rotations; `to_rpy` returns "
    "tf2's canonical ranges, which `eulerAngles` does not guarantee.",
    "eigen-default-precision": "`EIGEN_DEFAULT_PREC` re-exports "
    "`Eigen::NumTraits<double>::dummy_precision()` (1e-12) so Python compares with Eigen's "
    "own default tolerance instead of a duplicated literal.",
    "template-instance-name": "Python cannot spell a C++ template argument, so each instance of "
    "a function template gets its own function named after the type (`createMeshFromPath<ConvexMesh>` "
    "→ `createConvexMeshFromPath`, `Serialization::toArchiveStringXML<Environment>` → "
    "`environment_to_xml`); for a library's generic class name, the library prefix plus the "
    "instance suffix (`descartes_light::State<double>` → `DescartesStateD`), because the generic "
    "name collides with other bound classes (`ompl_base.State`).",
    "copied-vector-field-mutator": "A `std::vector` field converts to a fresh Python list on each "
    "access, so `obj.field.append(x)` edits a copy; `add…`/`clear…` methods mutate the C++ vector.",
    "boost-graph-property-tag": "Boost Graph property tags (`vertex_link_t`, …) select the "
    "properties `SceneGraph` stores; Python reaches the properties through `SceneGraph` methods.",
    "bound-in-other-module": "The class is bound in another extension module, which defines its "
    "Python type once for every module; its members are compared against that module's stub.",
    "yaml-node": "An overload taking a `YAML::Node` has no Python form (no Python type converts "
    "to one). It is accepted when a bound sibling overload of the same symbol takes a config file "
    "path or YAML text in that position and loads the same config, or the class binds a static "
    "`from_yaml(text: str)` that parses the text and calls this constructor (supplying a "
    "default `ProfilePluginFactory` where the constructor takes one).",
    "derived-element-container": "An overload taking a container of a derived class only upcasts "
    "into its base-element sibling (`CompoundMesh(vector<ConvexMesh>)` → "
    "`CompoundMesh(vector<PolygonMesh>)`, compound_mesh.cpp:59-63); Python passes a list of any "
    "subclass to the bound base-element overload.",
    "module-getattr": "A package `__init__.py` defines a PEP 562 module `__getattr__` to re-export "
    "a name lazily (an eager import would be an import cycle); it is an attribute-lookup hook, "
    "not API.",
    # #256 group 1.
    "reference-wrapper-view": "An overload taking `std::vector<std::reference_wrapper<T>>` "
    "views objects owned elsewhere; Python passes the owner to a bound sibling overload, which "
    "builds the same view (`InstructionsTrajectory(program)`). Accepted only when a sibling "
    "overload is covered.",
    "reference-wrapper-out-param": "`ContactResultMap.flattenWrapperResults` fills a "
    "`std::vector<std::reference_wrapper<ContactResult>>` with references into the map, which "
    "have no Python form; `flattenCopyResults` and `flattenMoveResults` return the same results "
    "(phaseB won't fix, C12).",
    "profile-key-template": "`ProfileDictionary::getProfile<ProfileType>(ns, name, default)` "
    "derives the profile key from its template argument; Python passes the key to the bound "
    "`getProfile(key, ns, name)` (`profile.getKey()`, or `ProfileType.createKey()` once bound), "
    "which returns the registered profile class, and checks `hasProfile` for the default.",
    "free-function-as-method": "A free predicate whose only parameter is `const T&` is bound as "
    "a method of `T` (`isWaitInstruction(const InstructionPoly&)` → "
    "`InstructionPoly.isWaitInstruction()`), beside `T`'s member predicates of the same shape "
    "(`isMoveInstruction`).",
    "type-index": "`getType()` returns a `std::type_index`, which has no Python form; the bound "
    "`is…()` predicates and `as…()` casts answer the same question by name.",
    "type-erasure-interface": "The abstract `…Interface` of a type-erased `…Poly` is its C++ "
    "extension point (pure virtuals, a protected `equals`, cereal registration); Python holds "
    "the concrete classes and their Polys, and reaches the stored object through the bound casts "
    "(`InstructionPoly.as…`, and the waypoint casts), not through the accessor that returns the "
    "interface by reference.",
    "algorithm-internal-namespace": "`time_parameterization::totg::{Path, PathSegment, PathData, "
    "Trajectory}` are the implementation classes of the bound `TimeOptimalTrajectoryGeneration`; "
    "Python reaches them through `compute`.",
    "reference-setter": "C++ assigns through a getter that returns a non-const reference "
    "(`getWaypoint() = wp`); Python cannot assign to a returned reference, so a `set…` method "
    "assigns through it.",
    "vector-forwarding-no-python-form": "A class that forwards the `std::vector` interface "
    "leaves `data()` (raw pointer), `emplace`/`emplace_back` (perfect-forwarding templates), "
    "the iterator-range `insert(pos, first, last)` template, `swap(std::vector&)` and the reverse "
    "iterators unbound: Python builds the element and passes it to `push_back`/`insert`, and "
    "`reversed()` walks any sequence with `__len__` and `__getitem__`.",
    "clone-as-copy": "`__copy__`/`__deepcopy__` bind the C++ copy constructor and return the "
    "concrete type; `clone()`, which returns the `unique_ptr<…Interface>` of the type erasure, "
    "is then covered by `__deepcopy__`.",
    "poly-alternative-overloads": "A parameter typed as a type-erased Poly (`InstructionPoly`, "
    "`WaypointPoly`) is also bound once per alternative the Poly wraps "
    "(`push_back(MoveInstructionPoly)`, `MoveInstruction(CartesianWaypointPoly, …)`), so the stub "
    "names the types a caller passes; C++ converts each alternative through the Poly's "
    "converting constructor.",
    # Group 2 (trajopt).
    "k-prefix-enumerator": "trajopt's `kCamelCase` enumerators are bound in `UPPER_SNAKE` without "
    "the `k` (`kLowerBound` → `LOWER_BOUND`, `kQPSolveFailed` → `QP_SOLVE_FAILED`), the Python "
    "enum convention every tesseract enum follows; an enumerator bound under any other name stays "
    "a gap and a deviation.",
    UNBOUND_DEPENDENCY: "A parameter, return or field typed by a library no module binds "
    "(`OsqpEigen::Settings`, `ompl::base::SpaceInformationPtr`), or by a type declared only in a "
    "header this report lists as not API (`trajopt::TermInfo`, `sco::ModelConfig`), has no "
    "Python form; the owning object exposes what Python needs instead: bound fields, or `set…` "
    "forwarders such as `OSQPEigenSolver`'s into its OSQP settings (`setPolish`, "
    "`setWarmStart(bool)`, `setMaxIteration`, …).",
    "parent-back-pointer": "`Var.getParent()` and `Node.getParent()` return a raw `const T*` to "
    "the owning container; Python reaches a `Var` or `Node` from its owner "
    "(`NodesVariables.getNodes`, `Node.getVar`), and a bound back-pointer would dangle once the "
    "owner is collected.",
    "stdout-printer": "`Component.print(index, tolerance)` writes a table to `std::cout`, which "
    "bypasses `sys.stdout`; Python reads the same values through the bound `getValues` and "
    "`getBounds`.",
    "unimplemented-observer": "`NodesObserver` has no implementation in trajopt 0.35.0 (the "
    "library exports only its base constructor), so `NodesVariables.addObserver` has nothing to "
    "register.",
    "split-var-pair": "`ContinuousCollisionConstraint` takes "
    "`std::array<std::shared_ptr<const Var>, 2> position_vars`; Python passes the pair as two "
    "named parameters, `position_var0` and `position_var1`, and the required "
    "`vars0_fixed`/`vars1_fixed` default to `False`.",
    "ifopt-problem-only": "An overload taking `trajopt_ifopt::Problem` serves the "
    "`IfoptQPProblem` path, which is not Python API (`trajopt_sqp/ifopt_qp_problem.h` is "
    "unaudited); `TrajOptQPProblem` is the bound problem.",
    "nodes-variables-factory": "`createNodesVariables` builds a `NodesVariables` of one vector "
    "`Var` per waypoint in one call, the layout every `TrajOptQPProblem` example uses; the C++ "
    "constructor takes `std::vector<std::unique_ptr<Node>>`, which this factory fills.",
    # Group 4 (task composer).
    "plugin-author-factory-interface": "An abstract `…Factory` whose pure virtual `create(…)` "
    "takes a `YAML::Node` is the interface a C++ plugin library implements; Python creates "
    "objects through the owning plugin factory (`TaskComposerPluginFactory`, "
    "`KinematicsPluginFactory`, `ContactManagersPluginFactory`) and cannot implement a plugin.",
    "plugin-config-ctor": "A `(…, const YAML::Node& config)` constructor that only the plugin "
    "factory calls (`TaskComposerExecutorFactoryImpl<T>::create(name, config)` passes the "
    "plugin's config block) has no Python caller; Python constructs the class from its other "
    "constructors or gets it from `TaskComposerPluginFactory.createTaskComposerExecutor`.",
    "std-library-enum": "The enum is declared by the C++ standard library, not an audited header; "
    "it is bound because a bound member returns it (`std::future_status` → `FutureStatus`, "
    "returned by `TaskComposerFuture.waitFor`).",
    "clock-deadline": "`TaskComposerFuture.waitUntil` takes a `std::chrono::high_resolution_clock` "
    "time point, which is `steady_clock` under libc++ and MSVC and has no epoch Python can name; "
    "`waitFor(duration)` is its Python form.",
    "context-from-data-storage": "`TaskComposerExecutor.run(node, data_storage, dotgraph=False)` "
    "builds the `TaskComposerContext` that the C++ `run(node, context)` takes, the shape of "
    "upstream's `TaskComposerServer::run(node, data_storage, dotgraph, executor_name)`.",
    "dot-dump-hook": "`TaskComposerNode::dump(std::ostream&, …)` and its overrides are the virtual "
    "hook behind `getDotgraph`/`saveDotgraph`, which return or write the same DOT text.",
    # Group 3 (motion planners).
    "type-erased-data": "`PlannerRequest.data` and `PlannerResponse.data` are "
    "`std::shared_ptr<void>`: a planner's internal problem, type-erased; Python has no type to "
    "convert `void` to or from.",
    "third-party-global": "A module function wraps a process-global of a linked third-party "
    "library (`RNG_setSeed`/`RNG_getSeed` call `ompl::RNG::setSeed`/`getSeed`); no owned header "
    "declares it, and Python needs it for reproducible planning (#103, #109).",
    "solver-behind-profile": "The classic-TrajOpt optimizer (`sco::Optimizer`, "
    "`BasicTrustRegionSQP`, `BasicTrustRegionSQPMultiThreaded`, `OptResults`, "
    "`BasicTrustRegionSQPResults`, `OptStatus`, `toString(OptStatus)`) shares "
    "`trajopt_sco/optimizers.hpp` with the bound `BasicTrustRegionSQPParameters`; "
    "`TrajOptMotionPlanner.solve` runs it, and Python sets its parameters through "
    "`TrajOptSolverProfile.opt_params`.",
    "eigen-value-wrapper": "`descartes_light::State` wraps one Eigen vector, `values`; its "
    "indexing, call, arithmetic, comma-initializer and conversion operators forward to it, and "
    "Python does the same on the bound `values` array.",
}
# Python and C++ names accepted by a named rule: (module, name) -> ACCEPTED key.
ACCEPTED_SYMBOLS = {
    ("tesseract_common", "Quaterniond.__init__"): "scalar-last-quaternion",
    ("tesseract_common", "Quaterniond.from_xyzw"): "scalar-last-quaternion",
    ("tesseract_common", "Hyperplane3d"): "eigen-template-instance",
    ("tesseract_common", "ParametrizedLine3d"): "eigen-template-instance",
    ("tesseract_common", "Quaterniond.from_rpy"): "quaternion-rpy",
    ("tesseract_common", "Quaterniond.to_rpy"): "quaternion-rpy",
    ("tesseract_common", "EIGEN_DEFAULT_PREC"): "eigen-default-precision",
    **{
        ("tesseract_geometry", f"create{t}MeshFrom{s}"): "template-instance-name"
        for t in ("Convex", "SDF")
        for s in ("Path", "Resource", "Bytes")
    },
    **{
        ("tesseract_serialization", f"{t}_{d}_{f}"): "template-instance-name"
        for t in ("composite_instruction", "environment", "scene_state")
        for d in ("to", "from")
        for f in ("xml", "file", "binary")
    },
    **{
        ("tesseract_scene_graph", f"Link.{op}{field}"): "copied-vector-field-mutator"
        for op in ("add", "clear")
        for field in ("Visual", "Collision")
    },
    **{
        ("tesseract_scene_graph", tag): "boost-graph-property-tag"
        for tag in ("vertex_link_t", "edge_joint_t", "graph_root_t", "property_kind")
    },
    # #256 group 1.
    **{
        ("tesseract_command_language", name): "type-erasure-interface"
        for name in (
            "WaypointInterface",
            "CartesianWaypointInterface",
            "JointWaypointInterface",
            "StateWaypointInterface",
            "InstructionInterface",
            "MoveInstructionInterface",
            "WaypointPoly.getWaypoint",
            "CartesianWaypointPoly.getCartesianWaypoint",
            "JointWaypointPoly.getJointWaypoint",
            "StateWaypointPoly.getStateWaypoint",
            "InstructionPoly.getInstruction",
            "MoveInstructionPoly.getMoveInstruction",
        )
    },
    **{
        ("tesseract_time_parameterization", cls): "algorithm-internal-namespace"
        for cls in ("Path", "PathSegment", "PathData", "Trajectory")
    },
    ("tesseract_command_language", "MoveInstruction.setWaypoint"): "reference-setter",
    ("tesseract_command_language", "MoveInstructionPoly.setWaypoint"): "reference-setter",
    # Group 2 (trajopt).
    ("trajopt_ifopt", "Var.getParent"): "parent-back-pointer",
    ("trajopt_ifopt", "Node.getParent"): "parent-back-pointer",
    ("trajopt_ifopt", "Component.print"): "stdout-printer",
    ("trajopt_ifopt", "NodesVariables.addObserver"): "unimplemented-observer",
    ("trajopt_ifopt", "calcNumericalCostGradient"): "ifopt-problem-only",
    ("trajopt_ifopt", "createNodesVariables"): "nodes-variables-factory",
    # tesseract_task_composer (#256 group 4; ledger phaseF-triage-tc.md). Instances of AnyPoly's
    # forwarding ctor templates (any_poly.h:121, :138): Python-only names.
    **{
        ("tesseract_task_composer", f"AnyPoly_wrap_{t}"): "template-instance-name"
        for t in (
            "CompositeInstruction",
            "ProfileDictionary",
            "EnvironmentConst",
            "TaskComposerDataStorage",
        )
    },
    ("tesseract_task_composer", "AnyInterface"): "type-erasure-interface",
    ("tesseract_task_composer", "AnyWrapper"): "type-erasure-interface",
    ("tesseract_task_composer", "AnyPoly.get"): "type-erasure-interface",
    ("tesseract_task_composer", "FutureStatus"): "std-library-enum",
    ("tesseract_task_composer", "TaskComposerFuture.waitUntil"): "clock-deadline",
    ("tesseract_task_composer", "TaskComposerNode.dump"): "dot-dump-hook",
    ("tesseract_task_composer", "TaskComposerNodeFactory"): "plugin-author-factory-interface",
    ("tesseract_task_composer", "TaskComposerExecutorFactory"): "plugin-author-factory-interface",
    # C20: the core plugin factory interfaces, under the same rule.
    ("tesseract_kinematics", "FwdKinFactory"): "plugin-author-factory-interface",
    ("tesseract_kinematics", "InvKinFactory"): "plugin-author-factory-interface",
    ("tesseract_collision", "DiscreteContactManagerFactory"): "plugin-author-factory-interface",
    ("tesseract_collision", "ContinuousContactManagerFactory"): "plugin-author-factory-interface",
    # C12 (accept by explicit entry): the core reference-wrapper flatten.
    (
        "tesseract_collision",
        "ContactResultMap.flattenWrapperResults",
    ): "reference-wrapper-out-param",
    # Group 3 (motion planners). R2: type-erased planner payloads.
    ("tesseract_motion_planners", "PlannerRequest.data"): "type-erased-data",
    ("tesseract_motion_planners", "PlannerResponse.data"): "type-erased-data",
    # R14 (C13 = keep): OMPL's process-global RNG.
    ("tesseract_motion_planners_ompl", "RNG_setSeed"): "third-party-global",
    ("tesseract_motion_planners_ompl", "RNG_getSeed"): "third-party-global",
    # R21: the classic-TrajOpt optimizer in optimizers.hpp, next to the bound parameters.
    **{
        ("tesseract_motion_planners_trajopt", name): "solver-behind-profile"
        for name in (
            "Optimizer",
            "BasicTrustRegionSQP",
            "BasicTrustRegionSQPMultiThreaded",
            "OptResults",
            "BasicTrustRegionSQPResults",
            "OptStatus",
            "toString",
        )
    },
    # R13 by explicit entry, where the type walk stops at an audited aggregate whose
    # payload is not API: `TrajOptTermInfos`/`TrajOptWaypointInfo` hold
    # `vector<shared_ptr<trajopt::TermInfo>>`, and the four `create` hooks return them,
    # so the hooks have no Python form either (#339).
    **{
        ("tesseract_motion_planners_trajopt", name): UNBOUND_DEPENDENCY
        for name in (
            "TrajOptMoveProfile.create",
            "TrajOptDefaultMoveProfile.create",
            "TrajOptCompositeProfile.create",
            "TrajOptDefaultCompositeProfile.create",
            # C3 = 8a: the hook results themselves; their payload is `trajopt::TermInfo`.
            "TrajOptTermInfos",
            "TrajOptWaypointInfo",
        )
    },
    # Task 10: descartes_light's generic class names, bound with the library prefix.
    **{
        ("tesseract_motion_planners_descartes", name): "template-instance-name"
        for name in (
            "DescartesStateD",
            "DescartesStateSampleD",
            "DescartesEdgeEvaluatorD",
            "DescartesWaypointSamplerD",
            "DescartesStateEvaluatorD",
        )
    },
}
# Classes that wrap one bound Eigen vector field; their operators forward to it (R32):
# (module, C++ class).
EIGEN_VALUE_WRAPPERS = frozenset({("tesseract_motion_planners_descartes", "State")})
# A C++ member template bound once per instance, under a name Python can spell
# (`InstructionPoly::as<MoveInstructionPoly>()` → `asMoveInstruction`): (module, C++ dotted
# name) -> the Python dotted names of its instances. The C++ row is accepted only when every
# listed instance is bound; each instance is accepted on the Python side (template-instance-name).
TEMPLATE_INSTANCES: dict[tuple[str, str], tuple[str, ...]] = {
    ("tesseract_command_language", "InstructionPoly.as"): tuple(
        f"InstructionPoly.as{t}Instruction"
        for t in ("Move", "Composite", "Wait", "Timer", "SetAnalog", "SetDigital", "SetTool")
    ),
    # The waypoint and move-instruction Polys' `as<T>()` (#268): to the typed Polys on
    # `WaypointPoly`, to the concrete type on each typed Poly.
    ("tesseract_command_language", "WaypointPoly.as"): tuple(
        f"WaypointPoly.as{t}Waypoint" for t in ("Cartesian", "Joint", "State")
    ),
    **{
        ("tesseract_command_language", f"{t}Poly.as"): (f"{t}Poly.as{t}",)
        for t in ("CartesianWaypoint", "JointWaypoint", "StateWaypoint", "MoveInstruction")
    },
    # tesseract_task_composer (#256 group 4): `template <T> T& AnyPoly::as()`.
    ("tesseract_task_composer", "AnyPoly.as"): (
        "AnyPoly_as_CompositeInstruction",
        "AnyPoly_as_TaskComposerDataStorage",
        "AnyPoly_as_ContactResultMapVector",
    ),
}
# The rule `TEMPLATE_INSTANCES` entries are accepted under.
TEMPLATE_INSTANCE_RULE = "template-instance-name"
# A C++ overload accepted unbound: (module, dotted name, `str(Arity)`) -> ACCEPTED rule. For a
# symbol Python binds, where the other overloads stay checked (T4).
ACCEPTED_CPP_OVERLOADS: dict[tuple[str, str, str], str] = {
    ("tesseract_command_language", "ProfileDictionary.getProfile", "2-3"): "profile-key-template",
    # composite_instruction.h:386 `template <class InputIt> insert(pos, first, last)` (#269).
    (
        "tesseract_command_language",
        "CompositeInstruction.insert",
        "3",
    ): "vector-forwarding-no-python-form",
    # Group 2 (trajopt).
    ("trajopt_ifopt", "ContinuousCollisionConstraint.__init__", "3-7"): "split-var-pair",
    # tesseract_task_composer (#256 group 4). any_poly.h:115 `AnyPoly(const AnyInterface&)`: the
    # arity-1 forwarding ctor templates (:121, :138) are template-instance-name before this table.
    ("tesseract_task_composer", "AnyPoly.__init__", "1"): "type-erasure-interface",
    # taskflow_task_composer_executor.h:71 `(std::string name, const YAML::Node& config)`: no
    # YAML-text sibling, so `yaml-node` falls through to here (T1).
    ("tesseract_task_composer", "TaskflowTaskComposerExecutor.__init__", "2"): "plugin-config-ctor",
}
# A Python-only overload accepted: (module, dotted name, `PyOverload.params`) -> ACCEPTED rule.
# Two overloads of one arity differ by their annotations (G2: `setWarmStart(bool)`).
ACCEPTED_PY_OVERLOADS: dict[tuple[str, str, tuple[str, ...]], str] = {
    # Group 2 (trajopt).
    ("trajopt_sqp", "OSQPEigenSolver.setWarmStart", ("bool",)): UNBOUND_DEPENDENCY,
    (
        "trajopt_ifopt",
        "ContinuousCollisionConstraint.__init__",
        ("ContinuousCollisionEvaluator", "Var", "Var", "bool", "bool", "int", "bool", "str"),
    ): "split-var-pair",
    # tesseract_task_composer (#256 group 4; C23 keeps the rule).
    (
        "tesseract_task_composer",
        "TaskComposerExecutor.run",
        ("TaskComposerNode", "TaskComposerDataStorage", "bool"),
    ): "context-from-data-storage",
}
# A type-erased Poly -> the bound classes it wraps, which C++ converts into it through the
# Poly's converting constructors (R8). A Python overload typed by an alternative stands for
# the Poly-typed C++ parameter.
POLY_ALTERNATIVES: dict[str, tuple[str, ...]] = {
    "WaypointPoly": ("CartesianWaypointPoly", "JointWaypointPoly", "StateWaypointPoly"),
    "InstructionPoly": (
        "MoveInstructionPoly",
        # instruction_poly.h:139 `InstructionPoly(const MoveInstructionInterface&)`.
        "MoveInstruction",
        "CompositeInstruction",
        "WaitInstruction",
        "TimerInstruction",
        "SetAnalogInstruction",
        "SetDigitalInstruction",
        "SetToolInstruction",
    ),
}
# Rules whose overload-level `absent_ok` also accepts a symbol Python does not bind at all,
# when every overload carries it, or that `CppSymbol.absent_ok` names for a field.
# (`yaml-node` and `serialization-default-ctor` do not: they name one overload of a symbol
# whose other overloads must be bound.)
WHOLE_SYMBOL_ABSENT_OK = frozenset({"type-index", UNBOUND_DEPENDENCY})
# std::vector members with no Python form, on a class that forwards the vector interface (R11).
VECTOR_FORWARDING_MEMBERS = frozenset(
    {"data", "emplace", "emplace_back", "swap", "rbegin", "rend", "crbegin", "crend"}
)
# What makes a class a std::vector forwarder: it declares all of these.
VECTOR_FORWARDER_MEMBERS = ("begin", "end", "size", "push_back")


class UnknownAcceptedRuleError(LookupError):
    """A rule table entry names a rule that `ACCEPTED` does not justify."""


def _check_rule_keys() -> None:
    """Every entry of `ACCEPTED_SYMBOLS`, both overload tables and `TEMPLATE_INSTANCES` names an
    `ACCEPTED` rule.

    Raises:
        UnknownAcceptedRuleError: one error naming every offending entry and its rule.
    """
    entries = [
        *ACCEPTED_SYMBOLS.items(),
        *ACCEPTED_CPP_OVERLOADS.items(),
        *ACCEPTED_PY_OVERLOADS.items(),
        *((key, TEMPLATE_INSTANCE_RULE) for key in TEMPLATE_INSTANCES),
    ]
    if bad := [f"{key} -> {rule!r}" for key, rule in entries if rule not in ACCEPTED]:
        raise UnknownAcceptedRuleError(f"rule table entries without an ACCEPTED rule: {bad}")


_check_rule_keys()


# trajopt's enumerator style: `k` + CamelCase (`kLowerBound`); bound as UPPER_SNAKE (R15).
K_ENUMERATOR = re.compile(r"k[A-Z][A-Za-z0-9]*")
# Word boundaries in CamelCase, an acronym run kept whole: `QPSolveFailed` → `QP|Solve|Failed`.
CAMEL_BOUNDARY = re.compile(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])")


def k_enumerator_python_name(leaf: str) -> str | None:
    """The mechanical Python name of a `kCamelCase` enumerator, or None if `leaf` is not one.

    `kLowerBound` → `LOWER_BOUND`, `kQPSolveFailed` → `QP_SOLVE_FAILED`.
    """
    if not K_ENUMERATOR.fullmatch(leaf):
        return None
    return CAMEL_BOUNDARY.sub("_", leaf[1:]).upper()


def _k_alias(name: str) -> str | None:
    """The dotted Python name a `kCamelCase` enumerator `name` is bound under (R15)."""
    owner, _, leaf = name.rpartition(".")
    alias = k_enumerator_python_name(leaf)
    return None if alias is None else f"{owner}.{alias}"
