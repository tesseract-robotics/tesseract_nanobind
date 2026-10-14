"""Type keys for the binding API audit: a C++ parameter type and the stub annotation nanobind
writes for it normalise to the same key, so `audit_bindings` can match overloads by type (#247).

A key is a nested tuple: `("str",)`, `("seq", K)`, `("map", K, V)`, `("tuple", K, ...)`,
`("union", K, ...)`, `("ndarray", dtype)`, `("class", "Outer.Inner")`. `None` is a type the
mapping does not know, at any depth; comparing it neither matches nor mismatches.
"""

from __future__ import annotations

import ast
from collections.abc import Iterable, Mapping
from types import MappingProxyType

import clang.cindex as ci

TypeKey = tuple | None
# Class name -> the names of all its (transitive) base classes, from the TU.
Ancestors = Mapping[str, frozenset[str]]
NO_ANCESTORS: Ancestors = MappingProxyType({})

RECORD_KINDS = frozenset(
    {ci.CursorKind.CLASS_DECL, ci.CursorKind.STRUCT_DECL, ci.CursorKind.CLASS_TEMPLATE}
)

INT_KINDS = frozenset(
    {
        ci.TypeKind.SHORT,
        ci.TypeKind.USHORT,
        ci.TypeKind.INT,
        ci.TypeKind.UINT,
        ci.TypeKind.LONG,
        ci.TypeKind.ULONG,
        ci.TypeKind.LONGLONG,
        ci.TypeKind.ULONGLONG,
        ci.TypeKind.UCHAR,
        ci.TypeKind.SCHAR,
    }
)
FLOAT_KINDS = frozenset({ci.TypeKind.FLOAT, ci.TypeKind.DOUBLE, ci.TypeKind.LONGDOUBLE})
CHAR_KINDS = frozenset({ci.TypeKind.CHAR_S, ci.TypeKind.CHAR_U})
# numpy dtype of an Eigen matrix scalar, as `NDArray[numpy.<dtype>]` spells it.
EIGEN_DTYPES = {
    ci.TypeKind.DOUBLE: "float64",
    ci.TypeKind.FLOAT: "float32",
    ci.TypeKind.INT: "int32",
    ci.TypeKind.LONG: "int64",
    ci.TypeKind.LONGLONG: "int64",
    ci.TypeKind.BOOL: "bool",
}
# Eigen geometry templates bound as classes in tesseract_common, by canonical spelling.
EIGEN_CLASSES = {
    "Eigen::Transform<double, 3, 1>": "Isometry3d",
    "Eigen::Quaternion<double>": "Quaterniond",
    "Eigen::AngleAxis<double>": "AngleAxisd",
    "Eigen::Translation<double, 3>": "Translation3d",
    "Eigen::Hyperplane<double, 3>": "Hyperplane3d",
    "Eigen::ParametrizedLine<double, 3>": "ParametrizedLine3d",
}
# Records a binding converts to a Python builtin, by canonical spelling: Geometry.getUUID /
# setUUID and JointTrajectory.uuid use the canonical string (boost::uuids::to_string).
CONVERTED_RECORDS = {"boost::uuids::uuid": ("str",)}
# Eigen wrappers a binding passes as the matrix they view.
EIGEN_VIEWS = frozenset({"Ref", "Map"})
STD_SEQUENCES = frozenset({"vector", "list", "deque"})
STD_MAPS = frozenset({"map", "unordered_map"})
STD_POINTERS = frozenset({"shared_ptr", "unique_ptr"})
TYPE_PARAMETER_PREFIX = "type-parameter-"  # libclang's spelling of a template parameter


def record_path(decl: ci.Cursor) -> str:
    """Dotted name of a record or enum within its namespace (`Outer.Inner`)."""
    parts = [decl.spelling]
    parent = decl.semantic_parent
    while parent is not None and parent.kind in RECORD_KINDS:
        parts.append(parent.spelling)
        parent = parent.semantic_parent
    return ".".join(reversed(parts))


def _top_namespace(decl: ci.Cursor) -> str | None:
    """The outermost namespace of a declaration (`std` for `std::__1::vector`)."""
    top = None
    parent = decl.semantic_parent
    while parent is not None and parent.kind != ci.CursorKind.TRANSLATION_UNIT:
        if parent.kind == ci.CursorKind.NAMESPACE:
            top = parent.spelling
        parent = parent.semantic_parent
    return top


def _template_args(t: ci.Type) -> list[ci.Type]:
    return [t.get_template_argument_type(i) for i in range(t.get_num_template_arguments())]


def _union(keys: Iterable[TypeKey]) -> TypeKey:
    members = sorted(set(keys), key=repr)
    return members[0] if len(members) == 1 else ("union", *members)


def _eigen_key(name: str, t: ci.Type) -> TypeKey:
    if name in EIGEN_VIEWS:
        args = _template_args(t)
        return cpp_key(args[0]) if args else None
    if name == "Matrix":
        args = _template_args(t)
        dtype = EIGEN_DTYPES.get(args[0].get_canonical().kind) if args else None
        return ("ndarray", dtype)
    cls = EIGEN_CLASSES.get(t.spelling.removeprefix("const "))
    return ("class", cls) if cls else None


def _std_key(name: str, t: ci.Type) -> TypeKey:
    args = _template_args(t)
    if name in ("basic_string", "basic_string_view") or (name == "path" and not args):
        return ("str",) if name != "path" else ("path",)
    if name in STD_SEQUENCES and args:
        return ("seq", cpp_key(args[0]))
    if name in STD_MAPS and len(args) >= 2:
        return ("map", cpp_key(args[0]), cpp_key(args[1]))
    if name in ("pair", "tuple"):
        return ("tuple", *(cpp_key(a) for a in args))
    if name in STD_POINTERS and args:
        return cpp_key(args[0])
    if name == "optional" and args:
        return cpp_key(args[0])
    if name == "variant" and args:
        members = [cpp_key(a) for a in args]
        return None if None in members else _union(members)
    if name == "function":
        return ("callable",)
    return None


def cpp_key(t: ci.Type) -> TypeKey:
    """The key of a C++ type: canonical, with references, cv and smart pointers stripped."""
    t = t.get_canonical()
    if t.kind in (ci.TypeKind.LVALUEREFERENCE, ci.TypeKind.RVALUEREFERENCE):
        t = t.get_pointee().get_canonical()
    if t.spelling.removeprefix("const ").startswith(TYPE_PARAMETER_PREFIX):
        return None
    if t.kind == ci.TypeKind.BOOL:
        return ("bool",)
    if t.kind in INT_KINDS:
        return ("int",)
    if t.kind in FLOAT_KINDS:
        return ("float",)
    if t.kind == ci.TypeKind.POINTER:
        return ("str",) if t.get_pointee().kind in CHAR_KINDS else None
    converted = CONVERTED_RECORDS.get(t.spelling.removeprefix("const "))
    if converted is not None:
        return converted
    decl = t.get_declaration()
    if decl.kind == ci.CursorKind.ENUM_DECL:
        return ("class", record_path(decl))
    if decl.kind not in RECORD_KINDS and decl.kind != ci.CursorKind.CLASS_TEMPLATE:
        return None
    namespace = _top_namespace(decl)
    if namespace == "std":
        return _std_key(decl.spelling, t)
    if namespace == "Eigen":
        return _eigen_key(decl.spelling, t.get_canonical())
    return ("class", record_path(decl))


def _typedef_names(t: ci.Type) -> list[str]:
    """Every typedef name on the way from the written type to its canonical type."""
    names: list[str] = []
    if t.kind in (ci.TypeKind.LVALUEREFERENCE, ci.TypeKind.RVALUEREFERENCE):
        t = t.get_pointee()
    while True:
        if t.kind == ci.TypeKind.ELABORATED:
            t = t.get_named_type()
        elif t.kind == ci.TypeKind.TYPEDEF:
            decl = t.get_declaration()
            names.append(decl.spelling)
            t = decl.underlying_typedef_type
        else:
            return names


def cpp_keys(t: ci.Type) -> frozenset[TypeKey] | None:
    """Keys a Python annotation may take for a C++ parameter: the structural key, and each
    typedef name as a class (a container bound opaquely, `nb::bind_vector<Items>(m, "Items")`).
    None when the structural key is unknown."""
    key = cpp_key(t)
    if key is None:
        return None
    return frozenset({key, *(("class", n) for n in _typedef_names(t))})


# Python spellings of the keys, by the last component of the annotation's name.
PY_SCALARS = {"str": ("str",), "int": ("int",), "float": ("float",), "bool": ("bool",)}
PY_SCALARS |= {"bytes": ("bytes",), "PathLike": ("path",)}
PY_SEQUENCES = frozenset({"Sequence", "list", "MutableSequence"})
PY_MAPS = frozenset({"Mapping", "dict", "MutableMapping"})
PY_UNKNOWN = frozenset({"object", "Any"})


def _dotted(node: ast.expr) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        owner = _dotted(node.value)
        return None if owner is None else f"{owner}.{node.attr}"
    return None


def _class_name(dotted: str) -> str:
    """`pkg.mod._mod.Outer.Inner` → `Outer.Inner`: the part after the extension module."""
    parts = dotted.split(".")
    private = [i for i, p in enumerate(parts) if p.startswith("_")]
    return ".".join(parts[private[-1] + 1 :] if private else parts)


def _union_members(node: ast.expr) -> list[ast.expr]:
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
        return [*_union_members(node.left), *_union_members(node.right)]
    return [node]


def _subscript_args(node: ast.Subscript) -> list[ast.expr]:
    s = node.slice
    return list(s.elts) if isinstance(s, ast.Tuple) else [s]


def py_key(node: ast.expr | None) -> TypeKey:
    """The key of a stub annotation, as nanobind spells C++ types."""
    if node is None or isinstance(node, ast.Constant):
        return None  # missing, `None`, or a quoted (unbound) C++ name
    members = [m for m in _union_members(node) if not _is_none(m)]
    if len(members) > 1:
        if any(_dotted(m) == "os.PathLike" for m in members):
            return ("path",)
        keys = [py_key(m) for m in members]
        return None if None in keys else _union(keys)
    node = members[0]
    if isinstance(node, ast.Subscript):
        name = (_dotted(node.value) or "").rpartition(".")[2]
        args = _subscript_args(node)
        if name == "Annotated":
            return py_key(args[0])
        if name == "Optional":
            return py_key(args[0])
        if name == "NDArray":
            dtype = _dotted(args[0])
            return ("ndarray", dtype.rpartition(".")[2] if dtype else None)
        if name in PY_SEQUENCES:
            return ("seq", py_key(args[0]))
        if name in PY_MAPS and len(args) == 2:
            return ("map", py_key(args[0]), py_key(args[1]))
        if name == "tuple":
            return ("tuple", *(py_key(a) for a in args))
        if name == "Callable":
            return ("callable",)
        return None
    dotted = _dotted(node)
    if dotted is None:
        return None
    leaf = dotted.rpartition(".")[2]
    if leaf in PY_UNKNOWN:
        return None
    if leaf in PY_SCALARS and dotted in (leaf, f"os.{leaf}"):
        return PY_SCALARS[leaf]
    if leaf == "NDArray":
        return ("ndarray", None)
    return ("class", _class_name(dotted))


def _is_none(node: ast.expr) -> bool:
    return isinstance(node, ast.Constant) and node.value is None


def class_ancestors(tu: ci.TranslationUnit) -> dict[str, frozenset[str]]:
    """Every class defined in the TU -> the names of its transitive base classes."""
    direct: dict[str, set[str]] = {}
    for c in tu.cursor.walk_preorder():
        if c.kind in RECORD_KINDS and c.is_definition():
            bases = direct.setdefault(record_path(c), set())
            for b in c.get_children():
                if b.kind == ci.CursorKind.CXX_BASE_SPECIFIER and b.referenced is not None:
                    bases.add(record_path(b.referenced))

    def closure(name: str) -> frozenset[str]:
        # Iterative with a visited set: names are unqualified, so two classes of one name in
        # different namespaces (std internals) can form a cycle.
        found: set[str] = set()
        todo = list(direct.get(name, ()))
        while todo:
            base = todo.pop()
            if base not in found:
                found.add(base)
                todo.extend(direct.get(base, ()))
        return frozenset(found)

    return {name: closure(name) for name in direct}


def _compare(cpp: TypeKey, py: TypeKey, ancestors: Ancestors) -> bool | None:
    if cpp is None or py is None:
        return None
    if cpp[0] != py[0] or len(cpp) != len(py):
        return False
    if cpp[0] == "class":
        # A derived class binds a base-class parameter: Python passes the subclass.
        return py[1] == cpp[1] or cpp[1] in ancestors.get(py[1], frozenset())
    results = [
        x == y if isinstance(x, str) and isinstance(y, str) else _compare(x, y, ancestors)
        for x, y in zip(cpp[1:], py[1:])
    ]
    if False in results:
        return False
    return None if None in results else True


def verdict(
    cpp: frozenset[TypeKey] | None, py: TypeKey, ancestors: Ancestors = NO_ANCESTORS
) -> bool | None:
    """True: the annotation spells one of the C++ keys (or a class derived from it); False:
    it spells none of them; None: an unknown type on either side decides nothing."""
    if cpp is None:
        return None
    results = {_compare(k, py, ancestors) for k in cpp}
    if True in results:
        return True
    return None if None in results else False


def signature_verdict(
    cpp: tuple[frozenset[TypeKey] | None, ...],
    py: tuple[TypeKey, ...],
    ancestors: Ancestors = NO_ANCESTORS,
) -> bool | None:
    """Position by position over the shorter signature: False if any position mismatches,
    None if none mismatches but one is unknown, True if all match."""
    results = {verdict(c, p, ancestors) for c, p in zip(cpp, py)}
    if False in results:
        return False
    return None if None in results else True


def is_known(keys: frozenset[TypeKey] | None) -> bool:
    """No unknown at any depth: two overloads with equal known keys are one Python form."""
    return keys is not None and all(_known(k) for k in keys)


def _known(key: TypeKey) -> bool:
    if key is None:
        return False
    return all(_known(x) for x in key[1:] if not isinstance(x, str))
