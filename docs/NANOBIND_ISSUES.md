# nanobind Binding Issues for tesseract_python

Report on issues encountered porting tesseract_pybind to nanobind.

## 1. Reference Leaks in Poly Types

**Severity**: High - Causes warnings at interpreter exit, potential memory corruption

**Symptoms**:
```
nanobind: leaked 48 types!
 - leaked type "WaypointPoly"
 - leaked type "InstructionPoly"
 ...
nanobind: leaked 156 functions!
 - leaked function "setManipulatorInfo"
 - leaked function "getWaypoint"
 ...
```

**Root Cause**: Type-erased `*Poly` wrappers store function pointers in global registries. These functions implicitly depend on module namespace, creating reference cycles that Python's GC cannot break.

**Affected Types**:
- `WaypointPoly`, `CartesianWaypointPoly`, `JointWaypointPoly`, `StateWaypointPoly`
- `InstructionPoly`, `MoveInstructionPoly`
- `CompositeInstruction` (iterates over InstructionPoly)

**Fix Required**: Implement `tp_traverse` and `tp_clear` for cyclic GC support:

```cpp
// Example for WaypointPoly
int waypointpoly_traverse(PyObject* self, visitproc visit, void* arg) {
    // Visit all Python object references held by this type
    return 0;
}

int waypointpoly_clear(PyObject* self) {
    // Clear member variables that form cycles
    return 0;
}

nb::class_<tp::WaypointPoly>(m, "WaypointPoly",
    nb::type_slots({
        {Py_tp_traverse, (void*)waypointpoly_traverse},
        {Py_tp_clear, (void*)waypointpoly_clear}
    }))
    ...
```

**Reference**: https://nanobind.readthedocs.io/en/latest/refleaks.html

---

## 2. TaskComposer Async Execution Crash

**Severity**: Critical - Segfaults during planning

**Symptoms**: Crash at `future.wait()` after `executor.run(task, task_data)`

**Context**:
- `TaskComposerExecutor.run()` returns `std::unique_ptr<TaskComposerFuture>` (abstract base)
- Actual returned type is `TaskflowTaskComposerFuture` (derived class)
- nanobind needs both classes bound with proper inheritance for polymorphism

**Current Workaround**: Tests marked `@pytest.mark.skip`

**Attempted Fixes**:
1. Added `TaskflowTaskComposerFuture` binding with inheritance
2. Added `gil_scoped_release` during `wait()`
3. Both base and derived classes re-expose methods

**pybind11 Solution** (from tesseract_pybind):
- Uses trampoline class `PyTaskComposerFuture` for abstract base
- Explicit `std::unique_ptr<T>` holder type in class binding
- Both base and derived need the unique_ptr holder

**nanobind Equivalent Required**:
```cpp
struct PyTaskComposerFuture : tp::TaskComposerFuture {
    NB_TRAMPOLINE(tp::TaskComposerFuture, 7);
    void clear() override { NB_OVERRIDE_PURE(clear); }
    bool valid() const override { NB_OVERRIDE_PURE(valid); }
    bool ready() const override { NB_OVERRIDE_PURE(ready); }
    void wait() const override { NB_OVERRIDE_PURE(wait); }
    ...
};

nb::class_<tp::TaskComposerFuture, PyTaskComposerFuture>(m, "TaskComposerFuture")
    ...
```

---

## 3. OPW Kinematics Plugin Loading Under pytest

**Severity**: Medium - Tests fail when using OPW-based robots

**Symptoms**:
```
Failed to create plugin instance 'OPWInvKinFactory'
Warning: This process is multi-threaded, use of fork() may lead to deadlocks
```

**Context**:
- OPW plugin loads fine in standalone Python scripts
- Fails when run under pytest due to fork/threading issues
- `boost::plugin_loader` uses thread-local caches that corrupt during fork

**Workaround Applied**: Use IIWA robot (KDL kinematics) instead of ABB IRB2400 (OPW) in tests

**Root Cause**: pytest's forking combined with boost::dll plugin loader's global state causes corruption. The plugin factory maintains thread-local caches that don't survive fork correctly.

**Long-term Fix Options**:
1. Run tests with `--forked` to isolate each test
2. Use threading-safe initialization in plugin loaders
3. Disable fork in pytest configuration

---

## 4. FilesystemPath Type Mismatch

**Severity**: Low - Fixed

**Symptoms**:
```
TypeError: __init__(): incompatible function arguments
```

**Root Cause**: SWIG compatibility wrapper defined FilesystemPath as Python `str` subclass. nanobind's strict type checking rejected this.

**Fix Applied**: Removed str subclass wrapper, use C++ binding directly:
```python
# Before (SWIG compatibility)
class FilesystemPath(str):
    def __new__(cls, s): return str.__new__(cls, str(s))

# After (nanobind)
from tesseract_robotics.tesseract_common._tesseract_common import FilesystemPath
```

---

## 5. Cross-Module Type Inheritance

**Severity**: Low - Workaround applied

**Issue**: Types bound in different modules cannot directly inherit. For example, `TaskComposerPluginFactory` constructor takes `ResourceLocator` from tesseract_common module.

**Workaround**: Manual type checking with `nb::handle`:
```cpp
m.def("__init__", [](Factory* self, const std::string& config, nb::handle locator_handle) {
    auto common_module = nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    auto grl_type = common_module.attr("GeneralResourceLocator");
    if (!nb::isinstance(locator_handle, grl_type)) {
        throw nb::type_error("locator must be a GeneralResourceLocator");
    }
    auto* locator = nb::cast<tc::GeneralResourceLocator*>(locator_handle);
    new (self) Factory(config, *locator);
});
```

---

## Test Status Summary

| Module | Tests | Status |
|--------|-------|--------|
| tesseract_common | 20+ | PASS |
| tesseract_collision | 10+ | PASS |
| tesseract_geometry | 10+ | PASS |
| tesseract_environment | 15+ | PASS |
| tesseract_kinematics | 10+ | PASS (KDL only) |
| tesseract_command_language | 20+ | PASS |
| tesseract_task_composer | 2 | SKIP (async crash) |
| planning API | 35 | PASS (1 skip) |

**Known Limitations**:
- OPW kinematics tests require `--forked` or manual execution
- TaskComposer pipeline tests crash (need trampoline fix)
- Leaked types/functions warnings at exit (cosmetic, not functional)
