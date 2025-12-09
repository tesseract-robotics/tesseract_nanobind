# tesseract_python_nanobind Status and Roadmap

## Current Status (December 2024)

### Test Suite Summary

| Metric | Count |
|--------|-------|
| **Passed** | 154 |
| **Skipped** | 24 |
| **Failed** | 0 |
| **Test Phases** | 15/15 |

All test phases pass when run via `./run_tests.sh`.

### Dependencies (tesseract 0.33.x)

```yaml
tesseract: 0.33.0
tesseract_planning: 0.33.1
trajopt: 0.33.0
descartes_light: 0.4.9
opw_kinematics: 0.5.2
boost_plugin_loader: 0.4.3
ruckig: 0.9.2
ifopt: 2.1.3
```

---

## Blocking Issues

### 1. C++ Plugin Factory Global State (Critical)

**Impact**: Prevents running full test suite in single pytest invocation

**Root Cause**: `KinematicsPluginFactory` and `TaskComposerPluginFactory` use C++ singletons/global registries. When multiple factories are created/destroyed in same process, the registry becomes corrupted.

**Symptoms**:
- Tests pass individually, fail when run together
- `Failed to create plugin instance 'OPWInvKinFactory'`
- `Failed to load symbol 'KDLFwdKinChainFactory'`
- Segfaults at plugin creation

**Current Workaround**:
- `run_tests.sh` runs test categories in separate pytest invocations
- `@pytest.mark.forked` on tests using kinematics plugin factory

**Affected Tests**: 22 tests require isolation

**Files**:
- `tesseract_kinematics/core/src/kinematics_plugin_factory.cpp:299-356`
- `tesseract_task_composer/core/src/task_composer_plugin_factory.cpp:381`

**Fix Options**:

| Option | Effort | Impact |
|--------|--------|--------|
| Per-factory plugin state | High | Full fix |
| Reference-counted handles | Medium | Partial |
| Never unload plugins | Low | Memory cost |
| Clear registry API | Medium | User burden |

**Upstream Issue**: Needs fix in tesseract C++ library

---

### 2. PipelineTaskFactory Loading (Critical)

**Impact**: Cannot run TrajOpt/OMPL pipelines in Python

**Symptoms**:
```
Failed to create plugin instance 'PipelineTaskFactory'
Pipeline 'TrajOptPipeline' not found
Pipeline 'OMPLPipeline' not found
```

**Search Paths**:
```
/Users/jelle/Code/CADCAM/tesseract_python_nanobind/ws/install/lib
  - libtesseract_task_composer_factories.dylib
  - libtesseract_task_composer_planning_factories.dylib
  - libtesseract_task_composer_taskflow_factories.dylib
```

**Suspected Cause**: `PipelineTaskFactory` plugin fails to load from `libtesseract_task_composer_planning_factories.dylib`. May be related to:
- RPATH/DYLD_LIBRARY_PATH issues on macOS
- Plugin registration order
- Missing transitive dependencies

**Diagnostic Steps**:
1. Check if library exists and is loadable:
   ```bash
   otool -L ws/install/lib/libtesseract_task_composer_planning_factories.dylib
   nm -g ws/install/lib/libtesseract_task_composer_planning_factories.dylib | grep Pipeline
   ```
2. Run with `TESSERACT_PLUGIN_LOADER_DEBUG=1` (if supported)
3. Test standalone C++ example to isolate Python vs C++ issue

**Skipped Tests**: 4 example tests, all planning pipeline tests

---

### 3. TaskComposer Async Execution Crash (High)

**Impact**: Cannot use `TaskComposerExecutor.run()` for async planning

**Symptoms**: Segfault at `future.wait()` after `executor.run(task, task_data)`

**Root Cause**: `TaskComposerExecutor.run()` returns `std::unique_ptr<TaskComposerFuture>` (abstract). Actual type is `TaskflowTaskComposerFuture`. nanobind needs proper trampoline class for polymorphism.

**Fix Required**:
```cpp
struct PyTaskComposerFuture : tp::TaskComposerFuture {
    NB_TRAMPOLINE(tp::TaskComposerFuture, 7);
    void clear() override { NB_OVERRIDE_PURE(clear); }
    bool valid() const override { NB_OVERRIDE_PURE(valid); }
    bool ready() const override { NB_OVERRIDE_PURE(ready); }
    void wait() const override { NB_OVERRIDE_PURE(wait); }
    std::future_status waitFor(std::chrono::duration<double> t) const override {
        NB_OVERRIDE_PURE(waitFor, t);
    }
    std::future_status waitUntil(std::chrono::time_point<std::chrono::high_resolution_clock> t) const override {
        NB_OVERRIDE_PURE(waitUntil, t);
    }
    TaskComposerContext::Ptr context() override { NB_OVERRIDE_PURE(context); }
};

nb::class_<tp::TaskComposerFuture, PyTaskComposerFuture>(m, "TaskComposerFuture")
    .def("wait", &tp::TaskComposerFuture::wait, nb::call_guard<nb::gil_scoped_release>())
    ...
```

**Reference**: `tesseract_pybind` uses trampoline + explicit `std::unique_ptr<T>` holder

**Skipped Tests**: 1 task composer test, 1 planning integration test

---

### 4. OPW Kinematics Plugin Under pytest (Medium)

**Impact**: Cannot test OPW-based robots (ABB IRB2400, etc.) under pytest

**Symptoms**:
```
Failed to create plugin instance 'OPWInvKinFactory'
Warning: This process is multi-threaded, use of fork() may lead to deadlocks
```

**Root Cause**:
- pytest uses fork for test isolation
- boost::dll plugin loader has thread-local caches
- fork doesn't clone thread-local storage correctly
- Plugin factory state becomes corrupted

**Workaround**: Use IIWA (KDL kinematics) instead of ABB (OPW) in tests

**Fix Options**:
1. `--forked` pytest flag (but increases test time)
2. Subprocess-based test isolation (current approach)
3. Upstream fix in boost_plugin_loader

**Skipped Tests**: 4 tests that require OPW kinematics

---

## nanobind-Specific Issues

### 5. Reference Leaks at Exit (Low)

**Impact**: Cosmetic warnings at interpreter exit

**Symptoms**:
```
nanobind: leaked 48 types!
 - leaked type "WaypointPoly"
 - leaked type "InstructionPoly"
nanobind: leaked 156 functions!
 - leaked function "setManipulatorInfo"
```

**Root Cause**: Type-erased `*Poly` wrappers store function pointers in global registries creating reference cycles.

**Fix Required**: Implement `tp_traverse` and `tp_clear` for cyclic GC:
```cpp
nb::class_<tp::WaypointPoly>(m, "WaypointPoly",
    nb::type_slots({
        {Py_tp_traverse, (void*)waypointpoly_traverse},
        {Py_tp_clear, (void*)waypointpoly_clear}
    }))
```

**Reference**: https://nanobind.readthedocs.io/en/latest/refleaks.html

---

### 6. InstructionsTrajectory API Not Bound (Low)

**Impact**: Cannot use old-style time parameterization API

**Skipped Tests**: 3 time parameterization tests

**Note**: New `ProfileDictionary`-based API is preferred; old API may be deprecated

---

## Test Categories and Status

### Fully Passing (No Skips)

| Module | Tests | Notes |
|--------|-------|-------|
| tesseract_common | 43 | Core types, transforms |
| tesseract_collision | 2 | FCL/Bullet collision |
| tesseract_geometry | 34 | Shapes, meshes |
| tesseract_command_language | 2 | Instructions, waypoints |
| tesseract_environment | 4 | Environment, commands |
| tesseract_scene_graph | 4 | Scene graph ops |
| planning API | 35 | High-level Robot API |

### Passing with Skips

| Module | Passed | Skipped | Skip Reason |
|--------|--------|---------|-------------|
| tesseract_kinematics | 4 | 1 | OPW plugin |
| tesseract_motion_planners | 12 | 5 | OPW + collision enum |
| tesseract_task_composer | 1 | 1 | Async crash |
| tesseract_time_parameterization | 0 | 10 | API not bound |
| test_examples | 14 | 4 | Pipeline loading |

---

## Development Priorities

### Priority 1: Fix PipelineTaskFactory Loading

This is the most impactful fix as it enables full motion planning.

**Investigation Steps**:
1. Verify library symbols: `nm -g libtesseract_task_composer_planning_factories.dylib`
2. Check transitive deps: `otool -L libtesseract_task_composer_planning_factories.dylib`
3. Test C++ standalone (isolate Python binding issues)
4. Add debug logging to plugin loader
5. Compare with working tesseract_pybind setup

**Expected Outcome**: 4 example tests enabled, planning pipelines work

---

### Priority 2: Fix TaskComposer Async Execution

**Steps**:
1. Add trampoline class for `TaskComposerFuture`
2. Bind `TaskflowTaskComposerFuture` with proper inheritance
3. Add `gil_scoped_release` to blocking calls
4. Test with simple sync execution first

**Files to Modify**:
- `tesseract_nanobind/src/tesseract_task_composer/tesseract_task_composer_bindings.cpp`

**Expected Outcome**: 2 tests enabled, async planning works

---

### Priority 3: Bind InstructionsTrajectory API

**Steps**:
1. Add `InstructionsTrajectory` class binding
2. Add trajectory conversion helpers
3. Update time parameterization bindings

**Expected Outcome**: 3 time parameterization tests enabled

---

### Priority 4: Fix Reference Leaks

**Steps**:
1. Identify all Poly types with cycles
2. Implement `tp_traverse`/`tp_clear` for each
3. Test with `gc.collect()` + `gc.garbage` inspection

**Expected Outcome**: Clean interpreter exit, no leak warnings

---

## File Structure Reference

```
tesseract_python_nanobind/
├── tesseract_nanobind/
│   ├── src/                          # C++ nanobind bindings
│   │   ├── tesseract_common/
│   │   ├── tesseract_collision/
│   │   ├── tesseract_geometry/
│   │   ├── tesseract_kinematics/
│   │   ├── tesseract_environment/
│   │   ├── tesseract_scene_graph/
│   │   ├── tesseract_command_language/
│   │   ├── tesseract_motion_planners/
│   │   ├── tesseract_task_composer/   # Priority 2
│   │   └── tesseract_time_parameterization/
│   └── tests/                         # Python tests
├── ws/
│   ├── src/                           # C++ dependencies (rosinstall)
│   └── install/                       # Built C++ libraries
├── examples/                          # Python examples
├── docs/
│   ├── KINEMATICS_PLUGIN_FACTORY_ISSUE.md
│   ├── NANOBIND_ISSUES.md
│   └── STATUS_AND_ROADMAP.md          # This file
├── run_tests.sh                       # Isolated test runner
└── dependencies.rosinstall            # C++ dependency versions
```

---

## Testing Commands

### Run Full Suite (Recommended)
```bash
./run_tests.sh
```

### Run Single Module
```bash
TESSERACT_HEADLESS=1 pytest tesseract_nanobind/tests/tesseract_common/ -v
```

### Run with Isolation
```bash
pytest tesseract_nanobind/tests/tesseract_kinematics/test_kdl_kinematics.py -v --forked
```

### Debug Plugin Loading
```bash
DYLD_LIBRARY_PATH=$PWD/ws/install/lib:$CONDA_PREFIX/lib python -c "
from tesseract_robotics import tesseract_task_composer
# Add debug output here
"
```

---

## Upstream Contributions Needed

1. **tesseract** (ros-industrial/tesseract)
   - Fix `KinematicsPluginFactory` global state issue
   - Expose registry clear/reset API

2. **tesseract_planning** (ros-industrial/tesseract_planning)
   - Fix `TaskComposerPluginFactory` global state issue
   - Investigate `PipelineTaskFactory` loading on macOS

3. **boost_plugin_loader** (tesseract-robotics/boost_plugin_loader)
   - Thread-safe initialization for fork compatibility

---

## Changelog

### 2024-12-10
- All 15 test phases passing (0 failures)
- 154 tests pass, 24 skipped
- Added `@pytest.mark.forked` to kinematics/planning tests
- Skipped in-process example tests (use subprocess-based instead)
- Documented all skip reasons with clear messages

### Previous
- Ported bindings from pybind11 to nanobind
- Fixed `FilesystemPath` type handling
- Added Pythonic planning API (`Robot`, `MotionProgram`, etc.)
