# KinematicsPluginFactory Global State Issue

## Summary

Creating multiple `KinematicsPluginFactory` instances in the same process causes segmentation faults when the factories load different plugin types (e.g., KDL vs OPW). This issue affects Python bindings but originates in the C++ tesseract library.

## Reproduction

```python
# Test 1: Create KDL-based kinematics (IIWA)
factory1 = tesseract_kinematics.KinematicsPluginFactory(iiwa_plugins_yaml, locator1)
inv_kin1 = factory1.createInvKin("manipulator", "KDLInvKinChainLMA", scene_graph1, state1)
# Works fine

# Test 2: Create OPW-based kinematics (ABB)
factory2 = tesseract_kinematics.KinematicsPluginFactory(abb_plugins_yaml, locator2)
inv_kin2 = factory2.createInvKin("manipulator", "OPWInvKin", scene_graph2, state2)
# SEGFAULT here
```

## Error Messages

```
Warning: Failed to load symbol 'KDLFwdKinChainFactory'
         at line 299 in kinematics_plugin_factory.cpp
```

or

```
Warning: Failed to create plugin instance 'OPWInvKinFactory' of type 'tesseract_kinematics::InvKinFactory'
Search Paths (including system folders)
    - /path/to/install/lib
Search Libraries:
    - libtesseract_kinematics_core_factories.dylib
    - libtesseract_kinematics_kdl_factories.dylib
    - libtesseract_kinematics_opw_factory.dylib
    ...
         at line 356 in kinematics_plugin_factory.cpp
```

## Root Cause Analysis

The `KinematicsPluginFactory` uses the `tesseract_common` plugin loading infrastructure which appears to use global state:

1. First factory loads `tesseract_kinematics_kdl_factories` library
2. Plugin symbols are registered in a global registry
3. First factory is destroyed (Python GC, test teardown)
4. Second factory tries to load `tesseract_kinematics_opw_factories`
5. Global registry is in inconsistent state
6. Symbol lookup fails or accesses freed memory → segfault

The issue is in `tesseract_kinematics/core/src/kinematics_plugin_factory.cpp` around line 299-356.

## Evidence

- Tests pass when run **individually** in separate processes
- Tests fail when run **together** in the same Python process
- Using pytest `--forked` option causes `dlopen` handle issues (plugin libraries not found in forked child)
- The error occurs at the boundary between different plugin library loads

## Affected Code

- `tesseract_kinematics/core/src/kinematics_plugin_factory.cpp`
- `tesseract_common` plugin loading infrastructure
- Any code that creates multiple `KinematicsPluginFactory` instances with different plugin configs

## Potential Solutions

### Option 1: Per-Factory Plugin State (Recommended)

Make the plugin registry per-factory instead of global. Each factory maintains its own map of loaded plugins and dlopen handles.

```cpp
class KinematicsPluginFactory {
private:
    std::map<std::string, PluginHandle> loaded_plugins_;  // Per-instance
    // Instead of global:
    // static std::map<std::string, PluginHandle> loaded_plugins_;
};
```

### Option 2: Reference-Counted Plugin Handles

Keep track of how many factories reference each loaded plugin library. Only unload when reference count drops to zero.

### Option 3: Never Unload Plugin Libraries

Once loaded, keep plugin libraries loaded for process lifetime. Simplest fix but uses more memory.

### Option 4: Clear Plugin Registry API

Expose an API to explicitly clear/reset the plugin registry:

```cpp
// New API
void KinematicsPluginFactory::clearGlobalRegistry();
```

Python users would call this between tests or when switching between different robot configurations.

## Current Workaround

Run tests in separate process invocations:

```bash
# Instead of:
pytest tests/

# Use:
pytest tests/test_kdl.py
pytest tests/test_opw.py  # Separate process
```

Or use pytest's `subprocess` isolation for affected tests.

## Impact

This issue affects:
- Python bindings (segfaults)
- C++ applications creating multiple factories
- Unit testing any code using KinematicsPluginFactory
- Applications switching between robot configurations at runtime

## Related Files

- `tesseract_kinematics/core/src/kinematics_plugin_factory.cpp`
- `tesseract_common/include/tesseract_common/plugin_loader.h`
- `tesseract_common/src/plugin_loader.cpp`

## Test Case for C++

```cpp
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

TEST(KinematicsPluginFactory, MultipleFactories) {
    // Create first factory with KDL plugins
    auto factory1 = std::make_shared<KinematicsPluginFactory>(iiwa_config, locator);
    auto kin1 = factory1->createInvKin(...);
    ASSERT_NE(kin1, nullptr);

    // Destroy first factory
    factory1.reset();

    // Create second factory with OPW plugins
    auto factory2 = std::make_shared<KinematicsPluginFactory>(abb_config, locator);
    auto kin2 = factory2->createInvKin(...);  // Should not crash
    ASSERT_NE(kin2, nullptr);
}
```
