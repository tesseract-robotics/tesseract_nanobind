# High-Severity Review Fixes

Addresses H1-H4 from REVIEW.md.

## H1: Remove PlannerConfig unused fields

`PlannerConfig` advertises velocity_scaling, acceleration_scaling, collision_safety_margin, smoothing, time_parameterization — none are read anywhere. Silent misconfiguration.

**Fix:** Remove `PlannerConfig` entirely. Replace `config` parameter with `pipeline: str | None` on `plan_freespace`, `plan_ompl`, `plan_cartesian`.

**Files:** `planner.py`

### Before
```python
config = config or PlannerConfig()
composer.plan(robot, program, pipeline=config.pipeline, profiles=profiles)
```

### After
```python
composer.plan(robot, program, pipeline=pipeline or "TrajOptPipeline", profiles=profiles)
```

## H2: Fix IFOPT pipeline profile selection

`"TrajOpt" in pipeline` matches `"TrajOptIfoptPipeline"`, so IFOPT pipelines get wrong (TrajOpt) profiles.

**Fix:** Check IFOPT-specific pipelines before generic TrajOpt/Freespace checks. Add `create_freespace_ifopt_pipeline_profiles` helper.

**Files:** `composer.py`, `profiles.py`

### Profile matching order (after fix)
```python
if "TrajOptIfopt" in pipeline:
    profiles = create_trajopt_ifopt_default_profiles()
elif "FreespaceIfopt" in pipeline:
    profiles = create_freespace_ifopt_pipeline_profiles()
elif "Freespace" in pipeline:
    profiles = create_freespace_pipeline_profiles()
elif "Cartesian" in pipeline:
    profiles = create_cartesian_pipeline_profiles()
elif "TrajOpt" in pipeline:
    profiles = create_trajopt_default_profiles()
elif "OMPL" in pipeline:
    profiles = create_ompl_default_profiles()
elif "Descartes" in pipeline:
    profiles = create_descartes_default_profiles()
```

### New helper: `create_freespace_ifopt_pipeline_profiles`
Combines OMPL profiles + TrajOptIfopt profiles (same pattern as `create_freespace_pipeline_profiles` but swaps TrajOpt for TrajOptIfopt).

## H3: Move config cache out of package dir

`_resolve_config_paths()` writes to `Path(__file__).parent / ".cache"` inside the installed package. Fails on read-only installs.

**Fix:** Write to `tempfile.gettempdir() / "tesseract_robotics"` instead. Same hash-based naming for invalidation.

**Files:** `__init__.py`

### Before
```python
cache_dir = Path(__file__).parent / ".cache"
```

### After
```python
cache_dir = Path(tempfile.gettempdir()) / "tesseract_robotics"
```

## H4: Fix broken error diagnostics

Error extraction uses `task_infos.items()` and `info.message` — but bindings expose `TaskComposerNodeInfoContainer` with `getAbortingNodeInfo()` returning dicts with `status_message`.

**Fix:** Use the actual binding API:
1. Try `getAbortingNodeInfo()` first (most useful — the failing node)
2. Fall back to `getAllInfos()` scanning for non-empty `status_message`

**Files:** `composer.py`

### Before
```python
for name, info in task_infos.items():
    if hasattr(info, "message") and info.message:
        msg = f"{name}: {info.message}"
```

### After
```python
aborting = task_infos.getAbortingNodeInfo()
if aborting and aborting.get("status_message"):
    msg = f"{aborting['name']}: {aborting['status_message']}"
else:
    for info in task_infos.getAllInfos():
        if info.get("status_message"):
            msg = f"{info['name']}: {info['status_message']}"
            break
```

## Implementation order

1. H1 (simplest, removes code)
2. H4 (isolated fix in error path)
3. H3 (isolated fix in __init__)
4. H2 (adds new code, depends on profiles understanding)
