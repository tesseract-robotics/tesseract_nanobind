# Threading and the GIL

The bindings release the GIL selectively: only around calls where holding it would serialise
real work or cause a deadlock. A getter on a `Pose` does not qualify; `MotionPlanner.solve` does.
Every release is listed, with its reason, in
[`scripts/nogil_entry_points.yaml`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/scripts/nogil_entry_points.yaml),
and CI checks the list against the bindings (#124).

## The rule

Release the GIL around a call only when all three hold:

1. **It is expensive or blocking.** Roughly above 50 µs, or it can wait: planning, IK, collision
   checks over a trajectory, time parameterization, URDF and mesh loading, waiting on a task
   composer future. A call that blocks on a C++ lock which a thread waiting for the GIL may hold
   counts too (`Environment.getState`, `clone`, `==`).
2. **It runs pure C++.** No Python object is created, copied or dropped in the released region.
   Python callbacks stay safe: nanobind trampolines (`NB_OVERRIDE`) and the `std::function`
   caster re-acquire the GIL themselves, as do the bindings' own callback wrappers.
3. **It does not mutate state Python code typically shares across threads.** Releasing the GIL
   lets other Python threads run; two threads mutating one C++ object is a data race whether or
   not the GIL was held when they started.

Value types, getters, setters and constructors never qualify.

## How to release it

nanobind has no global switch like SWIG's `-threads`. Per binding there are two forms:

```cpp
// The whole native call: arguments are converted before the release,
// the result is converted after the GIL is re-acquired.
.def("solve", &tp::MotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())

// Part of a lambda: validate (and raise Python errors) with the GIL held, then release.
.def("checkTrajectoryState", [](Manager& manager, const tc::TransformMap& state, ...) {
    require_active_link_transforms(manager, state, "checkTrajectoryState", "state");
    nb::gil_scoped_release release;
    ...
})
```

Use the in-body form when the lambda has to raise or touch Python objects before or after the
native call. Under `call_guard`, by-value arguments are destroyed without the GIL, so a binding
that takes `nb::object`, `nb::handle`, `nb::callable` or `nb::args` must not use it.

## The list

Adding or removing a release means editing this file in the same change. `pixi run -e audit
audit-gil` fails when a binding releases the GIL without an entry, or an entry no longer
releases it. An entry covers every overload of its name that releases the GIL.

```yaml
--8<-- "scripts/nogil_entry_points.yaml"
```

## Finding candidates

The list was compiled by reading code. To measure instead, `pixi run gil-hotspots`
(`scripts/gil_hotspots.py`) records the benchmarks, then the test suite, with
`py-spy record --gil` and prints the bindings ranked by the time they hold the GIL. `--gil`
keeps only samples of the thread holding it, so a binding that already releases it drops out.
Benchmarks come first: test timings are dominated by setup.

py-spy samples time but cannot count calls or name a nanobind function, so the same run loads
`scripts/gil_call_counts.py`, a pytest plugin that counts every binding call by the Python line
it is made from (`sys.monitoring` CALL events). A sample whose innermost Python frame is such a
line is time inside that binding: the binding runs below the line, and a Python callback it
calls has its own frame. Native frames (`--native`, Linux x86_64 and Windows only) are not
needed, so it runs on macOS too. py-spy attaches to the pytest process and asks for `sudo`
(macOS, and Linux unless `kernel.yama.ptrace_scope=0`).

Two limits show in the table: two bindings called on one line share a row with no per-call
mean, and Python work on a binding's line (building its arguments) counts toward it. Read the
calling line before acting on a row, then judge each candidate above roughly 50 µs per call
against the rule above. `python scripts/gil_hotspots.py report build/gil-hotspots/*.txt`
re-ranks a recording, for instance after the list changed.

## Tests

- `tests/audit/test_audit_gil.py`: the audit's contract, on a fixture TU with one binding per
  release form.
- `tests/test_gil_entry_points.py`: run-time checks of the releases #124 added. The oracle sets a
  switch interval far above the test's run time, so the main thread can only run while a worker
  is inside the call if the call dropped the GIL.
- `tests/tesseract_motion_planners/test_trajopt_ifopt_planner.py` (`_TASK_COMPOSER_RELEASE`): a
  Python composite profile and a Python cost run on the task composer's worker threads while the
  main thread waits on the future. Without the release in `TaskComposerFuture.wait` this deadlocks.

## Known limitation: Environment event callbacks

`Environment` fires its event callbacks under a shared lock on its mutex, after dropping the
unique lock it applied the change under. `applyCommand` (three overloads) and `applyCommands`
release the GIL, so a Python callback waits for the GIL while that shared lock is held. If another
Python thread meanwhile calls an `Environment` method that takes the unique lock without releasing
the GIL (`setState`, the other `applyCommand` overloads, `setName`, ...), each thread waits for
the other. On platforms whose shared mutex favours waiting writers, the same holds for a reader
queued behind a writer.

This needs a registered Python event callback *and* concurrent mutation from Python. Until every
`Environment` binding that takes its mutex releases the GIL, `Environment.init` does not release
it either, so as not to widen the window. A single writer thread is safe.

## Free-threaded builds

The same annotations serve a free-threaded interpreter (#260): `gil_scoped_release` only detaches
the thread state there. Concurrent use of shared C++ objects is the binding's real constraint in
both builds.
