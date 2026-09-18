# Issue work packages

Proposed grouping of all **12 open issues**, read from GitHub on **2026-09-18**, including their discussions and the three open pull requests. Each issue has one primary package. Cross-package dependencies do not duplicate ownership.

This is a planning record, not a claim that the proposed contracts are implemented. GitHub membership, labels, and milestones have not been changed. Implementation readiness below is based on source and documentation inspection, not new runtime tests.

## Docs-driven operating model

Follow the useful distinction in `tesseract_s3_slicer`: published architecture and user guides own the contract; issues describe a specific gap against that contract; executable plans describe how to close it. Keep proposed contracts clearly marked until decided, and distinguish decided behavior from demonstrated implementation.

For each package:

1. Reconcile the issue with the current code, documentation, tests, and existing PRs. Preserve evidence of partial completion; do not reimplement it.
2. Decide the contract in its owning documentation page: consumer, inputs, outputs, ownership, failure behavior, and compatibility implications.
3. Give each remaining issue a link to that section and a concrete acceptance criterion. Link the section back to its issues.
4. Execute bounded slices. Each plan names its consumer, expected change volume, stop rule, and non-goals, following the slicer's `plans/README.md` convention.
5. Record consumer-level evidence alongside the contract. Close issues only against their individual criteria; code merged, wheel released, and downstream adoption are separate states.

The table is the complete allocation. The sections below define proposed boundaries and closure evidence; they are not eight approved implementation plans.

| Package | Outcome | Primary issues | Documentation owner | First action |
|---|---|---|---|---|
| W1 | Explicit tool and reference-frame semantics | [#30], [#133] | [Kinematics](../user-guide/kinematics.md), [planning API](../api/planning.md) | Reconcile the existing TCP mapping with the missing selection/composition contract |
| W2 | Safe Python-thread concurrency at native boundaries | [#124], [#134] | Proposed `developer/threading.md`; [environment guide](../user-guide/environment.md) | Audit native clone ownership and callback behavior |
| W3 | Reusable planning sessions and SQP lifecycle | [#12], [#137] | [TaskComposer](../user-guide/task-composer.md), [low-level SQP](../user-guide/low-level-sqp.md) | Separate cold setup, repeated solves, and scene updates |
| W4 | Shared planning examples and reliable regression cases | [#84], [#109] | [Examples](../examples/index.md), proposed `developer/planning-fixtures.md` | Reconcile #109 with the merged planning fixes |
| W5 | Bounded collision-manager setup cost | [#140] | [Collision](../user-guide/collision.md), proposed `developer/collision-performance.md` | Preserve the reported benchmark cases and distinguish native options |
| W6 | A maintained lane following upstream development | [#141] | [Development](index.md), proposed `developer/upstream-tracking.md` | Specify branch, dependency, CI, and release boundaries |
| W7 | Clear native-binding and convenience-API boundaries | [#42] | [Architecture](index.md#architecture), [migration](migration.md) | Compare retained PR #78 with current main |
| W8 | Smaller wheels with the same supported functionality | [#106] | Platform wheel guides: [macOS](macos-wheels.md), [Linux](linux-wheels.md), [Windows](windows-wheels.md) | Establish whether an upstream dependency variant makes this actionable |

## W1 — Tool and reference-frame semantics

**Consumer:** a caller selecting a physical solver tip or a named SRDF TCP for FK, IK, and planning, including the downstream Qt tool selector.

These issues share one semantic boundary: a transform is useful only when its frame and composition rule are explicit. #30 covers guessed `tcp_frame` and `working_frame`; #133 covers named TCP discovery and interpretation.

**Current evidence:** `Robot.get_manipulator_info()` still chooses the last active link when no TCP is supplied and defaults the working frame to `base_link`. The SRDF binding already implements `group_tcps` through a copied nested mapping. #133 therefore needs a gap audit, not an assumption that the getter is absent.

**Slices:** establish physical-tip versus named-offset semantics; verify the mapping and document its copy/write behavior; replace ambiguous selection according to the agreed API contract and update callers.

**Closure evidence:** multiple named TCPs survive SRDF-to-Python round trips; documented transform composition produces the expected pose; branching groups cannot silently select the wrong tip; invalid or ambiguous selections fail explicitly. Document any signature migration.

**Non-goal:** package renaming or a general frame-type redesign.

## W2 — Native concurrency and ownership

**Consumer:** Python workers performing native operations while another Python thread remains responsive.

#124 supplies the API-wide policy; #134 is its first concrete application. A GIL release is justified per operation by native synchronization, shared state, object lifetime, and Python callbacks. It does not establish that concurrent operations on one mutable native object are safe.

**Slices:** document a bounded operation inventory and its classifications; audit and implement `Environment.clone()`; cover the remaining classified operations in separate, evidence-backed slices. Define the inventory before treating #124 as an executable task.

**Closure evidence:** an ownership/callback contract for each covered operation and a behavioral test that another Python thread progresses during clone. #134 additionally requires a published wheel and the downstream cold-pool responsiveness acceptance in [tesseract_qt_py#19]. A local overlay or source-only fix does not satisfy that issue.

**Non-goal:** blanket `gil_scoped_release` annotations or reducing Bullet allocation cost. W5 owns allocation; W2 owns safe concurrency.

## W3 — Planning-session and solver lifecycle

**Consumer:** applications issuing repeated plans or incremental optimization updates.

#137 and #12 both concern the lifecycle of expensive native state: what is created once, what can be reused, and what must change when the scene changes. Keep two bounded slices under that shared contract: TaskComposer reuse, then low-level SQP reuse.

**Current evidence:** the low-level SQP guide already covers API choice, setup, constraints, the solver loop, and warm-starting. It also states that the example rebuilds the problem when obstacle poses are baked into collision constraints. #12 remains a reconciliation task, especially for scene updates and efficient reuse. #137 reports repeated plugin setup through convenience wrappers.

**Closure evidence:** executable examples distinguish cold setup from repeated-call latency, state when scene changes require rebuilding, demonstrate correct results after updates, and handle solver failure explicitly. Choose and document whether convenience wrappers remain one-shot or accept reusable state. Do not assume a module-global composer is safe; W2's lifetime rules constrain that choice.

**Non-goal:** promising a universal replanning rate or rewriting the native solver.

## W4 — Reference workloads and planning reliability

**Consumer:** solver developers, regression tests, and before/after documentation figures using the same understandable toolpaths.

#84 defines reusable inputs; #109 supplies a concrete regression consumer. They belong together because stable fixture semantics make failures reproducible, but fixing or closing #109 must not wait for a large fixture library.

**Current evidence:** #84 identifies the existing RAPID zigzag generator as a candidate. Recent main-branch changes correct collision margins, time parameterization, and failure diagnostics. They overlap #109's symptoms; they do not by themselves prove that its xdist failure is closed or that its original reseeding explanation is correct.

**Slices:** first reconcile #109 against the merged fixes and preserve any remaining reproducer; then extract a small fixture set from existing cases. Declare frames, units, robot/TCP assumptions, and expected path properties. Keep path generation independent of planner execution.

**Closure evidence:** the same fixture feeds a test and a documented before/after example; the reported failure is either reproduced and fixed or closed with matching evidence. Assess collision/trajectory properties without claiming identical OMPL paths across seeds or platforms.

**Non-goal:** a broad benchmark framework or masking failures with larger timeouts.

## W5 — Collision-manager allocation cost

**Consumer:** Descartes and TrajOpt planning workloads whose initialization time or memory is dominated by contact-manager clones.

#140 reports Bullet pool allocation and per-waypoint clone ownership, and separately asks about redundant joint solutions. Keep the allocation work bounded; the joint-turn policy is a distinct semantic question within the issue, not permission to change a planner default as a performance fix.

**Current evidence:** timings and memory figures are issue-reported measurements, not remeasured here. The discussion proposes Coal as an alternative and describes upstream integration dependencies. That is a candidate direction, not demonstrated equivalence or current release availability.

**Slices:** preserve free-space and obstacle workloads; compare configurable Bullet pools, clone ownership changes, and the proposed Coal route when available. W6 can supply an upstream-development environment for the last option; the existing Bullet investigation does not depend on W6.

**Closure evidence:** report setup time, solve time, peak memory, collision results, and trajectory validity for the same workloads. Confirm supported behavior when configured pools are exceeded. Resolve the redundant-solution question explicitly without conflating configuration continuity with allocation savings.

**Non-goal:** choosing a new backend on timing alone or bundling GIL changes into the experiment.

## W6 — Upstream development lane

**Consumer:** binding contributors adopting native APIs before the next Tesseract release.

#141 currently has only a title. Its surrounding motivation appears in #140's discussion: release lag prevents consumers from trying recent native work. The first deliverable is an explicit lane policy, not immediately changing stable dependencies.

**Slices:** define a maintained tracking branch, supported dependency combinations and build route, representative contract checks, failure reporting, and promotion back to the released-dependency lane. Decide separately whether development wheels are distributed and how they are identified.

**Existing work:** open [PR #120] adds OMPL `simplify_time`; [PR #129] adds signed-distance-field bindings. Both descriptions cite upstream changes not yet released when written. Verify those prerequisites against current upstream before scheduling their integration. Neither is an additional open issue in this inventory.

**Closure evidence:** the lane builds against a documented current upstream combination, runs representative binding/consumer checks, exposes failures visibly, and has a repeatable update procedure without silently moving stable releases onto unreleased dependencies.

**Non-goal:** absorbing the namespace migration in W7 or dictating upstream's release cadence.

## W7 — Package structure and API discoverability

**Consumer:** users translating C++ examples, Python users choosing the convenience layer, and IDE/type-checking consumers of stubs.

#42 is a broad migration proposal. [PR #78] already replays much of the structural work. Its description and historical test results are evidence of retained work, not evidence that it is ready for current main.

**Slices:** compare the retained implementation with current main; decide public namespaces and migration behavior; handle source layout and build-generated stubs; migrate imports and documentation; distinguish direct-binding examples from convenience-API examples. Resolve namespace-collision and packaging questions before executing the move.

**Closure evidence:** installation/import, cross-module registration, generated-stub/IDE behavior, and both example layers meet the agreed contract across supported platforms. Intervening geometry, emitter, and profile APIs remain represented. Any eventual removal of existing paths needs the user's explicit authorization.

**Non-goal:** rewriting planning behavior or coupling the move to upstream-head adoption. Coordinate import edits with W6, but keep their acceptance gates independent.

## W8 — Wheel dependency footprint

**Consumer:** users downloading and installing portable wheels.

#106 explicitly describes a low-priority, currently non-actionable dependency chain: PCL pulls in VTK. Its reported size measurements are a baseline to refresh if work resumes, not a reason to delete bundled libraries.

**Slices:** establish a supported upstream dependency variant or other justified build change; then measure packaging impact and exercise the affected runtime functionality.

**Closure evidence:** a measured payload reduction with unchanged supported behavior on the affected wheel platforms. If no supported route exists, document the constraint and the exact external change needed to reopen implementation.

**Non-goal:** excluding libraries solely because a limited test suite did not load them. Park this package until its dependency condition changes.

## Order and coordination

1. **Reconcile first:** W1's existing mapping, W3's existing SQP guide, W4's recent fixes, and W7's retained PR. This prevents scheduling completed work and separates actual gaps from stale diagnoses.
2. **First delivery:** W2's clone slice is the strongest bounded starting point: it has a named downstream consumer and an explicit release/adoption gate. Establish W1's frame contract alongside the planning backlog; avoid overlapping implementation changes without assigned ownership.
3. **Early enabling work:** specify W6's development lane before attempting unreleased-native integrations. This can unblock a Coal experiment but is not required for W2 or for investigating current Bullet behavior.
4. **Next:** W3's lifecycle slices and W4's small fixture set support meaningful W5 measurements. Reuse existing cases immediately rather than waiting for the entire fixture package.
5. **Separate migration:** schedule W7 with an explicit consumer migration window. Keep W8 parked.

The packages are organized by shared contract and closure evidence, not by existing labels: #42's `CI` label hides an API migration, while #140 combines native allocation and joint-policy questions. No issue is assigned twice, and a one-issue package remains appropriate when its consumer and completion criterion are independent.

[#12]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/12
[#30]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/30
[#42]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/42
[#84]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/84
[#106]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/106
[#109]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/109
[#124]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/124
[#133]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/133
[#134]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/134
[#137]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/137
[#140]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/140
[#141]: https://github.com/tesseract-robotics/tesseract_nanobind/issues/141
[PR #78]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/78
[PR #120]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/120
[PR #129]: https://github.com/tesseract-robotics/tesseract_nanobind/pull/129
[tesseract_qt_py#19]: https://github.com/jf---/tesseract_qt_py/issues/19
