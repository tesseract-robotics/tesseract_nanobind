# MkDocs refresh — API-truth sweep

**Date:** 2026-04-24
**Status:** design
**Owner:** @jelleferinga

## Problem

`docs/` has drifted from `src/` to the point that key guides are no longer instructional — they are misleading. The worst offenders do not just describe an old API; they describe an **API that never existed**:

- `docs/index.md` and `docs/getting-started/quickstart.md` show a `Planner(robot)` class with `.plan(start, goal, planner="ompl")`. No such class exists; real planning is `plan_freespace(robot, MotionProgram(...))`.
- Quickstart shows `Composer(robot).add_freespace(...).add_cartesian(...)`. No such class; real class is `TaskComposer` with `.plan(robot, program, ...)`.
- Quickstart shows `robot.check_collision(joints)` and `robot.get_contacts(joints)`. Neither method exists.
- `robot.fk(joints, group="manipulator")` — wrong arg order; real signature is `fk(group_name, joint_positions, tip_link=None)`.
- `pip install tesseract-robotics-nanobind` — wrong package name; PyPI name is `tesseract_robotics`.

`docs/user-guide/low-level-sqp.md` is worse: every snippet uses the pre-0.34 `JointPosition` / `CartPosInfo` / `CollisionCache` API, and imports from `tesseract_robotics.ifopt` — a module **removed** in 0.34. Users following the page hit `ImportError` on line 1 of the basic example.

Meanwhile, real post-0.34 work isn't covered: the viewer submodule refactor (`f277eb5`), `TaskComposer.warmup()` (`06d25ad`), the polymorphic `CompositeInstruction.push_back` (`6d17314`, CHANGELOG 0.35.0.0 seed), and the `CHANGELOG.md` itself (added in `afbf858`, not linked from the rendered site).

Commit-history alone won't catch this. Much of the drift predates any identifiable "breaking commit" — it's hallucinated API that was never correct. The fix is a content-first sweep: treat current `src/` as source of truth and walk every page.

## Goal

Bring `docs/` into agreement with current `src/` (post-0.34, post-flatten, post-viewer-refactor, post-example-relocation). Wire `CHANGELOG.md` into the rendered site. Make `mkdocs build --strict` pass and keep it green via CI.

## Non-goals

- Writing-style rewrites for their own sake. Fix content; preserve voice where content is correct.
- `mkdocs-macros` / doctest / snippet-execution infrastructure. Flagship self-verification via existing `tests/examples/` is sufficient.
- Deprecating `changes.md`. It's a stable 0.33→0.34 migration artifact; relabel in nav, content untouched.
- Fresh 0.34→0.35 migration guide. `CHANGELOG.md` covers it. Revisit only if 0.35 ships breaking changes.
- Touching `pyproject.toml` / `mkdocs.yml` beyond: nav (CHANGELOG + Viewer), removing dead `polyfill.io` CDN, `changes.md` nav label.

## Methodology

1. **Source of truth priority:** current `src/tesseract_robotics/**` > `src/tesseract_robotics/examples/**` > commit log (for context only) > existing docs prose.
2. **Two snippet tiers:**
   - *Prose pages* (`index.md`, `getting-started/`, `user-guide/*.md` except flagship pointers, `api/` prose preambles, `developer/`): short hand-crafted snippets, each checked once against `src/`.
   - *Flagship workflows* (examples pages, pick-and-place / raster / online-planning / viewer walkthroughs): pulled verbatim from `src/tesseract_robotics/examples/<name>.py` via `--8<-- "src/…"` snippet markers. `pymdownx.snippets` is already enabled in `mkdocs.yml` with `base_path: ['.', 'docs']`, so this requires only `# --8<-- [start:region]` / `# --8<-- [end:region]` annotations in the example files — no config changes.
3. **Verification tiers:**
   - `mkdocs build --strict` green locally and in a new CI workflow.
   - Flagship snippets = self-verifying via existing `tests/examples/` suite.
   - Prose snippets = manually verified during refresh via signature match against `src/`.
4. **Order of attack:** one branch, one PR, commit per page group in priority order — worst drift first so each commit is individually reviewable and bisectable.

## Scope — per-page plan

**Legend:** 🔴 broken/wrong · 🟡 stale but mostly right · 🟢 correct, minor polish

### Narrative pages

| Page | State | Action |
|---|---|---|
| `index.md` | 🔴 | Rewrite quick example against real API (`plan_freespace(robot, program)` + `MotionProgram` builder). Fix `robot.fk` signature. Verify architecture mermaid vs. actual module graph. If Performance Hz table (128/73/5-10) can't be re-sourced from `online_planning_sqp_example.py`, drop specific numbers and link to the example. |
| `getting-started/installation.md` | 🟢 | Recent (`9fd37aa`, 2026-02). Spot-check pixi commands match `pyproject.toml [tool.pixi.tasks]`. Verify both user-via-PyPI and developer-via-pixi paths. |
| `getting-started/quickstart.md` | 🔴 | Full rewrite. Replace `Planner(robot).plan(...)` with `plan_freespace(robot, program)`. Replace `Composer(robot).add_freespace(...)` with `TaskComposer.plan(robot, program)`. Remove `robot.check_collision` / `robot.get_contacts`. Fix `pip install tesseract-robotics-nanobind` → `tesseract_robotics`. |
| `getting-started/concepts.md` | 🟡 | Remove final `robot.check_collision` ref. Keep Command Language / Poly explanation — still accurate. Update Task Composer pipeline ascii box to reference `get_available_pipelines()` (36 pipelines). |
| `user-guide/environment.md` | 🟡 | Last touched 2025-12; predates flatten + viewer refactor. Audit imports + signatures. |
| `user-guide/kinematics.md` | 🟡 | Audit FK/IK snippets against `planning/core.py`. Verify scalar-last quaternion convention is documented. |
| `user-guide/collision.md` | 🟡 | Verify against `tesseract_collision` + 0.34 `CollisionEvaluatorType` enum (`DISCRETE`/`CONTINUOUS`/`LVS_DISCRETE`/`LVS_CONTINUOUS`, not `SINGLE_TIMESTEP`). |
| `user-guide/planning.md` | 🔴 | Tab blocks use fictional `Planner(robot).plan(start, goal, planner="ompl")`. Replace with real `plan_freespace` / `plan_ompl` / `plan_cartesian`. Keep costs-vs-constraints + Motion Types + Profile Caching sections. Drop "Collision Configuration (0.33 API)" section header — it's just *the* API now. |
| `user-guide/task-composer.md` | 🟡 | Verify `TaskComposer` API against `planning/composer.py`. Document `warmup()` (shipped `06d25ad`, currently undocumented). Reference `get_available_pipelines()`. |
| `user-guide/serialization.md` | 🟢 | Recent (`56dce3e`). Spot-check. |
| `user-guide/low-level-sqp.md` | 🔴🔴 | **Full rewrite.** Replace `JointPosition` with `Var`/`Node`/`createNodesVariables`. Replace `CartPosInfo` with direct `CartPosConstraint` ctor. Remove `CollisionCache`. Replace `from tesseract_robotics.ifopt import Bounds` with `from tesseract_robotics.trajopt_ifopt import Bounds`. Replace `evaluateTotalExactCost(x)` with `getTotalExactCost()`. Use `online_planning_sqp_example.py` as canonical via `--8<--`. |
| `user-guide/viewer.md` | ➕ NEW | Create. Cover `TesseractViewer` basics (`update_environment`, `update_trajectory`, `start_serve_background`). Flagship walkthroughs via `--8<--` from `shapes_viewer.py`, `abb_irb2400_viewer.py`, `twc_workcell_positioner_viewer.py`. Note submodule requirement for TWC workcell. |

### Examples pages

| Page | State | Action |
|---|---|---|
| `examples/index.md` | 🟡 | Fix invocation: `python examples/foo.py` → `tesseract_<name>_example` console script (installed via `[project.scripts]`) or `python -m tesseract_robotics.examples.<name>`. Fix `pip install tesseract-robotics-nanobind` → `tesseract_robotics`. Add `reeds_shepp_example.py` + `twc_workcell_positioner_viewer.py` (currently missing from tables). |
| `examples/basic.md` | 🟡 | Audit each linked example; update invocation; use `--8<--` for flagship (kinematics). |
| `examples/planning.md` | 🟡 | Audit. Use `--8<--` for pick-and-place, raster, car-seat, chain flagships. |
| `examples/online-planning.md` | 🟡 | Audit. Align with low-level-sqp rewrite (cross-link). |

### API reference pages

| Page | State | Action |
|---|---|---|
| `api/index.md` | 🔴 | "Robot, Planner, Composer classes" line — Planner is fictional. Fix module table to: `Robot`, `MotionProgram`, `TaskComposer`, `plan_freespace/plan_ompl/plan_cartesian`. |
| `api/planning.md` | 🟡 | mkdocstrings auto-block is fine. Hand-written preamble says "Classes: Robot / Planner / Composer" — fix names. |
| `api/tesseract_common.md` | 🟡 | Audit `Isometry3d` snippets. Verify `Pose` class (from `planning/transforms.py`) is mentioned. |
| `api/tesseract_collision.md` | 🟡 | Verify 0.34 enum names (`CollisionEvaluatorType.DISCRETE` etc.). Verify `clone()` bindings (`dab4625`). |
| `api/tesseract_command_language.md` | 🟡 | Audit. Add polymorphic `push_back(CompositeInstruction)` (`6d17314`, 0.35.0.0). |
| `api/tesseract_environment.md` | 🟡 | Audit. Check `Robot.from_urdf` snippet shows both URDF + SRDF args. |
| `api/tesseract_geometry.md` | 🟢 | Spot-check against `geometry_showcase_example.py`. |
| `api/tesseract_kinematics.md` | 🟢 | Spot-check. |
| `api/tesseract_scene_graph.md` | 🟢 | Spot-check. |
| `api/tesseract_serialization.md` | 🟡 | Verify Boost→Cereal switch not leaked as Boost references. |
| `api/tesseract_motion_planners.md` | 🟡 | Verify `PlannerRequest` fields against current source. |
| `api/tesseract_motion_planners_ompl.md` | 🟡 | Verify profile class names + 0.34 enum values. |
| `api/tesseract_motion_planners_trajopt.md` | 🟡 | Verify `TrajOptCollisionConfig` location (trajopt_ifopt, re-exported from trajopt). |
| `api/tesseract_motion_planners_descartes.md` | 🟡 | Verify. |
| `api/tesseract_motion_planners_simple.md` | 🟡 | Verify. |
| `api/tesseract_task_composer.md` | 🟡 | Verify against current 36-pipeline list. |
| `api/trajopt_ifopt.md` | 🔴 | Rewrite to 0.34 (`Var`/`Node`/`NodesVariables`, no `JointPosition`, no `CartPosInfo`, no `CollisionCache`). |
| `api/trajopt_sqp.md` | 🟡 | Verify `IfoptProblem`/`IfoptQPProblem` two-level split (0.34 change). |

### Developer pages

| Page | State | Action |
|---|---|---|
| `developer/index.md` | 🟡 | Fix mermaid diagram ("Robot, Planner, Composer" → real names). Fix "run an example" command. Otherwise recent & accurate. |
| `developer/migration.md` | 🟢 | SWIG→Nanobind historical; stable. Spot-check only. |
| `developer/trajopt_ifopt_missing_constraints.md` | 🟡 | Verify "all 4 constraints bound" claim against current bindings. Fix obsolete `JointPosition` refs in shown C++ signatures. If claim holds, reframe page as historical reference and note completion. |

### Structural / config

| Item | Action |
|---|---|
| `mkdocs.yml` nav | Add top-level `Changelog: changelog.md` entry. Rename `Changes: "0.33 → 0.34"` → `Migration Guides > "0.33 → 0.34": changes.md`. Add `User Guide > Viewer: user-guide/viewer.md`. |
| `mkdocs.yml` CDN cleanup | Remove `polyfill.io` line (dead/hijacked domain). Audit `mathjax` CDN — if math isn't rendered in any doc, drop the whole math block. |
| `docs/CHANGELOG.md` | Symlink (or copy + add pre-commit sync hook) from repo root `CHANGELOG.md`. Symlink preferred; cross-platform concern is moot because mkdocs runs on CI + dev boxes (both POSIX). |
| `docs/QUICKSTART.md` | Currently excluded from mkdocs via `plugins.exclude.glob`. Decision: `git rm` if vestigial. Check `git log docs/QUICKSTART.md` for intent — if it was GitHub-landing content, leave it and keep exclusion. |
| `docs/requirements.txt` | Verify mkdocs-material / mkdocstrings pins are still sane vs `pyproject.toml [tool.pixi.dependencies]`. |
| `.github/workflows/docs.yml` | ➕ NEW — `mkdocs build --strict` on PRs touching `docs/**` or `mkdocs.yml`. |

## Commit sequence

Single branch `docs/refresh-api-truth`, single PR to `main`. Each commit is individually bisectable and reviewable.

1. `docs: fix fictional Robot/Planner/Composer API in quickstart + index` — `index.md`, `quickstart.md`.
2. `docs: drop invented robot.check_collision in concepts + collision audit` — `concepts.md`.
3. `docs(user-guide): rewrite planning.md against real API` — `user-guide/planning.md`.
4. `docs(user-guide): rewrite low-level-sqp to 0.34 Var/Node/NodesVariables` — `user-guide/low-level-sqp.md`.
5. `docs(user-guide): audit environment/kinematics/collision/task-composer` — add `warmup()`, reference `get_available_pipelines()`.
6. `docs(user-guide): add viewer page` — new `user-guide/viewer.md` using `--8<--` flagships.
7. `docs(examples): fix invocation + list missing examples` — console-script commands, add `reeds_shepp`, `twc_workcell_positioner_viewer`, wire `--8<--`.
8. `docs(api): fix hand-written preambles + code blocks` — including full `trajopt_ifopt.md` rewrite.
9. `docs(developer): fix index mermaid + example-run command; audit missing-constraints page`.
10. `docs(nav): wire CHANGELOG, relabel changes.md, drop polyfill.io` — `mkdocs.yml` + `docs/CHANGELOG.md` symlink.
11. `ci: add docs.yml workflow running mkdocs build --strict` — `.github/workflows/docs.yml`.

## Verification

- **Per commit, locally:** `pixi run docs-build` completes; `mkdocs build --strict` green.
- **At PR-open:** new `.github/workflows/docs.yml` runs strict build.
- **Flagship `--8<--` snippets:** self-verified via existing `tests/examples/` suite (already in pre-push hook + CI).
- **Prose snippets:** each verified against current `src/` during refresh. One-time manual check; no CI coverage (acceptable — the fictional-API drift that motivated this refresh happened once and is cheap to catch at review time).

## Done criteria

1. `mkdocs build --strict` green locally and in the new CI workflow.
2. Zero references in `docs/` to: `Planner(robot)`, `Composer(robot)`, `robot.check_collision`, `robot.get_contacts`, `from tesseract_robotics import ifopt`, `JointPosition`, `CartPosInfo`, `CollisionCache`, `python examples/` (as invocation), `tesseract-robotics-nanobind` (pip name), `polyfill.io`.
3. `CHANGELOG.md` wired into nav and rendered at `/changelog/`.
4. `user-guide/viewer.md` exists with working `--8<--` flagship snippets.
5. Single PR merged to `main`; nothing left on feature branches.

## Open items resolved during implementation

- **Performance numbers in `index.md`:** if the 128/73/5-10 Hz figures cannot be re-sourced from `online_planning_sqp_example.py` output, remove the specific numbers and link to the example.
- **`QUICKSTART.md` fate:** `git log docs/QUICKSTART.md` tells us intent — vestigial → `git rm`, GitHub-landing → leave + keep exclusion.
- **`trajopt_ifopt_missing_constraints.md` fate:** verify 4-constraint claim; if all bound, reframe as historical; else keep as actionable TODO.

## Risks

- **`--8<--` snippet regions require markers in example files.** Small one-time edit to `src/tesseract_robotics/examples/*.py` to add `# --8<-- [start:X]` / `# --8<-- [end:X]` comments. Markers are Python comments; runtime unaffected; `ruff` leaves them alone.
- **`docs/CHANGELOG.md` as symlink:** mkdocs follows symlinks by default. CI runners are Linux/macOS (POSIX). No Windows dev concern at present (Windows wheels still WIP per CHANGELOG). If this changes, convert to a pre-commit sync hook.
- **CI cost:** one extra GitHub Actions job per PR touching docs. Runs on `ubuntu-latest` in under a minute. Negligible.
