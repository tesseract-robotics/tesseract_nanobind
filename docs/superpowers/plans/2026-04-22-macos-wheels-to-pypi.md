# macOS wheels to PyPI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish macOS arm64 wheels to PyPI on tagged releases alongside the existing Linux x64 wheels.

**Architecture:** Three surgical edits to `.github/workflows/wheels.yml` — re-enable macOS in the `build` matrix, add macOS to the `test-wheel` matrix, and open the `publish` job's artifact download filter to all platforms. All supporting macOS steps (brew, colcon, delocate, plugin-bundle, YAML patch) already exist in the workflow and are guarded by `if: matrix.config.os == 'macos'`. No application code changes.

**Tech Stack:** GitHub Actions, pixi, colcon, pip, delocate, `pypa/gh-action-pypi-publish` (Trusted Publishing via OIDC).

**Related spec:** `docs/superpowers/specs/2026-04-22-macos-wheels-to-pypi-design.md` (commit `aeb684e`).

**Branch:** `jf/macos-wheels-to-pypi` (already created from main at `aeb684e`).

---

## File Structure

Only one file is modified: `.github/workflows/wheels.yml`. Three distinct edits at different locations:

- Lines 28–37 — `build.strategy.matrix.config` (re-enable macOS entry)
- Lines 390–397 — `test-wheel.strategy.matrix.config` (add macOS entry)
- Lines 473–477 — `publish.steps[download-artifact].with.pattern` (drop `-linux-*` suffix, remove TODO)

Each edit becomes its own commit so the PR history shows intent cleanly and any edit can be reverted independently.

---

## Task 1: Pre-flight — verify clean starting state

**Files:**
- Read: `.github/workflows/wheels.yml`

- [ ] **Step 1: Confirm branch and working tree**

Run:
```bash
git branch --show-current
git status
```
Expected:
```
jf/macos-wheels-to-pypi
On branch jf/macos-wheels-to-pypi
...
nothing added to commit but untracked files present (use "git add" to track)
```
(The one untracked file `docs/plans/2026-02-18-viewer-submodule.md` is unrelated and stays untracked.)

- [ ] **Step 2: Confirm current HEAD matches the spec commit**

Run:
```bash
git log --oneline -1
```
Expected: `aeb684e docs: spec for macos wheels to pypi`

- [ ] **Step 3: Verify the workflow YAML parses**

Run:
```bash
python -c "import yaml; yaml.safe_load(open('.github/workflows/wheels.yml'))"
```
Expected: no output, exit 0. This confirms the file is currently well-formed before editing.

---

## Task 2: Re-enable macOS in the `build` matrix

**Files:**
- Modify: `.github/workflows/wheels.yml` (lines 34–37)

- [ ] **Step 1: Apply the edit**

Replace this block (lines 34–37):
```yaml
        # macOS disabled: wheels exceed PyPI size limit
        # - os: macos
        #   arch: arm64
        #   runner: macos-14
```
with:
```yaml
        - os: macos
          arch: arm64
          runner: macos-14
```

No other changes in the `build` job. All macOS-conditional steps (lines 46–49 brew, 93–120 colcon, 142–150 pip wheel, 160–166 delocate, 167–213 plugin-bundle + YAML patch) are already present and guarded.

- [ ] **Step 2: Verify YAML still parses**

Run:
```bash
python -c "import yaml; d = yaml.safe_load(open('.github/workflows/wheels.yml')); print([c for c in d['jobs']['build']['strategy']['matrix']['config']])"
```
Expected output (one line, four entries after product with Python versions is not relevant — we just want both os entries):
```
[{'os': 'linux', 'arch': 'x64', 'runner': 'ubuntu-22.04'}, {'os': 'macos', 'arch': 'arm64', 'runner': 'macos-14'}]
```

- [ ] **Step 3: Commit**

Run:
```bash
git add .github/workflows/wheels.yml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "ci: re-enable macos arm64 in build matrix"
```

Expected: single-file commit, pre-commit hooks pass (ruff skipped — not a Python file).

---

## Task 3: Add macOS to the `test-wheel` matrix

**Files:**
- Modify: `.github/workflows/wheels.yml` (lines 392–397)

- [ ] **Step 1: Apply the edit**

Replace this block (lines 392–397):
```yaml
      matrix:
        python: ['3.9', '3.12']
        config:
          - os: linux
            arch: x64
            runner: ubuntu-24.04  # needs glibc 2.39 to match build env
```
with:
```yaml
      matrix:
        python: ['3.9', '3.12']
        config:
          - os: linux
            arch: x64
            runner: ubuntu-24.04  # needs glibc 2.39 to match build env
          - os: macos
            arch: arm64
            runner: macos-14
```

No other changes in `test-wheel`. The existing steps (checkout, setup-python, download-artifact, install wheel in venv, smoke test, full pytest) are shell-portable and run on macOS without modification. The artifact name `python-${{ matrix.python }}-${{ matrix.config.os }}-${{ matrix.config.arch }}` already resolves correctly for macOS because Task 2 produced a `python-3.12-macos-arm64` artifact (and likewise for 3.9).

- [ ] **Step 2: Verify YAML still parses**

Run:
```bash
python -c "import yaml; d = yaml.safe_load(open('.github/workflows/wheels.yml')); print([c for c in d['jobs']['test-wheel']['strategy']['matrix']['config']])"
```
Expected:
```
[{'os': 'linux', 'arch': 'x64', 'runner': 'ubuntu-24.04'}, {'os': 'macos', 'arch': 'arm64', 'runner': 'macos-14'}]
```

- [ ] **Step 3: Commit**

Run:
```bash
git add .github/workflows/wheels.yml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "ci: add macos arm64 to test-wheel matrix"
```

---

## Task 4: Open `publish` download filter to all platforms

**Files:**
- Modify: `.github/workflows/wheels.yml` (line 475)

- [ ] **Step 1: Apply the edit**

Replace line 475:
```yaml
        pattern: python-*-linux-*  # TODO: change back to python-* after PyPI size limit increase
```
with:
```yaml
        pattern: python-*
```

This drops the linux-only restriction and the now-obsolete TODO.

- [ ] **Step 2: Verify YAML still parses and the pattern updated**

Run:
```bash
python -c "import yaml; d = yaml.safe_load(open('.github/workflows/wheels.yml')); print(d['jobs']['publish']['steps'][0]['with']['pattern'])"
```
Expected:
```
python-*
```

- [ ] **Step 3: Confirm no stale TODO references remain**

Run:
```bash
grep -n "python-\*-linux-\*\|TODO.*size limit" .github/workflows/wheels.yml || echo "clean"
```
Expected: `clean`

- [ ] **Step 4: Commit**

Run:
```bash
git add .github/workflows/wheels.yml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "ci: publish macos wheels to pypi (drop linux-only filter)"
```

---

## Task 5: Push branch + open PR

**Files:**
- None (git + gh operations)

- [ ] **Step 1: Push branch**

Run:
```bash
git push -u origin jf/macos-wheels-to-pypi
```
Expected: remote branch created, tracking set up.

- [ ] **Step 2: Verify recent commits on the branch**

Run:
```bash
git log --oneline origin/main..HEAD
```
Expected (order newest first):
```
<sha> ci: publish macos wheels to pypi (drop linux-only filter)
<sha> ci: add macos arm64 to test-wheel matrix
<sha> ci: re-enable macos arm64 in build matrix
aeb684e docs: spec for macos wheels to pypi
```

- [ ] **Step 3: Open PR**

Run (explicit `--repo` per project convention):
```bash
gh pr create --repo tesseract-robotics/tesseract_nanobind \
  --base main --head jf/macos-wheels-to-pypi \
  --title "ci: publish macos arm64 wheels to pypi" \
  --body "$(cat <<'EOF'
## Summary

- Re-enable macOS arm64 in the `build` matrix (py3.9–3.12)
- Add macOS arm64 to the `test-wheel` matrix (py3.9, 3.12)
- Open `publish` download filter from `python-*-linux-*` to `python-*`

Unblocked by PyPI filesize limit increase to 250 MB.

Spec: `docs/superpowers/specs/2026-04-22-macos-wheels-to-pypi-design.md`

## Test plan

- [ ] PR CI green on all `build` jobs (linux + macos, py3.9/3.10/3.11/3.12)
- [ ] PR CI green on all `test-wheel` jobs (linux + macos, py3.9/3.12)
- [ ] `python-3.12-macos-arm64` artifact extracts correctly: contains 9 plugin factories in `.dylibs/`, task composer YAMLs contain `@PLUGIN_PATH@`
- [ ] Artifact size < 250 MB
EOF
)"
```

Expected: PR URL printed. Capture it for the next task.

- [ ] **Step 4: Confirm PR targets the correct repo**

Read the URL printed in the previous step. It MUST start with `https://github.com/tesseract-robotics/tesseract_nanobind/pull/`. If it points to a fork or different repo, close the PR and retry with the correct `--repo` flag.

---

## Task 6: Monitor PR CI and diagnose failures

**Files:**
- None (observation)

- [ ] **Step 1: Watch build jobs**

Run:
```bash
gh pr checks --repo tesseract-robotics/tesseract_nanobind --watch
```
This blocks until all checks complete. Build jobs typically take 15–25 min (cold cache) or 5–10 min (warm cache).

- [ ] **Step 2: Classify the outcome**

If all checks are green, skip to Task 7.

If any `build` job fails:
- macOS colcon build failure: most likely missing brew dep or an upstream cmake change. Check `Logs` → find the first cmake/compiler error. Fix in the `brew install` or `colcon build` step of the workflow. Commit fix with `ci: fix macos <specific issue>`.
- macOS delocate failure: usually a dylib not found on `delocate-path -L` paths. Add the missing path or copy the dylib explicitly in the plugin-bundle step.
- macOS plugin-bundle failure: a dylib is missing from `ws/install/lib`. Likely the colcon build didn't produce it — check colcon `--packages-ignore` list vs the `PLUGINS` array.

If any `test-wheel` job fails:
- Import error: usually rpath or `DYLD_LIBRARY_PATH` issue. Check the `debug libs` step output for missing deps reported by `otool -L` (macOS) or `ldd` (linux).
- pytest failure specific to macOS: add a platform-conditional `--ignore=` for the failing test directory in `run full tests`. Document *why* in the commit message.

- [ ] **Step 3: Apply any fixes and push**

If fixes are needed, commit with clear messages (`ci: fix macos <thing>`) and push. Re-run this task until green.

---

## Task 7: Inspect the macOS wheel artifact

**Files:**
- None (artifact inspection)

- [ ] **Step 1: Download the py3.12 macOS artifact from the PR run**

Get the PR run ID:
```bash
RUN_ID=$(gh run list --repo tesseract-robotics/tesseract_nanobind \
  --workflow wheels.yml --branch jf/macos-wheels-to-pypi \
  --limit 1 --json databaseId --jq '.[0].databaseId')
echo "Run: $RUN_ID"
```

Download the artifact:
```bash
mkdir -p /tmp/macos-wheel-inspect && cd /tmp/macos-wheel-inspect
gh run download "$RUN_ID" --repo tesseract-robotics/tesseract_nanobind \
  --name python-3.12-macos-arm64
ls -lah
```
Expected: one `.whl` file. Note its size.

- [ ] **Step 2: Verify size is under 250 MB**

Run:
```bash
WHEEL=$(ls /tmp/macos-wheel-inspect/tesseract*.whl)
SIZE_MB=$(stat -f '%z' "$WHEEL" | awk '{printf "%.1f", $1/1048576}')
echo "Wheel size: $SIZE_MB MB"
[[ $(echo "$SIZE_MB < 250" | bc) -eq 1 ]] && echo "OK (< 250 MB)" || echo "FAIL (>= 250 MB)"
```
Expected: `OK (< 250 MB)`. If not, this plan stops and the spec's "shrink the wheel" work becomes necessary.

- [ ] **Step 3: Verify the 9 plugin factories are bundled**

Run:
```bash
cd /tmp/macos-wheel-inspect
mkdir extract && unzip -q tesseract*.whl -d extract
ls extract/tesseract_robotics/.dylibs/ | grep -E 'factories|factory' | sort
```
Expected output (9 files):
```
libtesseract_collision_bullet_factories.dylib
libtesseract_collision_fcl_factories.dylib
libtesseract_kinematics_core_factories.dylib
libtesseract_kinematics_kdl_factories.dylib
libtesseract_kinematics_opw_factory.dylib
libtesseract_kinematics_ur_factory.dylib
libtesseract_task_composer_factories.dylib
libtesseract_task_composer_planning_factories.dylib
libtesseract_task_composer_taskflow_factories.dylib
```

- [ ] **Step 4: Verify task composer YAMLs are patched**

Run:
```bash
grep -l '@PLUGIN_PATH@' extract/tesseract_robotics/data/task_composer_config/*.yaml | wc -l
grep -l '/usr/local/lib' extract/tesseract_robotics/data/task_composer_config/*.yaml | wc -l
```
Expected: non-zero count for `@PLUGIN_PATH@`, zero count for `/usr/local/lib`.

- [ ] **Step 5: Cleanup inspection artifacts**

Run:
```bash
rm -rf /tmp/macos-wheel-inspect
```

---

## Task 8: Merge PR with linear history

**Files:**
- None (git + gh operations)

- [ ] **Step 1: Confirm all CI checks green**

Run:
```bash
gh pr checks --repo tesseract-robotics/tesseract_nanobind
```
Expected: all checks `pass`.

- [ ] **Step 2: Rebase onto latest main (in case main advanced)**

Run:
```bash
git fetch origin main
git rebase origin/main
```
If conflicts arise: resolve them — `wheels.yml` is the only file this PR touches, so conflicts would be with other `wheels.yml` edits landed on main. Resolve by keeping both sets of changes. After resolution: `git rebase --continue`, then `git push --force-with-lease` (confirm with user before force-pushing).

- [ ] **Step 3: Merge PR via rebase (linear history)**

Run:
```bash
gh pr merge --repo tesseract-robotics/tesseract_nanobind --rebase --delete-branch
```
Expected: PR merged, local `jf/macos-wheels-to-pypi` branch auto-deleted on GitHub, no merge commit.

- [ ] **Step 4: Sync local main**

Run:
```bash
git checkout main
git pull --ff-only
git branch -d jf/macos-wheels-to-pypi
```

---

## Task 9: Post-merge validation on main

**Files:**
- None (CI operation)

- [ ] **Step 1: Dispatch the workflow on main**

Run:
```bash
gh workflow run wheels.yml --repo tesseract-robotics/tesseract_nanobind --ref main
```
Expected: workflow queued.

- [ ] **Step 2: Watch the dispatch run**

Run:
```bash
sleep 10
RUN_ID=$(gh run list --repo tesseract-robotics/tesseract_nanobind \
  --workflow wheels.yml --event workflow_dispatch \
  --limit 1 --json databaseId --jq '.[0].databaseId')
gh run watch "$RUN_ID" --repo tesseract-robotics/tesseract_nanobind
```
Expected: all `build` and `test-wheel` jobs green. `publish` does NOT run because it's gated on `refs/tags/`.

- [ ] **Step 3: Confirm no regressions vs PR run**

If the dispatch run passes, we are release-ready. If it fails where the PR run passed, something on main differs from the PR branch — investigate before tagging.

---

## Task 10: Cut release tag and verify PyPI publish

**Files:**
- None (git + PyPI)

- [ ] **Step 1: Pick the next version**

Run:
```bash
git tag --sort=-creatordate | head -3
```
Current tag is `0.34.1.0`. The next patch tag consistent with the 4-segment trigger pattern is `0.34.1.1`. Confirm with user before tagging.

- [ ] **Step 2: Create and push the tag**

Run:
```bash
git tag -a 0.34.1.1 -m "release: macos arm64 wheels"
git push origin 0.34.1.1
```
Expected: tag pushed; `wheels.yml` tag trigger fires.

- [ ] **Step 3: Watch the tag-triggered run**

Run:
```bash
sleep 10
RUN_ID=$(gh run list --repo tesseract-robotics/tesseract_nanobind \
  --workflow wheels.yml --event push \
  --limit 1 --json databaseId --jq '.[0].databaseId')
gh run watch "$RUN_ID" --repo tesseract-robotics/tesseract_nanobind
```
Expected: `build`, `test-wheel`, AND `publish` jobs all green. The `publish` job runs because the ref starts with `refs/tags/`.

- [ ] **Step 4: Verify wheels on PyPI**

Run:
```bash
curl -s https://pypi.org/pypi/tesseract-robotics-nanobind/0.34.1.1/json \
  | python3 -c "import sys, json; d = json.load(sys.stdin); [print(f\"{f['filename']:70s} {f['size']/1e6:7.1f} MB\") for f in d['urls']]"
```
Expected: Linux and macOS wheels for each Python version listed (8 total: 4 linux + 4 macos).

- [ ] **Step 5: Smoke test installed wheel on a macOS arm64 machine**

Run in a clean environment:
```bash
python3.12 -m venv /tmp/tesseract-pypi-test
source /tmp/tesseract-pypi-test/bin/activate
pip install --no-cache-dir tesseract-robotics-nanobind==0.34.1.1
tesseract_nanobind_selftest
python -c "from tesseract_robotics import tesseract_common, tesseract_geometry, tesseract_scene_graph; print('OK')"
deactivate
rm -rf /tmp/tesseract-pypi-test
```
Expected: selftest passes, imports succeed.

---

## Rollback

If anything above fails catastrophically AND a broken wheel reaches PyPI, follow the spec's rollback procedure:

1. Yank the broken release via PyPI web UI (Manage → Release → Yank).
2. Revert the three CI commits on main (`git revert <sha>` for each of the three `ci:` commits from this plan).
3. Cut a new patch version to unblock future publishes.

If CI fails before tagging (Tasks 6–9), no rollback needed — just fix forward on the branch or abandon the PR.

---

## Self-review notes

Spec coverage check passes:
- Spec §"Scope → In scope" bullet 1 (enable macOS arm64 in build matrix) → Task 2
- Spec §"Scope → In scope" bullet 2 (add macOS to test-wheel) → Task 3
- Spec §"Scope → In scope" bullet 3 (flip publish filter) → Task 4
- Spec §"Scope → In scope" bullet 4 (remove TODO) → Task 4 (Step 1 drops the comment along with the filter change; Step 3 grep confirms)
- Spec §"Validation plan" steps 1–6 → Tasks 5, 6, 7, 8, 9, 10

No placeholders. Each commit-producing task shows exact edits and exact commit commands. Artifact inspection commands are concrete. The only genuinely unknown branch (CI failure diagnosis in Task 6, Step 2) provides three labelled failure modes with targeted fixes rather than hand-waving.
