# RAPID exporter — findings & plan

Working notes for evolving the RAPID backend. Reminder of what RoboDK's RAPID
post-processors expose as tunable, how it maps to ours, and the concrete next
steps.

## Provenance (read first)

Option names and semantics below come from **RoboDK's public post-processor
documentation** (`robodk.com/doc/en/Post-Processors.html`) and the readable
config shim of the bundled posts — they are documented customization variables,
not proprietary logic.

This repo derives **only** from the Apache-2.0 `ros-industrial-attic/robodk_postprocessors`
snapshot and public docs. RoboDK's current shipped posts are **compiled**
(Cython 3.2.x, with `CYTHON_COMPRESS_STRINGS` — the format templates are
compressed in the `.so`, unreadable by design). We do **not** reverse them.
Behavioral validation against RoboDK output is maintained privately, outside
this repo.

## Findings — RoboDK RAPID options vs our exporter

Surveyed against `ABB_RAPID_OmniCore` (newest ABB controller family). Each row:
the documented option, what it controls, our current state, verdict.

| RoboDK option | controls | ours today | verdict |
|---|---|---|---|
| `PROG_EXT` (`prg`/`mod`/`modx`) | file extension per controller (S4 / IRC5 / OmniCore) | `RapidBackend` hardcodes `.mod`; `emit_rapid` returns a bare `str`, so the name is discarded | tweak — surface it (see Plan #2) |
| `FIRST_PROG_AS_MAIN` | first proc named `Main()` | `emit_rapid(proc_name=...)` | ✅ covered |
| `MAX_LINES_X_PROG`, `INCLUDE_SUB_PROGRAMS`, `MAX_SUBPROG_LINES`, `EXTERNAL_DRIPFEEDER`, `RAPID_REMOTE_PATH` | split a long program into paged module files + a synthesized main caller | flat IR — not done | **v2: program splitting** |
| `CHARACTER_LIMIT = 32` | RAPID identifier length cap | `rapid_writer` TODO says *16* (wrong) and never enforces it | **bug — fix (Plan #1)** |
| `TURNTABLE_IGNORE`, `MECHANICAL_UNIT_NAME`, `AXES_RATIO`, `EXTAXES_IDX` | external-axis / turntable wiring | `ExternalAxis` stub only | **v2: external axes** |
| `MOVEJ_AS_MOVEABSJ = True` | render joint moves as `MoveAbsJ` vs `MoveJ` | always `MoveAbsJ` for `JointMove` | ✅ matches default (`MoveJ` needs a pose a `JointMove` doesn't carry) |
| `SPEED_MMS=500`, `SPEEDDATA='v500'`, `ZONEDATA='z1'`, `TOOLDATA='tool0'`, `WOBJDATA='wobj0'` | default speed/zone/tool/wobj | `RapidProfile` defaults `v200`/`z10`/`tool0`/`wobj0` | ⚠️ default mismatch (OmniCore ships `v500`/`z1`) — choice, not a bug |
| `USE_RWS_OVER_FTP`, `ProgSendRobot` | upload transport (RWS/FTP) | we emit files, never upload | ❌ out of scope |

**Takeaway:** the list mostly *confirms the roadmap* — program splitting and
external axes are the two big v2 items, and RoboDK exposing exactly those as the
headline knobs says we drew the v1 boundary in the right place. It also surfaced
one real bug (identifier cap) and one API seam (`emit_rapid` returns `str` while
every other brand returns `EmittedProgram`).

## Plan

### 1. Fix the identifier-limit bug (small, safe — do first)

RAPID identifiers cap at **32** chars (per OmniCore `CHARACTER_LIMIT`), not 16.
The `rapid_writer.RapidCommand` TODO is wrong and nothing enforces a cap.

- [ ] In `rapid_writer.py`, correct the `RapidCommand` TODO comment: 32, not 16.
- [ ] In `emit.py`, sanitize `module_name` and `proc_name` through
      `core.naming.safe_identifier(name, brand="RAPID", max_len=32)` before
      passing to `RapidBackend`.
- [ ] Add `tests/emitters/rapid/test_emit.py::test_long_name_sanitized` — a
      33-char or illegal-char module name emits a legal ≤32 identifier.
- [ ] Verify byte-identical for existing goldens (current names — `main_program`,
      `main`, `MOD_SPIKE` — are short + legal, so unchanged).

### 2. Unify `emit_rapid` → `EmittedProgram` + `file_ext` (decision required)

`emit_rapid` returns a bare `str`; `emit_krl`/`emit_ls`/`emit_jbi`/`emit_urscript`
return `EmittedProgram` (filename → text). Unifying lets RAPID expose `PROG_EXT`
(`mod`/`modx`/`prg`) and target OmniCore/S4, and removes the last cross-brand
API asymmetry.

- [ ] `emit_rapid(..., file_ext: str = "mod") -> EmittedProgram`.
- [ ] `RapidBackend.prog_finish` already returns `{f"{module}.{ext}": text}` —
      thread `file_ext` through.
- [ ] Update `test_zigzag_square` / `test_dispatcher` / `test_full_dsl_golden`
      callers from `emit_rapid(...)` (str) to `.text`. This is a deliberate API
      change, not a test fudge — the assertions themselves are unchanged.
- [ ] Re-export nothing new from `emitters/__init__` (signature change only).

**Decision:** this breaks the shipped `str` return of `emit_rapid`. Either do it
now (OmniCore is the forcing function and the unification is correct), or log it
for a single "emitters API 1.0" pass. Not started — needs a call.

### 3. v2 roadmap (confirmed by the options above — not v1)

- **Program splitting** (`MAX_LINES_X_PROG` + `INCLUDE_SUB_PROGRAMS` +
  `MAX_SUBPROG_LINES`): page a long program into multiple module files with a
  synthesized main that calls each. Cross-cutting — belongs in `core` (the
  deferred `core/paging.py`), not per-brand. Likely needs the IR to preserve
  `CompositeInstruction` hierarchy (sub-programs) instead of the current flat
  event stream.
- **External axes** (`TURNTABLE_IGNORE`, `MECHANICAL_UNIT_NAME`, `AXES_RATIO`,
  `EXTAXES_IDX`): an `ExternalAxesSpec` on the profile; per-brand slot rendering.
  `ExternalAxis` already stubs the RAPID literal side.
- **OmniCore `.modx` target**: trivial once #2 lands (just `file_ext="modx"`);
  verify any OmniCore RAPID dialect deltas vs IRC5 against the private oracle.

### Non-goals

Program upload (RWS/FTP), `.urp`-style binary wrappers, and anything requiring
RoboDK's compiled-post internals. We emit files; transport is the caller's.
