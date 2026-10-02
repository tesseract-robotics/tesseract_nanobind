# Building against upstream main

The `upstream-main` branch tracks the `master` branches of tesseract, trajopt and
tesseract_planning ahead of the next release (0.36), so the bindings catch up
while upstream moves rather than after it tags. See issue #141.

Two loops share the same pinned upstream SHAs:

| Loop | Command | What it builds | Use it for |
|---|---|---|---|
| inner | `pixi run -e upstream build-upstream` | upstream C++ from git checkouts, incrementally, then the bindings | day-to-day binding work |
| outer | `pixi run build-feedstock` | conda packages, through the tesseract-robotics-packaging feedstocks | checking the release packaging; CI |

## Layout

```text
upstream/                         # upstream sources (submodules, pinned SHAs)
├── boost_plugin_loader           # 0.4.5 — tesseract main needs it, the channel has 0.4.3
├── tesseract
├── trajopt
├── tesseract_planning
└── build/                        # persistent ninja build dirs (gitignored)
packaging/                        # feedstocks (submodules, `dev` branches)
├── boost-plugin-loader-feedstock
├── tesseract-robotics-feedstock
├── trajopt-feedstock
├── tesseract-robotics-planning-feedstock
├── descartes-light-feedstock     # not in the chain, see below
├── output/                       # local conda channel the outer loop writes (gitignored)
└── logs/                         # build logs (gitignored)
```

The feedstock submodules point at forks under `jf---`; their `dev` branches go
upstream to tesseract-robotics-packaging as PRs.

descartes_light and opw_kinematics come from the `tesseract-robotics` channel
in both loops: neither has build-relevant changes past the packaged versions
(descartes_light 0.4.10, opw_kinematics 0.5.3).

## Inner loop

```bash
git submodule update --init upstream packaging
pixi run -e upstream build-upstream-cpp   # C++ only
pixi run -e upstream build-upstream       # C++, then the bindings
```

`scripts/build_upstream.sh` builds boost_plugin_loader → tesseract → trajopt
(`trajopt_common`, `trajopt_sco`, `trajopt_ifopt`, `trajopt`, `trajopt_sqp`) →
tesseract_planning and installs each into the `upstream` env's prefix, where the
bindings find them exactly as they find the conda packages on `main`. It mirrors
the feedstocks' `recipe/build.sh`:

- same compilers: `clang_osx-arm64` / `clangxx_osx-arm64` 19 on macOS,
  `gcc_linux-64` / `gxx_linux-64` 14 on Linux, as pinned in the feedstocks'
  `.ci_support`. These activation packages export `CXX`, `CXXFLAGS` and
  `CMAKE_ARGS`; the `c-compiler` 2.x metapackage installs bare clang without
  them.
- same `${CMAKE_ARGS}` and `-D` flags. Outside conda-build `CMAKE_ARGS` carries
  no install prefix, so the script adds `CMAKE_INSTALL_PREFIX=$CONDA_PREFIX`.
- third-party deps: the `upstream` pixi feature lists the union of the
  feedstock recipes' `host` sections, and no tesseract packages.

Each package keeps a ninja build dir under `upstream/build/<name>` and compiles
through ccache; ninja uses every core. Measured on an M1 Max (10 cores):

| Build | Time |
|---|---|
| no-op | 1–2 s |
| one tesseract `.cpp` touched | 2 s (1 translation unit + relink) |

To reconfigure a package (new CMake options, or after a big SHA jump), remove
its build dir: `upstream/build/<name>`. To change one cache option in place, pass
the source dir explicitly — `cmake -B` alone takes the cwd as the source and
refuses:

```bash
pixi run -e upstream cmake -DOPTION=VALUE -S upstream/tesseract -B upstream/build/tesseract
```

The script configures with `CMAKE_BUILD_WITH_INSTALL_RPATH=ON`. Without it, every
repeat install prints `install_name_tool: no LC_RPATH load command` errors on
macOS: CMake's install script re-runs `-delete_rpath <build tree>` on files it
reports up to date, and the first install already removed that rpath. Nothing
runs from the build tree (tests are off), so the build-tree rpath has no use.

## Outer loop

```bash
pixi run build-feedstock
```

Runs each feedstock's `build-locally.py` — the same `.scripts/run_osx_build.sh`
/ `run_docker_build.sh` its CI runs — for the host platform's config
(`FEEDSTOCK_CONFIG`: `osx_arm64_` or `linux_64_`), in dependency order:
boost-plugin-loader → tesseract → trajopt → planning. Every build writes to
`packaging/output`, and rattler-build also reads that directory as a channel,
so each stage links the packages built before it rather than the 0.35 packages
on the published channel:

```text
boost-plugin-loader  0.4.5               output
tesseract-robotics   0.36.0.dev20260929  output
trajopt              0.36.0.dev20260930  output
```

Each package runs its recipe tests. Every run provisions a fresh build env and
compiles from scratch, which is why this is the outer loop, not the dev loop.

The `dev` recipes source a GitHub archive of the pinned SHA and version it
`0.36.0.dev<commit date>`: conda orders `.dev` below the release, so the 0.36.0
package supersedes it.

!!! warning "Apple SDK licence"
    macOS builds need `OSX_SDK_DIR`; conda-smithy downloads the macOS SDK there.
    Setting it means accepting Apple's SDK licence terms. The task sets it to
    `packaging/SDKs`.

### Gotchas

- `OSX_SDK_DIR` must exist before the build starts: `run_osx_build.sh` probes it
  with `mktemp` and aborts with "not writeable" otherwise. The task creates it.
- conda-forge's build setup appends `CONDA_BUILD_SYSROOT` to
  `.ci_support/<config>.yaml` in place. CI checkouts are thrown away; locally a
  second run fails with `Duplicate key "CONDA_BUILD_SYSROOT"`. Every task
  restores `.ci_support` with `git checkout` before building.
- tesseract main fails to compile against boost_plugin_loader 0.4.3
  (`no member named 'acquireLibraryLifetimeTokens'`); hence the 0.4.5 feedstock
  at the head of the chain.
- The `.ci_support` files still pin `console_bridge`, which the `dev` recipes no
  longer use (upstream switched to spdlog). Harmless; a conda-smithy rerender
  refreshes them.

## Binding status

At tesseract `f4cc080`, trajopt `de9e941`, tesseract_planning `4efed5f`, 11
binding modules fail to compile (`ninja -k 0` in `build/upstream-*`; counts are
lower bounds where clang hit its error limit):

| Module | Errors | Cause |
|---|---|---|
| tesseract_collision, tesseract_environment, tesseract_kinematics | ≥19 each | links and joints addressed by ID instead of name |
| tesseract_state_solver | 15 | name → ID API |
| tesseract_scene_graph | 12 | `Joint::parent_link_name` / `child_link_name`, `get*Names` removed |
| tesseract_command_language | 8 | `JointWaypoint` / `StateWaypoint` constructors |
| tesseract_common | 5 | `JointState::joint_names` → `joint_ids` |
| trajopt_ifopt | 2 | |
| tesseract_geometry | 1 | `tesseract/geometry/impl/sdf_mesh.h` removed |
| tesseract_task_composer | 1 | `tesseract/task_composer/task_composer_keys.h` removed |
