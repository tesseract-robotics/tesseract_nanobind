# Wheel build caching

The wheel workflow keeps three reusable layers:

- **vcpkg binary packages:** platform-specific artifacts seed vcpkg's own binary
  cache. vcpkg decides which packages remain compatible after dependency changes.
- **Installed native stack:** artifacts contain `ws/install` and `vcpkg/installed`.
  Compatible runs bypass vcpkg and colcon entirely. The runner image, compiler,
  Homebrew runtime versions, original install prefix and native build inputs must
  match. Binding-only changes do not invalidate this layer.
- **Compiler results:** ccache covers both colcon and nanobind compilation. Each
  platform starts with a 1 GiB budget. Restored entries are checked by ccache;
  changed source/header/compiler options cause recompilation. Binding PCH is disabled
  when the launcher is `ccache`, because caching PCH would require relaxing macro
  and timestamp checks. Ordinary builds retain PCH. Separate upstream
  and binding statistics appear in the job summary. Results are saved before
  wheel tests, including after compilation failures when setup succeeded.

Native input comparison uses the producer's actual checkout revision, including
GitHub's PR merge commit. Changes to dependencies, overlays, upstream gitlinks,
cache scripts or the wheel workflow cause a native rebuild. Artifact metadata
records the original environment because these CMake installs are not relocatable.
Only tested native builds from the default branch or the same source branch/PR
are eligible. The producing `native-<platform>` job must pass; a failed sibling
platform does not discard its results. Release jobs never consume PR artifacts. API, download and extraction
errors fail the job; an absent compatible artifact takes the source-build path.

A Monday 03:17 UTC build refreshes the native stack, retaining vcpkg binary reuse.
Artifacts expire after 14 days. The manual `refresh-native` input performs the
same refresh. **Scheduled execution starts only once the workflow reaches the
repository's default branch.** Normal cold builds can seed artifacts before then.
This implements the weekly-artifact approach in this repository without requiring
changes to `vcpkg-action`.

The compiler cache budget is provisional: increase it only when the summaries show
capacity pressure. A warm native hit avoids upstream compilation, while binding
changes still need compilation, linking, wheel repair and tests. Shared-header
changes can legitimately invalidate many translation units. Local cache tests do
not establish end-to-end CI timings; compare cold and warm workflow runs before
claiming a measured speedup.

Routine PR and branch builds cover Linux x64/ARM64, Windows x64 and macOS ARM64.
Intel macOS remains enabled on weekly refreshes and release tags; manual builds
can enable `include-intel-macos`. Its fresh-runner wheel test follows the same
selection, so routine runs do not wait for an artifact they did not build.
