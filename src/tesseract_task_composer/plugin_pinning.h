// Pinning of task composer plugin libraries for the process lifetime.
//
// A TaskComposerPluginFactory (also the one a TaskComposerServer owns) unloads the plugin
// libraries it loaded when it is destroyed, but what their code made outlives it in Python: nodes
// and executors, and the results of a run (a future's context holds data and node infos whose
// vtables live in the plugin that made them). Python frees those in any order, at exit too.
// - macOS: re-loading an unloaded dylib corrupts static state in OMPL / OSQP / TrajOpt (GH #48).
// - Windows: each DLL has its own copy of every template instantiation, so a result's vtable
//   points into the plugin DLL; freeing it after the DLL was unloaded is an access violation.
// - Linux: not pinned; neither failure occurs there (template instantiations resolve to one copy
//   process-wide).
// Callers snapshot the loaded libraries before a call that may load plugins and pin every library
// loaded since, so unloading them becomes a no-op.
#pragma once

#include <string>
#include <unordered_set>

namespace tesseract_nb {

// Paths of the shared libraries loaded in this process (empty on Linux, which pins nothing).
using LoadedLibraries = std::unordered_set<std::string>;

LoadedLibraries snapshot_loaded_libraries();

// Pin every library loaded since `before` for the process lifetime. A library that cannot be
// pinned logs a console_bridge warning: this runs in a destructor, also while unwinding.
void pin_libraries_loaded_since(const LoadedLibraries& before) noexcept;

}  // namespace tesseract_nb
