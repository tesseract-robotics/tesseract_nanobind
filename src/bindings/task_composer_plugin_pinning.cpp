// Pinning of task composer plugin libraries for the process lifetime (see task_composer_plugin_pinning.h).
// Its own translation unit: <windows.h> defines macros (min, max, ERROR, DELETE, ...) that must
// not reach the bindings.
#include "task_composer_plugin_pinning.h"

#include <tesseract/common/logging.h>

#include <cstdio>
#include <mutex>

#if defined(__APPLE__)
#include <dlfcn.h>
#include <mach-o/dyld.h>
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>  // EnumProcessModules (K32EnumProcessModules in kernel32)

#include <filesystem>
#include <vector>
#endif

namespace tesseract_nb {

namespace {

#if defined(__APPLE__) || defined(_WIN32)
// Log a pinned library once: repeated createTaskComposer* calls stay quiet after warmup. stderr
// rather than a tesseract info message, which tesseract's default threshold (ERROR) suppresses.
void report_pinned(const std::string& path) {
    const auto separator = path.find_last_of("/\\");
    const char* basename = separator == std::string::npos ? path.c_str() : path.c_str() + separator + 1;
    std::fprintf(stderr, "[tesseract_nanobind] pinned plugin library %s (GH #48)\n", basename);
}
#endif

#if defined(_WIN32)
// The process's modules; empty after a tesseract warning if they cannot be listed.
std::vector<HMODULE> process_modules() {
    std::vector<HMODULE> modules(256);
    for (;;) {
        DWORD needed = 0;
        const auto capacity = static_cast<DWORD>(modules.size() * sizeof(HMODULE));
        if (!EnumProcessModules(GetCurrentProcess(), modules.data(), capacity, &needed)) {
            TESSERACT_LOG_WARN("tesseract_nanobind: EnumProcessModules failed (error {}); plugin "
                               "libraries loaded by this call are not pinned",
                               GetLastError());
            return {};
        }
        if (needed <= capacity) {
            modules.resize(needed / sizeof(HMODULE));
            return modules;
        }
        modules.resize(needed / sizeof(HMODULE));
    }
}

// The module's path in UTF-8; empty after a tesseract warning if it cannot be read.
std::string module_path(HMODULE module) {
    // The longest path Windows accepts (with the \\?\ prefix), in UTF-16 code units.
    constexpr DWORD MAX_LONG_PATH = 32768;
    std::wstring path(MAX_LONG_PATH, L'\0');
    const DWORD length = GetModuleFileNameW(module, path.data(), MAX_LONG_PATH);
    if (length == 0 || length == MAX_LONG_PATH) {
        TESSERACT_LOG_WARN("tesseract_nanobind: GetModuleFileNameW failed (error {}); a plugin "
                           "library loaded by this call is not pinned",
                           GetLastError());
        return {};
    }
    path.resize(length);
    return std::filesystem::path(path).u8string();
}
#endif

}  // namespace

LoadedLibraries snapshot_loaded_libraries() {
    LoadedLibraries libraries;
#if defined(__APPLE__)
    const uint32_t count = _dyld_image_count();
    libraries.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (const char* name = _dyld_get_image_name(i))
            libraries.emplace(name);
    }
#elif defined(_WIN32)
    for (HMODULE module : process_modules()) {
        std::string path = module_path(module);
        if (!path.empty())
            libraries.insert(std::move(path));
    }
#endif
    return libraries;
}

void pin_libraries_loaded_since(const LoadedLibraries& before) noexcept {
#if defined(__APPLE__) || defined(_WIN32)
    static std::mutex mutex;
    static LoadedLibraries pinned;

    const LoadedLibraries after = snapshot_loaded_libraries();
    std::lock_guard<std::mutex> guard(mutex);
    for (const auto& path : after) {
        if (before.count(path) != 0 || pinned.count(path) != 0)
            continue;
#if defined(__APPLE__)
        // RTLD_NODELETE: a later dlclose leaves the library mapped. The handle is leaked.
        if (::dlopen(path.c_str(), RTLD_LAZY | RTLD_NODELETE) == nullptr) {
            TESSERACT_LOG_WARN("tesseract_nanobind: failed to pin plugin library '{}': {}", path, dlerror());
            continue;
        }
#else
        // GET_MODULE_HANDLE_EX_FLAG_PIN: the module stays loaded until the process ends, whatever
        // FreeLibrary calls follow.
        HMODULE module = nullptr;
        if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN, std::filesystem::u8path(path).c_str(), &module)) {
            TESSERACT_LOG_WARN("tesseract_nanobind: failed to pin plugin library '{}' (error {})", path,
                               GetLastError());
            continue;
        }
#endif
        pinned.insert(path);
        report_pinned(path);
    }
#else
    (void)before;
#endif
}

}  // namespace tesseract_nb
