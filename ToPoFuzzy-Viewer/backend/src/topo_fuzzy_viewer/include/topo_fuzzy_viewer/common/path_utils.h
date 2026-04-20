#pragma once

#include <filesystem>

namespace viewer_internal {

inline bool looksLikeProjectRoot(const std::filesystem::path& candidate) {
    namespace fs = std::filesystem;
    return fs::exists(candidate / "frontend") &&
           fs::exists(candidate / "backend") &&
           fs::exists(candidate / "data");
}

inline std::filesystem::path climbToProjectRoot(std::filesystem::path start) {
    namespace fs = std::filesystem;

    for (int depth = 0; depth < 12; ++depth) {
        if (start.empty()) {
            break;
        }
        if (looksLikeProjectRoot(start)) {
            return start;
        }
        const fs::path parent = start.parent_path();
        if (parent == start) {
            break;
        }
        start = parent;
    }

    return {};
}

inline std::filesystem::path resolveProjectRootFromExe() {
    namespace fs = std::filesystem;
    try {
        const fs::path exePath = fs::canonical("/proc/self/exe");
        const fs::path fromExe = climbToProjectRoot(exePath.parent_path());
        if (!fromExe.empty()) {
            return fromExe;
        }
    } catch (...) {
        // fallback below
    }

    const fs::path fromCwd = climbToProjectRoot(fs::current_path());
    if (!fromCwd.empty()) {
        return fromCwd;
    }

    return fs::current_path();
}

} // namespace viewer_internal
