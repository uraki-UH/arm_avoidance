#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <filesystem>
#include <optional>

namespace ros2_bridge {

struct RosbagInfo {
    std::string path; // Absolute path
    std::string name; // Folder name
    std::string relativePath; // Path relative to root
};

struct PlaybackStatus {
    bool isPlaying;
    std::string currentBag;
    int pid;
};

class RosbagManager {
public:
    RosbagManager(const std::string& rootDir);
    ~RosbagManager();

    std::vector<RosbagInfo> scanBags();
    bool playBag(const std::string& relativePath, const std::vector<std::string>& remaps, bool loop);
    bool stopBag();
    PlaybackStatus getStatus();

private:
    std::string rootDir_;
    
    // Playback state
    std::mutex mutex_;
    bool isPlaying_ = false;
    std::string currentBag_;
    int currentPid_ = -1;
    
    // Helper to find bag executable or use system call
    void monitorProcess();
};

} // namespace ros2_bridge
