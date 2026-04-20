#include "topo_fuzzy_viewer/services/rosbag_manager.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

namespace fs = std::filesystem;

namespace ros2_bridge {

RosbagManager::RosbagManager(const std::string& rootDir) : rootDir_(rootDir) {
    // Ensure root dir exists
    if (!fs::exists(rootDir_)) {
        std::cerr << "Rosbag root directory does not exist: " << rootDir_ << std::endl;
    }
}

RosbagManager::~RosbagManager() {
    stopBag();
}

std::vector<RosbagInfo> RosbagManager::scanBags() {
    std::vector<RosbagInfo> bags;
    if (!fs::exists(rootDir_)) return bags;

    try {
        for (const auto& entry : fs::recursive_directory_iterator(rootDir_)) {
            if (entry.is_directory()) {
                // Check if directory is a rosbag (contains metadata.yaml)
                bool isBag = fs::exists(entry.path() / "metadata.yaml");
                // Or check for mcap files if files are listed? Usually ros2 bag play takes the directory or mcap file.
                // Simpler check: If metadata.yaml exists, it's a bag folder.
                
                if (isBag) {
                    RosbagInfo info;
                    info.path = entry.path().string();
                    info.name = entry.path().filename().string();
                    
                    // Create relative path
                    std::string fullPath = entry.path().string();
                    if (fullPath.find(rootDir_) == 0) {
                        info.relativePath = fullPath.substr(rootDir_.length());
                        if (info.relativePath.front() == '/') {
                            info.relativePath.erase(0, 1);
                        }
                    } else {
                        info.relativePath = info.name;
                    }
                    
                    bags.push_back(info);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error scanning bags: " << e.what() << std::endl;
    }
    
    // Sort by name
    std::sort(bags.begin(), bags.end(), [](const RosbagInfo& a, const RosbagInfo& b) {
        return a.relativePath < b.relativePath;
    });

    return bags;
}

bool RosbagManager::playBag(const std::string& relativePath, const std::vector<std::string>& remaps, bool loop) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (isPlaying_) {
        // Already playing, stop first? Or return false?
        // Let's force stop previous
        kill(currentPid_, SIGINT);
        isPlaying_ = false;
        currentPid_ = -1;
        // Wait a bit?
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    fs::path bagPath = fs::path(rootDir_) / relativePath;
    if (!fs::exists(bagPath)) {
        std::cerr << "Bag path not found: " << bagPath << std::endl;
        return false;
    }

    std::string cmd = "ros2 bag play " + bagPath.string();
    if (loop) {
        cmd += " -l";
    }
    
    // Add remaps
    // Format: --remap topic_from:=topic_to
    for (const auto& remap : remaps) {
        if (!remap.empty()) {
            cmd += " --remap " + remap;
        }
    }
    
    // Run in background & capture PID
    // Using simple popen/backgrounding via shell is tricky to get PID.
    // Better: fork and exec? Or use 'setsid' + shell backgrounding -> but we need to kill it later.
    // Approach: Use fork/execvp directly
    
    pid_t pid = fork();
    if (pid == 0) {
        // Child
        // Construct args
        std::vector<std::string> args;
        args.push_back("ros2");
        args.push_back("bag");
        args.push_back("play");
        args.push_back(bagPath.string());
        if (loop) args.push_back("-l");
        
        for (const auto& remap : remaps) {
            args.push_back("--remap");
            args.push_back(remap);
        }
        
        // Convert to char* array
        std::vector<char*> c_args;
        for (const auto& arg : args) {
            c_args.push_back(const_cast<char*>(arg.c_str()));
        }
        c_args.push_back(nullptr);
        
        // Execute
        // Redirect stdout/stderr to avoid cluttering server logs? Or keep them.
        // setsid(); // Make new session so we can kill easily?
        execvp("ros2", c_args.data());
        
        // If here, failed
        exit(1);
    } else if (pid > 0) {
        // Parent
        isPlaying_ = true;
        currentBag_ = relativePath;
        currentPid_ = pid;
        
        // Spawn a thread to wait for it? 
        // If we wait, we block. We want non-blocking check.
        // For simple ViewerServer, we can just rely on manual Stop, 
        // or check waitpid(pid, &status, WNOHANG) in getStatus or a tick loop.
        
        // Start a detach thread to monitor
        std::thread([this, pid]() {
            int status;
            waitpid(pid, &status, 0); // Wait until finishes
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (currentPid_ == pid) {
                    isPlaying_ = false;
                    currentPid_ = -1;
                    currentBag_ = "";
                }
            }
        }).detach();
        
        return true;
    } else {
        std::cerr << "Failed to fork process for ros play" << std::endl;
        return false;
    }
}

bool RosbagManager::stopBag() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isPlaying_ || currentPid_ == -1) return false;
    
    // Send SIGINT (Ctrl+C)
    kill(currentPid_, SIGINT);
    // The monitor thread will clean up state
    return true;
}

PlaybackStatus RosbagManager::getStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    return { isPlaying_, currentBag_, currentPid_ };
}

} // namespace ros2_bridge
