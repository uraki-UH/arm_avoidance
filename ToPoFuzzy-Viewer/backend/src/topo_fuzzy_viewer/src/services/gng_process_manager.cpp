#include "topo_fuzzy_viewer/services/gng_process_manager.h"
#include <iostream>
#include <sstream>
#include <sys/wait.h>
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;
namespace ros2_bridge {

GngProcessManager::GngProcessManager() {
    // Register cleanup on destruction
}

GngProcessManager::~GngProcessManager() {
    stopGng();
}

std::string GngProcessManager::buildCommandArgs(const GngParams& params) const {
    std::ostringstream oss;
    
    // If config file is specified, use --params-file first
    if (!params.configFile.empty() && fs::exists(params.configFile)) {
        oss << " --params-file " << params.configFile;
    }
    
    // Base parameters (these override config file values)
    oss << " -p input.topic_name:=" << params.inputTopic;
    oss << " -p node.num_max:=" << params.maxNodes;
    oss << " -p node.learning_num:=" << params.learningNum;
    oss << " -p input.voxel_grid_unit:=" << params.voxelGridUnit;
    
    // Extra parameters
    for (const auto& [key, value] : params.extraParams) {
        oss << " -p " << key << ":=" << value;
    }
    
    return oss.str();
}

std::vector<GngProcessManager::ConfigFileInfo> GngProcessManager::listConfigFiles() const {
    std::vector<ConfigFileInfo> files;
    
    if (configDir_.empty() || !fs::exists(configDir_)) {
        return files;
    }
    
    try {
        for (const auto& entry : fs::directory_iterator(configDir_)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                if (ext == ".yaml" || ext == ".yml") {
                    ConfigFileInfo info;
                    info.path = entry.path().string();
                    info.name = entry.path().filename().string();
                    files.push_back(info);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[GngProcessManager] Error listing config files: " << e.what() << std::endl;
    }
    
    return files;
}

bool GngProcessManager::startGng(const GngParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Stop existing process if running
    if (childPid_ > 0 && isProcessAlive()) {
        std::cout << "[GngProcessManager] Stopping existing GNG process (PID: " << childPid_ << ")" << std::endl;
        // Kill entire process group
        killpg(childPid_, SIGTERM);
        usleep(500000); // 500ms
        killpg(childPid_, SIGKILL);
        int status;
        waitpid(childPid_, &status, 0);
        childPid_ = -1;
    }
    
    std::string args = buildCommandArgs(params);
    std::cout << "[GngProcessManager] Starting GNG with args:" << args << std::endl;
    
    pid_t pid = fork();
    
    if (pid < 0) {
        std::cerr << "[GngProcessManager] Failed to fork: " << strerror(errno) << std::endl;
        return false;
    }
    
    if (pid == 0) {
        // Child process
        // Create new process group with this process as leader
        setpgid(0, 0);
        
        // Source workspace before running GNG
        std::string sourceCmd;
        if (!projectRoot_.empty()) {
             // projectRoot_ already points at the backend workspace root.
             sourceCmd = "source " + projectRoot_ + "/install/setup.bash && ";
        }

        std::string fullCmd = sourceCmd + "ros2 run ais_gng ais_gng_node --ros-args" + args;
        
        // Execute via shell
        execl("/bin/bash", "bash", "-c", fullCmd.c_str(), nullptr);
        
        // If execl fails
        std::cerr << "[GngProcessManager] Failed to exec: " << strerror(errno) << std::endl;
        exit(1);
    }
    
    // Parent process
    childPid_ = pid;
    currentInputTopic_ = params.inputTopic;
    
    // Also set process group from parent side (race condition protection)
    setpgid(pid, pid);
    
    std::cout << "[GngProcessManager] GNG process started with PID: " << pid << " (process group)" << std::endl;
    return true;
}

bool GngProcessManager::stopGng() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (childPid_ <= 0) {
        return true; // Already stopped
    }
    
    std::cout << "[GngProcessManager] Stopping GNG process group (PGID: " << childPid_ << ")" << std::endl;
    
    // Send SIGTERM to entire process group for graceful shutdown
    if (killpg(childPid_, SIGTERM) != 0) {
        if (errno == ESRCH) {
            // Process group already dead
            childPid_ = -1;
            currentInputTopic_.clear();
            return true;
        }
        std::cerr << "[GngProcessManager] Failed to send SIGTERM to group: " << strerror(errno) << std::endl;
    }
    
    // Wait for process to terminate
    int status;
    pid_t result = waitpid(childPid_, &status, WNOHANG);
    
    if (result == 0) {
        // Process still running, wait up to 2 seconds
        for (int i = 0; i < 20; ++i) {
            usleep(100000); // 100ms
            result = waitpid(childPid_, &status, WNOHANG);
            if (result != 0) break;
        }
        
        if (result == 0) {
            // Force kill entire process group
            std::cout << "[GngProcessManager] Process group not responding, forcing kill" << std::endl;
            killpg(childPid_, SIGKILL);
            waitpid(childPid_, &status, 0);
        }
    }
    
    childPid_ = -1;
    currentInputTopic_.clear();
    std::cout << "[GngProcessManager] GNG process stopped" << std::endl;
    return true;
}

GngProcessManager::GngStatus GngProcessManager::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    GngStatus status;
    status.pid = childPid_;
    status.isRunning = (childPid_ > 0) && isProcessAlive();
    status.inputTopic = currentInputTopic_;
    return status;
}

bool GngProcessManager::isProcessAlive() const {
    if (childPid_ <= 0) return false;
    
    // Check if process group exists
    if (killpg(childPid_, 0) == 0) {
        return true;
    }
    
    // Process group doesn't exist or we don't have permission
    return false;
}

void GngProcessManager::cleanupChildProcess() {
    if (childPid_ > 0) {
        // Non-blocking wait to reap zombie processes
        int status;
        waitpid(childPid_, &status, WNOHANG);
    }
}

} // namespace ros2_bridge
