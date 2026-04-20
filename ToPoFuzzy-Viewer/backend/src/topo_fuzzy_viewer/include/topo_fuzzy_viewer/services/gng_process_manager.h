#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <map>
#include <unistd.h>
#include <csignal>

namespace ros2_bridge {

/**
 * @brief GNGプロセスのライフサイクル管理
 * 
 * `ros2 run ais_gng ais_gng_node --ros-args -p param:=value ...`コマンドで
 * GNGプロセスを起動・停止する。
 */
class GngProcessManager {
public:
    struct GngStatus {
        bool isRunning = false;
        pid_t pid = -1;
        std::string inputTopic;
    };

    struct GngParams {
        std::string inputTopic = "scan";
        std::string configFile;  // Path to config YAML file (optional)
        int maxNodes = 20000;
        int learningNum = 4000;
        double voxelGridUnit = 0.1;
        // 追加パラメータはmapで渡す
        std::map<std::string, std::string> extraParams;
    };

    struct ConfigFileInfo {
        std::string path;
        std::string name;
    };

    GngProcessManager();
    ~GngProcessManager();

    /**
     * @brief Configディレクトリを設定
     */
    void setConfigDir(const std::string& dir) { configDir_ = dir; }

    /**
     * @brief Project Rootディレクトリを設定
     */
    void setProjectRoot(const std::string& dir) { projectRoot_ = dir; }

    /**
     * @brief 利用可能なconfigファイル一覧を取得
     */
    std::vector<ConfigFileInfo> listConfigFiles() const;

    /**
     * @brief GNGプロセスを起動
     * @param params GNGのパラメータ
     * @return 起動成功時true
     */
    bool startGng(const GngParams& params);

    /**
     * @brief GNGプロセスを停止
     * @return 停止成功時true
     */
    bool stopGng();

    /**
     * @brief 現在の状態を取得
     */
    GngStatus getStatus() const;

    /**
     * @brief プロセスがまだ生きているか確認
     */
    bool isProcessAlive() const;

private:
    std::string buildCommandArgs(const GngParams& params) const;
    void cleanupChildProcess();

    mutable std::mutex mutex_;
    std::atomic<pid_t> childPid_{-1};
    std::string currentInputTopic_;
    std::string configDir_;
    std::string projectRoot_;
};

} // namespace ros2_bridge
