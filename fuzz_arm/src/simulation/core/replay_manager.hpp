#pragma once

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "simulation/world/environment_manager.hpp" // for RobotObstacleState

namespace robot_sim {
namespace simulation {

// --- バイナリフォーマットのマジックバイトとバージョン ---
static constexpr uint32_t REPLAY_FILE_MAGIC   = 0x52504C59; // 'RPLY'
static constexpr uint32_t REPLAY_FILE_VERSION = 2; // v2: full state

// --- GNG ノードの動的状態スナップショット ---
struct NodeSnapshot {
    int id = -1;
    bool valid = true;
    bool active = true;
    bool is_colliding = false;
    bool is_danger = false;
    bool is_mainland = true;
    int topology_group_id = -1;
};

// --- 1フレーム分の完全シミュレーション状態 ---
struct LogFrame {
    double timestamp = 0.0;

    // ロボット状態
    Eigen::VectorXf joint_positions;

    // 目標・障害物
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacle_pos = Eigen::Vector3d::Zero();

    // 全体的な危険フラグ (robot safety)
    bool is_danger = false;

    // GNG 全ノードのスナップショット (全状態保存)
    std::vector<NodeSnapshot> node_snapshots;

    // 当時の計画パス (ノードIDの列)
    std::vector<int> path_node_ids;

    // 障害物アームの状態
    std::vector<::simulation::RobotObstacleState> robot_obstacle_states;
};

// ========================================
// バイナリ シリアライズ ヘルパー
// ========================================
namespace detail {

inline void writeBytes(std::ofstream& f, const void* data, size_t n) {
    f.write(reinterpret_cast<const char*>(data), (std::streamsize)n);
}
inline void readBytes(std::ifstream& f, void* data, size_t n) {
    f.read(reinterpret_cast<char*>(data), (std::streamsize)n);
}

template<typename T>
void writeVal(std::ofstream& f, T v) { writeBytes(f, &v, sizeof(T)); }
template<typename T>
void readVal(std::ifstream& f, T& v) { readBytes(f, &v, sizeof(T)); }

inline void writeVecXf(std::ofstream& f, const Eigen::VectorXf& v) {
    uint32_t n = (uint32_t)v.size();
    writeVal(f, n);
    writeBytes(f, v.data(), n * sizeof(float));
}
inline void readVecXf(std::ifstream& f, Eigen::VectorXf& v) {
    uint32_t n; readVal(f, n);
    v.resize(n);
    readBytes(f, v.data(), n * sizeof(float));
}

inline void writeVec3d(std::ofstream& f, const Eigen::Vector3d& v) {
    writeBytes(f, v.data(), 3 * sizeof(double));
}
inline void readVec3d(std::ifstream& f, Eigen::Vector3d& v) {
    readBytes(f, v.data(), 3 * sizeof(double));
}

inline void writeString(std::ofstream& f, const std::string& s) {
    uint32_t n = (uint32_t)s.size();
    writeVal(f, n);
    f.write(s.c_str(), n);
}
inline void readString(std::ifstream& f, std::string& s) {
    uint32_t n; readVal(f, n);
    s.resize(n);
    f.read(&s[0], n);
}

} // namespace detail

class ReplayManager {
public:
    ReplayManager() = default;

    // フレームを追加 (フレームオブジェクトをそのまま受け取る)
    void addFrame(LogFrame frame) {
        if (!is_recording_) return;
        frames_.push_back(std::move(frame));
    }

    // 以前のシンプルな addFrame との互換 API
    void addFrame(double ts, const Eigen::VectorXf& q,
                  const Eigen::Vector3d& target, const Eigen::Vector3d& obs,
                  bool danger) {
        if (!is_recording_) return;
        LogFrame f;
        f.timestamp = ts;
        f.joint_positions = q;
        f.target_pos = target;
        f.obstacle_pos = obs;
        f.is_danger = danger;
        frames_.push_back(std::move(f));
    }

    void clear() {
        frames_.clear();
        current_playback_idx_ = 0;
    }

    bool isEmpty() const { return frames_.empty(); }
    size_t getFrameCount() const { return frames_.size(); }
    const std::vector<LogFrame>& getFrames() const { return frames_; }

    void startRecording() {
        clear();
        is_recording_ = true;
        std::cout << "[Replay] Recording started.\n";
    }

    void stopRecording() {
        is_recording_ = false;
        std::cout << "[Replay] Recording stopped. Captured " << frames_.size() << " frames.\n";
    }

    // ========================================
    // バイナリ保存
    // ========================================
    bool saveToFile(const std::string& filename) {
        std::ofstream f(filename, std::ios::binary);
        if (!f.is_open()) {
            std::cerr << "[Replay] Cannot open file for writing: " << filename << "\n";
            return false;
        }

        detail::writeVal(f, REPLAY_FILE_MAGIC);
        detail::writeVal(f, REPLAY_FILE_VERSION);
        uint32_t nframes = (uint32_t)frames_.size();
        detail::writeVal(f, nframes);

        for (const auto& fr : frames_) {
            // タイムスタンプ・ロボット
            detail::writeVal(f, fr.timestamp);
            detail::writeVecXf(f, fr.joint_positions);
            detail::writeVec3d(f, fr.target_pos);
            detail::writeVec3d(f, fr.obstacle_pos);
            detail::writeVal(f, (uint8_t)(fr.is_danger ? 1 : 0));

            // GNG ノードスナップショット
            uint32_t nn = (uint32_t)fr.node_snapshots.size();
            detail::writeVal(f, nn);
            for (const auto& sn : fr.node_snapshots) {
                detail::writeVal(f, sn.id);
                uint8_t flags = (sn.valid       ? 0x01 : 0)
                              | (sn.active      ? 0x02 : 0)
                              | (sn.is_colliding? 0x04 : 0)
                              | (sn.is_danger   ? 0x08 : 0)
                              | (sn.is_mainland ? 0x10 : 0);
                detail::writeVal(f, flags);
                detail::writeVal(f, sn.topology_group_id);
            }

            // パス
            uint32_t np = (uint32_t)fr.path_node_ids.size();
            detail::writeVal(f, np);
            for (int pid : fr.path_node_ids) detail::writeVal(f, pid);

            // 障害物アーム
            uint32_t nr = (uint32_t)fr.robot_obstacle_states.size();
            detail::writeVal(f, nr);
            for (const auto& rs : fr.robot_obstacle_states) {
                detail::writeString(f, rs.name);
                uint32_t nj = (uint32_t)rs.joint_positions.size();
                detail::writeVal(f, nj);
                for (double jv : rs.joint_positions) detail::writeVal(f, jv);
                detail::writeVec3d(f, rs.base_pos);
            }
        }

        std::cout << "[Replay] Saved " << frames_.size() << " frames to " << filename << "\n";
        return true;
    }

    // ========================================
    // バイナリ読み込み
    // ========================================
    bool loadFromFile(const std::string& filename) {
        std::ifstream f(filename, std::ios::binary);
        if (!f.is_open()) {
            std::cerr << "[Replay] Cannot open file: " << filename << "\n";
            return false;
        }

        uint32_t magic, version;
        detail::readVal(f, magic);
        detail::readVal(f, version);

        if (magic != REPLAY_FILE_MAGIC) {
            std::cerr << "[Replay] Invalid file format.\n";
            return false;
        }
        if (version != REPLAY_FILE_VERSION) {
            std::cerr << "[Replay] Unsupported version: " << version << "\n";
            return false;
        }

        uint32_t nframes;
        detail::readVal(f, nframes);

        frames_.clear();
        frames_.reserve(nframes);

        for (uint32_t fi = 0; fi < nframes; ++fi) {
            LogFrame fr;

            detail::readVal(f, fr.timestamp);
            detail::readVecXf(f, fr.joint_positions);
            detail::readVec3d(f, fr.target_pos);
            detail::readVec3d(f, fr.obstacle_pos);
            uint8_t danger; detail::readVal(f, danger);
            fr.is_danger = (danger != 0);

            uint32_t nn; detail::readVal(f, nn);
            fr.node_snapshots.resize(nn);
            for (uint32_t ni = 0; ni < nn; ++ni) {
                auto& sn = fr.node_snapshots[ni];
                detail::readVal(f, sn.id);
                uint8_t flags; detail::readVal(f, flags);
                sn.valid        = (flags & 0x01) != 0;
                sn.active       = (flags & 0x02) != 0;
                sn.is_colliding = (flags & 0x04) != 0;
                sn.is_danger    = (flags & 0x08) != 0;
                sn.is_mainland  = (flags & 0x10) != 0;
                detail::readVal(f, sn.topology_group_id);
            }

            uint32_t np; detail::readVal(f, np);
            fr.path_node_ids.resize(np);
            for (uint32_t pi = 0; pi < np; ++pi) detail::readVal(f, fr.path_node_ids[pi]);

            uint32_t nr; detail::readVal(f, nr);
            fr.robot_obstacle_states.resize(nr);
            for (uint32_t ri = 0; ri < nr; ++ri) {
                auto& rs = fr.robot_obstacle_states[ri];
                detail::readString(f, rs.name);
                uint32_t nj; detail::readVal(f, nj);
                rs.joint_positions.resize(nj);
                for (uint32_t ji = 0; ji < nj; ++ji) detail::readVal(f, rs.joint_positions[ji]);
                detail::readVec3d(f, rs.base_pos);
            }

            frames_.push_back(std::move(fr));
        }

        current_playback_idx_ = 0;
        std::cout << "[Replay] Loaded " << frames_.size() << " frames from " << filename << "\n";
        return true;
    }

    // CSV保存 (基本情報のみ、デバッグ用)
    void saveToCSV(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return;

        file << "timestamp";
        for (int i = 0; i < (frames_.empty() ? 0 : (int)frames_[0].joint_positions.size()); ++i) {
            file << ",q" << i;
        }
        file << ",target_x,target_y,target_z,obs_x,obs_y,obs_z,is_danger,path_length,node_snapshot_count\n";

        for (const auto& f : frames_) {
            file << f.timestamp;
            for (int i = 0; i < (int)f.joint_positions.size(); ++i) {
                file << "," << f.joint_positions[i];
            }
            file << "," << f.target_pos.x() << "," << f.target_pos.y() << "," << f.target_pos.z();
            file << "," << f.obstacle_pos.x() << "," << f.obstacle_pos.y() << "," << f.obstacle_pos.z();
            file << "," << (f.is_danger ? 1 : 0);
            file << "," << f.path_node_ids.size();
            file << "," << f.node_snapshots.size();
            file << "\n";
        }
        std::cout << "[Replay] CSV saved to " << filename << "\n";
    }

    const LogFrame* getFrameAt(size_t index) const {
        if (index < frames_.size()) return &frames_[index];
        return nullptr;
    }

    const LogFrame* getFrameAt(int index) const {
        if (index >= 0 && (size_t)index < frames_.size()) return &frames_[index];
        return nullptr;
    }

    const LogFrame* getFrameAtTime(double timestamp) const {
        if (frames_.empty()) return nullptr;
        for (const auto& f : frames_) {
            if (f.timestamp >= timestamp) return &f;
        }
        return &frames_.back();
    }

    bool isRecording() const { return is_recording_; }

private:
    std::vector<LogFrame> frames_;
    bool is_recording_ = false;
    size_t current_playback_idx_ = 0;
};

} // namespace simulation
} // namespace robot_sim
