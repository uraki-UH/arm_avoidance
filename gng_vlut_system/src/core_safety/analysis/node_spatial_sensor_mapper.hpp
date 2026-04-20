#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>

#include "../spatial/ispatial_index.hpp"
#include "../gng/node_status.hpp"

namespace robot_sim {
namespace analysis {

/**
 * @brief NodeSpatialSensorMapper
 * 
 * 把持物体や手先ノードを「空間センサー」として扱うための支援クラス。
 * 特定のノードの手先位置（weight_coord）が障害物に埋まった際、
 * VLUTを介して「他にどのノードが影響を受けるか（使用不能になるか）」をマッピングする。
 * 
 * ROS 2 環境への移植を考慮し、依存関係を ISpatialIndex インターフェースと
 * 基本的な Eigen 型に限定しています。
 */
class NodeSpatialSensorMapper {
public:
    struct Config {
        double proximity_threshold = 0.05; // 同一視する近傍ノードの距離しきい値 (m)
    };

    NodeSpatialSensorMapper(Config config = Config()) : config_(config) {}

    /**
     * @brief 全ノードに対して手先位置での干渉マップを構築する。
     * @tparam T_angle 関節角の型 (Eigen::VectorXf等)
     * @tparam T_coord 座標の型 (Eigen::Vector3f等)
     * @param nodes GNGノードのリスト
     * @param spatial_index 事前構築済みのVLUT (ISpatialIndex)
     */
    template <typename T_angle, typename T_coord>
    void buildMap(const std::vector<GNG::NeuronNode<T_angle, T_coord>>& nodes,
                  std::shared_ptr<ISpatialIndex> spatial_index) {
        
        sensor_to_affected_.clear();
        sensor_to_affected_.resize(nodes.size());

        if (!spatial_index) return;

        for (const auto& node : nodes) {
            if (node.id < 0 || (size_t)node.id >= nodes.size()) continue;

            // 手先位置の取得
            // T_coord が Eigen::Vector3f の場合を想定し double にキャスト
            Eigen::Vector3d pos = node.weight_coord.template cast<double>();

            // VLUTから、その座標（ボクセル）に含まれる全ノードIDを取得
            std::vector<int> affected_ids = spatial_index->getNodesInVoxel(pos);

            for (int aid : affected_ids) {
                // 自分自身はセンサー対象から除外
                if (aid == node.id) continue;
                if (aid < 0 || (size_t)aid >= nodes.size()) continue;

                // 「手先位置が極めて近い奴」を除外フィルタリング
                // これにより、トポロジー的に重複している近傍姿勢をノイズとして除去
                double dist_sq = (nodes[node.id].weight_coord - nodes[aid].weight_coord).squaredNorm();
                if (dist_sq < config_.proximity_threshold * config_.proximity_threshold) {
                    continue;
                }

                sensor_to_affected_[node.id].push_back(aid);
            }
        }
    }

    /**
     * @brief マップデータをバイナリファイルに作成
     */
    bool save(const std::string& filename) const {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs) return false;

        // Header: Magic(4) + Version(4)
        uint32_t magic = 0x4D534E47; // "GNSM" (GNG Sensor Map)
        uint32_t version = 1;
        ofs.write((char*)&magic, sizeof(magic));
        ofs.write((char*)&version, sizeof(version));

        // Content
        uint32_t num_sensors = static_cast<uint32_t>(sensor_to_affected_.size());
        ofs.write((char*)&num_sensors, sizeof(num_sensors));

        for (const auto& list : sensor_to_affected_) {
            uint32_t count = static_cast<uint32_t>(list.size());
            ofs.write((char*)&count, sizeof(count));
            if (count > 0) {
                ofs.write((char*)list.data(), count * sizeof(int));
            }
        }
        return true;
    }

    /**
     * @brief バイナリファイルからマップデータを読み込む
     */
    bool load(const std::string& filename) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs) return false;

        uint32_t magic = 0, version = 0;
        ifs.read((char*)&magic, sizeof(magic));
        ifs.read((char*)&version, sizeof(version));

        if (magic != 0x4D534E47) return false;

        uint32_t num_sensors = 0;
        ifs.read((char*)&num_sensors, sizeof(num_sensors));

        sensor_to_affected_.resize(num_sensors);
        for (uint32_t i = 0; i < num_sensors; ++i) {
            uint32_t count = 0;
            ifs.read((char*)&count, sizeof(count));
            if (count > 0) {
                sensor_to_affected_[i].resize(count);
                ifs.read((char*)sensor_to_affected_[i].data(), count * sizeof(int));
            } else {
                sensor_to_affected_[i].clear();
            }
        }
        return true;
    }

    /**
     * @brief 特定のノードをセンサーとした際に、影響を受ける（連動して衝突とみなすべき）ノード群を取得
     */
    const std::vector<int>& getAffectedNodes(int sensor_node_id) const {
        if (sensor_node_id >= 0 && (size_t)sensor_node_id < sensor_to_affected_.size()) {
            return sensor_to_affected_[sensor_node_id];
        }
        static const std::vector<int> empty;
        return empty;
    }

    /**
     * @brief マップデータのクリア
     */
    void clear() {
        sensor_to_affected_.clear();
    }

    size_t size() const { return sensor_to_affected_.size(); }

private:
    Config config_;
    // sensor_node_id -> list of affected_node_ids
    std::vector<std::vector<int>> sensor_to_affected_;
};

} // namespace analysis
} // namespace robot_sim
