#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <vector>
#include <optional>
#include "topo_fuzzy_viewer/services/parameter_manager.h"
#include "topo_fuzzy_viewer/common/types.h"

namespace ros2_bridge {

/**
 * @brief ROS2パラメータサービス経由でais_gngのパラメータを操作
 * 
 * 注意: SyncParametersClientは内部でノードをスピンするため、
 * 既にexecutorで回されているノードとは別の専用ノードを作成する必要がある
 */
class ROS2ParameterManager : public core::ParameterManager {
public:
    ROS2ParameterManager(const std::string& target_node = "/ais_gng");

    std::optional<core::NodeParameters> getNodeParameters() override;
    bool setParameter(const std::string& param_name, double value) override;
    bool setParameter(const std::string& param_name, const std::string& value) override;

private:
    std::string target_node_;
    std::shared_ptr<rclcpp::Node> param_node_;  // 専用ノード（SyncParametersClient用）
    std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
    std::vector<core::ParameterInfo> managed_params_;
};

} // namespace ros2_bridge
