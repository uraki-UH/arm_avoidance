#include "gng_vlut_system/self_recognition/self_recognition_viz_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <chrono>

#include "common/resource_utils.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "common/constants.hpp"

namespace robot_sim::self_recognition {

SelfRecognitionVizNode::SelfRecognitionVizNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_viz_node", options) {
    
    // パラメータ：計算に必要な最小限の設定
    declare_parameter("robot_urdf_path", "package://gng_vlut_system/urdf/topoarm_description/urdf/topoarm_dual.urdf.xacro");
    declare_parameter("joint_topic", "/joint_states");
    declare_parameter("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter("update_hz", 50.0); 
    declare_parameter<std::vector<std::string>>("root_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("leaf_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("exclude_links", std::vector<std::string>{});
    declare_parameter<std::string>("root_link", "");
    declare_parameter("padding", 0.02);
 
    // ボクセル展開パラメータの宣言
    declare_parameter("voxel_indexing.x_shift", ::robot_sim::common::Constants::DEFAULT_X_SHIFT);
    declare_parameter("voxel_indexing.y_shift", ::robot_sim::common::Constants::DEFAULT_Y_SHIFT);
    declare_parameter("voxel_indexing.z_shift", ::robot_sim::common::Constants::DEFAULT_Z_SHIFT);
    declare_parameter("voxel_indexing.offset", ::robot_sim::common::Constants::DEFAULT_OFFSET);

    const std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    const std::string joint_topic = get_parameter("joint_topic").as_string();
    const double voxel_size_param = get_parameter("voxel_size").as_double();

    // モデルとチェインの構築
    model_ = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));
    auto root_links = get_parameter("root_links").as_string_array();
    auto leaf_links = get_parameter("leaf_links").as_string_array();
    std::string global_root_link = get_parameter("root_link").as_string();
    if (global_root_link.empty()) global_root_link = model_->getRootLinkName();

    // 衝突形状を持つ全リンクと、木構造の全末端リンクを抽出
    std::vector<std::string> all_collision_links;
    std::vector<std::string> current_leaf_links = leaf_links;

    for (const auto &[link_name, link_props] : model_->getLinks()) {
        if (!link_props.collisions.empty()) {
            all_collision_links.push_back(link_name);
        }
        
        if (leaf_links.empty()) {
            bool has_child = false;
            for (const auto &[j_name, j_props] : model_->getJoints()) {
                if (j_props.parent_link == link_name) { has_child = true; break; }
            }
            if (!has_child && link_name != global_root_link) {
                current_leaf_links.push_back(link_name);
            }
        }
    }

    std::vector<simulation::ArmConfig> arm_configs;
    for (size_t i = 0; i < current_leaf_links.size(); ++i) {
        simulation::ArmConfig cfg;
        cfg.leaf_link = current_leaf_links[i];
        cfg.prefix = ""; 
        cfg.root_link = (i < root_links.size() && !root_links[i].empty()) ? root_links[i] : global_root_link;
        arm_configs.push_back(cfg);
    }

    auto multi_chain = simulation::createMultiArmKinematicChain(*model_, arm_configs, Eigen::Vector3d::Zero());
    chain_ = std::shared_ptr<kinematics::KinematicChain>(std::move(multi_chain));

    // マネージャーの初期化
    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    
    // グリッド設定の初期化（パラメータから反映）
    auto* grid = recognition_manager_->getIndexGrid();
    grid->setVoxelSize(voxel_size_param);
    grid->setIndexingParams(
        get_parameter("voxel_indexing.x_shift").as_int(),
        get_parameter("voxel_indexing.y_shift").as_int(),
        get_parameter("voxel_indexing.z_shift").as_int(),
        get_parameter("voxel_indexing.offset").as_int());

    double hz = get_parameter("update_hz").as_double();
    double padding = get_parameter("padding").as_double();

    // ボクセル化の実行（全衝突リンクを対象）
    auto voxel_data = ::simulation::RobotVoxelizer::build(*model_, all_collision_links, *grid, {}, padding);
    recognition_manager_->initialize(chain_, model_, voxel_data, voxel_size_param);

    // 通信
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10,
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            chain_->updateJointValuesByName(msg->name, msg->position);
            current_joints_ = chain_->getJointValues();
        });

    mask_pub_ = create_publisher<voxel_msgs::msg::Voxel>(
        "/self_recognition/voxel_mask", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(hz, 0.1))),
        [this]() { this->updateAndPublish(); });

    RCLCPP_INFO(get_logger(), "Voxel Mask Provider started (ID-only mode). Hz: %.1f", hz);
}

void SelfRecognitionVizNode::updateAndPublish() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (current_joints_.empty()) return;

    try {
        // 1. 関節角度の更新と順運動学 (FK)
        auto t_start = this->now();
        recognition_manager_->updateRobotState(current_joints_);
        auto t_fk = (this->now() - t_start).seconds() * 1000.0;

        // 2. ボクセルマスクの生成 (座標変換 + 重複排除)
        auto t_voxel_start = this->now();
        const auto vids = recognition_manager_->getSelfVoxelMask();
        auto t_voxel = (this->now() - t_voxel_start).seconds() * 1000.0;
        
        auto t_total = t_fk + t_voxel;

        // 配信処理
        auto mask_msg = std::make_shared<voxel_msgs::msg::Voxel>();
        mask_msg->header.stamp = get_clock()->now();
        mask_msg->header.frame_id = ::robot_sim::common::Constants::DEFAULT_FOOTPRINT_FRAME; 
        mask_msg->voxel_size = static_cast<float>(recognition_manager_->getVoxelSize());
        
        mask_msg->x_shift = get_parameter("voxel_indexing.x_shift").as_int();
        mask_msg->y_shift = get_parameter("voxel_indexing.y_shift").as_int();
        mask_msg->z_shift = get_parameter("voxel_indexing.z_shift").as_int();
        mask_msg->offset = get_parameter("voxel_indexing.offset").as_int();
        
        recognition_manager_->getIndexGrid()->setIndexingParams(
            mask_msg->x_shift, mask_msg->y_shift, mask_msg->z_shift, mask_msg->offset
        );

        mask_msg->data.assign(vids.begin(), vids.end());
        mask_pub_->publish(*mask_msg);

        // 毎フレームログ出力（学習検討用）
        RCLCPP_INFO(get_logger(), "Voxel Gen: Total %.2f ms | Vids: %zu (Pre: %zu)", 
                    recognition_manager_->getLastCalcTimeMs(), vids.size(), 
                    recognition_manager_->getLastPreUniqueCount());

    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error in updateAndPublish: %s", e.what());
    }
}

} // namespace robot_sim::self_recognition

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::SelfRecognitionVizNode)

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::self_recognition::SelfRecognitionVizNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
