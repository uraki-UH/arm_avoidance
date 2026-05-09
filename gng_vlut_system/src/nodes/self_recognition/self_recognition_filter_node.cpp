#include "gng_vlut_system/self_recognition/self_recognition_filter_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/resource_utils.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "robot_model/kinematic_adapter.hpp"

namespace robot_sim::self_recognition {

SelfRecognitionFilterNode::SelfRecognitionFilterNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_filter_node", options) {
    
    std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    std::string default_urdf = pkg_share + "/urdf/topoarm_description/urdf/topoarm.urdf.xacro";

    // パラメータ宣言（デフォルト値なし：YAML等での指定を必須とする）
    declare_parameter<std::string>("robot_urdf_path");
    declare_parameter<double>("voxel_size");
    declare_parameter<std::string>("input_topic");
    declare_parameter<std::string>("output_topic");
    declare_parameter<std::string>("root_link");
    declare_parameter<std::vector<std::string>>("leaf_links");
    declare_parameter<std::vector<std::string>>("prefixes");

    // パラメータの取得
    std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    double voxel_size = get_parameter("voxel_size").as_double();
    std::string input_topic = get_parameter("input_topic").as_string();
    std::string output_topic = get_parameter("output_topic").as_string();
    
    std::string root_link = get_parameter("root_link").as_string();
    std::vector<std::string> leaf_links = get_parameter("leaf_links").as_string_array();
    std::vector<std::string> prefixes = get_parameter("prefixes").as_string_array();

    voxel_size_inv_ = 1.0f / static_cast<float>(voxel_size);

    // ロボットモデルのロード
    auto model = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));
    
    // ルートリンクの設定
    if (root_link.empty()) {
        root_link = model->getRootLinkName();
    }
    frame_id_ = root_link;

    // 末端リンクの設定（YAMLが空の場合は自動検出を試みるが、警告を出す）
    if (leaf_links.empty()) {
        RCLCPP_WARN(get_logger(), "leaf_links is empty in YAML. Attempting auto-detection...");
        for (const auto &[link_name, link_props] : model->getLinks()) {
            bool is_parent = false;
            for (const auto &[j_name, j_props] : model->getJoints()) {
                if (j_props.parent_link == link_name) {
                    is_parent = true;
                    break;
                }
            }
            if (!is_parent && link_name != model->getRootLinkName()) {
                leaf_links.push_back(link_name);
            }
        }
    }
    
    // プリフィックスの設定
    if (prefixes.empty()) {
        RCLCPP_WARN(get_logger(), "prefixes is empty in YAML. Using default 'armN_'...");
        for (size_t i = 0; i < leaf_links.size(); ++i) {
            prefixes.push_back("arm" + std::to_string(i) + "_");
        }
    }

    if (leaf_links.size() != prefixes.size()) {
        throw std::runtime_error("Size mismatch between leaf_links and prefixes in YAML!");
    }

    RCLCPP_INFO(get_logger(), "--- Self Recognition Configuration ---");
    RCLCPP_INFO(get_logger(), "  Root Link : %s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "  Voxel Size: %.3f m", voxel_size);
    for (size_t i = 0; i < leaf_links.size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Arm [%zu]: Leaf=[%s], Prefix=[%s]", i, leaf_links[i].c_str(), prefixes[i].c_str());
    }
    RCLCPP_INFO(get_logger(), "---------------------------------------");

    // マルチアーム対応の KinematicChain 構築
    auto chain_ptr = simulation::createMultiArmKinematicChainFromModels(*model, leaf_links, prefixes, Eigen::Vector3d::Zero());
    auto chain = std::shared_ptr<kinematics::KinematicChain>(std::move(chain_ptr));

    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    recognition_manager_->initialize(*model, chain, voxel_size);

    // サブスクライバ / パブリッシャ
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SelfRecognitionFilterNode::joint_cb, this, std::placeholders::_1));

    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&SelfRecognitionFilterNode::pcl_cb, this, std::placeholders::_1));

    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    voxel_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/self_recognition/voxels", rclcpp::QoS(10).transient_local());
    aabb_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/self_recognition/aabb", rclcpp::QoS(10).transient_local());

    // 関節リストの準備
    for (size_t i = 0; i < chain->getNumJoints(); ++i) {
        if (chain->getJointDOF(i) > 0) {
            active_joint_names_.push_back(chain->getJointName(i));
        }
    }
    current_joints_.resize(active_joint_names_.size(), 0.0);

    // 10Hzタイマー
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SelfRecognitionFilterNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "Self Recognition Filter Node initialized.");
}

void SelfRecognitionFilterNode::joint_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::unordered_map<std::string, double> incoming_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (i < msg->position.size()) {
            incoming_map[msg->name[i]] = msg->position[i];
        }
    }
    
    for (size_t i = 0; i < active_joint_names_.size(); ++i) {
        if (incoming_map.count(active_joint_names_[i])) {
            current_joints_[i] = incoming_map[active_joint_names_[i]];
        }
    }
    has_received_joints_ = true;
    
    // マスク更新
    auto vids = recognition_manager_->getSelfVoxelMask(current_joints_);
    current_mask_vids_.clear();
    for (long vid : vids) {
        current_mask_vids_.insert(vid);
    }
}

void SelfRecognitionFilterNode::timer_cb() {
    if (!has_received_joints_) return;

    if (voxel_pub_->get_subscription_count() > 0 || aabb_pub_->get_subscription_count() > 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto markers = recognition_manager_->getVisualizationMarkers(current_joints_, frame_id_, this->now().seconds());
        
        if (voxel_pub_->get_subscription_count() > 0) {
            voxel_pub_->publish(markers.voxels);
        }
        if (aabb_pub_->get_subscription_count() > 0) {
            aabb_pub_->publish(markers.aabb);
        }
    }
}

void SelfRecognitionFilterNode::pcl_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::unordered_set<long> mask_vids;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_received_joints_ || current_mask_vids_.empty()) {
            pcl_pub_->publish(*msg);
            return;
        }
        mask_vids = current_mask_vids_;
    }

    auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(msg->data.size());
    uint32_t kept_points = 0;

    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        Eigen::Vector3i idx(
            static_cast<int>(std::floor(x * voxel_size_inv_)),
            static_cast<int>(std::floor(y * voxel_size_inv_)),
            static_cast<int>(std::floor(z * voxel_size_inv_))
        );
        long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);

        if (mask_vids.find(vid) == mask_vids.end()) {
            size_t point_offset = i * msg->point_step;
            const uint8_t* src_ptr = &msg->data[point_offset];
            filtered_data.insert(filtered_data.end(), src_ptr, src_ptr + msg->point_step);
            kept_points++;
        }
    }

    out_msg->data = std::move(filtered_data);
    out_msg->width = kept_points;
    out_msg->row_step = out_msg->width * out_msg->point_step;
    
    pcl_pub_->publish(*out_msg);
}

} // namespace robot_sim::self_recognition

#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::self_recognition::SelfRecognitionFilterNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::SelfRecognitionFilterNode)
