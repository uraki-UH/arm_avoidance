#include "gng_vlut_system/self_recognition/self_recognition_filter_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

    declare_parameter("robot_urdf_path", default_urdf);
    declare_parameter("voxel_size", 0.02);
    declare_parameter("input_topic", "/camera/transformed_points");
    declare_parameter("output_topic", "/camera/self_filtered_points");

    std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    double voxel_size = get_parameter("voxel_size").as_double();
    std::string input_topic = get_parameter("input_topic").as_string();
    std::string output_topic = get_parameter("output_topic").as_string();

    voxel_size_inv_ = 1.0f / static_cast<float>(voxel_size);

    // ロボットモデルの初期化
    auto model = std::make_shared<simulation::RobotModel>(simulation::loadRobotFromUrdf(urdf_path));
    auto chain = std::make_shared<kinematics::KinematicChain>(simulation::createKinematicChainFromModel(*model));

    recognition_manager_ = std::make_unique<robot_sim::recognition::SelfRecognitionManager>();
    recognition_manager_->initialize(*model, chain, voxel_size);

    // Sub/Pub
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SelfRecognitionFilterNode::joint_cb, this, std::placeholders::_1));

    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&SelfRecognitionFilterNode::pcl_cb, this, std::placeholders::_1));

    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(get_logger(), "Self Recognition Filter Node started. Voxel size: %.3f", voxel_size);
}

void SelfRecognitionFilterNode::joint_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_joints_ = msg->position;
    
    // 現在の姿勢に基づいてマスクを更新
    auto vids = recognition_manager_->getSelfVoxelMask(current_joints_);
    current_mask_vids_.clear();
    for (long vid : vids) {
        current_mask_vids_.insert(vid);
    }
}

void SelfRecognitionFilterNode::pcl_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::unordered_set<long> mask_vids;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_mask_vids_.empty()) {
            pcl_pub_->publish(*msg);
            return;
        }
        mask_vids = current_mask_vids_;
    }

    auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    // 出力用バッファ（とりあえず最大サイズで確保し、後でリサイズする）
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(msg->data.size());
    uint32_t kept_points = 0;

    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        // 座標からボクセルインデックスを計算
        Eigen::Vector3i idx(
            static_cast<int>(std::floor(x * voxel_size_inv_)),
            static_cast<int>(std::floor(y * voxel_size_inv_)),
            static_cast<int>(std::floor(z * voxel_size_inv_))
        );
        long vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx);

        // マスクに含まれていなければ残す
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
