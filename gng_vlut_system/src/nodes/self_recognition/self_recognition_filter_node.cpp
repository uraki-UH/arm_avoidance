#include "gng_vlut_system/self_recognition/self_recognition_filter_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "common/voxel_utils.hpp"
#include "common/constants.hpp"

namespace robot_sim::self_recognition {

SelfRecognitionFilterNode::SelfRecognitionFilterNode(const rclcpp::NodeOptions & options)
: Node("self_recognition_filter_node", options) {
    
    // パラメータ
    declare_parameter<double>("robot.voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<double>("voxel_size", ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<std::string>("input_topic", "/points");
    declare_parameter<std::string>("output_topic", "/self_filtered_points");
    declare_parameter<std::string>("self_output_topic", "/self_recognition_points");
    declare_parameter<std::string>("mask_topic", "/self_recognition/voxel_mask");
    declare_parameter<double>("self_recognition.voxel_size", 0.01);
    declare_parameter<std::string>("self_recognition.input_topic", "");
    declare_parameter<std::string>("self_recognition.output_topic", "");
    declare_parameter<std::string>("self_recognition.self_output_topic", "");
    declare_parameter<std::string>("self_recognition.mask_topic", "");

    auto get_string_with_fallback = [this](const std::string &nested_key,
                                           const std::string &legacy_key) {
        const std::string nested = get_parameter(nested_key).as_string();
        if (!nested.empty()) {
            return nested;
        }
        return get_parameter(legacy_key).as_string();
    };

    auto get_double_with_fallback = [this](const std::string &nested_key,
                                           const std::string &legacy_key) {
        const double nested = get_parameter(nested_key).as_double();
        if (nested > 0.0) {
            return nested;
        }
        return get_parameter(legacy_key).as_double();
    };

    grid_.setVoxelSize(get_double_with_fallback("self_recognition.voxel_size", "robot.voxel_size"));
    if (grid_.getVoxelSize() <= 0.0) {
        grid_.setVoxelSize(get_double_with_fallback("self_recognition.voxel_size", "voxel_size"));
    }
    std::string input_topic = get_string_with_fallback("self_recognition.input_topic", "input_topic");
    std::string output_topic = get_string_with_fallback("self_recognition.output_topic", "output_topic");
    std::string self_output_topic = get_string_with_fallback("self_recognition.self_output_topic", "self_output_topic");
    std::string mask_topic = get_string_with_fallback("self_recognition.mask_topic", "mask_topic");

    // マスク（他ノードが計算したもの）を受け取る
    mask_sub_ = create_subscription<voxel_msgs::msg::Voxel>(
        mask_topic, 10, [this](const voxel_msgs::msg::Voxel::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mask_mutex_);
            current_mask_vids_.clear();
            for (auto vid : msg->data) current_mask_vids_.insert(vid);
            // メッセージに含まれるパラメータを反映（動的な変更に対応）
            grid_.setVoxelSize(msg->voxel_size);
            grid_.setIndexingParams(msg->x_shift, msg->y_shift, msg->z_shift, msg->offset);
        });

    // メインの点群処理
    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&SelfRecognitionFilterNode::pcl_cb, this, std::placeholders::_1));

    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    self_pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(self_output_topic, 10);

    RCLCPP_INFO(
        get_logger(),
        "SelfRecognitionFilterNode initialized. Mask topic: %s, filtered output: %s, self output: %s",
        mask_topic.c_str(),
        output_topic.c_str(),
        self_output_topic.c_str());
}

sensor_msgs::msg::PointCloud2 SelfRecognitionFilterNode::makePointCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::vector<Eigen::Vector3f> & points) const
{
    sensor_msgs::msg::PointCloud2 out_msg;
    out_msg.header = input.header;

    sensor_msgs::PointCloud2Modifier modifier(out_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(out_msg, "x"), out_y(out_msg, "y"), out_z(out_msg, "z");
    for (const auto & p : points) {
        *out_x = p.x();
        *out_y = p.y();
        *out_z = p.z();
        ++out_x;
        ++out_y;
        ++out_z;
    }

    return out_msg;
}

void SelfRecognitionFilterNode::pcl_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::unordered_set<long> local_mask;
    {
        std::lock_guard<std::mutex> lock(mask_mutex_);
        if (current_mask_vids_.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "No mask received yet. Passing through raw point cloud (%ux%u)", msg->width, msg->height);
            pcl_pub_->publish(*msg);
            self_pcl_pub_->publish(makePointCloud(*msg, {}));
            return;
        }
        local_mask = current_mask_vids_;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
        "Filtering point cloud (%ux%u) using mask with %zu voxels", msg->width, msg->height, local_mask.size());

    std::vector<Eigen::Vector3f> filtered_points;
    std::vector<Eigen::Vector3f> self_points;
    filtered_points.reserve(msg->width * msg->height);
    self_points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Eigen::Vector3f p(*iter_x, *iter_y, *iter_z);
        Eigen::Vector3i idx = ::common::geometry::VoxelUtils::worldToVoxel(p, (float)grid_.getVoxelSize());
        
        // インスタンス経由でID計算
        long vid = grid_.getFlatVoxelId(idx);

        if (local_mask.find(vid) == local_mask.end()) {
            filtered_points.push_back(p);
        } else {
            self_points.push_back(p);
        }
    }

    pcl_pub_->publish(makePointCloud(*msg, filtered_points));
    self_pcl_pub_->publish(makePointCloud(*msg, self_points));
}

} // namespace robot_sim::self_recognition

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::SelfRecognitionFilterNode)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_sim::self_recognition::SelfRecognitionFilterNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
