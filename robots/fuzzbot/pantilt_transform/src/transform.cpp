#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <functional>
#include <memory>
#include <string>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

class DSVOXEL : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    OnSetParametersCallbackHandle::SharedPtr param_handle_;

    Eigen::Transform<double, 3, Eigen::Isometry> t_;
    double x, y, z, roll, pitch, yaw, voxel;

   public:
    DSVOXEL()
        : Node("transform") {
        param_handle_ = this->add_on_set_parameters_callback(std::bind(&DSVOXEL::param_cb, this, _1));
        this->declare_parameter("linear.x", 0.0);
        this->declare_parameter("linear.y", 0.0);
        this->declare_parameter("linear.z", 0.0);
        this->declare_parameter("angular.roll", 0.0);
        this->declare_parameter("angular.pitch", 0.0);
        this->declare_parameter("angular.yaw", 0.0);
        this->declare_parameter("voxel", 0.00);
        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&DSVOXEL::pcl_cb, this, _1));
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&DSVOXEL::joint_cb, this, _1));
        RCLCPP_INFO(this->get_logger(), "transform");
    }

   private:
    void pcl_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 out = *msg;
        uint32_t step = msg->point_step;
        Eigen::Vector3d p;
        float pp[3];
        uint32_t size = msg->width * msg->height;
        for (uint32_t i = 0; i < size; ++i) {
            const float *f = ((const float *)&out.data[i * step]);
            p[0] = f[2];
            p[1] = -f[0];
            p[2] = -f[1];
            p = t_ * p;
            pp[0] = p[0];
            pp[1] = p[1];
            pp[2] = p[2];
            std::copy((uint8_t *)pp, (uint8_t *)pp + 12, out.data.begin() + i * step);
        }
        if (voxel >= 0.05) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(out, *cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
            voxelGrid.setInputCloud(cloud);
            voxelGrid.setLeafSize(voxel, voxel, voxel);
            voxelGrid.filter(*cloud_filtered);

            sensor_msgs::msg::PointCloud2 sensor_msg;
            pcl::toROSMsg(*cloud_filtered, sensor_msg);
            sensor_msg.header = msg->header;
            sensor_msg.header.frame_id = "map";
            pcl_pub_->publish(sensor_msg);
        }else{
            out.header = msg->header;
            out.header.frame_id = "map";
            pcl_pub_->publish(out);
        }

    }
    rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.reason = "success";
        result.successful = true;
        for (auto &p : params){
            auto name = p.get_name();
            auto value = p.as_double();
            if (name == "linear.x")
                x = value;
            else if (name == "linear.y")
                y = value;
            else if (name == "linear.z")
                z = value;
            else if (name == "angular.roll")
                roll = value;
            else if (name == "angular.pitch")
                pitch = value;
            else if (name == "angular.yaw")
                yaw = value;
            else if (name == "voxel")
                voxel = value;
            RCLCPP_INFO(this->get_logger(), "param: %s %f", name.c_str(), value);
        }
        RCLCPP_INFO(this->get_logger(), "update");
        Translation<double, 3> trans(x, y, z);
        Eigen::Quaternion<double> q = AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        t_ = trans * q;

        return result;
    }
    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if(msg->position.size() == 4){
            pitch = -msg->position[3];
            yaw = msg->position[2];
            Translation<double, 3> trans(x, y, z);
            Eigen::Quaternion<double> q = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitX());
            t_ = trans * q;
        }
        
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DSVOXEL>());
    rclcpp::shutdown();
    return 0;
}