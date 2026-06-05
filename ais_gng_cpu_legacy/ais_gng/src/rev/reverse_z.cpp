#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class ReverseZPointCloud : public rclcpp::Node {
public:
    ReverseZPointCloud() : Node("reverse_z_pointcloud") {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "scan");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "scan/reversed_z");

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&ReverseZPointCloud::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ReverseZPointCloud: input='%s', output='%s'",
                    input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto out = *msg;

        if (out.fields.empty()) {
            pub_->publish(out);
            return;
        }

        try {
            sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
            for (; iter_z != iter_z.end(); ++iter_z) {
                *iter_z = -(*iter_z);
            }
        } catch (const std::runtime_error &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "PointCloud2 missing 'z' field: %s", e.what());
        }

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReverseZPointCloud>());
    rclcpp::shutdown();
    return 0;
}
