#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "bridge/udp_comm.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>

class RobotBridgeNode : public rclcpp::Node {
public:
    RobotBridgeNode() : Node("robot_bridge_node") {
        // --- 1. Get Parameters (Declare & Get in one shot) ---
        const std::string robot_ip = declare_parameter<std::string>("robot_ip", "192.168.4.40");
        const int robot_port = declare_parameter<int>("robot_port", 8888);
        const int listen_port = declare_parameter<int>("listen_port", 8886);
        const double hz = declare_parameter<double>("update_hz", 50.0);
        const std::string js_topic = declare_parameter<std::string>("joint_state_topic", "joint_states");
        const std::string jc_topic = declare_parameter<std::string>("joint_command_topic", "joint_commands");
        joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", {"j1", "j2", "j3", "j4", "j5", "j6"});

        // --- 2. ROS Interfaces ---
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
        joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            jc_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(command_mutex_);
                latest_command_ = *msg;
                has_new_command_ = true;
            });

        // --- 3. Hardware Setup ---
        receiver_ = std::make_unique<UdpReceiver>(listen_port);
        sender_ = std::make_unique<UdpSender>(robot_ip, robot_port);
        
        running_ = true;
        receiver_thread_ = std::thread([this]() {
            while (running_ && rclcpp::ok()) {
                std::string packet = receiver_->receive();
                if (!packet.empty()) parseMultilineStatus(packet);
            }
        });
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            std::bind(&RobotBridgeNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Robot Bridge: %s:%d -> %s", robot_ip.c_str(), robot_port, js_topic.c_str());
    }

    ~RobotBridgeNode() {
        running_ = false;
        if (receiver_thread_.joinable()) receiver_thread_.detach(); 
    }

private:
    void timerCallback() {
        std::string packet;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!has_new_command_) {
                for (size_t i = 0; i < joint_names_.size(); ++i) packet += "0,";
            } else {
                std::unordered_map<std::string, double> cmd_map;
                for (size_t i = 0; i < latest_command_.name.size(); ++i) {
                    if (i < latest_command_.position.size()) cmd_map[latest_command_.name[i]] = latest_command_.position[i];
                }
                for (const auto& name : joint_names_) {
                    double rad = cmd_map.count(name) ? cmd_map[name] : 0.0;
                    packet += std::to_string(static_cast<int>(rad * 180.0 / M_PI * 10.0)) + ",";
                }
            }
        }
        if (sender_ && !packet.empty()) sender_->send(packet);
    }

    void parseMultilineStatus(const std::string& msg) {
        std::stringstream ss(msg);
        std::string line;
        while (std::getline(ss, line)) {
            if (line.compare(0, 4, "agl,") != 0) continue;
            
            std::vector<double> angles;
            std::stringstream line_ss(line.substr(4));
            std::string item;
            while (std::getline(line_ss, item, ',')) {
                if (!item.empty()) {
                    try {
                        angles.push_back((std::stod(item) / 10.0) * M_PI / 180.0);
                    } catch (...) {}
                }
            }

            if (!angles.empty()) {
                sensor_msgs::msg::JointState out_msg;
                out_msg.header.stamp = now();
                out_msg.name = joint_names_;
                out_msg.position = angles;
                if (out_msg.position.size() < joint_names_.size()) out_msg.position.resize(joint_names_.size(), 0.0);
                joint_state_pub_->publish(out_msg);
            }
        }
    }

    std::unique_ptr<UdpReceiver> receiver_;
    std::unique_ptr<UdpSender> sender_;
    std::vector<std::string> joint_names_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    std::thread receiver_thread_;
    std::atomic<bool> running_{false};
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex command_mutex_;
    sensor_msgs::msg::JointState latest_command_;
    bool has_new_command_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
