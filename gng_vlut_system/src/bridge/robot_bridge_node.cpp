#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "bridge/udp_comm.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <mutex>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>

/**
 * @brief Universal Robot Hardware Bridge Node.
 * Connects ROS 2 topics to robot hardware via UDP.
 * Fully configurable via ROS parameters for multi-robot support.
 */
class RobotBridgeNode : public rclcpp::Node {
public:
    RobotBridgeNode() : Node("robot_bridge_node") {
        // --- 1. Declare Parameters ---
        this->declare_parameter<std::string>("robot_ip", "192.168.4.40");
        this->declare_parameter<int>("robot_port", 8888);
        this->declare_parameter<int>("listen_port", 8886);
        this->declare_parameter<double>("update_hz", 50.0);
        this->declare_parameter<std::string>("joint_state_topic", "joint_states");
        this->declare_parameter<std::string>("joint_command_topic", "joint_commands");
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{
            "j1", "j2", "j3", "j4", "j5", "j6"
        });

        // --- 2. Get Parameters ---
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        int robot_port = this->get_parameter("robot_port").as_int();
        int listen_port = this->get_parameter("listen_port").as_int();
        double hz = this->get_parameter("update_hz").as_double();
        std::string js_topic = this->get_parameter("joint_state_topic").as_string();
        std::string jc_topic = this->get_parameter("joint_command_topic").as_string();
        joint_names_ = this->get_parameter("joint_names").as_string_array();

        // --- 3. ROS Interfaces ---
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            jc_topic, 10, std::bind(&RobotBridgeNode::commandCallback, this, std::placeholders::_1));

        // --- 4. Hardware Communication setup ---
        receiver_ = std::make_unique<UdpReceiver>(listen_port);
        sender_ = std::make_unique<UdpSender>(robot_ip, robot_port);

        // --- 5. Start Execution ---
        running_ = true;
        receiver_thread_ = std::thread(&RobotBridgeNode::receiveLoop, this);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            std::bind(&RobotBridgeNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), 
                    "Universal Robot Bridge started.\n"
                    " - IP: %s:%d (Listen: %d)\n"
                    " - State Topic: %s\n"
                    " - Cmd Topic:   %s\n"
                    " - Joints:      %zu", 
                    robot_ip.c_str(), robot_port, listen_port, 
                    js_topic.c_str(), jc_topic.c_str(), joint_names_.size());
    }

    ~RobotBridgeNode() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.detach(); 
        }
    }

private:
    void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        latest_command_ = *msg;
        has_new_command_ = true;
    }

    void timerCallback() {
        std::stringstream ss;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!has_new_command_) {
                // Heartbeat: All zeros
                for (size_t i = 0; i < joint_names_.size(); ++i) ss << "0,";
            } else {
                // Map incoming joint states to the order defined in joint_names_
                std::unordered_map<std::string, double> cmd_map;
                for (size_t i = 0; i < latest_command_.name.size(); ++i) {
                    if (i < latest_command_.position.size()) {
                        cmd_map[latest_command_.name[i]] = latest_command_.position[i];
                    }
                }

                for (const auto& name : joint_names_) {
                    double rad = cmd_map.count(name) ? cmd_map[name] : 0.0;
                    int scaled = static_cast<int>(rad * 180.0 / M_PI * 10.0);
                    ss << scaled << ",";
                }
            }
        }

        std::string packet = ss.str();
        if (sender_ && !packet.empty()) {
            sender_->send(packet);
        }
    }

    void receiveLoop() {
        while (running_ && rclcpp::ok()) {
            std::string packet = receiver_->receive();
            if (!packet.empty()) {
                parseMultilineStatus(packet);
            }
        }
    }

    void parseMultilineStatus(const std::string& msg) {
        std::stringstream ss(msg);
        std::string line;
        
        while (std::getline(ss, line)) {
            if (line.size() < 4) continue;
            
            // Format: agl,val1,val2,val3...
            if (line.substr(0, 4) == "agl,") {
                std::vector<double> angles;
                std::string body = line.substr(4);
                std::stringstream line_ss(body);
                std::string item;
                while (std::getline(line_ss, item, ',')) {
                    if (item.empty()) continue;
                    try {
                        double degree = std::stod(item) / 10.0;
                        angles.push_back(degree * M_PI / 180.0);
                    } catch (...) {}
                }

                if (!angles.empty()) {
                    auto out_msg = sensor_msgs::msg::JointState();
                    out_msg.header.stamp = this->now();
                    out_msg.name = joint_names_;
                    out_msg.position = angles;
                    
                    // Pad if hardware sends fewer joints than expected
                    if (out_msg.position.size() < joint_names_.size()) {
                        out_msg.position.resize(joint_names_.size(), 0.0);
                    }
                    joint_state_pub_->publish(out_msg);
                }
                break; 
            }
        }
    }

    // Members
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
    auto node = std::make_shared<RobotBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
