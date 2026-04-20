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

//１１１
class RobotBridgeNode : public rclcpp::Node {
public:
    RobotBridgeNode() : Node("robot_bridge_node") {
        // Parameters
        this->declare_parameter("robot_ip", "192.168.4.40");
        this->declare_parameter("robot_port", 8888);
        this->declare_parameter("listen_port", 8886);
        this->declare_parameter("update_hz", 50.0);
        this->declare_parameter("joint_state_topic", "/joint_states_real");

        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        int robot_port = this->get_parameter("robot_port").as_int();
        int listen_port = this->get_parameter("listen_port").as_int();
        double hz = this->get_parameter("update_hz").as_double();
        std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();

        // Joint Names (6+1 Slave, 6+1 Master)
        joint_names_ = {
            "slave_j1", "slave_j2", "slave_j3", "slave_j4", "slave_j5", "slave_j6", "slave_gripper",
            "master_j1", "master_j2", "master_j3", "master_j4", "master_j5", "master_j6", "master_gripper"
        };

        // ROS Interfaces
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
        
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10, std::bind(&RobotBridgeNode::commandCallback, this, std::placeholders::_1));

        // Hardware Communication setup
        receiver_ = std::make_unique<UdpReceiver>(listen_port);
        sender_ = std::make_unique<UdpSender>(robot_ip, robot_port);

        // Start Hardware Receiver Thread
        running_ = true;
        receiver_thread_ = std::thread(&RobotBridgeNode::receiveLoop, this);
        
        // Start Timer for Command Transmission (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            std::bind(&RobotBridgeNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Robot Bridge Node started. Target: %s:%d joint_state_topic=%s", 
                    robot_ip.c_str(), robot_port, joint_state_topic.c_str());
    }

    ~RobotBridgeNode() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            // Note: UdpReceiver::receive is blocking. 
            // In standalone node, we could use non-blocking or just let it exit.
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
        std::string packet;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            std::stringstream ss;
            
            // Format: j1,j2,j3,j4,j5,j6,g1,j8,j9,j10,j11,j12,j13,g2,
            // (degree * 10)
            if (!has_new_command_) {
                // Heartbeat / Initial command
                packet = "0,0,0,0,0,0,0,0,0,0,0,0,0,0,";
            } else {
                for (size_t i = 0; i < 14; ++i) {
                    double rad = (i < latest_command_.position.size()) ? latest_command_.position[i] : 0.0;
                    int scaled = static_cast<int>(rad * 180.0 / M_PI * 10.0);
                    ss << scaled << ",";
                }
                packet = ss.str();
            }
        }

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
        
        std::vector<double> angles;
        bool found_agl = false;

        while (std::getline(ss, line)) {
            if (line.size() < 4) continue;
            
            // Check for 'agl,' header
            if (line.substr(0, 4) == "agl,") {
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
                found_agl = true;
                break; // Found primary info
            }
        }

        if (found_agl && !angles.empty()) {
            auto out_msg = sensor_msgs::msg::JointState();
            out_msg.header.stamp = this->now();
            out_msg.name = joint_names_;
            out_msg.position = angles;
            if (out_msg.position.size() < 14) {
                out_msg.position.resize(14, 0.0);
            }
            joint_state_pub_->publish(out_msg);
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
