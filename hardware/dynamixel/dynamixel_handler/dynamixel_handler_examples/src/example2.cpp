#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
using namespace dynamixel_handler_msgs::msg;

#include <map>
#include <chrono>
using namespace std::chrono_literals;

class Example2Node : public rclcpp::Node {
public:
    Example2Node() : Node("example2_node") {
        sub_st_ = this->create_subscription<DxlStates>("dynamixel/states", 10, 
            std::bind(&Example2Node::CallbackDxlStates, this, std::placeholders::_1));
        pub_cmd_ = this->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
        main_loop_timer_ = this->create_wall_timer(1.0s, std::bind(&Example2Node::MainLoop, this));
    }

    void CallbackDxlStates(const DxlStates::SharedPtr msg) {
        for (size_t i = 0; i < msg->status.id_list.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "- servo [%d], torque %s, has %s, ping is %s, mode is %s", 
                msg->status.id_list[i],
                msg->status.torque[i] ? "on" : "off",
                msg->status.error[i] ? "error" : "no error",
                msg->status.ping[i] ? "response" : "no response",
                msg->status.mode[i].c_str()
            ); 
        }

        if(!msg->present.id_list.empty()) updated_time_ = msg->stamp;
        for (size_t i = 0; i < msg->present.id_list.size(); i++) {
            auto id = msg->present.id_list[i];
            dxl_pos_[id] = msg->present.position_deg[i];
            dxl_vel_[id] = msg->present.velocity_deg_s[i];
            dxl_cur_[id] = msg->present.current_ma[i];
        }
    }

    void MainLoop() {
        RCLCPP_INFO(this->get_logger(), "* Present value updated time %f", updated_time_.seconds());

        for (const auto& [id, _] : dxl_pos_) {
            RCLCPP_INFO(this->get_logger(), "* servo [%d], pos %f, vel %f, cur %f", id, dxl_pos_[id], dxl_vel_[id], dxl_cur_[id]);
        }

        auto cmd = DxlCommandsX();
        for (const auto& [id, pos] : dxl_pos_) {
            // トルクをオンに．
            cmd.status.id_list.push_back(id);
            cmd.status.torque.push_back(true);
            // 電流を300mAに制限しつつ， +-45degで往復運動させる．
            auto target = (pos < 0) ? 45 : -45;
            auto& cmd_cpos = cmd.current_base_position_control;
            cmd_cpos.id_list.push_back(id);
            cmd_cpos.current_ma.push_back(300/*mA*/);
            cmd_cpos.position_deg.push_back(target/*deg*/);
        }
        if (!cmd.status.id_list.empty()) pub_cmd_->publish(cmd);
    }

    rclcpp::Time updated_time_;
    std::map<uint8_t, double> dxl_pos_, dxl_vel_, dxl_cur_;
    rclcpp::Subscription<DxlStates>::SharedPtr sub_st_;
    rclcpp::Publisher<DxlCommandsX>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Example2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
