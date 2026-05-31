#include "dynamixel_handler.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_error.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_extra.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_gain.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_goal.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_limit.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_status.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <utility>

template <typename MsgT, typename CallbackT>
rclcpp::SubscriptionBase::SharedPtr CreateGenericSubscriptionWithTypedCallback(
    rclcpp::Node* node,
    const char* topic_name,
    const char* topic_type,
    size_t qos_depth,
    CallbackT&& callback,
    const rclcpp::SubscriptionOptions& options) {
    return node->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(qos_depth),
        [callback = std::forward<CallbackT>(callback)](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
            MsgT msg;
            rclcpp::Serialization<MsgT> serializer;
            serializer.deserialize_message(serialized_msg.get(), &msg);
            callback(msg);
        }, options);
}

void SetupRosInterfacesByCliX(DynamixelHandler* self, const rclcpp::SubscriptionOptions& sub_options) {
    self->sub_ctrl_x_pwm_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXPwm>(
        self, "dynamixel/command/x/pwm_control", "dynamixel_handler_msgs/msg/DynamixelControlXPwm", 4,
        [self](const DynamixelControlXPwm& msg) { self->CallbackCmd_X_Pwm(msg); }, sub_options);
    self->sub_ctrl_x_cur_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXCurrent>(
        self, "dynamixel/command/x/current_control", "dynamixel_handler_msgs/msg/DynamixelControlXCurrent", 4,
        [self](const DynamixelControlXCurrent& msg) { self->CallbackCmd_X_Current(msg); }, sub_options);
    self->sub_ctrl_x_vel_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXVelocity>(
        self, "dynamixel/command/x/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlXVelocity", 4,
        [self](const DynamixelControlXVelocity& msg) { self->CallbackCmd_X_Velocity(msg); }, sub_options);
    self->sub_ctrl_x_pos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXPosition>(
        self, "dynamixel/command/x/position_control", "dynamixel_handler_msgs/msg/DynamixelControlXPosition", 4,
        [self](const DynamixelControlXPosition& msg) { self->CallbackCmd_X_Position(msg); }, sub_options);
    self->sub_ctrl_x_epos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXExtendedPosition>(
        self, "dynamixel/command/x/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlXExtendedPosition", 4,
        [self](const DynamixelControlXExtendedPosition& msg) { self->CallbackCmd_X_ExtendedPosition(msg); }, sub_options);
    self->sub_ctrl_x_cpos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlXCurrentBasePosition>(
        self, "dynamixel/command/x/current_base_position_control", "dynamixel_handler_msgs/msg/DynamixelControlXCurrentBasePosition", 4,
        [self](const DynamixelControlXCurrentBasePosition& msg) { self->CallbackCmd_X_CurrentBasePosition(msg); }, sub_options);
}

void SetupRosInterfacesByCliP(DynamixelHandler* self, const rclcpp::SubscriptionOptions& sub_options) {
    self->sub_ctrl_p_pwm_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlPPwm>(
        self, "dynamixel/command/p/pwm_control", "dynamixel_handler_msgs/msg/DynamixelControlPPwm", 4,
        [self](const DynamixelControlPPwm& msg) { self->CallbackCmd_P_Pwm(msg); }, sub_options);
    self->sub_ctrl_p_cur_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlPCurrent>(
        self, "dynamixel/command/p/current_control", "dynamixel_handler_msgs/msg/DynamixelControlPCurrent", 4,
        [self](const DynamixelControlPCurrent& msg) { self->CallbackCmd_P_Current(msg); }, sub_options);
    self->sub_ctrl_p_vel_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlPVelocity>(
        self, "dynamixel/command/p/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlPVelocity", 4,
        [self](const DynamixelControlPVelocity& msg) { self->CallbackCmd_P_Velocity(msg); }, sub_options);
    self->sub_ctrl_p_pos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlPPosition>(
        self, "dynamixel/command/p/position_control", "dynamixel_handler_msgs/msg/DynamixelControlPPosition", 4,
        [self](const DynamixelControlPPosition& msg) { self->CallbackCmd_P_Position(msg); }, sub_options);
    self->sub_ctrl_p_epos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlPExtendedPosition>(
        self, "dynamixel/command/p/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlPExtendedPosition", 4,
        [self](const DynamixelControlPExtendedPosition& msg) { self->CallbackCmd_P_ExtendedPosition(msg); }, sub_options);
}

void SetupRosInterfacesByCliPro(DynamixelHandler* self, const rclcpp::SubscriptionOptions& sub_options) {
    self->sub_ctrl_pro_cur_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlProCurrent>(
        self, "dynamixel/command/pro/current_control", "dynamixel_handler_msgs/msg/DynamixelControlProCurrent", 4,
        [self](const DynamixelControlProCurrent& msg) { self->CallbackCmd_Pro_Current(msg); }, sub_options);
    self->sub_ctrl_pro_vel_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlProVelocity>(
        self, "dynamixel/command/pro/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlProVelocity", 4,
        [self](const DynamixelControlProVelocity& msg) { self->CallbackCmd_Pro_Velocity(msg); }, sub_options);
    self->sub_ctrl_pro_pos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlProPosition>(
        self, "dynamixel/command/pro/position_control", "dynamixel_handler_msgs/msg/DynamixelControlProPosition", 4,
        [self](const DynamixelControlProPosition& msg) { self->CallbackCmd_Pro_Position(msg); }, sub_options);
    self->sub_ctrl_pro_epos_ = CreateGenericSubscriptionWithTypedCallback<DynamixelControlProExtendedPosition>(
        self, "dynamixel/command/pro/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlProExtendedPosition", 4,
        [self](const DynamixelControlProExtendedPosition& msg) { self->CallbackCmd_Pro_ExtendedPosition(msg); }, sub_options);
}

void DynamixelHandler::SetupRosInterfaces_byCLI() {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cbg_command_;
    if (use_[ "x" ]) SetupRosInterfacesByCliX(this, sub_options);
    if (use_[ "p" ]) SetupRosInterfacesByCliP(this, sub_options);
    if (use_["pro"]) SetupRosInterfacesByCliPro(this, sub_options);

    sub_status_ = CreateGenericSubscriptionWithTypedCallback<DynamixelStatus>(
        this, "dynamixel/command/status", "dynamixel_handler_msgs/msg/DynamixelStatus", 4,
        [this](const DynamixelStatus& msg) { CallbackCmd_Status(msg); }, sub_options);
    sub_goal_ = CreateGenericSubscriptionWithTypedCallback<DynamixelGoal>(
        this, "dynamixel/command/goal"  , "dynamixel_handler_msgs/msg/DynamixelGoal"  , 4,
        [this](const DynamixelGoal&   msg) { CallbackCmd_Goal(msg);   }, sub_options);
    sub_gain_ = CreateGenericSubscriptionWithTypedCallback<DynamixelGain>(
        this, "dynamixel/command/gain"  , "dynamixel_handler_msgs/msg/DynamixelGain"  , 4,
        [this](const DynamixelGain&   msg) { CallbackCmd_Gain(msg);   }, sub_options);
    sub_limit_ = CreateGenericSubscriptionWithTypedCallback<DynamixelLimit>(
        this, "dynamixel/command/limit" , "dynamixel_handler_msgs/msg/DynamixelLimit" , 4,
        [this](const DynamixelLimit&  msg) { CallbackCmd_Limit(msg); }, sub_options);
    sub_extra_ = CreateGenericSubscriptionWithTypedCallback<DynamixelExtra>(
        this, "dynamixel/command/extra" , "dynamixel_handler_msgs/msg/DynamixelExtra" , 4,
        [this](const DynamixelExtra&  msg) { CallbackCmd_Extra(msg); }, sub_options);

    pub_status_  = create_publisher<DynamixelStatus >("dynamixel/state/status" , 4);
    pub_present_ = create_publisher<DynamixelPresent>("dynamixel/state/present", 4);
    pub_goal_    = create_publisher<DynamixelGoal   >("dynamixel/state/goal"   , 4);
    pub_gain_    = create_publisher<DynamixelGain   >("dynamixel/state/gain"   , 4);
    pub_limit_   = create_publisher<DynamixelLimit  >("dynamixel/state/limit"  , 4);
    pub_error_   = create_publisher<DynamixelError  >("dynamixel/state/error"  , 4);
    pub_extra_   = create_publisher<DynamixelExtra  >("dynamixel/state/extra"  , 4);
}
