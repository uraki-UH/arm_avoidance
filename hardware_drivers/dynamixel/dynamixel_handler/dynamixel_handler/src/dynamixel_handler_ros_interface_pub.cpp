#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_debug.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_error.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_extra.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_gain.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_goal.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_limit.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_status.hpp"

// 角度変換
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
double round4(double val) { return round(val*10000.0)/10000.0; }

template <typename MsgT>
static void publish_if(const rclcpp::PublisherBase::SharedPtr& pub, const MsgT& msg) {
    if ( pub && rclcpp::ok() ) std::static_pointer_cast<rclcpp::Publisher<MsgT>>(pub)->publish(msg);
}

DynamixelStatus DynamixelHandler::BroadcastState_Status(){
    DynamixelStatus msg;
    for (const auto id : id_set_ ) {
        msg.id_list.push_back(id);
        msg.torque.push_back(tq_mode_r_[id]);
        msg.error.push_back(hw_err_r_[id].any());
        msg.ping.push_back(ping_err_[id]==0);
        switch(op_mode_r_[id]) {
            case OPERATING_MODE_PWM:                  msg.mode.push_back(msg.CONTROL_PWM                  ); break;
            case OPERATING_MODE_CURRENT:              msg.mode.push_back(msg.CONTROL_CURRENT              ); break;
            case OPERATING_MODE_VELOCITY:             msg.mode.push_back(msg.CONTROL_VELOCITY             ); break;
            case OPERATING_MODE_POSITION:             msg.mode.push_back(msg.CONTROL_POSITION             ); break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.mode.push_back(msg.CONTROL_EXTENDED_POSITION    ); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.mode.push_back(msg.CONTROL_CURRENT_BASE_POSITION); break;
            default:                                  msg.mode.push_back(""                               ); break;
        }
    }
    publish_if(pub_status_, msg);
    return msg;
}

DynamixelPresent DynamixelHandler::BroadcastState_Present(){
    DynamixelPresent msg;
    for (const auto id : id_set_ ) { const auto& value = present_r_[id];
        msg.id_list.push_back(id);
        if ( do_pub_pre_all_ ) {
            msg.pwm_percent.push_back         (round4(value[PRESENT_PWM          ]    ));
            msg.current_ma.push_back          (round4(value[PRESENT_CURRENT      ]    ));
            msg.velocity_deg_s.push_back      (round4(value[PRESENT_VELOCITY     ]/DEG));
            msg.position_deg.push_back        (round4(value[PRESENT_POSITION     ]/DEG));
            msg.vel_trajectory_deg_s.push_back(round4(value[VELOCITY_TRAJECTORY  ]/DEG));
            msg.pos_trajectory_deg.push_back  (round4(value[POSITION_TRAJECTORY  ]/DEG));
            msg.input_voltage_v.push_back     (round4(value[PRESENT_INPUT_VOLTAGE]    ));
            msg.temperature_degc.push_back    (round4(value[PRESENT_TEMPERATURE  ]    ));

        } else for (auto state : present_indice_read_) switch(state) {
            case PRESENT_PWM:          msg.pwm_percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current_ma.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity_deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position_deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory_deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory_deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERATURE:  msg.temperature_degc.push_back    (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage_v.push_back     (round4(value[state]    )); break;
            default:                                                                                 break;
        }
    }
    publish_if(pub_present_, msg);
    return msg;
}

DynamixelError DynamixelHandler::BroadcastState_Error(){
    DynamixelError msg;
    for (const auto id: id_set_) {
        msg.id_list.push_back(id);
        msg.input_voltage.push_back    (hw_err_r_[id][HARDWARE_ERROR_INPUT_VOLTAGE     ]);
        msg.motor_hall_sensor.push_back(hw_err_r_[id][HARDWARE_ERROR_MOTOR_HALL_SENSOR ]);
        msg.overheating.push_back      (hw_err_r_[id][HARDWARE_ERROR_OVERHEATING       ]);
        msg.motor_encoder.push_back    (hw_err_r_[id][HARDWARE_ERROR_MOTOR_ENCODER     ]);
        msg.electrical_shock.push_back (hw_err_r_[id][HARDWARE_ERROR_ELECTRONICAL_SHOCK]);
        msg.overload.push_back         (hw_err_r_[id][HARDWARE_ERROR_OVERLOAD          ]);
    }
    publish_if(pub_error_, msg);
    return msg;
}

DynamixelExtra DynamixelHandler::BroadcastState_Extra() {
    DynamixelExtra msg;
    for ( const auto id : id_set_ ) {
        msg.id_list.push_back(id);

        switch ( series_[id] ) {
            case SERIES_X:   msg.model.push_back("X");   break;
            case SERIES_P:   msg.model.push_back("P");   break;
            case SERIES_PRO: msg.model.push_back("Pro"); break;
            default:         msg.model.push_back("");    break;
        }
        msg.model_number.push_back    (model_[id]);
        msg.firmware_version.push_back(extra_u8_r_[id][EXTRA_FIRMWARE_VERSION]);
        msg.protocol_type.push_back   (extra_u8_r_[id][EXTRA_PROTOCOL_TYPE   ]);
        msg.shadow_id.push_back       (extra_u8_r_[id][EXTRA_SHADOW_ID       ]);

        const auto extra_dv = bitset<8>(extra_u8_r_[id][EXTRA_DRIVE_MODE]);
        msg.drive_mode.torque_on_by_goal_update.push_back(extra_dv[DRV_MODE_AUTO_ACTIVATE]);
        msg.drive_mode.profile_configuration.push_back(
            extra_dv[DRV_MODE_PROFILE_TIME_BASED] ? msg.drive_mode.TIME_BASED : msg.drive_mode.VELOCITY_BASED);
        msg.drive_mode.reverse_mode.push_back(extra_dv[DRV_MODE_REVERSE_MODE]);

        const auto extra_s = bitset<8>(extra_u8_r_[id][EXTRA_SHUTDOWN]);
        msg.shutdown.overload_error.push_back         (extra_s[SHUTDOWN_OVERLOAD         ]);
        msg.shutdown.electrical_shock_error.push_back (extra_s[SHUTDOWN_ELECTRICAL_SHOCK ]);
        msg.shutdown.motor_encoder_error.push_back    (extra_s[SHUTDOWN_MOTOR_ENCODER    ]);
        msg.shutdown.motor_hall_sensor_error.push_back(extra_s[SHUTDOWN_MOTOR_HALL_SENSOR]);
        msg.shutdown.overheating_error.push_back     (extra_s[SHUTDOWN_OVERHEATING     ]);
        msg.shutdown.input_voltage_error.push_back   (extra_s[SHUTDOWN_INPUT_VOLTAGE   ]);

        const auto extra_rc = bitset<8>(extra_u8_r_[id][EXTRA_RESTORE_CONFIG]);
        msg.restore_configuration.ram_restore.push_back      (extra_rc[STARTUP_CONFIG_RAM_RESTORE]);
        msg.restore_configuration.startup_torque_on.push_back(extra_rc[STARTUP_CONFIG_TORQUE_ON]);

        const auto extra_ms = bitset<8>(extra_u8_r_[id][EXTRA_MOVING_STATUS]);
        switch ( extra_ms[MOVING_STATUS_PROFILE_LOW] + extra_ms[MOVING_STATUS_PROFILE_HIGH] * 2 ) {
            case MOVING_PROFILE_RECTANGULAR: msg.moving_status.velocity_profile.push_back(msg.moving_status.PROFILE_RECTANGULAR); break;
            case MOVING_PROFILE_TRIANGULAR : msg.moving_status.velocity_profile.push_back(msg.moving_status.PROFILE_TRIANGULAR ); break;
            case MOVING_PROFILE_TRAPEZOIDAL: msg.moving_status.velocity_profile.push_back(msg.moving_status.PROFILE_TRAPEZOIDAL); break;
            default:                         msg.moving_status.velocity_profile.push_back(msg.moving_status.PROFILE_NONE       ); break;
        }
        msg.moving_status.following_error.push_back(extra_ms[MOVING_STATUS_FOLLOWING_ERROR]);
        msg.moving_status.profile_ongoing.push_back(extra_ms[MOVING_STATUS_PROFILE_ONGOING]);
        msg.moving_status.in_position.push_back    (extra_ms[MOVING_STATUS_IN_POSITION    ]);
        msg.moving.push_back         (extra_ms[EXTRA_U8_MOVING_STATUS_MOVING_BIT]);

        msg.led.red_percent.push_back  (extra_db_r_[id][EXTRA_LED_RED  ]);
        msg.led.green_percent.push_back(extra_db_r_[id][EXTRA_LED_GREEN]);
        msg.led.blue_percent.push_back (extra_db_r_[id][EXTRA_LED_BLUE ]);

        msg.homing_offset_deg.push_back     (extra_db_r_[id][EXTRA_HOMING_OFFSET    ] / DEG);
        msg.return_delay_time_us.push_back  (extra_db_r_[id][EXTRA_RETURN_DELAY_TIME]);
        msg.moving_threshold_deg_s.push_back(extra_db_r_[id][EXTRA_MOVING_THRESHOLD ] / DEG);
        msg.pwm_slope_percent.push_back     (extra_db_r_[id][EXTRA_PWM_SLOPE        ]);
        msg.realtime_tick_s.push_back       (extra_db_r_[id][EXTRA_REALTIME_TICK    ] / 1000.0);
        msg.bus_watchdog_ms.push_back       (extra_db_r_[id][EXTRA_BUS_WATCHDOG     ] < 0.0 ? -1.0 : extra_db_r_[id][EXTRA_BUS_WATCHDOG]);

        msg.reboot.push_back(false);
    }

    publish_if(pub_extra_, msg);
    return msg;
}

DynamixelLimit DynamixelHandler::BroadcastState_Limit(){
    DynamixelLimit msg;
    for (const auto id : id_set_ ) { const auto& limit = limit_r_[id];
        msg.id_list.push_back(id);
        msg.temperature_limit_degc.push_back   (round4(limit[TEMPERATURE_LIMIT ]    ));
        msg.max_voltage_limit_v.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]    ));
        msg.min_voltage_limit_v.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]    ));
        msg.pwm_limit_percent.push_back        (round4(limit[PWM_LIMIT         ]    ));
        msg.current_limit_ma.push_back         (round4(limit[CURRENT_LIMIT     ]    ));
        msg.acceleration_limit_deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit_deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit_deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit_deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    publish_if(pub_limit_, msg);
    return msg;
}

DynamixelGain DynamixelHandler::BroadcastState_Gain(){
    DynamixelGain msg;
    for ( const auto id : id_set_ ) { const auto& gain = gain_r_[id];
        msg.id_list.push_back(id);
        msg.velocity_i_gain_pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain_pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain_pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain_pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain_pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_2nd_gain_pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_1st_gain_pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    publish_if(pub_gain_, msg);
    return msg;
}

DynamixelGoal DynamixelHandler::BroadcastState_Goal(){
    DynamixelGoal msg;
    for ( auto id : id_set_ ) { const auto& goal = goal_r_[id];
        msg.id_list.push_back(id);
        msg.pwm_percent.push_back       (round4(goal[GOAL_PWM     ]));
        msg.current_ma.push_back        (round4(goal[GOAL_CURRENT ]));
        msg.velocity_deg_s.push_back    (round4(goal[GOAL_VELOCITY]/DEG));
        msg.profile_vel_deg_s.push_back (round4(goal[PROFILE_VEL  ]/DEG));
        msg.profile_acc_deg_ss.push_back(round4(goal[PROFILE_ACC  ]/DEG));
        msg.position_deg.push_back      (round4(goal[GOAL_POSITION]/DEG));
    }
    publish_if(pub_goal_, msg);
    return msg;
}

void DynamixelHandler::BroadcastDebug(const DxlStates& st_msg){
    static DynamixelDebug msg;
    if ( !st_msg.status.id_list.empty() ) msg.status = st_msg.status;
    msg.current_ma.present.clear()    ; msg.current_ma.goal.clear();
    msg.velocity_deg_s.present.clear(); msg.velocity_deg_s.goal.clear();
    msg.position_deg.present.clear()  ; msg.position_deg.goal.clear();
    for (const auto id : msg.status.id_list ) {
        msg.current_ma.present.push_back    (round4(present_r_[id][PRESENT_CURRENT ]));
        msg.current_ma.goal.push_back       (round4(   goal_r_[id][GOAL_CURRENT    ]));
        msg.velocity_deg_s.present.push_back(round4(present_r_[id][PRESENT_VELOCITY]/DEG));
        msg.velocity_deg_s.goal.push_back   (round4(   goal_r_[id][GOAL_VELOCITY   ]/DEG));
        msg.position_deg.present.push_back  (round4(present_r_[id][PRESENT_POSITION]/DEG));
        msg.position_deg.goal.push_back     (round4(   goal_r_[id][GOAL_POSITION   ]/DEG));
    }
    publish_if(pub_debug_, msg);
}

void DynamixelHandler::BroadcastStates(const DxlStates& msg){
    publish_if(pub_dxl_states_, msg);
}
