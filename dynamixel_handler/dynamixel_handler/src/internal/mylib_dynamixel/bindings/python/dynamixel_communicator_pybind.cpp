#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>
#include <map>

#include "dynamixel_communicator.h"

namespace py = pybind11;

PYBIND11_MODULE(_mylib_dynamixel, m) {
  m.doc() = "pybind11 bindings for mylib_dynamixel";

  py::enum_<DynamixelSeries>(m, "DynamixelSeries")
      .value("SERIES_UNKNOWN", SERIES_UNKNOWN)
      .value("SERIES_X", SERIES_X)
      .value("SERIES_P", SERIES_P)
      .value("SERIES_PRO", SERIES_PRO)
      .value("SERIES_PRO_PLUS", SERIES_PRO_PLUS)
      .export_values();

  py::enum_<FactoryResetLevel>(m, "FactoryResetLevel")
      .value("FACTORY_RESET_EXCLUDE_ID", FACTORY_RESET_EXCLUDE_ID)
      .value("FACTORY_RESET_EXCLUDE_ID_BAUDRATE", FACTORY_RESET_EXCLUDE_ID_BAUDRATE)
      .value("FACTORY_RESET_ALL", FACTORY_RESET_ALL)
      .export_values();

  py::enum_<DynamixelBaudrateIndex>(m, "DynamixelBaudrateIndex")
      .value("BAUDRATE_INDEX_9600", BAUDRATE_INDEX_9600)
      .value("BAUDRATE_INDEX_57600", BAUDRATE_INDEX_57600)
      .value("BAUDRATE_INDEX_115200", BAUDRATE_INDEX_115200)
      .value("BAUDRATE_INDEX_1M", BAUDRATE_INDEX_1M)
      .value("BAUDRATE_INDEX_2M", BAUDRATE_INDEX_2M)
      .value("BAUDRATE_INDEX_3M", BAUDRATE_INDEX_3M)
      .value("BAUDRATE_INDEX_4M", BAUDRATE_INDEX_4M)
      .value("BAUDRATE_INDEX_4M5", BAUDRATE_INDEX_4M5)
      .value("BAUDRATE_INDEX_6M", BAUDRATE_INDEX_6M)
      .value("BAUDRATE_INDEX_10M5", BAUDRATE_INDEX_10M5)
      .export_values();

  py::enum_<DynamixelOperatingMode>(m, "DynamixelOperatingMode")
      .value("OPERATING_MODE_CURRENT", OPERATING_MODE_CURRENT)
      .value("OPERATING_MODE_VELOCITY", OPERATING_MODE_VELOCITY)
      .value("OPERATING_MODE_POSITION", OPERATING_MODE_POSITION)
      .value("OPERATING_MODE_EXTENDED_POSITION", OPERATING_MODE_EXTENDED_POSITION)
      .value("OPERATING_MODE_CURRENT_BASE_POSITION", OPERATING_MODE_CURRENT_BASE_POSITION)
      .value("OPERATING_MODE_PWM", OPERATING_MODE_PWM)
      .export_values();

  py::enum_<DynamixelExternalPortMode>(m, "DynamixelExternalPortMode")
      .value("EXTERNAL_PORT_MODE_AIN", EXTERNAL_PORT_MODE_AIN)
      .value("EXTERNAL_PORT_MODE_DOUT", EXTERNAL_PORT_MODE_DOUT)
      .value("EXTERNAL_PORT_MODE_DIN_PULLUP", EXTERNAL_PORT_MODE_DIN_PULLUP)
      .value("EXTERNAL_PORT_MODE_DIN_PULLDOWN", EXTERNAL_PORT_MODE_DIN_PULLDOWN)
      .export_values();

  py::enum_<DynamixelTorquePermission>(m, "DynamixelTorquePermission")
      .value("TORQUE_DISABLE", TORQUE_DISABLE)
      .value("TORQUE_ENABLE", TORQUE_ENABLE)
      .export_values();

  py::enum_<DynamixelStatusReturnLevel>(m, "DynamixelStatusReturnLevel")
      .value("STATUS_RETURN_LEVEL_PING_ONLY", STATUS_RETURN_LEVEL_PING_ONLY)
      .value("STATUS_RETURN_LEVEL_READ_ONLY", STATUS_RETURN_LEVEL_READ_ONLY)
      .value("STATUS_RETURN_LEVEL_ALL", STATUS_RETURN_LEVEL_ALL)
      .export_values();

  py::enum_<DynamixelHardwareErrorIndex>(m, "DynamixelHardwareErrorIndex")
      .value("HARDWARE_ERROR_INPUT_VOLTAGE", HARDWARE_ERROR_INPUT_VOLTAGE)
      .value("HARDWARE_ERROR_MOTOR_HALL_SENSOR", HARDWARE_ERROR_MOTOR_HALL_SENSOR)
      .value("HARDWARE_ERROR_OVERHEATING", HARDWARE_ERROR_OVERHEATING)
      .value("HARDWARE_ERROR_MOTOR_ENCODER", HARDWARE_ERROR_MOTOR_ENCODER)
      .value("HARDWARE_ERROR_ELECTRONICAL_SHOCK", HARDWARE_ERROR_ELECTRONICAL_SHOCK)
      .value("HARDWARE_ERROR_OVERLOAD", HARDWARE_ERROR_OVERLOAD)
      .export_values();

  py::enum_<DynamixelDataType>(m, "DynamixelDataType")
      .value("TYPE_INT8", TYPE_INT8)
      .value("TYPE_UINT8", TYPE_UINT8)
      .value("TYPE_INT16", TYPE_INT16)
      .value("TYPE_UINT16", TYPE_UINT16)
      .value("TYPE_INT32", TYPE_INT32)
      .value("TYPE_UINT32", TYPE_UINT32)
      .export_values();

  py::enum_<DynamixelPhysicalUnit>(m, "DynamixelPhysicalUnit")
      .value("NOUNIT", NOUNIT)
      .value("UNIT_POSITION", UNIT_POSITION)
      .value("UNIT_POSITION_OFFSET", UNIT_POSITION_OFFSET)
      .value("UNIT_VELOCITY", UNIT_VELOCITY)
      .value("UNIT_ACCELERATION", UNIT_ACCELERATION)
      .value("UNIT_CURRENT", UNIT_CURRENT)
      .value("UNIT_VOLTAGE", UNIT_VOLTAGE)
      .value("UNIT_TEMPERATURE", UNIT_TEMPERATURE)
      .value("UNIT_PWM", UNIT_PWM)
      .value("UNIT_RETURN_DELAY_TIME", UNIT_RETURN_DELAY_TIME)
      .value("UNIT_BUS_WATCHDOG", UNIT_BUS_WATCHDOG)
      .value("UNIT_REALTIME_TICK", UNIT_REALTIME_TICK)
      .export_values();

  py::class_<DynamixelAddress>(m, "DynamixelAddress")
      .def(py::init<uint16_t, DynamixelDataType, DynamixelPhysicalUnit>(),
           py::arg("address"),
           py::arg("data_type"),
           py::arg("physical_unit") = NOUNIT)
      .def("is_dummy", &DynamixelAddress::is_dummy)
      .def("size", &DynamixelAddress::size)
      .def("address", &DynamixelAddress::address)
      .def("data_type", &DynamixelAddress::data_type)
      .def("physical_unit", &DynamixelAddress::physical_unit)
      .def("make_dummy", &DynamixelAddress::make_dummy)
      .def("val2pulse", &DynamixelAddress::val2pulse, py::arg("val"), py::arg("model_num"))
      .def("pulse2val", &DynamixelAddress::pulse2val, py::arg("pulse"), py::arg("model_num"));

  m.def("dynamixel_series", &dynamixel_series, py::arg("model_num"));
  m.def("series_from_model", &dynamixel_series, py::arg("model_num"));
  m.def("has_external_port", &has_external_port, py::arg("model_num"));
  m.def("has_current_sensor", &has_current_sensor, py::arg("model_num"));

  py::class_<DynamixelCommunicator>(m, "DynamixelCommunicator")
      .def(py::init<>())
      .def(py::init<const char*, int, int>(),
           py::arg("port_name"),
           py::arg("baudrate"),
           py::arg("latency_timer") = 16)
      .def("GetPortHandler", &DynamixelCommunicator::GetPortHandler, py::arg("port_name"))
      .def("set_baudrate", &DynamixelCommunicator::set_baudrate, py::arg("baudrate"))
      .def("set_latency_timer", &DynamixelCommunicator::set_latency_timer, py::arg("latency_timer"))
      .def("set_status_return_level", &DynamixelCommunicator::set_status_return_level, py::arg("level"))
      .def("set_verbose", &DynamixelCommunicator::set_verbose, py::arg("verbose"))
      .def("port_name", &DynamixelCommunicator::port_name)
      .def("timeout_last_read", &DynamixelCommunicator::timeout_last_read)
      .def("comm_error_last_read", &DynamixelCommunicator::comm_error_last_read)
      .def("hardware_error_last_read", &DynamixelCommunicator::hardware_error_last_read)
      .def("hardware_error_id_last_read", &DynamixelCommunicator::hardware_error_id_last_read)
      .def("ping_id_model_map_last_read", &DynamixelCommunicator::ping_id_model_map_last_read)
      .def("dead_time_retry_ping", &DynamixelCommunicator::dead_time_retry_ping)
      .def("wait_time_ping_broadcast", &DynamixelCommunicator::wait_time_ping_broadcast)
      .def("set_retry_config", &DynamixelCommunicator::set_retry_config, py::arg("num_try"), py::arg("msec_interval"))
      .def("OpenPort", &DynamixelCommunicator::OpenPort)
      .def("ClosePort", &DynamixelCommunicator::ClosePort)
      .def("Reboot", &DynamixelCommunicator::Reboot, py::arg("servo_id"))
      .def("FactoryReset", &DynamixelCommunicator::FactoryReset, py::arg("servo_id"), py::arg("level"))
      .def("Ping", &DynamixelCommunicator::Ping, py::arg("servo_id"))
      .def("Ping_broadcast", &DynamixelCommunicator::Ping_broadcast)

      // single parameter operations
      .def("Write", py::overload_cast<DynamixelAddress, uint8_t, int64_t>(&DynamixelCommunicator::Write),
           py::arg("addr"), py::arg("servo_id"), py::arg("value"))
      .def("SyncWrite", py::overload_cast<DynamixelAddress, const std::vector<uint8_t>&, const std::vector<int64_t>&>(&DynamixelCommunicator::SyncWrite),
           py::arg("addr"), py::arg("servo_id_list"), py::arg("value_list"))
      .def("SyncWrite", py::overload_cast<DynamixelAddress, const std::map<uint8_t, int64_t>&>(&DynamixelCommunicator::SyncWrite),
           py::arg("addr"), py::arg("id_value_map"))
      .def("Read", py::overload_cast<DynamixelAddress, uint8_t>(&DynamixelCommunicator::Read),
           py::arg("addr"), py::arg("servo_id"))
      .def("BulkRead", py::overload_cast<const std::map<uint8_t, DynamixelAddress>&>(&DynamixelCommunicator::BulkRead),
           py::arg("id_addr_map"))
      .def("BulkRead_fast", py::overload_cast<const std::map<uint8_t, DynamixelAddress>&>(&DynamixelCommunicator::BulkRead_fast),
           py::arg("id_addr_map"))
      .def("BulkWrite", py::overload_cast<const std::map<uint8_t, DynamixelAddress>&, const std::map<uint8_t, int64_t>&>(&DynamixelCommunicator::BulkWrite),
           py::arg("id_addr_map"), py::arg("id_value_map"))
      .def("BulkWrite", py::overload_cast<const std::vector<DynamixelAddress>&, const std::vector<uint8_t>&, const std::vector<int64_t>&>(&DynamixelCommunicator::BulkWrite),
           py::arg("addr_list"), py::arg("servo_id_list"), py::arg("value_list"))
      .def("SyncRead", py::overload_cast<DynamixelAddress, const std::vector<uint8_t>&>(&DynamixelCommunicator::SyncRead),
           py::arg("addr"), py::arg("servo_id_list"))
      .def("SyncRead_fast", py::overload_cast<DynamixelAddress, const std::vector<uint8_t>&>(&DynamixelCommunicator::SyncRead_fast),
           py::arg("addr"), py::arg("servo_id_list"))

      // multi parameter operations
      .def("Write", py::overload_cast<const std::vector<DynamixelAddress>&, uint8_t, const std::vector<int64_t>&>(&DynamixelCommunicator::Write),
           py::arg("addr_list"), py::arg("servo_id"), py::arg("value_list"))
      .def("SyncWrite", py::overload_cast<const std::vector<DynamixelAddress>&, const std::vector<uint8_t>&, const std::vector<std::vector<int64_t>>&>(&DynamixelCommunicator::SyncWrite),
           py::arg("addr_list"), py::arg("servo_id_list"), py::arg("value_matrix"))
      .def("SyncWrite", py::overload_cast<const std::vector<DynamixelAddress>&, const std::map<uint8_t, std::vector<int64_t>>&>(&DynamixelCommunicator::SyncWrite),
           py::arg("addr_list"), py::arg("id_values_map"))
      .def("Read", py::overload_cast<const std::vector<DynamixelAddress>&, uint8_t>(&DynamixelCommunicator::Read),
           py::arg("addr_list"), py::arg("servo_id"))
      .def("BulkRead", py::overload_cast<const std::map<uint8_t, std::vector<DynamixelAddress>>&>(&DynamixelCommunicator::BulkRead),
           py::arg("id_addr_list_map"))
      .def("BulkRead_fast", py::overload_cast<const std::map<uint8_t, std::vector<DynamixelAddress>>&>(&DynamixelCommunicator::BulkRead_fast),
           py::arg("id_addr_list_map"))
      .def("BulkWrite", py::overload_cast<const std::map<uint8_t, std::vector<DynamixelAddress>>&, const std::map<uint8_t, std::vector<int64_t>>&>(&DynamixelCommunicator::BulkWrite),
           py::arg("id_addr_list_map"), py::arg("id_values_map"))
      .def("BulkWrite", py::overload_cast<const std::vector<std::vector<DynamixelAddress>>&, const std::vector<uint8_t>&, const std::vector<std::vector<int64_t>>&>(&DynamixelCommunicator::BulkWrite),
           py::arg("addr_list_per_servo"), py::arg("servo_id_list"), py::arg("value_matrix"))
      .def("SyncRead", py::overload_cast<const std::vector<DynamixelAddress>&, const std::vector<uint8_t>&>(&DynamixelCommunicator::SyncRead),
           py::arg("addr_list"), py::arg("servo_id_list"))
      .def("SyncRead_fast", py::overload_cast<const std::vector<DynamixelAddress>&, const std::vector<uint8_t>&>(&DynamixelCommunicator::SyncRead_fast),
           py::arg("addr_list"), py::arg("servo_id_list"))

      // retry helpers
      .def("tryPing", &DynamixelCommunicator::tryPing, py::arg("servo_id"))
      .def("tryWrite", py::overload_cast<DynamixelAddress, uint8_t, int64_t>(&DynamixelCommunicator::tryWrite),
           py::arg("addr"), py::arg("servo_id"), py::arg("value"))
      .def("tryRead", py::overload_cast<DynamixelAddress, uint8_t>(&DynamixelCommunicator::tryRead),
           py::arg("addr"), py::arg("servo_id"))
      .def("tryWrite", py::overload_cast<const std::vector<DynamixelAddress>&, uint8_t, const std::vector<int64_t>&>(&DynamixelCommunicator::tryWrite),
           py::arg("addr_list"), py::arg("servo_id"), py::arg("value_list"))
      .def("tryRead", py::overload_cast<const std::vector<DynamixelAddress>&, uint8_t>(&DynamixelCommunicator::tryRead),
           py::arg("addr_list"), py::arg("servo_id"))

      // snake_case aliases for python ergonomics
      .def("set_port_handler", &DynamixelCommunicator::GetPortHandler, py::arg("port_name"))
      .def("open_port", &DynamixelCommunicator::OpenPort)
      .def("close_port", &DynamixelCommunicator::ClosePort)
      .def("try_ping", &DynamixelCommunicator::tryPing, py::arg("servo_id"))
      .def("ping_broadcast", &DynamixelCommunicator::Ping_broadcast)
      .def("try_read_uint16",
           [](DynamixelCommunicator& self, int address, int servo_id) {
             DynamixelAddress dp(static_cast<uint16_t>(address), TYPE_UINT16);
             return static_cast<int>(self.tryRead(dp, static_cast<uint8_t>(servo_id)));
           },
           py::arg("address"), py::arg("servo_id"))
      .def("try_write_uint8",
           [](DynamixelCommunicator& self, int address, int servo_id, int value) {
             DynamixelAddress dp(static_cast<uint16_t>(address), TYPE_UINT8);
             return self.tryWrite(dp, static_cast<uint8_t>(servo_id), static_cast<int64_t>(value));
           },
           py::arg("address"), py::arg("servo_id"), py::arg("value"));

  // address constants
  auto export_addr = [&](const char* name, const DynamixelAddress& addr) {
    m.attr(name) = py::int_(addr.address());
  };

  // AddrCommon
  export_addr("ADDR_COMMON_MODEL_NUMBER", AddrCommon::model_number);
  export_addr("ADDR_COMMON_ID", AddrCommon::id);

  // AddrX
  export_addr("ADDR_X_BAUDRATE", AddrX::baudrate);
  export_addr("ADDR_X_RETURN_DELAY_TIME", AddrX::return_delay_time);
  export_addr("ADDR_X_DRIVE_MODE", AddrX::drive_mode);
  export_addr("ADDR_X_OPERATING_MODE", AddrX::operating_mode);
  export_addr("ADDR_X_SHADOW_ID", AddrX::shadow_id);
  export_addr("ADDR_X_HOMING_OFFSET", AddrX::homing_offset);
  export_addr("ADDR_X_MOVING_THRESHOLD", AddrX::moving_threshold);
  export_addr("ADDR_X_TEMPERATURE_LIMIT", AddrX::temperature_limit);
  export_addr("ADDR_X_MAX_VOLTAGE_LIMIT", AddrX::max_voltage_limit);
  export_addr("ADDR_X_MIN_VOLTAGE_LIMIT", AddrX::min_voltage_limit);
  export_addr("ADDR_X_PWM_LIMIT", AddrX::pwm_limit);
  export_addr("ADDR_X_CURRENT_LIMIT", AddrX::current_limit);
  export_addr("ADDR_X_VELOCITY_LIMIT", AddrX::velocity_limit);
  export_addr("ADDR_X_MAX_POSITION_LIMIT", AddrX::max_position_limit);
  export_addr("ADDR_X_MIN_POSITION_LIMIT", AddrX::min_position_limit);
  export_addr("ADDR_X_EXTERNAL_PORT_MODE_1", AddrX::external_port_mode_1);
  export_addr("ADDR_X_EXTERNAL_PORT_MODE_2", AddrX::external_port_mode_2);
  export_addr("ADDR_X_EXTERNAL_PORT_MODE_3", AddrX::external_port_mode_3);
  export_addr("ADDR_X_SHUTDOWN", AddrX::shutdown);
  export_addr("ADDR_X_TORQUE_ENABLE", AddrX::torque_enable);
  export_addr("ADDR_X_LED", AddrX::led);
  export_addr("ADDR_X_STATUS_RETURN_LEVEL", AddrX::status_return_level);
  export_addr("ADDR_X_REGISTERED_INSTRUCTION", AddrX::registered_instruction);
  export_addr("ADDR_X_HARDWARE_ERROR_STATUS", AddrX::hardware_error_status);
  export_addr("ADDR_X_VELOCITY_I_GAIN", AddrX::velocity_i_gain);
  export_addr("ADDR_X_VELOCITY_P_GAIN", AddrX::velocity_p_gain);
  export_addr("ADDR_X_POSITION_D_GAIN", AddrX::position_d_gain);
  export_addr("ADDR_X_POSITION_I_GAIN", AddrX::position_i_gain);
  export_addr("ADDR_X_POSITION_P_GAIN", AddrX::position_p_gain);
  export_addr("ADDR_X_FEEDFORWARD_2ND_GAIN", AddrX::feedforward_2nd_gain);
  export_addr("ADDR_X_FEEDFORWARD_1ST_GAIN", AddrX::feedforward_1st_gain);
  export_addr("ADDR_X_BUS_WATCHDOG", AddrX::bus_watchdog);
  export_addr("ADDR_X_GOAL_PWM", AddrX::goal_pwm);
  export_addr("ADDR_X_GOAL_CURRENT", AddrX::goal_current);
  export_addr("ADDR_X_GOAL_VELOCITY", AddrX::goal_velocity);
  export_addr("ADDR_X_PROFILE_ACCELERATION", AddrX::profile_acceleration);
  export_addr("ADDR_X_PROFILE_VELOCITY", AddrX::profile_velocity);
  export_addr("ADDR_X_GOAL_POSITION", AddrX::goal_position);
  export_addr("ADDR_X_REALTIME_TICK", AddrX::realtime_tick);
  export_addr("ADDR_X_MOVING", AddrX::moving);
  export_addr("ADDR_X_MOVING_STATUS", AddrX::moving_status);
  export_addr("ADDR_X_PRESENT_PWM", AddrX::present_pwm);
  export_addr("ADDR_X_PRESENT_CURRENT", AddrX::present_current);
  export_addr("ADDR_X_PRESENT_VELOCITY", AddrX::present_velocity);
  export_addr("ADDR_X_PRESENT_POSITION", AddrX::present_position);
  export_addr("ADDR_X_VELOCITY_TRAJECTORY", AddrX::velocity_trajectory);
  export_addr("ADDR_X_POSITION_TRAJECTORY", AddrX::position_trajectory);
  export_addr("ADDR_X_PRESENT_INPUT_VOLTAGE", AddrX::present_input_voltage);
  export_addr("ADDR_X_PRESENT_TEMPERATURE", AddrX::present_temperature);
  export_addr("ADDR_X_EXTERNAL_PORT_DATA_1", AddrX::external_port_data_1);
  export_addr("ADDR_X_EXTERNAL_PORT_DATA_2", AddrX::external_port_data_2);
  export_addr("ADDR_X_EXTERNAL_PORT_DATA_3", AddrX::external_port_data_3);
  export_addr("ADDR_X_ACCELERATION_LIMIT", AddrX::acceleration_limit);
  export_addr("ADDR_X_EXTERNAL_PORT_MODE_4", AddrX::external_port_mode_4);
  export_addr("ADDR_X_EXTERNAL_PORT_DATA_4", AddrX::external_port_data_4);

  // AddrP
  export_addr("ADDR_P_BAUDRATE", AddrP::baudrate);
  export_addr("ADDR_P_RETURN_DELAY_TIME", AddrP::return_delay_time);
  export_addr("ADDR_P_DRIVE_MODE", AddrP::drive_mode);
  export_addr("ADDR_P_OPERATING_MODE", AddrP::operating_mode);
  export_addr("ADDR_P_SHADOW_ID", AddrP::shadow_id);
  export_addr("ADDR_P_HOMING_OFFSET", AddrP::homing_offset);
  export_addr("ADDR_P_MOVING_THRESHOLD", AddrP::moving_threshold);
  export_addr("ADDR_P_TEMPERATURE_LIMIT", AddrP::temperature_limit);
  export_addr("ADDR_P_MAX_VOLTAGE_LIMIT", AddrP::max_voltage_limit);
  export_addr("ADDR_P_MIN_VOLTAGE_LIMIT", AddrP::min_voltage_limit);
  export_addr("ADDR_P_PWM_LIMIT", AddrP::pwm_limit);
  export_addr("ADDR_P_CURRENT_LIMIT", AddrP::current_limit);
  export_addr("ADDR_P_ACCELERATION_LIMIT", AddrP::acceleration_limit);
  export_addr("ADDR_P_VELOCITY_LIMIT", AddrP::velocity_limit);
  export_addr("ADDR_P_MAX_POSITION_LIMIT", AddrP::max_position_limit);
  export_addr("ADDR_P_MIN_POSITION_LIMIT", AddrP::min_position_limit);
  export_addr("ADDR_P_EXTERNAL_PORT_MODE_1", AddrP::external_port_mode_1);
  export_addr("ADDR_P_EXTERNAL_PORT_MODE_2", AddrP::external_port_mode_2);
  export_addr("ADDR_P_EXTERNAL_PORT_MODE_3", AddrP::external_port_mode_3);
  export_addr("ADDR_P_EXTERNAL_PORT_MODE_4", AddrP::external_port_mode_4);
  export_addr("ADDR_P_SHUTDOWN", AddrP::shutdown);
  export_addr("ADDR_P_TORQUE_ENABLE", AddrP::torque_enable);
  export_addr("ADDR_P_LED_RED", AddrP::led_red);
  export_addr("ADDR_P_LED_GREEN", AddrP::led_green);
  export_addr("ADDR_P_LED_BLUE", AddrP::led_blue);
  export_addr("ADDR_P_STATUS_RETURN_LEVEL", AddrP::status_return_level);
  export_addr("ADDR_P_REGISTERED_INSTRUCTION", AddrP::registered_instruction);
  export_addr("ADDR_P_HARDWARE_ERROR_STATUS", AddrP::hardware_error_status);
  export_addr("ADDR_P_VELOCITY_I_GAIN", AddrP::velocity_i_gain);
  export_addr("ADDR_P_VELOCITY_P_GAIN", AddrP::velocity_p_gain);
  export_addr("ADDR_P_POSITION_D_GAIN", AddrP::position_d_gain);
  export_addr("ADDR_P_POSITION_I_GAIN", AddrP::position_i_gain);
  export_addr("ADDR_P_POSITION_P_GAIN", AddrP::position_p_gain);
  export_addr("ADDR_P_FEEDFORWARD_2ND_GAIN", AddrP::feedforward_2nd_gain);
  export_addr("ADDR_P_FEEDFORWARD_1ST_GAIN", AddrP::feedforward_1st_gain);
  export_addr("ADDR_P_BUS_WATCHDOG", AddrP::bus_watchdog);
  export_addr("ADDR_P_GOAL_PWM", AddrP::goal_pwm);
  export_addr("ADDR_P_GOAL_CURRENT", AddrP::goal_current);
  export_addr("ADDR_P_GOAL_VELOCITY", AddrP::goal_velocity);
  export_addr("ADDR_P_PROFILE_ACCELERATION", AddrP::profile_acceleration);
  export_addr("ADDR_P_PROFILE_VELOCITY", AddrP::profile_velocity);
  export_addr("ADDR_P_GOAL_POSITION", AddrP::goal_position);
  export_addr("ADDR_P_REALTIME_TICK", AddrP::realtime_tick);
  export_addr("ADDR_P_MOVING", AddrP::moving);
  export_addr("ADDR_P_MOVING_STATUS", AddrP::moving_status);
  export_addr("ADDR_P_PRESENT_PWM", AddrP::present_pwm);
  export_addr("ADDR_P_PRESENT_CURRENT", AddrP::present_current);
  export_addr("ADDR_P_PRESENT_VELOCITY", AddrP::present_velocity);
  export_addr("ADDR_P_PRESENT_POSITION", AddrP::present_position);
  export_addr("ADDR_P_VELOCITY_TRAJECTORY", AddrP::velocity_trajectory);
  export_addr("ADDR_P_POSITION_TRAJECTORY", AddrP::position_trajectory);
  export_addr("ADDR_P_PRESENT_INPUT_VOLTAGE", AddrP::present_input_voltage);
  export_addr("ADDR_P_PRESENT_TEMPERATURE", AddrP::present_temperature);
  export_addr("ADDR_P_EXTERNAL_PORT_DATA_1", AddrP::external_port_data_1);
  export_addr("ADDR_P_EXTERNAL_PORT_DATA_2", AddrP::external_port_data_2);
  export_addr("ADDR_P_EXTERNAL_PORT_DATA_3", AddrP::external_port_data_3);
  export_addr("ADDR_P_EXTERNAL_PORT_DATA_4", AddrP::external_port_data_4);

  // AddrPro
  export_addr("ADDR_PRO_BAUDRATE", AddrPro::baudrate);
  export_addr("ADDR_PRO_RETURN_DELAY_TIME", AddrPro::return_delay_time);
  export_addr("ADDR_PRO_OPERATING_MODE", AddrPro::operating_mode);
  export_addr("ADDR_PRO_HOMING_OFFSET", AddrPro::homing_offset);
  export_addr("ADDR_PRO_MOVING_THRESHOLD", AddrPro::moving_threshold);
  export_addr("ADDR_PRO_TEMPERATURE_LIMIT", AddrPro::temperature_limit);
  export_addr("ADDR_PRO_MAX_VOLTAGE_LIMIT", AddrPro::max_voltage_limit);
  export_addr("ADDR_PRO_MIN_VOLTAGE_LIMIT", AddrPro::min_voltage_limit);
  export_addr("ADDR_PRO_ACCELERATION_LIMIT", AddrPro::acceleration_limit);
  export_addr("ADDR_PRO_TORQUE_LIMIT", AddrPro::torque_limit);
  export_addr("ADDR_PRO_VELOCITY_LIMIT", AddrPro::velocity_limit);
  export_addr("ADDR_PRO_MAX_POSITION_LIMIT", AddrPro::max_position_limit);
  export_addr("ADDR_PRO_MIN_POSITION_LIMIT", AddrPro::min_position_limit);
  export_addr("ADDR_PRO_EXTERNAL_PORT_MODE_1", AddrPro::external_port_mode_1);
  export_addr("ADDR_PRO_EXTERNAL_PORT_MODE_2", AddrPro::external_port_mode_2);
  export_addr("ADDR_PRO_EXTERNAL_PORT_MODE_3", AddrPro::external_port_mode_3);
  export_addr("ADDR_PRO_EXTERNAL_PORT_MODE_4", AddrPro::external_port_mode_4);
  export_addr("ADDR_PRO_SHUTDOWN", AddrPro::shutdown);
  export_addr("ADDR_PRO_TORQUE_ENABLE", AddrPro::torque_enable);
  export_addr("ADDR_PRO_LED_RED", AddrPro::led_red);
  export_addr("ADDR_PRO_LED_GREEN", AddrPro::led_green);
  export_addr("ADDR_PRO_LED_BLUE", AddrPro::led_blue);
  export_addr("ADDR_PRO_VELOCITY_I_GAIN", AddrPro::velocity_i_gain);
  export_addr("ADDR_PRO_VELOCITY_P_GAIN", AddrPro::velocity_p_gain);
  export_addr("ADDR_PRO_POSITION_P_GAIN", AddrPro::position_p_gain);
  export_addr("ADDR_PRO_GOAL_POSITION", AddrPro::goal_position);
  export_addr("ADDR_PRO_GOAL_VELOCITY", AddrPro::goal_velocity);
  export_addr("ADDR_PRO_GOAL_TORQUE", AddrPro::goal_torque);
  export_addr("ADDR_PRO_GOAL_ACCELERATION", AddrPro::goal_acceleration);
  export_addr("ADDR_PRO_MOVING", AddrPro::moving);
  export_addr("ADDR_PRO_PRESENT_POSITION", AddrPro::present_position);
  export_addr("ADDR_PRO_PRESENT_VELOCITY", AddrPro::present_velocity);
  export_addr("ADDR_PRO_PRESENT_CURRENT", AddrPro::present_current);
  export_addr("ADDR_PRO_PRESENT_INPUT_VOLTAGE", AddrPro::present_input_voltage);
  export_addr("ADDR_PRO_PRESENT_TEMPERATURE", AddrPro::present_temperature);
  export_addr("ADDR_PRO_EXTERNAL_PORT_DATA_1", AddrPro::external_port_data_1);
  export_addr("ADDR_PRO_EXTERNAL_PORT_DATA_2", AddrPro::external_port_data_2);
  export_addr("ADDR_PRO_EXTERNAL_PORT_DATA_3", AddrPro::external_port_data_3);
  export_addr("ADDR_PRO_EXTERNAL_PORT_DATA_4", AddrPro::external_port_data_4);
  export_addr("ADDR_PRO_REGISTERED_INSTRUCTION", AddrPro::registered_instruction);
  export_addr("ADDR_PRO_STATUS_RETURN_LEVEL", AddrPro::status_return_level);
  export_addr("ADDR_PRO_HARDWARE_ERROR_STATUS", AddrPro::hardware_error_status);
  export_addr("ADDR_PRO_PWM_LIMIT", AddrPro::pwm_limit);
  export_addr("ADDR_PRO_CURRENT_LIMIT", AddrPro::current_limit);
  export_addr("ADDR_PRO_GOAL_CURRENT", AddrPro::goal_current);
  export_addr("ADDR_PRO_GOAL_PWM", AddrPro::goal_pwm);
  export_addr("ADDR_PRO_PROFILE_VELOCITY", AddrPro::profile_velocity);
  export_addr("ADDR_PRO_PROFILE_ACCELERATION", AddrPro::profile_acceleration);
  export_addr("ADDR_PRO_PRESENT_PWM", AddrPro::present_pwm);
  export_addr("ADDR_PRO_VELOCITY_TRAJECTORY", AddrPro::velocity_trajectory);
  export_addr("ADDR_PRO_POSITION_TRAJECTORY", AddrPro::position_trajectory);
  export_addr("ADDR_PRO_POSITION_D_GAIN", AddrPro::position_d_gain);
  export_addr("ADDR_PRO_POSITION_I_GAIN", AddrPro::position_i_gain);
  export_addr("ADDR_PRO_FEEDFORWARD_2ND_GAIN", AddrPro::feedforward_2nd_gain);
  export_addr("ADDR_PRO_FEEDFORWARD_1ST_GAIN", AddrPro::feedforward_1st_gain);
  export_addr("ADDR_PRO_BUS_WATCHDOG", AddrPro::bus_watchdog);

  // compatibility aliases for existing scripts
  m.attr("ADDR_MODEL_NUMBER") = py::int_(AddrCommon::model_number.address());
  m.attr("ADDR_ID") = py::int_(AddrCommon::id.address());
  m.attr("ADDR_X_MODEL_NUMBER") = py::int_(AddrCommon::model_number.address());
  m.attr("ADDR_X_ID") = py::int_(AddrCommon::id.address());
  m.attr("ADDR_P_MODEL_NUMBER") = py::int_(AddrCommon::model_number.address());
  m.attr("ADDR_P_ID") = py::int_(AddrCommon::id.address());
  m.attr("ADDR_PRO_MODEL_NUMBER") = py::int_(AddrCommon::model_number.address());
  m.attr("ADDR_PRO_ID") = py::int_(AddrCommon::id.address());
  m.attr("ADDR_BAUDRATE") = py::int_(AddrX::baudrate.address());
  m.attr("ADDR_X_TORQUE_ENABLE") = py::int_(AddrX::torque_enable.address());
  m.attr("ADDR_P_TORQUE_ENABLE") = py::int_(AddrP::torque_enable.address());
  m.attr("ADDR_PRO_TORQUE_ENABLE") = py::int_(AddrPro::torque_enable.address());
  m.attr("TORQUE_DISABLE") = py::int_(static_cast<int>(TORQUE_DISABLE));
  m.attr("SERIES_X") = py::int_(static_cast<int>(SERIES_X));
  m.attr("SERIES_P") = py::int_(static_cast<int>(SERIES_P));

  m.attr("BAUDRATE_INDEX_9600") = py::int_(static_cast<int>(BAUDRATE_INDEX_9600));
  m.attr("BAUDRATE_INDEX_57600") = py::int_(static_cast<int>(BAUDRATE_INDEX_57600));
  m.attr("BAUDRATE_INDEX_115200") = py::int_(static_cast<int>(BAUDRATE_INDEX_115200));
  m.attr("BAUDRATE_INDEX_1M") = py::int_(static_cast<int>(BAUDRATE_INDEX_1M));
  m.attr("BAUDRATE_INDEX_2M") = py::int_(static_cast<int>(BAUDRATE_INDEX_2M));
  m.attr("BAUDRATE_INDEX_3M") = py::int_(static_cast<int>(BAUDRATE_INDEX_3M));
  m.attr("BAUDRATE_INDEX_4M") = py::int_(static_cast<int>(BAUDRATE_INDEX_4M));
  m.attr("BAUDRATE_INDEX_4M5") = py::int_(static_cast<int>(BAUDRATE_INDEX_4M5));
  m.attr("BAUDRATE_INDEX_6M") = py::int_(static_cast<int>(BAUDRATE_INDEX_6M));
  m.attr("BAUDRATE_INDEX_10M5") = py::int_(static_cast<int>(BAUDRATE_INDEX_10M5));
}
