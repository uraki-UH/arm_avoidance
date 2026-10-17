#include "dynamixel_parameters.h"

#include <limits>

DynamixelAddress::DynamixelAddress( 
  uint16_t address, DynamixelDataType data_type, DynamixelPhysicalUnit physical_unit)
    : is_dummy_(false), address_(address), data_type_(data_type), physical_unit_(physical_unit) {
  switch (data_type_) {
    case TYPE_INT8:
    case TYPE_UINT8:  size_ = 1; break;
    case TYPE_INT16:
    case TYPE_UINT16: size_ = 2; break;
    case TYPE_INT32:
    case TYPE_UINT32: size_ = 4; break;
  }
}

DynamixelAddress DynamixelAddress::make_dummy() const {
  auto addr = DynamixelAddress(address_, data_type_, physical_unit_);
  addr.is_dummy_ = true;
  return addr;
}

int64_t DynamixelAddress::val2pulse(double val, uint16_t model_num) const {
  assert(dynamixel_series(model_num) != SERIES_UNKNOWN);  // 未対応のシリーズの場合はエラー
  double pulse;
  switch (physical_unit_) {
    case UNIT_POSITION:          pulse =         val /*rad*/   / (2 * M_PI) * pos_resolution(model_num); break;
    case UNIT_POSITION_OFFSET:   pulse = (M_PI + val)/*rad*/   / (2 * M_PI) * pos_resolution(model_num); break;
    case UNIT_VELOCITY:          pulse =         val /*rad_s*/ / vel_factor(model_num); break;
    case UNIT_ACCELERATION:      pulse =         val /*rad_ss*// acc_factor(model_num); break;
    case UNIT_CURRENT:           pulse =         val /*mA*/    / cur_factor(model_num); break;
    case UNIT_VOLTAGE:           pulse =         val /*V*/     / 0.1 /*V*/; break;
    case UNIT_TEMPERATURE:       pulse =         val /*degC*/  / 1.0 /*degC*/; break;
    case UNIT_PWM:               pulse =         val /*%*/     / 100.0 * pwm_resolution(model_num) /*degC*/; break;
    case UNIT_RETURN_DELAY_TIME: pulse =         val /*us*/    / 2.0 /*us*/; break;
    case UNIT_BUS_WATCHDOG:      pulse =         val /*ms*/    / 20.0 /*ms*/; break;
    case UNIT_REALTIME_TICK:     pulse =         val /*ms*/    / 1.0 /*ms*/; break;
    default:                     pulse =         val; break;
  }
  return pulse > 0 ? ceil(pulse) : floor(pulse);
}

double DynamixelAddress::pulse2val(int64_t pulse, uint16_t model_num) const {
  assert(dynamixel_series(model_num) != SERIES_UNKNOWN);  // 未対応のシリーズの場合はエラー
  if (is_dummy_) return std::numeric_limits<double>::quiet_NaN();
  switch (physical_unit_) {
    case UNIT_POSITION:              return pulse * (2 * M_PI) / pos_resolution(model_num) /*rad*/;
    case UNIT_POSITION_OFFSET:       return pulse * (2 * M_PI) / pos_resolution(model_num) - M_PI /*rad*/;
    case UNIT_VELOCITY:              return pulse * vel_factor(model_num) /*rad_s*/;
    case UNIT_ACCELERATION:          return pulse * acc_factor(model_num) /*rad_ss*/;
    case UNIT_CURRENT:               return pulse * cur_factor(model_num) /*mA*/;
    case UNIT_VOLTAGE:               return pulse * 0.1 /*V*/;
    case UNIT_TEMPERATURE:           return pulse * 1.0 /*degC*/;
    case UNIT_PWM:                   return pulse * 100.0 / pwm_resolution(model_num) /*%*/;
    case UNIT_RETURN_DELAY_TIME:     return pulse * 2.0 /*us*/;
    case UNIT_BUS_WATCHDOG:          return pulse * 20.0 /*ms*/;
    case UNIT_REALTIME_TICK:         return pulse * 1.0 /*ms*/;
    default:                         return pulse;
  }
}

uint32_t DynamixelAddress::pos_resolution(uint16_t model_num) const {
  switch (model_num) {
    // Pシリーズ
    case MODEL_PH54_200_S500:
    case MODEL_PH54_100_S500: return 1003846;
    case MODEL_PH42_020_S300: return 607500;
    case MODEL_PM54_060_S250:
    case MODEL_PM54_040_S250: return 502834;
    case MODEL_PM42_010_S260: return 526374;
    // ProPlusシリーズ
    case MODEL_H54_200_S500_A:
    case MODEL_H54_100_S500_A: return 1003846;
    case MODEL_H42_020_S300_A: return 607500;
    case MODEL_M54_060_S250_A:
    case MODEL_M54_040_S250_A: return 502834;
    case MODEL_M42_010_S260_A: return 526375;
    // Proシリーズ
    case MODEL_H54_200_S500:
    case MODEL_H54_100_S500: return 501923;
    case MODEL_H42_020_S300: return 303750;
    case MODEL_M54_060_S250:
    case MODEL_M54_040_S250: return 251417;
    case MODEL_M42_010_S260: return 263187;
    case MODEL_L54_050_S290: return 207692;
    case MODEL_L54_050_S500: return 361384;
    case MODEL_L54_030_S400: return 288395;
    case MODEL_L54_030_S500: return 316384;
    case MODEL_L42_010_S300: return 4096;
    // Xシリーズのデフォルト値
    default: return 4096;
  }
}

uint16_t DynamixelAddress::pwm_resolution(uint16_t model_num) const {
  if      ( dynamixel_series(model_num) == SERIES_P ) return 2009;
  else if ( dynamixel_series(model_num) == SERIES_X ) return 885;
  else   /*dynamixel_series(model_num) == SERIES_PRO*/ assert(false); // 旧ProシリーズではPWM制御できないはず．
  return 1000; // ここには来ないはずで，コンパイルエラー回避のためのダミーの値
}

double DynamixelAddress::cur_factor(uint16_t model_num) const {
  if (dynamixel_series(model_num) == SERIES_P) return 1.00;
  // dynamixel_series(model_num) == SERIES_X || SERIES_PRO
  switch (model_num) {
    // Proシリーズ
    case MODEL_H42_020_S300:
    case MODEL_M42_010_S260:
    case MODEL_L42_010_S300: return 8250.0 / 2048.0;
    case MODEL_H54_200_S500:
    case MODEL_H54_100_S500:
    case MODEL_M54_060_S250:
    case MODEL_M54_040_S250:
    case MODEL_L54_050_S290:
    case MODEL_L54_050_S500:
    case MODEL_L54_030_S400:
    case MODEL_L54_030_S500: return 33000.0 / 2048.0;
    // Xシリーズ
    case MODEL_XL330_M077:
    case MODEL_XL330_M288:
    case MODEL_XC330_M181:
    case MODEL_XC330_M288:
    case MODEL_XC330_T181:
    case MODEL_XC330_T288: return 1.00;
    case MODEL_XH430_V210:
    case MODEL_XH430_V350: return 1.34;
    default: return 2.69; /*mA*/
  }
}

double DynamixelAddress::vel_factor(uint16_t model_num) const {
  if (dynamixel_series(model_num) == SERIES_P) return 0.010 /*rev/min*/ / 60.0 * (2 * M_PI);
  if (dynamixel_series(model_num) == SERIES_X) return 0.229 /*rev/min*/ / 60.0 * (2 * M_PI);
  /*dynamixel_series(model_num) == SERIES_PRO*/
  switch (model_num) {
    case MODEL_H54_200_S500:
    case MODEL_H54_100_S500: return 1 / 501.923;
    case MODEL_H42_020_S300: return 1 / 303.750;
    case MODEL_M54_060_S250:
    case MODEL_M54_040_S250: return 1 / 251.417;
    case MODEL_L54_050_S290: return 1 / 288.461;
    case MODEL_L54_050_S500:
    case MODEL_L54_030_S500: return 1 / 501.923;
    case MODEL_L54_030_S400: return 1 / 400.550;
    case MODEL_L42_010_S300: return 1 / 303.8;
    default: return 1 / 100.0;  // 適当な値
  }
}

double DynamixelAddress::acc_factor(uint16_t model_num) const {
  if (dynamixel_series(model_num) == SERIES_P) return 1.0 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);  // ベステクの数値は間違ってる
  if (dynamixel_series(model_num) == SERIES_X) return 214.577 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
  /*dynamixel_series(model_num) == SERIES_PRO*/
  switch (model_num) {
    case MODEL_H54_200_S500:
    case MODEL_H54_100_S500: return 58000.0 / 501.923 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_H42_020_S300: return 58000.0 / 303.750 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_M54_060_S250:
    case MODEL_M54_040_S250: return 58000.0 / 251.417 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_L54_050_S290: return 58000.0 / 288.461 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_L54_050_S500:
    case MODEL_L54_030_S500: return 58000.0 / 501.923 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_L54_030_S400: return 58000.0 / 400.550 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    case MODEL_L42_010_S300: return 58000.0 / 303.8 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);
    default: return 58000.0 / 100.0 /*rev/min^2*/ / 60.0 / 60.0 * (2 * M_PI);  // 適当な値
  }
}

bool has_external_port(uint16_t model_num) {
  switch (model_num) {
    // XH540
    case MODEL_XH540_W150:
    case MODEL_XH540_V150:
    case MODEL_XH540_W270:
    case MODEL_XH540_V270:
    // XD540
    case MODEL_XD540_T150:
    case MODEL_XD540_T270:
    // Pシリーズ
    case MODEL_PH54_200_S500:
    case MODEL_PH54_100_S500:
    case MODEL_PH42_020_S300:
    case MODEL_PM54_060_S250:
    case MODEL_PM54_040_S250:
    case MODEL_PM42_010_S260:
    case MODEL_H54_200_S500_A:
    case MODEL_H54_100_S500_A:
    case MODEL_H42_020_S300_A:
    case MODEL_M54_060_S250_A:
    case MODEL_M54_040_S250_A:
    case MODEL_M42_010_S260_A:
    // 旧Proシリーズ
    case MODEL_H54_200_S500:
    case MODEL_H54_100_S500:
    case MODEL_H42_020_S300:
    case MODEL_M54_060_S250:
    case MODEL_M54_040_S250:
    case MODEL_L54_050_S290:
    case MODEL_L54_050_S500:
    case MODEL_L54_030_S400:
    case MODEL_L54_030_S500:
    case MODEL_L42_010_S300: return true;
    default: return false;
  }
}

bool has_current_sensor(uint16_t model_num) {
  switch (model_num) {
    case MODEL_XL430_W250:
    case MODEL_2XL430_W250:
    case MODEL_XC430_W150:
    case MODEL_XC430_W240:
    case MODEL_2XC430_W250: return false;
    default: return true;
  }
}

bool has_pwm_slope(uint16_t model_num) {
  switch (model_num) {
    case MODEL_XL330_M077:
    case MODEL_XL330_M288:
    case MODEL_XC330_T181:
    case MODEL_XC330_T288:
    case MODEL_XC330_M181:
    case MODEL_XC330_M288: return true;
    default: return false;
  }
}

bool has_drive_mode(uint16_t model_num) {
  switch (dynamixel_series(model_num)) {
    case SERIES_X:
    case SERIES_P: return true;
    default: return false;
  }
}

bool has_restore_configuration(uint16_t model_num) {
  switch (dynamixel_series(model_num)) {
    case SERIES_X:
    case SERIES_P: return true;
    default: return false;
  }
}

bool has_bus_watchdog(uint16_t model_num) {
  switch (dynamixel_series(model_num)) {
    case SERIES_X:
    case SERIES_P: return true;
    default: return false;
  }
}

bool has_shadow_id(uint16_t model_num) {
  switch (dynamixel_series(model_num)) {
    case SERIES_X:
    case SERIES_P: return true;
    default: return false;
  }
}
