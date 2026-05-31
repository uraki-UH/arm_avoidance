#pragma once

#include <string>

class ControlInterface {
public:
  virtual ~ControlInterface() = default;

  // トルク制御
  virtual void setTorque(const std::string &joint_name, double torque) = 0;

  // 速度制御（角速度制御）
  virtual void setVelocity(const std::string &joint_name, double velocity) = 0;

  // 姿勢制御（目標角度制御）
  virtual void setPosition(const std::string &joint_name,
                           double target_angle) = 0;

  // 制御を更新（シミュレーションステップごとに呼び出す）
  virtual void update(double time_step) = 0;
};
