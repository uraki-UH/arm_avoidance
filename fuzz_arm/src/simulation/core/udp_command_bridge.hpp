#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_sim {
namespace simulation {

/**
 * @brief UDP経由で受信した制御コマンドを保持する構造体
 */
struct JointCommand {
  std::vector<double> joint_positions; // 6軸アーム (ラジアン)
  double gripper_position;             // グリッパー (メートル)
  bool is_valid = false;
  uint64_t timestamp = 0; // 受信時刻 (ミリ秒)
};

/**
 * @brief UDP通信とデータパース、最新状態の保持を行う汎用クラス
 * 直接物理エンジン(ODE)を操作せず、受信した状態を「コマンド」としてカプセル化します。
 */
class UdpCommandBridge {
public:
  UdpCommandBridge(int port = 12345);
  ~UdpCommandBridge();

  void start();
  void stop();

  /**
   * @brief 最新の有効なコマンドを取得します。
   * @param out_command 最新のデータで上書きされる構造体
   * @return 新しいデータが受信されていれば true
   */
  bool getLatestCommand(JointCommand &out_command);

  bool isRunning() const { return running_; }

private:
  void receiverLoop();
  void parseCsv(const std::string &csv);

  int port_;
  int sockfd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;

  std::mutex mutex_;
  JointCommand latest_command_;
  bool has_new_data_ = false;

  uint64_t getCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
};

} // namespace simulation
} // namespace robot_sim
