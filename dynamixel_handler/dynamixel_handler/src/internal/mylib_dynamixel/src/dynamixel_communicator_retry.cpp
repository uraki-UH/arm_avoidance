#include "dynamixel_communicator.h"
#include <stdio.h>
#include <thread>
using std::this_thread::sleep_for;
#include <chrono>
using namespace std::chrono_literals;

/** @fn
 * @brief 指定したDynamixelにpingを送り，応答があるかをboolで返す
 * @param uint8_t servo_id 対象のID
 * @return bool 応答があったかどうか
 */
bool DynamixelCommunicator::tryPing(uint8_t servo_id) {
  for (int i=0; i<num_try_; i++) {
    if ( Ping(servo_id) ) return true;
    sleep_for(1ms*msec_interval_);
  }
  return false;
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @param int64_t data_int 書き込むデータ．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::tryWrite(DynamixelAddress dp, uint8_t servo_id, int64_t data_int) {
  for (int i=0; i<num_try_; i++) {
    if ( Write(dp, servo_id, data_int) ) return true;
    sleep_for(1ms*msec_interval_);
  }
  return false;
}

/** @fn
 * @brief Dynamixelから情報を読み込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @return (int64_t) 読み込んだデータ．intのまま
 */
int64_t DynamixelCommunicator::tryRead(DynamixelAddress dp, uint8_t servo_id) {
    int64_t data_int = 0;
    for (int i=0; i<num_try_; i++) {
        data_int = Read(dp, servo_id);
        if ( !comm_error_last_read_ && !timeout_last_read_ ) return data_int;
        sleep_for(1ms*msec_interval_);
    }
    return data_int;
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param uint8_t servo_id 対象のID
 * @param vector<int64_t> data_int_list 書き込むデータの配列．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::tryWrite(const vector<DynamixelAddress>& dp_list, uint8_t servo_id, const vector<int64_t>& data_int_list) {
  for (int i=0; i<num_try_; i++) {
    if ( Write(dp_list, servo_id, data_int_list) ) return true;
    sleep_for(1ms*msec_interval_);
  }
  return false;
}

/** @fn
 * @brief Dynamixelから複数の情報を同時に読み込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param uint8_t servo_id 対象のID
 * @return vector<int64_t> 読み込んだデータを格納する配列．intに変換済みのもの．
 */
vector<int64_t> DynamixelCommunicator::tryRead(const vector<DynamixelAddress>& dp_list, uint8_t servo_id) {
    vector<int64_t> data_int_list(dp_list.size());
    for (int i=0; i<num_try_; i++) {
        data_int_list = Read(dp_list, servo_id);
        if ( !comm_error_last_read_ && !timeout_last_read_ ) return data_int_list;
        sleep_for(1ms*msec_interval_);
    }
    return data_int_list;
}