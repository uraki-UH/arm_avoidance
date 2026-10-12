#include "dynamixel_communicator.h"
#include <stdio.h>
#include <thread>
using std::this_thread::sleep_for;
#include <chrono>
using namespace std::chrono_literals;

// #define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xff))
// #define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

static constexpr uint8_t INSTRUCTION_PING           = 0x01;
static constexpr uint8_t INSTRUCTION_READ           = 0x02;
static constexpr uint8_t INSTRUCTION_WRITE          = 0x03;
static constexpr uint8_t INSTRUCTION_RED_WRITE      = 0x04;
static constexpr uint8_t INSTRUCTION_ACTION         = 0x05;
static constexpr uint8_t INSTRUCTION_FACTORY_RESET  = 0x06;
static constexpr uint8_t INSTRUCTION_REBOOT         = 0x08;
static constexpr uint8_t INSTRUCTION_SYNC_READ      = 0x82;
static constexpr uint8_t INSTRUCTION_SYNC_WRITE     = 0x83;
static constexpr uint8_t INSTRUCTION_FAST_SYNC_READ = 0x8A;
static constexpr uint8_t INSTRUCTION_BULK_READ      = 0x92;
static constexpr uint8_t INSTRUCTION_BULK_WRITE     = 0x93;
static constexpr uint8_t INSTRUCTION_FAST_BULK_READ = 0x9A;

static constexpr uint8_t HEADER[4] = {0xFF, 0xFF, 0xFD, 0x00};
static constexpr uint8_t BROADCAST_ID = 0xFE;

static constexpr uint16_t HEADER_SIZE = 4;
static constexpr uint16_t ID_SIZE = 1;
static constexpr uint16_t LENGTH_SIZE = 2;
static constexpr uint16_t INSTRUCTION_SIZE = 1;
static constexpr uint16_t ERROR_SIZE = 1;
static constexpr uint16_t CRC_SIZE = 2;
static constexpr uint16_t ADDR_SIZE = 2;
static constexpr uint16_t DATA_LEN_SIZE = 2;
static constexpr uint16_t PING_INFO_SIZE = 3;
static constexpr uint16_t FACTORY_RESET_OPTION_SIZE = 1;

static constexpr uint16_t COMMAND_TX_PACKET_SIZE =
    HEADER_SIZE + ID_SIZE + LENGTH_SIZE + INSTRUCTION_SIZE + CRC_SIZE;  // 10
static constexpr uint16_t STATUS_PACKET_SIZE =
    HEADER_SIZE + ID_SIZE + LENGTH_SIZE + INSTRUCTION_SIZE + ERROR_SIZE + CRC_SIZE;  // 11
static constexpr uint16_t READ_TX_PACKET_SIZE =
    COMMAND_TX_PACKET_SIZE + ADDR_SIZE + DATA_LEN_SIZE;  // 14
static constexpr uint16_t PING_PACKET_SIZE =
    STATUS_PACKET_SIZE + PING_INFO_SIZE;  // 14
static constexpr uint16_t FACTORY_RESET_TX_PACKET_SIZE =
    COMMAND_TX_PACKET_SIZE + FACTORY_RESET_OPTION_SIZE;
static constexpr uint16_t FAST_SYNC_FRAME_PREFIX_SIZE =
    HEADER_SIZE + ID_SIZE + LENGTH_SIZE + INSTRUCTION_SIZE;  // 8
static constexpr uint16_t FAST_SYNC_PER_SERVO_OVERHEAD =
    ERROR_SIZE + ID_SIZE + CRC_SIZE;  // 4


static constexpr uint8_t COM_ERROR_RESULT_FAIL = 0;
static constexpr uint8_t COM_ERROR_INSTRUCTION = 1;
static constexpr uint8_t COM_ERROR_CRC         = 2;
static constexpr uint8_t COM_ERROR_DATA_RANGE  = 3;
static constexpr uint8_t COM_ERROR_DATA_LENGTH = 4;
static constexpr uint8_t COM_ERROR_DATA_LIMIT  = 5;
static constexpr uint8_t COM_ERROR_ACCESS      = 6;
static constexpr uint8_t COM_ERROR_ALERT       = 7;

inline void clear_port_with_drain(PortHandler* port_handler) {
  uint8_t discard[256];
  while (true) {
    const int available = port_handler->getBytesAvailable();
    if (available <= 0) break;
    const int n = port_handler->readPort(
        discard, std::min(available, static_cast<int>(sizeof(discard))));
    if (n <= 0) break;
  }
}

uint16_t DynamixelCommunicator::CalcChecksum(const uint8_t* data, size_t length) {
  uint16_t crc_accum = 0;
  static constexpr uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 
    0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 
    0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 
    0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 
    0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 
    0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 
    0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 
    0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 
    0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 
    0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 
    0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 
    0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 
    0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 
    0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 
    0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 
    0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 
    0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 
    0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 
    0x832B, 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 
    0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 
    0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 
    0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 
    0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 
    0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 
    0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 
    0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 
    0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 
    0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 
    0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 
    0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 
    0x0264, 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 
    0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 
    0x0208, 0x820D, 0x8207, 0x0202 
  };

  for (size_t j = 0; j < length; j++) {
    const uint16_t i = ((uint16_t)(crc_accum >> 8) ^ data[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

/** @fn
 * @brief 送信用にdata_write_[]にデータを格納する
 * @param DynamixelDataType type 変換後のデータ型
 * @param int64_t data_int 書き込む値．int型に変換済み．どの型にも対応できるようにint64_t
 * @return void
 */
void DynamixelCommunicator::EncodeDataWrite(DynamixelDataType type, int64_t data_int) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t d = int8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_UINT8 : {
      uint8_t d = uint8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_INT16 : {
      int16_t d = int16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_UINT16 : {
      uint16_t d = uint16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_INT32 : {
      int32_t d = int32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
    case TYPE_UINT32 : {
      uint32_t d = uint32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
  }
}

/** @fn
 * @brief data_read_[]に格納されたデータをつなげて一つのデータに復元する
 * @param DynamixelDataType type 変換後のデータ型
 * @return (int64_t) 変換後の値．どの型にも対応できるようにint64_t
 */
int64_t DynamixelCommunicator::DecodeDataRead(DynamixelDataType type) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
    }
    case TYPE_UINT8 : {
      uint8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
    }
    case TYPE_INT16 : {
      int16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
    }
    case TYPE_UINT16 :  {
      uint16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
    }
    case TYPE_INT32 : {
      int32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
    }
    case TYPE_UINT32 : {
      uint32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
    }
  }
  return 0;
}

/** @fn
 * @brief port_handler_を用いて，ポートを開く
 */
bool DynamixelCommunicator::OpenPort() {
  if (port_handler_->openPort()) {
    if (verbose_) printf("Succeeded to open the port : %s!\n", port_name_);
  } else {
    printf("Failed to open the port : %s!\n", port_name_);
    return false;
  }

  port_handler_->setLatencyTimer(latency_timer_);
  if (verbose_) printf("Succeeded to change the latency timer : %d!\n", latency_timer_);

  if (port_handler_->setBaudRate(baudrate_)) {
    if (verbose_) printf("Succeeded to change the baudrate : %d!\n", baudrate_);
    return true;
  } else {
    printf("Failed to change the baudrate : %d!\n", baudrate_);
    return false;
  }
}

/** @fn
 * @brief port_handler_を用いて，ポートを閉じる
 */
bool DynamixelCommunicator::ClosePort() {
  port_handler_->closePort();
  if (verbose_) printf("Close port : %s!\n", port_name_);
  return true;
}

/** @fn
 * @brief 指定したDynamixelをリブートする
 * @param uint8_t servo_id 対象のID
 */
void DynamixelCommunicator::Reboot(uint8_t servo_id) {
  uint8_t send_data[COMMAND_TX_PACKET_SIZE] = {0};
  uint16_t length = INSTRUCTION_SIZE + CRC_SIZE;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_REBOOT;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  clear_port_with_drain(port_handler_);
  port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE);

  if (status_return_level_ != STATUS_RETURN_LEVEL_ALL) return;
 
  timeout_last_read_ = false;
  port_handler_->setPacketTimeout(STATUS_PACKET_SIZE);
  while(port_handler_->getBytesAvailable() < STATUS_PACKET_SIZE) {
    if (port_handler_->isPacketTimeout()) {
      if(verbose_) printf("Reboot Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
      timeout_last_read_ = true;
      return;
    }
    sleep_for(20us);
  }

  uint8_t read_data[STATUS_PACKET_SIZE];
  port_handler_->readPort(read_data, STATUS_PACKET_SIZE);
}

/** @fn
 * @brief 指定したDynamixelをファクトリーリセットする
 * @param uint8_t servo_id 対象のID
 * @param FactoryResetLevel level リセットレベル
 */
void DynamixelCommunicator::FactoryReset(uint8_t servo_id, FactoryResetLevel level) {
  uint8_t send_data[FACTORY_RESET_TX_PACKET_SIZE] = {0};
  uint16_t length = INSTRUCTION_SIZE + FACTORY_RESET_OPTION_SIZE + CRC_SIZE;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_FACTORY_RESET;  // instruction
  send_data[8] = level;
  uint16_t sum = CalcChecksum(send_data, 9);
  send_data[9] = sum & 0xFF;
  send_data[10] = (sum>>8) & 0xFF;
  port_handler_->writePort(send_data, FACTORY_RESET_TX_PACKET_SIZE);

  if (status_return_level_ != STATUS_RETURN_LEVEL_ALL) return; 

  uint8_t read_data[STATUS_PACKET_SIZE];
  port_handler_->readPort(read_data, STATUS_PACKET_SIZE);
}


/** @fn
 * @brief 指定したDynamixelにpingを送り，応答があるかをboolで返す
 * @param uint8_t servo_id 対象のID
 * @return bool 応答があったかどうか
 */
vector<uint8_t> DynamixelCommunicator::Ping_broadcast() {
  uint8_t send_data[COMMAND_TX_PACKET_SIZE] = {0};
  uint16_t length = INSTRUCTION_SIZE + CRC_SIZE;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = BROADCAST_ID;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_PING;  // instruction
  uint16_t sum = CalcChecksum(send_data, COMMAND_TX_PACKET_SIZE - CRC_SIZE);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  clear_port_with_drain(port_handler_);
  port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE);

  ping_id_model_map_last_read_.clear();
  vector<uint8_t> found_id_list;
  uint8_t read_data[PING_PACKET_SIZE] = {0};
  port_handler_->setPacketTimeout(msec_watch_ping_broadcast_); // Dynamixel2Arduino issue #106 より、Broadcast Ping は最大759ms程度の待機が必要。
  while (!port_handler_->isPacketTimeout()) {
    if (port_handler_->getBytesAvailable() < PING_PACKET_SIZE) {
      sleep_for(20us);
      continue;
    }
    const int read_length = port_handler_->readPort(read_data, PING_PACKET_SIZE);
    if (read_length != PING_PACKET_SIZE) {
      if (verbose_) printf("Ping Error(read length): expected %d, read %d\n", PING_PACKET_SIZE, read_length);
      continue;
    }
    const uint8_t id = read_data[4];
    if (id == BROADCAST_ID || id > 252) {
      if (verbose_) printf("Ping Error(id): expected != %d return %d\n", BROADCAST_ID, id);
      continue;
    }
    const uint16_t model_number = uint16_t(read_data[9]) | (uint16_t(read_data[10]) << 8);
    ping_id_model_map_last_read_[id] = model_number;
    found_id_list.push_back(id);
  }
  if (verbose_ && ping_id_model_map_last_read_.empty()) {
    printf("Ping Error(time out): ID %d, available bytes %d\n", BROADCAST_ID, port_handler_->getBytesAvailable());
  }
  return found_id_list;
}

bool DynamixelCommunicator::Ping(uint8_t servo_id) {
  uint8_t send_data[COMMAND_TX_PACKET_SIZE] = {0};
  uint16_t length = INSTRUCTION_SIZE + CRC_SIZE;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_PING;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8); // ここまででCRC計算
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  clear_port_with_drain(port_handler_);
  port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE);

  ping_id_model_map_last_read_.clear();
  port_handler_->setPacketTimeout(PING_PACKET_SIZE);
  while(port_handler_->getBytesAvailable() < PING_PACKET_SIZE) {
    if (port_handler_->isPacketTimeout()) {
        if(verbose_) printf("Ping Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
        return false;
    }
    sleep_for(20us);
  }

  uint8_t read_data[PING_PACKET_SIZE];
  int read_length = port_handler_->readPort(read_data, PING_PACKET_SIZE);
    if (read_length != PING_PACKET_SIZE) {
        if(verbose_) printf("Ping Error(no data): ID %d, data_length = %d\n", servo_id, read_length);
        return false;
    }
  if( read_data[4] == servo_id) {
    const uint16_t model_number = uint16_t(read_data[9]) | (uint16_t(read_data[10]) << 8);
    ping_id_model_map_last_read_[servo_id] = model_number;
    return true;
  } else {
    if (verbose_) printf("Ping Error(header or id): expected ID %d, return ID %d\n", servo_id, read_data[4]);
    return false;
  }
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @param int64_t data_int 書き込むデータ．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::Write(DynamixelAddress dp, uint8_t servo_id, int64_t data_int) {
    if (dp.is_dummy()) {
        if (verbose_) printf("Sync Write Error(dummy address is readonly): ID %d\n", servo_id);
        return false;
    }// dummy addressは書き込まない  
    uint8_t send_data[COMMAND_TX_PACKET_SIZE + ADDR_SIZE + 4] = {0}; // 書き込むdpのサイズによって変わるが， dp.size()は最大4 4+12=16より十分
    uint16_t length = static_cast<uint16_t>(dp.size() + INSTRUCTION_SIZE + ADDR_SIZE + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_WRITE;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    EncodeDataWrite(dp.data_type(), data_int);
    for(size_t i=0; i<dp.size(); i++) {
        send_data[10+i] = data_write_[i];
    }
    uint16_t sum = CalcChecksum(send_data, COMMAND_TX_PACKET_SIZE + dp.size());
    send_data[COMMAND_TX_PACKET_SIZE + dp.size()] = sum & 0xFF;
    send_data[COMMAND_TX_PACKET_SIZE + dp.size() + 1] = (sum>>8) & 0xFF;

    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE + ADDR_SIZE + dp.size());

    if (status_return_level_ != STATUS_RETURN_LEVEL_ALL) return true;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(STATUS_PACKET_SIZE);
    while(port_handler_->getBytesAvailable() < STATUS_PACKET_SIZE) {
        if (port_handler_->isPacketTimeout()) {
        if(verbose_) printf("Write Error(return status time out): ID %d, available bytes %d/%d\n", (int)servo_id, port_handler_->getBytesAvailable(), STATUS_PACKET_SIZE);
        timeout_last_read_ = true;
        return false;
        }
        sleep_for(20us);
    }
    uint8_t read_data[STATUS_PACKET_SIZE]; // Writeのステータスパケットは固定で11
    port_handler_->readPort(read_data, STATUS_PACKET_SIZE);

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<sizeof(send_data); i++) printf("%02X ", send_data[i]); printf("\n");}
    // if(verbose_) {printf("read:" ); for (size_t i=0; i<STATUS_PACKET_SIZE; i++) printf("%02X ", read_data[i]); printf("\n");}

    // エラーチェック
    comm_error_last_read_ = false;
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Write Error(return status header or id): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Write Error(return status packet error) : ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    } 
    uint16_t sum_est = CalcChecksum(read_data, STATUS_PACKET_SIZE - CRC_SIZE);
    uint16_t sum_read = uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE]) |
        uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + 1]) << 8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Write Error(return status crc): ID %d, est=%d, read=%d\n", (int)servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return false;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    return true;
}

/** @fn
 * @brief Dynamixelから情報を読み込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param uint8_t servo_id 対象のID
 * @return (int64_t) 読み込んだデータ．intのまま
 */
int64_t DynamixelCommunicator::Read(DynamixelAddress dp, uint8_t servo_id) {
    uint8_t send_data[READ_TX_PACKET_SIZE] = {0}; // Readのインストラクションパケットは固定で14
    uint16_t length = INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_READ;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    send_data[10] = dp.size() & 0xFF;
    send_data[11] = (dp.size()>>8) & 0xFF;
    uint16_t sum = CalcChecksum(send_data, 12);
    send_data[12] = sum & 0xFF;
    send_data[13] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, READ_TX_PACKET_SIZE);

    if (status_return_level_ == STATUS_RETURN_LEVEL_PING_ONLY) return 0;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(STATUS_PACKET_SIZE + dp.size()));
    while(port_handler_->getBytesAvailable() < STATUS_PACKET_SIZE + dp.size()) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Read Error(time out): ID %d, available bytes %d / %d\n", (int)servo_id, port_handler_->getBytesAvailable(), (int)(STATUS_PACKET_SIZE + dp.size()));
            deficient_byte_ = STATUS_PACKET_SIZE + dp.size() - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return 0;
        }
        sleep_for(20us);
    }

    uint8_t read_data[STATUS_PACKET_SIZE + 4] = {0}; // 11+dp.size()<=11+4=15より十分
    int read_length = port_handler_->readPort(read_data, STATUS_PACKET_SIZE + dp.size());

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<READ_TX_PACKET_SIZE; i++) printf("%02X ", send_data[i]); printf("\n");}
    // if(verbose_) {printf("read:" ); for (size_t i=0; i<sizeof(read_data); i++) printf("%02X ", read_data[i]); printf("\n");}
    
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_length != STATUS_PACKET_SIZE + dp.size()) {
        if(verbose_) printf("Read Error(no data): ID %d, data_length = %d\n", (int)servo_id, read_length);
        comm_error_last_read_ = true;
        return 0;
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Read Error(header or id): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return 0;
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Read Error(packet error) : ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return 0;
    } 
    uint16_t sum_est = CalcChecksum(read_data, STATUS_PACKET_SIZE - CRC_SIZE + dp.size());
    uint16_t sum_read = uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + dp.size()]) |
        uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + dp.size() + 1]) << 8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Read Error(crc): ID %d, est=%d, read=%d\n", (int)servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return 0;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[9+i];
    return DecodeDataRead(dp.data_type());
}


/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報を書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list servo_id_list 書き込むサーボのIDのベクトル
 * @param vector<uint64_t> data_int_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(DynamixelAddress dp,  const vector<uint8_t>& servo_id_list, const vector<int64_t>& data_int_list) {
  constexpr size_t max_write_servo = 100;
  constexpr size_t max_data_size = 4;
  if (dp.is_dummy()) {
        if (verbose_) printf("Sync Write Error(dummy address is readonly)\n");
        return false;
  }// dummy addressは書き込まない  
  if (servo_id_list.size() > max_write_servo) {
    if(verbose_) printf("Sync Write Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_write_servo);
    return false;
  }
  if (servo_id_list.size() != data_int_list.size()) {
    if(verbose_) printf("Sync Write Error(mismatch servo and data num): servo num=%d, data num=%d\n", (int)servo_id_list.size(), (int)data_int_list.size());
    return false;
  }
  uint8_t send_data[READ_TX_PACKET_SIZE + (1 + max_data_size) * max_write_servo] = {0}; // 書き込むサーボの数によって変わるが， dp.size()は最大4 (4+1)*100+14=514 100サーボに同時に4バイト書き込むことはないので十分
  const size_t num_servo = servo_id_list.size();
  uint16_t length = static_cast<uint16_t>((1+dp.size()) * num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_WRITE;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  send_data[10] = dp.size() & 0xFF;
  send_data[11] = (dp.size()>>8) & 0xFF;
  for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo*(dp.size()+1)] = servo_id_list[i_servo];
    EncodeDataWrite(dp.data_type(), data_int_list[i_servo]);
    for(size_t i_data=0; i_data<dp.size(); i_data++) {
      send_data[12+i_servo*(dp.size()+1)+1+i_data] = data_write_[i_data];
    }
  }
  uint16_t sum = CalcChecksum(send_data, 12+num_servo*(dp.size()+1));
  send_data[12+num_servo*(dp.size()+1)] = sum & 0xFF;
  send_data[13+num_servo*(dp.size()+1)] = (sum>>8) & 0xFF;

  clear_port_with_drain(port_handler_);
  port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo * (dp.size() + 1));
  return true;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param map<uint8_t, int64_t> id_data_int_map 書き込むサーボのIDと書き込むデータのマップ
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(DynamixelAddress dp, const map<uint8_t, int64_t>& id_data_int_map) {
    std::vector<uint8_t>  servo_id_list; servo_id_list.reserve(id_data_int_map.size());
    std::vector<int64_t> data_int_list; data_int_list.reserve(id_data_int_map.size());
    for (const auto& id_data : id_data_int_map) {
        servo_id_list.push_back(id_data.first);    // キー（サーボID）を追加
        data_int_list.push_back(id_data.second);   // 値（データ）を追加
    }
    return SyncWrite( dp, servo_id_list, data_int_list );
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレスから情報を読み込む
 * @param map<uint8_t, DynamixelAddress> id_dp_map 読み込むサーボIDとアドレスのマップ
 * @return map<uint8_t, int64_t> 読み込んだサーボIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::BulkRead(const map<uint8_t, DynamixelAddress>& id_dp_map) {
    map<uint8_t, vector<DynamixelAddress>> id_dp_list_map;
    for (const auto& id_dp : id_dp_map) id_dp_list_map[id_dp.first] = {id_dp.second};
    const auto id_data_vec_map = BulkRead(id_dp_list_map);

    map<uint8_t, int64_t> id_data_map;
    for (const auto& id_data_vec : id_data_vec_map) {
        if (id_data_vec.second.empty()) continue;
        id_data_map[id_data_vec.first] = id_data_vec.second[0];
    }
    return id_data_map;
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレスから情報を読み込む(高速版)
 * @param map<uint8_t, DynamixelAddress> id_dp_map 読み込むサーボIDとアドレスのマップ
 * @return map<uint8_t, int64_t> 読み込んだサーボIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::BulkRead_fast(const map<uint8_t, DynamixelAddress>& id_dp_map) {
    map<uint8_t, vector<DynamixelAddress>> id_dp_list_map;
    for (const auto& id_dp : id_dp_map) id_dp_list_map[id_dp.first] = {id_dp.second};
    const auto id_data_vec_map = BulkRead_fast(id_dp_list_map);

    map<uint8_t, int64_t> id_data_map;
    for (const auto& id_data_vec : id_data_vec_map) {
        if (id_data_vec.second.empty()) continue;
        id_data_map[id_data_vec.first] = id_data_vec.second[0];
    }
    return id_data_map;
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレスに情報を書き込む
 * @param map<uint8_t, DynamixelAddress> id_dp_map 書き込むサーボIDとアドレスのマップ
 * @param map<uint8_t, int64_t> id_data_int_map 書き込むサーボIDとデータのマップ
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::BulkWrite(const map<uint8_t, DynamixelAddress>& id_dp_map, const map<uint8_t, int64_t>& id_data_int_map) {
    vector<uint8_t> servo_id_list; servo_id_list.reserve(id_dp_map.size());
    vector<DynamixelAddress> dp_list; dp_list.reserve(id_dp_map.size());
    vector<int64_t> data_int_list; data_int_list.reserve(id_dp_map.size());
    for (const auto& [id, dp] : id_dp_map) {
        if (!id_data_int_map.count(id)) {
            if (verbose_) printf("Bulk Write Error(id data map): missing ID %d\n", id);
            return false;
        }
        servo_id_list.push_back(id);
        dp_list.push_back(dp);
        data_int_list.push_back(id_data_int_map.at(id));
    }
    return BulkWrite(dp_list, servo_id_list, data_int_list);
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレスに情報を書き込む(簡易版)
 * @param vector<DynamixelAddress> dp_list 書き込むアドレスリスト
 * @param vector<uint8_t> servo_id_list 書き込むサーボのIDリスト
 * @param vector<int64_t> data_int_list 書き込むデータリスト
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::BulkWrite(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list, const vector<int64_t>& data_int_list) {
    if (servo_id_list.size() != dp_list.size() || servo_id_list.size() != data_int_list.size()) {
        if (verbose_) printf("Bulk Write Error(mismatch id/addr/data num): id=%d, addr=%d, data=%d\n", (int)servo_id_list.size(), (int)dp_list.size(), (int)data_int_list.size());
        return false;
    }
    vector<vector<DynamixelAddress>> dp_lists; dp_lists.reserve(dp_list.size());
    vector<vector<int64_t>> data_vec_list; data_vec_list.reserve(data_int_list.size());
    for (size_t i=0; i<servo_id_list.size(); i++) {
        dp_lists.push_back({dp_list[i]});
        data_vec_list.push_back({data_int_list[i]});
    }
    return BulkWrite(dp_lists, servo_id_list, data_vec_list);
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, int64_t> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::SyncRead( DynamixelAddress dp, const vector<uint8_t>& servo_id_list) {
    constexpr size_t max_read_servo = 100;
    if (servo_id_list.size() > max_read_servo) {
        if(verbose_) printf("Sync Read Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_read_servo);
        return map<uint8_t, int64_t>();
    }
    if (servo_id_list.empty()) return map<uint8_t, int64_t>();
    uint8_t send_data[READ_TX_PACKET_SIZE + max_read_servo] = {0}; // 読み込むサーボの数によって変わるが， 100+14=114 100サーボに同時に読み込むことはないので十分
    const size_t num_servo = servo_id_list.size();
    uint16_t length = static_cast<uint16_t>(num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = 0xFE;  // id
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_SYNC_READ;  // instruction
    send_data[8] = dp.address() & 0xFF;
    send_data[9] = (dp.address()>>8) & 0xFF;
    send_data[10] = dp.size() & 0xFF;
    send_data[11] = (dp.size()>>8) & 0xFF;
    for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
        send_data[12+i_servo] = servo_id_list[i_servo];
    }
    uint16_t sum = CalcChecksum(send_data, 12+num_servo);
    send_data[12+num_servo] = sum & 0xFF;
    send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo);

    // データ読み込みの処理
    map<uint8_t, int64_t> id_data_int_map;
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    const uint8_t packet_len = STATUS_PACKET_SIZE + dp.size();
    const int expected_len = packet_len * num_servo;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(expected_len));
    while (port_handler_->getBytesAvailable() < expected_len) {
        if (port_handler_->isPacketTimeout()) {
            if (verbose_) printf("Sync Read Error(time out): available bytes %d / %d\n", port_handler_->getBytesAvailable(), expected_len);
            deficient_byte_ = expected_len - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            break;
        }
        sleep_for(20us);
    }

    const int bytes_to_read = std::min(port_handler_->getBytesAvailable(), expected_len);
    if (bytes_to_read <= 0) return id_data_int_map;

    std::vector<uint8_t> read_data(bytes_to_read, 0);
    int read_length = port_handler_->readPort(read_data.data(), bytes_to_read);
    if (read_length == -1) {
        if(verbose_) printf("Sync Read Error(read port)\n");
        comm_error_last_read_ = true;
        return id_data_int_map;
    }

    const int num_packets_read = read_length / packet_len;
    if (num_packets_read < static_cast<int>(num_servo)) {
        timeout_last_read_ = true;
        deficient_byte_ = expected_len - read_length;
    }

    for (int i_servo = 0; i_servo < num_packets_read; i_servo++) {
        uint8_t* packet = &read_data[i_servo * packet_len];

        if (packet[0] != HEADER[0] or
            packet[1] != HEADER[1] or
            packet[2] != HEADER[2] or
            packet[3] != HEADER[3]) {
            if(verbose_) printf("Sync Read Error(header): seq=%d\n", i_servo);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t id = packet[4];
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Sync Read Error(packet id) : est ID=%d, read ID=%d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t error = (uint8_t)packet[8];
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue;
        }
        uint16_t sum_est = CalcChecksum(packet, STATUS_PACKET_SIZE - CRC_SIZE + dp.size());
        uint16_t sum_read = uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + dp.size()]) |
            uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + dp.size() + 1]) << 8;
        if (sum_est != sum_read) {
            if(verbose_) printf("Sync Read Error(crc): ID %d, est=%d, read=%d\n", id, sum_est, sum_read);
            comm_error_last_read_ = true;
            continue;
        }

        // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if ( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        for(size_t i = 0; i < dp.size(); i++) data_read_[i] = packet[9 + i];
        id_data_int_map[id] = DecodeDataRead(dp.data_type());
    }
    return id_data_int_map;
}


/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む,パケットを最小限にすることで高速化したもの．サーボ1つでも失敗するとすべて読み込めない
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, int64_t> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, int64_t> DynamixelCommunicator::SyncRead_fast(DynamixelAddress dp, const vector<uint8_t>& servo_id_list) {
    constexpr size_t max_read_servo = 50;
    if (servo_id_list.size() > max_read_servo) {
        if(verbose_) printf("Fast Sync Read Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_read_servo);
        return map<uint8_t, int64_t>();
    }
	uint8_t send_data[READ_TX_PACKET_SIZE + max_read_servo] = {0}; // 読み込むサーボの数によって変わるが， 50+14=64 50サーボに同時に読み込むことはないので十分
	const size_t num_servo = servo_id_list.size();
	uint16_t length = static_cast<uint16_t>(num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
	send_data[0] = HEADER[0];
	send_data[1] = HEADER[1];
	send_data[2] = HEADER[2];
	send_data[3] = HEADER[3];
	send_data[4] = 0xFE;  // id
	send_data[5] = length & 0xFF;
	send_data[6] = (length>>8) & 0xFF;
	send_data[7] = INSTRUCTION_FAST_SYNC_READ;  // instruction
	send_data[8] = dp.address() & 0xFF;
	send_data[9] = (dp.address()>>8) & 0xFF;
	send_data[10] = dp.size() & 0xFF;
	send_data[11] = (dp.size()>>8) & 0xFF;
	for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
		send_data[12+i_servo] = servo_id_list[i_servo];
	}
	uint16_t sum = CalcChecksum(send_data, 12+num_servo);
	send_data[12+num_servo] = sum & 0xFF;
	send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read fast Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
	clear_port_with_drain(port_handler_);
	port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo);

    // データ読み込みの処理
	uint8_t length_a_servo = FAST_SYNC_PER_SERVO_OVERHEAD + dp.size(); // id error {data} crc crc
	uint16_t length_read_data = FAST_SYNC_FRAME_PREFIX_SIZE + length_a_servo * num_servo;
    timeout_last_read_ = false;
    port_handler_->setPacketTimeout( uint16_t(length_read_data) );
    while(port_handler_->getBytesAvailable() < length_read_data) {
		if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Fast Sync Read Error(time out) : available bytes %d / %d\n", port_handler_->getBytesAvailable(), length_read_data);
            deficient_byte_ = length_read_data - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return map<uint8_t, int64_t>();
		}
        sleep_for(20us);
	}

	// パケットの読み込み
    uint8_t read_data[FAST_SYNC_FRAME_PREFIX_SIZE + (FAST_SYNC_PER_SERVO_OVERHEAD + 4 /*max_data_size*/) * max_read_servo] = {0}; // 読み込むサーボとデータサイズによって変わるが， (4+4)*50+8=408 50サーボに同時に読み込むことはないので十分
    int read_length = port_handler_->readPort(read_data, length_read_data);
    // 全体のエラーチェック
    comm_error_last_read_ = false;
    if (read_length == -1) {
		if(verbose_) printf("Fast Sync Read Error(read port)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	if (read_data[0] != HEADER[0] || // 読み込めたパケットのヘッダーの確認
	    read_data[1] != HEADER[1] ||
	    read_data[2] != HEADER[2] ||
	    read_data[3] != HEADER[3]) {
		if(verbose_) printf("Fast Sync Read Error(header)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	if ( read_data[4] != 0xFE ) { // 読み込めたパケットのIDがブロードキャスト用のものか確認
		if(verbose_) printf("Fast Sync Read Error(broad cast id)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, int64_t>();
	}
	// 読み込めたパケットを個々のサーボのパケットに分割してエラーと正常系の処理,チェックサムの処理は行わない
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    map<uint8_t, int64_t> id_data_int_map;
	for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
		uint8_t id = (uint8_t)read_data[9 + i_servo*length_a_servo]; // servo id
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Fast Sync Read Error(packet id) : expected %d return %d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            return map<uint8_t, int64_t>(); // これ以降すべての読み込みを諦める．
        }
		uint8_t error = (uint8_t)read_data[8 + i_servo*length_a_servo]; // error
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Fast Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue; // 他のidのデータは生きている可能性があるので続行
        }
		// 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if ( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
		for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[10 + i_servo*length_a_servo + i];
		id_data_int_map[id] = DecodeDataRead(dp.data_type());
	}
	return id_data_int_map;
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param vector<DynamixelAddress> dp_list_unsorted 対象のパラメータのインスタンスのリスト
 * @param uint8_t servo_id 対象のID
 * @param vector<int64_t> data_int_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::Write(const vector<DynamixelAddress>& dp_list_sorted, uint8_t servo_id, const vector<int64_t>& data_int_list) {
    constexpr size_t max_param_num = 10;
    constexpr size_t max_data_size = 4;
    if (dp_list_sorted.size() > max_param_num) {
        if(verbose_) printf("Write Error(too many param): param num=%d > %d\n", (int)dp_list_sorted.size(), (int)max_param_num);
        return false;
    }
    if ( dp_list_sorted.size() != data_int_list.size() ) {
        if(verbose_) printf("Write Error(mismatch param and data num): param num=%d, data num=%d\n", (int)dp_list_sorted.size(), (int)data_int_list.size());
        return false;
    }
    // 書き込むデータの範囲を決定, ソート済みかつdummyでないアドレスが連続していないとNG
    DynamixelAddress dp_min = *dp_list_sorted.begin();
    DynamixelAddress dp_max = *dp_list_sorted.rbegin();
    for (size_t i=0; i<dp_list_sorted.size(); i++) { // アドレスが連続しているか確認
        if (dp_list_sorted[i].is_dummy()) {
            if (verbose_) printf("Write Error(dummy address is readonly): ID %d\n", servo_id);
            return false;
        }// dummy addressは書き込まない  
        if (i+1<dp_list_sorted.size() && dp_list_sorted[i].address() + dp_list_sorted[i].size() != dp_list_sorted[i+1].address()) {
            if(verbose_) printf("Write Error(address is not continuous):  ID %d, addr1=%d (%d), addr2=%d\n", servo_id, dp_list_sorted[i].address(), dp_list_sorted[i].size(), dp_list_sorted[i+1].address());
            return false;
        }
    }
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();
    uint8_t send_data[COMMAND_TX_PACKET_SIZE + ADDR_SIZE + max_data_size * max_param_num] = {0}; // 書き込むdpの数とサイズによって変わるが， 4*10+12=52 4バイトのデータ10個を同時に書き込むことはないので十分
    uint16_t length = static_cast<uint16_t>(size_total_dp + INSTRUCTION_SIZE + ADDR_SIZE + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_WRITE;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    for (size_t i=0; i<dp_list_sorted.size(); i++) {
        const DynamixelAddress& dp = dp_list_sorted[i];
        uint8_t index = dp.address() - dp_min.address();
        EncodeDataWrite(dp.data_type(), data_int_list[i]);
        for(size_t i=0; i<dp.size(); i++) {
            send_data[10+index+i] = data_write_[i];
        }
    }
    uint16_t sum = CalcChecksum(send_data, COMMAND_TX_PACKET_SIZE + size_total_dp);
    send_data[COMMAND_TX_PACKET_SIZE + size_total_dp] = sum & 0xFF;
    send_data[COMMAND_TX_PACKET_SIZE + size_total_dp + 1] = (sum>>8) & 0xFF;

    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE + ADDR_SIZE + size_total_dp);

    if (status_return_level_ != STATUS_RETURN_LEVEL_ALL) return true;
    // 以降はwriteと同じ

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(STATUS_PACKET_SIZE);
    while(port_handler_->getBytesAvailable() < STATUS_PACKET_SIZE) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Write Error(return status time out): ID %d, available bytes %d/%d\n", servo_id, port_handler_->getBytesAvailable(), STATUS_PACKET_SIZE);
            timeout_last_read_ = true;
            return false;
        }
        sleep_for(20us);
    }
    uint8_t read_data[STATUS_PACKET_SIZE] = {0}; // writeのステータスパケットは固定で11
    port_handler_->readPort(read_data, STATUS_PACKET_SIZE);
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
        if(verbose_) printf("Write Error(return status header or id): ID %d\n", servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint8_t error = (uint8_t)read_data[8]; // error
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Write Error(return status packet error): ID %d\n", (int)servo_id);
        comm_error_last_read_ = true;
        return false;
    }
    uint16_t sum_est = CalcChecksum(read_data, STATUS_PACKET_SIZE - CRC_SIZE);
    uint16_t sum_read = uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE]) |
        uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + 1]) << 8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Write Error(return status crc): ID %d, est=%d, read=%d\n", servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return false;
    }
    // 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    return true;
}


/** @fn
 * @brief Dynamixelから複数の情報を同時に読み込む
 * @param uint8_t servo_id 対象のID
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @return vector<int64_t> 読み込んだデータを格納する配列．intに変換済みのもの．
 */
vector<int64_t> DynamixelCommunicator::Read(const vector<DynamixelAddress>& dp_list, uint8_t servo_id) {
    constexpr size_t max_param_num = 10;
    constexpr size_t max_data_size = 4;
    if (dp_list.size() > max_param_num) {
        if(verbose_) printf("Read Error(too many param): ID %d, param num=%d > %d\n", (int)servo_id, (int)dp_list.size(), (int)max_param_num);
        return vector<int64_t>();
    }
    // 読み込むデータの範囲を決定, 連続していなくても許容
    auto [dp_min_it, dp_max_it] = minmax_element(dp_list.begin(), dp_list.end(),
        [](const DynamixelAddress& a, const DynamixelAddress& b){ return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_min_it;
    DynamixelAddress dp_max = *dp_max_it;
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

    // instruction packetを作成
    uint8_t send_data[READ_TX_PACKET_SIZE] = {0}; // Readのインストラクションパケットは固定で14
    uint16_t length = INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE;
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = servo_id;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_READ;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    send_data[10] = size_total_dp & 0xFF;
    send_data[11] = (size_total_dp>>8) & 0xFF;
    uint16_t sum = CalcChecksum(send_data, 12);
    send_data[12] = sum & 0xFF;
    send_data[13] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, READ_TX_PACKET_SIZE);

    if (status_return_level_ == STATUS_RETURN_LEVEL_PING_ONLY) return vector<int64_t>(dp_list.size(), 0); 

    // データ読み込みの処理
    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(STATUS_PACKET_SIZE + size_total_dp));
    while(port_handler_->getBytesAvailable() < STATUS_PACKET_SIZE + size_total_dp) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Read Error(time out): ID %d, available bytes %d / %d\n", servo_id, port_handler_->getBytesAvailable(), STATUS_PACKET_SIZE + size_total_dp);
            deficient_byte_ = STATUS_PACKET_SIZE + size_total_dp - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return vector<int64_t>(dp_list.size(), 0); // これ以降すべての読み込みを諦める．
        }
        sleep_for(20us);
    }

    uint8_t read_data[STATUS_PACKET_SIZE + max_data_size * max_param_num] = {0}; // 読み込むdpの数とサイズによって変わるが， 4*10+11=51 4バイトのデータ10個を同時に読み込むことはないので十分
    int read_length = port_handler_->readPort(read_data, STATUS_PACKET_SIZE + size_total_dp);
    // エラーチェック
    comm_error_last_read_ = false;
    if (read_length != STATUS_PACKET_SIZE + size_total_dp) {
        if(verbose_) printf("Read Error(no data): ID %d, data_length = %d\n", servo_id, read_length);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3]) {
        if(verbose_) printf("Read Error(header): ID %d\n", servo_id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint8_t id = read_data[4];
    if ( id != servo_id ) {
        if(verbose_) printf("Read Error(packet id) : ID %d\n", id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint8_t error = (uint8_t)read_data[8];
    if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
        if(verbose_) printf("Read Error(packet error) : ID %d\n", id);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }
    uint16_t sum_est = CalcChecksum(read_data, STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp);
    uint16_t sum_read = uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp]) |
        uint16_t(read_data[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp + 1]) << 8;
    if (sum_est != sum_read) {
        if(verbose_) printf("Read Error(crc): ID %d, est=%d, read=%d\n", servo_id, sum_est, sum_read);
        comm_error_last_read_ = true;
        return vector<int64_t>(dp_list.size(), 0);
    }

    // 正常なデータ
    hardware_error_last_read_ = ( error & 0x80 ); // error の最上位ビットが1のとき，ハードウェアエラーが発生している
    vector<int64_t> data_int_list(dp_list.size(), 0);
    for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
        const DynamixelAddress& dp = dp_list[i_dp];
        uint8_t index = dp.address() - dp_min.address();
        for(size_t i=0; i<dp.size(); i++) {
            data_read_[i] = read_data[9+index+i];
        }
        data_int_list[i_dp] = DecodeDataRead(dp.data_type());
    }
    return data_int_list;
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレス群から情報を読み込む
 * @param map<uint8_t, vector<DynamixelAddress>> id_dp_list_map 読み込むサーボIDとアドレス群のマップ
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボIDとデータ群のマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::BulkRead(const map<uint8_t, vector<DynamixelAddress>>& id_dp_list_map) {
    constexpr size_t max_read_servo = 100;
    constexpr size_t max_param_num = 10;
    if (id_dp_list_map.size() > max_read_servo) {
        if(verbose_) printf("Bulk Read Error(too many servo): servo num=%d > %d\n", (int)id_dp_list_map.size(), (int)max_read_servo);
        return map<uint8_t, vector<int64_t>>();
    }
    if (id_dp_list_map.empty()) return map<uint8_t, vector<int64_t>>();
    for (const auto& [id, dp_list] : id_dp_list_map) {
        if (dp_list.size() > max_param_num) {
            if(verbose_) printf("Bulk Read Error(too many param): ID %d, param num=%d > %d\n", id, (int)dp_list.size(), (int)max_param_num);
            return map<uint8_t, vector<int64_t>>();
        }
    }
    // 読み込むデータの範囲を決定, 連続していなくても許容
    struct BulkReadParam {
        uint8_t id;
        vector<DynamixelAddress> dp_list;
        DynamixelAddress dp_min;
        uint16_t size_total_dp;
    };
    vector<BulkReadParam> servo_param_list; servo_param_list.reserve(id_dp_list_map.size());
    for (const auto& [id, dp_list] : id_dp_list_map) {
        if (dp_list.empty()) continue;
        auto [dp_min_it, dp_max_it] = minmax_element(dp_list.begin(), dp_list.end(),
            [](const DynamixelAddress& a, const DynamixelAddress& b){ return a.address() < b.address(); });
        const DynamixelAddress dp_min = *dp_min_it;
        const DynamixelAddress dp_max = *dp_max_it;
        const uint16_t size_total_dp = static_cast<uint16_t>(dp_max.address() + dp_max.size() - dp_min.address());

        servo_param_list.emplace_back(BulkReadParam{id, dp_list, dp_min, size_total_dp});
    }
    if (servo_param_list.empty()) return map<uint8_t, vector<int64_t>>();

    const size_t num_servo = servo_param_list.size();
    const uint16_t param_size = static_cast<uint16_t>(num_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE));
    uint8_t send_data[COMMAND_TX_PACKET_SIZE + max_read_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE)] = {0};
    const uint16_t length = static_cast<uint16_t>(INSTRUCTION_SIZE + param_size + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = BROADCAST_ID;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_BULK_READ;
    for (size_t i_servo=0; i_servo<num_servo; i_servo++) {
        const size_t index = COMMAND_TX_PACKET_SIZE - CRC_SIZE + i_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE);
        send_data[index + 0] = servo_param_list[i_servo].id;
        send_data[index + 1] =  servo_param_list[i_servo].dp_min.address() & 0xFF;
        send_data[index + 2] = (servo_param_list[i_servo].dp_min.address() >> 8) & 0xFF;
        send_data[index + 3] =  servo_param_list[i_servo].size_total_dp & 0xFF;
        send_data[index + 4] = (servo_param_list[i_servo].size_total_dp >> 8) & 0xFF;
    }
    const uint16_t sum = CalcChecksum(send_data, COMMAND_TX_PACKET_SIZE - CRC_SIZE + param_size);
    send_data[8 + param_size] = sum & 0xFF;
    send_data[9 + param_size] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Bulk Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE + param_size);

    map<uint8_t, vector<int64_t>> id_data_vec_map;
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    int expected_len = 0; 
    for (const auto& param : servo_param_list) expected_len += STATUS_PACKET_SIZE + param.size_total_dp;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(expected_len));
    while (port_handler_->getBytesAvailable() < expected_len) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Bulk Read Error(time out): available bytes %d / %d\n", port_handler_->getBytesAvailable(), expected_len);
            deficient_byte_ = expected_len - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            break;
        }
        sleep_for(20us);
    }

    const int bytes_to_read = std::min(port_handler_->getBytesAvailable(), expected_len);
    if (bytes_to_read <= 0) return id_data_vec_map;

    vector<uint8_t> read_data(bytes_to_read, 0);
    int read_length = port_handler_->readPort(read_data.data(), bytes_to_read);
    if (read_length == -1) {
        if(verbose_) printf("Bulk Read Error(read port)\n");
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }
    if (read_length < expected_len) {
        timeout_last_read_ = true;
        deficient_byte_ = expected_len - read_length;
    }

    size_t read_offset = 0;
    for (size_t i_servo=0; i_servo<servo_param_list.size(); i_servo++) {
        const uint16_t size_total_dp = servo_param_list[i_servo].size_total_dp;
        const size_t packet_len = STATUS_PACKET_SIZE + size_total_dp;
        if (read_offset + packet_len > static_cast<size_t>(read_length)) break;
        uint8_t* packet = &read_data[read_offset];
        read_offset += packet_len;

        if (packet[0] != HEADER[0] or
            packet[1] != HEADER[1] or
            packet[2] != HEADER[2] or
            packet[3] != HEADER[3]) {
            if(verbose_) printf("Bulk Read Error(header): seq=%d\n", (int)i_servo);
            comm_error_last_read_ = true;
            continue;
        }
        const uint8_t id = packet[4];
        if (id != servo_param_list[i_servo].id) {
            if(verbose_) printf("Bulk Read Error(packet id): expected %d return %d\n", servo_param_list[i_servo].id, id);
            comm_error_last_read_ = true;
            continue;
        }
        const uint8_t error = packet[8];
        if (error & 0x7F) {
            if(verbose_) printf("Bulk Read Error(packet error): ID %d\n", id);
            comm_error_last_read_ = true;
            continue;
        }
        uint16_t sum_est = CalcChecksum(packet, STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp);
        uint16_t sum_read = uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp]) |
            uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp + 1]) << 8;
        if (sum_est != sum_read) {
            if(verbose_) printf("Bulk Read Error(crc): ID %d, est=%d, read=%d\n", id, sum_est, sum_read);
            comm_error_last_read_ = true;
            continue;
        }

        if (error & 0x80) hardware_error_last_read_ = true;
        if (error & 0x80) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(servo_param_list[i_servo].dp_list.size(), 0);
        for (size_t i_dp=0; i_dp<servo_param_list[i_servo].dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = servo_param_list[i_servo].dp_list[i_dp];
            const uint16_t index = dp.address() - servo_param_list[i_servo].dp_min.address();
            for (size_t i_data=0; i_data<dp.size(); i_data++) data_read_[i_data] = packet[9 + index + i_data];
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
    }
    return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレス群から情報を読み込む(高速版)
 * @param map<uint8_t, vector<DynamixelAddress>> id_dp_list_map 読み込むサーボIDとアドレス群のマップ
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボIDとデータ群のマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::BulkRead_fast(const map<uint8_t, vector<DynamixelAddress>>& id_dp_list_map) {
    constexpr size_t max_read_servo = 50;
    constexpr size_t max_param_num = 10;
    if (id_dp_list_map.size() > max_read_servo) {
        if(verbose_) printf("Fast Bulk Read Error(too many servo): servo num=%d > %d\n", (int)id_dp_list_map.size(), (int)max_read_servo);
        return map<uint8_t, vector<int64_t>>();
    }
    if (id_dp_list_map.empty()) return map<uint8_t, vector<int64_t>>();
    for (const auto& [id, dp_list] : id_dp_list_map) {
        if (dp_list.size() > max_param_num) {
            if(verbose_) printf("Fast Bulk Read Error(too many param): ID %d, param num=%d > %d\n", id, (int)dp_list.size(), (int)max_param_num);
            return map<uint8_t, vector<int64_t>>();
        }
    }
    struct BulkReadParam {
        uint8_t id;
        vector<DynamixelAddress> dp_list;
        DynamixelAddress dp_min;
        uint16_t size_total_dp;
    };
    vector<BulkReadParam> servo_param_list; servo_param_list.reserve(id_dp_list_map.size());
    // 読み込むデータの範囲を決定, 連続していなくても許容
    for (const auto& [id, dp_list] : id_dp_list_map) {
        if (dp_list.empty()) continue;
        auto [dp_min_it, dp_max_it] = minmax_element(dp_list.begin(), dp_list.end(),
            [](const DynamixelAddress& a, const DynamixelAddress& b){ return a.address() < b.address(); });
        const DynamixelAddress dp_min = *dp_min_it;
        const DynamixelAddress dp_max = *dp_max_it;
        const uint16_t size_total_dp = static_cast<uint16_t>(dp_max.address() + dp_max.size() - dp_min.address());

        servo_param_list.emplace_back(BulkReadParam{id, dp_list, dp_min, size_total_dp});
    }
    if (servo_param_list.empty()) return map<uint8_t, vector<int64_t>>();

    const size_t num_servo = servo_param_list.size();
    const uint16_t param_size = static_cast<uint16_t>(num_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE));
    uint8_t send_data[COMMAND_TX_PACKET_SIZE + max_read_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE)] = {0};
    const uint16_t length = static_cast<uint16_t>(INSTRUCTION_SIZE + param_size + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = BROADCAST_ID;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_FAST_BULK_READ;
    for (size_t i_servo=0; i_servo<num_servo; i_servo++) {
        const size_t index = COMMAND_TX_PACKET_SIZE - CRC_SIZE + i_servo * (ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE);
        send_data[index + 0] = servo_param_list[i_servo].id;
        send_data[index + 1] = servo_param_list[i_servo].dp_min.address() & 0xFF;
        send_data[index + 2] = (servo_param_list[i_servo].dp_min.address() >> 8) & 0xFF;
        send_data[index + 3] = servo_param_list[i_servo].size_total_dp & 0xFF;
        send_data[index + 4] = (servo_param_list[i_servo].size_total_dp >> 8) & 0xFF;
    }
    const uint16_t sum = CalcChecksum(send_data, COMMAND_TX_PACKET_SIZE - CRC_SIZE + param_size);
    send_data[8 + param_size] = sum & 0xFF;
    send_data[9 + param_size] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Fast Bulk Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, COMMAND_TX_PACKET_SIZE + param_size);

    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    int expected_len = FAST_SYNC_FRAME_PREFIX_SIZE;
    for (const auto& bulk_read_param : servo_param_list) expected_len += FAST_SYNC_PER_SERVO_OVERHEAD + bulk_read_param.size_total_dp;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(expected_len));
    while (port_handler_->getBytesAvailable() < expected_len) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Fast Bulk Read Error(time out): available bytes %d / %d\n", port_handler_->getBytesAvailable(), expected_len);
            deficient_byte_ = expected_len - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            return map<uint8_t, vector<int64_t>>();
        }
        sleep_for(20us);
    }

    vector<uint8_t> read_data(expected_len, 0);
    int read_length = port_handler_->readPort(read_data.data(), expected_len);
    map<uint8_t, vector<int64_t>> id_data_vec_map;
    if (read_length == -1) {
        if(verbose_) printf("Fast Bulk Read Error(read port)\n");
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }
    if (read_length < expected_len) {
        if(verbose_) printf("Fast Bulk Read Error(short read): expected %d return %d\n", expected_len, read_length);
        timeout_last_read_ = true;
        deficient_byte_ = expected_len - read_length;
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }
    if (read_data[0] != HEADER[0] ||
        read_data[1] != HEADER[1] ||
        read_data[2] != HEADER[2] ||
        read_data[3] != HEADER[3]) {
        if(verbose_) printf("Fast Bulk Read Error(header)\n");
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }
    if (read_data[4] != BROADCAST_ID) {
        if(verbose_) printf("Fast Bulk Read Error(broad cast id)\n");
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }

    size_t read_offset = FAST_SYNC_FRAME_PREFIX_SIZE;
    for (size_t i_servo=0; i_servo<servo_param_list.size(); i_servo++) {
        const uint16_t size_total_dp = servo_param_list[i_servo].size_total_dp;
        const size_t packet_len = FAST_SYNC_PER_SERVO_OVERHEAD + size_total_dp;
        if (read_offset + packet_len > static_cast<size_t>(read_length)) {
            if (verbose_) printf("Fast Bulk Read Error(packet length): seq=%d\n", (int)i_servo);
            comm_error_last_read_ = true;
            return map<uint8_t, vector<int64_t>>();
        }
        uint8_t error = read_data[read_offset + 0];
        uint8_t id = read_data[read_offset + 1];
        if (id != servo_param_list[i_servo].id) {
            if(verbose_) printf("Fast Bulk Read Error(packet id): expected %d return %d\n", servo_param_list[i_servo].id, id);
            comm_error_last_read_ = true;
            return map<uint8_t, vector<int64_t>>();
        }
        if (error & 0x7F) {
            if(verbose_) printf("Fast Bulk Read Error(packet error): ID %d\n", id);
            comm_error_last_read_ = true;
            read_offset += packet_len;
            continue;
        }

        if (error & 0x80) hardware_error_last_read_ = true;
        if (error & 0x80) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(servo_param_list[i_servo].dp_list.size(), 0);
        for (size_t i_dp=0; i_dp<servo_param_list[i_servo].dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = servo_param_list[i_servo].dp_list[i_dp];
            const uint16_t index = dp.address() - servo_param_list[i_servo].dp_min.address();
            for (size_t i_data=0; i_data<dp.size(); i_data++) {
                data_read_[i_data] = read_data[read_offset + ERROR_SIZE + ID_SIZE + index + i_data];
            }
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
        read_offset += packet_len;
    }
    return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレス群に情報を書き込む
 * @param map<uint8_t, vector<DynamixelAddress>> id_dp_list_map 書き込むサーボIDとアドレス群のマップ
 * @param map<uint8_t, vector<int64_t>> id_data_vec_map 書き込むサーボIDとデータ群のマップ
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::BulkWrite(const map<uint8_t, vector<DynamixelAddress>>& id_dp_list_map, const map<uint8_t, vector<int64_t>>& id_data_vec_map) {
    vector<uint8_t> servo_id_list; servo_id_list.reserve(id_dp_list_map.size());
    vector<vector<DynamixelAddress>> dp_lists; dp_lists.reserve(id_dp_list_map.size());
    vector<vector<int64_t>> data_vec_list; data_vec_list.reserve(id_dp_list_map.size());
    for (const auto& [id, dp_list] : id_dp_list_map) {
        if (!id_data_vec_map.count(id)) {
            if(verbose_) printf("Bulk Write Error(id data map): missing ID %d\n", id);
            return false;
        }
        servo_id_list.push_back(id);
        dp_lists.push_back(dp_list);
        data_vec_list.push_back(id_data_vec_map.at(id));
    }
    return BulkWrite(dp_lists, servo_id_list, data_vec_list);
}

/** @fn
 * @brief 複数のDynamixelのそれぞれ異なるアドレス群に情報を書き込む(簡易版)
 * @param vector<vector<DynamixelAddress>> dp_lists サーボごとのアドレス群
 * @param vector<uint8_t> servo_id_list 書き込むサーボIDリスト
 * @param vector<vector<int64_t>> data_vec_list サーボごとのデータ群
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::BulkWrite(const vector<vector<DynamixelAddress>>& dp_lists, const vector<uint8_t>& servo_id_list, const vector<vector<int64_t>>& data_vec_list) {
    constexpr size_t max_write_servo = 100;
    constexpr size_t max_param_num = 10;
    if (servo_id_list.size() > max_write_servo) {
        if(verbose_) printf("Bulk Write Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_write_servo);
        return false;
    }
    if (servo_id_list.size() != dp_lists.size() || servo_id_list.size() != data_vec_list.size()) {
        if (verbose_) printf("Bulk Write Error(mismatch id/addr/data num): id=%d, addr=%d, data=%d\n", (int)servo_id_list.size(), (int)dp_lists.size(), (int)data_vec_list.size());
        return false;
    }
    if (servo_id_list.empty()) return true;

    for (size_t i_servo=0; i_servo<servo_id_list.size(); i_servo++) {
        const uint8_t id = servo_id_list[i_servo];
        const auto& dp_list = dp_lists[i_servo];
        const auto& data_vec = data_vec_list[i_servo];
        if (dp_list.size() > max_param_num) {
            if(verbose_) printf("Bulk Write Error(too many param): ID %d, param num=%d > %d\n", id, (int)dp_list.size(), (int)max_param_num);
            return false;
        }
        if (dp_list.size() != data_vec.size()) {
            if(verbose_) printf("Bulk Write Error(mismatch param and data num): ID %d, param num=%d, data num=%d\n", id, (int)dp_list.size(), (int)data_vec.size());
            return false;
        }
        for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
            if (dp_list[i_dp].is_dummy()) {
                if (verbose_) printf("Bulk Write Error(dummy address is readonly): ID %d\n", id);
                return false;
            }
            if (i_dp+1<dp_list.size() && dp_list[i_dp].address() + dp_list[i_dp].size() != dp_list[i_dp+1].address()) {
                if(verbose_) printf("Bulk Write Error(address is not continuous): ID %d, addr1=%d (%d), addr2=%d\n", id, dp_list[i_dp].address(), dp_list[i_dp].size(), dp_list[i_dp+1].address());
                return false;
            }
        }
    }

    struct BulkWriteParam {
        uint8_t id;
        uint16_t dp_start_addr;
        vector<uint8_t> write_data;
        size_t param_offset;
    };
    vector<BulkWriteParam> servo_param_list; servo_param_list.reserve(servo_id_list.size());
    size_t total_param_size = 0;
    for (size_t i_servo=0; i_servo<servo_id_list.size(); i_servo++) {
        const DynamixelAddress dp_min = *dp_lists[i_servo].begin();
        const DynamixelAddress dp_max = *dp_lists[i_servo].rbegin();
        const uint16_t size_total_dp = static_cast<uint16_t>(dp_max.address() + dp_max.size() - dp_min.address());
        vector<uint8_t> write_data(size_total_dp, 0);
        for (size_t i_dp=0; i_dp<dp_lists[i_servo].size(); i_dp++) {
            const DynamixelAddress& dp = dp_lists[i_servo][i_dp];
            const uint16_t index = dp.address() - dp_min.address();
            EncodeDataWrite(dp.data_type(), data_vec_list[i_servo][i_dp]);
            for (size_t i_data=0; i_data<dp.size(); i_data++) write_data[index + i_data] = data_write_[i_data];
        }
        servo_param_list.push_back(BulkWriteParam{servo_id_list[i_servo], dp_min.address(), write_data, total_param_size});
        total_param_size += ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE + write_data.size();
    }

    vector<uint8_t> send_data(COMMAND_TX_PACKET_SIZE + total_param_size, 0);
    uint16_t length = static_cast<uint16_t>(INSTRUCTION_SIZE + total_param_size + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = BROADCAST_ID;
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_BULK_WRITE;
    for (size_t i_servo=0; i_servo<servo_param_list.size(); i_servo++) {
        const size_t index = COMMAND_TX_PACKET_SIZE - CRC_SIZE + servo_param_list[i_servo].param_offset;
        send_data[index + 0] = servo_param_list[i_servo].id;
        send_data[index + 1] = servo_param_list[i_servo].dp_start_addr & 0xFF;
        send_data[index + 2] = (servo_param_list[i_servo].dp_start_addr >> 8) & 0xFF;
        send_data[index + 3] = servo_param_list[i_servo].write_data.size() & 0xFF;
        send_data[index + 4] = (servo_param_list[i_servo].write_data.size() >> 8) & 0xFF;
        for (size_t i_data=0; i_data<servo_param_list[i_servo].write_data.size(); i_data++)
            send_data[index + ID_SIZE + ADDR_SIZE + DATA_LEN_SIZE + i_data] = servo_param_list[i_servo].write_data[i_data];
    }
    uint16_t sum = CalcChecksum(send_data.data(), COMMAND_TX_PACKET_SIZE - CRC_SIZE + total_param_size);
    send_data[8 + total_param_size] = sum & 0xFF;
    send_data[9 + total_param_size] = (sum>>8) & 0xFF;

    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data.data(), static_cast<int>(send_data.size()));
    return true;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::SyncRead(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list) {
    constexpr size_t max_read_servo = 100;
    constexpr size_t max_param_num = 10;
    if (servo_id_list.size() > max_read_servo) {
        if(verbose_) printf("Sync Read Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_read_servo);
        return map<uint8_t, vector<int64_t>>();
    }
    if (dp_list.size() > max_param_num) {
        if(verbose_) printf("Sync Read Error(too many param): param num=%d > %d\n", (int)dp_list.size(), (int)max_param_num);
        return map<uint8_t, vector<int64_t>>();
    }
    if (servo_id_list.empty() || dp_list.empty()) return map<uint8_t, vector<int64_t>>();
    // 読み込むデータの範囲を決定, 連続していなくても許容
    auto [dp_min_it, dp_max_it] = minmax_element(dp_list.begin(), dp_list.end(),
        [](const DynamixelAddress& a, const DynamixelAddress& b){ return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_min_it;
    DynamixelAddress dp_max = *dp_max_it;
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

    const size_t num_servo = servo_id_list.size();
    uint8_t send_data[READ_TX_PACKET_SIZE + max_read_servo] = {0}; // 読み込むサーボの数によって変わるが， 100+14=114 100サーボに同時に読み込むことはないので十分
    uint16_t length = static_cast<uint16_t>(num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = HEADER[2];
    send_data[3] = HEADER[3];
    send_data[4] = 0xFE;  // id
    send_data[5] = length & 0xFF;
    send_data[6] = (length>>8) & 0xFF;
    send_data[7] = INSTRUCTION_SYNC_READ;  // instruction
    send_data[8] = dp_min.address() & 0xFF;
    send_data[9] = (dp_min.address()>>8) & 0xFF;
    send_data[10] = size_total_dp & 0xFF;
    send_data[11] = (size_total_dp>>8) & 0xFF;
    for(size_t i_servo=0; i_servo<num_servo; i_servo++) 
        send_data[12+i_servo] = servo_id_list[i_servo];
    uint16_t sum = CalcChecksum(send_data, 12+num_servo);
    send_data[12+num_servo] = sum & 0xFF;
    send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
    clear_port_with_drain(port_handler_);
    port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo);

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<READ_TX_PACKET_SIZE + num_servo; i++) printf("%02X ", send_data[i]); printf("\n");}

    // データ読み込みの処理
    map<uint8_t, vector<int64_t>> id_data_vec_map;
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    comm_error_last_read_ = false;
    const uint8_t packet_len = STATUS_PACKET_SIZE + size_total_dp;
    const int expected_len = packet_len * num_servo;

    timeout_last_read_ = false;
    port_handler_->setPacketTimeout(uint16_t(expected_len));
    while (port_handler_->getBytesAvailable() < expected_len) {
        if (port_handler_->isPacketTimeout()) {
            if(verbose_) printf("Sync Read Error(time out): available bytes %d / %d\n", port_handler_->getBytesAvailable(), expected_len);
            deficient_byte_ = expected_len - port_handler_->getBytesAvailable();
            timeout_last_read_ = true;
            break;
        }
        sleep_for(20us);
    }

    const int bytes_to_read = std::min(port_handler_->getBytesAvailable(), expected_len);
    if (bytes_to_read <= 0) return id_data_vec_map;

    std::vector<uint8_t> read_data(bytes_to_read, 0);
    int read_length = port_handler_->readPort(read_data.data(), bytes_to_read);
    if (read_length == -1) {
        if(verbose_) printf("Sync Read Error(read port)\n");
        comm_error_last_read_ = true;
        return id_data_vec_map;
    }

    const int num_packets_read = read_length / packet_len;
    if (num_packets_read < static_cast<int>(num_servo)) {
        timeout_last_read_ = true;
        deficient_byte_ = expected_len - read_length;
    }

    for (int i_servo = 0; i_servo < num_packets_read; i_servo++) {
        uint8_t* packet = &read_data[i_servo * packet_len];

        if (packet[0] != HEADER[0] or
            packet[1] != HEADER[1] or
            packet[2] != HEADER[2] or
            packet[3] != HEADER[3]) {
            if(verbose_) printf("Sync Read Error(header): seq=%d\n", i_servo);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t id = packet[4];
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Sync Read Error(packet id) : est ID=%d, read ID=%d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            continue;
        }
        uint8_t error = (uint8_t)packet[8];
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue;
        }
        uint16_t sum_est = CalcChecksum(packet, STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp);
        uint16_t sum_read = uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp]) |
            uint16_t(packet[STATUS_PACKET_SIZE - CRC_SIZE + size_total_dp + 1]) << 8;
        if (sum_est != sum_read) {
            if(verbose_) printf("Sync Read Error(crc): ID %d, est=%d, read=%d\n", id, sum_est, sum_read);
            comm_error_last_read_ = true;
            continue;
        }

        // 正常なデータ
        if ( error & 0x80 ) hardware_error_last_read_ = true ; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if ( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(dp_list.size());
        for (size_t i_dp = 0; i_dp < dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = dp_list[i_dp];
            uint8_t index = dp.address() - dp_min.address();
            for(size_t i = 0; i < dp.size(); i++) data_read_[i] = packet[9 + index + i];
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
    }
    return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスから情報を読み込む,パケットを最小限にすることで高速化したもの．サーボ1つでも失敗するとすべて読み込めない
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 読み込むサーボのIDのリスト
 * @return map<uint8_t, vector<int64_t>> 読み込んだサーボのIDとデータのマップ
 */
map<uint8_t, vector<int64_t>> DynamixelCommunicator::SyncRead_fast(const vector<DynamixelAddress>& dp_list, const vector<uint8_t>& servo_id_list) {
    constexpr size_t max_read_servo = 50;
    constexpr size_t max_addr_range = 25;
    constexpr size_t max_param_num = 10;
    if ( servo_id_list.size() > max_read_servo ) {
        if(verbose_) printf("Fast Sync Read Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_read_servo);
        return map<uint8_t, vector<int64_t>>();
    }
    if ( dp_list.size() > max_param_num ) {
        if(verbose_) printf("Fast Sync Read Error(too many param): param num=%d > %d\n", (int)dp_list.size(), (int)max_param_num);
        return map<uint8_t, vector<int64_t>>();
    }    
    // 読み込むデータの範囲を決定, 連続していなくても許容
    auto [dp_min_it, dp_max_it] = minmax_element(dp_list.begin(), dp_list.end(),
        [](const DynamixelAddress& a, const DynamixelAddress& b){ return a.address() < b.address(); });
    DynamixelAddress dp_min = *dp_min_it;
    DynamixelAddress dp_max = *dp_max_it;
    auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();

	uint8_t send_data[READ_TX_PACKET_SIZE + max_read_servo] = {0}; // 読み込むサーボの数によって変わるが， 50+14=64 50サーボに同時に読み込むことはないので十分 
	const size_t num_servo = servo_id_list.size();
	uint16_t length = static_cast<uint16_t>(num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
	send_data[0] = HEADER[0];
	send_data[1] = HEADER[1];
	send_data[2] = HEADER[2];
	send_data[3] = HEADER[3];
	send_data[4] = 0xFE;  // id
	send_data[5] = length & 0xFF;
	send_data[6] = (length>>8) & 0xFF;
	send_data[7] = INSTRUCTION_FAST_SYNC_READ;  // instruction
	send_data[8] = dp_min.address() & 0xFF;
	send_data[9] = (dp_min.address()>>8) & 0xFF;
	send_data[10] = size_total_dp & 0xFF;
	send_data[11] = (size_total_dp>>8) & 0xFF;
	for(size_t i_servo=0; i_servo<num_servo; i_servo++)
		send_data[12+i_servo] = servo_id_list[i_servo];
	uint16_t sum = CalcChecksum(send_data, 12+num_servo);
	send_data[12+num_servo] = sum & 0xFF;
	send_data[13+num_servo] = (sum>>8) & 0xFF;

    port_handler_->setPacketTimeout( uint16_t(deficient_byte_) );
    while( timeout_last_read_ && port_handler_->getBytesAvailable() < deficient_byte_) {
        if (port_handler_->isPacketTimeout()) { 
            if(verbose_) printf("Sync Read fast Warn: clear buffer time out\n");
            break;
        }
        sleep_for(20us);
    }
	clear_port_with_drain(port_handler_);
	port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo);

    // データ読み込みの処理
 	uint8_t length_a_servo = FAST_SYNC_PER_SERVO_OVERHEAD + size_total_dp;
	uint16_t length_read_data = FAST_SYNC_FRAME_PREFIX_SIZE + length_a_servo * num_servo;
    timeout_last_read_ = false;
	port_handler_->setPacketTimeout( uint16_t(length_read_data) );
	while(port_handler_->getBytesAvailable() < length_read_data) {
		if (port_handler_->isPacketTimeout() ) {
		if(verbose_) printf("Fast Sync Read Error(time out) : available bytes %d / %d\n", port_handler_->getBytesAvailable(), length_read_data);
        deficient_byte_ = length_read_data - port_handler_->getBytesAvailable();
		timeout_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
		}
        sleep_for(20us);
	}

	// パケットの読み込み
    uint8_t read_data[FAST_SYNC_FRAME_PREFIX_SIZE + (max_addr_range + FAST_SYNC_PER_SERVO_OVERHEAD) * max_read_servo] = {0}; // 読み込むサーボとデータサイズによって変わるが， 最大でも同時読み込みしたいアドレスの幅は25程度なので，(25+4)*50+8=1458;
    int read_length = port_handler_->readPort(read_data, length_read_data);

    // if(verbose_) {printf("write:" ); for (size_t i=0; i<READ_TX_PACKET_SIZE + num_servo; i++) printf("%02X ", send_data[i]); printf("\n");}
    // if(verbose_) {printf("read:" ); for (size_t i=0; i<read_length; i++) printf("%02X ", read_data[i]); printf("\n");}

    // 全体のエラーチェック
    comm_error_last_read_ = false;
	if (read_length == -1) {
		if(verbose_) printf("Fast Sync Read Error(read port)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットのヘッダーの確認
	if (read_data[0] != HEADER[0] ||
		read_data[1] != HEADER[1] ||
		read_data[2] != HEADER[2] ||
		read_data[3] != HEADER[3]) {
		if(verbose_) printf("Fast Sync Read Error(header)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットのIDがブロードキャスト用のものか確認
	if ( read_data[4] != 0xFE ) {
		if(verbose_) printf("Fast Sync Read Error(broad cast id)\n");
		comm_error_last_read_ = true;
		return map<uint8_t, vector<int64_t>>();
	}
	// 読み込めたパケットを個々のサーボのパケットに分割して処理,チェックサムの処理は行わない
    hardware_error_last_read_ = false;
    hardware_error_id_last_read_.clear();
    map<uint8_t, vector<int64_t>> id_data_vec_map;
	for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
		uint8_t id = (uint8_t)read_data[9 + i_servo*length_a_servo]; // servo id
        if ( id != servo_id_list[i_servo] ) {
            if(verbose_) printf("Fast Sync Read Error(packet id) : expected %d, return %d\n", servo_id_list[i_servo], id);
            comm_error_last_read_ = true;
            return map<uint8_t, vector<int64_t>>(); // これ以降すべての読み込みを諦める．
        }
		uint8_t error = (uint8_t)read_data[8 + i_servo*length_a_servo];
        if ( error & 0x7F ) { // error の最上位ビット以外が1のとき，通信状態の異常がある
            if(verbose_) printf("Fast Sync Read Error(packet error) : ID %d\n", id);
            comm_error_last_read_ = true;
            continue; // 他のidのデータは生きている可能性があるので続行
        }
		// 正常なデータ, hardware error は通信の可否に影響はないので以降でチェックする．
        if( error & 0x80 ) hardware_error_last_read_ = true; // error の最上位ビットが1のとき，ハードウェアエラーが発生している
        if( error & 0x80 ) hardware_error_id_last_read_.push_back(id);
        id_data_vec_map[id].resize(dp_list.size(), 0);
        for (size_t i_dp=0; i_dp<dp_list.size(); i_dp++) {
            const DynamixelAddress& dp = dp_list[i_dp];
            uint8_t index = dp.address() - dp_min.address();
            for(size_t i=0; i<dp.size(); i++) data_read_[i] = read_data[10+i_servo*length_a_servo+index+i];
            id_data_vec_map[id][i_dp] = DecodeDataRead(dp.data_type());
        }
	}
	return id_data_vec_map;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報を書き込む
 * @param vector<DynamixelAddress> dp_list 対象のパラメータのインスタンスの配列
 * @param vector<uint8_t> servo_id_list 書き込むサーボのIDのリスト
 * @param vector<vector<int64_t>> data_vec_list 書き込むデータのリスト．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(const vector<DynamixelAddress>& dp_list_sorted, const vector<uint8_t>& servo_id_list, const vector<vector<int64_t>>& data_vec_list) {
  constexpr size_t max_read_servo = 50;
  constexpr size_t max_addr_range = 25;
  constexpr size_t max_param_num = 10;
  constexpr size_t max_write_servo = 100;
  if (dp_list_sorted.size() > max_param_num) {
    if(verbose_) printf("Sync Write Error(too many param): param num=%d > %d\n", (int)dp_list_sorted.size(), (int)max_param_num);
    return false;
  }
  if (servo_id_list.size() > max_write_servo) {
    if(verbose_) printf("Sync Write Error(too many servo): servo num=%d > %d\n", (int)servo_id_list.size(), (int)max_write_servo);
    return false;
  }
  if (servo_id_list.size() != data_vec_list.size()) {
    if(verbose_) printf("Sync Write Error(mismatch servo and data_vec num): servo num=%d, data_vec num=%d\n", (int)servo_id_list.size(), (int)data_vec_list.size());
    return false;
  }
  for (const auto& data_vec : data_vec_list) if ( dp_list_sorted.size() != data_vec.size() ) {
    if(verbose_) printf("Sync Write Error(mismatch param and data num): param num=%d, data num=%d\n", (int)servo_id_list.size(), (int)data_vec.size());
    return false;
  }
  // 書き込むデータの範囲を決定, ソート済みかつdummyでないアドレスが連続していることを確認
  DynamixelAddress dp_min = *dp_list_sorted.begin();
  DynamixelAddress dp_max = *dp_list_sorted.rbegin();
  for (size_t i=0; i<dp_list_sorted.size(); i++) {
    if (dp_list_sorted[i].is_dummy()) {
      if (verbose_) printf("Sync Write Error(dummy address is readonly)\n");
      return false;
    }// dummy addressは書き込まない  
    if (i+1<dp_list_sorted.size() && dp_list_sorted[i].address() + dp_list_sorted[i].size() != dp_list_sorted[i+1].address()) {
      if(verbose_) printf("Sync Write Error(address is not continuous): addr1=%d (%d), addr2=%d\n", dp_list_sorted[i].address(), dp_list_sorted[i].size(), dp_list_sorted[i+1].address());
      return false;
    }
  }
  auto size_total_dp = dp_max.address() + dp_max.size() - dp_min.address();
  uint8_t send_data[READ_TX_PACKET_SIZE + (max_addr_range + 1) * max_read_servo] = {0}; // 書き込むdpの数とサイズとサーボの数によって変わるが， 最大でも同時読み込みしたいアドレスの幅は25程度なので，(25+1)*50+14=1314
  const size_t num_servo = servo_id_list.size();
  uint16_t length = static_cast<uint16_t>((1+size_total_dp) * num_servo + INSTRUCTION_SIZE + ADDR_SIZE + DATA_LEN_SIZE + CRC_SIZE);
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_WRITE;  // instruction
  send_data[8] = dp_min.address() & 0xFF;
  send_data[9] = (dp_min.address()>>8) & 0xFF;
  send_data[10] = size_total_dp & 0xFF;
  send_data[11] = (size_total_dp>>8) & 0xFF;
  for(size_t i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo*(size_total_dp+1)] = servo_id_list[i_servo];
    for (size_t i_dp=0; i_dp<dp_list_sorted.size(); i_dp++) {
      const DynamixelAddress& dp = dp_list_sorted[i_dp];
      uint8_t index = dp.address() - dp_min.address();
      EncodeDataWrite(dp.data_type(), data_vec_list[i_servo][i_dp]);
      for(size_t i_data=0; i_data<dp.size(); i_data++) {
        send_data[12+i_servo*(size_total_dp+1)+1+index+i_data] = data_write_[i_data];
      }
    }
  }
  uint16_t sum = CalcChecksum(send_data, 12+num_servo*(size_total_dp+1));
  send_data[12+num_servo*(size_total_dp+1)] = sum & 0xFF;
  send_data[13+num_servo*(size_total_dp+1)] = (sum>>8) & 0xFF;


  clear_port_with_drain(port_handler_);
  port_handler_->writePort(send_data, READ_TX_PACKET_SIZE + num_servo * (size_total_dp + 1));
  return true;
}

/** @fn
 * @brief 複数のDynamixelの同一のアドレスに情報書き込む
 * @param DynamixelAddress dp 対象のパラメータのインスタンス
 * @param map<uint8_t, int64_t> id_data_int_map 書き込むサーボのIDと書き込むデータのマップ
 * @return  bool 通信成功判定
 */
bool DynamixelCommunicator::SyncWrite(const vector<DynamixelAddress>& dp_list, const map<uint8_t, vector<int64_t>>& id_data_int_map) {
    vector<uint8_t>         servo_id_list; servo_id_list.reserve(id_data_int_map.size());
    vector<vector<int64_t>> data_vec_list; data_vec_list.reserve(id_data_int_map.size());
    for (const auto& id_data : id_data_int_map) {
        servo_id_list.push_back(id_data.first);    // キー（サーボID）を追加
        data_vec_list.push_back(id_data.second);   // 値（データ）を追加
    }
    return SyncWrite( dp_list, servo_id_list, data_vec_list );
}
