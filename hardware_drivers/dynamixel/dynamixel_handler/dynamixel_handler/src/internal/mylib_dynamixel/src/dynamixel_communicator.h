#ifndef DYNAMIXEL_COMMUNICATOR_H_
#define DYNAMIXEL_COMMUNICATOR_H_

#include <map>
using std::map;
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <cmath>
#include <algorithm>

#include "../download/port_handler_fix.h"
#include "dynamixel_parameters.h"


class DynamixelCommunicator {
    public:
        DynamixelCommunicator() { }
        DynamixelCommunicator(const char *port_name, int baudrate, int latency_timer = 16) {
            GetPortHandler(port_name);
            set_baudrate(baudrate);
            set_latency_timer(latency_timer);
        }

        void GetPortHandler(const char *port_name) {
            port_name_ = port_name;
            port_handler_ = PortHandler::getPortHandler(port_name);
        }

        void set_baudrate(int baudrate) { baudrate_ = baudrate; }
        void set_latency_timer(uint8_t latency_timer) { latency_timer_ = latency_timer; }
        void set_status_return_level(int level) { status_return_level_ = level; }
        void set_verbose(bool verbose) { verbose_ = verbose; }

        string port_name(){ return string(port_name_); }

        bool timeout_last_read(){ return timeout_last_read_; }
        bool comm_error_last_read(){ return comm_error_last_read_; }
        bool hardware_error_last_read(){ return hardware_error_last_read_; }
        vector<uint8_t>& hardware_error_id_last_read() { return hardware_error_id_last_read_; }
        map<uint8_t, uint16_t>& ping_id_model_map_last_read() { return ping_id_model_map_last_read_; }
        double dead_time_retry_ping(){ return num_try_ * latency_timer_; }
        double wait_time_ping_broadcast(){ return msec_watch_ping_broadcast_; }

        void set_retry_config(int num_try, int msec_interval){
            num_try_ = std::max(num_try, 1);
            msec_interval_ = std::max(msec_interval, 1);
        }

        bool OpenPort();
        bool ClosePort();
        void Reboot(uint8_t servo_id);
        void FactoryReset(uint8_t servo_id, FactoryResetLevel level);
        bool Ping(uint8_t servo_id);
        vector<uint8_t> Ping_broadcast();

        // パラメータ1つ
        bool Write    (DynamixelAddress, uint8_t               , int64_t               );
        bool SyncWrite(DynamixelAddress, const vector<uint8_t>&, const vector<int64_t>&);
        bool SyncWrite(DynamixelAddress, const map<uint8_t, int64_t>&);
        int64_t               Read         (DynamixelAddress, uint8_t               );
        map<uint8_t, int64_t> BulkRead     (const map<uint8_t, DynamixelAddress>&);
        map<uint8_t, int64_t> BulkRead_fast(const map<uint8_t, DynamixelAddress>&);
        map<uint8_t, int64_t> SyncRead     (DynamixelAddress, const vector<uint8_t>&);
        map<uint8_t, int64_t> SyncRead_fast(DynamixelAddress, const vector<uint8_t>&);
        bool BulkWrite(const map<uint8_t, DynamixelAddress>&, const map<uint8_t, int64_t>&);
        bool BulkWrite(const vector<DynamixelAddress>&, const vector<uint8_t>&, const vector<int64_t>&);
        // 複数パラメータ同時
        bool Write    (const vector<DynamixelAddress>&, uint8_t               , const vector<int64_t>&        );
        bool SyncWrite(const vector<DynamixelAddress>&, const vector<uint8_t>&, const vector<vector<int64_t>>&);
        bool SyncWrite(const vector<DynamixelAddress>&, const map<uint8_t, vector<int64_t>>&);
        vector<int64_t>               Read         (const vector<DynamixelAddress>&, uint8_t               );
        map<uint8_t, vector<int64_t>> BulkRead     (const map<uint8_t, vector<DynamixelAddress>>&);
        map<uint8_t, vector<int64_t>> BulkRead_fast(const map<uint8_t, vector<DynamixelAddress>>&);
        map<uint8_t, vector<int64_t>> SyncRead     (const vector<DynamixelAddress>&, const vector<uint8_t>&);
        map<uint8_t, vector<int64_t>> SyncRead_fast(const vector<DynamixelAddress>&, const vector<uint8_t>&);
        bool BulkWrite(const map<uint8_t, vector<DynamixelAddress>>&, const map<uint8_t, vector<int64_t>>&);
        bool BulkWrite(const vector<vector<DynamixelAddress>>&, const vector<uint8_t>&, const vector<vector<int64_t>>&);
        // Retry付
        bool            tryPing (uint8_t servo_id);
        bool            tryWrite(DynamixelAddress, uint8_t, int64_t);
        int64_t         tryRead (DynamixelAddress, uint8_t);
        bool            tryWrite(const vector<DynamixelAddress>&, uint8_t, const vector<int64_t>&);
        vector<int64_t> tryRead (const vector<DynamixelAddress>&, uint8_t);
        
    private:
        static uint16_t CalcChecksum(const uint8_t* data, size_t length);
        void   EncodeDataWrite(DynamixelDataType type, int64_t data_int);
        int64_t DecodeDataRead(DynamixelDataType type);

        const char *port_name_;
        PortHandler *port_handler_;
        uint32_t baudrate_;
        uint8_t latency_timer_ = 16;
        uint8_t status_return_level_ = STATUS_RETURN_LEVEL_ALL;
        uint8_t  data_write_[4];
        uint8_t  data_read_[4];

        bool timeout_last_read_        = false; // 直前の通信でタイムアウトが発生したことを示すフラグ
        bool comm_error_last_read_     = false; // 直前の通信に何らかの異常が発生したことを示すフラグ
        bool hardware_error_last_read_ = false; // ハードウェアエラーの有無を示すフラグ status packetのerror部分から取得する
        vector<uint8_t> hardware_error_id_last_read_; // ハードウェアエラーが発生したサーボのIDリスト
        map<uint8_t, uint16_t> ping_id_model_map_last_read_; // 直前のPingで取得した ID->model_number
        
        bool verbose_ = false;
        uint8_t num_try_       = 5;
        uint8_t msec_interval_ = 5;
        int32_t deficient_byte_ = 0;
        double msec_watch_ping_broadcast_ = 800.0;
};

#endif /* DYNAMIXEL_COMMUNICATOR_H_ */
