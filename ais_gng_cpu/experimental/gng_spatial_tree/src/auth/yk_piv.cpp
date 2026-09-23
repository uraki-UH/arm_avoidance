#include "yk_piv.hpp"

YK_PIV::YK_PIV(){

}

YK_PIV::~YK_PIV(){
    if(state_){
        ykpiv_done(state_);
    }
}

std::vector<unsigned char> YK_PIV::ykpiv_sign_task(ykpiv_state *state, const std::array<unsigned char, AUTH_REQUEST_LEN> req){
    unsigned char res[AUTH_RESPONSE_LEN];
    size_t res_len = AUTH_RESPONSE_LEN;
    auto rc = ykpiv_sign_data(state, req.data(), AUTH_REQUEST_LEN, res, &res_len, YKPIV_ALGO_ECCP256, YKPIV_KEY_CARDAUTH);
    if(rc == YKPIV_OK){
        return std::vector<unsigned char>(res, res + res_len);
    }
    return std::vector<unsigned char>();
}

bool YK_PIV::init(){
    // Yubikeyの初期化
    rc_ = ykpiv_init(&state_, 0);
    if (rc_ != YKPIV_OK) {
        // RCLCPP_ERROR(logger, "Failed to initialize libykpiv: %s", ykpiv_strerror(rc_));
        return false;
    }

    // Yubikeyを開く
    rc_ = ykpiv_connect(state_, "");
    if (rc_ != YKPIV_OK) {
        // RCLCPP_ERROR(logger, "Failed to connect to YubiKey: %s", ykpiv_strerror(rc_));
        ykpiv_done(state_);
        return false;
    }
    
    // Yubikeyのシリアル番号を確認
    // uint32_t serial;
    // rc_ = ykpiv_get_serial(state_, &serial);
    // if (rc_ != YKPIV_OK) {
    //     // RCLCPP_ERROR(logger, "Failed to get Yubikey serial number: %s", ykpiv_strerror(rc_));
    //     return false;
    // }

    // if(serial != yubikey_serial_number){
    //     // RCLCPP_ERROR(logger, "Yubikey serial number mismatch: expected %u, got %u", yubikey_serial_number, serial);
    //     return false;
    // }
    return true;
}
bool YK_PIV::challenge(const unsigned char *req, unsigned char *res, size_t *res_len){
    if(wait_for_response){
        return false;
    }
    *res_len = AUTH_RESPONSE_LEN;
    rc_ = ykpiv_sign_data(state_, req, AUTH_REQUEST_LEN, res, res_len, YKPIV_ALGO_ECCP256, YKPIV_KEY_CARDAUTH);
    return (rc_ == YKPIV_OK);
}

bool YK_PIV::challengeAsync(const unsigned char *req) {
    // 処理中
    if(wait_for_response){
        return false;
    }
    std::array<unsigned char, AUTH_REQUEST_LEN> req_array;
    std::copy(req, req + AUTH_REQUEST_LEN, req_array.begin());
    future_result_ = std::async(std::launch::async, &YK_PIV::ykpiv_sign_task, this, state_, req_array);
    wait_for_response = true;
    return true;
}

bool YK_PIV::checkFinished(unsigned char *res, size_t *res_len) {
    if (future_result_.valid() && 
        future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto response = future_result_.get();
        size_t len = response.size();
        wait_for_response = false;
        if (len <= AUTH_RESPONSE_LEN) {
            memcpy(res, response.data(),len);
            *res_len = len;
        }
        return true;
    }
    return false;
}