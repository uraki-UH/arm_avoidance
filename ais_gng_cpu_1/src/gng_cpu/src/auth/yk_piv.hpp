#include <future>
#include "../utils/utils.hpp"
#include <ykpiv/ykpiv.h>

class YK_PIV {
    public:
    //  uint32_t yubikey_serial_number = 0; // Yubikeyのシリアル番号
     YK_PIV();
     ~YK_PIV();

     bool init();
     bool challenge(const unsigned char *req, unsigned char *res, size_t *res_len);
     bool challengeAsync(const unsigned char *req);
     bool checkFinished(unsigned char *res, size_t *res_len);
     void clearState(){ wait_for_response = false; }

    private:
        bool wait_for_response = false;
        ykpiv_state *state_ = nullptr;
        ykpiv_rc rc_;
        std::future<std::vector<unsigned char>> future_result_;
        std::vector<unsigned char> ykpiv_sign_task(ykpiv_state *state, const std::array<unsigned char, AUTH_REQUEST_LEN> req);
};