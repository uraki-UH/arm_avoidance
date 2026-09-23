#pragma once

#include "../utils/utils.hpp"

class Auth {
    public:
        Auth();
        ~Auth();

        unsigned char req[AUTH_REQUEST_LEN];
        bool getRequest(unsigned char *req_code);
        bool checkResponse(const unsigned char* res, const size_t res_len, const uint32_t *key1, const uint32_t *key2, const uint32_t *key3, const uint32_t *key4, const uint32_t *key5);
        bool checkFile(const char *binary_path, const uint32_t *key1, const uint32_t *key2, const uint32_t *key3, const uint32_t *key4, const uint32_t *key5);


        const uint32_t ykey0[4] = {_YK_KEY0_1, _YK_KEY0_2, _YK_KEY0_3, _YK_KEY0_4};
        const uint32_t ykey6[4] = {_YK_KEY6_1, _YK_KEY6_2, _YK_KEY6_3, _YK_KEY6_4};

        const uint32_t fkey0[4] = {_FILE_KEY0_1, _FILE_KEY0_2, _FILE_KEY0_3, _FILE_KEY0_4};
        const uint32_t fkey6[4] = {_FILE_KEY6_1, _FILE_KEY6_2, _FILE_KEY6_3, _FILE_KEY6_4};
};