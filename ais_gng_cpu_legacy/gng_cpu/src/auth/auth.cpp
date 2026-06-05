#include "auth.hpp"

#include <openssl/rand.h>
#include <openssl/evp.h>
#include <openssl/ec.h>
#include <openssl/ecdsa.h>
#include <openssl/obj_mac.h>
#include <openssl/x509.h>
#include <openssl/sha.h> // SHA256_DIGEST_LENGTH と SHA256() のため
#include <openssl/err.h>   // エラー表示のため
#include <sys/stat.h>   // stat
#include <fcntl.h>      // open
#include <unistd.h>     // read, close
#include <stdlib.h>     // malloc, free
#include <filesystem>
#include <fstream>

Auth::Auth(){

}

Auth::~Auth(){

}

bool Auth::getRequest(unsigned char *req_code){
    if(req_code == nullptr){
        return false;
    }
    while(RAND_bytes(req, AUTH_REQUEST_LEN) != 1) {
        // RAND_bytes failed, retry
    }
    memcpy(req_code, req, AUTH_REQUEST_LEN);
    return true;
}

bool Auth::checkResponse(const unsigned char* res, const size_t res_len, const uint32_t *ykey1, const uint32_t *ykey2, const uint32_t *ykey3, const uint32_t *ykey4, const uint32_t *ykey5) {
    if(res == nullptr){
        return false;
    }
    // 鍵を作成
    uint32_t p[24] = {0};
    for (int n = 0; n < 4; ++n) p[n + 4*5] = ykey6[n] ^ ykey5[n];
    for (int n = 0; n < 4; ++n) p[n + 4*4] = ykey5[n] ^ ykey4[n];
    for (int n = 0; n < 4; ++n) p[n + 4*3] = ykey4[n] ^ ykey3[n];
    for (int n = 0; n < 4; ++n) p[n + 4*2] = ykey3[n] ^ ykey2[n];
    for (int n = 0; n < 4; ++n) p[n + 4*1] = ykey2[n] ^ ykey1[n];
    for (int n = 0; n < 4; ++n) p[n + 4*0] = ykey1[n] ^ ykey0[n];
    // uint32_t key[4] = {0x98fad9be, 0xc54c195, 0xcd01853f, 0xcd451afd};
    // uint32_t en1[4] = {0x8bca808e, 0x8a7ec693, 0xcf3c4b77, 0xe74d1cfc};
    // uint32_t en2[4] = {0xb604c808, 0x8979c790, 0xfd384b35, 0x3c090fc5};
    // uint32_t en3[4] = {0xe0f60709, 0xd06a7549, 0x1984fd44, 0xe021dd74};
    // uint32_t en4[4] = {0x1f9318a0, 0x2a38b0c3, 0x3ba155f3, 0x3ff82bc0};
    // uint32_t en5[4] = {0xdb57d45b, 0xe6d9040a, 0xbb7e7007, 0xeeded126};
    // uint32_t en6[4] = {0x463122a0, 0xdea1f038, 0xeadbba71, 0xbeff8736};
    // for (int n = 0; n < 4; ++n) p[n + 4*5] = en6[n] ^ en5[n];
    // for (int n = 0; n < 4; ++n) p[n + 4*4] = en5[n] ^ en4[n];
    // for (int n = 0; n < 4; ++n) p[n + 4*3] = en4[n] ^ en3[n];
    // for (int n = 0; n < 4; ++n) p[n + 4*2] = en3[n] ^ en2[n];
    // for (int n = 0; n < 4; ++n) p[n + 4*1] = en2[n] ^ en1[n];
    // for (int n = 0; n < 4; ++n) p[n + 4*0] = en1[n] ^ key[n];
    const unsigned char* p_ptr = (const unsigned char*)p;
    EVP_PKEY *ec_key = d2i_PUBKEY(nullptr, &p_ptr, _YK_KEY_LEN);
    if (!ec_key) {
        ERR_print_errors_fp(stderr);
        return false;
    }
    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(ec_key, NULL);
    if (ctx && EVP_PKEY_verify_init(ctx) == 0) {
        return false;
    }
    if (EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()) == 0){
        return false;
    }
    
    int result = EVP_PKEY_verify(ctx, res, res_len, req, AUTH_REQUEST_LEN);
    EVP_PKEY_CTX_free(ctx);
    EVP_PKEY_free(ec_key);

    return (result == 1);
}
bool Auth::checkFile(const char *binary_path, const uint32_t *fkey1, const uint32_t *fkey2, const uint32_t *fkey3, const uint32_t *fkey4, const uint32_t *fkey5) {
    if(binary_path == nullptr) {
        return false;
    }
    // 1. ハッシュ値の計算
    struct stat st;
    unsigned char libHash[SHA256_DIGEST_LENGTH];
    if (stat(binary_path, &st) == 0) {
        int fd = open(binary_path, O_RDONLY);
        if (fd != -1) {
            unsigned char hash[SHA256_DIGEST_LENGTH];
            SHA256_CTX sha256;
            SHA256_Init(&sha256);

            unsigned char* buffer = (unsigned char*)malloc(st.st_size);
            if (buffer && read(fd, buffer, st.st_size) == st.st_size) {
                SHA256_Update(&sha256, buffer, st.st_size);
                SHA256_Final(hash, &sha256);    
            }
            memcpy(libHash, hash, SHA256_DIGEST_LENGTH);
            free(buffer);
            close(fd);
        }else
            return false;
    }else
        return false;

    // 2. 署名ファイルの読み込み
    unsigned char sig[256];
    std::streamsize sig_size;
    // "/home/ubuntu/ros2_ws/src/AiS-GNG/ais_gng/libgng/lib/libgng_x86_64.sig"
    std::filesystem::path sig_path(binary_path);
    sig_path.replace_extension(".sig");
    std::ifstream sig_file(sig_path, std::ios::binary);
    if (sig_file) {
        sig_file.seekg(0, std::ios::end);
        sig_size = sig_file.tellg();
        sig_file.seekg(0, std::ios::beg);
        if (sig_size < sizeof(sig)){
            sig_file.read(reinterpret_cast<char*>(sig), sig_size);
        }
    } else {
        return false;
    }
    sig_file.close();

    // 3. ハッシュ値の検証
    uint32_t p[24] = {0};
    for (int n = 0; n < 4; ++n) p[n + 4*5] = fkey6[n] ^ fkey5[n];
    for (int n = 0; n < 4; ++n) p[n + 4*4] = fkey5[n] ^ fkey4[n];
    for (int n = 0; n < 4; ++n) p[n + 4*3] = fkey4[n] ^ fkey3[n];
    for (int n = 0; n < 4; ++n) p[n + 4*2] = fkey3[n] ^ fkey2[n];
    for (int n = 0; n < 4; ++n) p[n + 4*1] = fkey2[n] ^ fkey1[n];
    for (int n = 0; n < 4; ++n) p[n + 4*0] = fkey1[n] ^ fkey0[n];
    const unsigned char* p_ptr = (const unsigned char*)p;
    EVP_PKEY *ec_key = d2i_PUBKEY(nullptr, &p_ptr, _FILE_KEY_LEN);
    if (!ec_key) {
        ERR_print_errors_fp(stderr);
        return false;
    }
    EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(ec_key, NULL);
    if (ctx && EVP_PKEY_verify_init(ctx) == 0) {
        return false;
    }
    if (EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()) == 0){
        return false;
    }

    int result = EVP_PKEY_verify(ctx, sig, sig_size, libHash, SHA256_DIGEST_LENGTH);
    EVP_PKEY_CTX_free(ctx);
    EVP_PKEY_free(ec_key);

    return (result == 1);
}