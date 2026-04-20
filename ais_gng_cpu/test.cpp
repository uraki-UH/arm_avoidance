#include <openssl/hmac.h>
int main() {
    HMAC(nullptr, nullptr, 0, nullptr, 0, nullptr, nullptr);
    return 0;
}
