#include "logger.hpp"

char* Logger::get(uint32_t *len) {
    *len = buff_n_;
    buff_n_ = 0;
    return buff_;
}

void Logger::println(char* fmt, ...) {
    if (buff_n_ > LOGGER_BUFFER_SIZE / 2)
        return;

    va_list arg;
    int len;

    va_start(arg, fmt);

    len = vsprintf(buff_ + buff_n_, fmt, arg);

    va_end(arg);

    if (len >= 0) {
        buff_n_ += len;
        if (buff_n_ < LOGGER_BUFFER_SIZE - 1) {
            buff_[buff_n_] = '\n';
            buff_n_ += 1;
        }
    }
}

void Logger::print(char* fmt, ...) {
    if (buff_n_ > LOGGER_BUFFER_SIZE / 2)
        return;

    va_list arg;
    int len;

    va_start(arg, fmt);

    len = vsprintf(buff_ + buff_n_, fmt, arg);

    va_end(arg);

    if (len >= 0) {
        buff_n_ += len;
    }
}
void Logger::printBytes(const uint8_t *data, const uint32_t len) {
    if (buff_n_ + len * 3 > LOGGER_BUFFER_SIZE)
        return;

    for (uint32_t i = 0; i < len; ++i) {
        buff_n_ += sprintf(buff_ + buff_n_, "%02x ", data[i]);
    }
    buff_[buff_n_++] = '\n';
}