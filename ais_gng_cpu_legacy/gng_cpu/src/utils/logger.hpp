#pragma once

#include "utils.hpp"

#define LOGGER_BUFFER_SIZE 2048

class Logger {
    char buff_[LOGGER_BUFFER_SIZE];
    int buff_n_ = 0;

   public:
    Logger() {};
    ~Logger() {};
    char* get(uint32_t* len);
    void println(char* fmt, ...);
    void print(char*fmt, ...);
    void printBytes(const uint8_t *data, const uint32_t len);
};