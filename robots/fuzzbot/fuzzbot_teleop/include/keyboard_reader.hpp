#pragma once

#include <termios.h>
#include <unistd.h>  // for read, tcgetattr, tcsetattr
#include <cstring>   // for memcpy
#include <functional>
#include <stdexcept>

static constexpr char KEYCODE_UP    = 0x41;
static constexpr char KEYCODE_DOWN  = 0x42;
static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT  = 0x44;
static constexpr char KEYCODE_SPACE = 0x20;
static constexpr char KEYCODE_0 = 0x30;
static constexpr char KEYCODE_1 = 0x31;
static constexpr char KEYCODE_2 = 0x32;
static constexpr char KEYCODE_3 = 0x33;
static constexpr char KEYCODE_4 = 0x34;
static constexpr char KEYCODE_5 = 0x35;
static constexpr char KEYCODE_A = 0x61;
static constexpr char KEYCODE_B = 0x62;
static constexpr char KEYCODE_C = 0x63;
static constexpr char KEYCODE_D = 0x64;
static constexpr char KEYCODE_E = 0x65;
static constexpr char KEYCODE_F = 0x66;
static constexpr char KEYCODE_G = 0x67;
static constexpr char KEYCODE_H = 0x68;
static constexpr char KEYCODE_I = 0x69;
static constexpr char KEYCODE_J = 0x6a;
static constexpr char KEYCODE_K = 0x6b;
static constexpr char KEYCODE_L = 0x6c;
static constexpr char KEYCODE_M = 0x6d;
static constexpr char KEYCODE_N = 0x6e;
static constexpr char KEYCODE_O = 0x6f;
static constexpr char KEYCODE_P = 0x70;
static constexpr char KEYCODE_Q = 0x71;
static constexpr char KEYCODE_R = 0x72;
static constexpr char KEYCODE_S = 0x73;
static constexpr char KEYCODE_T = 0x74;
static constexpr char KEYCODE_U = 0x75;
static constexpr char KEYCODE_V = 0x76;
static constexpr char KEYCODE_W = 0x77;
static constexpr char KEYCODE_X = 0x78;
static constexpr char KEYCODE_Y = 0x79;
static constexpr char KEYCODE_Z = 0x7a;

class KeyboardReader
{
private:
    struct termios cooked_;
public:
    KeyboardReader();
    ~KeyboardReader();
    char readOne();
};