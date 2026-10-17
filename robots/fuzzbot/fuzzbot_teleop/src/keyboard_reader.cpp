#include "keyboard_reader.hpp"

KeyboardReader::KeyboardReader()
{
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
        throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
        throw std::runtime_error("Failed to set new console mode");
    }
} 

KeyboardReader::~KeyboardReader()
{
    tcsetattr(0, TCSANOW, &cooked_);
}

char KeyboardReader::readOne()
{
    char c = 0;
    int rc = read(0, &c, 1);
    if (rc < 0)
    {
        throw std::runtime_error("read failed");
    }
    return c;
}