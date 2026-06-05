#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

class Plugin {
public:
    virtual ~Plugin() = default;
    virtual void init(rclcpp::Node *node) = 0;
    virtual bool setParameter(const std::string &name, int index, double value) = 0;
    virtual bool setParameter(const std::string &, int, const std::string &) { return false; }
};
