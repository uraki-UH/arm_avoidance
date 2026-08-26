#pragma once

#include <fuzzrobo/libgng/api.h>
#include <vector>
#include <torch/torch.h>
#include <torch/script.h>
#include <string>
#include "ais_gng/plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ais_gng_msgs/msg/topological_map.hpp"

class ClusterClassification : public Plugin {
    c10::Device device_ = torch::kCPU;
    torch::jit::script::Module model_human_;
    torch::jit::script::Module model_car_;

    bool human_enable_ = true;
    std::string human_model_name_ = "NN_human.pt";
    bool car_enable_ = false;
    std::string car_model_name_ = "NN_car.pt";
    float threshold_ = 0.8f;
    std::string device_name_ = "cpu";

   public:
    ClusterClassification();
    ~ClusterClassification();

    void init(rclcpp::Node *node) override;
    bool setParameter(const std::string &name, int index, double value) override;
    bool setParameter(const std::string &name, int index, const std::string &value) override;

    void classify(std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map,
                                         std::vector<uint32_t> &cluster_ids,
                                         std::vector<uint32_t> &cluster_ages,
                                         std::vector<uint8_t> &cluster_labels);
};