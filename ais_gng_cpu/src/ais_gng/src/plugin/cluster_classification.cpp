#include <ais_gng/plugin/cluster_classification.hpp>

#include <algorithm>
#include <cctype>

#include "ament_index_cpp/get_package_share_directory.hpp"  // install下へinstallされたモデルのpathを取得するため

ClusterClassification::ClusterClassification(){};
ClusterClassification::~ClusterClassification(){};

void ClusterClassification::init(rclcpp::Node *node) {
    auto requested_device = device_name_;
    std::transform(
        requested_device.begin(), requested_device.end(), requested_device.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (requested_device == "auto") {
        if (torch::cuda::is_available()) {
            device_ = torch::Device(torch::kCUDA);
            RCLCPP_INFO(node->get_logger(), "Classify device auto selected CUDA.");
        } else {
            device_ = torch::Device(torch::kCPU);
            RCLCPP_INFO(node->get_logger(), "Classify device auto selected CPU.");
        }
    } else if (requested_device == "cuda" || requested_device == "gpu") {
        if (torch::cuda::is_available()) {
            device_ = torch::Device(torch::kCUDA);
            RCLCPP_INFO(node->get_logger(), "Classify device: CUDA.");
        } else {
            device_ = torch::Device(torch::kCPU);
            RCLCPP_WARN(node->get_logger(), "classify.device=%s requested, but CUDA is not available. Falling back to CPU.", device_name_.c_str());
        }
    } else if (requested_device == "cpu") {
        device_ = torch::Device(torch::kCPU);
        RCLCPP_INFO(node->get_logger(), "Classify device: CPU.");
    } else {
        device_ = torch::Device(torch::kCPU);
        RCLCPP_WARN(node->get_logger(), "Unknown classify.device=%s. Falling back to CPU.", device_name_.c_str());
    }

    std::string package_share_path = ament_index_cpp::get_package_share_directory("gng_classification");
    try {
        if(human_enable_){
            model_human_ = torch::jit::load(package_share_path+"/model/"+human_model_name_);
            model_human_.to(device_);
            model_human_.eval();
        }
        if (car_enable_) {
            model_car_ = torch::jit::load(package_share_path+"/model/"+car_model_name_);
            model_car_.to(device_);
            model_car_.eval();
        }
    }catch (const c10::Error& e) {
        RCLCPP_ERROR(node->get_logger(), "Model Loading Error!");
        RCLCPP_ERROR(node->get_logger(), "%s",  e.what());
        rclcpp::shutdown();
    }
}


bool ClusterClassification::setParameter(const std::string &name, int, double value) {
    if (name == "classify.human") {
        human_enable_ = static_cast<bool>(value);
    } else if (name == "classify.car") {
        car_enable_ = static_cast<bool>(value);
    } else if (name == "classify.threshold") {
        threshold_ = static_cast<float>(value);
    } else {
        return false;
    }
    return true;
}

bool ClusterClassification::setParameter(const std::string &name, int, const std::string &value) {
    if (name == "classify.human.model") {
        human_model_name_ = value;
    } else if (name == "classify.car.model") {
        car_model_name_ = value;
    } else if (name == "classify.device") {
        device_name_ = value;
    } else {
        return false;
    }
    return true;
}

void ClusterClassification::classify(std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map, std::vector<uint32_t> &cluster_ids, std::vector<uint32_t> &cluster_frames, std::vector<uint8_t> &cluster_labels) {
    // どちらも無効な場合は何もしない
    if(!human_enable_ && !car_enable_){
        return;
    }
    torch::Tensor input_tensor;
    std::vector<int> c_ids;
    int id = -1;
    for (auto &c : map->clusters) {
        id++;
        if (c.nodes.size() < 30)
            continue;
        if (
            c.label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT ||
            c.label == ais_gng_msgs::msg::TopologicalMap::HUMAN ||
            c.label == ais_gng_msgs::msg::TopologicalMap::CAR
        ) {
            /* Size Check */
            bool flag = c.pos.z+c.scale.z/2 < 2.0 && c.scale.z > 0.7;
            if (!flag) continue;
            /* Neuaral Networks */
            std::vector<std::array<float,6>> vec(c.nodes.size());
            for (size_t i=0; i<c.nodes.size(); i++) {
            uint16_t idx = c.nodes[i];
            vec[i] = {map->nodes[idx].pos.x, map->nodes[idx].pos.y, map->nodes[idx].pos.z,
                        map->nodes[idx].normal.x, map->nodes[idx].normal.y, map->nodes[idx].normal.z,};
            }
            long num_nodes = vec.size();
            long num_features = 6;
            torch::Tensor feature = torch::std(torch::from_blob(vec.data(), {num_nodes, num_features}).clone(), 0, true, true);
            if (!input_tensor.defined()) {
                input_tensor = feature;
            }
            else {
                input_tensor = torch::cat({input_tensor, feature}, 0);
            }
            c_ids.emplace_back(id);
        }
    }
    if (input_tensor.size(0) <= 0){
        return;
    }

    /* 推論 */
    torch::Tensor output_human, labels_human, labels_human_cpu, output_human_cpu;
    torch::Tensor output_car, labels_car, labels_car_cpu, output_car_cpu;

    torch::Tensor input_tensor_gpu = input_tensor.to(device_);

    if (human_enable_) {
        output_human = torch::softmax(model_human_.forward({input_tensor_gpu}).toTensor(), 1);
        labels_human = torch::argmax(output_human, 1);
        labels_human_cpu = labels_human.to(torch::kCPU);
        output_human_cpu = output_human.to(torch::kCPU);
    }

    if (car_enable_) {
        output_car = torch::softmax(model_car_.forward({input_tensor_gpu}).toTensor(), 1);
        labels_car = torch::argmax(output_car, 1);
        labels_car_cpu = labels_car.to(torch::kCPU);
        output_car_cpu = output_car.to(torch::kCPU);
    }

    uint8_t label_human = 0, label_car = 0;
    float reliability_human = 0.0f, reliability_car = 0.0f;
    for (size_t i=0; i < c_ids.size(); i++) {
        
        if (human_enable_){
            label_human = labels_human_cpu.data_ptr<int64_t>()[i];
            reliability_human = output_human_cpu[i][label_human].item<float>();
        }
        if (car_enable_) {
            label_car = labels_car_cpu.data_ptr<int64_t>()[i];
            reliability_car = output_car_cpu[i][label_car].item<float>();
        }

        if (human_enable_ && label_human == 1 && reliability_human > threshold_){
            map->clusters[c_ids[i]].label_inferred = ais_gng_msgs::msg::TopologicalMap::HUMAN;
            map->clusters[c_ids[i]].label_reliability = reliability_human;
        } else if (car_enable_ && label_car == 1 && reliability_car > threshold_) {
            map->clusters[c_ids[i]].label_inferred = ais_gng_msgs::msg::TopologicalMap::CAR;
            map->clusters[c_ids[i]].label_reliability = reliability_car;
        } else {
            map->clusters[c_ids[i]].label_inferred = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
            map->clusters[c_ids[i]].label_reliability = 0;
        }
        cluster_ids.emplace_back(map->clusters[c_ids[i]].id);
        cluster_frames.emplace_back(map->clusters[c_ids[i]].frame);
        cluster_labels.emplace_back(map->clusters[c_ids[i]].label_inferred);
    }
}