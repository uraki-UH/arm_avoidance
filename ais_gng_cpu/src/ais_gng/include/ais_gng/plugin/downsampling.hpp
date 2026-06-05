#pragma once

#include <algorithm>
#include <cstddef>
#include <random>
#include <vector>

#include "ais_gng/plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Downsampling : public Plugin {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_, human_pub_;

    bool transformed_ = true; // アフィン変換後の点群をPublishするかどうか
    int all_num_max_ = 4000;
    int unknown_num_max_ = 2000;
    int human_num_max_ = 2000;

   public:
    Downsampling();
    ~Downsampling();
    void init(rclcpp::Node *node) override;
    bool setParameter(const std::string &name, int index, double value) override;

    bool isTransformed() const { return transformed_; }

    template <typename T>
    void publish(const T &msg, const uint8_t *labels, const uint32_t label_num, const std_msgs::msg::Header &header){
        std::vector<int> default_index, unknown_index, human_index;
        default_index.reserve(label_num);
        unknown_index.reserve(label_num);
        human_index.reserve(label_num);
        for (uint32_t i = 0; i < label_num; ++i) {
            switch (labels[i]) {
                case 0b001:
                    default_index.emplace_back(i);
                    break;
                case 0b111:
                    human_index.emplace_back(i);
                case 0b011:
                    unknown_index.emplace_back(i);
                    break;
            }
        }

        // ダウンサンプリングの実行
        if(all_num_max_ >= unknown_num_max_){
            randomsampling(unknown_index, unknown_num_max_);
            const int unknown_num = static_cast<int>(unknown_index.size());
            const int all_num = std::min(
                all_num_max_, static_cast<int>(default_index.size()) + unknown_num);
            unknown_index.resize(all_num);
            randomsampling(default_index, all_num - unknown_num);
            std::copy(default_index.begin(), default_index.end(), unknown_index.begin() + unknown_num);

            publish<T>(msg, unknown_pub_, unknown_index, header);
        }
        // 人のダウンサンプリング
        randomsampling(human_index, human_num_max_);
        publish<T>(msg, human_pub_, human_index, header);
    }

   private:
    template <typename T>
    void publish(const T &msg, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
        const std::vector<int> &index,
        const std_msgs::msg::Header &header){
                int pcl_num = index.size();
            auto pcl_msg = sensor_msgs::msg::PointCloud2();
            pcl_msg.header = header;
            pcl_msg.height = 1;
            pcl_msg.width = pcl_num;
            pcl_msg.fields = msg->fields;
            pcl_msg.is_bigendian = msg->is_bigendian;
            pcl_msg.point_step = msg->point_step;
            pcl_msg.is_dense = msg->is_dense;
            pcl_msg.row_step = pcl_msg.point_step * pcl_num;
            pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);

            for (int i = 0; i < pcl_num; ++i) {
                std::copy(msg->data.begin() + index[i] * pcl_msg.point_step, msg->data.begin() + (index[i] + 1) * pcl_msg.point_step, pcl_msg.data.begin() + i * pcl_msg.point_step);
            }
            pub->publish(pcl_msg);
    }
    void randomsampling(std::vector<int> &index, int num) {
        if (num <= 0) {
            index.clear();
            return;
        }
        if (index.size() <= static_cast<std::size_t>(num))
            return;
        std::random_device seed_gen;
        std::mt19937 engine{seed_gen()};

        std::vector<int> result;
        std::sample(index.begin(),
                    index.end(),
                    std::back_inserter(result),
                    num,
                    engine);
        index.resize(num);
        std::copy(result.begin(), result.end(), index.begin());
    }
};