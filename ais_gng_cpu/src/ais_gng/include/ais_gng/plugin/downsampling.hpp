#pragma once

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

#include "ais_gng/plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Downsampling : public Plugin {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_, human_pub_;

    bool is_transformed_ = true; // アフィン変換後の点群の配信設定
    int max_all_num_ = 4000;
    int max_unknown_num_ = 2000;
    int max_human_num_ = 2000;

   public:
    Downsampling();
    ~Downsampling();
    void init(rclcpp::Node *node) override;
    bool setParameter(const std::string &name, int index, double value) override;

    bool isTransformed() const { return is_transformed_; }

    template <typename T>
    void publish(const T &msg, const uint8_t *labels, const uint32_t label_num, const std_msgs::msg::Header &header){
        const bool has_unknown_subscribers = unknown_pub_->get_subscription_count() > 0 ||
            unknown_pub_->get_intra_process_subscription_count() > 0;
        const bool has_human_subscribers = human_pub_->get_subscription_count() > 0 ||
            human_pub_->get_intra_process_subscription_count() > 0;
        if (!has_unknown_subscribers && !has_human_subscribers) {return;}
        std::vector<int> default_indices, unknown_indices, human_indices;
        if (has_unknown_subscribers) {
            default_indices.reserve(label_num);
            unknown_indices.reserve(label_num);
        }
        if (has_human_subscribers) {human_indices.reserve(label_num);}
        for (uint32_t i = 0; i < label_num; ++i) {
            switch (labels[i]) {
                case 0b001:
                    if (has_unknown_subscribers) {default_indices.emplace_back(i);}
                    break;
                case 0b111:
                    if (has_human_subscribers) {human_indices.emplace_back(i);}
                    break;
                case 0b011:
                    if (has_unknown_subscribers) {unknown_indices.emplace_back(i);}
                    break;
            }
        }

        // 未知点群優先、残枠へのDEFAULT点群の一様抽出。
        if(has_unknown_subscribers && max_all_num_ >= max_unknown_num_){
            random_sampling(unknown_indices, max_unknown_num_);
            const int remaining_budget = std::max(
                0, max_all_num_ - static_cast<int>(unknown_indices.size()));
            random_sampling(default_indices, remaining_budget);
            unknown_indices.reserve(unknown_indices.size() + default_indices.size());
            unknown_indices.insert(unknown_indices.end(), default_indices.begin(), default_indices.end());
            publish<T>(msg, unknown_pub_, unknown_indices, header);
        }
        // 人のダウンサンプリング
        if (has_human_subscribers) {
            random_sampling(human_indices, max_human_num_);
            publish<T>(msg, human_pub_, human_indices, header);
        }
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
    void random_sampling(std::vector<int> &index, int num) {
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
