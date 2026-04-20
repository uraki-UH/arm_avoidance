#pragma once

#include <algorithm>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

class Downsampling {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_, human_pub_;

   public:
    bool transformed = true; // アフィン変換後の点群をPublishするかどうか
    Downsampling();
    ~Downsampling();
    void init(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub,
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_pub);

    int all_num_max = 4000;
    int unknown_num_max = 2000;
    int human_num_max = 2000;

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
        if(all_num_max >= unknown_num_max){
            randomsampling(unknown_index, unknown_num_max);
            int unknown_num = unknown_index.size();
            int all_num = MIN(all_num_max, default_index.size() + unknown_num);
            unknown_index.resize(all_num);
            randomsampling(default_index, all_num - unknown_num);
            std::copy(default_index.begin(), default_index.end(), unknown_index.begin() + unknown_num);

            publish<T>(msg, unknown_pub_, unknown_index, header);
        }
        // 人のダウンサンプリング
        randomsampling(human_index, human_num_max);
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
        if (index.size() <= num)
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