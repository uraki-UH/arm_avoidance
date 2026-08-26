#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ais_gng/plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Downsampling : public Plugin {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_, grasp_support_pub_, human_pub_;

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
                    break;
                case 0b011:
                    unknown_index.emplace_back(i);
                    break;
            }
        }

        // Grasp support is not an "unknown only" cloud.  UNKNOWN_OBJECT gets
        // first claim on the budget, then DEFAULT points fill the remainder so
        // that a temporarily uncertain object surface does not lose support.
        // Use coverage sampling rather than a pure random subset: thin objects
        // are otherwise likely to have empty support cells downstream.
        if(all_num_max_ >= unknown_num_max_){
            coverageSampling(*msg, unknown_index, unknown_num_max_);
            const int remaining_budget = std::max(
                0, all_num_max_ - static_cast<int>(unknown_index.size()));
            coverageSampling(*msg, default_index, remaining_budget);
            unknown_index.reserve(unknown_index.size() + default_index.size());
            unknown_index.insert(unknown_index.end(), default_index.begin(), default_index.end());

            // The explicit name is used by new grasp pre-filter launches.
            publish<T>(msg, grasp_support_pub_, unknown_index, header);
            // Keep the historical topic during migration.  Its payload is now
            // the same human-free grasp-support cloud, not unknown-only data.
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

    static bool coordinateOffsets(
        const sensor_msgs::msg::PointCloud2 &cloud,
        uint32_t &x_offset, uint32_t &y_offset, uint32_t &z_offset) {
        const auto find_offset = [&cloud](const char *name, uint32_t &offset) {
            for (const auto &field : cloud.fields) {
                if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
                    offset = field.offset;
                    return true;
                }
            }
            return false;
        };
        return cloud.point_step >= 3U * sizeof(float) &&
               find_offset("x", x_offset) && find_offset("y", y_offset) && find_offset("z", z_offset) &&
               x_offset + sizeof(float) <= cloud.point_step &&
               y_offset + sizeof(float) <= cloud.point_step &&
               z_offset + sizeof(float) <= cloud.point_step;
    }

    static bool pointCoordinates(
        const sensor_msgs::msg::PointCloud2 &cloud, int index,
        uint32_t x_offset, uint32_t y_offset, uint32_t z_offset,
        float &x, float &y, float &z) {
        if (index < 0) {
            return false;
        }
        const std::size_t byte_index = static_cast<std::size_t>(index) * cloud.point_step;
        if (byte_index + cloud.point_step > cloud.data.size()) {
            return false;
        }
        const auto *data = cloud.data.data() + byte_index;
        std::memcpy(&x, data + x_offset, sizeof(float));
        std::memcpy(&y, data + y_offset, sizeof(float));
        std::memcpy(&z, data + z_offset, sizeof(float));
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    // Derive the stratification resolution from the requested sample budget,
    // rather than introducing another metre-valued tuning parameter.  One
    // representative is retained from each occupied coarse bin before the
    // remaining budget is filled normally.
    void coverageSampling(
        const sensor_msgs::msg::PointCloud2 &cloud, std::vector<int> &index, int num) {
        if (num <= 0) {
            index.clear();
            return;
        }
        if (index.size() <= static_cast<std::size_t>(num)) {
            return;
        }

        uint32_t x_offset = 0;
        uint32_t y_offset = 0;
        uint32_t z_offset = 0;
        if (!coordinateOffsets(cloud, x_offset, y_offset, z_offset)) {
            randomsampling(index, num);
            return;
        }

        struct SamplePoint {
            int index = 0;
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
        };
        std::vector<SamplePoint> valid_points;
        valid_points.reserve(index.size());
        float minimum_x = std::numeric_limits<float>::infinity();
        float minimum_y = std::numeric_limits<float>::infinity();
        float minimum_z = std::numeric_limits<float>::infinity();
        float maximum_x = -std::numeric_limits<float>::infinity();
        float maximum_y = -std::numeric_limits<float>::infinity();
        float maximum_z = -std::numeric_limits<float>::infinity();
        for (const int point_index : index) {
            SamplePoint point;
            point.index = point_index;
            if (!pointCoordinates(
                    cloud, point_index, x_offset, y_offset, z_offset,
                    point.x, point.y, point.z)) {
                continue;
            }
            minimum_x = std::min(minimum_x, point.x);
            minimum_y = std::min(minimum_y, point.y);
            minimum_z = std::min(minimum_z, point.z);
            maximum_x = std::max(maximum_x, point.x);
            maximum_y = std::max(maximum_y, point.y);
            maximum_z = std::max(maximum_z, point.z);
            valid_points.push_back(point);
        }
        if (valid_points.empty()) {
            randomsampling(index, num);
            return;
        }

        const auto resolution = static_cast<uint32_t>(std::max(
            1.0, std::ceil(std::cbrt(static_cast<double>(num)))));
        const auto bucket_axis = [resolution](float value, float minimum, float maximum) {
            const float range = maximum - minimum;
            if (!std::isfinite(range) || range <= std::numeric_limits<float>::epsilon()) {
                return 0U;
            }
            const double normalized = std::clamp(
                static_cast<double>((value - minimum) / range), 0.0, 1.0);
            return std::min(
                resolution - 1U,
                static_cast<uint32_t>(normalized * static_cast<double>(resolution)));
        };

        std::unordered_map<std::uint64_t, int> representatives;
        representatives.reserve(valid_points.size());
        for (const auto &point : valid_points) {
            const uint32_t x = bucket_axis(point.x, minimum_x, maximum_x);
            const uint32_t y = bucket_axis(point.y, minimum_y, maximum_y);
            const uint32_t z = bucket_axis(point.z, minimum_z, maximum_z);
            const std::uint64_t bucket =
                (static_cast<std::uint64_t>(x) * resolution + y) * resolution + z;
            representatives.emplace(bucket, point.index);
        }

        std::vector<int> selected;
        selected.reserve(std::min<std::size_t>(static_cast<std::size_t>(num), representatives.size()));
        for (const auto &[bucket, representative] : representatives) {
            (void)bucket;
            selected.push_back(representative);
        }
        randomsampling(selected, num);
        if (selected.size() < static_cast<std::size_t>(num)) {
            std::unordered_set<int> selected_set(selected.begin(), selected.end());
            std::vector<int> remaining;
            remaining.reserve(valid_points.size() - selected.size());
            for (const auto &point : valid_points) {
                if (selected_set.find(point.index) == selected_set.end()) {
                    remaining.push_back(point.index);
                }
            }
            randomsampling(remaining, num - static_cast<int>(selected.size()));
            selected.insert(selected.end(), remaining.begin(), remaining.end());
        }
        index = std::move(selected);
    }
};
