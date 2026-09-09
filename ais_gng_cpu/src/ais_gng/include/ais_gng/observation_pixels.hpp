#pragma once

#include <fuzzrobo/libgng/observation_angle_range.hpp>
#include <fuzzrobo/libgng/observation_pixel_view.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <vector>

namespace fuzzrobo::observation_pixels {

// 歪みなし・ROIなしの画素とGNG座標系の整数角度との対応表。
struct angle_table {
    std::vector<gng_observation::ray_angles> values;
    std::array<double, 13> key{};
    uint32_t width = 0, height = 0;
    uint32_t build_num = 0;

    bool prepare(const sensor_msgs::msg::CameraInfo &info, const std::array<double, 9> &rotation) {
        const uint64_t num = static_cast<uint64_t>(info.width) * info.height;
        constexpr uint32_t max_pixel_num = 4194304;
        if (!num || num > max_pixel_num || !std::isfinite(info.k[0]) || !std::isfinite(info.k[4]) ||
            info.k[0] <= 0 || info.k[4] <= 0 || !std::isfinite(info.k[2]) || !std::isfinite(info.k[5]) ||
            info.k[1] != 0 || info.k[3] != 0 || info.k[6] != 0 || info.k[7] != 0 || info.k[8] != 1 ||
            info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
            info.roi.width || info.roi.height || info.roi.do_rectify) {return false;}
        for (const auto value : info.d) {if (value != 0) {return false;}}
        for (uint32_t idx = 0; idx < 9; ++idx) {
            if (info.r[idx] != (idx % 4 == 0 ? 1.0 : 0.0) || !std::isfinite(rotation[idx])) {return false;}
        }
        // 校正済み画像のPとKの不一致は対象外。P未設定の単眼情報は許容。
        if (info.p[0] != 0) {
            for (uint32_t row = 0; row < 3; ++row) {
                for (uint32_t column = 0; column < 3; ++column) {
                    if (info.p[row * 4 + column] != info.k[row * 3 + column]) {return false;}
                }
                if (info.p[row * 4 + 3] != 0) {return false;}
            }
        }
        std::array<double, 13> next_key{info.k[0], info.k[4], info.k[2], info.k[5]};
        std::copy(rotation.begin(), rotation.end(), next_key.begin() + 4);
        if (width == info.width && height == info.height && next_key == key && values.size() == num) {return true;}
        std::vector<gng_observation::ray_angles> next_values(num);
        for (uint32_t row = 0; row < info.height; ++row) {
            for (uint32_t column = 0; column < info.width; ++column) {
                const double x = (column - info.k[2]) / info.k[0];
                const double y = (row - info.k[5]) / info.k[4];
                if (!gng_observation::can_quantize_ray(rotation[0] * x + rotation[1] * y + rotation[2],
                    rotation[3] * x + rotation[4] * y + rotation[5],
                    rotation[6] * x + rotation[7] * y + rotation[8], next_values[row * info.width + column])) {return false;}
            }
        }
        values = std::move(next_values);
        width = info.width; height = info.height; key = next_key; ++build_num;
        return true;
    }
};

inline const sensor_msgs::msg::PointField *find_field(const sensor_msgs::msg::PointCloud2 &cloud, const char *name) {
    for (const auto &field : cloud.fields) {if (field.name == name) {return &field;}}
    return nullptr;
}

// フィールド配置の事前検査のみ。画素番号の読み出しと範囲確認は学習点の選択後。
inline bool make_view(const sensor_msgs::msg::PointCloud2 &cloud, uint32_t width, uint32_t height,
    bool enable_organized, const std::vector<uint32_t> *selected_ids, uint32_t point_num,
    gng_observation::pixel_view &result) {
    result = {};
    gng_observation::pixel_view next;
    next.data = cloud.data.data(); next.data_size = cloud.data.size();
    next.width = cloud.width; next.height = cloud.height;
    next.point_step = cloud.point_step; next.row_step = cloud.row_step;
    next.image_width = width; next.image_height = height;
    next.is_bigendian = cloud.is_bigendian; next.point_num = point_num;
    if (selected_ids) {next.selected_ids = selected_ids->data(); next.selected_num = selected_ids->size();}
    if (selected_ids && selected_ids->size() < point_num) {return false;}
    const auto *pixel = find_field(cloud, "pixel_idx");
    const auto *u = find_field(cloud, "u");
    const auto *v = find_field(cloud, "v");
    const auto field_size = [](const sensor_msgs::msg::PointField *field) -> uint8_t {
        if (!field || field->count != 1) {return 0;}
        return field->datatype == field->UINT16 ? 2 : field->datatype == field->UINT32 ? 4 : 0;
    };
    if (pixel) {
        next.mode = gng_observation::pixel_view::format::pixel;
        next.first_offset = pixel->offset; next.first_size = field_size(pixel);
    } else if (u && v) {
        next.mode = gng_observation::pixel_view::format::uv;
        next.first_offset = u->offset; next.first_size = field_size(u);
        next.second_offset = v->offset; next.second_size = field_size(v);
    } else if (enable_organized) {next.mode = gng_observation::pixel_view::format::organized;}
    if (!next.is_valid()) {return false;}
    result = next;
    return true;
}

}
