#pragma once

#include <cstdint>
#include <cstring>

namespace gng_observation {

// 元点群・間引き番号の借用ビュー。全点走査なし、参照対象は学習で選択した点のみ。
struct pixel_view {
    enum class format : uint8_t {none, pixel, uv, organized};
    format mode = format::none;
    const uint8_t *data = nullptr;
    uint64_t data_size = 0;
    const uint32_t *selected_ids = nullptr;
    uint32_t selected_num = 0;
    uint32_t point_num = 0;
    uint32_t width = 0, height = 0, point_step = 0, row_step = 0;
    uint32_t image_width = 0, image_height = 0;
    uint32_t first_offset = 0, second_offset = 0;
    uint8_t first_size = 0, second_size = 0;
    bool is_bigendian = false;

    bool is_valid() const {
        const uint64_t raw_num = static_cast<uint64_t>(width) * height;
        const uint64_t image_num = static_cast<uint64_t>(image_width) * image_height;
        if (!point_num || !raw_num || raw_num > UINT32_MAX || !image_num || image_num > UINT32_MAX ||
            !point_step || static_cast<uint64_t>(width) * point_step > row_step ||
            static_cast<uint64_t>(height) * row_step > data_size ||
            (selected_ids ? selected_num < point_num : point_num > raw_num)) {return false;}
        if (mode == format::organized) {return width == image_width && height == image_height;}
        const auto has_field = [&](uint32_t offset, uint8_t size) {
            return (size == 2 || size == 4) && offset <= point_step && size <= point_step - offset;
        };
        if (!data || !has_field(first_offset, first_size)) {return false;}
        return mode == format::pixel || (mode == format::uv && has_field(second_offset, second_size));
    }

    uint32_t read(const uint8_t *point, uint32_t offset, uint8_t size) const {
        // 非アライン配置とホストのエンディアンに依存しない整数読み出し。
        const auto *bytes = point + offset;
        if (is_bigendian) {
            return size == 2 ? (uint32_t(bytes[0]) << 8) | bytes[1] :
                (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) | (uint32_t(bytes[2]) << 8) | bytes[3];
        }
        return size == 2 ? uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) :
            uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
    }

    // is_valid確認済みビューの参照。無効な点番号・画素番号はUINT32_MAX。
    uint32_t get(uint32_t input_idx) const {
        if (mode == format::none || input_idx >= point_num) {return UINT32_MAX;}
        const uint32_t raw_idx = selected_ids ? selected_ids[input_idx] : input_idx;
        if (raw_idx >= static_cast<uint64_t>(width) * height) {return UINT32_MAX;}
        if (mode == format::organized) {return raw_idx;}
        const uint64_t offset = row_step == static_cast<uint64_t>(width) * point_step ?
            static_cast<uint64_t>(raw_idx) * point_step :
            static_cast<uint64_t>(raw_idx / width) * row_step + static_cast<uint64_t>(raw_idx % width) * point_step;
        const auto *point = data + offset;
        const uint32_t first = read(point, first_offset, first_size);
        if (mode == format::pixel) {
            return first < static_cast<uint64_t>(image_width) * image_height ? first : UINT32_MAX;
        }
        const uint32_t second = read(point, second_offset, second_size);
        return first < image_width && second < image_height ? second * image_width + first : UINT32_MAX;
    }
};

}
