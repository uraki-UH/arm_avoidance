#pragma once
#include "voxel_grid.hpp"
#include <array>
#include <utility>

// 全32bitを使用する安定な4パスソート。最終結果は元の配列へ格納。
inline void radix_sort_voxels(Voxel *values, Voxel *buffer, uint32_t num) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        std::array<uint32_t, 256> counts{};
        for (uint32_t idx = 0; idx < num; ++idx) {
            ++counts[(values[idx].voxel_index >> shift) & 255];
        }
        uint32_t offset = 0;
        for (auto &count : counts) {
            const uint32_t current_num = count;
            count = offset;
            offset += current_num;
        }
        for (uint32_t idx = 0; idx < num; ++idx) {
            buffer[counts[(values[idx].voxel_index >> shift) & 255]++] = values[idx];
        }
        std::swap(values, buffer);
    }
}
