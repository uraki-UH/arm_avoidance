#pragma once
#include "../utils/param.hpp"

// 有限なYAML境界と上端の点を含むセル番号。空間全体の点群配列の確保なし。
struct cell_grid {
    Vec3f min_pos, max_pos;
    float unit = 0, inverse_unit = 0;
    uint32_t num_cells[3]{}, num_xy = 0, num_total = 0;
    bool init(const Vec3f &min_value, const Vec3f &max_value, float cell_unit) {
        if (!std::isfinite(cell_unit) || cell_unit <= 0) {return false;}
        min_pos = min_value; max_pos = max_value;
        unit = cell_unit; inverse_unit = 1.0f / unit;
        uint64_t total = 1;
        for (int axis = 0; axis < 3; ++axis) {
            const double count = std::floor(static_cast<double>(
                (max_pos.p[axis] - min_pos.p[axis]) * inverse_unit)) + 1;
            if (!std::isfinite(count) || count < 1 || count >= UINT32_MAX) {return false;}
            num_cells[axis] = static_cast<uint32_t>(count);
            total *= num_cells[axis];
            if (total >= UINT32_MAX) {return false;}
        }
        num_xy = num_cells[0] * num_cells[1];
        num_total = static_cast<uint32_t>(total);
        return true;
    }
    uint32_t get_cell_idx(const Vec3f &point) const {
        uint32_t coord[3];
        for (int axis = 0; axis < 3; ++axis) {
            const auto value = point.p[axis];
            if (!std::isfinite(value) || value < min_pos.p[axis] || value > max_pos.p[axis]) {return UINT32_MAX;}
            coord[axis] = std::min(num_cells[axis] - 1,
                static_cast<uint32_t>((value - min_pos.p[axis]) * inverse_unit));
        }
        return coord[0] + coord[1] * num_cells[0] + coord[2] * num_xy;
    }
};
