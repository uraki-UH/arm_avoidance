#include "cpu/radix_sort.hpp"
#include <algorithm>
#include <iostream>
#include <random>

int main() {
    std::mt19937 random(20260924);
    for (uint32_t num : {0U, 1U, 257U, 8192U, 200000U}) {
        std::vector<Voxel> values(num + 1), buffer(num), expected;
        for (uint32_t idx = 0; idx < num; ++idx) {
            const uint32_t key = idx % 3 == 0 ? random() % 256 : random();
            values[idx] = Voxel(key, idx);
        }
        if (num > 1) {values[0].voxel_index = UINT32_MAX; values[1].voxel_index = 0;}
        values[num] = Voxel(123456, 789);
        expected = values;
        std::stable_sort(expected.begin(), expected.begin() + num);
        radix_sort_voxels(values.data(), buffer.data(), num);
        // 全32bit順序・同一セル内の安定性・元点の保存・末尾の保護。
        for (uint32_t idx = 0; idx <= num; ++idx) {
            if (values[idx].voxel_index != expected[idx].voxel_index ||
                values[idx].raw_index != expected[idx].raw_index) {return 1;}
        }
    }
    std::cout << "radix_sort_test=passed\n";
}
