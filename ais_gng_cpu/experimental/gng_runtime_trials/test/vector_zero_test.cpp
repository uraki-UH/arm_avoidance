#include "../src/utils/vec3f.hpp"
#include <new>
#include <stdexcept>
int main() {
    alignas(Vec3f) unsigned char storage[sizeof(Vec3f)];
    std::memset(storage, 0x42, sizeof(storage));
    auto *point = new(storage) Vec3f;
    for (int axis = 0; axis < 3; ++axis) {
        if (point->p[axis] != 0) {throw std::runtime_error("既定ベクトルのゼロ初期化失敗");}
    }
    const auto normal = point->normalized();
    for (int axis = 0; axis < 3; ++axis) {
        if (normal.p[axis] != 0) {throw std::runtime_error("退化法線のゼロ初期化失敗");}
    }
    point->~Vec3f();
}
