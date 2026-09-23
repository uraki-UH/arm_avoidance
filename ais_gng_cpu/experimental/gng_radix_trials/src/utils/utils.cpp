#include "utils.hpp"

float squaredNormXY(Vec3 &p1, Vec3 &p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

vector<int> make_rand_array(const int size, int rand_min, int rand_max) {
    if (rand_min > rand_max) swap(rand_min, rand_max);
    const size_t max_min_diff = static_cast<size_t>(rand_max - rand_min + 1);
    vector<int> tmp;
    if (max_min_diff < size)
        return tmp;

    random_device seed;
    mt19937 engine(seed());
    uniform_int_distribution<int> distribution(rand_min, rand_max);

    // const size_t make_size = static_cast<size_t>(size * 1.2);
    const size_t make_size = static_cast<size_t>(size * 1.0);  // フルスケール(10mまで描画)

    tmp.reserve(size);
    while (tmp.size() < size) {
        while (tmp.size() < make_size) tmp.emplace_back(distribution(engine));
        sort(tmp.begin(), tmp.end());
        auto unique_end = unique(tmp.begin(), tmp.end());

        if (int(size) < distance(tmp.begin(), unique_end)) {
            unique_end = next(tmp.begin(), size);
        }
        tmp.erase(unique_end, tmp.end());
    }
    return tmp;
}

vector<int> make_rand_array_unique(const int size, int rand_min, int rand_max) {
    // if (rand_min > rand_max) swap(rand_min, rand_max);
    const size_t max_min_diff = static_cast<size_t>(rand_max - rand_min + 1);
    vector<int> tmp;
    if (max_min_diff < size)
        return tmp;

    random_device seed;
    mt19937 engine(seed());
    uniform_int_distribution<int> distribution(rand_min, rand_max);

    // const size_t make_size = static_cast<size_t>(size * 1.2);
    const size_t make_size = static_cast<size_t>(size * 1.0);  // フルスケール(10mまで描画)

    tmp.reserve(size);
    while (tmp.size() < size) {
        while (tmp.size() < make_size)
            tmp.emplace_back(distribution(engine));
        sort(tmp.begin(), tmp.end());
        auto unique_end = unique(tmp.begin(), tmp.end());

        if (int(size) < distance(tmp.begin(), unique_end)) {
            unique_end = next(tmp.begin(), size);
        }
        tmp.erase(unique_end, tmp.end());
    }
    shuffle(tmp.begin(), tmp.end(), engine);
    return tmp;
}