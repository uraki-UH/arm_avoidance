// 閉区間範囲検索と動的更新の全走査照合。
#include <bsp3d/bsp3d.hpp>
#include <algorithm>
#include <limits>
#include <random>
#include <stdexcept>

static std::size_t num_checks = 0;
static void check(bool is_valid) {
  ++num_checks;
  if (!is_valid) throw std::runtime_error("AABB照合の不一致");
}

template <typename scalar, typename policy>
void verify(int enable_bbox) {
  using point = bsp3d::point3<scalar>;
  struct entry {
    point position;
    void *spatial_handle = nullptr;
    int index_in_cell = -1;
    std::size_t idx = 0;
    bool is_active = false;
  };
  using tree_type = SpatialTree::MovingBSPTree<entry, scalar, 3,
      SpatialTree::SpatialTraits<entry, scalar, 3>, policy>;
  bsp3d::index_params<scalar> params;
  params.max_leaf_size = 8;
  params.use_bbox = enable_bbox;
  tree_type tree(params);
  std::vector<entry> entries(512);
  std::mt19937 random(24317);
  std::uniform_real_distribution<double> uniform(-8, 8);
  const scalar margin = policy::enabled ? scalar(0.25) : scalar(0);
  const auto random_point = [&] {
    return point{scalar(uniform(random)), scalar(uniform(random)), scalar(uniform(random))};
  };
  const auto compare = [&](const point &min_point, const point &max_point) {
    std::vector<std::size_t> expected, actual;
    for (const auto &value : entries) {
      if (!value.is_active) continue;
      bool is_inside = true;
      for (int axis = 0; axis < 3; ++axis)
        is_inside = is_inside && value.position[axis] >= min_point[axis] &&
            value.position[axis] <= max_point[axis];
      if (is_inside) expected.push_back(value.idx);
    }
    tree.query_aabb(min_point, max_point, [&](const auto *value) { actual.push_back(value->idx); }, margin);
    std::sort(actual.begin(), actual.end());
    check(actual == expected);
  };
  compare(point{-1,-1,-1},point{1,1,1});
  for (std::size_t idx = 0; idx < entries.size(); ++idx) {
    auto &value = entries[idx];
    value.idx = idx;
    value.position = idx % 9 == 0 ? point{0,0,0} : random_point();
    tree.add(&value);
    value.is_active = true;
  }
  for (int iter = 0; iter < 4000; ++iter) {
    auto &value = entries[random() % entries.size()];
    switch (random() % 3) {
      case 0:
        if (value.is_active) tree.updatePosition(&value, random_point(), margin);
        break;
      case 1:
        if (value.is_active) { tree.remove(&value); value.is_active = false; }
        break;
      default:
        if (!value.is_active) {
          value.position = random_point();
          tree.add(&value);
          value.is_active = true;
        }
    }
    const point center = random_point();
    const point half{scalar(0.125),scalar(0.75),scalar(2)};
    compare(center-half, center+half);
    if (iter % 20 == 0) {
      compare(point{-20,-20,-20},point{20,20,20});
      compare(point{0,0,0},point{0,0,0});
      if (value.is_active) compare(value.position,value.position);
      check(tree.checkInvariants().empty());
    }
  }
  // 浮動小数点境界と座標の精度維持。
  for (auto &value : entries) {
    if (value.is_active) tree.remove(&value);
    value.is_active = false;
  }
  for (int idx = 0; idx < 3; ++idx) {
    auto &value = entries[idx];
    const scalar coordinate = idx == 0 ? scalar(0.125) :
        std::nextafter(scalar(0.125),idx == 1 ? scalar(0) : scalar(1));
    value.position = point{coordinate,0,0};
    value.is_active = true;
    tree.add(&value);
  }
  compare(point{scalar(0.125),0,0},point{scalar(0.125),0,0});
  for (const scalar invalid : {std::numeric_limits<scalar>::quiet_NaN(),
                               std::numeric_limits<scalar>::infinity()}) {
    bool has_thrown = false;
    try { tree.query_aabb(point{invalid,0,0},point{1,1,1},[](auto *){}); }
    catch (const std::invalid_argument &) { has_thrown = true; }
    check(has_thrown);
  }
  bool has_thrown = false;
  try { tree.query_aabb(point{2,0,0},point{1,1,1},[](auto *){}); }
  catch (const std::invalid_argument &) { has_thrown = true; }
  check(has_thrown);
}

int main() {
  for (int enable_bbox : {0,1}) {
    verify<float,SpatialTree::NoHysteresis>(enable_bbox);
    verify<double,SpatialTree::NoHysteresis>(enable_bbox);
    verify<float,SpatialTree::AdaptiveHysteresis>(enable_bbox);
    verify<double,SpatialTree::AdaptiveHysteresis>(enable_bbox);
  }
  std::cout << "AABB checks=" << num_checks << " failures=0\n";
}
