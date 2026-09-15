#include <SpatialTree/SpatialTree.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <string>

// 同一ソースを両ライブラリ・同一精度でビルドするための比較用切替
#ifndef SPATIAL_BENCH_SCALAR
#define SPATIAL_BENCH_SCALAR double
#endif
using scalar = SPATIAL_BENCH_SCALAR;
using point = SpatialTree::Point<scalar, 3>;
struct entry {
  point position;
  void *spatial_handle = nullptr;
  int index_in_cell = -1;
  std::size_t idx = 0;
};
using tree_type = SpatialTree::AdaptiveTree<entry, scalar, 3>;
using bounds = SpatialTree::BoundingBox<scalar, 3>;
using clock_type = std::chrono::steady_clock;

// 両版共通の比較用範囲検索。公開Cellを利用し、元ライブラリの変更なし
template<typename Visitor>
void query_box(const tree_type::Cell &cell, const bounds &box, Visitor &visitor) {
  for (int axis = 0; axis < 3; ++axis)
    if (std::abs(cell.bounds.center[axis] - box.center[axis]) >
        cell.bounds.half_extents[axis] + box.half_extents[axis]) return;
  if (cell.is_subdivided) {
    for (int idx = 0; idx < 8; ++idx) query_box(cell.children_block[idx], box, visitor);
  } else {
    for (const auto *value : cell.elements) if (box.contains(value->position)) visitor(*value);
  }
}

double elapsed_ms(clock_type::time_point start) {
  return std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
}

int main(int argc, char **argv) {
  if (argc != 2) throw std::runtime_error("座標ファイルの指定が必要");
  std::ifstream input(argv[1]);
  std::vector<entry> original;
  double x, y, z;
  while (input >> x >> y >> z) original.push_back({point{x, y, z}, nullptr, -1, original.size()});
  if (original.empty()) throw std::runtime_error("空の座標ファイル");
  point min_point = original.front().position, max_point = min_point;
  for (const auto &value : original) for (int axis = 0; axis < 3; ++axis) {
    min_point[axis] = std::min(min_point[axis], value.position[axis]);
    max_point[axis] = std::max(max_point[axis], value.position[axis]);
  }
  const point center = (min_point + max_point) * scalar(0.5);
  const point span = max_point - min_point;
  const scalar half = std::max({span[0], span[1], span[2], scalar(0.01)}) * scalar(0.5) + scalar(0.01);
  const bounds world{center, point{half, half, half}};
  std::mt19937 random(81427);
  std::uniform_real_distribution<double> offset(-0.15, 0.15);
  std::vector<bounds> queries;
  for (int idx = 0; idx < 1024; ++idx) {
    point p = original[random() % original.size()].position;
    for (int axis = 0; axis < 3; ++axis) p[axis] += scalar(offset(random));
    const scalar radius = idx % 2 == 0 ? scalar(0.025) : scalar(0.075);
    queries.push_back({p, point{radius, radius, radius}});
  }
  std::vector<double> builds, boxes, neighbours;
  std::uint64_t checksum = 0;
  for (int iter = 0; iter < 12; ++iter) {
    auto values = original;
    const auto build_start = clock_type::now();
    tree_type tree(world, SpatialTree::SpatialTreeParams<scalar>{});
    for (auto &value : values) tree.add(&value);
    const tree_type::Cell *root = nullptr;
    tree.visitCells([&](const auto &cell, int depth) { if (depth == 0) root = &cell; });
    const double build_ms = elapsed_ms(build_start);
    const auto box_start = clock_type::now();
    for (const auto &box : queries) {
      auto visit = [&](const entry &value) { checksum += value.idx + 1; };
      query_box(*root, box, visit);
    }
    const double box_ms = elapsed_ms(box_start);
    const auto nearest_start = clock_type::now();
    for (const auto &box : queries) {
      const auto found = tree.findNBest(box.center, 8);
      for (const auto &value : found) checksum += value.element->idx + 1;
    }
    const double nearest_ms = elapsed_ms(nearest_start);
    if (iter >= 2) {
      builds.push_back(build_ms); boxes.push_back(box_ms); neighbours.push_back(nearest_ms);
    }
    if (iter != 0) continue;
    // 全件走査による範囲検索・8近傍距離の照合。同距離のID順は規約外
    for (const auto &box : queries) {
      std::vector<std::size_t> expected, actual;
      std::vector<scalar> distances;
      for (const auto &value : values) {
        if (box.contains(value.position)) expected.push_back(value.idx);
        distances.push_back((value.position - box.center).squaredNorm());
      }
      auto visit = [&](const entry &value) { actual.push_back(value.idx); };
      query_box(*root, box, visit);
      std::sort(actual.begin(), actual.end());
      if (actual != expected) throw std::runtime_error("範囲検索の不一致");
      const auto count = std::min<std::size_t>(8, values.size());
      std::partial_sort(distances.begin(), distances.begin() + count, distances.end());
      const auto found = tree.findNBest(box.center, 8);
      if (found.size() != count) throw std::runtime_error("近傍件数の不一致");
      for (std::size_t idx = 0; idx < count; ++idx)
        if (found[idx].distance_sq != distances[idx]) throw std::runtime_error("近傍距離の不一致");
    }
  }
  const auto median = [](auto values) { std::sort(values.begin(), values.end()); return values[values.size() / 2]; };
  std::cout << std::setprecision(8) << "nodes=" << original.size() << " scalar_bytes=" << sizeof(scalar)
            << " queries=1024 build_ms=" << median(builds) << " aabb_ms=" << median(boxes)
            << " nearest8_ms=" << median(neighbours) << " checksum=" << checksum
            << " mismatches=0\n";
}
