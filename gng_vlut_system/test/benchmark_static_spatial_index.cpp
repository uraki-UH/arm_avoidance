#ifdef BSP3D_BENCH_REFERENCE
#include <SpatialTree/SpatialTree.hpp>
#else
#include <bsp3d/bsp3d.hpp>
#endif
#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <string>
#ifdef SPATIAL_BENCH_SELECTION
#include "nodes/planning/goal_node_selection.hpp"
#endif

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
#ifdef BSP3D_BENCH_REFERENCE
using tree_type = SpatialTree::AdaptiveTree<entry, scalar, 3>;
#else
using tree_type = bsp3d::Index<entry, scalar>;
#ifndef BSP3D_LEAF_NUM
#define BSP3D_LEAF_NUM 32
#endif
#ifndef BSP3D_SPLIT_RULE
#define BSP3D_SPLIT_RULE 0
#endif
#ifndef BSP3D_ENABLE_BBOX
#define BSP3D_ENABLE_BBOX 0
#endif
#endif
using bounds = SpatialTree::BoundingBox<scalar, 3>;
using clock_type = std::chrono::steady_clock;

// 両方式の公開APIによる閉区間範囲検索。
template<typename visitor_type>
void query_box(const tree_type &tree, const bounds &box, visitor_type &visitor) {
  tree.query_aabb(box.center - box.half_extents, box.center + box.half_extents,
      [&](const entry *value) { visitor(*value); });
}

double elapsed_ms(clock_type::time_point start) {
  return std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
}

#ifdef SPATIAL_BENCH_SELECTION
int benchmark_selection(const char *path) {
  using namespace robot_sim::planning;
  auto map = std::make_shared<ais_gng_msgs::msg::TopologicalMap>();
  map->header.frame_id = "base";
  std::ifstream input(path);
  double x, y, z;
  ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray features;
  while (input >> x >> y >> z) {
    auto &node = map->nodes.emplace_back();
    node.id = map->nodes.size() - 1;
    node.pos.x = x; node.pos.y = y; node.pos.z = z;
    node.normal.z = 1;
    node.label = node.id % 13 == 0 ? ais_gng_msgs::msg::TopologicalMap::WALL : 1;
    auto &feature = features.features.emplace_back();
    feature.node_id = node.id; feature.manip_valid = true; feature.manip_condition_number = 1 + node.id % 100;
  }
  if (map->nodes.empty()) throw std::runtime_error("空の座標ファイル");
  goal_spatial_index cache;
  const auto start = clock_type::now();
  cache.update(map);
  std::cout << "cached_build_ms=" << elapsed_ms(start) << '\n';
  const goal_selection_options options;
  tf2::Quaternion rotation;
  rotation.setRPY(0.3, -0.2, 0.7);
  const tf2::Transform transform(rotation, tf2::Vector3(0.31, -0.17, 0.23));
  const goal_transform_lookup lookup = [&](const auto &, const auto &) { return transform; };
  for (const int num_candidates : {1, 20, 100}) {
    gng_control_msgs::msg::GraspCandidateArray source;
    source.header.frame_id = "base"; source.evaluation_header.frame_id = "reach";
    source.voxel_size = 0.05;
    for (int idx = 0; idx < num_candidates; ++idx) {
      const auto &p = map->nodes[(idx * 103) % map->nodes.size()].pos;
      auto &candidate = source.candidates.emplace_back();
      candidate.state = gng_control_msgs::msg::GraspCandidate::INSIDE;
      candidate.pose.position.x = p.x; candidate.pose.position.y = p.y; candidate.pose.position.z = p.z;
      candidate.pose.orientation.w = 1;
    }
    std::vector<double> old_ms, new_ms, receive_ms;
    for (int iter = 0; iter < 220; ++iter) {
      auto copy = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*map);
      copy->frame_number = iter;
      const auto receive_start = clock_type::now();
      if (cache.update(copy)) throw std::runtime_error("同一座標での不要な再構築");
      const auto receive = elapsed_ms(receive_start);
      std::array<goal_selection_result, 2> results;
      std::array<double, 2> times;
      for (int order = 0; order < 2; ++order) {
        const int mode = (iter + order) % 2;
        const auto query_start = clock_type::now();
        results[mode] = select_goal_nodes(copy.get(), source, &features, options, lookup, mode ? &cache : nullptr);
        times[mode] = elapsed_ms(query_start);
      }
      if (results[0].ids != results[1].ids || results[0].map != results[1].map)
        throw std::runtime_error("目標選択結果の不一致");
      if (iter >= 20) { old_ms.push_back(times[0]); new_ms.push_back(times[1]); receive_ms.push_back(receive); }
    }
    const auto median = [](auto values) { std::sort(values.begin(), values.end()); return values[values.size() / 2]; };
    std::cout << "candidates=" << num_candidates << " nodes=" << map->nodes.size()
              << " samples=200 old_ms=" << median(old_ms) << " new_ms=" << median(new_ms)
              << " same_positions_ms=" << median(receive_ms) << " mismatches=0\n";
  }
  return 0;
}
#endif

int main(int argc, char **argv) {
  if (argc != 2) throw std::runtime_error("座標ファイルの指定が必要");
#ifdef SPATIAL_BENCH_SELECTION
  return benchmark_selection(argv[1]);
#endif
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
#ifdef BSP3D_BENCH_REFERENCE
    tree_type tree(world, SpatialTree::SpatialTreeParams<scalar>{});
#else
    bsp3d::index_params<scalar> params;
    params.max_leaf_size = BSP3D_LEAF_NUM;
    params.use_bbox = BSP3D_ENABLE_BBOX;
    params.split_rule = BSP3D_SPLIT_RULE;
    tree_type tree(params);
#endif
#ifdef BSP3D_BULK_BUILD
    tree.build(values.begin(), values.end());
#else
    for (auto &value : values) tree.add(&value);
#endif
    const double build_ms = elapsed_ms(build_start);
    const auto box_start = clock_type::now();
    for (const auto &box : queries) {
      auto visit = [&](const entry &value) { checksum += value.idx + 1; };
      query_box(tree, box, visit);
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
      query_box(tree, box, visit);
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
