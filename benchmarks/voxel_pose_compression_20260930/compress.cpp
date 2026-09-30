#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// 実サンプルの固定代表とリンク別ボクセル集合による姿勢圧縮の検証用実装。
struct cell {
  int32_t x;
  int32_t y;
  int32_t z;
  bool operator==(const cell &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator<(const cell &other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

struct bounds {
  cell min_cell{};
  cell max_cell{};
};

struct link_mask {
  std::vector<cell> cells;
  bounds box;
};

struct pose {
  int32_t original_id;
  std::vector<float> angles;
  std::vector<float> tcp;
  std::vector<link_mask> links;
};

struct dataset {
  uint32_t num_file_nodes;
  uint32_t num_links;
  uint32_t angle_dim;
  uint32_t num_coord_layers;
  float resolution;
  std::vector<pose> nodes;
};

static uint32_t read_u32(std::istream &input) {
  std::array<unsigned char, 4> bytes{};
  if (!input.read(reinterpret_cast<char *>(bytes.data()), 4)) {
    throw std::runtime_error("Unexpected end of input");
  }
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

static int32_t read_i32(std::istream &input) {
  const auto raw = read_u32(input);
  int32_t value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

static float read_float(std::istream &input) {
  const auto raw = read_u32(input);
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite input value");
  return value;
}

static dataset read_dataset(const std::string &path, size_t max_nodes) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot open input: " + path);
  std::array<char, 8> magic{};
  if (!input.read(magic.data(), magic.size()) ||
      std::string(magic.data(), magic.size()) != "VOXPOSE1") {
    throw std::runtime_error("Invalid VOXPOSE1 header");
  }
  dataset data;
  data.num_file_nodes = read_u32(input);
  data.num_links = read_u32(input);
  data.angle_dim = read_u32(input);
  data.num_coord_layers = read_u32(input);
  data.resolution = read_float(input);
  if (data.num_links == 0 || data.num_links > 4096 ||
      data.angle_dim > 4096 || data.num_coord_layers > 4096 ||
      data.resolution <= 0.0f) {
    throw std::runtime_error("Invalid dataset dimensions");
  }
  const size_t num_nodes = std::min<size_t>(data.num_file_nodes, max_nodes);
  data.nodes.reserve(num_nodes);
  std::unordered_set<int32_t> original_ids;
  for (size_t node_idx = 0; node_idx < num_nodes; ++node_idx) {
    pose current;
    current.original_id = read_i32(input);
    if (!original_ids.insert(current.original_id).second) {
      throw std::runtime_error("Duplicate original_id");
    }
    current.angles.resize(data.angle_dim);
    for (auto &value : current.angles) value = read_float(input);
    current.tcp.resize(3 * data.num_coord_layers);
    for (auto &value : current.tcp) value = read_float(input);
    current.links.resize(data.num_links);
    for (auto &link : current.links) {
      const uint32_t num_cells = read_u32(input);
      link.cells.reserve(num_cells);
      for (uint32_t cell_idx = 0; cell_idx < num_cells; ++cell_idx) {
        const cell value{read_i32(input), read_i32(input), read_i32(input)};
        if (!link.cells.empty() && !(link.cells.back() < value)) {
          throw std::runtime_error("Input cells must be sorted and unique");
        }
        if (link.cells.empty()) {
          link.box.min_cell = value;
          link.box.max_cell = value;
        } else {
          link.box.min_cell.x = std::min(link.box.min_cell.x, value.x);
          link.box.min_cell.y = std::min(link.box.min_cell.y, value.y);
          link.box.min_cell.z = std::min(link.box.min_cell.z, value.z);
          link.box.max_cell.x = std::max(link.box.max_cell.x, value.x);
          link.box.max_cell.y = std::max(link.box.max_cell.y, value.y);
          link.box.max_cell.z = std::max(link.box.max_cell.z, value.z);
        }
        link.cells.push_back(value);
      }
    }
    data.nodes.push_back(std::move(current));
  }
  return data;
}

struct bucket {
  int64_t x;
  int64_t y;
  int64_t z;
  bool operator==(const bucket &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

static uint64_t mix_hash(uint64_t value) {
  value ^= value >> 30;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27;
  value *= 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

struct bucket_hash {
  size_t operator()(const bucket &value) const {
    return mix_hash(static_cast<uint64_t>(value.x)) ^
           mix_hash(static_cast<uint64_t>(value.y) + 0x9e3779b97f4a7c15ULL) ^
           mix_hash(static_cast<uint64_t>(value.z) + 0x243f6a8885a308d3ULL);
  }
};

static int64_t floor_div(int64_t value, int64_t width) {
  const int64_t result = value / width;
  return result - (value < 0 && value % width != 0 ? 1 : 0);
}

static bucket make_bucket(const link_mask &mask, int radius_cells) {
  // 空集合専用の領域。非空集合の座標との衝突回避。
  if (mask.cells.empty()) return {std::numeric_limits<int64_t>::min(), 0, 0};
  const int64_t width = static_cast<int64_t>(radius_cells) + 1;
  return {floor_div(mask.box.min_cell.x, width),
          floor_div(mask.box.min_cell.y, width),
          floor_div(mask.box.min_cell.z, width)};
}

static size_t select_link(const dataset &data, int radius_cells) {
  size_t selected_link_idx = 0;
  size_t max_num_buckets = 0;
  // bbox.min の区画数による候補削減用リンクの選択。
  for (size_t link_idx = 0; link_idx < data.num_links; ++link_idx) {
    std::unordered_set<bucket, bucket_hash> occupied_buckets;
    occupied_buckets.reserve(data.nodes.size());
    for (const auto &node : data.nodes) {
      occupied_buckets.insert(make_bucket(node.links[link_idx], radius_cells));
    }
    if (occupied_buckets.size() > max_num_buckets) {
      max_num_buckets = occupied_buckets.size();
      selected_link_idx = link_idx;
    }
  }
  return selected_link_idx;
}

static uint64_t shape_hash(const pose &node) {
  uint64_t value = 0x6a09e667f3bcc909ULL;
  for (const auto &link : node.links) {
    value = mix_hash(value ^ link.cells.size());
    for (const auto &point : link.cells) {
      value = mix_hash(value ^ static_cast<uint32_t>(point.x));
      value = mix_hash(value ^ static_cast<uint32_t>(point.y));
      value = mix_hash(value ^ static_cast<uint32_t>(point.z));
    }
  }
  return value;
}

static bool is_axis_close(int32_t first, int32_t second, int radius_cells) {
  return std::abs(static_cast<int64_t>(first) - second) <= radius_cells;
}

static bool is_bbox_close(const link_mask &first, const link_mask &second,
                          int radius_cells) {
  if (first.cells.empty() || second.cells.empty()) {
    return first.cells.empty() == second.cells.empty();
  }
  return is_axis_close(first.box.min_cell.x, second.box.min_cell.x, radius_cells) &&
         is_axis_close(first.box.min_cell.y, second.box.min_cell.y, radius_cells) &&
         is_axis_close(first.box.min_cell.z, second.box.min_cell.z, radius_cells) &&
         is_axis_close(first.box.max_cell.x, second.box.max_cell.x, radius_cells) &&
         is_axis_close(first.box.max_cell.y, second.box.max_cell.y, radius_cells) &&
         is_axis_close(first.box.max_cell.z, second.box.max_cell.z, radius_cells);
}

struct search_column {
  int x;
  int y;
  int max_z;
};

static std::vector<search_column> make_columns(int radius_cells) {
  std::vector<search_column> columns;
  for (int x = -radius_cells; x <= radius_cells; ++x) {
    for (int y = -radius_cells; y <= radius_cells; ++y) {
      const int remaining = radius_cells * radius_cells - x * x - y * y;
      if (remaining >= 0) {
        columns.push_back({x, y, static_cast<int>(std::sqrt(remaining))});
      }
    }
  }
  std::sort(columns.begin(), columns.end(), [](const auto &first, const auto &second) {
    return first.x * first.x + first.y * first.y <
           second.x * second.x + second.y * second.y;
  });
  return columns;
}

static bool has_near_cell(const std::vector<cell> &target, const cell &point,
                          const std::vector<search_column> &columns) {
  if (std::binary_search(target.begin(), target.end(), point)) return true;
  // 各 XY 列の球内 Z 区間による整数格子上の厳密探索。
  for (const auto &column : columns) {
    const int64_t x = static_cast<int64_t>(point.x) + column.x;
    const int64_t y = static_cast<int64_t>(point.y) + column.y;
    if (x < std::numeric_limits<int32_t>::min() ||
        x > std::numeric_limits<int32_t>::max() ||
        y < std::numeric_limits<int32_t>::min() ||
        y > std::numeric_limits<int32_t>::max()) continue;
    const int64_t min_z = std::max<int64_t>(
        std::numeric_limits<int32_t>::min(), static_cast<int64_t>(point.z) - column.max_z);
    const int64_t max_z = static_cast<int64_t>(point.z) + column.max_z;
    const cell lower{static_cast<int32_t>(x), static_cast<int32_t>(y),
                     static_cast<int32_t>(min_z)};
    const auto found = std::lower_bound(target.begin(), target.end(), lower);
    if (found != target.end() && found->x == x && found->y == y && found->z <= max_z) {
      return true;
    }
  }
  return false;
}

static bool is_directed_close(const std::vector<cell> &source,
                              const std::vector<cell> &target,
                              const std::vector<search_column> &columns) {
  for (const auto &point : source) {
    if (!has_near_cell(target, point, columns)) return false;
  }
  return true;
}

struct counters {
  uint64_t num_candidates = 0;
  uint64_t num_bbox_checks = 0;
  uint64_t num_mask_checks = 0;
};

static bool is_pose_close(const pose &first, const pose &second, int radius_cells,
                          size_t selected_link_idx,
                          const std::vector<search_column> &columns,
                          counters &counts) {
  // 全リンクの bbox 必要条件による集合照合前の早期棄却。
  for (size_t pass_idx = 0; pass_idx < first.links.size(); ++pass_idx) {
    const size_t link_idx = (selected_link_idx + pass_idx) % first.links.size();
    ++counts.num_bbox_checks;
    if (!is_bbox_close(first.links[link_idx], second.links[link_idx], radius_cells)) {
      return false;
    }
  }
  for (size_t pass_idx = 0; pass_idx < first.links.size(); ++pass_idx) {
    const size_t link_idx = (selected_link_idx + pass_idx) % first.links.size();
    const auto &first_cells = first.links[link_idx].cells;
    const auto &second_cells = second.links[link_idx].cells;
    ++counts.num_mask_checks;
    if (radius_cells == 0) {
      if (first_cells != second_cells) return false;
    } else if (!is_directed_close(first_cells, second_cells, columns) ||
               !is_directed_close(second_cells, first_cells, columns)) {
      return false;
    }
  }
  return true;
}

using steady_clock = std::chrono::steady_clock;

static double elapsed_ms(steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(steady_clock::now() - start).count();
}

int main(int argc, char **argv) {
  try {
    if (argc < 5) {
      throw std::runtime_error("Usage: compress input radius_cells seed output_prefix [--exhaustive] [--max-nodes N]");
    }
    const std::string input_path = argv[1];
    const int radius_cells = std::stoi(argv[2]);
    const uint32_t seed = static_cast<uint32_t>(std::stoul(argv[3]));
    const std::string output_prefix = argv[4];
    if (radius_cells < 0 || radius_cells > 128) {
      throw std::runtime_error("radius_cells must be in [0, 128]");
    }
    bool is_exhaustive = false;
    size_t max_nodes = std::numeric_limits<size_t>::max();
    for (int arg_idx = 5; arg_idx < argc; ++arg_idx) {
      const std::string arg = argv[arg_idx];
      if (arg == "--exhaustive") {
        is_exhaustive = true;
      } else if (arg == "--max-nodes" && arg_idx + 1 < argc) {
        max_nodes = std::stoull(argv[++arg_idx]);
      } else {
        throw std::runtime_error("Unknown or incomplete option: " + arg);
      }
    }
    const auto load_start = steady_clock::now();
    const dataset data = read_dataset(input_path, max_nodes);
    const double load_ms = elapsed_ms(load_start);
    const auto cluster_start = steady_clock::now();
    const size_t selected_link_idx = select_link(data, radius_cells);
    const auto columns = make_columns(radius_cells);
    std::vector<size_t> order(data.nodes.size());
    std::iota(order.begin(), order.end(), 0);
    std::mt19937 generator(seed);
    std::shuffle(order.begin(), order.end(), generator);
    std::vector<size_t> representatives;
    std::vector<size_t> assignments(data.nodes.size());
    std::vector<std::vector<size_t>> groups;
    std::unordered_map<bucket, std::vector<size_t>, bucket_hash> bucket_map;
    std::unordered_map<uint64_t, std::vector<size_t>> shape_map;
    counters counts;
    std::vector<size_t> candidates;
    for (const size_t node_idx : order) {
      const auto &node = data.nodes[node_idx];
      candidates.clear();
      uint64_t current_hash = 0;
      bucket current_bucket{};
      if (is_exhaustive) {
        candidates.resize(representatives.size());
        std::iota(candidates.begin(), candidates.end(), 0);
      } else if (radius_cells == 0) {
        current_hash = shape_hash(node);
        const auto found = shape_map.find(current_hash);
        if (found != shape_map.end()) candidates = found->second;
      } else {
        const auto &index_mask = node.links[selected_link_idx];
        current_bucket = make_bucket(index_mask, radius_cells);
        if (index_mask.cells.empty()) {
          const auto found = bucket_map.find(current_bucket);
          if (found != bucket_map.end()) candidates = found->second;
        } else {
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                const bucket adjacent{current_bucket.x + dx, current_bucket.y + dy,
                                      current_bucket.z + dz};
                const auto found = bucket_map.find(adjacent);
                if (found != bucket_map.end()) {
                  candidates.insert(candidates.end(), found->second.begin(), found->second.end());
                }
              }
            }
          }
          // 全探索と同一の「最初の適合代表」による決定性の維持。
          std::sort(candidates.begin(), candidates.end());
        }
      }
      size_t group_idx = representatives.size();
      for (const size_t candidate_idx : candidates) {
        ++counts.num_candidates;
        if (is_pose_close(node, data.nodes[representatives[candidate_idx]], radius_cells,
                          selected_link_idx, columns, counts)) {
          group_idx = candidate_idx;
          break;
        }
      }
      if (group_idx == representatives.size()) {
        representatives.push_back(node_idx);
        groups.emplace_back();
        if (!is_exhaustive) {
          if (radius_cells == 0) shape_map[current_hash].push_back(group_idx);
          else bucket_map[current_bucket].push_back(group_idx);
        }
      }
      assignments[node_idx] = group_idx;
      groups[group_idx].push_back(node_idx);
    }
    const double cluster_ms = elapsed_ms(cluster_start);
    uint64_t num_original_link_voxel_refs = 0;
    uint64_t num_representative_link_voxel_refs = 0;
    uint64_t num_group_union_link_voxel_refs = 0;
    uint64_t num_additional_union_link_voxel_refs = 0;
    uint64_t num_union_false_negative_refs = 0;
    uint64_t num_members_with_representative_missing_voxels = 0;
    double max_joint_abs_diff_rad = 0.0;
    double max_tcp_dist_m = 0.0;
    size_t max_num_group_members = 0;
    const auto union_start = steady_clock::now();
    for (size_t group_idx = 0; group_idx < groups.size(); ++group_idx) {
      const auto &representative = data.nodes[representatives[group_idx]];
      const auto &members = groups[group_idx];
      max_num_group_members = std::max(max_num_group_members, members.size());
      for (const size_t member_idx : members) {
        const auto &member = data.nodes[member_idx];
        bool has_missing_voxels = false;
        for (size_t angle_idx = 0; angle_idx < data.angle_dim; ++angle_idx) {
          max_joint_abs_diff_rad = std::max(max_joint_abs_diff_rad,
              std::abs(static_cast<double>(member.angles[angle_idx]) - representative.angles[angle_idx]));
        }
        for (size_t layer_idx = 0; layer_idx < data.num_coord_layers; ++layer_idx) {
          double squared_dist = 0.0;
          for (size_t axis_idx = 0; axis_idx < 3; ++axis_idx) {
            const size_t coord_idx = layer_idx * 3 + axis_idx;
            const double diff = static_cast<double>(member.tcp[coord_idx]) - representative.tcp[coord_idx];
            squared_dist += diff * diff;
          }
          max_tcp_dist_m = std::max(max_tcp_dist_m, std::sqrt(squared_dist));
        }
        for (size_t link_idx = 0; link_idx < data.num_links; ++link_idx) {
          const auto &target = representative.links[link_idx].cells;
          for (const auto &point : member.links[link_idx].cells) {
            if (!std::binary_search(target.begin(), target.end(), point)) {
              has_missing_voxels = true;
              break;
            }
          }
          if (has_missing_voxels) break;
        }
        if (has_missing_voxels) ++num_members_with_representative_missing_voxels;
      }
      for (size_t link_idx = 0; link_idx < data.num_links; ++link_idx) {
        const auto &representative_cells = representative.links[link_idx].cells;
        num_representative_link_voxel_refs += representative_cells.size();
        std::vector<cell> union_cells;
        size_t num_cells = 0;
        for (const size_t member_idx : members) {
          num_cells += data.nodes[member_idx].links[link_idx].cells.size();
        }
        num_original_link_voxel_refs += num_cells;
        union_cells.reserve(num_cells);
        for (const size_t member_idx : members) {
          const auto &cells = data.nodes[member_idx].links[link_idx].cells;
          union_cells.insert(union_cells.end(), cells.begin(), cells.end());
        }
        std::sort(union_cells.begin(), union_cells.end());
        union_cells.erase(std::unique(union_cells.begin(), union_cells.end()), union_cells.end());
        num_group_union_link_voxel_refs += union_cells.size();
        for (const auto &point : union_cells) {
          if (!std::binary_search(representative_cells.begin(), representative_cells.end(), point)) {
            ++num_additional_union_link_voxel_refs;
          }
        }
        // 各元サンプルの占有が集合和に残ることの独立走査。
        for (const size_t member_idx : members) {
          for (const auto &point : data.nodes[member_idx].links[link_idx].cells) {
            if (!std::binary_search(union_cells.begin(), union_cells.end(), point)) {
              ++num_union_false_negative_refs;
            }
          }
        }
      }
    }
    const double union_ms = elapsed_ms(union_start);
    std::ofstream output(output_prefix + ".assignments.csv");
    if (!output) throw std::runtime_error("Cannot create assignments output");
    output << "original_id,representative_id,group_idx,is_representative\n";
    for (size_t node_idx = 0; node_idx < data.nodes.size(); ++node_idx) {
      const size_t group_idx = assignments[node_idx];
      const size_t representative_idx = representatives[group_idx];
      output << data.nodes[node_idx].original_id << ','
             << data.nodes[representative_idx].original_id << ',' << group_idx << ','
             << (node_idx == representative_idx ? 1 : 0) << '\n';
    }
    output.close();
    if (!output) throw std::runtime_error("Failed to write assignments output");
    std::ofstream metrics(output_prefix + ".metrics.json");
    if (!metrics) throw std::runtime_error("Cannot create metrics output");
    metrics << std::setprecision(17)
      << "{\n"
      << "  \"num_file_nodes\": " << data.num_file_nodes << ",\n"
      << "  \"num_nodes\": " << data.nodes.size() << ",\n"
      << "  \"num_links\": " << data.num_links << ",\n"
      << "  \"angle_dim\": " << data.angle_dim << ",\n"
      << "  \"num_coord_layers\": " << data.num_coord_layers << ",\n"
      << "  \"resolution_m\": " << data.resolution << ",\n"
      << "  \"radius_cells\": " << radius_cells << ",\n"
      << "  \"radius_m\": " << static_cast<double>(data.resolution) * radius_cells << ",\n"
      << "  \"seed\": " << seed << ",\n"
      << "  \"is_exhaustive\": " << (is_exhaustive ? "true" : "false") << ",\n"
      << "  \"selected_link_idx\": " << selected_link_idx << ",\n"
      << "  \"num_representatives\": " << representatives.size() << ",\n"
      << "  \"max_num_group_members\": " << max_num_group_members << ",\n"
      << "  \"num_candidates\": " << counts.num_candidates << ",\n"
      << "  \"num_bbox_checks\": " << counts.num_bbox_checks << ",\n"
      << "  \"num_mask_checks\": " << counts.num_mask_checks << ",\n"
      << "  \"load_ms\": " << load_ms << ",\n"
      << "  \"cluster_ms\": " << cluster_ms << ",\n"
      << "  \"union_ms\": " << union_ms << ",\n"
      << "  \"num_original_link_voxel_refs\": " << num_original_link_voxel_refs << ",\n"
      << "  \"num_representative_link_voxel_refs\": " << num_representative_link_voxel_refs << ",\n"
      << "  \"num_group_union_link_voxel_refs\": " << num_group_union_link_voxel_refs << ",\n"
      << "  \"num_additional_union_link_voxel_refs\": " << num_additional_union_link_voxel_refs << ",\n"
      << "  \"num_members_with_representative_missing_voxels\": " << num_members_with_representative_missing_voxels << ",\n"
      << "  \"num_union_false_negative_refs\": " << num_union_false_negative_refs << ",\n"
      << "  \"max_joint_abs_diff_rad\": " << max_joint_abs_diff_rad << ",\n"
      << "  \"max_tcp_dist_m\": " << max_tcp_dist_m << "\n"
      << "}\n";
    metrics.close();
    if (!metrics) throw std::runtime_error("Failed to write metrics output");
    std::cout << "nodes=" << data.nodes.size() << " representatives=" << representatives.size()
              << " cluster_ms=" << cluster_ms << " union_ms=" << union_ms
              << " union_false_negative_refs=" << num_union_false_negative_refs << '\n';
    return num_union_false_negative_refs == 0 ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
