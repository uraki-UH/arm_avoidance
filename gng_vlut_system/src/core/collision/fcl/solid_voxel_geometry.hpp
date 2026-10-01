#pragma once

#include <Eigen/Dense>
#include <octomap/OcTree.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/voxelizer_engine.hpp"
#include "robot_model/stl_loader.hpp"

namespace collision {

// バイナリ STL の完全性と有限値を検査した読込み
inline simulation::MeshData load_solid_voxel_stl(
    const std::string& path, const Eigen::Vector3d& scale) {
  if (!scale.allFinite() || (scale.array() == 0.0).any()) {
    throw std::invalid_argument("Invalid solid voxel mesh scale");
  }
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream || stream.tellg() < 84) {
    throw std::runtime_error("Missing or truncated binary STL: " + path);
  }
  const auto num_bytes = static_cast<std::uint64_t>(stream.tellg());
  stream.seekg(0);
  std::array<unsigned char, 84> header{};
  stream.read(reinterpret_cast<char*>(header.data()), header.size());
  const auto read_uint32 = [](const unsigned char* bytes) {
    return static_cast<std::uint32_t>(bytes[0]) |
        (static_cast<std::uint32_t>(bytes[1]) << 8) |
        (static_cast<std::uint32_t>(bytes[2]) << 16) |
        (static_cast<std::uint32_t>(bytes[3]) << 24);
  };
  const std::uint32_t num_triangles = read_uint32(header.data() + 80);
  constexpr std::uint32_t max_triangles = 5000000;
  if (!stream || num_triangles == 0 || num_triangles > max_triangles ||
      num_bytes != 84ULL + 50ULL * num_triangles) {
    throw std::runtime_error("Invalid binary STL size or triangle count: " + path);
  }
  simulation::MeshData mesh;
  mesh.vertices.reserve(static_cast<std::size_t>(num_triangles) * 9);
  mesh.indices.reserve(static_cast<std::size_t>(num_triangles) * 3);
  for (std::uint32_t triangle_idx = 0; triangle_idx < num_triangles;
       ++triangle_idx) {
    std::array<unsigned char, 50> record{};
    stream.read(reinterpret_cast<char*>(record.data()), record.size());
    if (!stream) {
      throw std::runtime_error("Truncated binary STL triangle: " + path);
    }
    for (std::size_t coordinate_idx = 0; coordinate_idx < 9; ++coordinate_idx) {
      const std::uint32_t bits = read_uint32(
          record.data() + 12 + coordinate_idx * 4);
      float value;
      std::memcpy(&value, &bits, sizeof(value));
      const float scaled_value = value *
          static_cast<float>(scale[coordinate_idx % 3]);
      if (!std::isfinite(value) || !std::isfinite(scaled_value)) {
        throw std::runtime_error("Non-finite binary STL vertex: " + path);
      }
      if (coordinate_idx % 3 == 0) {
        mesh.indices.push_back(static_cast<std::uint32_t>(
            mesh.vertices.size() / 3));
      }
      mesh.vertices.push_back(scaled_value);
    }
  }
  return mesh;
}

// 表面セルと外部到達不能セルによる保守的な剛体占有形状
struct solid_voxel_geometry {
  std::shared_ptr<octomap::OcTree> tree;
  // 非退化な連結表面ごとの元メッシュ頂点
  std::vector<Eigen::Vector3d> surface_component_points;
  std::size_t num_surface_cells = 0;
  std::size_t num_interior_cells = 0;
  std::size_t num_grid_cells = 0;
};

inline solid_voxel_geometry build_solid_voxel_geometry(
    const simulation::MeshData& mesh, double voxel_size,
    std::size_t max_cells = 20000000) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0 || max_cells == 0 ||
      max_cells > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("Invalid solid voxel resolution or cell budget");
  }
  if (mesh.vertices.empty() || mesh.vertices.size() % 3 != 0 ||
      mesh.indices.empty() || mesh.indices.size() % 3 != 0) {
    throw std::invalid_argument("Invalid solid voxel mesh arrays");
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::size_t> vertex_ids;
  std::map<std::array<double, 3>, std::size_t> unique_vertices;
  Eigen::Vector3d min_point = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_point = -min_point;
  for (std::size_t idx = 0; idx < mesh.vertices.size(); idx += 3) {
    const Eigen::Vector3d point(mesh.vertices[idx], mesh.vertices[idx + 1],
                                mesh.vertices[idx + 2]);
    if (!point.allFinite()) {
      throw std::invalid_argument("Non-finite solid voxel mesh vertex");
    }
    const std::array<double, 3> key{point.x(), point.y(), point.z()};
    const auto entry = unique_vertices.emplace(key, unique_vertices.size());
    vertex_ids.push_back(entry.first->second);
    vertices.push_back(point);
    min_point = min_point.cwiseMin(point);
    max_point = max_point.cwiseMax(point);
  }

  // 面積ゼロの接続面を除いた表面成分の頂点集合
  std::vector<std::size_t> component_parent_idxs(unique_vertices.size());
  std::iota(component_parent_idxs.begin(), component_parent_idxs.end(), 0);
  std::vector<std::size_t> component_sizes(unique_vertices.size(), 1);
  std::vector<std::uint8_t> has_surface_vertex(unique_vertices.size(), 0);
  const auto find_component = [&](std::size_t idx) {
    while (component_parent_idxs[idx] != idx) {
      component_parent_idxs[idx] = component_parent_idxs[component_parent_idxs[idx]];
      idx = component_parent_idxs[idx];
    }
    return idx;
  };
  const auto merge_components = [&](std::size_t first_idx, std::size_t second_idx) {
    first_idx = find_component(first_idx);
    second_idx = find_component(second_idx);
    if (first_idx == second_idx) return;
    if (component_sizes[first_idx] < component_sizes[second_idx]) {
      std::swap(first_idx, second_idx);
    }
    component_parent_idxs[second_idx] = first_idx;
    component_sizes[first_idx] += component_sizes[second_idx];
  };

  std::vector<Eigen::Vector3d> triangles;
  triangles.reserve(mesh.indices.size());
  struct edge_face_count {
    std::size_t num_faces = 0;
    std::int64_t direction_balance = 0;
  };
  std::map<std::pair<std::size_t, std::size_t>, edge_face_count> edge_counts;
  for (std::size_t idx = 0; idx < mesh.indices.size(); idx += 3) {
    std::array<std::size_t, 3> face{};
    for (std::size_t corner_idx = 0; corner_idx < face.size(); ++corner_idx) {
      face[corner_idx] = mesh.indices[idx + corner_idx];
      if (face[corner_idx] >= vertices.size()) {
        throw std::invalid_argument("Invalid solid voxel mesh triangle index");
      }
    }
    const auto& first = vertices[face[0]];
    const auto& second = vertices[face[1]];
    const auto& third = vertices[face[2]];
    // 面積ゼロの接続面は辺の収支だけ維持。微小な有効面は表面にも収録
    if ((second - first).cross(third - first).squaredNorm() != 0.0) {
      triangles.insert(triangles.end(), {first, second, third});
      for (const auto vertex_idx : face) {
        has_surface_vertex[vertex_ids[vertex_idx]] = 1;
      }
      merge_components(vertex_ids[face[0]], vertex_ids[face[1]]);
      merge_components(vertex_ids[face[0]], vertex_ids[face[2]]);
    }
    for (std::size_t edge_idx = 0; edge_idx < face.size(); ++edge_idx) {
      auto left = vertex_ids[face[edge_idx]];
      auto right = vertex_ids[face[(edge_idx + 1) % face.size()]];
      if (left == right) {
        continue;
      }
      const bool is_reversed = left > right;
      if (is_reversed) {
        std::swap(left, right);
      }
      auto& count = edge_counts[{left, right}];
      ++count.num_faces;
      count.direction_balance += is_reversed ? -1 : 1;
    }
  }
  if (triangles.empty()) {
    throw std::invalid_argument("Empty solid voxel mesh surface");
  }
  // 向きの釣り合う閉じた面集合。複数固体の接触辺の偶数重複も許容
  for (const auto& entry : edge_counts) {
    if (entry.second.num_faces % 2 != 0 || entry.second.direction_balance != 0) {
      throw std::invalid_argument("Open solid voxel mesh boundary");
    }
  }

  const double bbox_margin = voxel_size * 0.5 +
      robot_sim::common::Constants::GEOM_EPSILON;
  Eigen::Vector3i min_idx;
  Eigen::Vector3i grid_size;
  std::size_t num_cells = 1;
  for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
    const double low = std::floor(
        (min_point[axis_idx] - bbox_margin) / voxel_size) - 1.0;
    const double high = std::floor(
        (max_point[axis_idx] + bbox_margin) / voxel_size) + 1.0;
    // OctoMap の深さ 16 の格子キー範囲
    if (!std::isfinite(low) || !std::isfinite(high) ||
        low < -32768.0 || high > 32767.0 || high < low) {
      throw std::invalid_argument("Solid voxel mesh exceeds OctoMap key range");
    }
    min_idx[axis_idx] = static_cast<int>(low);
    grid_size[axis_idx] = static_cast<int>(high - low + 1.0);
    const auto axis_size = static_cast<std::size_t>(grid_size[axis_idx]);
    if (num_cells > max_cells / axis_size) {
      throw std::length_error("Solid voxel mesh exceeds cell budget");
    }
    num_cells *= axis_size;
  }

  GNG::Analysis::IndexVoxelGrid grid(voxel_size);
  std::unordered_set<long> surface_cells;
  robot_sim::common::VoxelizerEngine::voxelizeMeshTriangles(
      triangles, grid, surface_cells);
  if (surface_cells.empty()) {
    throw std::runtime_error("Empty solid voxel mesh voxelization");
  }

  const auto num_x = static_cast<std::size_t>(grid_size.x());
  const auto num_y = static_cast<std::size_t>(grid_size.y());
  const auto num_z = static_cast<std::size_t>(grid_size.z());
  const std::size_t layer_size = num_x * num_y;
  const auto flat_idx = [num_x, layer_size](const Eigen::Vector3i& idx) {
    return static_cast<std::size_t>(idx.x()) +
        static_cast<std::size_t>(idx.y()) * num_x +
        static_cast<std::size_t>(idx.z()) * layer_size;
  };
  // 0: 未訪問、1: 表面、2: 外部空間
  std::vector<std::uint8_t> cell_states(num_cells, 0);
  for (const auto cell : surface_cells) {
    const Eigen::Vector3i idx = grid.getIndexFromFlatId(cell) - min_idx;
    if ((idx.array() <= 0).any() ||
        (idx.array() >= (grid_size.array() - 1)).any()) {
      throw std::runtime_error("Solid voxel mesh escaped padded grid");
    }
    cell_states[flat_idx(idx)] = 1;
  }

  std::vector<std::uint32_t> outside_cells;
  outside_cells.reserve(num_cells);
  outside_cells.push_back(0);
  cell_states[0] = 2;
  const auto append_outside = [&](std::size_t idx) {
    if (cell_states[idx] == 0) {
      cell_states[idx] = 2;
      outside_cells.push_back(static_cast<std::uint32_t>(idx));
    }
  };
  for (std::size_t queue_idx = 0; queue_idx < outside_cells.size(); ++queue_idx) {
    const std::size_t idx = outside_cells[queue_idx];
    const std::size_t x = idx % num_x;
    const std::size_t y = (idx / num_x) % num_y;
    const std::size_t z = idx / layer_size;
    if (x > 0) append_outside(idx - 1);
    if (x + 1 < num_x) append_outside(idx + 1);
    if (y > 0) append_outside(idx - num_x);
    if (y + 1 < num_y) append_outside(idx + num_x);
    if (z > 0) append_outside(idx - layer_size);
    if (z + 1 < num_z) append_outside(idx + layer_size);
  }
  std::vector<std::uint32_t>().swap(outside_cells);

  solid_voxel_geometry result;
  std::unordered_set<std::size_t> component_idxs;
  for (std::size_t vertex_idx = 0; vertex_idx < vertices.size(); ++vertex_idx) {
    const auto unique_idx = vertex_ids[vertex_idx];
    if (has_surface_vertex[unique_idx] &&
        component_idxs.insert(find_component(unique_idx)).second) {
      result.surface_component_points.push_back(vertices[vertex_idx]);
    }
  }
  result.num_grid_cells = num_cells;
  result.num_surface_cells = surface_cells.size();
  result.tree = std::make_shared<octomap::OcTree>(voxel_size);
  for (std::size_t idx = 0; idx < num_cells; ++idx) {
    if (cell_states[idx] == 2) {
      continue;
    }
    const int x = static_cast<int>(idx % num_x) + min_idx.x();
    const int y = static_cast<int>((idx / num_x) % num_y) + min_idx.y();
    const int z = static_cast<int>(idx / layer_size) + min_idx.z();
    // 両ライブラリで共通のセル中心 (idx + 0.5) * voxel_size
    octomap::OcTreeKey key;
    if (!result.tree->coordToKeyChecked(
            (static_cast<double>(x) + 0.5) * voxel_size,
            (static_cast<double>(y) + 0.5) * voxel_size,
            (static_cast<double>(z) + 0.5) * voxel_size, key) ||
        !result.tree->updateNode(key, true, true)) {
      throw std::runtime_error("Failed to register solid voxel mesh cell");
    }
    if (cell_states[idx] == 0) {
      ++result.num_interior_cells;
    }
  }
  result.tree->updateInnerOccupancy();
  result.tree->prune();
  return result;
}

}  // namespace collision
