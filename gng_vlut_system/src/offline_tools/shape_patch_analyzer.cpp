#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <zlib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

namespace {

struct Node {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

struct PlaneCluster {
  std::uint32_t id = 0U;
  std::vector<int> node_indices;
};

struct Descriptor {
  bool surfel_valid = false;
  bool quadric_valid = false;
  std::size_t sample_count = 0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d covariance_eigenvalues = Eigen::Vector3d::Zero();
  double quadric_mean_error = std::numeric_limits<double>::quiet_NaN();
  double quadric_p95_error = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 10, 1> quadric_coefficients = Eigen::Matrix<double, 10, 1>::Zero();
};

struct SurfaceComponent {
  std::uint32_t id = 0U;
  std::string model_type;
  std::vector<int> node_indices;
  std::vector<std::uint32_t> adjacent_plane_cluster_ids;
  Descriptor descriptor;
};

struct VersionResult {
  std::string name;
  std::size_t point_count = 0;
  std::size_t assigned_count = 0;
  std::size_t surfel_valid_count = 0;
  std::size_t quadric_valid_count = 0;
  double elapsed_ms = 0.0;
  std::vector<Descriptor> descriptors;
};

struct Options {
  std::string template_path;
  std::string source_dir;
  float voxel_size = 0.0f;
  std::string output_path;
  bool self_test = false;
};

Eigen::Vector3d json_vec3(const Json &node, const char *x, const char *y, const char *z) {
  return Eigen::Vector3d(node.value(x, 0.0), node.value(y, 0.0), node.value(z, 0.0));
}

std::string read_file(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open " + path);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string read_gzip(const std::string &path) {
  gzFile file = gzopen(path.c_str(), "rb");
  if (!file) throw std::runtime_error("cannot open gzip file " + path);
  std::string result;
  char buffer[64 * 1024];
  int read = 0;
  while ((read = gzread(file, buffer, sizeof(buffer))) > 0) result.append(buffer, read);
  const int error = gzclose(file);
  if (error != Z_OK) throw std::runtime_error("failed to read gzip file " + path);
  return result;
}

Json load_json(const std::string &path) {
  const bool is_gzip = path.size() >= 3 && path.compare(path.size() - 3, 3, ".gz") == 0;
  return Json::parse(is_gzip ? read_gzip(path) : read_file(path));
}

std::vector<Node> load_nodes(const Json &root, std::vector<std::pair<int, int>> &edges) {
  std::vector<Node> nodes;
  const auto &gng = root.at("gng");
  for (const auto &item : gng.at("nodes")) {
    Node node;
    node.position = json_vec3(item, "x", "y", "z");
    node.normal = json_vec3(item, "nx", "ny", "nz");
    nodes.push_back(node);
  }
  for (const auto &edge : gng.value("edges", Json::array())) {
    if (edge.size() != 2) continue;
    const int a = edge[0].get<int>();
    const int b = edge[1].get<int>();
    if (a >= 0 && b >= 0 && a < static_cast<int>(nodes.size()) && b < static_cast<int>(nodes.size())) {
      edges.emplace_back(a, b);
    }
  }
  return nodes;
}

std::vector<PlaneCluster> load_plane_clusters(const Json &root, const std::size_t node_count) {
  std::vector<PlaneCluster> clusters;
  for (const auto &item : root.at("gng").value("plane_clusters", Json::array())) {
    PlaneCluster cluster;
    cluster.id = item.value("id", static_cast<std::uint32_t>(clusters.size()));
    const auto &indices = item.contains("idx") ? item.at("idx") : item.value("node_indices", Json::array());
    for (const auto &index : indices) {
      const int node_index = index.get<int>();
      if (node_index >= 0 && node_index < static_cast<int>(node_count)) {
        cluster.node_indices.push_back(node_index);
      }
    }
    if (!cluster.node_indices.empty()) clusters.push_back(std::move(cluster));
  }
  return clusters;
}

std::vector<std::vector<int>> build_patch_neighbors(
    const std::vector<Node> &nodes, const std::vector<std::pair<int, int>> &edges) {
  std::vector<std::vector<int>> neighbors(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) neighbors[i].push_back(static_cast<int>(i));
  constexpr double normal_cos = 0.5;  // Keep locally coherent patches (within 60 degrees).
  for (const auto &[a, b] : edges) {
    const double na = nodes[a].normal.norm();
    const double nb = nodes[b].normal.norm();
    if (na > 1e-9 && nb > 1e-9 && nodes[a].normal.dot(nodes[b].normal) / (na * nb) < normal_cos) continue;
    neighbors[a].push_back(b);
    neighbors[b].push_back(a);
  }
  return neighbors;
}

Eigen::Matrix<double, 10, 1> quadric_features(const Eigen::Vector3d &p) {
  const double x = p.x(), y = p.y(), z = p.z();
  Eigen::Matrix<double, 10, 1> phi;
  phi << x * x, y * y, z * z, x * y, x * z, y * z, x, y, z, 1.0;
  return phi;
}

double quadric_value(const Eigen::Matrix<double, 10, 1> &q, const Eigen::Vector3d &p) {
  return q.dot(quadric_features(p));
}

double quadric_distance(const Eigen::Matrix<double, 10, 1> &q, const Eigen::Vector3d &p) {
  const double x = p.x(), y = p.y(), z = p.z();
  Eigen::Vector3d gradient(
      2.0 * q(0) * x + q(3) * y + q(4) * z + q(6),
      2.0 * q(1) * y + q(3) * x + q(5) * z + q(7),
      2.0 * q(2) * z + q(4) * x + q(5) * y + q(8));
  return std::abs(quadric_value(q, p)) / std::max(gradient.norm(), 1e-12);
}

Descriptor fit_descriptor(const std::vector<Eigen::Vector3d> &points) {
  Descriptor descriptor;
  descriptor.sample_count = points.size();
  if (points.size() < 6) return descriptor;

  for (const auto &point : points) descriptor.mean += point;
  descriptor.mean /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &point : points) {
    const Eigen::Vector3d delta = point - descriptor.mean;
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= static_cast<double>(points.size() - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance_solver(covariance);
  if (covariance_solver.info() == Eigen::Success) {
    descriptor.covariance_eigenvalues = covariance_solver.eigenvalues();
    descriptor.surfel_valid = descriptor.covariance_eigenvalues(2) > 1e-12;
  }

  if (points.size() < 30) return descriptor;
  Eigen::Matrix<double, 10, 10> moment = Eigen::Matrix<double, 10, 10>::Zero();
  for (const auto &point : points) {
    const auto phi = quadric_features(point - descriptor.mean);
    moment.noalias() += phi * phi.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 10, 10>> solver(moment);
  if (solver.info() != Eigen::Success) return descriptor;
  const auto eigenvalues = solver.eigenvalues();
  const double total = std::max(eigenvalues.cwiseAbs().sum(), 1e-18);
  if (eigenvalues(9) <= 1e-12 * total) return descriptor;

  const auto q = solver.eigenvectors().col(0);
  descriptor.quadric_coefficients = q;
  std::vector<double> errors;
  errors.reserve(points.size());
  for (const auto &point : points) errors.push_back(quadric_distance(q, point - descriptor.mean));
  std::sort(errors.begin(), errors.end());
  descriptor.quadric_mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  descriptor.quadric_p95_error = errors[static_cast<std::size_t>(0.95 * (errors.size() - 1))];
  descriptor.quadric_valid = std::isfinite(descriptor.quadric_mean_error);
  return descriptor;
}

std::vector<std::vector<Eigen::Vector3d>> assign_points_to_nodes(
    const Cloud::ConstPtr &cloud, const std::vector<Node> &nodes);

VersionResult analyze_version(
    const std::string &name, const Cloud::ConstPtr &cloud, const std::vector<Node> &nodes,
    const std::vector<std::vector<int>> &patch_neighbors) {
  const auto begin = std::chrono::steady_clock::now();
  VersionResult result;
  result.name = name;
  result.point_count = cloud->size();
  result.descriptors.resize(nodes.size());

  const auto node_points = assign_points_to_nodes(cloud, nodes);
  for (const auto &points : node_points) result.assigned_count += points.size();

  for (std::size_t i = 0; i < nodes.size(); ++i) {
    std::vector<Eigen::Vector3d> patch;
    for (const int neighbor : patch_neighbors[i]) {
      patch.insert(patch.end(), node_points[neighbor].begin(), node_points[neighbor].end());
    }
    result.descriptors[i] = fit_descriptor(patch);
    result.surfel_valid_count += result.descriptors[i].surfel_valid ? 1 : 0;
    result.quadric_valid_count += result.descriptors[i].quadric_valid ? 1 : 0;
  }
  result.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}

double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

void print_result(const VersionResult &result) {
  std::cout << "  " << std::left << std::setw(10) << result.name << std::right
            << " points=" << std::setw(6) << result.point_count
            << " assigned=" << std::setw(6) << result.assigned_count
            << " surfel=" << std::setw(4) << result.surfel_valid_count
            << " quadric=" << std::setw(4) << result.quadric_valid_count
            << " time_ms=" << std::fixed << std::setprecision(3) << result.elapsed_ms << '\n';
  std::vector<double> quadric_errors;
  for (const auto &descriptor : result.descriptors) {
    if (descriptor.quadric_valid) quadric_errors.push_back(descriptor.quadric_mean_error);
  }
  if (!quadric_errors.empty()) {
    std::cout << "    quadric mean residual median=" << std::setprecision(6) << median(quadric_errors) << " m\n";
  }
}

void print_stability(const VersionResult &reference, const VersionResult &other) {
  std::vector<double> axis_changes;
  std::vector<double> residual_changes;
  for (std::size_t i = 0; i < reference.descriptors.size(); ++i) {
    const auto &a = reference.descriptors[i];
    const auto &b = other.descriptors[i];
    if (a.surfel_valid && b.surfel_valid) {
      const auto ar = a.covariance_eigenvalues.array().max(1e-12);
      const auto br = b.covariance_eigenvalues.array().max(1e-12);
      axis_changes.push_back((ar.log() - br.log()).cwiseAbs().maxCoeff());
    }
    if (a.quadric_valid && b.quadric_valid) {
      residual_changes.push_back(std::abs(a.quadric_mean_error - b.quadric_mean_error));
    }
  }
  std::cout << "  stability full_vs_" << other.name
            << ": surfel_matched=" << axis_changes.size()
            << " median_log_eigen_change=" << std::setprecision(6) << median(axis_changes)
            << " quadric_matched=" << residual_changes.size()
            << " median_residual_change_m=" << median(residual_changes) << '\n';
}

Cloud::Ptr every_nth(const Cloud::ConstPtr &source, int stride) {
  Cloud::Ptr result(new Cloud);
  result->reserve((source->size() + stride - 1) / stride);
  for (std::size_t i = 0; i < source->size(); i += static_cast<std::size_t>(stride)) result->push_back((*source)[i]);
  return result;
}

std::vector<std::vector<Eigen::Vector3d>> assign_points_to_nodes(
    const Cloud::ConstPtr &cloud, const std::vector<Node> &nodes) {
  pcl::PointCloud<Point>::Ptr node_cloud(new pcl::PointCloud<Point>);
  node_cloud->resize(nodes.size());
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    (*node_cloud)[i].x = static_cast<float>(nodes[i].position.x());
    (*node_cloud)[i].y = static_cast<float>(nodes[i].position.y());
    (*node_cloud)[i].z = static_cast<float>(nodes[i].position.z());
  }
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(node_cloud);
  std::vector<std::vector<Eigen::Vector3d>> node_points(nodes.size());
  for (const auto &point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    std::vector<int> local_indices;
    std::vector<float> local_distances;
    if (tree.nearestKSearch(point, 1, local_indices, local_distances) == 1) {
      node_points[local_indices[0]].emplace_back(point.x, point.y, point.z);
    }
  }
  return node_points;
}

std::vector<SurfaceComponent> build_surface_components(
    const Cloud::ConstPtr &cloud, const std::vector<Node> &nodes,
    const std::vector<std::pair<int, int>> &edges,
    const std::vector<PlaneCluster> &plane_clusters) {
  const auto node_points = assign_points_to_nodes(cloud, nodes);
  std::vector<int> plane_owner(nodes.size(), -1);
  for (std::size_t cluster_index = 0; cluster_index < plane_clusters.size(); ++cluster_index) {
    for (const int node_index : plane_clusters[cluster_index].node_indices) {
      if (plane_owner[node_index] < 0) plane_owner[node_index] = static_cast<int>(cluster_index);
    }
  }

  std::vector<std::vector<int>> adjacency(nodes.size());
  for (const auto &[a, b] : edges) {
    adjacency[a].push_back(b);
    adjacency[b].push_back(a);
  }

  std::vector<bool> visited(nodes.size(), false);
  std::vector<SurfaceComponent> components;
  for (std::size_t start = 0; start < nodes.size(); ++start) {
    if (plane_owner[start] >= 0 || visited[start]) continue;
    SurfaceComponent component;
    std::vector<int> queue{static_cast<int>(start)};
    visited[start] = true;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const int node_index = queue[cursor];
      component.node_indices.push_back(node_index);
      for (const int neighbor : adjacency[node_index]) {
        if (plane_owner[neighbor] >= 0 || visited[neighbor]) continue;
        visited[neighbor] = true;
        queue.push_back(neighbor);
      }
    }
    if (component.node_indices.size() < 2U) continue;

    std::set<std::uint32_t> adjacent_cluster_ids;
    for (const int node_index : component.node_indices) {
      for (const int neighbor : adjacency[node_index]) {
        if (plane_owner[neighbor] >= 0) {
          adjacent_cluster_ids.insert(plane_clusters[plane_owner[neighbor]].id);
        }
      }
    }
    component.adjacent_plane_cluster_ids.assign(
        adjacent_cluster_ids.begin(), adjacent_cluster_ids.end());

    std::vector<Eigen::Vector3d> points;
    for (const int node_index : component.node_indices) {
      points.insert(points.end(), node_points[node_index].begin(), node_points[node_index].end());
    }
    component.descriptor = fit_descriptor(points);
    if (component.descriptor.quadric_valid && !component.adjacent_plane_cluster_ids.empty()) {
      component.model_type = "surface_extension";
    } else if (component.descriptor.quadric_valid) {
      component.model_type = "curved_component";
    } else {
      component.model_type = "unassigned_nonplane";
    }
    component.id = static_cast<std::uint32_t>(components.size());
    components.push_back(std::move(component));
  }
  return components;
}

Json descriptor_json(const Descriptor &descriptor) {
  Json output = {
    {"sample_count", descriptor.sample_count},
    {"surfel_valid", descriptor.surfel_valid},
    {"quadric_valid", descriptor.quadric_valid},
    {"mean", {descriptor.mean.x(), descriptor.mean.y(), descriptor.mean.z()}},
    {"covariance_eigenvalues", {
        descriptor.covariance_eigenvalues(0), descriptor.covariance_eigenvalues(1),
        descriptor.covariance_eigenvalues(2)}},
    {"quadric_mean_error_m", descriptor.quadric_mean_error},
    {"quadric_p95_error_m", descriptor.quadric_p95_error},
  };
  if (descriptor.quadric_valid) {
    output["quadric_coefficients_local"] = std::vector<double>(
        descriptor.quadric_coefficients.data(), descriptor.quadric_coefficients.data() + 10);
  }
  return output;
}

void print_surface_components(const std::vector<SurfaceComponent> &components) {
  std::cout << "surface_components=" << components.size() << '\n';
  for (const auto &component : components) {
    std::cout << "  component=" << component.id << " type=" << component.model_type
              << " nodes=" << component.node_indices.size()
              << " adjacent_planes=" << component.adjacent_plane_cluster_ids.size()
              << " points=" << component.descriptor.sample_count
              << " quadric=" << (component.descriptor.quadric_valid ? "valid" : "invalid")
              << " residual_m=" << std::setprecision(6)
              << component.descriptor.quadric_mean_error << '\n';
  }
}

void write_surface_components(
    const std::string &path, const std::vector<SurfaceComponent> &components) {
  Json output = Json::object();
  output["schema"] = "surface_components_prototype_v1";
  output["components"] = Json::array();
  for (const auto &component : components) {
    output["components"].push_back({
      {"id", component.id},
      {"model_type", component.model_type},
      {"node_indices", component.node_indices},
      {"adjacent_plane_cluster_ids", component.adjacent_plane_cluster_ids},
      {"descriptor", descriptor_json(component.descriptor)},
    });
  }
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot write output " + path);
  file << std::setw(2) << output << '\n';
}

bool self_test() {
  std::vector<Eigen::Vector3d> cylinder;
  for (int iz = 0; iz < 41; ++iz) {
    const double z = -0.2 + 0.4 * iz / 40.0;
    for (int it = 0; it < 360; ++it) {
      const double theta = 2.0 * M_PI * it / 360.0;
      cylinder.emplace_back(0.1 * std::cos(theta), 0.1 * std::sin(theta), z);
    }
  }
  std::vector<Eigen::Vector3d> sphere;
  for (int ip = 1; ip < 90; ++ip) {
    const double phi = M_PI * ip / 90.0;
    for (int it = 0; it < 180; ++it) {
      const double theta = 2.0 * M_PI * it / 180.0;
      sphere.emplace_back(0.12 * std::sin(phi) * std::cos(theta),
                          0.12 * std::sin(phi) * std::sin(theta), 0.12 * std::cos(phi));
    }
  }
  const auto cylinder_descriptor = fit_descriptor(cylinder);
  const auto sphere_descriptor = fit_descriptor(sphere);
  const bool cylinder_ok = cylinder_descriptor.quadric_valid && cylinder_descriptor.quadric_mean_error < 1e-6;
  const bool sphere_ok = sphere_descriptor.quadric_valid && sphere_descriptor.quadric_mean_error < 1e-6;
  std::cout << "self_test cylinder: valid=" << cylinder_descriptor.quadric_valid
            << " mean_error_m=" << std::setprecision(9) << cylinder_descriptor.quadric_mean_error << '\n';
  std::cout << "self_test sphere:   valid=" << sphere_descriptor.quadric_valid
            << " mean_error_m=" << std::setprecision(9) << sphere_descriptor.quadric_mean_error << '\n';
  return cylinder_ok && sphere_ok;
}

void usage(const char *program) {
  std::cout << "Usage: " << program << " TEMPLATE.json[.gz] [--source-dir DIR] [--voxel-size M] [--output PATH]\n"
            << "       " << program << " --self-test\n";
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--self-test") {
      options.self_test = true;
    } else if (arg == "--source-dir" && i + 1 < argc) {
      options.source_dir = argv[++i];
    } else if (arg == "--voxel-size" && i + 1 < argc) {
      options.voxel_size = std::stof(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      options.output_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else if (options.template_path.empty()) {
      options.template_path = arg;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parse_options(argc, argv);
    if (options.self_test) return self_test() ? 0 : 1;
    if (options.template_path.empty()) {
      usage(argv[0]);
      return 2;
    }

    const Json root = load_json(options.template_path);
    std::vector<std::pair<int, int>> edges;
    const auto nodes = load_nodes(root, edges);
    const auto patch_neighbors = build_patch_neighbors(nodes, edges);
    const auto &source = root.at("source").at("point_cloud");
    fs::path source_path = source.value("file_name", "");
    if (!source_path.is_absolute()) {
      source_path = options.source_dir.empty() ? fs::path(options.template_path).parent_path() : fs::path(options.source_dir);
      source_path /= source.value("file_name", "");
    }

    Cloud::Ptr cloud(new Cloud);
    if (pcl::io::loadPCDFile<Point>(source_path.string(), *cloud) != 0) {
      throw std::runtime_error("failed to load source PCD " + source_path.string());
    }
    if (options.voxel_size > 0.0f) {
      pcl::VoxelGrid<Point> filter;
      filter.setInputCloud(cloud);
      filter.setLeafSize(options.voxel_size, options.voxel_size, options.voxel_size);
      Cloud::Ptr filtered(new Cloud);
      filter.filter(*filtered);
      cloud = filtered;
    }

    std::cout << "shape_patch_analyzer\n"
              << "  template=" << options.template_path << '\n'
              << "  source=" << source_path << '\n'
              << "  nodes=" << nodes.size() << " edges=" << edges.size()
              << " patch_normal_limit_deg=60\n";
    const auto full = analyze_version("full", cloud, nodes, patch_neighbors);
    const auto half = analyze_version("stride2", every_nth(cloud, 2), nodes, patch_neighbors);
    const auto quarter = analyze_version("stride4", every_nth(cloud, 4), nodes, patch_neighbors);
    print_result(full);
    print_result(half);
    print_result(quarter);
    print_stability(full, half);
    print_stability(full, quarter);
    const auto plane_clusters = load_plane_clusters(root, nodes.size());
    const auto surface_components = build_surface_components(cloud, nodes, edges, plane_clusters);
    std::vector<bool> plane_owned(nodes.size(), false);
    for (const auto &cluster : plane_clusters) {
      for (const int node_index : cluster.node_indices) plane_owned[node_index] = true;
    }
    const auto plane_owned_count = std::count(plane_owned.begin(), plane_owned.end(), true);
    std::cout << "plane_clusters=" << plane_clusters.size()
              << " plane_owned_nodes=" << plane_owned_count
              << " nonplane_nodes=" << nodes.size() - plane_owned_count << '\n';
    print_surface_components(surface_components);
    if (!options.output_path.empty()) write_surface_components(options.output_path, surface_components);
  } catch (const std::exception &error) {
    std::cerr << "shape_patch_analyzer: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
