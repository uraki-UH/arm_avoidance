#include <teaser/registration.h>

#include <zlib.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

using json = nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxCorrespondenceNum = 120U;
constexpr std::size_t kSpacingSampleNum = 128U;
constexpr double kFallbackSpacing = 0.005;
constexpr double kMinNoiseBound = 0.006;
constexpr double kMaxNoiseBound = 0.020;
constexpr double kMinNormalAlignment = 0.50;
constexpr int kYawStepDeg = 10;
constexpr std::size_t kIcpIterationNum = 8U;

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Node
{
  Vec3 position;
  Vec3 normal;
};

struct Graph
{
  std::string template_id;
  std::vector<Node> nodes;
};

struct MatchResult
{
  std::string template_id;
  std::size_t correspondence_num = 0U;
  std::size_t inlier_num = 0U;
  double inlier_ratio = 0.0;
  double median_dist = std::numeric_limits<double>::infinity();
  double score = 0.0;
  double teaser_ms = 0.0;
  double yaw_deg = 0.0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct RigidTransform
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct SpatialMatch
{
  std::size_t query_index = 0U;
  std::size_t template_index = 0U;
  double distance = 0.0;
};

bool isFinite(const double value)
{
  return std::isfinite(value);
}

Vec3 subtract(const Vec3 &first, const Vec3 &second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

double dot(const Vec3 &first, const Vec3 &second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

double length(const Vec3 &value)
{
  return std::sqrt(dot(value, value));
}

Vec3 normalize(const Vec3 &value)
{
  const double value_length = length(value);
  if (value_length <= 1e-12) {
    return {0.0, 0.0, 1.0};
  }
  return {value.x / value_length, value.y / value_length, value.z / value_length};
}

std::string readDatasetText(const std::filesystem::path &dataset_path)
{
  if (dataset_path.extension() != ".gz") {
    std::ifstream stream(dataset_path);
    if (!stream) {
      throw std::runtime_error("テンプレートJSONを開けません: " + dataset_path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }

  gzFile file = gzopen(dataset_path.c_str(), "rb");
  if (file == nullptr) {
    throw std::runtime_error("テンプレートgzipを開けません: " + dataset_path.string());
  }
  std::string text;
  std::array<char, 64U * 1024U> buffer{};
  int read_byte_num = 0;
  while ((read_byte_num = gzread(file, buffer.data(), buffer.size())) > 0) {
    text.append(buffer.data(), static_cast<std::size_t>(read_byte_num));
  }
  const bool has_read_error = read_byte_num < 0;
  const int close_result = gzclose(file);
  if (has_read_error || close_result != Z_OK) {
    throw std::runtime_error("テンプレートgzipの読込失敗: " + dataset_path.string());
  }
  return text;
}

Graph loadGraph(const std::filesystem::path &dataset_path)
{
  const json root = json::parse(readDatasetText(dataset_path));
  const json *template_root = &root;
  if (root.contains("gng_template") && root.at("gng_template").is_object()) {
    template_root = &root.at("gng_template");
  }
  if (!template_root->contains("gng") || !template_root->at("gng").is_object() ||
    !template_root->at("gng").contains("nodes") || !template_root->at("gng").at("nodes").is_array())
  {
    throw std::runtime_error("GNG node配列がありません: " + dataset_path.string());
  }

  Graph graph;
  graph.template_id = root.value(
    "template_id", template_root->value("template_id", dataset_path.stem().string()));
  for (const json &node_json : template_root->at("gng").at("nodes")) {
    if (!node_json.is_object()) {
      continue;
    }
    const Vec3 position = {
      node_json.value("x", std::numeric_limits<double>::quiet_NaN()),
      node_json.value("y", std::numeric_limits<double>::quiet_NaN()),
      node_json.value("z", std::numeric_limits<double>::quiet_NaN())};
    const Vec3 normal = normalize({
      node_json.value("nx", 0.0), node_json.value("ny", 0.0), node_json.value("nz", 1.0)});
    if (!isFinite(position.x) || !isFinite(position.y) || !isFinite(position.z)) {
      continue;
    }
    graph.nodes.push_back({position, normal});
  }
  if (graph.nodes.size() < 3U) {
    throw std::runtime_error("GNG node数が不足しています: " + dataset_path.string());
  }
  return graph;
}

double estimateSpacing(const Graph &graph)
{
  const std::size_t sample_num = std::min(kSpacingSampleNum, graph.nodes.size());
  double distance_sum = 0.0;
  for (std::size_t sample_index = 0U; sample_index < sample_num; ++sample_index) {
    const std::size_t node_index = sample_index * graph.nodes.size() / sample_num;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (std::size_t other_index = 0U; other_index < graph.nodes.size(); ++other_index) {
      if (other_index == node_index) {
        continue;
      }
      nearest_dist = std::min(nearest_dist, length(subtract(
        graph.nodes[node_index].position, graph.nodes[other_index].position)));
    }
    distance_sum += nearest_dist;
  }
  return sample_num == 0U ? kFallbackSpacing : distance_sum / sample_num;
}

Eigen::Vector3d centroid(const Graph &graph)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (const Node &node : graph.nodes) {
    result += Eigen::Vector3d(node.position.x, node.position.y, node.position.z);
  }
  return result / static_cast<double>(graph.nodes.size());
}

Eigen::Matrix3d makeYawRotation(const double yaw_deg)
{
  const double yaw_rad = yaw_deg * kPi / 180.0;
  Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
  result(0, 0) = std::cos(yaw_rad);
  result(0, 1) = -std::sin(yaw_rad);
  result(1, 0) = std::sin(yaw_rad);
  result(1, 1) = std::cos(yaw_rad);
  return result;
}

Vec3 transformPosition(const RigidTransform &transform, const Vec3 &position)
{
  const Eigen::Vector3d transformed = transform.rotation * Eigen::Vector3d(
    position.x, position.y, position.z) + transform.translation;
  return {transformed.x(), transformed.y(), transformed.z()};
}

std::vector<SpatialMatch> makeSpatialMatches(
  const Graph &query, const Graph &candidate, const RigidTransform &transform)
{
  std::vector<SpatialMatch> matches;
  matches.reserve(query.nodes.size());
  for (std::size_t query_index = 0U; query_index < query.nodes.size(); ++query_index) {
    const Vec3 transformed_position = transformPosition(transform, query.nodes[query_index].position);
    SpatialMatch best_match;
    best_match.query_index = query_index;
    best_match.distance = std::numeric_limits<double>::infinity();
    for (std::size_t template_index = 0U; template_index < candidate.nodes.size(); ++template_index) {
      const double distance = length(subtract(transformed_position, candidate.nodes[template_index].position));
      if (distance < best_match.distance) {
        best_match.template_index = template_index;
        best_match.distance = distance;
      }
    }
    matches.push_back(best_match);
  }
  std::sort(matches.begin(), matches.end(), [](const SpatialMatch &first, const SpatialMatch &second) {
    return first.distance < second.distance;
  });
  return matches;
}

RigidTransform estimateRigidTransform(
  const Graph &query, const Graph &candidate, const std::vector<SpatialMatch> &matches)
{
  Eigen::Vector3d query_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d template_center = Eigen::Vector3d::Zero();
  for (const SpatialMatch &match : matches) {
    const Vec3 &query_position = query.nodes[match.query_index].position;
    const Vec3 &template_position = candidate.nodes[match.template_index].position;
    query_center += Eigen::Vector3d(query_position.x, query_position.y, query_position.z);
    template_center += Eigen::Vector3d(template_position.x, template_position.y, template_position.z);
  }
  query_center /= static_cast<double>(matches.size());
  template_center /= static_cast<double>(matches.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const SpatialMatch &match : matches) {
    const Vec3 &query_position = query.nodes[match.query_index].position;
    const Vec3 &template_position = candidate.nodes[match.template_index].position;
    const Eigen::Vector3d query_centered =
      Eigen::Vector3d(query_position.x, query_position.y, query_position.z) - query_center;
    const Eigen::Vector3d template_centered =
      Eigen::Vector3d(template_position.x, template_position.y, template_position.z) - template_center;
    covariance += query_centered * template_centered.transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d right = svd.matrixV();
  const Eigen::Matrix3d left = svd.matrixU();
  Eigen::Matrix3d rotation = right * left.transpose();
  if (rotation.determinant() < 0.0) {
    right.col(2) *= -1.0;
    rotation = right * left.transpose();
  }
  return {rotation, template_center - rotation * query_center};
}

double median(std::vector<double> values);

RigidTransform findYawIcpTransform(const Graph &query, const Graph &candidate)
{
  const Eigen::Vector3d query_center = centroid(query);
  const Eigen::Vector3d template_center = centroid(candidate);
  RigidTransform best_transform;
  double best_median_dist = std::numeric_limits<double>::infinity();
  for (int yaw_deg = 0; yaw_deg < 360; yaw_deg += kYawStepDeg) {
    RigidTransform transform;
    transform.rotation = makeYawRotation(static_cast<double>(yaw_deg));
    transform.translation = template_center - transform.rotation * query_center;
    for (std::size_t iteration_index = 0U; iteration_index < kIcpIterationNum; ++iteration_index) {
      std::vector<SpatialMatch> matches = makeSpatialMatches(query, candidate, transform);
      const std::size_t keep_num = std::max(
        static_cast<std::size_t>(3U), (matches.size() * 3U) / 4U);
      matches.resize(keep_num);
      transform = estimateRigidTransform(query, candidate, matches);
    }
    const std::vector<SpatialMatch> matches = makeSpatialMatches(query, candidate, transform);
    std::vector<double> distances;
    distances.reserve(matches.size());
    for (const SpatialMatch &match : matches) {
      distances.push_back(match.distance);
    }
    const double median_dist = median(std::move(distances));
    if (median_dist < best_median_dist) {
      best_median_dist = median_dist;
      best_transform = transform;
    }
  }
  return best_transform;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const std::size_t middle_index = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle_index), values.end());
  const double upper = values[middle_index];
  if (values.size() % 2U != 0U) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle_index - 1U), values.end());
  return 0.5 * (upper + values[middle_index - 1U]);
}

MatchResult evaluateTemplate(const Graph &query, const Graph &candidate)
{
  MatchResult result;
  result.template_id = candidate.template_id;
  const RigidTransform icp_transform = findYawIcpTransform(query, candidate);
  const std::vector<SpatialMatch> spatial_matches = makeSpatialMatches(
    query, candidate, icp_transform);
  std::set<std::size_t> used_template_indices;
  std::vector<SpatialMatch> matches;
  matches.reserve(kMaxCorrespondenceNum);
  for (const SpatialMatch &match : spatial_matches) {
    if (!used_template_indices.insert(match.template_index).second) {
      continue;
    }
    matches.push_back(match);
    if (matches.size() == kMaxCorrespondenceNum) {
      break;
    }
  }
  result.correspondence_num = matches.size();
  if (result.correspondence_num < 3U) {
    return result;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> source_points(3, matches.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> target_points(3, matches.size());
  for (std::size_t index = 0U; index < matches.size(); ++index) {
    const Vec3 &query_position = query.nodes[matches[index].query_index].position;
    const Vec3 &template_position = candidate.nodes[matches[index].template_index].position;
    source_points.col(static_cast<Eigen::Index>(index)) <<
      query_position.x, query_position.y, query_position.z;
    target_points.col(static_cast<Eigen::Index>(index)) <<
      template_position.x, template_position.y, template_position.z;
  }

  const double mean_spacing = 0.5 * (estimateSpacing(query) + estimateSpacing(candidate));
  const double noise_bound = std::clamp(1.5 * mean_spacing, kMinNoiseBound, kMaxNoiseBound);
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = noise_bound;
  params.cbar2 = 1.0;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = noise_bound;

  teaser::RobustRegistrationSolver solver(params);
  const auto begin = std::chrono::steady_clock::now();
  solver.solve(source_points, target_points);
  const auto end = std::chrono::steady_clock::now();
  const auto solution = solver.getSolution();
  result.teaser_ms = std::chrono::duration<double, std::milli>(end - begin).count();
  result.yaw_deg = std::atan2(solution.rotation(1, 0), solution.rotation(0, 0)) * 180.0 / kPi;
  result.translation = solution.translation;

  const double inlier_dist_th = 2.0 * noise_bound;
  const RigidTransform result_transform{solution.rotation, solution.translation};
  const std::vector<SpatialMatch> result_matches = makeSpatialMatches(
    query, candidate, result_transform);
  std::vector<double> residuals;
  residuals.reserve(query.nodes.size());
  for (const SpatialMatch &match : result_matches) {
    residuals.push_back(match.distance);
    const Vec3 &query_normal = query.nodes[match.query_index].normal;
    const Eigen::Vector3d rotated_normal = solution.rotation * Eigen::Vector3d(
      query_normal.x, query_normal.y, query_normal.z);
    const Vec3 &template_normal = candidate.nodes[match.template_index].normal;
    const double normal_alignment = std::abs(dot(
      {rotated_normal.x(), rotated_normal.y(), rotated_normal.z()}, template_normal));
    if (match.distance <= inlier_dist_th && normal_alignment >= kMinNormalAlignment) {
      ++result.inlier_num;
    }
  }
  result.inlier_ratio = static_cast<double>(result.inlier_num) / query.nodes.size();
  result.median_dist = median(std::move(residuals));
  result.score = result.inlier_ratio * std::exp(-result.median_dist / inlier_dist_th);
  return result;
}

bool hasTemplateSuffix(const std::string &name)
{
  constexpr std::string_view gzip_suffix = "_gng_template.json.gz";
  constexpr std::string_view json_suffix = "_gng_template.json";
  return (name.size() >= gzip_suffix.size() &&
    name.compare(name.size() - gzip_suffix.size(), gzip_suffix.size(), gzip_suffix) == 0) ||
    (name.size() >= json_suffix.size() &&
    name.compare(name.size() - json_suffix.size(), json_suffix.size(), json_suffix) == 0);
}

std::vector<std::filesystem::path> findTemplatePaths(const std::filesystem::path &dataset_dir)
{
  std::vector<std::filesystem::path> paths;
  for (const auto &entry : std::filesystem::directory_iterator(dataset_dir)) {
    if (entry.is_regular_file() && hasTemplateSuffix(entry.path().filename().string())) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

void printUsage()
{
  std::cerr <<
    "使用法: teaser_template_retrieval dataset_dir query_dataset [除外template_id ...]" << std::endl;
}

}  // 無名namespace

int main(int argc, char **argv)
{
  if (argc < 3) {
    printUsage();
    return 1;
  }
  try {
    const std::filesystem::path dataset_dir(argv[1]);
    const std::filesystem::path query_path(argv[2]);
    const Graph query = loadGraph(query_path);
    std::set<std::string> excluded_ids;
    excluded_ids.insert(query.template_id);
    for (int index = 3; index < argc; ++index) {
      excluded_ids.insert(argv[index]);
    }

    std::vector<MatchResult> results;
    for (const auto &template_path : findTemplatePaths(dataset_dir)) {
      const Graph candidate = loadGraph(template_path);
      if (excluded_ids.count(candidate.template_id) > 0U) {
        continue;
      }
      results.push_back(evaluateTemplate(query, candidate));
    }
    std::sort(results.begin(), results.end(), [](const MatchResult &first, const MatchResult &second) {
      return first.score == second.score ? first.template_id < second.template_id :
             first.score > second.score;
    });

    std::cout << "query=" << query.template_id << " nodes=" << query.nodes.size() << std::endl;
    std::cout << "rank template score support median_m correspondences teaser_ms yaw_deg tx ty tz" << std::endl;
    for (std::size_t index = 0U; index < results.size(); ++index) {
      const MatchResult &result = results[index];
      std::cout << index + 1U << ' ' << result.template_id << ' ' << result.score << ' ' <<
        result.inlier_ratio << ' ' << result.median_dist << ' ' << result.correspondence_num << ' ' <<
        result.teaser_ms << ' ' << result.yaw_deg << ' ' << result.translation.x() << ' ' <<
        result.translation.y() << ' ' << result.translation.z() << std::endl;
    }
  } catch (const std::exception &error) {
    std::cerr << "評価失敗: " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
