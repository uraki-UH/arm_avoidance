#include <zlib.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

using json = nlohmann::json;

constexpr double kDefaultMaxSizeExcessRatio = 0.35;
constexpr double kDefaultSizeExcessDev = 0.25;
constexpr double kDefaultSizeNearDev = 0.75;
constexpr double kDefaultNormalZDev = 0.20;
constexpr double kDefaultScaleProbeRatio = 1.5;
constexpr double kDefaultMaxSizeExpandRatio = 3.0;
constexpr std::size_t kDefaultMaxTokenNumPerPlaneProbe = 24U;
constexpr std::size_t kTopResultNum = 20U;

struct PlaneCluster
{
  double short_extent = 0.0;
  double long_extent = 0.0;
  double abs_normal_z = 0.0;
};

struct TemplateData
{
  std::string template_id;
  std::vector<PlaneCluster> plane_clusters;
};

struct PlaneToken
{
  std::size_t template_idx = 0U;
  PlaneCluster plane_cluster;
};

struct PlaneTokenIndex
{
  std::vector<PlaneToken> tokens;
  std::vector<std::size_t> short_order;
  std::vector<std::size_t> long_order;
};

struct Options
{
  double max_size_excess_ratio = kDefaultMaxSizeExcessRatio;
  double size_excess_dev = kDefaultSizeExcessDev;
  double size_near_dev = kDefaultSizeNearDev;
  double normal_z_dev = kDefaultNormalZDev;
  double scale_probe_ratio = kDefaultScaleProbeRatio;
  double max_size_expand_ratio = kDefaultMaxSizeExpandRatio;
  std::size_t max_token_num_per_plane_probe = kDefaultMaxTokenNumPerPlaneProbe;
};

struct Result
{
  std::string template_id;
  double score = 0.0;
  double min_plane_score = 0.0;
  double max_size_excess_ratio = 0.0;
};

bool isFinite(const double value)
{
  return std::isfinite(value);
}

double gaussianScore(const double value, const double dev)
{
  if (dev <= 0.0) {
    return value == 0.0 ? 1.0 : 0.0;
  }
  const double normalized_value = value / dev;
  return std::exp(-0.5 * normalized_value * normalized_value);
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

TemplateData loadTemplateData(const std::filesystem::path &dataset_path)
{
  const json root = json::parse(readDatasetText(dataset_path));
  const json *template_root = &root;
  if (root.contains("gng_template") && root.at("gng_template").is_object()) {
    template_root = &root.at("gng_template");
  }
  if (!template_root->contains("gng") || !template_root->at("gng").is_object()) {
    throw std::runtime_error("GNGがありません: " + dataset_path.string());
  }
  const json &gng = template_root->at("gng");
  TemplateData template_data;
  template_data.template_id = root.value(
    "template_id", template_root->value("template_id", dataset_path.stem().string()));
  if (!gng.contains("plane_clusters") || !gng.at("plane_clusters").is_array()) {
    return template_data;
  }
  for (const json &cluster_json : gng.at("plane_clusters")) {
    if (!cluster_json.is_object() || !cluster_json.contains("extent") ||
      !cluster_json.at("extent").is_array() || cluster_json.at("extent").size() != 2U ||
      !cluster_json.contains("normal") || !cluster_json.at("normal").is_array() ||
      cluster_json.at("normal").size() != 3U)
    {
      continue;
    }
    const double first_extent = cluster_json.at("extent").at(0).get<double>();
    const double second_extent = cluster_json.at("extent").at(1).get<double>();
    const double normal_z = cluster_json.at("normal").at(2).get<double>();
    if (!isFinite(first_extent) || !isFinite(second_extent) || !isFinite(normal_z) ||
      first_extent <= 0.0 || second_extent <= 0.0)
    {
      continue;
    }
    template_data.plane_clusters.push_back({
      std::min(first_extent, second_extent),
      std::max(first_extent, second_extent),
      std::abs(normal_z)});
  }
  return template_data;
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

double calculateSizeExcessRatio(const PlaneCluster &observed, const PlaneCluster &expected)
{
  return std::max({
    observed.short_extent / expected.short_extent - 1.0,
    observed.long_extent / expected.long_extent - 1.0,
    0.0});
}

double calculatePlaneScore(
  const PlaneCluster &observed, const PlaneCluster &expected, const Options &options)
{
  const double size_excess_ratio = calculateSizeExcessRatio(observed, expected);
  const double size_excess_score = gaussianScore(size_excess_ratio, options.size_excess_dev);
  const double observed_area = observed.short_extent * observed.long_extent;
  const double expected_area = expected.short_extent * expected.long_extent;
  const double size_near_score = gaussianScore(
    std::log(observed_area / expected_area), options.size_near_dev);
  const double normal_score = gaussianScore(
    std::abs(observed.abs_normal_z - expected.abs_normal_z), options.normal_z_dev);
  return 0.6 * size_excess_score + 0.2 * size_near_score + 0.2 * normal_score;
}

bool parseDoubleOption(
  const std::string &name, const std::string &value, double *destination)
{
  try {
    const double parsed_value = std::stod(value);
    if (!isFinite(parsed_value) || parsed_value < 0.0) {
      return false;
    }
    *destination = parsed_value;
    return true;
  } catch (const std::exception &) {
    static_cast<void>(name);
    return false;
  }
}

bool parseSizeOption(const std::string &value, std::size_t *destination)
{
  try {
    const unsigned long long parsed_value = std::stoull(value);
    if (parsed_value == 0U || parsed_value > std::numeric_limits<std::size_t>::max()) {
      return false;
    }
    *destination = static_cast<std::size_t>(parsed_value);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

PlaneTokenIndex buildPlaneTokenIndex(const std::vector<TemplateData> &templates)
{
  PlaneTokenIndex index;
  for (std::size_t template_idx = 0U; template_idx < templates.size(); ++template_idx) {
    for (const PlaneCluster &plane_cluster : templates[template_idx].plane_clusters) {
      index.tokens.push_back({template_idx, plane_cluster});
    }
  }
  index.short_order.resize(index.tokens.size());
  index.long_order.resize(index.tokens.size());
  for (std::size_t token_idx = 0U; token_idx < index.tokens.size(); ++token_idx) {
    index.short_order[token_idx] = token_idx;
    index.long_order[token_idx] = token_idx;
  }
  std::sort(index.short_order.begin(), index.short_order.end(), [&index](
    const std::size_t first_idx, const std::size_t second_idx) {
    return index.tokens[first_idx].plane_cluster.short_extent <
           index.tokens[second_idx].plane_cluster.short_extent;
  });
  std::sort(index.long_order.begin(), index.long_order.end(), [&index](
    const std::size_t first_idx, const std::size_t second_idx) {
    return index.tokens[first_idx].plane_cluster.long_extent <
           index.tokens[second_idx].plane_cluster.long_extent;
  });
  return index;
}

double calculateLogRatioDistance(const double first_value, const double second_value)
{
  return std::abs(std::log(first_value / second_value));
}

std::vector<std::size_t> findNearestTokenIndices(
  const PlaneTokenIndex &index, const std::vector<std::size_t> &order,
  const double target_extent, const bool use_short_extent, const std::size_t max_token_num)
{
  const auto lower = std::lower_bound(order.begin(), order.end(), target_extent, [&index, use_short_extent](
    const std::size_t token_idx, const double target) {
    const PlaneCluster &plane_cluster = index.tokens[token_idx].plane_cluster;
    const double extent = use_short_extent ? plane_cluster.short_extent : plane_cluster.long_extent;
    return extent < target;
  });
  std::ptrdiff_t left_idx = std::distance(order.begin(), lower) - 1;
  std::size_t right_idx = static_cast<std::size_t>(std::distance(order.begin(), lower));
  std::vector<std::size_t> nearest_indices;
  nearest_indices.reserve(max_token_num);
  while (nearest_indices.size() < max_token_num &&
    (left_idx >= 0 || right_idx < order.size()))
  {
    const bool has_left = left_idx >= 0;
    const bool has_right = right_idx < order.size();
    double left_distance = std::numeric_limits<double>::infinity();
    double right_distance = std::numeric_limits<double>::infinity();
    if (has_left) {
      const PlaneCluster &plane_cluster =
        index.tokens[order[static_cast<std::size_t>(left_idx)]].plane_cluster;
      left_distance = calculateLogRatioDistance(
        use_short_extent ? plane_cluster.short_extent : plane_cluster.long_extent, target_extent);
    }
    if (has_right) {
      const PlaneCluster &plane_cluster = index.tokens[order[right_idx]].plane_cluster;
      right_distance = calculateLogRatioDistance(
        use_short_extent ? plane_cluster.short_extent : plane_cluster.long_extent, target_extent);
    }
    if (left_distance <= right_distance) {
      nearest_indices.push_back(order[static_cast<std::size_t>(left_idx)]);
      --left_idx;
    } else {
      nearest_indices.push_back(order[right_idx]);
      ++right_idx;
    }
  }
  return nearest_indices;
}

std::vector<double> makeScaleProbes(const Options &options)
{
  std::vector<double> probes{1.0};
  if (options.scale_probe_ratio <= 1.0) {
    return probes;
  }
  for (double scale = options.scale_probe_ratio;
    scale <= options.max_size_expand_ratio + 1e-9;
    scale *= options.scale_probe_ratio)
  {
    probes.push_back(scale);
  }
  if (probes.back() < options.max_size_expand_ratio) {
    probes.push_back(options.max_size_expand_ratio);
  }
  return probes;
}

std::unordered_set<std::size_t> retrieveTokenIndices(
  const PlaneTokenIndex &index, const TemplateData &query, const Options &options)
{
  std::unordered_set<std::size_t> retrieved_indices;
  const std::vector<double> probes = makeScaleProbes(options);
  for (const PlaneCluster &query_cluster : query.plane_clusters) {
    for (const double scale : probes) {
      const std::vector<std::size_t> short_indices = findNearestTokenIndices(
        index, index.short_order, query_cluster.short_extent * scale, true,
        options.max_token_num_per_plane_probe);
      const std::vector<std::size_t> long_indices = findNearestTokenIndices(
        index, index.long_order, query_cluster.long_extent * scale, false,
        options.max_token_num_per_plane_probe);
      std::unordered_set<std::size_t> nearby_token_indices(
        short_indices.begin(), short_indices.end());
      nearby_token_indices.insert(long_indices.begin(), long_indices.end());
      for (const std::size_t token_idx : nearby_token_indices) {
        if (calculateSizeExcessRatio(query_cluster, index.tokens[token_idx].plane_cluster) <=
          options.max_size_excess_ratio)
        {
          retrieved_indices.insert(token_idx);
        }
      }
    }
  }
  return retrieved_indices;
}

void printUsage()
{
  std::cerr <<
    "使用法: plane_cluster_fuzzy_retrieval dataset_dir query_dataset "
    "[--max_size_excess_ratio 値] [--size_excess_dev 値] "
    "[--size_near_dev 値] [--normal_z_dev 値] "
    "[--scale_probe_ratio 値] [--max_size_expand_ratio 値] "
    "[--max_token_num_per_plane_probe 個数]" << std::endl;
}

}  // 無名namespace

int main(int argc, char **argv)
{
  if (argc < 3) {
    printUsage();
    return 1;
  }
  try {
    Options options;
    for (int index = 3; index < argc; index += 2) {
      if (index + 1 >= argc) {
        printUsage();
        return 1;
      }
      const std::string name(argv[index]);
      const std::string value(argv[index + 1]);
      double *destination = nullptr;
      if (name == "--max_size_excess_ratio") {
        destination = &options.max_size_excess_ratio;
      } else if (name == "--size_excess_dev") {
        destination = &options.size_excess_dev;
      } else if (name == "--size_near_dev") {
        destination = &options.size_near_dev;
      } else if (name == "--normal_z_dev") {
        destination = &options.normal_z_dev;
      } else if (name == "--scale_probe_ratio") {
        destination = &options.scale_probe_ratio;
      } else if (name == "--max_size_expand_ratio") {
        destination = &options.max_size_expand_ratio;
      } else if (name == "--max_token_num_per_plane_probe") {
        if (!parseSizeOption(value, &options.max_token_num_per_plane_probe)) {
          throw std::runtime_error("個数オプションが不正です: " + name);
        }
        continue;
      } else {
        printUsage();
        return 1;
      }
      if (!parseDoubleOption(name, value, destination)) {
        throw std::runtime_error("数値オプションが不正です: " + name);
      }
    }

    const std::filesystem::path dataset_dir(argv[1]);
    const auto load_start_time = std::chrono::steady_clock::now();
    const TemplateData query = loadTemplateData(argv[2]);
    if (query.plane_clusters.empty()) {
      throw std::runtime_error("クエリに有効な平面クラスタがありません。");
    }
    std::vector<TemplateData> templates;
    std::size_t skipped_template_num = 0U;
    for (const auto &template_path : findTemplatePaths(dataset_dir)) {
      TemplateData template_data = loadTemplateData(template_path);
      if (template_data.template_id != query.template_id) {
        if (template_data.plane_clusters.empty()) {
          ++skipped_template_num;
        } else {
          templates.push_back(std::move(template_data));
        }
      }
    }
    const auto index_start_time = std::chrono::steady_clock::now();
    const PlaneTokenIndex token_index = buildPlaneTokenIndex(templates);
    const auto retrieve_start_time = std::chrono::steady_clock::now();
    const std::unordered_set<std::size_t> retrieved_token_indices = retrieveTokenIndices(
      token_index, query, options);
    std::vector<bool> is_candidate_template(templates.size(), false);
    for (const std::size_t token_idx : retrieved_token_indices) {
      is_candidate_template[token_index.tokens[token_idx].template_idx] = true;
    }
    std::vector<Result> results;
    for (std::size_t template_idx = 0U; template_idx < templates.size(); ++template_idx) {
      if (!is_candidate_template[template_idx]) {
        continue;
      }
      const TemplateData &template_data = templates[template_idx];

      double sum_score = 0.0;
      double min_plane_score = 1.0;
      double max_size_excess_ratio = 0.0;
      bool has_hard_rejection = false;
      for (const PlaneCluster &query_cluster : query.plane_clusters) {
        double best_score = -std::numeric_limits<double>::infinity();
        double best_size_excess_ratio = 0.0;
        for (const PlaneCluster &template_cluster : template_data.plane_clusters) {
          const double size_excess_ratio = calculateSizeExcessRatio(query_cluster, template_cluster);
          if (size_excess_ratio > options.max_size_excess_ratio) {
            continue;
          }
          const double score = calculatePlaneScore(query_cluster, template_cluster, options);
          if (score > best_score) {
            best_score = score;
            best_size_excess_ratio = size_excess_ratio;
          }
        }
        if (!isFinite(best_score)) {
          has_hard_rejection = true;
          break;
        }
        sum_score += best_score;
        min_plane_score = std::min(min_plane_score, best_score);
        max_size_excess_ratio = std::max(max_size_excess_ratio, best_size_excess_ratio);
      }
      if (!has_hard_rejection) {
        results.push_back({
          template_data.template_id,
          sum_score / static_cast<double>(query.plane_clusters.size()),
          min_plane_score,
          max_size_excess_ratio});
      }
    }
    const auto score_finish_time = std::chrono::steady_clock::now();
    std::sort(results.begin(), results.end(), [](const Result &first, const Result &second) {
      return first.score == second.score ? first.template_id < second.template_id :
             first.score > second.score;
    });

    std::cout << "query=" << query.template_id << " plane_clusters=" <<
      query.plane_clusters.size() << " max_size_excess_ratio=" <<
      options.max_size_excess_ratio << " size_excess_dev=" << options.size_excess_dev <<
      " size_near_dev=" << options.size_near_dev << " normal_z_dev=" <<
      options.normal_z_dev << " indexed_plane_tokens=" << token_index.tokens.size() <<
      " retrieved_plane_tokens=" << retrieved_token_indices.size() <<
      " candidate_templates=" << results.size() << " skipped_templates=" <<
      skipped_template_num << std::endl;
    const auto load_ms = std::chrono::duration<double, std::milli>(
      index_start_time - load_start_time).count();
    const auto index_ms = std::chrono::duration<double, std::milli>(
      retrieve_start_time - index_start_time).count();
    const auto retrieve_and_score_ms = std::chrono::duration<double, std::milli>(
      score_finish_time - retrieve_start_time).count();
    std::cout << "load_ms=" << load_ms << " index_ms=" << index_ms <<
      " retrieve_and_score_ms=" << retrieve_and_score_ms << std::endl;
    std::cout << "survivors=" << results.size() << std::endl;
    std::cout << "rank template score min_plane_score max_size_excess_ratio" << std::endl;
    for (std::size_t index = 0U; index < results.size() && index < kTopResultNum; ++index) {
      const Result &result = results[index];
      std::cout << index + 1U << ' ' << result.template_id << ' ' << result.score << ' ' <<
        result.min_plane_score << ' ' << result.max_size_excess_ratio << std::endl;
    }
  } catch (const std::exception &error) {
    std::cerr << "評価失敗: " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
