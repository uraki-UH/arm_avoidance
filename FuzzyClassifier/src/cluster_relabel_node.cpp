#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
double safe_double(const double value)
{
  return std::isfinite(value) ? value : 0.0;
}

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double normalized_ratio(const double value, const double scale)
{
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0 || value <= 0.0) {
    return 0.0;
  }
  return value / (value + scale);
}

double cluster_speed_mps(const ais_gng_msgs::msg::TopologicalCluster & cluster)
{
  const double vx = safe_double(cluster.velocity.x);
  const double vy = safe_double(cluster.velocity.y);
  const double vz = safe_double(cluster.velocity.z);
  return std::sqrt(vx * vx + vy * vy + vz * vz);
}

double cluster_volume_m3(const ais_gng_msgs::msg::TopologicalCluster & cluster)
{
  return std::abs(
    safe_double(cluster.scale.x) *
    safe_double(cluster.scale.y) *
    safe_double(cluster.scale.z));
}

struct TrapezoidRange
{
  double lowest{0.0};
  double low{0.0};
  double high{0.0};
  double highest{0.0};
};

double trapezoid_membership(const TrapezoidRange & range, const double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (value <= range.lowest || value >= range.highest) {
    return 0.0;
  }
  if (value >= range.low && value <= range.high) {
    return 1.0;
  }
  if (value < range.low) {
    const double den = range.low - range.lowest;
    return den > 0.0 ? (value - range.lowest) / den : 0.0;
  }
  const double den = range.highest - range.high;
  return den > 0.0 ? (range.highest - value) / den : 0.0;
}

struct RelabelScoreConfig
{
  int min_nodes{10};
  bool preserve_structural_labels{true};

  double human_threshold{0.75};
  double human_bias{0.0};
  double human_speed_weight{0.7};
  double human_reliability_weight{0.8};
  double human_match_weight{0.2};
  double human_node_count_weight{0.01};
  double human_inv_volume_weight{0.5};

  double car_threshold{0.50};
  double car_threshold_for_small{0.50};
  double car_threshold_for_big{0.50};
  double car_bias{0.0};
  double car_speed_weight{0.6};
  double car_reliability_weight{0.8};
  double car_match_weight{0.2};
  double car_node_count_weight{0.01};
  double car_volume_weight{0.3};

  TrapezoidRange class_human_small_width{0.20, 0.35, 0.35, 0.50};
  TrapezoidRange class_human_big_width{0.35, 0.50, 0.50, 0.70};
  TrapezoidRange class_human_small_diff_tate_yoko{0.00, 0.20, 0.60, 0.70};
  TrapezoidRange class_human_big_diff_tate_yoko{0.00, 0.40, 0.60, 0.90};
  TrapezoidRange class_human_small_height{1.20, 1.40, 1.50, 1.60};
  TrapezoidRange class_human_big_height{1.50, 1.60, 1.80, 2.00};

  TrapezoidRange class_car_small_width{1.20, 1.30, 1.65, 1.75};
  TrapezoidRange class_car_big_width{1.65, 1.75, 2.50, 2.60};
  TrapezoidRange class_car_small_diff_tate_yoko{0.90, 1.10, 4.00, 5.00};
  TrapezoidRange class_car_big_diff_tate_yoko{4.00, 5.00, 11.00, 13.00};
  TrapezoidRange class_car_small_height{0.80, 1.00, 1.90, 2.10};
  TrapezoidRange class_car_big_height{1.90, 2.10, 3.80, 4.00};

  double class_human_cf_bbb{0.30};
  double class_human_cf_bbs{0.00};
  double class_human_cf_bsb{0.00};
  double class_human_cf_bss{0.00};
  double class_human_cf_sbb{0.00};
  double class_human_cf_sbs{0.00};
  double class_human_cf_ssb{1.00};
  double class_human_cf_sss{0.70};

  double class_car_cf_small_car_bbb{0.00};
  double class_car_cf_small_car_bbs{0.00};
  double class_car_cf_small_car_bsb{0.00};
  double class_car_cf_small_car_bss{0.00};
  double class_car_cf_small_car_sbb{0.40};
  double class_car_cf_small_car_sbs{0.40};
  double class_car_cf_small_car_ssb{0.80};
  double class_car_cf_small_car_sss{1.00};

  double class_car_cf_big_car_bbb{1.00};
  double class_car_cf_big_car_bbs{0.80};
  double class_car_cf_big_car_bsb{0.00};
  double class_car_cf_big_car_bss{0.00};
  double class_car_cf_big_car_sbb{0.60};
  double class_car_cf_big_car_sbs{0.60};
  double class_car_cf_big_car_ssb{0.00};
  double class_car_cf_big_car_sss{0.00};
};

uint8_t relabel_from_config(
  const ais_gng_msgs::msg::TopologicalCluster & cluster,
  const RelabelScoreConfig & config,
  double & human_score,
  double & car_score)
{
  if (config.preserve_structural_labels &&
    (cluster.label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN ||
    cluster.label == ais_gng_msgs::msg::TopologicalMap::WALL))
  {
    human_score = 0.0;
    car_score = 0.0;
    return cluster.label;
  }

  if (static_cast<int>(cluster.nodes.size()) < config.min_nodes) {
    human_score = 0.0;
    car_score = 0.0;
    return cluster.label;
  }

  const double speed = cluster_speed_mps(cluster);
  const double volume = cluster_volume_m3(cluster);
  const double reliability = clamp01(safe_double(cluster.label_reliability));
  const double match = clamp01(safe_double(cluster.match));
  const double node_count = static_cast<double>(cluster.nodes.size());
  const double speed_norm = normalized_ratio(speed, 1.0);
  const double node_count_norm = normalized_ratio(
    node_count,
    static_cast<double>(std::max(config.min_nodes, 1)));
  const double volume_norm = normalized_ratio(volume, 1.0);
  const double inv_volume_norm = 1.0 - volume_norm;

  const double sx = std::abs(safe_double(cluster.scale.x));
  const double sy = std::abs(safe_double(cluster.scale.y));
  const double sz = std::abs(safe_double(cluster.scale.z));
  const double width = std::min(sx, sy);
  const double diff_tate_yoko = std::abs(sx - sy);
  const double height = sz;

  const double human_shape_width_small = trapezoid_membership(config.class_human_small_width, width);
  const double human_shape_height_small = trapezoid_membership(config.class_human_small_height, height);
  const double human_shape_diff_small = trapezoid_membership(config.class_human_small_diff_tate_yoko, diff_tate_yoko);

  const double human_shape_width_big = trapezoid_membership(config.class_human_big_width, width);
  const double human_shape_height_big = trapezoid_membership(config.class_human_big_height, height);
  const double human_shape_diff_big = trapezoid_membership(config.class_human_big_diff_tate_yoko, diff_tate_yoko);

  const double car_shape_width_small = trapezoid_membership(config.class_car_small_width, width);
  const double car_shape_height_small = trapezoid_membership(config.class_car_small_height, height);
  const double car_shape_diff_small = trapezoid_membership(config.class_car_small_diff_tate_yoko, diff_tate_yoko);

  const double car_shape_width_big = trapezoid_membership(config.class_car_big_width, width);
  const double car_shape_height_big = trapezoid_membership(config.class_car_big_height, height);
  const double car_shape_diff_big = trapezoid_membership(config.class_car_big_diff_tate_yoko, diff_tate_yoko);


  human_score =
    std::max({
      human_shape_width_big * human_shape_diff_big * human_shape_height_big * config.class_human_cf_bbb,
      human_shape_width_big * human_shape_diff_big * human_shape_height_small * config.class_human_cf_bbs,
      human_shape_width_big * human_shape_diff_small * human_shape_height_big * config.class_human_cf_bsb,
      human_shape_width_big * human_shape_diff_small * human_shape_height_small * config.class_human_cf_bss,
      human_shape_width_small * human_shape_diff_big * human_shape_height_big * config.class_human_cf_sbb,
      human_shape_width_small * human_shape_diff_big * human_shape_height_small * config.class_human_cf_sbs,
      human_shape_width_small * human_shape_diff_small * human_shape_height_big * config.class_human_cf_ssb,
      human_shape_width_small * human_shape_diff_small * human_shape_height_small * config.class_human_cf_sss,
    });

  const double small_car_score =
    std::max({
      car_shape_width_big * car_shape_diff_big * car_shape_height_big * config.class_car_cf_small_car_bbb,
      car_shape_width_big * car_shape_diff_big * car_shape_height_small * config.class_car_cf_small_car_bbs,
      car_shape_width_big * car_shape_diff_small * car_shape_height_big * config.class_car_cf_small_car_bsb,
      car_shape_width_big * car_shape_diff_small * car_shape_height_small * config.class_car_cf_small_car_bss,
      car_shape_width_small * car_shape_diff_big * car_shape_height_big * config.class_car_cf_small_car_sbb,
      car_shape_width_small * car_shape_diff_big * car_shape_height_small * config.class_car_cf_small_car_sbs,
      car_shape_width_small * car_shape_diff_small * car_shape_height_big * config.class_car_cf_small_car_ssb,
      car_shape_width_small * car_shape_diff_small * car_shape_height_small * config.class_car_cf_small_car_sss,
    });

  const double big_car_score =
    std::max({
      car_shape_width_big * car_shape_diff_big * car_shape_height_big * config.class_car_cf_big_car_bbb,
      car_shape_width_big * car_shape_diff_big * car_shape_height_small * config.class_car_cf_big_car_bbs,
      car_shape_width_big * car_shape_diff_small * car_shape_height_big * config.class_car_cf_big_car_bsb,
      car_shape_width_big * car_shape_diff_small * car_shape_height_small * config.class_car_cf_big_car_bss,
      car_shape_width_small * car_shape_diff_big * car_shape_height_big * config.class_car_cf_big_car_sbb,
      car_shape_width_small * car_shape_diff_big * car_shape_height_small * config.class_car_cf_big_car_sbs,
      car_shape_width_small * car_shape_diff_small * car_shape_height_big * config.class_car_cf_big_car_ssb,
      car_shape_width_small * car_shape_diff_small * car_shape_height_small * config.class_car_cf_big_car_sss,
    });

//  const double human_weight_sum =
//    1.0 +
//    std::max(config.human_speed_weight, 0.0) +
//    std::max(config.human_reliability_weight, 0.0) +
//    std::max(config.human_match_weight, 0.0) +
//    std::max(config.human_node_count_weight, 0.0) +
//    std::max(config.human_inv_volume_weight, 0.0);
//  const double human_weighted_sum =
//    human_score +
//    std::max(config.human_speed_weight, 0.0) * speed_norm +
//    std::max(config.human_reliability_weight, 0.0) * reliability +
//    std::max(config.human_match_weight, 0.0) * match +
//    std::max(config.human_node_count_weight, 0.0) * node_count_norm +
//    std::max(config.human_inv_volume_weight, 0.0) * inv_volume_norm;
//  human_score = clamp01(config.human_bias + human_weighted_sum / human_weight_sum);
//
//  const double car_shape_score = std::max(small_car_score, big_car_score);
//  const double car_weight_sum =
//    1.0 +
//    std::max(config.car_speed_weight, 0.0) +
//    std::max(config.car_reliability_weight, 0.0) +
//    std::max(config.car_match_weight, 0.0) +
//    std::max(config.car_node_count_weight, 0.0) +
//    std::max(config.car_volume_weight, 0.0);
//  const double car_weighted_sum =
//    car_shape_score +
//    std::max(config.car_speed_weight, 0.0) * speed_norm +
//    std::max(config.car_reliability_weight, 0.0) * reliability +
//    std::max(config.car_match_weight, 0.0) * match +
//    std::max(config.car_node_count_weight, 0.0) * node_count_norm +
//    std::max(config.car_volume_weight, 0.0) * volume_norm;
//  car_score = clamp01(config.car_bias + car_weighted_sum / car_weight_sum);

  const bool is_human = human_score >= config.human_threshold;
  const bool is_small_car = small_car_score >= config.car_threshold_for_small;
  const bool is_big_car = big_car_score >= config.car_threshold_for_big;
  if (is_human) {
    return ais_gng_msgs::msg::TopologicalMap::HUMAN;
  }
  if (is_small_car || is_big_car) {
    return ais_gng_msgs::msg::TopologicalMap::CAR;
  }

  return ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
}
}  // namespace

class ClusterRelabelNode : public rclcpp::Node
{
public:
  ClusterRelabelNode()
  : Node("fuzzy_cluster_relabel")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map");
    output_topic_ =
      this->declare_parameter<std::string>("output_topic", "/fuzzy_classifier/topological_map");
    rewrite_label_ = this->declare_parameter<bool>("rewrite_label", true);
    rewrite_label_inferred_ = this->declare_parameter<bool>("rewrite_label_inferred", false);
    preserve_structural_labels_ =
      this->declare_parameter<bool>("preserve_structural_labels", true);
    score_min_nodes_ = static_cast<int>(this->declare_parameter<std::int64_t>("score.min_nodes", 10));
    set_reliability_from_score_ =
      this->declare_parameter<bool>("score.set_reliability_from_score", false);

    human_threshold_ = this->declare_parameter<double>("score.human.threshold", 0.20);
    human_bias_ = this->declare_parameter<double>("score.human.bias", 0.0);
    human_speed_weight_ = this->declare_parameter<double>("score.human.speed_weight", 0.7);
    human_reliability_weight_ =
      this->declare_parameter<double>("score.human.reliability_weight", 0.8);
    human_match_weight_ = this->declare_parameter<double>("score.human.match_weight", 0.2);
    human_node_count_weight_ =
      this->declare_parameter<double>("score.human.node_count_weight", 0.01);
    human_inv_volume_weight_ =
      this->declare_parameter<double>("score.human.inv_volume_weight", 0.5);

    car_threshold_ = this->declare_parameter<double>("score.car.threshold", 0.50);
    car_threshold_for_small_ =
      this->declare_parameter<double>("score.car.threshold_for_small", car_threshold_);
    car_threshold_for_big_ =
      this->declare_parameter<double>("score.car.threshold_for_big", car_threshold_);
    car_bias_ = this->declare_parameter<double>("score.car.bias", 0.0);
    car_speed_weight_ = this->declare_parameter<double>("score.car.speed_weight", 0.6);
    car_reliability_weight_ = this->declare_parameter<double>("score.car.reliability_weight", 0.8);
    car_match_weight_ = this->declare_parameter<double>("score.car.match_weight", 0.2);
    car_node_count_weight_ = this->declare_parameter<double>("score.car.node_count_weight", 0.01);
    car_volume_weight_ = this->declare_parameter<double>("score.car.volume_weight", 0.3);

    auto declare_range = [this](const std::string & key_prefix, TrapezoidRange & range) {
      range.lowest = this->declare_parameter<double>(key_prefix + ".lowest", range.lowest);
      range.low = this->declare_parameter<double>(key_prefix + ".low", range.low);
      range.high = this->declare_parameter<double>(key_prefix + ".high", range.high);
      range.highest = this->declare_parameter<double>(key_prefix + ".highest", range.highest);
    };

    declare_range("class.human.small.width", class_human_small_width_);
    declare_range("class.human.big.width", class_human_big_width_);
    declare_range("class.human.small.diff_tate_yoko", class_human_small_diff_tate_yoko_);
    declare_range("class.human.big.diff_tate_yoko", class_human_big_diff_tate_yoko_);
    declare_range("class.human.small.height", class_human_small_height_);
    declare_range("class.human.big.height", class_human_big_height_);
    declare_range("class.car.small.width", class_car_small_width_);
    declare_range("class.car.big.width", class_car_big_width_);
    declare_range("class.car.small.diff_tate_yoko", class_car_small_diff_tate_yoko_);
    declare_range("class.car.big.diff_tate_yoko", class_car_big_diff_tate_yoko_);
    declare_range("class.car.small.height", class_car_small_height_);
    declare_range("class.car.big.height", class_car_big_height_);

    auto declare_cf_rule = [this](
      const std::string & key_prefix,
      double & bbb,
      double & bbs,
      double & bsb,
      double & bss,
      double & sbb,
      double & sbs,
      double & ssb,
      double & sss)
    {
      bbb = this->declare_parameter<double>(key_prefix + ".bbb", bbb);
      bbs = this->declare_parameter<double>(key_prefix + ".bbs", bbs);
      bsb = this->declare_parameter<double>(key_prefix + ".bsb", bsb);
      bss = this->declare_parameter<double>(key_prefix + ".bss", bss);
      sbb = this->declare_parameter<double>(key_prefix + ".sbb", sbb);
      sbs = this->declare_parameter<double>(key_prefix + ".sbs", sbs);
      ssb = this->declare_parameter<double>(key_prefix + ".ssb", ssb);
      sss = this->declare_parameter<double>(key_prefix + ".sss", sss);
    };

    declare_cf_rule(
      "class.human.cf",
      class_human_cf_bbb_,
      class_human_cf_bbs_,
      class_human_cf_bsb_,
      class_human_cf_bss_,
      class_human_cf_sbb_,
      class_human_cf_sbs_,
      class_human_cf_ssb_,
      class_human_cf_sss_);
    declare_cf_rule(
      "class.car.cf.small_car",
      class_car_cf_small_car_bbb_,
      class_car_cf_small_car_bbs_,
      class_car_cf_small_car_bsb_,
      class_car_cf_small_car_bss_,
      class_car_cf_small_car_sbb_,
      class_car_cf_small_car_sbs_,
      class_car_cf_small_car_ssb_,
      class_car_cf_small_car_sss_);
    declare_cf_rule(
      "class.car.cf.big_car",
      class_car_cf_big_car_bbb_,
      class_car_cf_big_car_bbs_,
      class_car_cf_big_car_bsb_,
      class_car_cf_big_car_bss_,
      class_car_cf_big_car_sbb_,
      class_car_cf_big_car_sbs_,
      class_car_cf_big_car_ssb_,
      class_car_cf_big_car_sss_);

    sanitize_parameters();
    log_active_score_parameters();

    param_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ClusterRelabelNode::param_cb, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(output_topic_, 10);
    map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_,
      10,
      std::bind(&ClusterRelabelNode::topological_map_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Cluster relabel node started: subscribe=%s publish=%s min_nodes=%d",
      input_topic_.c_str(),
      output_topic_.c_str(),
      score_min_nodes_);
  }

private:
  void sanitize_parameters()
  {
    score_min_nodes_ = std::max(score_min_nodes_, 0);
    human_threshold_ = clamp01(human_threshold_);
    car_threshold_ = clamp01(car_threshold_);
    car_threshold_for_small_ = clamp01(car_threshold_for_small_);
    car_threshold_for_big_ = clamp01(car_threshold_for_big_);

    human_speed_weight_ = std::max(human_speed_weight_, 0.0);
    human_reliability_weight_ = std::max(human_reliability_weight_, 0.0);
    human_match_weight_ = std::max(human_match_weight_, 0.0);
    human_node_count_weight_ = std::max(human_node_count_weight_, 0.0);
    human_inv_volume_weight_ = std::max(human_inv_volume_weight_, 0.0);

    car_speed_weight_ = std::max(car_speed_weight_, 0.0);
    car_reliability_weight_ = std::max(car_reliability_weight_, 0.0);
    car_match_weight_ = std::max(car_match_weight_, 0.0);
    car_node_count_weight_ = std::max(car_node_count_weight_, 0.0);
    car_volume_weight_ = std::max(car_volume_weight_, 0.0);
  }

  RelabelScoreConfig make_score_config_from_parameters() const
  {
    RelabelScoreConfig config;
    config.min_nodes = score_min_nodes_;
    config.preserve_structural_labels = preserve_structural_labels_;

    // Example mapping from relabel.yaml:
    // score.human.speed_weight -> human_speed_weight_ -> config.human_speed_weight
    // score.car.volume_weight -> car_volume_weight_ -> config.car_volume_weight
    config.human_threshold = human_threshold_;
    config.human_bias = human_bias_;
    config.human_speed_weight = human_speed_weight_;
    config.human_reliability_weight = human_reliability_weight_;
    config.human_match_weight = human_match_weight_;
    config.human_node_count_weight = human_node_count_weight_;
    config.human_inv_volume_weight = human_inv_volume_weight_;
    config.car_threshold_for_small = car_threshold_for_small_;
    config.car_threshold_for_big = car_threshold_for_big_;
    config.car_threshold = std::min(config.car_threshold_for_small, config.car_threshold_for_big);
    config.car_bias = car_bias_;
    config.car_speed_weight = car_speed_weight_;
    config.car_reliability_weight = car_reliability_weight_;
    config.car_match_weight = car_match_weight_;
    config.car_node_count_weight = car_node_count_weight_;
    config.car_volume_weight = car_volume_weight_;

    config.class_human_small_width = class_human_small_width_;
    config.class_human_big_width = class_human_big_width_;
    config.class_human_small_diff_tate_yoko = class_human_small_diff_tate_yoko_;
    config.class_human_big_diff_tate_yoko = class_human_big_diff_tate_yoko_;
    config.class_human_small_height = class_human_small_height_;
    config.class_human_big_height = class_human_big_height_;
    config.class_car_small_width = class_car_small_width_;
    config.class_car_big_width = class_car_big_width_;
    config.class_car_small_diff_tate_yoko = class_car_small_diff_tate_yoko_;
    config.class_car_big_diff_tate_yoko = class_car_big_diff_tate_yoko_;
    config.class_car_small_height = class_car_small_height_;
    config.class_car_big_height = class_car_big_height_;

    config.class_human_cf_bbb = class_human_cf_bbb_;
    config.class_human_cf_bbs = class_human_cf_bbs_;
    config.class_human_cf_bsb = class_human_cf_bsb_;
    config.class_human_cf_bss = class_human_cf_bss_;
    config.class_human_cf_sbb = class_human_cf_sbb_;
    config.class_human_cf_sbs = class_human_cf_sbs_;
    config.class_human_cf_ssb = class_human_cf_ssb_;
    config.class_human_cf_sss = class_human_cf_sss_;

    config.class_car_cf_small_car_bbb = class_car_cf_small_car_bbb_;
    config.class_car_cf_small_car_bbs = class_car_cf_small_car_bbs_;
    config.class_car_cf_small_car_bsb = class_car_cf_small_car_bsb_;
    config.class_car_cf_small_car_bss = class_car_cf_small_car_bss_;
    config.class_car_cf_small_car_sbb = class_car_cf_small_car_sbb_;
    config.class_car_cf_small_car_sbs = class_car_cf_small_car_sbs_;
    config.class_car_cf_small_car_ssb = class_car_cf_small_car_ssb_;
    config.class_car_cf_small_car_sss = class_car_cf_small_car_sss_;

    config.class_car_cf_big_car_bbb = class_car_cf_big_car_bbb_;
    config.class_car_cf_big_car_bbs = class_car_cf_big_car_bbs_;
    config.class_car_cf_big_car_bsb = class_car_cf_big_car_bsb_;
    config.class_car_cf_big_car_bss = class_car_cf_big_car_bss_;
    config.class_car_cf_big_car_sbb = class_car_cf_big_car_sbb_;
    config.class_car_cf_big_car_sbs = class_car_cf_big_car_sbs_;
    config.class_car_cf_big_car_ssb = class_car_cf_big_car_ssb_;
    config.class_car_cf_big_car_sss = class_car_cf_big_car_sss_;
    return config;
  }

  void log_active_score_parameters() const
  {
    RCLCPP_INFO(
      this->get_logger(),
      "score params from YAML: human(th=%.3f,bias=%.3f,speed=%.3f,rel=%.3f,match=%.3f,node=%.3f,inv_vol=%.3f) "
      "car(th_small=%.3f,th_big=%.3f,th_eff=%.3f,bias=%.3f,speed=%.3f,rel=%.3f,match=%.3f,node=%.3f,vol=%.3f)",
      human_threshold_,
      human_bias_,
      human_speed_weight_,
      human_reliability_weight_,
      human_match_weight_,
      human_node_count_weight_,
      human_inv_volume_weight_,
      car_threshold_for_small_,
      car_threshold_for_big_,
      std::min(car_threshold_for_small_, car_threshold_for_big_),
      car_bias_,
      car_speed_weight_,
      car_reliability_weight_,
      car_match_weight_,
      car_node_count_weight_,
      car_volume_weight_);
    RCLCPP_INFO(
      this->get_logger(),
      "class params from YAML: human.small.width=[%.2f, %.2f, %.2f, %.2f] car.small.width=[%.2f, %.2f, %.2f, %.2f]",
      class_human_small_width_.lowest,
      class_human_small_width_.low,
      class_human_small_width_.high,
      class_human_small_width_.highest,
      class_car_small_width_.lowest,
      class_car_small_width_.low,
      class_car_small_width_.high,
      class_car_small_width_.highest);
    if (human_threshold_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "score.human.threshold is %.3f. Too small values can classify most clusters as HUMAN.",
        human_threshold_);
    }
    if (car_threshold_for_small_ <= 0.0 && car_threshold_for_big_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "score.car.threshold_for_small/big are both <= 0.0. CAR classification may become too permissive.");
    }
  }

  uint8_t relabel(
    const ais_gng_msgs::msg::TopologicalCluster & cluster,
    double & human_score,
    double & car_score) const
  {
    const RelabelScoreConfig config = make_score_config_from_parameters();
    return relabel_from_config(cluster, config, human_score, car_score);
  }

  void topological_map_cb(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
  {
    auto out = *msg;
    std::size_t changed = 0;
    std::size_t human_count = 0;
    std::size_t car_count = 0;
    std::size_t unknown_count = 0;
    double human_score_sum = 0.0;
    double car_score_sum = 0.0;
    if (rewrite_label_inferred_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Parameter rewrite_label_inferred is ignored: ais_gng_gpu TopologicalCluster has no label_inferred field.");
    }
    for (auto & cluster : out.clusters) {
      double human_score = 0.0;
      double car_score = 0.0;
      const auto new_label = relabel(cluster, human_score, car_score);
      human_score_sum += human_score;
      car_score_sum += car_score;
      if (new_label == ais_gng_msgs::msg::TopologicalMap::HUMAN) {
        ++human_count;
      } else if (new_label == ais_gng_msgs::msg::TopologicalMap::CAR) {
        ++car_count;
      } else if (new_label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT) {
        ++unknown_count;
      }

      if (rewrite_label_ && cluster.label != new_label) {
        cluster.label = new_label;
        ++changed;
      }
      if (set_reliability_from_score_) {
        cluster.label_reliability = static_cast<float>(
          std::clamp(std::max(human_score, car_score), 0.0, 1.0));
      }
    }

    map_pub_->publish(std::move(out));
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "relabel output: clusters=%zu changed=%zu human=%zu car=%zu unknown=%zu avg_human_score=%.3f avg_car_score=%.3f th_h=%.3f th_car_small=%.3f th_car_big=%.3f",
      msg->clusters.size(),
      changed,
      human_count,
      car_count,
      unknown_count,
      out.clusters.empty() ? 0.0 : human_score_sum / static_cast<double>(out.clusters.size()),
      out.clusters.empty() ? 0.0 : car_score_sum / static_cast<double>(out.clusters.size()),
      human_threshold_,
      car_threshold_for_small_,
      car_threshold_for_big_);
  }

  rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> & params)
  {
    auto update_range_param =
      [](const rclcpp::Parameter & param, const std::string & key_prefix, TrapezoidRange & range) {
        const std::string & key = param.get_name();
        if (key == key_prefix + ".lowest") {
          range.lowest = param.as_double();
          return true;
        }
        if (key == key_prefix + ".low") {
          range.low = param.as_double();
          return true;
        }
        if (key == key_prefix + ".high") {
          range.high = param.as_double();
          return true;
        }
        if (key == key_prefix + ".highest") {
          range.highest = param.as_double();
          return true;
        }
        return false;
      };

    auto update_cf_rule_param = [](
      const rclcpp::Parameter & param,
      const std::string & key_prefix,
      double & bbb,
      double & bbs,
      double & bsb,
      double & bss,
      double & sbb,
      double & sbs,
      double & ssb,
      double & sss)
    {
      const std::string & key = param.get_name();
      if (key == key_prefix + ".bbb") {
        bbb = param.as_double();
        return true;
      }
      if (key == key_prefix + ".bbs") {
        bbs = param.as_double();
        return true;
      }
      if (key == key_prefix + ".bsb") {
        bsb = param.as_double();
        return true;
      }
      if (key == key_prefix + ".bss") {
        bss = param.as_double();
        return true;
      }
      if (key == key_prefix + ".sbb") {
        sbb = param.as_double();
        return true;
      }
      if (key == key_prefix + ".sbs") {
        sbs = param.as_double();
        return true;
      }
      if (key == key_prefix + ".ssb") {
        ssb = param.as_double();
        return true;
      }
      if (key == key_prefix + ".sss") {
        sss = param.as_double();
        return true;
      }
      return false;
    };

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "input_topic" || name == "output_topic") {
        continue;
      } else if (name == "rewrite_label") {
        rewrite_label_ = p.as_bool();
      } else if (name == "rewrite_label_inferred") {
        rewrite_label_inferred_ = p.as_bool();
      } else if (name == "preserve_structural_labels") {
        preserve_structural_labels_ = p.as_bool();
      } else if (name == "score.min_nodes") {
        score_min_nodes_ = static_cast<int>(p.as_int());
      } else if (name == "score.set_reliability_from_score") {
        set_reliability_from_score_ = p.as_bool();
      } else if (name == "score.human.threshold") {
        human_threshold_ = p.as_double();
      } else if (name == "score.human.bias") {
        human_bias_ = p.as_double();
      } else if (name == "score.human.speed_weight") {
        human_speed_weight_ = p.as_double();
      } else if (name == "score.human.reliability_weight") {
        human_reliability_weight_ = p.as_double();
      } else if (name == "score.human.match_weight") {
        human_match_weight_ = p.as_double();
      } else if (name == "score.human.node_count_weight") {
        human_node_count_weight_ = p.as_double();
      } else if (name == "score.human.inv_volume_weight") {
        human_inv_volume_weight_ = p.as_double();
      } else if (name == "score.car.threshold") {
        car_threshold_ = p.as_double();
        car_threshold_for_small_ = car_threshold_;
        car_threshold_for_big_ = car_threshold_;
      } else if (name == "score.car.threshold_for_small") {
        car_threshold_for_small_ = p.as_double();
      } else if (name == "score.car.threshold_for_big") {
        car_threshold_for_big_ = p.as_double();
      } else if (name == "score.car.bias") {
        car_bias_ = p.as_double();
      } else if (name == "score.car.speed_weight") {
        car_speed_weight_ = p.as_double();
      } else if (name == "score.car.reliability_weight") {
        car_reliability_weight_ = p.as_double();
      } else if (name == "score.car.match_weight") {
        car_match_weight_ = p.as_double();
      } else if (name == "score.car.node_count_weight") {
        car_node_count_weight_ = p.as_double();
      } else if (name == "score.car.volume_weight") {
        car_volume_weight_ = p.as_double();
      } else if (
        update_range_param(p, "class.human.small.width", class_human_small_width_) ||
        update_range_param(p, "class.human.big.width", class_human_big_width_) ||
        update_range_param(p, "class.human.small.diff_tate_yoko", class_human_small_diff_tate_yoko_) ||
        update_range_param(p, "class.human.big.diff_tate_yoko", class_human_big_diff_tate_yoko_) ||
        update_range_param(p, "class.human.small.height", class_human_small_height_) ||
        update_range_param(p, "class.human.big.height", class_human_big_height_) ||
        update_range_param(p, "class.car.small.width", class_car_small_width_) ||
        update_range_param(p, "class.car.big.width", class_car_big_width_) ||
        update_range_param(p, "class.car.small.diff_tate_yoko", class_car_small_diff_tate_yoko_) ||
        update_range_param(p, "class.car.big.diff_tate_yoko", class_car_big_diff_tate_yoko_) ||
        update_range_param(p, "class.car.small.height", class_car_small_height_) ||
        update_range_param(p, "class.car.big.height", class_car_big_height_) ||
        update_cf_rule_param(
          p,
          "class.human.cf",
          class_human_cf_bbb_,
          class_human_cf_bbs_,
          class_human_cf_bsb_,
          class_human_cf_bss_,
          class_human_cf_sbb_,
          class_human_cf_sbs_,
          class_human_cf_ssb_,
          class_human_cf_sss_) ||
        update_cf_rule_param(
          p,
          "class.car.cf.small_car",
          class_car_cf_small_car_bbb_,
          class_car_cf_small_car_bbs_,
          class_car_cf_small_car_bsb_,
          class_car_cf_small_car_bss_,
          class_car_cf_small_car_sbb_,
          class_car_cf_small_car_sbs_,
          class_car_cf_small_car_ssb_,
          class_car_cf_small_car_sss_) ||
        update_cf_rule_param(
          p,
          "class.car.cf.big_car",
          class_car_cf_big_car_bbb_,
          class_car_cf_big_car_bbs_,
          class_car_cf_big_car_bsb_,
          class_car_cf_big_car_bss_,
          class_car_cf_big_car_sbb_,
          class_car_cf_big_car_sbs_,
          class_car_cf_big_car_ssb_,
          class_car_cf_big_car_sss_))
      {
        continue;
      }
    }

    sanitize_parameters();
    log_active_score_parameters();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  std::string input_topic_;
  std::string output_topic_;
  bool rewrite_label_{true};
  bool rewrite_label_inferred_{false};
  bool preserve_structural_labels_{true};
  int score_min_nodes_{10};
  bool set_reliability_from_score_{false};

  double human_threshold_{0.75};
  double human_bias_{0.0};
  double human_speed_weight_{0.7};
  double human_reliability_weight_{0.8};
  double human_match_weight_{0.2};
  double human_node_count_weight_{0.01};
  double human_inv_volume_weight_{0.5};

  double car_threshold_{0.50};
  double car_threshold_for_small_{0.50};
  double car_threshold_for_big_{0.50};
  double car_bias_{0.0};
  double car_speed_weight_{0.6};
  double car_reliability_weight_{0.8};
  double car_match_weight_{0.2};
  double car_node_count_weight_{0.01};
  double car_volume_weight_{0.3};

  TrapezoidRange class_human_small_width_{0.30, 0.35, 0.35, 0.50};
  TrapezoidRange class_human_big_width_{0.35, 0.50, 0.50, 0.70};
  TrapezoidRange class_human_small_diff_tate_yoko_{0.10, 0.20, 0.20, 0.30};
  TrapezoidRange class_human_big_diff_tate_yoko_{0.20, 0.40, 0.40, 0.60};
  TrapezoidRange class_human_small_height_{1.20, 1.40, 1.50, 1.60};
  TrapezoidRange class_human_big_height_{1.50, 1.60, 1.80, 2.00};

  TrapezoidRange class_car_small_width_{1.20, 1.30, 1.65, 1.75};
  TrapezoidRange class_car_big_width_{1.65, 1.75, 2.50, 2.60};
  TrapezoidRange class_car_small_diff_tate_yoko_{0.90, 1.10, 4.00, 5.00};
  TrapezoidRange class_car_big_diff_tate_yoko_{4.00, 5.00, 11.00, 13.00};
  TrapezoidRange class_car_small_height_{0.80, 1.00, 1.90, 2.10};
  TrapezoidRange class_car_big_height_{1.90, 2.10, 3.80, 4.00};

  double class_human_cf_bbb_{0.30};
  double class_human_cf_bbs_{0.00};
  double class_human_cf_bsb_{0.00};
  double class_human_cf_bss_{0.00};
  double class_human_cf_sbb_{0.00};
  double class_human_cf_sbs_{0.00};
  double class_human_cf_ssb_{1.00};
  double class_human_cf_sss_{0.70};

  double class_car_cf_small_car_bbb_{0.00};
  double class_car_cf_small_car_bbs_{0.00};
  double class_car_cf_small_car_bsb_{0.00};
  double class_car_cf_small_car_bss_{0.00};
  double class_car_cf_small_car_sbb_{0.40};
  double class_car_cf_small_car_sbs_{0.40};
  double class_car_cf_small_car_ssb_{0.80};
  double class_car_cf_small_car_sss_{1.00};

  double class_car_cf_big_car_bbb_{1.00};
  double class_car_cf_big_car_bbs_{0.80};
  double class_car_cf_big_car_bsb_{0.00};
  double class_car_cf_big_car_bss_{0.00};
  double class_car_cf_big_car_sbb_{0.60};
  double class_car_cf_big_car_sbs_{0.60};
  double class_car_cf_big_car_ssb_{0.00};
  double class_car_cf_big_car_sss_{0.00};

  OnSetParametersCallbackHandle::SharedPtr param_handle_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_pub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr map_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusterRelabelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
