#include <ais_gng/self_point_filter.hpp>
#include <gtest/gtest.h>

using namespace fuzzrobo;
namespace sf = fuzzrobo::self_point_filter;

namespace {
voxel_msgs::msg::Voxel mask_message() {
  voxel_msgs::msg::Voxel mask;
  mask.header.frame_id = "robot"; mask.header.stamp.sec = 1;
  mask.voxel_size = .25F; mask.x_shift = 42; mask.y_shift = 21; mask.z_shift = 0; mask.offset = 1000000;
  voxel_idx::VoxelIndexingSchema schema{42, 21, 0, 1000000, .25};
  mask.data = {int64_t(schema.pack({0, 0, 0})), int64_t(schema.pack({-1, 0, 0}))};
  return mask;
}

sensor_msgs::msg::PointCloud2 cloud_message() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "sensor"; cloud.header.stamp.sec = 1;
  cloud.width = 32; cloud.height = 16; cloud.point_step = 16; cloud.row_step = cloud.width * 16 + 12;
  cloud.data.resize(cloud.row_step * cloud.height, 0xab);
  for (uint32_t axis = 0; axis < 3; ++axis) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, "xyz"[axis]); field.offset = axis * 4;
    field.datatype = field.FLOAT32; field.count = 1; cloud.fields.push_back(field);
  }
  for (uint32_t idx = 0; idx < cloud.width * cloud.height; ++idx) {
    const float xyz[] = {idx % 2 ? 1.0F : .1F, .1F, .1F};
    std::memcpy(cloud.data.data() + (idx / cloud.width) * cloud.row_step + (idx % cloud.width) * 16, xyz, sizeof(xyz));
  }
  return cloud;
}
}

TEST(SelfPointFilter, CellBoundsOriginAndNegativeCoordinates) {
  auto message = mask_message();
  sf::mask_snapshot mask(message);
  EXPECT_TRUE(mask.contains({0, 0, 0}));
  EXPECT_TRUE(mask.contains({-.25, .1, .1}));
  EXPECT_FALSE(mask.contains({-.250001, .1, .1}));
  EXPECT_FALSE(mask.contains({.25, .1, .1}));
  EXPECT_FALSE(mask.contains({.1, .25, .1}));
  EXPECT_FALSE(mask.contains({1e100, .1, .1}));
  EXPECT_FALSE(mask.contains({NAN, .1, .1}));
  message.origin_x = 2; sf::mask_snapshot shifted(message);
  EXPECT_TRUE(shifted.contains({2.1, .1, .1}));
  EXPECT_FALSE(shifted.contains({.1, .1, .1}));
}

TEST(SelfPointFilter, shared_membership_matches_all_sampling_modes_and_labels) {
  const auto cloud = cloud_message();
  voxel_idx::roi_point_membership membership;
  membership.cells = {{1, true, true}};
  membership.point_cells.resize(cloud.width * cloud.height, voxel_idx::roi_point_membership::no_cell);
  for (uint32_t idx = 0; idx < membership.point_cells.size(); idx += 2)
    membership.point_cells[idx] = 1 | voxel_idx::roi_point_membership::self_flag;
  const sf::mask_snapshot mask(mask_message());
  for (auto mode : {PointSamplingMode::Head, PointSamplingMode::Uniform,
      PointSamplingMode::Random, PointSamplingMode::Stratified}) {
    std::vector<uint8_t> labels, reference_labels;
    const auto reference = sf::select_points(cloud, 71, 42, mode, mask,
      Eigen::Isometry3d::Identity(), &reference_labels);
    const auto selected = sf::select_shared_points(cloud, 71, 42, mode, membership, &labels);
    EXPECT_EQ(reference, selected);
    EXPECT_EQ(labels, reference_labels);
    EXPECT_EQ(selected, sf::select_shared_points(cloud, 71, 42, mode, membership));
  }
  membership.point_cells.pop_back();
  EXPECT_THROW(sf::select_shared_points(cloud, 71, 42, PointSamplingMode::Random, membership), std::invalid_argument);
}

TEST(SelfPointFilter, SparseMaskNotBoundingBoxRemoval) {
  auto message = mask_message();
  voxel_idx::VoxelIndexingSchema schema{42, 21, 0, 1000000, .25};
  message.data.push_back(schema.pack({4, 0, 0}));
  sf::mask_snapshot mask(message);
  EXPECT_FALSE(mask.contains({.6, .1, .1}));
  EXPECT_TRUE(mask.contains({1.1, .1, .1}));
}

TEST(SelfPointFilter, cell_lookup_matches_floor_at_each_boundary) {
  auto message = mask_message(); message.voxel_size = .02F;
  message.origin_x = .31; message.origin_y = -.27; message.origin_z = .13;
  message.data.clear();
  voxel_idx::VoxelIndexingSchema schema{42, 21, 0, 1000000, message.voxel_size};
  for (int x = -5; x <= 5; ++x) for (int y = -5; y <= 5; ++y) for (int z = -5; z <= 5; ++z)
    if ((x + y + z) % 2 == 0) message.data.push_back(schema.pack({x, y, z}));
  const std::unordered_set<int64_t> ids(message.data.begin(), message.data.end());
  const sf::mask_snapshot mask(message);
  const auto reference = [&](const Eigen::Vector3d &point) {
    const Eigen::Array3d cell = ((point - mask.origin) / schema.voxel_size).array().floor();
    return ids.count(schema.pack({int(cell.x()), int(cell.y()), int(cell.z())})) != 0;
  };
  for (int axis = 0; axis < 3; ++axis) for (int idx = -6; idx <= 6; ++idx) {
    Eigen::Vector3d point = mask.origin + Eigen::Vector3d::Constant(schema.voxel_size * .5);
    const double boundary = mask.origin[axis] + idx * schema.voxel_size;
    for (double value : {std::nextafter(boundary, -INFINITY), boundary, std::nextafter(boundary, INFINITY)}) {
      point[axis] = value;
      EXPECT_EQ(mask.contains(point), reference(point));
    }
  }
  std::mt19937 random(42); std::uniform_real_distribution<double> unit(-.2, .2);
  for (uint32_t idx = 0; idx < 4096; ++idx) {
    const Eigen::Vector3d point = mask.origin + Eigen::Vector3d(unit(random), unit(random), unit(random));
    EXPECT_EQ(mask.contains(point), reference(point));
  }
  EXPECT_FALSE(mask.contains({INFINITY, .1, .1}));
  EXPECT_FALSE(mask.contains({-INFINITY, .1, .1}));
}

TEST(SelfPointFilter, EverySamplingModeExcludesSelfBeforeAllocation) {
  const sf::mask_snapshot mask(mask_message());
  const auto cloud = cloud_message();
  for (auto mode : {PointSamplingMode::Head, PointSamplingMode::Uniform, PointSamplingMode::Random, PointSamplingMode::Stratified}) {
    std::vector<uint8_t> labels;
    const auto selected = sf::select_points(cloud, 100, 42, mode, mask, Eigen::Isometry3d::Identity(), &labels);
    ASSERT_EQ(selected.size(), 100U);
    ASSERT_EQ(labels.size(), 512U);
    for (const auto idx : selected) EXPECT_EQ(idx % 2, 1U);
    for (uint32_t idx = 0; idx < labels.size(); ++idx) EXPECT_EQ(labels[idx], idx % 2 ? 0 : 1);
  }
}

TEST(SelfPointFilter, TransformAndAllSelfEmptyInput) {
  const auto cloud = cloud_message();
  const sf::mask_snapshot mask(mask_message());
  auto transform = Eigen::Isometry3d::Identity(); transform.translation().x() = -1;
  const auto selected = sf::select_points(cloud, 0, 3, PointSamplingMode::Random, mask, transform);
  ASSERT_EQ(selected.size(), 256U);
  for (const auto idx : selected) EXPECT_EQ(idx % 2, 0U);
  transform.linear() = Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(1.1, .2, 0);
  const auto rotated = sf::select_points(cloud, 0, 3, PointSamplingMode::Stratified, mask, transform);
  ASSERT_EQ(rotated.size(), 256U);
  for (const auto idx : rotated) EXPECT_EQ(idx % 2, 0U);
  auto all_self = cloud; all_self.width = 1; all_self.height = 1;
  EXPECT_TRUE(sf::select_points(all_self, 100, 1, PointSamplingMode::Head, mask, Eigen::Isometry3d::Identity()).empty());
  auto empty = cloud; empty.width = 0; empty.height = 0; empty.data.clear();
  EXPECT_TRUE(sf::select_points(empty, 100, 1, PointSamplingMode::Head, mask, transform).empty());
}

TEST(SelfPointFilter, InvalidPointsLabelsAndFieldPreservation) {
  auto cloud = cloud_message();
  const float invalid = NAN; std::memcpy(cloud.data.data() + 16, &invalid, 4);
  const sf::mask_snapshot mask(mask_message());
  std::vector<uint8_t> labels;
  const auto selected = sf::select_points(cloud, 0, 2, PointSamplingMode::Stratified, mask, Eigen::Isometry3d::Identity(), &labels);
  EXPECT_EQ(selected.size(), 255U); EXPECT_EQ(labels[1], 2);
  const auto output = sf::make_labelled_cloud(cloud, labels);
  EXPECT_EQ(output.width, cloud.width); EXPECT_EQ(output.height, cloud.height);
  EXPECT_EQ(output.header, cloud.header); EXPECT_EQ(output.point_step, 17U);
  EXPECT_EQ(output.fields.back().name, "self_candidate");
  for (uint32_t idx = 0; idx < labels.size(); ++idx) {
    const auto *original = cloud.data.data() + (idx / cloud.width) * cloud.row_step + (idx % cloud.width) * 16;
    EXPECT_EQ(std::memcmp(original, output.data.data() + idx * 17, 16), 0);
    EXPECT_EQ(output.data[idx * 17 + 16], labels[idx]);
  }
  EXPECT_EQ(sf::make_labelled_cloud(output, labels).data, output.data);
}

TEST(SelfPointFilter, sensor_bounds_keep_exact_labels_after_rotation) {
  auto cloud = cloud_message();
  const sf::mask_snapshot mask(mask_message());
  for (uint32_t iter = 0; iter < 16; ++iter) {
    auto transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::AngleAxisd(iter * .31, Eigen::Vector3d(.3, .6, .1).normalized()).toRotationMatrix();
    transform.translation() = Eigen::Vector3d(2.3, -4.1, .7);
    const auto inverse = transform.inverse();
    for (uint32_t idx = 0; idx < 512; ++idx) {
      const Eigen::Vector3d point(.025 * (int(idx % 16) - 8), .025 * (int(idx / 16 % 4) - 1),
          .025 * (int(idx / 64) - 1));
      const Eigen::Vector3f xyz = (inverse * point).cast<float>();
      std::memcpy(cloud.data.data() + (idx / 32) * cloud.row_step + (idx % 32) * 16, xyz.data(), 12);
    }
    // 非剛体の異常入力では粗判定なしの直接照合への退避
    if (iter == 15) transform.linear() *= 1.01;
    std::vector<uint8_t> labels;
    sf::select_points(cloud, 20, 42, PointSamplingMode::Random, mask, transform, &labels);
    for (uint32_t idx = 0; idx < 512; ++idx) {
      Eigen::Vector3f xyz;
      std::memcpy(xyz.data(), cloud.data.data() + (idx / 32) * cloud.row_step + (idx % 32) * 16, 12);
      EXPECT_EQ(labels[idx], mask.contains(transform * xyz.cast<double>()) ? 1 : 0);
    }
  }
}

TEST(SelfPointFilter, EndiannessAndMalformedCloud) {
  auto cloud = cloud_message(); const sf::mask_snapshot mask(mask_message());
  const auto original = sf::select_points(cloud, 20, 9, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity());
  for (uint32_t idx = 0; idx < 512; ++idx) for (uint32_t axis = 0; axis < 3; ++axis) {
    auto *value = cloud.data.data() + (idx / 32) * cloud.row_step + (idx % 32) * 16 + axis * 4;
    std::reverse(value, value + 4);
  }
  cloud.is_bigendian = true;
  EXPECT_EQ(sf::select_points(cloud, 20, 9, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity()), original);
  cloud.data.resize(10);
  EXPECT_THROW(sf::select_points(cloud, 20, 9, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity()), std::invalid_argument);
}

TEST(SelfPointFilter, EmptyMaskMatchesLegacyRandomAndStratified) {
  auto message = mask_message(); message.data.clear(); const sf::mask_snapshot mask(message);
  const auto cloud = cloud_message();
  EXPECT_EQ(sf::select_points(cloud, 100, 42, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity()), pointcloud_sampling::select_random(cloud, 100, 42));
  EXPECT_EQ(sf::select_points(cloud, 100, 42, PointSamplingMode::Stratified, mask, Eigen::Isometry3d::Identity()), pointcloud_sampling::select_stratified(cloud, 100, 42));
}

TEST(SelfPointFilter, InvalidMaskRejected) {
  auto message = mask_message(); message.x_shift = 64; EXPECT_THROW(sf::mask_snapshot{message}, std::invalid_argument);
  message = mask_message(); message.voxel_size = NAN; EXPECT_THROW(sf::mask_snapshot{message}, std::invalid_argument);
  message = mask_message(); message.header.stamp.sec = 0; EXPECT_THROW(sf::mask_snapshot{message}, std::invalid_argument);
  message = mask_message(); message.origin_z = INFINITY; EXPECT_THROW(sf::mask_snapshot{message}, std::invalid_argument);
}

TEST(SelfPointFilter, BoundedMaskMemoryAndSparseFallback) {
  auto message = mask_message();
  const sf::mask_snapshot compact(message);
  EXPECT_FALSE(compact.bits.empty()); EXPECT_TRUE(compact.ids.empty());
  EXPECT_EQ(compact.num_cells, 2U);
  voxel_idx::VoxelIndexingSchema schema{42, 21, 0, 1000000, .25};
  message.data.push_back(schema.pack({10000, 10000, 10000}));
  const sf::mask_snapshot sparse(message);
  EXPECT_TRUE(sparse.bits.empty()); EXPECT_EQ(sparse.ids.size(), 3U);
  EXPECT_TRUE(sparse.contains({.1, .1, .1}));
  EXPECT_TRUE(sparse.contains({2500.1, 2500.1, 2500.1}));
  EXPECT_FALSE(sparse.contains({1250.1, 1250.1, 1250.1}));
}

TEST(SelfPointFilter, random_candidate_checks_stop_at_quota_and_never_repeat) {
  const auto cloud = cloud_message();
  std::vector<uint32_t> visits(512, 0);
  uint32_t num_checks = 0;
  const auto accept_all = [&](uint32_t idx, const std::array<double, 3> &) {
    ++visits[idx]; ++num_checks; return true;
  };
  const auto selected = pointcloud_sampling::select_random_if(cloud, 20, 42, accept_all);
  EXPECT_EQ(selected.size(), 20U); EXPECT_EQ(num_checks, 20U);
  for (const auto count : visits) EXPECT_LE(count, 1U);
  visits.assign(512, 0); num_checks = 0;
  const auto full = pointcloud_sampling::select_random_if(cloud, 20, 42, accept_all, true);
  EXPECT_EQ(selected, full); EXPECT_EQ(num_checks, 512U);
  for (const auto count : visits) EXPECT_EQ(count, 1U);

  num_checks = 0;
  const auto rejected = pointcloud_sampling::select_random_if(cloud, 20, 42,
      [&](uint32_t, const std::array<double, 3> &) { ++num_checks; return false; });
  EXPECT_TRUE(rejected.empty()); EXPECT_EQ(num_checks, 512U);
}

TEST(SelfPointFilter, random_labels_do_not_change_selection_or_quota) {
  auto cloud = cloud_message();
  const float invalid = NAN;
  std::memcpy(cloud.data.data() + 16, &invalid, 4);
  const sf::mask_snapshot mask(mask_message());
  for (uint32_t seed = 0; seed < 8; ++seed) for (uint32_t cap : {0U, 1U, 100U, 255U, 300U, 1000U}) {
    std::vector<uint8_t> labels;
    const auto partial = sf::select_points(cloud, cap, seed, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity());
    const auto full = sf::select_points(cloud, cap, seed, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity(), &labels);
    EXPECT_EQ(partial, full);
    EXPECT_EQ(partial.size(), cap ? std::min(cap, 255U) : 255U);
    std::unordered_set<uint32_t> unique(partial.begin(), partial.end());
    EXPECT_EQ(unique.size(), partial.size());
    for (auto idx : partial) { EXPECT_EQ(idx % 2, 1U); EXPECT_NE(idx, 1U); }
    for (uint32_t idx = 0; idx < labels.size(); ++idx)
      EXPECT_EQ(labels[idx], idx == 1 ? 2 : (idx % 2 ? 0 : 1));
  }
}

TEST(SelfPointFilter, random_sampling_has_no_fixed_region_bias) {
  auto cloud = cloud_message(); cloud.width = 16; cloud.height = 1;
  const sf::mask_snapshot mask(mask_message());
  std::array<uint32_t, 16> counts{};
  for (uint32_t seed = 0; seed < 4096; ++seed) {
    const auto selected = sf::select_points(cloud, 3, seed, PointSamplingMode::Random, mask, Eigen::Isometry3d::Identity());
    ASSERT_EQ(selected.size(), 3U);
    for (const auto idx : selected) ++counts[idx];
  }
  // 8有効点への各1536回の期待値。分布の極端な偏りの検出用
  for (uint32_t idx = 0; idx < 16; ++idx) {
    if (idx % 2) EXPECT_NEAR(counts[idx], 1536, 200);
    else EXPECT_EQ(counts[idx], 0U);
  }
}

TEST(SelfPointFilter, random_high_rejection_switch_keeps_order_and_single_evaluation) {
  auto cloud = cloud_message(); cloud.width = 4096; cloud.height = 1;
  cloud.row_step = cloud.width * cloud.point_step; cloud.data.resize(cloud.row_step);
  for (uint32_t idx = 0; idx < cloud.width; ++idx) {
    const float xyz[] = {float(idx), .1F, .1F};
    std::memcpy(cloud.data.data() + idx * cloud.point_step, xyz, sizeof(xyz));
  }
  for (uint32_t cap : {80U, 300U}) {
    std::vector<uint32_t> visits(cloud.width, 0);
    const auto predicate = [&](uint32_t idx, const std::array<double, 3> &) {
      ++visits[idx]; return idx % 32 == 31;
    };
    const auto partial = pointcloud_sampling::select_random_if(cloud, cap, 42, predicate);
    EXPECT_EQ(partial.size(), std::min(cap, 128U));
    for (const auto count : visits) EXPECT_EQ(count, 1U);
    visits.assign(cloud.width, 0);
    const auto full = pointcloud_sampling::select_random_if(cloud, cap, 42, predicate, true);
    EXPECT_EQ(partial, full);
    for (const auto count : visits) EXPECT_EQ(count, 1U);
    std::unordered_set<uint32_t> unique(partial.begin(), partial.end());
    EXPECT_EQ(unique.size(), partial.size());
  }
}

TEST(SelfPointFilter, mixed_xyz_types_unaligned_offsets_and_byte_order) {
  for (const bool is_bigendian : {false, true}) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 4; cloud.height = 2; cloud.point_step = 23; cloud.row_step = 97;
    cloud.is_bigendian = is_bigendian; cloud.data.resize(194, 0xab);
    const std::array<uint32_t, 3> offsets{1, 9, 13}, sizes{8, 4, 8};
    const uint16_t endian_probe = 1;
    const bool is_native_big_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
    for (uint32_t axis = 0; axis < 3; ++axis) {
      sensor_msgs::msg::PointField field;
      field.name = std::string(1, "xyz"[axis]); field.offset = offsets[axis];
      field.datatype = sizes[axis] == 4 ? field.FLOAT32 : field.FLOAT64; field.count = 1;
      cloud.fields.push_back(field);
    }
    for (uint32_t idx = 0; idx < 8; ++idx) for (uint32_t axis = 0; axis < 3; ++axis) {
      auto *bytes = cloud.data.data() + (idx / 4) * cloud.row_step + (idx % 4) * 23 + offsets[axis];
      const double value = idx == 1 && axis == 2 ? NAN : idx + axis * .125;
      const float short_value = value;
      if (sizes[axis] == 4) std::memcpy(bytes, &short_value, 4);
      else std::memcpy(bytes, &value, 8);
      if (is_bigendian != is_native_big_endian) std::reverse(bytes, bytes + sizes[axis]);
    }
    uint32_t num_checks = 0;
    const auto predicate = [&](uint32_t idx, const std::array<double, 3> &xyz) {
      ++num_checks;
      for (uint32_t axis = 0; axis < 3; ++axis) EXPECT_DOUBLE_EQ(xyz[axis], idx + axis * .125);
      return true;
    };
    auto selected = pointcloud_sampling::select_random_if(cloud, 0, 42, predicate);
    EXPECT_EQ(num_checks, 7U);
    std::sort(selected.begin(), selected.end());
    const std::vector<uint32_t> expected{0, 2, 3, 4, 5, 6, 7};
    EXPECT_EQ(selected, expected);
    EXPECT_EQ(pointcloud_sampling::select_stratified_if(cloud, 0, 42, predicate), expected);
  }
}

TEST(SelfPointFilter, random_layout_validation_before_partial_read) {
  auto cloud = cloud_message(); cloud.row_step = 1;
  EXPECT_THROW(pointcloud_sampling::select_random(cloud, 1, 42), std::invalid_argument);
  cloud = cloud_message(); cloud.fields[2].offset = 16;
  EXPECT_THROW(pointcloud_sampling::select_random(cloud, 1, 42), std::invalid_argument);
  cloud = cloud_message(); cloud.fields.pop_back();
  EXPECT_THROW(pointcloud_sampling::select_random(cloud, 1, 42), std::invalid_argument);
  cloud = cloud_message(); cloud.width = 0; cloud.height = 0; cloud.data.clear();
  EXPECT_TRUE(pointcloud_sampling::select_random(cloud, 1, 42).empty());
}
