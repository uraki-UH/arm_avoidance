#include <topo_fuzzy_viewer/common/pcl_converter.h>
#include <pointcloud_sampling/stratified.hpp>
#include <gtest/gtest.h>
#include <cstring>

TEST(pointcloud_sampling, viewer_preserves_rgb_intensity_and_pixels) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->width = 64; cloud->height = 48;
  cloud->point_step = 20; cloud->row_step = 64 * 20 + 16;
  cloud->data.resize(cloud->row_step * cloud->height);
  for (uint32_t idx = 0; idx < 5; ++idx) {
    sensor_msgs::msg::PointField field;
    field.name = std::array<std::string, 5>{"x", "y", "z", "rgb", "intensity"}[idx];
    field.offset = idx * 4; field.datatype = 7; field.count = 1;
    cloud->fields.push_back(field);
  }
  for (uint32_t idx = 0; idx < 64 * 48; ++idx) {
    const uint32_t rgb = idx % 2 ? 0x123456 : 0;
    const float xyz[] = {float(idx), 2, idx % 7 ? 1.0F : NAN};
    const float intensity = float(idx) * 2;
    auto *point = cloud->data.data() + (idx / 64) * cloud->row_step + (idx % 64) * 20;
    std::memcpy(point, xyz, 12);
    std::memcpy(point + 12, &rgb, 4);
    std::memcpy(point + 16, &intensity, 4);
  }
  const auto result = utils::convertFromRosMsg(cloud, 1000);
  ASSERT_EQ(result.pointCount, 1000U);
  ASSERT_EQ(result.colors.size(), 3000U);
  ASSERT_EQ(result.intensities.size(), 1000U);
  uint32_t num_black = 0;
  std::vector<uint32_t> selected_ids;
  for (uint32_t idx = 0; idx < 1000; ++idx) {
    const auto source_idx = static_cast<uint32_t>(result.positions[idx * 3]);
    ASSERT_LT(source_idx, 64U * 48);
    EXPECT_NE(source_idx % 7, 0U);
    selected_ids.push_back(source_idx);
    EXPECT_EQ(result.intensities[idx], source_idx * 2);
    EXPECT_EQ(result.colors[idx * 3], source_idx % 2 ? 0x12 : 0);
    EXPECT_EQ(result.colors[idx * 3 + 1], source_idx % 2 ? 0x34 : 0);
    EXPECT_EQ(result.colors[idx * 3 + 2], source_idx % 2 ? 0x56 : 0);
    num_black += source_idx % 2 == 0;
  }
  EXPECT_GT(num_black, 0U);
  std::sort(selected_ids.begin(), selected_ids.end());
  EXPECT_EQ(std::unique(selected_ids.begin(), selected_ids.end()), selected_ids.end());
  EXPECT_NE(utils::convertFromRosMsg(cloud, 1000).positions, result.positions);
  const auto complete = utils::convertFromRosMsg(cloud, 10000);
  EXPECT_EQ(complete.pointCount, 2633U);
  EXPECT_EQ(utils::convertFromRosMsg(cloud, 10000).positions, complete.positions);
  EXPECT_EQ(utils::convertFromRosMsg(cloud, 2633).positions, complete.positions);
  EXPECT_EQ(utils::convertFromRosMsg(cloud, 0).positions, complete.positions);
  for (uint32_t idx = 1; idx < complete.pointCount; ++idx)
    EXPECT_LT(complete.positions[(idx - 1) * 3], complete.positions[idx * 3]);
}
