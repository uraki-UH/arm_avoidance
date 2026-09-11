#include <ais_gng/observation_pixels.hpp>
#include <pointcloud_sampling/stratified.hpp>
#include <gtest/gtest.h>

namespace pixels = fuzzrobo::observation_pixels;

TEST(observation_pixels, random_selection_keeps_original_pixel_ids) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 32; cloud.height = 32; cloud.point_step = 16; cloud.row_step = 32 * 16 + 8;
    cloud.data.resize(cloud.row_step * cloud.height);
    for (uint32_t axis = 0; axis < 3; ++axis) {
        sensor_msgs::msg::PointField field;
        field.name = std::string(1, "xyz"[axis]); field.offset = axis * 4;
        field.datatype = field.FLOAT32; field.count = 1; cloud.fields.push_back(field);
    }
    for (uint32_t idx = 0; idx < 1024; ++idx) {
        const float xyz[] = {float(idx), 0, idx % 3 ? 1.0F : NAN};
        auto *point = cloud.data.data() + idx / 32 * cloud.row_step + idx % 32 * 16;
        std::memcpy(point, xyz, 12);
        const uint32_t pixel_idx = 1023 - idx;
        std::memcpy(point + 12, &pixel_idx, 4);
    }
    const auto selected = pointcloud_sampling::select_random(cloud, 100, 1);
    ASSERT_EQ(selected.size(), 100U);
    auto sorted = selected;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(std::unique(sorted.begin(), sorted.end()), sorted.end());
    EXPECT_EQ(selected, pointcloud_sampling::select_random(cloud, 100, 1));
    EXPECT_NE(selected, pointcloud_sampling::select_random(cloud, 100, 2));
    const auto complete = pointcloud_sampling::select_random(cloud, 2000, 1);
    EXPECT_EQ(complete.size(), 682U);
    EXPECT_FALSE(std::is_sorted(complete.begin(), complete.end()));
    EXPECT_NE(complete, pointcloud_sampling::select_random(cloud, 2000, 2));
    EXPECT_EQ(complete, pointcloud_sampling::select_random(cloud, 0, 1));
    for (const auto idx : complete) EXPECT_NE(idx % 3, 0U);
    gng_observation::pixel_view view;
    ASSERT_TRUE(pixels::make_view(cloud, 32, 32, true, &selected, selected.size(), view));
    for (uint32_t idx = 0; idx < selected.size(); ++idx) EXPECT_EQ(view.get(idx), selected[idx]);
    sensor_msgs::msg::PointField field;
    field.name = "pixel_idx"; field.offset = 12; field.datatype = field.UINT32; field.count = 1;
    cloud.fields.push_back(field);
    ASSERT_TRUE(pixels::make_view(cloud, 32, 32, false, &selected, selected.size(), view));
    for (uint32_t idx = 0; idx < selected.size(); ++idx) EXPECT_EQ(view.get(idx), 1023 - selected[idx]);
}

TEST(observation_pixels, borrowed_view_with_sampling_and_row_padding) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 2; cloud.height = 2; cloud.point_step = 5; cloud.row_step = 13;
    cloud.data.resize(26);
    sensor_msgs::msg::PointField field;
    field.name = "pixel_idx"; field.offset = 1; field.datatype = field.UINT32; field.count = 1;
    cloud.fields = {field};
    std::vector<uint32_t> selected{3, 0, 2, UINT32_MAX};
    const std::array<uint32_t, 4> expected{11, 8, UINT32_MAX, UINT32_MAX};
    for (const bool is_bigendian : {false, true}) {
        cloud.is_bigendian = is_bigendian;
        for (uint32_t idx = 0; idx < 4; ++idx) {
            const uint32_t value = idx == 2 ? UINT32_MAX : idx + 8;
            for (uint32_t byte = 0; byte < 4; ++byte) {
                cloud.data[(idx / 2)*13 + (idx % 2)*5 + 1 + byte] = value >> (8 * (is_bigendian ? 3-byte : byte));
            }
        }
        gng_observation::pixel_view view;
        ASSERT_TRUE(pixels::make_view(cloud, 8, 6, false, &selected, selected.size(), view));
        EXPECT_EQ(view.data, cloud.data.data());
        EXPECT_EQ(view.selected_ids, selected.data());
        for (uint32_t idx = 0; idx < selected.size(); ++idx) {EXPECT_EQ(view.get(idx), expected[idx]);}
        EXPECT_EQ(view.get(selected.size()), UINT32_MAX);
        selected[0] = 1;
        EXPECT_EQ(view.get(0), 9U);
        selected[0] = 3;
    }
    gng_observation::pixel_view view;
    cloud.fields[0].offset = 2;
    EXPECT_FALSE(pixels::make_view(cloud, 8, 6, false, nullptr, 4, view));
    cloud.fields.clear();
    ASSERT_TRUE(pixels::make_view(cloud, 2, 2, true, &selected, 4, view));
    EXPECT_EQ(view.get(0), 3U);
    EXPECT_EQ(view.get(3), UINT32_MAX);
    EXPECT_FALSE(pixels::make_view(cloud, 8, 6, true, nullptr, 4, view));
    EXPECT_FALSE(pixels::make_view(cloud, 2, 2, true, &selected, 5, view));
    cloud.data.resize(25);
    EXPECT_FALSE(pixels::make_view(cloud, 2, 2, true, nullptr, 4, view));
}

TEST(observation_pixels, borrowed_uv_and_invalid_pixel) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 2; cloud.height = 1; cloud.point_step = 4; cloud.row_step = 8;
    sensor_msgs::msg::PointField u, v;
    u.name = "u"; u.offset = 0; u.datatype = u.UINT16; u.count = 1;
    v = u; v.name = "v"; v.offset = 2;
    cloud.fields = {u, v}; cloud.data = {1,0,2,0, 8,0,0,0};
    gng_observation::pixel_view view;
    ASSERT_TRUE(pixels::make_view(cloud, 8, 6, false, nullptr, 2, view));
    EXPECT_EQ(view.get(0), 17U);
    EXPECT_EQ(view.get(1), UINT32_MAX);
    cloud.fields[1].datatype = v.FLOAT32;
    EXPECT_FALSE(pixels::make_view(cloud, 8, 6, false, nullptr, 2, view));
}

TEST(observation_pixels, table_cache_and_calibration_change) {
    sensor_msgs::msg::CameraInfo info;
    info.width = 8; info.height = 6;
    info.k = {4, 0, 3, 0, 4, 2, 0, 0, 1};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    pixels::angle_table table;
    ASSERT_TRUE(table.prepare(info, rotation));
    EXPECT_EQ(table.build_num, 1U);
    for (uint32_t row = 0; row < 6; ++row) {
        for (uint32_t column = 0; column < 8; ++column) {
            gng_observation::ray_angles expected;
            gng_observation::can_quantize_ray((column - 3.0)/4, (row - 2.0)/4, 1, expected);
            const auto actual = table.values[row*8+column];
            EXPECT_EQ(actual.yaw, expected.yaw);
            EXPECT_EQ(actual.pitch, expected.pitch);
            EXPECT_EQ(actual.has_yaw, expected.has_yaw);
        }
    }
    ASSERT_TRUE(table.prepare(info, rotation));
    EXPECT_EQ(table.build_num, 1U);
    rotation = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
    ASSERT_TRUE(table.prepare(info, rotation));
    EXPECT_EQ(table.build_num, 2U);
    info.k[0] = 5;
    ASSERT_TRUE(table.prepare(info, rotation));
    EXPECT_EQ(table.build_num, 3U);
    info.d = {0.1};
    EXPECT_FALSE(table.prepare(info, rotation));
    EXPECT_EQ(table.build_num, 3U);
}

TEST(observation_pixels, explicit_ids_and_sampling) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 4; cloud.height = 1; cloud.point_step = 4; cloud.row_step = 16;
    sensor_msgs::msg::PointField field;
    field.name = "pixel_idx"; field.offset = 0; field.datatype = field.UINT32; field.count = 1;
    cloud.fields = {field};
    cloud.data = {7,0,0,0, 3,0,0,0, 255,255,255,255, 12,0,0,0};
    std::vector<uint32_t> selected{3, 0, 2};
    gng_observation::pixel_view view;
    ASSERT_TRUE(pixels::make_view(cloud, 8, 6, false, &selected, 3, view));
    EXPECT_EQ(view.get(0), 12U);
    EXPECT_EQ(view.get(1), 7U);
    EXPECT_EQ(view.get(2), UINT32_MAX);
    cloud.is_bigendian = true;
    cloud.data = {0,0,0,7, 0,0,0,3, 255,255,255,255, 0,0,0,12};
    ASSERT_TRUE(pixels::make_view(cloud, 8, 6, false, &selected, 3, view));
    EXPECT_EQ(view.get(0), 12U);
    EXPECT_EQ(view.get(1), 7U);
    EXPECT_EQ(view.get(2), UINT32_MAX);
    cloud.fields[0].datatype = field.FLOAT32;
    EXPECT_FALSE(pixels::make_view(cloud, 8, 6, false, nullptr, 4, view));
}

TEST(observation_pixels, organized_and_uv_with_row_padding) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 2; cloud.height = 2; cloud.point_step = 4; cloud.row_step = 12;
    cloud.data.resize(24);
    gng_observation::pixel_view view;
    EXPECT_FALSE(pixels::make_view(cloud, 2, 2, false, nullptr, 4, view));
    ASSERT_TRUE(pixels::make_view(cloud, 2, 2, true, nullptr, 4, view));
    for (uint32_t idx = 0; idx < 4; ++idx) {EXPECT_EQ(view.get(idx), idx);}
    sensor_msgs::msg::PointField u, v;
    u.name = "u"; u.offset = 0; u.datatype = u.UINT16; u.count = 1;
    v = u; v.name = "v"; v.offset = 2;
    cloud.fields = {u, v};
    cloud.data = {1,0,1,0, 0,0,1,0, 99,99,99,99, 1,0,0,0, 0,0,0,0, 99,99,99,99};
    ASSERT_TRUE(pixels::make_view(cloud, 2, 2, false, nullptr, 4, view));
    for (uint32_t idx = 0; idx < 4; ++idx) {EXPECT_EQ(view.get(idx), 3U - idx);}
    cloud.data.resize(23);
    EXPECT_FALSE(pixels::make_view(cloud, 2, 2, false, nullptr, 4, view));
}
