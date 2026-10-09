#include <ais_gng/self_point_filter.hpp>
#include <fstream>
#include <iostream>

// 同一の合成点群・同一seedによる選択前処理の比較。ROS通信・GNG本体・マスク生成は対象外
int main(int argc, char **argv) {
  if (argc != 6) return 2;
  const std::string mode = argv[1];
  const uint32_t seed = std::stoul(argv[2]), self_percent = std::stoul(argv[3]);
  const uint32_t max_points = std::stoul(argv[4]);
  if ((mode != "off" && mode != "filter" && mode != "labels") || self_percent > 100) return 2;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 640; cloud.height = 480; cloud.point_step = 16; cloud.row_step = cloud.width * 16;
  cloud.data.resize(cloud.row_step * cloud.height);
  for (uint32_t axis = 0; axis < 3; ++axis) {
    sensor_msgs::msg::PointField field; field.name = std::string(1, "xyz"[axis]);
    field.offset = axis * 4; field.datatype = field.FLOAT32; field.count = 1; cloud.fields.push_back(field);
  }
  std::mt19937 random(seed); std::uniform_real_distribution<float> unit(.001F, .999F);
  auto mask_from_cloud = Eigen::Isometry3d::Identity();
  mask_from_cloud.linear() = Eigen::AngleAxisd(.37, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  mask_from_cloud.translation() = Eigen::Vector3d(.3, -.2, .1);
  const auto cloud_from_mask = mask_from_cloud.inverse();
  for (uint32_t idx = 0; idx < cloud.width * cloud.height; ++idx) {
    const Eigen::Vector3d point((idx % 100 < self_percent ? 0.F : 2.F) + unit(random), unit(random), unit(random));
    const Eigen::Vector3f xyz = (cloud_from_mask * point).cast<float>();
    std::memcpy(cloud.data.data() + idx * 16, xyz.data(), 12);
  }
  voxel_msgs::msg::Voxel message;
  message.header.frame_id = "robot"; message.header.stamp.sec = 1; message.voxel_size = .02F;
  message.x_shift = 42; message.y_shift = 21; message.z_shift = 0; message.offset = 1000000;
  voxel_idx::VoxelIndexingSchema schema{42, 21, 0, 1000000, .02};
  for (int x = 0; x < 50; ++x) for (int y = 0; y < 50; ++y) for (int z = 0; z < 50; ++z)
    message.data.push_back(schema.pack({x, y, z}));
  const auto mask_start = std::chrono::steady_clock::now();
  const fuzzrobo::self_point_filter::mask_snapshot mask(message);
  const double mask_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - mask_start).count();
  std::vector<uint8_t> labels; double total_ms = 0; uint64_t checksum = 0, num_selected = 0;
  for (uint32_t iter = 0; iter < 25; ++iter) {
    const auto start = std::chrono::steady_clock::now();
    auto selected = mode == "off" ? pointcloud_sampling::select_random(cloud, max_points, seed + iter) :
        fuzzrobo::self_point_filter::select_points(cloud, max_points, seed + iter, fuzzrobo::PointSamplingMode::Random,
            mask, mask_from_cloud, mode == "labels" ? &labels : nullptr);
    if (mode == "labels") {
      const auto output = fuzzrobo::self_point_filter::make_labelled_cloud(cloud, labels);
      checksum += output.data.size();
    }
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (iter >= 5) total_ms += elapsed;
    num_selected = selected.size();
    const uint32_t num_eligible = mode == "off" ? 307200 : 3072 * (100 - self_percent);
    if (num_selected != (max_points ? std::min(max_points, num_eligible) : num_eligible)) return 3;
    std::vector<uint8_t> seen(307200, 0);
    for (const uint32_t idx : selected) {
      if (idx >= seen.size() || seen[idx]++ || (mode != "off" && idx % 100 < self_percent)) return 3;
      checksum += idx;
    }
    if (mode == "labels") for (uint32_t idx = 0; idx < labels.size(); ++idx)
      if (labels[idx] != (idx % 100 < self_percent ? 1 : 0)) return 3;
  }
  std::ofstream output(argv[5]);
  output << "{\"input_points\":307200,\"mask_cells\":" << mask.num_cells
         << ",\"max_points\":" << max_points << ",\"self_percent\":" << self_percent
         << ",\"selected_points\":" << num_selected << ",\"mean_ms\":" << total_ms / 20
         << ",\"mask_build_ms\":" << mask_ms << ",\"checksum\":" << checksum << "}\n";
  std::cout << mode << ": " << total_ms / 20 << " ms\n";
  return output.good() ? 0 : 4;
}
