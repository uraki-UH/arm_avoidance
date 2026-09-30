#include "robot_model/robot_voxelizer.hpp"
#include "robot_model/urdf_loader.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// 実FK経路を含まない、リンクローカル形状の独立照合用出力。
int main(int argc, char **argv) {
  try {
    if (argc != 6) {
      throw std::runtime_error("usage: dump_local_voxels URDF RESOURCE_ROOT MESH_ROOT RESOLUTION OUTPUT");
    }
    const double resolution = std::stod(argv[4]);
    if (!std::isfinite(resolution) || resolution <= 0.0 || std::filesystem::exists(argv[5])) {
      throw std::runtime_error("invalid resolution or existing output");
    }
    const auto model = simulation::loadRobotFromUrdf(argv[1], argv[2], argv[3]);
    std::vector<std::string> names;
    for (const std::string side : {"L", "R"}) {
      names.push_back(side + "_shoulder_mount");
      for (int idx = 2; idx <= 7; ++idx) names.push_back(side + "_link" + std::to_string(idx));
      names.push_back(side + "_gripper_base");
      names.push_back(side + "_tcp");
      names.push_back(side + "_finger_left");
      names.push_back(side + "_finger_right");
    }
    const GNG::Analysis::IndexVoxelGrid grid(resolution);
    const auto clouds = simulation::RobotVoxelizer::build(model, names, grid, {}, 0.0);
    nlohmann::json result = {{"urdf_path", argv[1]}, {"resolution_m", resolution},
                             {"num_link_slots", names.size()}, {"num_nonempty_links", clouds.size()}};
    result["links"] = nlohmann::json::array();
    result["geometry_files"] = nlohmann::json::array();
    result["geometry_files"].push_back(argv[1]);
    for (std::size_t idx = 0; idx < names.size(); ++idx) {
      nlohmann::json entry = {{"link_id", idx}, {"name", names[idx]},
                              {"centers", nlohmann::json::array()}};
      for (const auto &cloud : clouds) {
        if (cloud.name != names[idx]) continue;
        for (const auto &point : cloud.local_voxel_centers) {
          entry["centers"].push_back({point.x(), point.y(), point.z()});
        }
      }
      result["links"].push_back(std::move(entry));
      const auto *link = model.getLink(names[idx]);
      if (link) {
        for (const auto &shape : link->collisions) {
          if (shape.geometry.type == simulation::GeometryType::MESH) {
            result["geometry_files"].push_back(shape.geometry.mesh_filename);
          }
        }
      }
    }
    if (clouds.size() != 18 || names.size() != 22) throw std::runtime_error("unexpected link count");
    std::ofstream output(argv[5]);
    if (!output) throw std::runtime_error("cannot open output");
    output << result.dump(2) << '\n';
    output.close();
    if (!output) throw std::runtime_error("cannot finish output");
    std::cout << "local voxel dump completed: links=" << clouds.size() << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
