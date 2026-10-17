#include "back_end/reglib.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  if (argc < 4) {
    std::cerr << "Usage: headless_corresp source.pcd target.pcd correspondence_count [backend]"
              << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source) != 0 ||
      pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *target) != 0) {
    std::cerr << "PCD read failed" << std::endl;
    return 2;
  }

  const int requested = std::max(3, std::atoi(argv[3]));
  const int count = std::min({requested, static_cast<int>(source->size()),
      static_cast<int>(target->size())});
  if (count < 3) {
    std::cerr << "At least 3 corresponding points are required" << std::endl;
    return 3;
  }

  Eigen::MatrixX3d source_corresp(count, 3);
  Eigen::MatrixX3d target_corresp(count, 3);
  for (int index = 0; index < count; ++index) {
    source_corresp.row(index) << source->points[index].x, source->points[index].y,
      source->points[index].z;
    target_corresp.row(index) << target->points[index].x, target->points[index].y,
      target->points[index].z;
  }

  g3reg::Config config;
  config.reset_config();
  config.verbose = false;
  config.back_end = argc >= 5 ? argv[4] : "pagor";
  config.vertex_type = "point";
  config.vertex_info.type = clique_solver::VertexType::POINT;
  config.set_noise_bounds({0.005});
  config.verify_mtd = "pc";

  const FRGresult result = g3reg::SolveFromCorresp(
    source_corresp, target_corresp, source_corresp, target_corresp, config);
  std::cout << "correspondences=" << count << std::endl;
  std::cout << "backend=" << config.back_end << std::endl;
  std::cout << "graph_time_ms=" << result.graph_time << std::endl;
  std::cout << "clique_time_ms=" << result.clique_time << std::endl;
  std::cout << "solver_time_ms=" << result.tf_solver_time << std::endl;
  std::cout << "verify_time_ms=" << result.verify_time << std::endl;
  std::cout << "total_time_ms=" << result.total_time << std::endl;
  std::cout << "transform=" << std::endl << result.tf << std::endl;
  return 0;
}
