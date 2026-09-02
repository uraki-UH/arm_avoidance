#include <Eigen/Core>

#include <teaser/registration.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
  if (argc < 4) {
    std::cerr <<
      "Usage: teaser_correspondence source.pcd target.pcd correspondence_count [outlier_count]"
      << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> source;
  pcl::PointCloud<pcl::PointXYZ> target;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], source) != 0 ||
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], target) != 0)
  {
    std::cerr << "PCD read failed" << std::endl;
    return 2;
  }

  const int requested = std::max(3, std::atoi(argv[3]));
  const int count = std::min({requested, static_cast<int>(source.size()),
      static_cast<int>(target.size())});
  const int outlier_count = std::min(
    std::max(0, argc >= 5 ? std::atoi(argv[4]) : 0), count);
  if (count < 3) {
    std::cerr << "At least 3 corresponding points are required" << std::endl;
    return 3;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> source_points(3, count);
  Eigen::Matrix<double, 3, Eigen::Dynamic> target_points(3, count);
  for (int index = 0; index < count; ++index) {
    source_points.col(index) << source.points[index].x, source.points[index].y,
      source.points[index].z;
    target_points.col(index) << target.points[index].x, target.points[index].y,
      target.points[index].z;
  }

  std::mt19937 random_engine(20260902U);
  std::uniform_real_distribution<double> offset(-1.0, 1.0);
  for (int index = 0; index < outlier_count; ++index) {
    target_points.col(index).array() += Eigen::Vector3d(
      offset(random_engine), offset(random_engine), offset(random_engine)).array();
  }

  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.005;
  params.cbar2 = 1.0;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  teaser::RobustRegistrationSolver solver(params);
  const auto begin = std::chrono::steady_clock::now();
  solver.solve(source_points, target_points);
  const auto end = std::chrono::steady_clock::now();
  const auto solution = solver.getSolution();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(end - begin).count();
  std::size_t inlier_count = 0U;
  for (int index = 0; index < count; ++index) {
    const Eigen::Vector3d residual = solution.rotation * source_points.col(index) +
      solution.translation - target_points.col(index);
    if (residual.norm() <= params.noise_bound) {
      ++inlier_count;
    }
  }

  std::cout << "correspondences=" << count << std::endl;
  std::cout << "outliers=" << outlier_count << std::endl;
  std::cout << "time_ms=" << elapsed_ms << std::endl;
  std::cout << "inliers=" << inlier_count << std::endl;
  std::cout << "rotation=" << std::endl << solution.rotation << std::endl;
  std::cout << "translation=" << std::endl << solution.translation << std::endl;
  return 0;
}
