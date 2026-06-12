#include <SpatialTree/GNG/GNG.hpp>
#include <SpatialTree/GNG/GNGUtility.hpp>
#include <SpatialTree/extra/Visualizer.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

using namespace SpatialTree;

int main() {
  // 1. Setup Parameters
  GNGUtilityParams<double> g_params;
  g_params.lambda = 50;
  g_params.max_nodes = 2500;
  g_params.max_age = 50;
  g_params.utility_k = 5.0;

  SpatialTreeParams<double> s_params;
  s_params.max_nodes_per_cell = 20;
  s_params.min_cell_size = 0.01; // 無限分割を防ぐための安全値

  // 2. Initialize GNG (3D)
  GrowingNeuralGasUtility<double, 3, LazyNeighborUpdate, true> gng(
      Point<double, 3>(100.0, 100.0, 100.0), g_params, s_params);

  // 3. Generators
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> u_dist(0, 1.0);
  std::uniform_real_distribution<double> v_dist(0, 1.0);
  std::normal_distribution<double> noise(0, 2.0);
  std::uniform_real_distribution<double> line_dist(-35.0, 35.0);
  std::uniform_int_distribution<int> axis_dist(0, 2);

  auto generate_sphere = [&]() {
    double theta = 2.0 * M_PI * u_dist(gen);
    double phi = std::acos(2.0 * v_dist(gen) - 1.0);
    double radius = 30.0 + noise(gen);
    return Point<double, 3>(radius * std::sin(phi) * std::cos(theta),
                            radius * std::sin(phi) * std::sin(theta),
                            radius * std::cos(phi));
  };

  auto generate_cross = [&]() {
    int axis = axis_dist(gen);
    double val = line_dist(gen);
    double n1 = noise(gen);
    double n2 = noise(gen);
    if (axis == 0)
      return Point<double, 3>(val, n1, n2);
    if (axis == 1)
      return Point<double, 3>(n1, val, n2);
    return Point<double, 3>(n1, n2, val);
  };

  // 4. Pre-generate Data
  const int phase1_steps = 50000;
  const int phase2_steps = 150000;
  std::cout << "Pre-generating " << (phase1_steps + phase2_steps)
            << " samples..." << std::endl;

  std::vector<Point<double, 3>> samples_p1;
  samples_p1.reserve(phase1_steps);
  for (int i = 0; i < phase1_steps; ++i)
    samples_p1.push_back(generate_sphere());

  std::vector<Point<double, 3>> samples_p2;
  samples_p2.reserve(phase2_steps);
  for (int i = 0; i < phase2_steps; ++i)
    samples_p2.push_back(generate_cross());

  // 4. Visualizer
  Visualizer3D<double>::Config v_config;
  v_config.video_path = "dynamic_3d_cross.mp4";
  v_config.show_window = false;
  v_config.auto_rotate = true;
  Visualizer3D<double> viz(v_config, Point<double, 3>(100.0, 100.0, 100.0));

  // 5. Training Loop
  const int record_interval = 500; // 200,000 / 500 = 400 frames (~13s)

  std::cout << "Starting GNG-U 3D Demo (Recording to dynamic_3d_cross.mp4)"
            << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  std::cout << "Phase 1: Sphere Distribution..." << std::endl;
  for (int i = 0; i < phase1_steps; ++i) {
    gng.train_step(samples_p1[i]);
    if (i % record_interval == 0) {
      viz.recordFrame(gng, samples_p1[i]);
    }
  }

  std::cout << "Phase 2: 3D Cross Distribution..." << std::endl;
  for (int i = 0; i < phase2_steps; ++i) {
    gng.train_step(samples_p2[i]);
    if (i % record_interval == 0) {
      viz.recordFrame(gng, samples_p2[i]);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);

  std::cout << "\nComplete! (Total 200,000 steps)" << std::endl;
  std::cout << "Final Active Nodes: " << gng.getNodesCount() << std::endl;
  std::cout << "Total Time (Core Logic): " << duration.count() << " ms"
            << std::endl;
  std::cout << "Average Time per step: "
            << (double)duration.count() / (phase1_steps + phase2_steps) * 1000.0
            << " us" << std::endl;

  return 0;
}
