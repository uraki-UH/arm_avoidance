#include <SpatialTree/GNG/GNG.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

int main() {
  using namespace SpatialTree;

  // 1. Setup Parameters
  GNGParams<double> g_params;
  g_params.lambda = 100000000; // Disable further node addition
  g_params.max_nodes = 1000000;
  g_params.max_age = 200;

  SpatialTreeParams<double> s_params;
  s_params.max_nodes_per_cell = 20;
  s_params.min_cell_size = 0.0;

  // 2. Initialize GNG (3D Octree)
  GrowingNeuralGas<double, 3, LazyNeighborUpdate, false> gng(
      Point<double, 3>(100.0, 100.0, 100.0), g_params, s_params);

  // 3. Setup Random Source (Spherical Shell Distribution)
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> u_dist(0, 1.0);
  std::uniform_real_distribution<double> v_dist(0, 1.0);
  std::normal_distribution<double> noise(0, 1.5);

  auto generate_sample = [&]() {
    double theta = 2.0 * M_PI * u_dist(gen);
    double phi = std::acos(2.0 * v_dist(gen) - 1.0);
    double radius = 35.0 + noise(gen);

    return Point<double, 3>(radius * std::sin(phi) * std::cos(theta),
                            radius * std::sin(phi) * std::sin(theta),
                            radius * std::cos(phi));
  };

  // 4. 最大ノード数まで事前に追加
  std::cout << "Pre-filling" << g_params.max_nodes << 
  "nodes to reach steady state..." << std::endl;
  for (int i = 0; i < g_params.max_nodes; ++i) {
    gng.addNode(generate_sample());
  }

  // 5. Pre-generate Data (Pure Benchmarking Methodology)
  const int iterations = 1000000;
  std::cout << "Pre-generating " << iterations << " training samples..." << std::endl;
  std::vector<Point<double, 3>> samples;
  samples.reserve(iterations);
  for (int i = 0; i < iterations; ++i) {
    samples.push_back(generate_sample());
  }

  // 6. Training Loop (Pure Benchmark)
  std::cout << "Starting Pure GNG training on a sphere (3D Octree, 100k nodes)..." << std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iterations; ++i) {
    gng.train_step(samples[i]);
    if (i % 100000 == 0) {
      std::cout << "Step " << i << ": Active Nodes = " << gng.getNodesCount()
                << std::endl;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  std::cout << "\nTraining Complete!" << std::endl;
  std::cout << "Total Time (Core Logic): " << duration.count() << " ms" << std::endl;
  std::cout << "Average Time per iteration: " << (double)duration.count() / iterations * 1000.0 << " us" << std::endl;
  std::cout << "Final Active Nodes = " << gng.getNodesCount() << std::endl;

  return 0;
}
