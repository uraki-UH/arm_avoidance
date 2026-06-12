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
  g_params.lambda = 10;
  g_params.max_nodes = 1000000;
  g_params.max_age = 200;
  g_params.n_best_candidates = 2;

  SpatialTreeParams<double> s_params;
  s_params.max_nodes_per_cell = 20;
  s_params.min_cell_size = 0.0;

  // 2. Initialize GNG (2D Quadtree)
  // 2. Initialize GNG (Quick Hysteresis mode)
  GrowingNeuralGas<double, 2, LazyNeighborUpdate, false> gng(
      Point<double, 2>(100.0, 100.0), g_params, s_params);

  // 3. Pre-generate Samples
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> dist(0, 2.0 * M_PI);
  std::uniform_real_distribution<double> radius_dist(30.0, 40.0);

  const int iterations = 1000000;
  std::cout << "Pre-generating " << iterations << " samples..." << std::endl;
  std::vector<Point<double, 2>> samples;
  samples.reserve(iterations);
  for (int i = 0; i < iterations; ++i) {
    double angle = dist(gen);
    double r = radius_dist(gen);
    samples.push_back(Point<double, 2>(r * std::cos(angle), r * std::sin(angle)));
  }

  // 4. Pre-fill nodes
  std::cout << "Pre-filling" << g_params.max_nodes << "nodes to reach steady state..." << std::endl;
  for (int i = 0; i < g_params.max_nodes; ++i) {
    double angle = dist(gen);
    double r = radius_dist(gen);
    gng.addNode(Point<double, 2>(r * std::cos(angle), r * std::sin(angle)));
  }

  // 5. Training Loop (Pure Benchmark at 100k nodes)
  std::cout << "Starting Pure GNG training on a circle (2D Quadtree, 100k nodes)..." << std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iterations; ++i) {
    gng.train_step(samples[i]);
    if (i > 0 && i % 100000 == 0) {
      std::cout << "Step " << i << ": Active Nodes = " << gng.getNodesCount()
                << std::endl;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  std::cout << "\nTraining Complete!" << std::endl;
  std::cout << "Total Time (Core Logic): " << duration.count() << " ms" << std::endl;
  std::cout << "Average Time per iteration: " << (double)duration.count() / iterations * 1000.0 << " us" << std::endl;
  std::cout << "Final Active Nodes = " << gng.getNodesCount() << std::endl;

  return 0;
}
