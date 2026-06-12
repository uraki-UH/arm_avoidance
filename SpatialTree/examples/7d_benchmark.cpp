#include <SpatialTree/GNG/GNG.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace SpatialTree;

template <int Dim>
Point<double, Dim> sample_hypersphere_surface(std::mt19937 &gen,
                                              double radius) {
  std::normal_distribution<double> dist(0.0, 1.0);
  Point<double, Dim> p;
  double norm = 0;
  for (int i = 0; i < Dim; ++i) {
    p[i] = dist(gen);
    norm += p[i] * p[i];
  }
  norm = std::sqrt(norm);
  for (int i = 0; i < Dim; ++i) {
    p[i] = (p[i] / norm) * radius;
  }
  return p;
}

int main() {
  const int Dim = 7;
  const int iterations = 1000000;

  std::cout << "Starting 7D Pure GNG Performance Benchmark..." << std::endl;

  GNGParams<double> g_params;
  g_params.lambda = 100;
  g_params.max_nodes = 100000;

  SpatialTreeParams<double> s_params;
  s_params.max_nodes_per_cell = 500; // 7D requires even more nodes per cell
  s_params.min_cell_size = 0.0;

  GrowingNeuralGas<double, Dim, LazyNeighborUpdate, false> gng(
      Point<double, Dim>(100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0),
      g_params, s_params);

  // 3. Pre-generate Training Samples
  std::mt19937 gen(42);
  std::cout << "Pre-generating " << iterations << " training samples..."
            << std::endl;
  std::vector<Point<double, Dim>> samples;
  samples.reserve(iterations);
  for (int i = 0; i < iterations; ++i) {
    samples.push_back(sample_hypersphere_surface<Dim>(gen, 40.0));
  }

  // 4. Pre-fill nodes 
  std::cout << "Pre-filling" << g_params.max_nodes
            << " nodes to reach steady state..."
            << std::endl;
  for (int i = 0; i < g_params.max_nodes; ++i) {
    gng.addNode(sample_hypersphere_surface<Dim>(gen, 40.0));
  }

  // 5. Training Loop (Pure Benchmark)
  std::cout << "Starting 7D Pure GNG training (100k nodes)..." << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iterations; ++i) {
    gng.train_step(samples[i]);
    if (i > 0 && i % 100000 == 0) {
      std::cout << "Step " << i << ": Active Nodes = " << gng.getNodesCount()
                << std::endl;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);

  std::cout << "\n7D Training Complete!" << std::endl;
  std::cout << "Total Time (Core Logic): " << duration.count() << " ms"
            << std::endl;
  std::cout << "Average Time per iteration: "
            << (double)duration.count() / iterations * 1000.0 << " us"
            << std::endl;
  std::cout << "Final Active Nodes = " << gng.getNodesCount() << std::endl;

  return 0;
}
