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
  g_params.utility_k = 4.0;      // 削除の厳しさを緩和（単位統一後の適正値）
  g_params.utility_beta = 0.995; // Error(0.995)より少しだけ早く過去を忘れる
  g_params.utility_inheritance = 0.1; // 新基本ノードの初期貢献度は低く保つ
  g_params.eps_w = 0.1;
  g_params.eps_n = 0.001;

  SpatialTreeParams<double> s_params;
  s_params.max_nodes_per_cell = 20;
  s_params.min_cell_size = 0.01; // 無限分割を防ぐための安全値

  // 2. Initialize GNG (2D) - 爆速版オプションを有効化
  GrowingNeuralGasUtility<double, 2, LazyNeighborUpdate, true> gng(
      Point<double, 2>(100.0, 100.0), g_params, s_params);

  // 3. Generators
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> angle_dist(0, 2.0 * M_PI);
  std::normal_distribution<double> noise(0, 2.0);
  std::uniform_real_distribution<double> line_dist(-35.0, 35.0);

  auto generate_circle = [&]() {
    double angle = angle_dist(gen);
    double radius = 30.0 + noise(gen);
    return Point<double, 2>(radius * std::cos(angle), radius * std::sin(angle));
  };

  // 十字架の「太さ」を定義
  const double cross_width = 2.0;
  const double half_len = 40.0;

  double area1 = (2 * half_len) * (2 * cross_width);
  double area2 = (2 * cross_width) * (half_len - cross_width);
  double area3 = area2;
  std::discrete_distribution<> area_dist({area1, area2, area3});

  auto generate_cross = [&]() {
    int region = area_dist(gen);
    std::uniform_real_distribution<double> ux(-half_len, half_len);
    std::uniform_real_distribution<double> uw(-cross_width, cross_width);
    std::uniform_real_distribution<double> uy_top(cross_width, half_len);
    std::uniform_real_distribution<double> uy_bot(-half_len, -cross_width);

    if (region == 0) { // 横棒
      return Point<double, 2>(ux(gen), uw(gen));
    } else if (region == 1) { // 縦棒上部
      return Point<double, 2>(uw(gen), uy_top(gen));
    } else { // 縦棒下部
      return Point<double, 2>(uw(gen), uy_bot(gen));
    }
  };

  // 4. Pre-generate Data
  const int phase1_steps = 50000;
  const int phase2_steps = 200000;
  std::cout << "Pre-generating " << (phase1_steps + phase2_steps)
            << " samples..." << std::endl;

  std::vector<Point<double, 2>> samples_p1;
  samples_p1.reserve(phase1_steps);
  for (int i = 0; i < phase1_steps; ++i)
    samples_p1.push_back(generate_circle());

  std::vector<Point<double, 2>> samples_p2;
  samples_p2.reserve(phase2_steps);
  for (int i = 0; i < phase2_steps; ++i)
    samples_p2.push_back(generate_cross());

  // 4. Visualizer
  Visualizer2D<double>::Config v_config;
  v_config.video_path = "dynamic_2d_cross.mp4";
  v_config.show_window = false;
  Visualizer2D<double> viz(v_config, Point<double, 2>(100.0, 100.0));

  // 5. Training Loop
  const int record_interval = 500; // 200,000 / 500 = 400 frames (~13s)

  auto start_time = std::chrono::high_resolution_clock::now();

  auto check_nodes = [&](int step) {
    auto nodes = gng.getActiveNodes();
    if (nodes.empty())
      return;

    Point<double, 2> avg(0, 0);
    int nan_count = 0, zero_count = 0;

    for (auto *n : nodes) {
      if (std::isnan(n->position[0]) || std::isnan(n->position[1]))
        nan_count++;
      if (std::abs(n->position[0]) < 1e-9 && std::abs(n->position[1]) < 1e-9)
        zero_count++;
      avg += n->position;
    }
    avg = (1.0 / nodes.size()) * avg;

    if (step % 10000 == 0 || nan_count > 0 || zero_count == nodes.size()) {
      std::cout << "[Step " << step << "] Nodes=" << nodes.size() << " Avg=("
                << avg[0] << "," << avg[1] << ")"
                << " NaNs=" << nan_count << " ZeroNodes=" << zero_count
                << std::endl;
    }

    if (nan_count > 0 || (nodes.size() > 10 && zero_count == nodes.size())) {
      std::cerr << "CRITICAL: Node corruption detected!" << std::endl;
      std::exit(1);
    }
  };

  std::cout << "Starting GNG-U 2D Demo (Recording to dynamic_2d_cross.mp4)"
            << std::endl;
  std::cout << "Phase 1: Circle Distribution..." << std::endl;
  for (int i = 0; i < phase1_steps; ++i) {
    gng.train_step(samples_p1[i]);
    if (i % 5000 == 0)
      check_nodes(i);
    if (i % record_interval == 0) {
      viz.recordFrame(gng, samples_p1[i]);
    }
  }

  std::cout << "Phase 2: 2D Cross Distribution..." << std::endl;
  for (int i = 0; i < phase2_steps; ++i) {
    gng.train_step(samples_p2[i]);
    if (i % 5000 == 0)
      check_nodes(phase1_steps + i);
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
