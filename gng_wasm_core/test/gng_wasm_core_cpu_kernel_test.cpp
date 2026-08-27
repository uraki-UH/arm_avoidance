#include "gng_wasm_core/gng_kernel.hpp"

#include <array>
#include <iostream>
#include <vector>

int main() {
  gng_wasm_core::Config config;
  config.max_nodes = 64;
  config.iterations = 500;
  config.max_age = 80;
  config.eb = 0.055f;
  config.en = 0.006f;

  const std::array<gng_wasm_core::Point3f, 9> points{{
      {-0.10f, -0.10f, 0.00f},
      {-0.10f, 0.00f, 0.00f},
      {-0.10f, 0.10f, 0.00f},
      {0.00f, -0.10f, 0.00f},
      {0.00f, 0.00f, 0.05f},
      {0.00f, 0.10f, 0.00f},
      {0.10f, -0.10f, 0.00f},
      {0.10f, 0.00f, 0.00f},
      {0.10f, 0.10f, 0.00f},
  }};

  gng_wasm_core::GngKernel kernel;
  kernel.set_config(config);
  kernel.set_points(std::vector<gng_wasm_core::Point3f>(points.begin(), points.end()));
  if (!kernel.run()) {
    std::cerr << "基幹GNG学習コアの実行失敗\n";
    return 1;
  }
  if (kernel.nodes().size() < 2 || kernel.edges().empty()) {
    std::cerr << "基幹GNG学習コアのグラフ出力不足\n";
    return 2;
  }
  if (kernel.iteration() != config.iterations) {
    std::cerr << "基幹GNG学習回数の不一致\n";
    return 3;
  }

  std::cout << "nodes=" << kernel.nodes().size()
            << " edges=" << kernel.edges().size()
            << " iterations=" << kernel.iteration() << '\n';
  return 0;
}
