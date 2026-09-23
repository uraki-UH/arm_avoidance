// この索引を使った Growing Neural Gas の学習
#include <bsp3d/bsp3d.hpp>
#include <chrono>
#include <cstdio>
#include <random>

int main() {
  bsp3d::GNGParams gp;
  gp.lambda = 100;
  gp.max_nodes = 20000;
  bsp3d::Point3 extents(200, 200, 200);        // MovingBSPTree では使われない（互換のため）
  bsp3d::GNG gng(extents, gp, bsp3d::Params{});

  std::mt19937 gen(1);
  std::uniform_real_distribution<float> u(0, 100);
  auto t0 = std::chrono::steady_clock::now();
  const int iters = 2000000;
  for (int i = 0; i < iters; ++i)
    gng.train_step(bsp3d::Point3(u(gen), u(gen), u(gen)));
  auto t1 = std::chrono::steady_clock::now();
  std::printf("ノード %d, 1反復 %.3f us\n", gng.getNodesCount(),
              std::chrono::duration<double, std::micro>(t1 - t0).count() / iters);
  return 0;
}
