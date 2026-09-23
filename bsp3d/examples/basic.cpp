// 索引の基本操作: 追加・移動・最近傍探索・削除
#include <bsp3d/bsp3d.hpp>
#include <cstdio>
#include <random>
#include <vector>

struct MyNode {
  bsp3d::Point3 position;      // 必須
  void *spatial_handle = nullptr;  // 必須（索引が使う）
  int index_in_cell = -1;          // 必須（索引が使う）
  int id = 0;                      // 任意
};

int main() {
  bsp3d::Index<MyNode> index;            // 既定値は3次元向けに自動調整される
  std::vector<MyNode> nodes(10000);
  std::mt19937 gen(0);
  std::uniform_real_distribution<float> u(0, 100);
  for (int i = 0; i < (int)nodes.size(); ++i) {
    nodes[i].position = bsp3d::Point3(u(gen), u(gen), u(gen));
    nodes[i].id = i;
    index.add(&nodes[i]);
  }
  // 最近傍2点
  bsp3d::Point3 q(50, 50, 50);
  std::array<bsp3d::SearchResult<MyNode>, 64> res;
  int k = index.findNBest(q, 2, res);
  std::printf("最近傍 %d 件:", k);
  for (int i = 0; i < k; ++i)
    std::printf(" id=%d 距離=%.3f", res[i].element->id, std::sqrt(res[i].distance_sq));
  std::printf("\n");

  // 移動（座標は必ず updatePosition 経由で変える）
  for (int step = 0; step < 100000; ++step) {
    MyNode &n = nodes[gen() % nodes.size()];
    bsp3d::Point3 p = n.position;
    for (int d = 0; d < 3; ++d) p[d] += (u(gen) - 50) * 0.01f;
    index.updatePosition(&n, p);
  }
  k = index.findNBest(q, 2, res);
  std::printf("移動後の最近傍: id=%d 距離=%.3f / 要素数 %d\n",
              res[0].element->id, std::sqrt(res[0].distance_sq), index.getTotalNodes());

  index.remove(&nodes[0]);
  std::printf("削除後の要素数 %d\n", index.getTotalNodes());
  return 0;
}
