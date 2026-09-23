// 総当たりとの突き合わせテスト（追加・移動・削除を混ぜて kNN を比較）
#include <bsp3d/bsp3d.hpp>
#include <algorithm>
#include <cstdio>
#include <random>
#include <vector>

struct N { bsp3d::Point3 position; void *spatial_handle = nullptr; int index_in_cell = -1; bool in = false; };
static int failures = 0;
#define CHECK(c, m) do { if (!(c)) { std::printf("FAIL: %s\n", m); ++failures; } } while (0)

int main() {
  const int N_NODES = 2000;
  std::vector<N> nodes(N_NODES);
  bsp3d::Index<N> idx;
  std::mt19937 gen(7);
  std::uniform_real_distribution<float> u(0, 100);
  std::normal_distribution<float> g(0, 0.5f);
  auto rnd = [&] { return bsp3d::Point3(u(gen), u(gen), u(gen)); };

  for (int i = 0; i < N_NODES / 2; ++i) { nodes[i].position = rnd(); idx.add(&nodes[i]); nodes[i].in = true; }
  for (int op = 0; op < 40000; ++op) {
    int i = gen() % N_NODES;
    float r = u(gen) / 100.0f;
    if (r < 0.5f) { if (!nodes[i].in) continue;
      bsp3d::Point3 p = nodes[i].position; for (int d = 0; d < 3; ++d) p[d] += g(gen);
      idx.updatePosition(&nodes[i], p);
    } else if (r < 0.65f) { if (nodes[i].in) continue;
      nodes[i].position = rnd(); idx.add(&nodes[i]); nodes[i].in = true;
    } else if (r < 0.8f) { if (!nodes[i].in) continue;
      idx.remove(&nodes[i]); nodes[i].in = false;
    } else {
      bsp3d::Point3 q = rnd();
      std::vector<float> want;
      for (auto &n : nodes) if (n.in) want.push_back((n.position - q).squaredNorm());
      std::sort(want.begin(), want.end());
      std::array<bsp3d::SearchResult<N>, 64> res;
      for (int k : {1, 2, 8}) {
        int got = idx.findNBest(q, k, res);
        CHECK(got == (int)std::min<size_t>(k, want.size()), "件数が合わない");
        for (int j = 0; j < got; ++j)
          CHECK(std::abs(res[j].distance_sq - want[j]) <= 1e-3f * (1 + want[j]), "距離が総当たりと違う");
      }
    }
    if (op % 2000 == 0) CHECK(idx.checkInvariants().empty(), "不変条件が壊れた");
  }
  CHECK(idx.checkInvariants().empty(), "不変条件が壊れた");
  std::printf(failures ? "%d 件の失敗\n" : "ALL PASSED\n", failures);
  return failures ? 1 : 0;
}
