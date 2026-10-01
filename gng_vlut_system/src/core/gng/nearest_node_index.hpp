#ifndef GNG_NEAREST_NODE_INDEX_HPP
#define GNG_NEAREST_NODE_INDEX_HPP

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace GNG {

// 学習ノードから独立した所有権を持つ動的近傍索引。
class nearest_node_index {
public:
  explicit nearest_node_index(int num_dims);
  ~nearest_node_index();
  nearest_node_index(const nearest_node_index &) = delete;
  nearest_node_index &operator=(const nearest_node_index &) = delete;

  bool is_supported() const;
  void clear();
  void set_point(int node_id, const float *values);
  void remove_point(int node_id);
  std::size_t size() const;

  // 従来の二乗距離と ID 順による上位候補。false は呼出元の全探索への切替条件。
  // calc_dist の前提: 登録座標と values の差の float 二乗和。
  bool query(const float *values, int num_candidates,
             const std::function<float(int)> &calc_dist,
             std::vector<std::pair<float, int>> &result) const;
  std::string check_invariants() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

} // namespace GNG

#endif // GNG_NEAREST_NODE_INDEX_HPP
