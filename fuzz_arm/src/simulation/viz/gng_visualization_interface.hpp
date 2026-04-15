#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace simulation {

/**
 * @brief 描画用ノード情報
 */
struct VisualNode {
  int id;
  Eigen::Vector3f position;
  int level;              // 階層レベル
  bool is_surface;        // 表面ノードフラグ
  bool is_active_surface; // 動的表面ノードフラグ
  bool active;            // アクティブフラグ
  bool is_collision = false;
  bool is_hazard = false;
  bool is_influence = false;
  bool is_path = false;

  // 追加の属性（色など）を拡張可能にするためのメタデータ
  struct {
    float r, g, b, a;
  } color = {0.8f, 0.8f, 0.8f, 1.0f};
};

/**
 * @brief 描画用エッジ情報
 */
struct VisualEdge {
  int node1_id;
  int node2_id;
  bool is_path; // パスの一部かどうかのフラグ
  int level;    // 階層レベル

  struct {
    float r, g, b, a;
  } color = {0.5f, 0.5f, 0.5f, 0.5f};
};

/**
 * @brief GNG可視化器の抽象インターフェース
 */
class IGngVisualizer {
public:
  virtual ~IGngVisualizer() = default;

  /**
   * @brief 描画用データの更新
   */
  virtual void update(const std::vector<VisualNode> &nodes,
                      const std::vector<VisualEdge> &edges) = 0;

  /**
   * @brief 描画実行（描画ループ内で呼ばれる）
   */
  virtual void draw(const Eigen::Vector3d &offset = Eigen::Vector3d::Zero()) = 0;

  /**
   * @brief 特定のパスを強調表示
   */
  virtual void setHighlightedPath(const std::vector<int> &path_node_ids) = 0;

  /**
   * @brief 表示フィルタの設定
   */
  struct FilterSettings {
    int max_level = 999;
    bool show_only_active = false;
    bool show_only_surface = false;
    bool show_only_active_surface = false;
    bool show_only_path = false;
  };
  virtual void setFilter(const FilterSettings &filter) = 0;
};

} // namespace simulation
