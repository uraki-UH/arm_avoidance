# 2026-09-02 - 上方向把持の平面クラスタ入力既定値

## 変更

- `top_grasp_surface_estimator.launch.py`の平面クラスタ入力既定値を、廃止済みの`/topological_planar_clusters_incremental`から`/plane_clusters`へ変更

## 影響

- CPU版`ais_gng.launch.py`と上方向把持候補のlaunchを、追加引数なしで接続
- `/top_grasp_pose_cands`、`/top_grasp_pose_cand_scores`、`/top_grasp_pose_cands/summary`への候補判定結果の配信
