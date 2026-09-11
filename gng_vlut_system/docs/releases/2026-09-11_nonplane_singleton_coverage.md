# 2026-09-11 - 単独ノードを含む非平面成分の全保持

## Changed

- `extract_components(map, plane_clusters)`で全平面未所属ノードを抽出。平面ノードを経由する成分の統合なし。
- 孤立点、平面にだけ接続する単独点、接続消失で単独化した点にも成分IDを付与。
- CPU直結経路の`/nonplane_components`と`topological_map.nodes[].nonplane_component_id`へ同じ全成分を反映。
- 独立Marker経路の`surface_component.min_nodes`は表示専用。既定2を維持し、全成分抽出後に表示対象を選択。

- CPU直結の平面・非平面抽出が有効な同一フレームで、平面所属集合と非平面所属集合による全ノード被覆。
- GNG学習・平面クラスタの所属判定・曲面推定の方式に変更なし。
- 成分IDはフレーム内の識別用。単独成分の追加に伴う既存成分ID・表示色の変化の可能性。時系列の永続IDではない点に注意。
- Viewerが`/nonplane_components`から生成する表示には単独成分も到達。`surface_component.min_nodes`は独立Marker経路専用であり、Viewer転送経路への新しい表示フィルタの追加なし。

## Topics / Params / Messages

- トピック名とメッセージ形式の変更なし。C++の`extractor_options`は廃止、外部の直接呼出しは修正・再ビルドが必要。
- 通常YAMLとlaunchの`nonplane_component.min_component_nodes`指定は削除。
- 旧`nonplane_component.min_component_nodes`はCPUノードで読込互換のため宣言のみ保持。1以外の値は起動時警告、抽出結果への影響なし。
- `nonplane_component.direct_enabled`と独立Marker表示の`surface_component.min_nodes`は維持。

## Verification

非平面6件、平面・曲面の回帰テストを維持。専用ROSスクリプト129行は削除し、全被覆・重複・属性一致の検査を既存`boundary_candidates_ros_test.py`へ統合。
以下は既存Docker内。今回の整理では本体変更・再ビルドなし。

```bash
source /ros2_ws/install/setup.bash
timeout --signal=INT --kill-after=5s 60s ctest --test-dir /ros2_ws/build/ais_gng \
  -R '^test_(nonplane_component_extractor|plane_cluster_incremental|surface_model)$' --output-on-failure
timeout --signal=INT --kill-after=5s 90s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/boundary_candidates_ros_test.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --gateway-executable /ros2_ws/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node
```

統合テストはROS_DOMAIN_ID=189・ROS_LOCALHOST_ONLY=1でGNG・Viewerを起動。境界ON/OFF・次数上限4/0の3条件と、旧最小ノード数100000指定時の全被覆を検査。子プロセスの起動引数は実行ログへ出力。
単独成分は単体テストで確認。旧ROS検査は10フレーム成功、単独成分の出現0。画面表示・曲面所属と全被覆は別。
本体はビルド済み、次回通常launch起動から反映。旧検証GNG PID 73285は停止済み。
統合後は3条件で各8フレーム、計24フレームの全被覆確認に成功。既存の境界・Viewer転送・入力停止検査とC++回帰テストも成功。
検証用GNG・Viewer・driverは停止・回収済み。既存プロセスの停止・再起動なし。作業中に別途起動された通常GNG・Viewer・frontendは維持。
