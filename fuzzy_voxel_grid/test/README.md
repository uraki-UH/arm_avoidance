# 共通点群経路の回帰検証

## 要約

2026-09-26、既存`gng_cpu_container`内のROS 2 Humble・Releaseで検証。
共有経路は[パッケージ仕様](../README.md)のビルドを使用。

- 共通ストアのgtest 7件：channel・索引・元属性の同一ポインタ、二重writer拒否、
  スナップショット寿命・交換・破棄、負座標・非有限点・空フレーム。
  dense/hash/独立計算の10万点一致、境界・不正条件、同設定の同時読取の結果共用、
  旧集計の不変性、別設定の分離、広域・狭域検索の点集合と統計値も検証。
- 既存`test_reachability_voxel_accumulator`の21件：ROI・world bucket・
  persistent depth・直接voxel化の回帰テストを再実行。
- ROS実起動：worldとFVGを別コンポーネントとして同一プロセスへロード。
  入力点群購読1件、FVG独立購読0件、共有先でのセル件数を確認。
  2つ目のFVGの実ロード後も購読数1件、同一セル出力を確認。
- 独立計算と照合：センサー→worldとworld→ROIの異なる回転・並進、
  world幅0.2 m、ROI幅0.05 m、FVG幅(0.1, 0.2, 0.15) m、非零原点。
- 重複点、NaN、ROI外・FVG除外箱、空入力、時刻巻戻し、
  Tmapのframe不一致・古過ぎる時刻・未来時刻を検証。
- TF未接続フレームの拒否と、正しいTF入力へ戻した場合の復帰を確認。
- freeze/resume：world座標による除外と、出力の元センサー座標・intensityバイト保持。
- 100,000点／空入力の交互16フレーム：全セル件数を照合、旧フレームの混在なし。
- 動く10万点の4フレーム、Tmap単独セルの追加・更新・消失、狭い範囲のbucket検索を照合。
- 共通化前・隔離ビルド・通常配布先のdense/hashでセル出力を全フィールド比較し一致。
  Markerの位置・色・scale・ID・配列順も一致。セル・Marker内点の並びだけ正規化。
- 共有ノード終了後、FVG単独とworld単独を順番に起動して互換性を確認。

## 条件・検証

通常のROS環境を変更しないため`ROS_DOMAIN_ID=181`・localhost限定。
スクリプトはdomainを検査し、起動したプロセスグループだけをfinallyで終了。
BestEffortの初回DDS接続中の欠落に対して、試験入力だけを0.2秒間隔で再送。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /ros2_ws/install/fuzzy_voxel_grid/share/fuzzy_voxel_grid/local_setup.bash
ctest --test-dir /ros2_ws/build/voxel_idx --output-on-failure
ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure \
  -R test_reachability_voxel_accumulator
cd /ros2_ws/src
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 timeout -s INT -k20 180 \
python3 fuzzy_voxel_grid/test/test_shared_world_ros.py
```

同じコマンドに`FVG_TEST_DENSE_LIMIT=0`を付けてhash経路も照合。
`FVG_TEST_OUTPUT`で出力先を分離し、各`snapshots.json`を`cmp`で変更前と比較。

内部起動コマンドはスクリプトの`START`出力へ全引数を記録。
対象は`ros2 launch fuzzy_voxel_grid shared_world_voxel.launch.py`、
`voxel_grid_node --ros-args --params-file ...`、
`world_index_to_voxel_node --ros-args --params-file ...`。
最終試行は4プロセスグループとも終了コード0、共有コンテナの子プロセスも終了。
既存bag・Viewer・Dockerコンテナは停止・再起動していない。

今回のローカル出力：`artifacts/shared_voxel_common_20260926/`（Git対象外）。
`installed_build.log`、`installed_test.log`、`installed_sparse_test.log`。
単体テストの詳細は各buildディレクトリの`test_results`・`Testing`。
比較用出力は`baseline_test`・`common_test`・`sparse_test`・
`installed_test`・`installed_sparse_test`内の`snapshots.json`。
別prefixはその`local_setup.bash`をsourceし、`FVG_TEST_PREFIX`で単独実行先を指定。
`FVG_TEST_OUTPUT`で試験YAML・ログ・比較用出力の保存先を分離可能。
初回ビルドのamentリンク記法・Humble Time API差、実起動でのコンポーネント登録スコープ・
単独実行のライブラリ探索を修正後に全経路を再検証。
初回単独試験のDDS接続中の単発入力欠落は試験側の再送で解消。
今回の隔離ビルドは旧共有ライブラリの参照で初回失敗。
`voxel_idx_DIR`と`LD_LIBRARY_PATH`を検証対象prefixへ揃え、通常配布先でも再検証。

この検証は機能・所有権・更新の回帰。性能は[測定記録](../../benchmarks/shared_voxel_cost_20260926/README.md)へ分離。
実bag認識品質・Viewer実描画は未検証。GNG本体の学習実装は変更対象外。
