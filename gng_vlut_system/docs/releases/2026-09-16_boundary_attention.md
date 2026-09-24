# 2026-09-16 - 境界ノード周辺への重点学習

## 1. 要約

前フレームの低次数境界ノード周辺へのガウス距離重み付き重点学習を追加。[仕様](../../../ais_gng_cpu/docs/boundary_attention.md)。

`graspnet.yaml`で境界重点を有効化。対象半径0.03 m、配分率0.2、有効期間0.5 s。
既存の把持重点と正規化重みを混合し、通常方式の学習枠を維持。
ユーザー編集の`node.interval`および作業中の支持統計設定変更を維持。

重み付き重点入力API、境界最近傍検索、失効条件、配分混合、回帰テスト。

新たな重み付き経路でも、入力範囲フィルタ後の添字と重みの対応を維持。

**削除**

既存機能・設定の削除なし。

## 2. 条件・検証

重点学習は位置・エッジを更新するが観測統計へ非計上。境界候補の数・点群分布により追加処理時間が変化。
稼働中GNGへの自動反映なし。更新済みバイナリとYAMLを使う再起動が必要。

新トピック・メッセージなし。起動時パラメータ`enable_boundary_attention`、`boundary_attention.radius`、`boundary_attention.ratio`、`boundary_attention.timeout_sec`を追加。
CPU API `gng_set_weighted_priority_input`を追加。GPU・WASMへの追加なし。

既存コンテナ内、`/ros2_ws`で実行:

```bash
source /ros2_ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select gng_cpu ais_gng --executor sequential --cmake-args -DBUILD_TESTING=ON -DGNG_BUILD_BENCHMARKS=ON
ctest --test-dir /ros2_ws/build/gng_cpu -R priority_input --output-on-failure
ctest --test-dir /ros2_ws/build/ais_gng -R 'test_(boundary_attention|grasp_attention)$' --output-on-failure
/ros2_ws/build/ais_gng/test_boundary_attention
```

- 初回全体ビルドは非平面成分抽出のリンクエラーで失敗。同箇所のソース変更なしの再ビルドで両パッケージ成功。初回失敗の原因は未確定。
- 境界単体6件成功：距離減衰・範囲・非有限値、最近傍と総当たりの一致、重複境界、把持重点との重複・片方失効、時刻・frame条件。
- 既存把持重点テストと重点APIテスト成功。重み付きAPIの通常学習枠、無効値、次回への失効を確認。
- 合成10万点・境界2000点の選択処理は、kd-tree版の1回測定で25.7 ms。初期の格子＋順序付き連想配列版は247 ms。実入力のフレーム時間ではない。
- インストール済みYAMLの追加4設定と共有ライブラリの公開シンボルを確認。`git diff --check`成功。
- すべて有限のビルド・テストコマンドは終了済み。ROSノード・再生プロセスの新規起動／既存プロセスの停止なし。

**制約**

境界候補は物体境界の確定情報ではない。重点によるノイズ強調や前フレーム位置のずれの可能性あり。
実点群での小物分離改善、最適半径・配分率、全体実行時間は未検証。
