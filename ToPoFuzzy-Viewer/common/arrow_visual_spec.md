# 矢印表示の共通仕様

状態: 実装済み。WS v2、共通描画、ブラウザ設定、ROS Marker生成に適用。

## 目的と責務

把持候補、平面法線、進行方向、ROS Markerの矢印に共通する描画契約。
候補ID・形状スコア・到達状態などの意味情報と、色・寸法などの表示情報の分離。
各用途の変換処理による表示属性の決定と、共通描画部による同一仕様での描画。
矢印表示のためだけの専用ROSメッセージ追加は不要。

`arrows/geometry.ts`で端点・寸法を計算し、`ArrowBatch`の円柱・円錐インスタンスで描画。
未使用の`NormalVectorRenderer`と単体ラッパー`DirectionalArrow`は削除。
姿勢・始点終点の入力変換と設定UIの解釈は`arrows/marker_input.ts`へ集約。
ROS Markerも座標系・設定ごとの一括描画。
矢印・通常Markerの色変換は`marker_input.ts`の`marker_color`へ集約。
TF・手動変換・通常Markerの姿勢適用は`MarkerFrame`へ集約。姿勢のEuler角への変換と形状ごとの重複処理は不要。

## 共通フィールド

寸法はすべてm。方向ベクトルの大きさとは独立した表示寸法。

| フィールド | 意味・規約 |
| --- | --- |
| `id` | レイヤー内の表示ID。入力候補ID等との対応 |
| `frame_id` | 位置・姿勢の座標系 |
| `position` | `anchor`で指定した基準位置、`[x,y,z]` |
| `orientation` | ローカル座標系から`frame_id`へのクォータニオン、`[x,y,z,w]` |
| `primary_axis` | 主矢印のローカル正方向。`x` / `y` / `z` |
| `anchor` | `tail`=根元、`tip`=矢先、`center`=全長の中点。既定値`tail` |
| `length` | 矢先を含む主矢印の全長 |
| `shaft_diameter` | 軸の直径。ピクセル幅ではなく空間寸法 |
| `head_length` | 矢先の軸方向長さ |
| `head_diameter` | 矢先底面の直径 |
| `color` | sRGBの`#RRGGBB`指定。ROS等の入力色空間からの変換は入力アダプター側 |
| `opacity` | 0〜1の不透明度。主矢印全体への適用 |
| `is_visible` | 全体の表示状態 |
| `depth_mode` | `scene`=通常の奥行き判定、`overlay`=前面表示。既定値`scene` |
| `enable_transverse_axes` | 主軸に直交する残り2軸の正方向矢印の表示。既定値false |
| `transverse_axes` | 補助軸ごとの色・全長・軸直径・矢先長さ・矢先直径 |

`length`、`shaft_diameter`、`head_length`、`head_diameter`は正の有限値、
`head_length <= length`。クォータニオンは有限・非ゼロを確認後に正規化。
不正な寸法や姿勢は非表示。矢先長が全長を超える設定はUIに説明を表示。暗黙の長さ制限・任意の代替方向は不使用。

## 基準位置と補助軸

主軸の単位方向を`d`、指定位置を`p`、全長を`L`とした端点定義。

| `anchor` | 根元 | 矢先 |
| --- | --- | --- |
| `tail` | `p` | `p + L*d` |
| `tip` | `p - L*d` | `p` |
| `center` | `p - L*d/2` | `p + L*d/2` |

補助2軸は常に指定位置`p`を根元とする右手系。
`primary_axis=z`なら補助はローカル+X、+Y。クォータニオンによる回転後も直交性を維持。
主矢印の`anchor`変更による、補助軸原点や元の把持点の移動はなし。
補助軸の既定色はX=赤、Y=緑、Z=青。各軸で上書き可能。
各軸の寸法は共通の寸法規約に従い、透明度・奥行き設定は全体設定を共有。

方向ベクトルだけの入力では主軸まわりの回転が未定義。
その場合の補助軸は非表示とし、入力アダプターに完全な姿勢がある場合だけ表示可能。

## 把持候補への適用例

把持点を`position`、TCP姿勢を`orientation`として保持。
`primary_axis=z`、`anchor=tip`なら、下向きTCPに対して上方から把持点へ向かう矢印。
入力位置の移動や、矢印表示による把持候補の書換えはなし。
補助X・Y軸の追加により、主矢印だけでは区別できないグリッパの水平回転も表示可能。
到達状態の判定は候補生成側の責務。表示側は受信したstateと設定済みパレットを対応付け。
既定値は未評価=黄、範囲内=緑、範囲外=灰。`enable_state_colors=false`で固定色へ切替。
配色の正規定義は`arrow_visualization/state_colors.hpp`のsRGB値。ROS Markerへはlinear RGBに変換。
Viewerは同じ定義を`arrow_styles.candidate_state.state_colors`として受信し、独立した既定パレットを持たない。

## 入力・設定・送信頻度

| 入力 | 共通描画への変換 | 補助2軸 |
| --- | --- | --- |
| `GraspCandidateArray` | ID、位置、正規化姿勢、state。既定主軸+Z | 有効な姿勢の場合のみ |
| `PoseArray` | 配列添字、位置、正規化姿勢。既定主軸+Z | 有効な姿勢の場合のみ |
| ROS Markerのpose方式 | ローカル+X、scaleから全長・軸直径・矢先直径 | 有効な姿勢の場合のみ |
| ROS Markerの始点終点方式 | pointsから方向、scaleから軸直径・矢先直径・矢先長。poseは配置変換 | 不可 |
| グラフ法線・速度 | 位置・方向。元の大きさによる表示長は用途別の固定値 | 不可 |

Marker・姿勢・把持候補の表示設定はレイヤーID単位でブラウザの`localStorage`に保存。キーは`topofuzzy.arrow.v1:<layer>`。
共通の「矢印表示」パネルから編集。初期値・ROS Marker解釈も描画とUIで共用。
グラフのノード法線・速度とクラスタ詳細の法線は表示ON/OFFのみ。詳細設定GUIは非表示。
色・寸法は`arrows/geometry.ts`の共通値を使用。旧グラフ専用の色・倍率設定と、旧法線・速度のブラウザ保存値は参照対象外。
優先順位は「共通既定値 → 用途別／入力値 → ブラウザ上書き」。リセットで入力値へ復帰。
表示設定の操作によるROSメッセージ・候補座標の変更なし。

姿勢入力のWSエントリは`pos`、`orientation`、`state`等のデータ。候補には状態パレットの辞書参照を付与。色・寸法・矢先座標の毎回送信なし。
標準ROS Marker入力は`color`・`scale`を`arrow_styles`辞書へ集約し、各矢印には`arrow_style_id`だけを付与。
辞書は初回・変更時・再接続時に送信。省略時は前回の辞書を保持、空辞書は明示的な解除。
辞書参照が解決できない矢印は非表示。新しいROS型やスタイル変更RPCの追加なし。

ROS Marker自体の定義では色・寸法が必須のため、既存ROS MarkerのROS区間には設定値を含む。
通常の把持表示は`GraspCandidateArray`を直接購読し、この重複を回避。
`GraspCandidateArray`はID・形状スコア・到達状態を必要とする意味データとして維持。
単なる姿勢群には標準`PoseArray`、RViz互換出力には標準`MarkerArray`を使用。
旧`/grasp_pose_cand_scores`は削除し、`candidates[].shape_score`へ一本化。

## ROS送信側の共通実装

`arrow_visualization/include/arrow_visualization/arrow_marker.hpp`の`make_arrow`と`make_pose_arrows`を共用。
対象は平面法線、クラスタ速度、PoseArray／把持候補のRViz向けブリッジ。標準Markerの始点終点方式に統一。
無効方向・寸法・位置・色は同一IDのDELETE。座標基準と寸法は上記と同じ規約。
姿勢ブリッジは共通`make_pose_arrows`を呼び出すだけ。把持専用の軸生成コードは削除。
完全なクォータニオンから主軸と補助軸を生成し、無効姿勢では代替方向を生成しない。
候補生成側は既存の`grasp_candidate_publisher.hpp`による候補配列の共通配信を継続。

## Markerブリッジの起動

launchの未指定引数はノード既定値を使用。直接の`ros2 run`と同一設定。
`input_type=pose_array`（既定）は`/pose_array`・主軸−X、`grasp_candidates`は`/grasp_pose_cands`・主軸+Z。
全長0.12 m、軸直径0.006 m、矢先直径0.012 m、根元基準、補助軸あり。
入力型の自動判別はなし。`input_topic`は同じ型の別トピックへの切替用。

```bash
ros2 launch gng_vlut_system grasp_pose_marker_bridge.launch.py input_type:=grasp_candidates
ros2 run gng_vlut_system grasp_pose_marker_bridge_node --ros-args -p input_type:=grasp_candidates
```

`primary_axis_idx`（0=X、1=Y、2=Z）、`primary_axis_sign`、`anchor`、`head_length`、
`helper_axis_length_ratio`、`enable_transverse_axes`等はlaunchとrunの両方で指定可能。
設定は起動時に読込。候補IDは各軸namespaceの末尾に保持し、範囲外候補も配信。
候補状態の既定色は未評価=黄、範囲内=緑、範囲外=灰。
固定色を使う場合は`enable_state_colors=false`。空入力はDELETEALLで旧表示を消去。
入力QoSは送信元に追従。全送信元がreliableの場合のみreliable、全送信元がtransient_localの場合のみtransient_local。
未検出時はbest_effort・volatile。500 msごとの確認で、必要なQoSが変わった場合だけ再購読。
出力はreliable・transient_localの標準MarkerArray。ViewerのMarker・PoseArray・候補配列購読も同じ共通処理。
通常のViewer表示は候補トピックを直接購読可能で、このRViz向けブリッジの起動は不要。

単独`ais_gng_cpu/docker-compose.yaml`は`arrow_visualization`、`pointcloud_sampling`、`voxel_msgs`もマウント。
新環境では`colcon build --packages-up-to ais_gng --symlink-install`で依存を含めて構築。
Dockerfileの`cb`・`cbd`も依存を含めたビルドへ統一。変更の適用はイメージ再構築後。
Viewer単独イメージ・ソース配布には`gng_control_msgs`と`pointcloud_sampling`も同梱。

## 受け入れ条件

- `tail` / `tip` / `center`それぞれの端点一致と、TF適用後の一致。
- 主軸の選択、180度反転、クォータニオンによる補助軸の回転・直交性。
- 軸・矢先の実寸、色・透明度・奥行き設定の反映。
- 同一IDの設定変更、空配列、削除、TF欠落時の旧表示残留なし。
- 既存Marker・候補表示の移行確認と、大量矢印の描画コスト計測。
- Viewerのbackendビルド、frontend lint・buildの成功。

実施した検証と計測の範囲: [2026-09-12 共通矢印](../../gng_vlut_system/docs/releases/2026-09-12_common_arrow_visualization.md)。
