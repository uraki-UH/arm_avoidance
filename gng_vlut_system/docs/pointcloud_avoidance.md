# 機体設定による共通Gazebo点群回避

固定基台のマニピュレータ向け構成。機体ごとのlaunch複製は不要。URDF・学習済みGNG/VLUT・計画関節を機体YAMLで選択。関節名のL/R接頭辞や片腕7関節への依存なし。

```text
Gazebo LiDAR → PointCloud2 → ROIボクセル → 自己形状除去
    → 占有・危険ボクセル → VLUT/GNG状態 → 経路探索・局所補正
    → 統合操作 → joint_trajectory_controller → Gazebo実測joint_states
```

## 起動と操作

```bash
docker exec -it gng_cpu_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
ros2 launch gng_vlut_system pointcloud_avoidance.launch.py
```

既定機体: ToPoDualArm。保存済みデータの対象は左腕7関節。右腕・胴体・指は実測姿勢の保持。起動後はホールド、Aで回避開始／ホールド復帰、Spaceで停止、停止後のLで解除後ホールド、Ctrl+Cで終了。実機UDP出力・USBドライバ起動なし。

Viewer併用時は`ROS_DOMAIN_ID`と`ROS_LOCALHOST_ONLY`を既存Viewerに合わせる。このPCの確認時設定はdomain 25、localhost限定なし。

機体切替例:

```bash
ros2 launch gng_vlut_system pointcloud_avoidance.launch.py \
  robot_config:=/ros2_ws/src/gng_vlut_system/config/pointcloud_avoidance_max.yaml
```

同梱設定: `pointcloud_avoidance_topodualarm.yaml`、`pointcloud_avoidance_max.yaml`、`pointcloud_avoidance_max_long.yaml`。max系は左右14関節分のGNG/VLUTが必要。このPCでは当該ファイル未配置のため、起動前検査で停止。max系の新しい共通構成での通し動作は未検証。

launch引数: `robot_config`、`gui`、`enable_keyboard`、`enable_viewer`、`gazebo_master_uri`。既定のGazeboポートは11355。GUI不要時は`gui:=false`、Viewer配信不要時は`enable_viewer:=false`。キー操作にはTTYが必要。

## 別機体の追加

1. 通常の機体パラメータYAMLを用意。`/**/ros__parameters`の`robot_name`、`urdf_path`、`mesh_root_dir`、`gng.data_directory`、`gng.experiment_id`、既存自己認識・モデル読込み設定を指定。URDFとデータのパスは起動環境内の絶対パス。
2. 対象URDF・関節順・座標系に一致するGNGとVLUTを配置。ファイル名は`gng.gng_model_filename`と`gng.vlut_filename`、省略時は`gng.bin`と`vlut.bin`。
3. 次の機体YAMLを追加し、`robot_config`で指定。設定ファイルの相対パスは機体YAMLのディレクトリ基準。

```yaml
common_config: pointcloud_avoidance_common.yaml
params_file: example_robot.yaml
planning_groups:
  - name: manipulator
    joint_names: [shoulder, elbow, wrist]
    link_names: [upper_arm, forearm, tool]
overrides:
  sides: [left]
  hand_y: 0.25
  hand_z: 0.35
  pipeline:
    roi_min: [-0.6, -0.9, 0.04]
    roi_max: [0.9, 0.9, 1.0]
    lidar:
      pose: [0.85, 0.0, 0.75, 0.0, 0.4, 3.141592653589793]
```

- `planning_groups`: 独立した可動グループ。グループを順につなげた`joint_names`が保存GNGの角度配列順。同じ関節や監視リンクのグループ間重複は拒否。fixed・mimic関節は計画関節に指定不可。
- `link_names`: 計画対象のリンク。指定リンクと計画関節の子リンクから、外装・指を含む全子孫を自動追加。その他の全身形状も自己干渉監視に使用。
- GNG検査: ファイルの存在・ヘッダー・関節数の一致。旧保存形式には関節名がなく、同じ関節数での順序違いは自動検出不可。URDF・関節順・VLUTの対応確認は設定作成時に必要。
- 基準座標: URDFの一意なルートリンク。world原点への固定基台。GNG/VLUTも同じ基準での生成が必要。
- `overrides`: 共通設定への辞書単位の上書き。模擬前腕の位置・寸法・接近時間、計画余裕、LiDAR位置・視野・解像度、ROIなどを機体寸法に合わせて指定。`sides`は障害物を置くY方向の符号で、関節グループ名とは独立。
- ROI: 指定範囲に追加余白なし。床は既定ROIの外側で、形状による床・作業台との干渉検査を併用。
- 退避条件: 点群の接近、または現在姿勢の最寄りGNGノード自身・直接隣接の危険／衝突。隣接安全を確認できない場合も保持・復帰への移行は禁止。グラフ側の危険に対する退避は計画対象関節全体で追従し、距離だけによる対象腕の縮小なし。各ステップの点群余裕増加は必須条件から除外、経路の最低余裕・自己干渉・関節制限の検査は継続。
- 自律退避先: 自身と辺で直接つながる全隣接ノードが安全な候補へ限定。二次隣接は対象外。C++では安全条件で絞ってから候補数を制限し、移動中の退避先の隣接悪化は再選定。Pythonグラフ探索でも終点へ同条件を適用。安全な終点への脱出経路を確保するため、中間ノードには自身の安全を要求し、隣接までの安全は要求しない。
- 回避・復帰のチャタリング抑制: 共通YAMLの`return_clear_sec: 0.5`で復帰条件の継続時間を指定（実時間の秒、0で待機無効）。C++計画接続では隣接安全・退避目標余裕・復帰経路の安全確認中は`waiting_for_clearance`で保持、継続成立後に復帰。Python単独計画にも既存復帰条件の継続確認を適用。危険再検出時の回避は待機なし。条件不成立・確認間隔の入力期限超過・停止・再開始で計測を初期化。`obstacle_wait`からの再開確認`resume_clear_sec`とは別設定。設定反映はGazebo launch再起動後。
- 停止マージンの正本: `config/pointcloud_avoidance_common.yaml`の`clearance_margins`。`min_clearance_th`は点群の開始・停止・待機・経路検査・QP下限（0.01 m）、`min_internal_clearance_th`は自己干渉・床・作業台の開始・停止余裕（0.005 m）、`min_planning_clearance_th`は計画・QP時の内部形状余裕（0.01 m）。計画余裕は内部停止余裕を確保する値、各値は有限正数。点群の経路下限`min_cloud_clearance_th`は自動導出、独立設定なし。`target_clearance`は退避目標・自動再開条件として別設定。
- 設定移行: 新共通設定で旧トップレベルの距離項目を併記すると起動拒否。機体・入力固有の上書きも`clearance_margins`内に記載。共通launch以外の旧デモ設定は従来互換。変更反映はGazebo launch再起動後、稼働中への動的反映なし。
- 点群経路検査: 指定の計画余裕と`min_clearance_th`の大きい方を区間サンプルの下限として使用。距離は外接球とボクセル外接球の間の保守的な余裕で、実物表面間の測距値とは別。距離不足による開始拒否・停止時は`avoidance/status.stop_clearance`へ事象・リンク・URDFルート座標の最接近点・距離・球半径・関節位置を保存。後続入力で上書きせず、次の回避開始成功時に初期化。開始拒否の事象は`start_rejected`、実行中の距離停止は`running_stop`。
- 点群待機の自動再開: `enable_live_obstacles`と`enable_obstacle_auto_resume`の両方が有効な場合、点群距離不足を`obstacle_wait`で保持。`target_clearance`とGNG隣接安全の`resume_clear_sec`秒継続後に同じ実行を再開。共通既定はOFF、通常Viewerの実環境入力設定でON。手動停止・入力欠測・内部干渉・関節異常の解除は対象外。自己除去後ROIが空の場合も既存の入力待ち・欠測停止扱いを維持。

実時間のRealSense点群による継続回避は[RealSense実点群を使うGazebo回避](realsense_gazebo.md)を参照。以下はシミュレーション時刻で既に配信されている外部点群の接続設定:

```yaml
overrides:
  pipeline:
    enable_lidar: false
    points_topic: /sensor/points
```

この場合、仮想LiDARとその固定TFの生成なし。外部配信側でPointCloud2のframeから`sim_<robot_name>/<root_link>`へのTF、シミュレーション時刻に整合する更新stampが必要。空点群・入力失効・探索失敗は停止扱い。

## 局所QPによる出力補正（試験機能）

実装: OSQP 1.0.4。GNG/V-LUTまたは既存C++の経路・目標選択を維持し、Gazebo軌道の出力直前に関節変位を補正。実機出力の許可設定・電流制限とは別機能。

設定先: `pointcloud_avoidance_common.yaml`の`local_qp`。既定OFF。機体YAMLの`overrides.local_qp`または入力YAML直下の`local_qp`で上書き可能。通常のlaunch引数追加なし。

```yaml
local_qp:
  enable_qp: true
  max_joint_acceleration: 3.0
  max_solve_sec: 0.02
```

上記はGazeboでの比較用初期値。加速度単位は回転関節rad/s²・直動関節m/s²、時間単位はs。古いコンテナは`python3 -m pip install osqp==1.0.4`と`gng_vlut_system`の再ビルドが必要。Dockerfileに同版を追加済み。

- 変数: 選択された計画関節の変位`Δq`。他の関節は実測位置のまま固定。
- 目的関数: `0.5 ||Δq − Δq_nom||²`。候補待ちで名目変位がゼロの場合のみ、近傍点群から目標余裕へ離れる項`100 ||J_cloud Δq − (target_clearance − d_cloud)||²`を追加。
- 距離制約: `J Δq ≥ −0.5 (d − d_min)`。点群との外接球表面間距離、既存の自己干渉ペア、床・作業台を対象。点群の`d_min`は`max(min_cloud_clearance_th, min_clearance_th)`、内部形状は`min_planning_clearance_th`。
- 関節制約: URDF範囲内の次姿勢。5次補間の速度・加速度最大値から`|Δq| ≤ min(v_max T / 1.875, a_max T² / (10/√3))`。`v_max`は既存速度設定とURDF上限の小さい方、`T`は既存制御周期。
- 軽量化: 球中心の前進差分と最近傍面法線による距離勾配。変位上限だけで満足する線形制約の除外。距離制約数の上限打切りなし。
- 棄却: 初期余裕不足、非有限値、求解失敗・不正確解・時間超過、制約残差超過、既存の非線形区間検査失敗、求解後の入力失効、処理全体の制御周期超過。元の未補正指令へのフォールバックなし、既存の停止・保持処理へ移行。
- 診断: `avoidance/gng_status.local_qp`の`status`・`num_calls`・`num_constraints`・`solve_ms`・`total_ms`。`solve_ms`はOSQPのsetup・solve時間、`total_ms`は距離勾配・区間検査を含む処理時間。
- 制限: 動く障害物の未来位置推定なし。距離の局所線形化と離散的な区間検査であり、連続時間の衝突回避保証・未知領域の安全保証なし。加速度制約は生成した静止端点間軌道に対する値で、実機追従・再計画時の加速度保証ではない。短い周期では許容変位が小さくなるため、速度設定だけを上げても高速化しない。

参考: [OSQP Python API](https://osqp.org/docs/interfaces/python.html)、[求解状態](https://osqp.org/docs/interfaces/status_values.html)。

## 適用範囲と制限

- 物理構成: Gazebo Classic、固定基台、有限力／トルクの関節制御。URDFの慣性・関節上限・effort・velocity設定が必要。移動台車・歩行ロボット・浮遊基台への対応なし。
- 回避幾何: STLメッシュ、box、sphere、cylinderの保守的な外接球列。STLはURDFからの相対パスまたは絶対パス。床z=0と同梱worldの作業台に対応。任意worldへの自動適応・厳密なメッシュ衝突保証なし。
- 初期姿勢: 独立関節0。非ゼロ初期姿勢を必須とする機体、異なる台配置、直動関節を計画に含む場合の速度・補間単位の個別調整は未対応。直動グリッパーの保持・停止は対応。
- 通し検証: ToPoDualArmの左腕。その他の機体は設定・URDF・学習データの整合と実際の回避・停止の確認が必要。設定追加だけで任意機体の動作保証にはならない。
- 表示: Gazebo GUIと既存Viewerへのロボット姿勢配信。Viewerサーバーの自動起動なし。

## 再現試験

コンテナ内、ROS環境読込み後の実行。試験専用domain 96、Gazeboポート11369、出力先は未使用ディレクトリ。

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_pointcloud_avoidance.py \
  --output /ros2_ws/src/artifacts/common_pointcloud_trial
```

試験内容: 実レイ点群の受信・自己除去後ボクセル・GNG経路利用・退避復帰・入力欠測停止・実測静止・所有プロセス回収。試験ハーネスの対象はToPoDualArm。結果と失敗記録は[リリースノート](releases/2026-10-01_common_pointcloud_avoidance.md)を参照。
