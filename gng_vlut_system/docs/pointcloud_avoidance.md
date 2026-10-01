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

実時間のRealSense点群による継続回避は[RealSense実点群を使うGazebo回避](realsense_gazebo.md)を参照。以下はシミュレーション時刻で既に配信されている外部点群の接続設定:

```yaml
overrides:
  pipeline:
    enable_lidar: false
    points_topic: /sensor/points
```

この場合、仮想LiDARとその固定TFの生成なし。外部配信側でPointCloud2のframeから`sim_<robot_name>/<root_link>`へのTF、シミュレーション時刻に整合する更新stampが必要。空点群・入力失効・探索失敗は停止扱い。

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
