# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
cd uraki_ws
docker compose down && docker compose up --build  && docker compose up -d
docker compose exec gng_cpu bash
## frontendの起動
cd ~/uraki_ws
docker compose exec gng_cpu bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'

docker compose --profile manual up  frontend

chrome://restart

##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

## ロボットを座標変換
python3 test_tf_publisher.py --world-frame world --frame-id ToPoDualArm/base_link --x 0.35 --y 0.15 --z -0.3 --yaw 3.2


## ダミー把持姿勢をPoseArrayで流す
ros2 launch gng_vlut_system grasp_pose_dummy_publisher.launch.py \
  frame_id:=world \
  candidate_count:=1

## 把持候補姿勢、軌道、動く
ros2 launch gng_vlut_system grasp_goal_planning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  enable_motion:=false


## HTML起動
python3 -m http.server 8000
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html


## 点群から占有ボクセルに変換
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  input_topic:=/semantic_points \
  output_topic:=/topo_voxel_ids

### `/dataset/points`をToPoDualArmのVLUTへ反映

`/dataset/points`のheader frameは`world`とし、点群のframeを強制的に
`ToPoDualArm/base_link`として扱わない。点群時刻のTF
`world`→`ToPoDualArm/base_link`でロボット座標系へ変換した後、2 cm voxel IDを
VLUT入力へ渡す構成。

```bash
# ROI voxel化、world index、occupied/danger更新の起動
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

このlaunchはViewer gatewayやロボットを起動しない。先に別terminalで
`gng_viewer_bridge.launch.py`を起動した状態で使用する。既定で`ToPoDualArm.yaml`からVLUT fileを特定し、`vlut.bin` headerの
`voxel_size`をROI voxel化と`occupied_voxels`出力の両方へ自動適用する。したがって、
通常は`voxel_size`や`output_voxel_size`を指定しない。別のrobot設定では、同じrobotの
`params_file`を指定する。headerを持たない旧VLUTだけは、起動ログを警告して手動設定値の
`0.02 m`へフォールバックする。

`environment_voxelization.world_index`では、index構築とROI抽出経路を別々に選択できる。

| `enable_build` | `enable_roi_query` | 動作 |
| --- | --- | --- |
| `false` | `false` | indexを構築せず、各robot座標系へ直接ROI voxel化 |
| `true` | `false` | world indexはViewer確認用に構築し、ROI voxel化は直接方式 |
| `true` | `true` | world indexを構築し、bucket AABB抽出後にROI voxel化 |

`false`と`true`の組合せは不正として起動を停止する。`enable_bucket_publish`はworld bucketの
Viewer出力だけを制御する。`enable_build`が`false`ならbucket出力も無効になる。旧設定の
`world_index.enable`は、新しい2項目が未指定の場合だけ両方の既定値として扱う後方互換設定である。
`ToPoDualArm.yaml`の既定値は`enable_build: true`、`enable_roi_query: true`、`0.2 m` bucketである。

### ToPo Fuzzy Viewerでのworld index確認

Viewer gatewayを別途起動後、Viewerのtopic一覧から次の`voxel_msgs/Voxel`を有効にする。

- `/ToPoDualArm/roi_voxel_ids`: `ToPoDualArm/base_link`座標系のVLUT入力ROI voxel
- `/ToPoDualArm/world_index_buckets`: `world`座標系の非空world bucket voxel

どちらもreliable/transient-localでpublishするため、Viewerを後から接続しても直近の状態を
受信する。`world index更新:`ログの`roi_voxels`と`world_buckets`が0以外であること、Viewer側で
両topicのレイヤーを有効化すること、`world`から`ToPoDualArm/base_link`へのTFが存在することが
可視化の確認条件である。

`environment_voxelization.base_frame`と`robot_name`から、入力点群の変換先
`ToPoDualArm/base_link`を自動解決する。`source_frame_id`が空なら、入力点群headerの
frameを使用する。移動マニピュレータは自己位置推定またはSLAMが毎時刻の
`world`→`ToPoDualArm/base_link`をpublishするため、YAMLの`enable_static_tf`は`false`のままにする。
固定設置だけは同YAMLの`enable_static_tf: true`と`static_tf_*`へ外部キャリブレーション値を設定する。
既存のlocalization TFがある場合に静的TFを追加してはならない。

### 複数robotの共有world index

`environment_voxelization.world_index.consumers`に各robotの`params_file`、`robot_name`、
`voxel_topic`を列挙すると、`environment_to_vlut.launch.py`がworld bucket索引を1回だけ構築し、
各robotのTF・ROI・VLUT header解像度で個別にROI voxelを出力する。
`config/shared_world_index.yaml`は同一URDFを2台使う動作例であり、実機では各要素の`params_file`を
各robotの設定へ置き換える。

```bash
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/shared_world_index.yaml
```

robot本体とViewer gatewayは各robotごとに別terminalで起動する。共有world index launchはそれらを
起動しないため、既存の`gng_viewer_bridge.launch.py`とnode名・topic名が重複しない。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm_A
```

2台目以降も、そのrobotの`params_file`と`robot_name`で同様に起動する。

`/ToPoDualArm/occupied_voxels`と`/ToPoDualArm/danger_voxels`はfull snapshotとしてpublishする。
TopoFuzzyの`SafetyVlutMapper`は前回snapshotとの差分だけをVLUT node countへ加減算するため、
追加・削除ボクセルのための別launchは不要。full snapshotにより、点群から消えた占有も安全に解除する。

danger判定方式は`ToPoDualArm.yaml`の`environment_voxelization.danger_source`で選択する。

- `environment_inflation`: 現在の既定値。`danger_inflation`だけ環境voxelを膨張してdangerへ送る方式
- `vlut_distance`: 環境側膨張を0にし、VLUT relationの距離値を`vlut_danger_dist`で判定する方式

現行の`ToPoDualArm10000/vlut.bin`はrelation距離が全て0のため、`vlut_distance`には切り替えない。
距離付きVLUTを生成した後にだけ、次の設定へ変更する。

```yaml
environment_voxelization:
  danger_source: "vlut_distance"
  vlut_danger_dist: 0.025
```

起動後は、`/dataset/points`、`/ToPoDualArm/roi_voxel_ids`、
`/ToPoDualArm/occupied_voxels`が順に更新されることを確認する。

```bash
ros2 topic hz /dataset/points
ros2 topic echo --once /ToPoDualArm/roi_voxel_ids
ros2 topic echo --once /ToPoDualArm/occupied_voxels
```

既定では`ToPoDualArm/base_link`へ点群を変換し、同座標系のreachability範囲
`x=[-0.1, 0.5] m`、`y=[-1.0, 1.0] m`、`z=[-1.0, 1.0] m`だけを
各軸0.2 m拡張した領域を逐次ボクセル化する。別ロボットでは次の引数をrobot configの
TCPサンプリング範囲へ合わせる。

```bash
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  target_frame_id:=<robot_base_frame> \
  min_reachability_x:=<min_x> max_reachability_x:=<max_x> \
  min_reachability_y:=<min_y> max_reachability_y:=<max_y> \
  min_reachability_z:=<min_z> max_reachability_z:=<max_z> \
  reachability_margin_x:=<margin_x> \
  reachability_margin_y:=<margin_y> \
  reachability_margin_z:=<margin_z>
```

marginには把持物の最大張り出し、位置推定誤差、安全余裕を含める。移動マニピュレータでは、
ボクセルを保持したい移動範囲を`reachability_margin_x`と`reachability_margin_y`へ加える。
領域は各点群時刻のTFで現在のロボット基準座標系へ追従するため、marginには先読みする
台車移動範囲を指定する。不要な高さ方向のボクセル増加を避けるため、各軸を個別指定する。

全点を従来どおりボクセル化する場合は`enable_reachability_filter:=false`を指定する。
dense bitmapは既定で最大8,000,000 voxelとし、超える範囲では再利用hashへ自動fallbackする。
メモリ上限を調整する場合は`max_dense_voxel_num`を指定する。

### 複数ロボット共有world bucketの比較

`world_bucket_benchmark_node`は入力点群から0.2 mのworld bucketを1回構築し、1、2、4、8台分の
直接走査とbucket抽出を比較する。各robot座標系で2 cm voxel IDまで生成し、ID不一致数も確認後に
`frame_num`で自動終了する。

```bash
ros2 run gng_vlut_system world_bucket_benchmark_node --ros-args \
  -p input_topic:=/camera/camera/depth/color/points \
  -p frame_num:=30 \
  -p bucket_size:=0.2 \
  -p parallel_thread_num:=8
```

出力の`bucket_total`には全ロボットで共有するbucket構築時間を含む。`mismatch_num=0`が
直接方式とVLUT IDが一致した状態。`PARALLEL_RESULT`は8台分をrobot単位で並列化したwall time。
比較用ノードのためvoxel topicはpublishしない。

局所chunk相当のROI感度試験例：

```bash
ros2 run gng_vlut_system world_bucket_benchmark_node --ros-args \
  -p frame_num:=30 -p bucket_size:=0.2 \
  -p robot_spacing_x:=0.0 -p robot_yaw_step_deg:=0.0 \
  -p min_reachability_x:=-0.05 -p max_reachability_x:=0.25 \
  -p min_reachability_y:=-0.15 -p max_reachability_y:=0.15 \
  -p min_reachability_z:=-0.15 -p max_reachability_z:=0.15 \
  -p reachability_margin_x:=0.03 \
  -p reachability_margin_y:=0.03 \
  -p reachability_margin_z:=0.03
```

world indexを複数フレーム再利用した場合のstale差分評価：

```bash
ros2 run gng_vlut_system world_bucket_benchmark_node --ros-args \
  -p index_refresh_frame_num:=2 \
  -p depth_topic:=/camera/camera/depth/image_rect_raw
```

`REUSE_RESULT`の`recall`は現在フレームに存在するoccupied IDの保持率、`precision`は再利用indexの
occupied IDのうち現在フレームにも存在する割合。安全用途ではfalse negativeを許容しないため、
単純再利用ではなくdepth差分による新規点の即時overlayが必要。

### depth画素handle付きpersistent world indexの比較

`depth_world_index_benchmark_node`は固定解像度のraw depth画素を安定handleにし、world bucket内の
点だけを差分更新する。各フレームのdepth全画素を読むが、複数robotのためのworld index構築は1回だけで、
各robotには局所AABB query後の点だけをVLUT IDへ変換する。debug有効時だけ、確認用voxel topicもpublishする。

camera-to-worldが固定である前提のため、実機では`camera_world_*`に固定外部パラメータを指定する。
camera姿勢、camera_infoの画像寸法、intrinsicsの変更時は安全側でworld indexを全再構築する。

```bash
# ターミナル1: persistent indexの実行設定
ros2 run gng_vlut_system depth_world_index_benchmark_node --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/depth_world_index_benchmark.yaml

# ターミナル2: raw depthとcamera_infoを同時に配信
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41 \
  --topics \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info
```

YAML既定値では、毎フレームの`direct8`全再構築比較を実行しない。`robot_num:=1`のpersistent queryと
debug voxel出力だけのため、Viewer確認時に基準方式のCPU負荷を加えない。比較が必要なときだけ
`enable_comparison_benchmark:=true`を指定する。`depth_update_mm_th:=1`と`free_confirmation_num:=1`では、
比較有効時の`mismatch_num=0`、`recall=1`、`precision=1`が毎フレーム全再構築と同じVLUT IDの条件となる。`free_confirmation_num:=3`は
depth値が0になった画素を3フレーム残す保守設定であり、false negativeを抑える代わりに一時的な
false positiveを許容する。valid depthが奥へ移る変化は、現観測を優先して即時更新する。

常駐時の毎フレーム計測ログは既定で有効である。処理時間、world bucket数、ROI voxel数を出力し、
ログを抑止する場合だけ`enable_runtime_log:=false`を指定する。

YAMLのROIは今回の狭い局所chunkの実測条件である。`robot_spacing_x:=0.0`と
`robot_yaw_step_deg:=0.0`はworld indexの共有費用だけを分離する比較条件であり、実機のrobot配置には
各robotのworld poseを使う。実機の固定cameraでは同YAMLの`camera_world_*`を外部キャリブレーション値へ変更する。

#### ROI voxelとworld indexの視覚確認

YAMLの既定値ではdebug publishが有効で、`frame_num:=0`のためCtrl-Cまで継続する。ToPo Fuzzy Viewerでは次の2 topicだけを有効化する。

- `/depth_world_index/debug/roi_voxels` (`voxel_msgs/Voxel`): ROIで採用された2 cm voxel。VLUT入力と同一形式
- `/depth_world_index/debug/world_buckets_voxels` (`voxel_msgs/Voxel`): 非空world bucketを`bucket_size=0.2 m`のvoxelとして出力。world indexの格子確認用

camera-to-worldをidentityにしたraw depth rosbagでは、両topicの`frame_id`は`camera_depth_optical_frame`になる。
実機設定では、`camera_world_*`でworld座標系へ変換し、同時に`debug_frame_id`をそのworld frameへ指定する。

##　ボクセルからGNGのoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system voxel_to_vlut.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/topo_voxel_ids \
  danger_inflation:=0.08

## AISGNG実行
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=d435.yaml
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=topo_points.yaml

ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=graspnet.yaml


## RVizでロボットを表示
ros2 launch gng_vlut_system visualize_robot_rviz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm


==============================================================

## 把持ノードから接続構造を追って、指定距離以内のノードを抽出する (今の所未使用)
ros2 launch ais_gng topological_query.launch.py \
  input_topic:=/topological_map/merged \
  relation_mode:=graph_edges \
  semantic_label:=1 \
  max_euclidean_distance:=0.5 \
  max_hops:=-1

## グリッド所属を半セルずらしで出す（未使用）
ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  output_topic:=/topological_grid_voxels_shifted \
  grid_size:=0.01 \
  origin_shift_half:=true

##　dynamixel handlerの起動（使えない可能性が高い）
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
USB の番号が変わる環境では、こちらのラッパーの方が安定。
ros2 launch topoarm_bringup dynamixel_handler_auto.launch.py

##　dynamixelの/dynamixel/state/present　トピックをjoint_statesに変換
ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm

ros2 launch dynamixel_joint_state_bridge \
  dynamixel_joint_state_bridge.launch.py \
  namespace:=/ToPoDualArm

realsense
ros2 run dynamixel_joint_state_bridge dynamixel_joint_state_bridge_node \
  --ros-args \
  -r __ns:=/ToPoDualArm \
  --params-file /ros2_ws/src/dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge.yaml \
  -p output_topic:=viewer_joint_states

##　自己認識ボクセルの起動
ros2 launch gng_vlut_system self_recognition_viz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  root_link:=right_link2 \
  leaf_link:=right_link8 \
  mask_topic:=/ToPoDualArm/right_arm_voxel

##　自己認識ボクセルをoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system voxel_to_vlut.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/ToPoDualArm/right_arm_voxel \
  danger_inflation:=0.05
  (output_voxel_size:=0.02)

## 自己認識ボクセル内外の点群に分けてパブリッシュ
ros2 run gng_vlut_system self_recognition_filter_node
  # self-recognition voxel内の点群: /self_recognition_points
  # self-recognition voxel外の点群: /self_filtered_points


# joint_statesのダミー
(topoarmの場合)
python3 dummy_joint_pub.py --robot topoarm

## realsenseのrosbag + 点群座標変換
# bagのraw点群は内部トピックへ退避し、変換後の点群が元のトピック名を引き継ぐ。
# 入出力を同じトピックにすると変換ノードが自己購読するため、直接同名にはしない。

# ターミナル1: raw点群を /camera/camera/depth/color/points_raw へリマップして再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics /camera/camera/depth/color/points \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw \
  --loop

# ターミナル2: 変換後の点群を元のトピック名でpublish
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py \
  input_topic:=/camera/camera/depth/color/points_raw \
  output_topic:=/camera/camera/depth/color/points

### 深度画像ベースの動体ノード削除判定ベンチ

`/camera/camera/depth/image_rect_raw` を直接読むため、点群から深度バッファを再構築しない。
GNG は従来どおり変換済み点群を入力にする。ベンチマークは map と同時刻の深度画像を30フレーム照合し、
等倍・実行時1/2・実行時1/4 min-pooling を比較して自動終了する。

```bash
# ターミナル1: 生点群は変換用に退避し、深度画像と内部パラメータも同時に再生する
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics \
    /camera/camera/depth/color/points \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info \
  --remap /camera/camera/depth/color/points:=/visibility/raw_points \
  --loop

# ターミナル2: GNG 入力用の点群を base_link へ変換する
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py \
  input_topic:=/visibility/raw_points \
  output_topic:=/camera/camera/depth/color/points

# ターミナル3: GNG
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml

# ターミナル4: 深度画像の削除証拠判定を計測する
ros2 run fuzzy_voxel_grid depth_visibility_benchmark
```

この bag でのC++計測では、等倍の深度画像を直接参照する方式が p50 約1.30 ms で最速だった。
実行時の1/2・1/4 min-pooling は各フレームで全画素を走査するため、p50 約7.38 ms / 6.74 ms となり遅い。
解像度を下げるなら、このノード内でpoolingするのではなく、カメラ側で低解像度深度画像を出す。

### 変換済み点群を新しいrosbagへ1周分だけ保存
# 先に上の変換ノードを起動し、次にrecordを開始してから、最後にbagを--loopなしで再生する。

# ターミナル3: 変換済み点群と可視性判定用の生depthをrecord
ros2 bag record \
  -o /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed \
  /camera/camera/depth/color/points \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info

# ターミナル1: record開始後にraw bagを1回だけ再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics \
    /camera/camera/depth/color/points \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw

# 再生終了後、作成後は変換御殿軍を次で再生。
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed --loop

## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \gng_profile_names:=left_arm 
  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証

## 衝突urdfの球化
ros2 launch gng_vlut_system voxel_spherized_robot_viewer.launch.py  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true ( trial_return_home:=true
)

ros2 launch gng_vlut_system target_joint_state_executor.launch.py   robot_name:=ToPoDualArm   target_topic:=target_joint_states   state_topic:=joint_states   command_topic:=joint_commands   max_joint_velocity:=0.6   publish_hz:=20.0


python3 test_tf_once_publisher.py   --world-frame world   --frame-id ToPoDualArm/base_link   --x 0.35   --y 0.15   --z -0.3 --yaw 3.2  --hold-seconds 1.0   --publish-hz 20


同じ座標系でそのまま通す場合は、`voxel_to_vlut` の `target_frame_id` は指定しません。
このとき、入力 voxel の frame をそのまま使って再エンコードします。


python3 -m pip install --user torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu


## GNGノードを、把持候補の前段となるラベル付きボクセルへ変換?
`/downsampling/grasp_support` は `UNKNOWN_OBJECT` を優先し、`DEFAULT`で残りを補う
把持支持点群である。`graspnet.yaml`では最大10,000点（unknown最大5,000点）を空間的な
カバレッジ優先で選ぶ。GNG本体は、別に設定した入力点群を均一選択して学習する。
旧`/downsampling/unknown`は同じ内容を出す互換トピックであり、
新規の処理には使わない。
`HUMAN`、`CAR`を除外し、現在の点群支持があるセルを対象にする。`SAFE_TERRAIN`は候補としては除外するが、
床際物体の履歴を切らないため内部の構造証拠としては保持する。`auto`支持は各セルごとに
GNGの`inpcl_ids`と現在点群の近傍支持の大きい方を使うため、IDが一部のノードで欠けても安定した点群を失わない。セルの有効／無効は、
点群支持の在席率と切替率から求める時間安定度で決める。現在の点群支持は周囲27セルの中央値で正規化し、
点群・ノード密度や`grid_size`が変わっても固定個数閾値への依存を避ける。EMAはMap headerの時刻差から
更新するため、点群レートにも依存しない。単発の欠落は残し、繰り返すON/OFFは切替率が上がるため抑制する。
時間安定度は `在席率 × (1 - 切替率)` であり、rayを使わずに一時欠落と反復フリッカへの扱いを分ける。
さらにネイティブ深度画像が同時刻にあるときは、各ボクセル中心を1回だけ射影して3×3画素の中央値を比較する。
画角外と手前物体による遮蔽では履歴を保持し、対応画素の深度がセルより奥にある（その場所が空いた）場合だけ
負の証拠として減衰させる。全rayの走査はしない。深度または座標変換が得られないフレームは従来の時間減衰に戻る。
`depth_visibility_*` と `camera_to_map.*` は通常触らず、プロファイル YAML にだけ置いてある。TFを優先し、
録画に `base_link`→カメラTFがない場合だけ既存の点群変換キャリブレーションをフォールバックとして使う。
深度がない場合も、過去セルのGNG近傍が複数残り、局所的に静止または移動方向が不一致なら、その既知セルだけを保持する。
近傍が局所edge間隔に対して十分に大きく、同方向に並進したときは、移動物体の旧位置だけでなく
現在観測中のセルにも連続的な負の証拠を入れる。そのため、腕だけでなく小さく揺れる胴体も
把持候補として残り続けない。単ノード移動では発火せず、少なくとも2近傍の整合した並進だけを使う。
これは前後フレームの既存GNG edgeとノード位置だけを1回走査する。新規セルの膨張、点群kNN・PCA、全ボクセル走査は行わない。
単ノード法線方向の動きは補助オプションであり、既定では無効である。
`SAFE_TERRAIN`は新規候補にはしないが、内部ではGNG構造・在席の証拠として残す。
そのため床際の物体で`UNKNOWN_OBJECT`と`SAFE_TERRAIN`が交互になっても、terrainを観測欠損としては扱わない。
ただし単発unknownは候補化せず、同一フレームのGNG edge連結成分が**4ノードかつ4つの点群支持済み出力セル**を
占有したときだけ物体証拠として履歴へ記録する。`unknown_component_event_count`、
`unknown_component_event_node_count`、`unknown_component_event_voxel_count`で確認できる。
直接観測セルの連結性は26近傍セルではなく、両端が安定したGNG edgeで判定する。細かい`grid_size`で
途中セルが抜けても、そのedge上だけを補間して連結を維持する。edge長の除外も固定距離ではなく、
各端点の周辺GNG edge長に対するロバストな外れ値で決める。GNG edgeを持たない直接観測セルだけを
`<output_topic>/isolated`へ分離する。
現在点があるセルでも、周囲のラベルセル（unknownを含む）がなく、別セルへ出るGNG edgeもない場合は
孤立ノイズとして時間安定度に負の証拠を入れる。即時削除ではないため、同じ
`temporal_time_constant_sec`とヒステリシスで減衰する。細かい`grid_size`によるセル間の隙間を
GNG edgeが跨ぐ場合は、この孤立減衰を適用しない。
`output_topic`は非孤立の直接観測と補間セルの和集合で、`edge_inferred`と`triangle_inferred`は補間由来だけを出す。

ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/grasp_support \
  output_topic:=/topo_voxel_ids \
  grid_size:=0.02

## GNGエッジから増分平面クラスタを作る

点群全体にRANSACを掛けず、`TopologicalMap`の既存GNG edgeとGNGノード法線から平面クラスタを作る。
GNGノードID単位の所属を持ち越して差分だけ直すため、所属が定常状態に落ち着く。出力の
`node_indices`は入力`TopologicalMap.nodes`への対応である。
1フレームは`O(N + E)`とクラスタ数ぶんの3x3固有値分解だけで、優先度付きキューや
クラスタ同士の総当たりを使わない。

所属が動くのは、平面から離れすぎたとき（解放）、未所属で条件を満たしたとき（取り込み）、
別クラスタへ明確により適合するとき（移動）に限る。取り込みと移動には
**移動先クラスタにすでに所属している隣接ノードが2つ以上あること**を要求し、
1本のエッジだけで所属が漏れ出すのを防ぐ。

```bash
ros2 launch ais_gng plane_cluster_incremental.launch.py \
  input_topic:=/topological_map
```

`/topological_planar_clusters_incremental/markers/obb` と
`/topological_planar_clusters_incremental/markers/nodes` 。


## 把持ボクセルテンプレート（左グリッパ、POC）

上の`/topo_voxel_ids`を、最大把持体積・最小把持体積・グリッパ開閉の掃引禁止体積へ照合して
TCP姿勢候補を出す。通常は引数なしでよい。この起動で`/grasp_pose_markers`も同時に出力する。

```bash
ros2 launch grasping_system grasp_voxel_template.launch.py
```

候補と照合内訳は次で確認する。

```bash
ros2 topic echo /grasp_pose_cands
ros2 topic echo /grasp_pose_cand_cells
ros2 topic echo /grasp_pose_cands/summary
```

`/grasp_pose_markers`をViewerの`Connection & Streams`で有効化する。

`/grasp_pose_cand_cells`は、テンプレート照合・対向接触・禁止体積の各条件を通過し、
同じTCPセルに密集する姿勢を1件へ代表化した候補セルである。`labels`は現在すべて`1`であり、
「テンプレート上の把持候補TCPセル」を表す。平行移動なしの実ロボット到達性やIK可否は、この段階では
含まれない。`/grasp_pose_cands`の`PoseArray`は各候補セルの代表姿勢だけを出力する。

summaryには、代表化前の`raw_candidate_num`、TCPセル数の`candidate_cell_num`、同一TCPセルから
抑制した`suppressed_same_tcp_cell_num`、および法線・平面クラスタとの接触根拠を出力する。
`contact_normal_alignment`は対向する両接触帯にあるGNG法線とグリッパ閉鎖軸の絶対内積の小さい側、
`planar_contact_ratio`は両接触帯で平面クラスタに所属する法線の割合の小さい側である。法線または
同一フレームの平面クラスタが未着時は`-1`であり、これらの値だけで候補を棄却しない。

同一フレームの平面クラスタに所属するGNGノードは、`grip_V`の外側にある指・基部の掃引禁止体積に
対する疎な衝突面としても使う。床・壁・周辺物体の平面ノードが禁止体積へ入る候補は、この衝突判定で
除外する。一方、把持対象の平面ノードが`grip_V`内にある場合は、把持対象との接触を許容するため
衝突扱いにしない。凸包やOBBを塗りつぶさず、実在するGNGノードセルだけを使うため、未観測穴を
障害物として過大に埋めない。summaryの`planar_collision_cell_num`は、この衝突面に使ったセル数である。

さらに、単眼の点群で未観測側を空き空間とみなさないため、深度可視性を有効にした場合は深度画像
`/camera/camera/depth/image_rect_raw`とCameraInfo
`/camera/camera/depth/camera_info`も参照する。各姿勢について`grip_V`の外側にある掃引体積の代表セルを
深度画像へ投影し、セル中心より深い深度が得られる観測自由空間だけを通過させる。深度欠損、画角外、
手前の物体による遮蔽、TF未取得はいずれも候補の棄却条件である。深度との差分余裕は`grid_size`と同じ
ため、固定のmmしきい値は用いない。深度はボクセル入力と同じヘッダ時刻のフレームだけを使う。深度または
CameraInfoが未着の間は、古い候補を残さず空の候補を出力する。summaryの`rejected_visibility`で、この判定
による棄却数を確認できる。

現在の`graspnet_player`は点群だけを配信するため、通常起動では深度可視性を無効にしている。深度画像・
CameraInfo・時刻整合するTFも配信できる入力でだけ、次の詳細指定により有効化する。

```bash
ros2 launch grasping_system grasp_voxel_template.launch.py \
  enable_depth_visibility:=true
```

これは安定した物体候補ボクセルを対象にする一次POCである。`grip_sweptV`はグリッパ基部と
開状態から閉状態までの指形状を合成した禁止体積である。ただし最大開口の`grip_V`内は把持対象が
接触してよい領域として除外する。さらに、同じ開度で左右の接触帯が同時に占有されることを要求し、
指の閉鎖軸と直交する二軸の両端まで連続する面状占有は除外する。GNG成分に基づく「取り出せる
物体」への分離、全環境点群から作る衝突専用占有、進入経路の掃引判定は次段で追加する。平面クラスタは
ペットボトルなど把持対象の表面にもなり得るため、`grip_V`内外の区別を伴う掃引衝突面として用いる。

通常起動で指定できる値は上の4つだけである。詳細設定は
`ais_gng_cpu/src/ais_gng/config/topological_grid.yaml`に集約した。
別プロファイルを試すときだけ、`params_file:=/absolute/path/profile.yaml`を追加する。

主な調整点は`temporal_time_constant_sec`（在席率・切替率の時間定数）、`temporal_activation_score`（発火）、
`temporal_retention_score`（消失）である。
`retention_score < activation_score`のヒステリシスを保つ。`unknown_shape_filter_enabled`は既定で無効で、
形状の逸脱だけを物体判定にしない。summaryにはこれらの設定値と活動度を出す。
`<output_topic>/summary`は更新ごとの集計だけを送る。全セルのデバッグ明細は、必要なときだけ
`<output_topic>/assignments`を購読する。購読者がいない通常運用では明細JSONを組み立てないため、
多数ボクセル時の通信量とCPU負荷を増やさない。
深度が有効なときは `depth_visibility_free_count`、`depth_visibility_occluded_count`、
`depth_visibility_out_of_view_count` もsummaryに出る。移動物体の削除証拠として見るのは `free_count` だけである。
局所構造の判定数は `local_structure_static_node_count`、`local_structure_moving_node_count`、
`local_structure_ambiguous_node_count` に、これで保持された旧セル数は `retained_by_local_structure`、
現在セルで運動抑制が残っている数は `local_motion_suppressed_voxel_count` に出る。
GNG法線方向の補助スコアは `normal_drift_mean_score` と `normal_drift_maximum_score` に出る。
`edge_max_length: 0.0` は局所GNG edge長から自動で外れedgeを除く設定である。出力の`grid_size`は
把持テンプレートの必要解像度で選び、物体連結性のために大きくする必要はない。
`point_support_radius_m` を使う場合は、同一フレームで多数回行う半径内の点群支持数を疎な
8×8×8セルの積分キャッシュから取得する。従来と同じ格子立方体内の点数になるため候補結果は
変えない。極端に疎な入力またはキャッシュが大きくなり過ぎる入力では自動的に従来の直接探索へ戻る。

`point_activity_update_enabled:=true`では、点群占有頻度と点密度から重い更新の実行間隔を連続的に変える。
静止点群では更新を間引き、新しい占有や消失が多いと毎入力へ近づく。出力ボクセルとは別の物理セルで統計を取るため、`grid_size`を小さくしても活動度の統計セル数が過剰に増えにくい。

## realsense 
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true


## Gazeboに召喚
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true

## Gazeboでベースをワールドに固定したい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint

## Gazeboのピックアンドプレース用worldを使う場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint \
  world:=/ros2_ws/src/gng_vlut_system/worlds/pick_and_place.world

このworldには作業台、動的な立方体・円柱・直方体、配置用トレイ2個が含まれる。
現在の`dual_arm_robot.urdf`には`gazebo_ros2_control`がないため、Gazebo内の関節と
グリッパを指令して実際に把持するには、別途Gazebo用controller接続が必要。

## GazeboでTF追従させたい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  follow_tf_frame:=ToPoDualArm/base_footprint






必要に応じて
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

alias sh='source /opt/ros/humble/setup.bash'
alias sw='source /ros2_ws/install/setup.bash'



### このリポジトリでの前提
- `Dockerfile` に `ros-humble-rosbridge-server` が入っています
- `docker-compose.yaml` に `rosbridge` サービスがあります
- そのため、通常は `docker compose up -d` で足ります

### 素の ROS2 環境で入れる場合
```bash
sudo apt install ros-humble-rosbridge-server
```

### 起動手順
```bash
# 1) rosbridge を起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

このlaunchはWebSocketに加えて`rosapi`を起動する。単体HTMLは`rosapi`から
`*_grip_V_topological_map`、`*_grip_minV_topological_map`、`*_grip_baseV_topological_map`、
`*_grip_sweptV_topological_map`
topicを自動発見するため、`ros2 run
rosbridge_server rosbridge_websocket`だけではなく上記launchを使用する。

```bash
python3 -m http.server 8000
```

```text
# 3) ブラウザで開く
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html
```


## 左腕をtopological_map_avoidanceで動かす
ros2 launch gng_vlut_system topological_map_avoidance.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  trial_mode:=true \
  trial_safe_only:=true
