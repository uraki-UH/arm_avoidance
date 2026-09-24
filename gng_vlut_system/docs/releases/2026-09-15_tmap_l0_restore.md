# 2026-09-15 - Tmap集約L0の配信復旧

## 1. 要約

`/ToPoDualArm/Tmap_static`と同じ元GNGから集約した`/ToPoDualArm/Tmap_vis_L0`を、
指定の`gng_viewer_bridge.launch.py`だけで配信可能な状態へ復旧。

- `ToPoDualArm10000/vis_gng_L0.bin`をVIZGNG4/version 4からVIZGNG5/version 5へ再生成。
- 同じ生成結果の`vis_gng_static_L0.bin`も更新。既存の150ノード設定を維持。
- 読込失敗ログへ現在のtrainerでの再生成案内を追加し、成功ログのtopic名を実際の`_L0`表記へ修正。
- RUN_GUIDEへ元マップ・集約マップの表示先を追記。

新規ROSノード・launch・トピック・メッセージ・パラメータなし。

`visualization_gng.enabled: true`にもかかわらず、保存データとreaderの形式不一致によって
集約マップの読み込みがスキップされていた状態。

**削除**

元GNG・VLUT・既存機能の削除なし。旧集約binは`tmp/tmap_l0_20260915/backup/`へ退避済み。

## 2. 条件・検証

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

- 元マップ: `/ToPoDualArm/Tmap_static`、10,801ノード。
- 集約マップ: `/ToPoDualArm/Tmap_vis_L0`、150ノード・554エッジ。
- 全元ノードの所属を保持し、連結成分1、孤立ノード0。
- 両者のframeは`ToPoDualArm/base_link`。ViewerのTopicsで表示を選択。
- 起動時は保存済みモデルの読み込みのみ。元マップの再購読・オンライン再学習は追加なし。
- 既に起動済みのbridgeにはbinの再読込機能がないため、反映には上記launchの再起動が必要。

既存の`visualization_gng.enabled/path_prefix/topic_prefix`を使用。
両出力とも`ais_gng_msgs/msg/TopologicalMap`、reliable・transient_local。
`L0`は既存の座標レイヤー0の名称であり、新たな多階層プランナーの導入ではない。
所属元の安全状態集約・既存軌道変換は従来の実装を継続。

- Dockerで現在のbridgeとtrainerをReleaseビルド。
- オフライン再生成4.24028秒。全10,801ノードの重複なし所属、代表ノード・代表関節角、
  エッジ・遷移列の保存後再読込検証に成功。
- ROS domain 219で上記と同一launchを実行し、両トピックの配信を確認。
  元ノードID集合と集約bin所属集合の一致、集約位置・エッジのbinとの一致、frame一致、
  有限座標、自己ループ・重複エッジなしを検証。
- 専用Gatewayへの遅延接続で両トピックの自動発見・購読に成功。
  Viewer向けTMG1の集約データ14,897 bytesを取得し、ノード数・エッジ数・frameを照合。
  ブラウザ画面の目視は今回未検証。
- 元`gng.bin`・`vlut.bin`のSHA256が作業前後で一致。旧集約binの退避先も元SHA256と一致。

実行コマンド:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/gng_vlut_system --target topofuzzy_bridge_node visualization_gng_trainer -j2'

docker exec -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/tmap_l0_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 360s nice -n 10 /ros2_ws/build/gng_vlut_system/src/visualization_gng_trainer --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin --output-prefix /ros2_ws/src/tmp/tmap_l0_20260915/generated/vis_gng --target-nodes 150 --iterations 200000 --seed 42 --joint-motion-weight 1.0 --workspace-motion-sec-per-m 1.0 --workspace-sample-resolution 0.05 --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml > /ros2_ws/src/tmp/tmap_l0_20260915/trainer.log 2>&1'

docker exec -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/tmap_l0_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 100s python3 /ros2_ws/src/tmp/tmap_l0_20260915/check_launch.py'
```

検証スクリプトの子プロセス（domain 219）:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node --ros-args -r __node:=tmap_l0_gateway_check -p port:=19096
```

再生成結果の照合後、旧binを退避して元の配置先へ入替済み。
別のノード数へ変更する場合は`--target-nodes`を変更して再生成。
直接の配置先は`--output-prefix /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng`。

trainer・検証launch PID 1774999と全子ノード・Gateway PID 1775133は終了済み。
専用ROSログを削除し、ポート19096の待受け消滅を確認。
`ss`未導入のため待受け検査は`/proc/net/tcp`・`tcp6`で実施。
検証終了時のSIGINTで初期化中の一部子ノードにexit -2の記録あり。配信検証完了後の意図的停止。
既存launch PID 1773998と他の既存ROS・コンテナを維持し、停止・再起動操作なし。

**制約**

- 集約マップは元関節姿勢群の要約。集約ノード間の任意の移動や安全性を保証する経路ではない。
- binは従来からGit管理外の生成物。今回の更新は作業環境内のファイルに適用。
  別環境では同じ元GNGに対して再生成が必要。
- 再現用スクリプト、生成結果、旧bin、ROS/WS取得結果を`tmp/tmap_l0_20260915/`に保持。

### 集約形状の追加調査

ユーザーからエッジのガタつきを指摘され、現行生成処理と新旧binを読取比較。
生成処理・パラメータ・binの追加変更なし。

- 150ノードの座標は旧version 4のbinと完全一致。今回の再生成でノード配置は改善していない。
- 所属は関節移動時間と手先位置の複合特徴量で決定。描画位置は学習点や重心ではなく所属元の代表TCP位置。
- 学習直後の元coord edge縮約結果を、後続のFK補間・空間最近傍対応・短辺選択で上書き。
  始終点の所属と中間点の対応で基準が異なり、対応先なしの補間点も省略。
- 新binのエッジ長は中央値0.087678 m、95パーセンタイル0.147931 m、最大0.243705 m。
  ノードの空間最近傍距離中央値は0.047005 m。旧binは4,483エッジ、中央値0.233100 m、最大0.690107 m。
- 上記はEuclidean距離の全辺集計と生成コードから確認した事実。
  これらの構成は空間的なガタつきの要因になり得るが、画面上の症状との目視照合は未実施。
  前回の配信・連結性検証は、形状の滑らかさや元位相保持の品質検証ではない。
