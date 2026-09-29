# 2026-09-28 - Gazebo点群・GNG・VLUTによる双腕退避

## 1. 要約

実レイLiDAR → 自己点群除去 → 占有・危険voxel → VLUT → 学習済みGNG姿勢 → 関節指令を接続。
経路の中間姿勢も点群で確認し、入口が見つからない場合と探索待ち中は局所退避で補完。探索は別プロセス。
位置直接設定を有限トルクODEモータへ変更。URDFの速度・トルク上限の95%を駆動上限として使用。
**全21関節の実測上限照合は成功。3秒接近は左右成功例もあるが、再試験では右側が停止距離に到達し、安定回避は未達。**

```bash
ros2 launch gng_vlut_system dual_arm_gng_lidar_demo.launch.py
```

maxが既定。`gui:=false`で画面なし、`enable_auto_start:=false`で手動開始。
開始・停止は`/sim_topo_dual_arm_max/avoidance/{start,stop}`のTrigger。通常・幾何退避デモとは別に起動。
max / longのGNG＋VLUTは[再生成済み](2026-09-28_effectivity_map_refresh.md)。有限トルクでの実行検証はmaxのみ。
Viewerでは`sim_topo_dual_arm_max`、同名前空間の`lidar_points`・`Tmap_static`・`plan_Tmap`・`avoidance/markers`を表示。

観測点群との腕別余裕から対象腕を選択。片側接近では非対象腕・胴体・指を実測姿勢に固定し、
GNG目標・局所補完・復帰に適用。両側接近では両腕を対象とし、接近終了後の復帰は片腕ずつ。
対象腕の切替で古い経路と非同期探索結果を破棄。診断の`active_arm_joints`で対象関節を確認可能。
全身の自己干渉・点群余裕を中間姿勢でも確認。片腕探索で経路が見つからず、その探索中に
左右腕間の干渉を検出した場合だけ両腕へ拡張して再探索。床・作業台・胴体との干渉、
点群による行き止まり、探索時間切れだけでは拡張しない。片腕で不可能という厳密な証明ではない。
協調経路にも同じ安全判定を適用。採用した経路の実行中は協調対象を維持し、完了後に腕別選択へ復帰。
協調探索も失敗した場合は元の対象腕へ戻り、局所補完または保持。`is_coordinated`を診断に追加。
探索上限`max_plan_sec`は各段階に適用し、二段階時は最大約2倍にソート・単一判定の処理時間を加算。
14関節GNGを対象腕へ射影するため、元のVLUTラベルだけで射影姿勢の安全性を判断しない。
元ノードの安全ラベルによる候補制限は残り、片腕経路の完全性は保証しない。
`plan_Tmap`は元ノード列の表示であり、固定した反対腕を含む実行姿勢の表示ではない。
幾何退避のみの別デモと共有C++計画器は今回の対象外。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 学習 | max / longとも上限10,000、初期240万回・衝突考慮10万回・座標エッジ各10万回。初回maxは実数10,000ノード・左右2層、VLUTまで1,719.646 s。最新の両モデル再生成結果は[更新記録](2026-09-28_effectivity_map_refresh.md) |
| LiDAR | CPU ray、180×72、10 Hz、距離0.07～2.5 m、world位置(0.85,0,0.75)、pitch=0.4 rad、yaw=π |
| 点群 | base_link座標、2 cm voxel、自己除去、危険領域12 cm膨張 |
| 経路・配信 | VLUT安全ノード・関節角エッジ、最大0.08 rad刻みの中間確認。形状・角度は初回取得、`gng_node_states`はID・labelの交互配列 |
| 補完 | `enable_local_refinement`で局所退避の切替。GNGだけの方式とは区別 |
| 接近 | 0.42 mを3 s（0.14 m/s）。各側で保持2 s・後退6 s・復帰待機20 s。シミュレーション時刻 |
| 駆動 | 腕5.7 N m・2.85 rad/s。補間指令2.5 rad/s、指令周期0.15 s、物理・モータ更新1 ms。上限・ゲインは[物理設定メモ](../dual_arm_simulation.md) |
| 監視 | 実測速度・反力のURDF上限超過／欠落、関節・前腕・点群・voxel・グラフの失効で停止。自動再開なし |
| 数値許容差 | JointState受信値と角度差分の照合。URDF上限に対する丸め許容差は各単位1e-6。駆動余裕で測定誤差を吸収 |

最終設定の結果は`artifacts/dual_arm_effort_20260928/`。

| 試験 | 結果 |
| --- | --- |
| `motor_margin_test` | 左右退避・復帰・Gazebo停止・LiDAR単独欠測・復旧後停止維持に成功。最小推定余裕0.07224 m |
| 同試験の上限 | 21関節すべてURDF内。腕の最大反力5.71094 N m、最大速度2.85000 rad/s。駆動トルクと関節反力は別の値 |
| `avoidance_limits_test` | 再試験でも21関節すべてURDF内。腕最大反力5.70035 N m、最大速度2.85000 rad/s。右側余裕0.03463 mで停止（判定距離0.035 m）、回避全体は失敗 |
| `normal_limits_test` | 通常8姿勢・指開閉・途中停止・再開・状態失効停止に成功。最大速度0.10442 rad/s、最終角度誤差0.000313 rad |
| 単体・ビルド | 上限超過・mimic速度・欠落／NaN監視3件（CTest成功）、既存GNG経路6件成功。新規hardwareをReleaseビルド・install反映 |

3秒接近の成功例の制御tickは平均14.560 ms／最大160.453 ms。探索・診断配信を除く値で周期保証ではない。
通常はODE quick、点群回避はYAMLの`physics_solver: world`。直接解法の`LCP internal error`警告は残存。
ソルバーや実行タイミングで回避成否が変わるため、単一成功例を実環境の回避能力の証明として扱わない。
退避先選択は観測点群のみ。前腕真値はシナリオ移動・表示・停止距離監視用。遮蔽・連続軌道の安全保証は対象外。
URDFの質量・慣性・上限と実機定格の整合、実機PID・遅延・接触応答は未検証。

実行した最終試験コマンド（コンテナでROS環境をsource、各試験のlaunchはGazeboポート11359）：

```bash
export ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1
python3 /ros2_ws/src/gng_vlut_system/test/check_dual_arm_gazebo_demo.py \
  --output /ros2_ws/src/artifacts/dual_arm_effort_20260928/normal_limits_test
python3 /ros2_ws/src/gng_vlut_system/test/check_dual_arm_avoidance_demo.py \
  --launch-file dual_arm_gng_lidar_demo.launch.py \
  --output /ros2_ws/src/artifacts/dual_arm_effort_20260928/avoidance_limits_test
```

他の試行も同コマンドの出力先を各試行名へ変更。設定・実装の最終スナップショットは`final_configuration.json`、上限集計は`limits_reports.json`。
ビルドは`cmake --build /ros2_ws/build/gng_vlut_system --target bounded_gazebo_system -j2`、`cmake --install /ros2_ws/build/gng_vlut_system`。
CTestは`ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^test_dual_arm_limits$' --output-on-failure`。

初期試行のPD発散、読込み前の物理開始、可動端のモータ力、YAML別名・実行属性・CMake依存順の問題を修正。
反復解法での3秒接近は停止、直接解法での通常指開閉は速度超過を記録。固定基部の接触除外は効果未確認で撤回。
上限100%駆動では反力が6.019 N mに達したため95%へ変更。判定を緩めず、現行の実測照合はURDF上限内。
旧位置直接設定の3秒試験では4.555 rad/sを観測。旧結果は`artifacts/dual_arm_speed_3sec_20260928/`に保存し、現行結果と区別。
Viewerの全配信は移行前にWebSocketで確認済み（`artifacts/dual_arm_gng_lidar_20260928/`）。今回のGUI目視・Viewer再試験は未実施。

学習の再現は`ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml gng_data_directory:=/ros2_ws/src/artifacts/dual_arm_gng_10000_20260928/staging`。
学習時の衝突削除はノード・エッジとも0。旧1,000ノードのバックアップと学習ログは同artifactディレクトリ。
今回起動した全試験launch・探索子プロセス・開始補助ノードを終了。既存プロセス・コンテナへの停止操作なし。
終了時に別作業の`/tmp/ode_probe_20260928/`を使用するGazeboを観測したが、操作なし。

腕別選択・協調再探索の変更後は次の単体試験19件に成功。左右選択・両側接近・非対象関節の保持・
復帰・古い探索の破棄・自己干渉棄却・協調成功と失敗・拡張不要条件・次周期の対象維持を含む。上表は腕別選択変更前の結果。変更後の結果は以下。

```bash
docker exec gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/local_setup.bash; python3 /ros2_ws/src/gng_vlut_system/test/test_gng_lidar_path.py'
```

この追加試験と探索子プロセスは終了済み。新規ROSノード起動なし。
SciPyのNumPyバージョン警告あり、試験は成功。

新max mapと腕別選択のGazebo試験は左右退避・復帰を完了し、最小推定余裕0.05199 m。
診断653件中、左のみ117件・右のみ122件・選択なし414件、両腕協調0件。
経路採用13回・GNG指令選択35回。協調分岐自体は単体試験で検証し、このシナリオでは未発生。
ただし試験全体は起動直後の速度上限超過で失敗。L_joint2はJointState最大8.974 rad/s、上限3 rad/s。
記録した超過7件は時刻0.730～0.733 s、回避状態の初回受信前。原因の確定・物理系の修正は未実施。
後続の欠測停止検査へは未到達。試験launch・Gazebo・探索子プロセスは終了済み。
詳細・実行コマンドは[再生成資料](../../../artifacts/effectivity_refresh_20260928/README.md)。
