# 代表姿勢と元GNGの可動域被覆検証（2026-10-01）

結論: 元の1万ノードGNG自体に、既知の到達可能位置に対する大きな未被覆領域あり。今回の代表抽出による追加損失だけでは説明できない結果。元GNGと代表版のいずれも、可動域全体の近傍被覆を満たすモデルとしては不十分。

評価基準: 参照TCP点から最寄りGNGノードのTCPまでのユークリッド距離。5 cm以内にノードがある参照点の割合。空間体積の厳密な割合、IK成功率、経路実行成功率とは別の指標。

## 独立可動域との比較

| モデル | 腕 | 参照点数 | 元GNGの5 cm以内率 | 代表版の5 cm以内率 | 元GNGの最大距離 | 代表版の最大距離 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| max | 左 | 6,927 | 22.95% | 21.97% | 42.63 cm | 42.63 cm |
| max | 右 | 6,954 | 16.01% | 15.55% | 49.57 cm | 49.67 cm |
| long | 左 | 10,789 | 23.54% | 23.25% | 49.75 cm | 49.75 cm |
| long | 右 | 10,730 | 22.23% | 21.90% | 50.64 cm | 50.64 cm |

参照: 元GNGから独立して生成された既存reachabilityマップ4個。片腕ごとに5万の低差異サンプル＋2万の補完サンプル、3 cmセルごとの実関節角を保存。自己干渉判定あり、他腕・腰・頭・グリッパーはゼロ姿勢。環境・床の衝突、双腕同時姿勢の成立、手先姿勢角は対象外。[既存マップの生成条件](../../gng_vlut_system/docs/reachability_maps.md)。

位置の取扱い: マップの表示座標はセル中心のため、そのまま比較せず、各セルに保存された証拠関節角をURDFの独立FKで実TCPへ復元。既知到達セル1個につき1点を同じ重みで集計。未登録セルの到達不能やセル全域の到達可能性を意味しない。

座標の検査: 元GNGの全1万姿勢を同じ独立FKで再計算し、保存TCPとの最大差4.21e-8 m。全証拠姿勢のTCPが元3 cmセル内（各軸±1.5 cm）に存在。URDF rootはbase_footprint、base_linkへの変換は恒等。格子中心誤差や座標系の取り違えによる数十cmの差ではないことを確認。

既存マップの独立1万姿勢検査はmaxで約99.2%、longで約97.6%のセル一致。この値は既存マップ自身の関節サンプルに対する再現率であり、今回のGNGの被覆率とは別。今回、FCLの再実行や新しいランダム姿勢生成はなし。

## 元GNGを基準とした圧縮前後の比較

| モデル | 元/代表数 | 左TCPが2 cm以内 | 右TCPが2 cm以内 | 同じ代表で左右とも2 cm以内 | 同じ代表で左右とも4 cm以内 |
| --- | ---: | ---: | ---: | ---: | ---: |
| max | 10,000 / 1,593 | 99.01% | 99.68% | 19.75% | 69.09% |
| long | 10,000 / 4,831 | 99.46% | 99.55% | 49.33% | 77.19% |

片腕ごとの最近傍は、左右で異なる代表を選択可能。左右同時は同じ14関節代表に対する左右距離の大きい方を最小化。元モデルに対する99%以上の保持率は、可動域全体の被覆保証にはならないことを確認。今回の代表はViewer用trial 2、許容差8 cm、seed 20260931。

関節姿勢: 14関節の最大絶対差を最小化する代表との距離の中央値はmax 52.74度、long 24.36度。周期補正なし。近いTCP位置に同じ関節姿勢が残ることは未保証。詳細な全体・非代表のみの分布を生JSONへ保存。

接続: 関節空間側の保存辺では元・代表とも1連結成分。座標側は元から断片化があり、代表ではmax左978/右996、long左2,681/右2,836ノードが孤立。全保存辺による構造上の集計であり、補間軌道の衝突検証とは別。

## 元モデルの被覆が狭い理由

確認事項: maxのL_joint4はURDFで約−115〜95度、元GNGでは約−43.3〜21.5度（幅30.9%）。他の関節も元GNGの最小最大がURDFの全範囲に達していない状態。全14関節の値・範囲比をranges.jsonへ保存。

学習実装: 14関節角のユークリッド距離で勝者選択、勝者と隣接ノードの角度平均化、新ノードの角度中点による追加。TCPの未被覆領域や可動域端の証拠姿勢を固定保持する条件なし。各腕の到達可能領域を許容誤差内で覆う目的関数とは異なる構成。

生成ログ: 両モデルともuse_task_density_bias=false、1万ノード、240万＋10万学習。初期学習後・精緻化後のStrictFilterは削除ノード0・削除辺0。大量の衝突フィルタ削除による欠落という説明とは不整合。

原因の解釈: 角度空間での平均化と未被覆点の保存条件不足が有力。平均化・隣接更新・ノード上限それぞれの寄与率は比較実験していないため、単独原因の断定は未実施。

根拠: [GNGの更新実装](../../gng_vlut_system/src/core/gng/GrowingNeuralGas.cpp)、[学習器](../../gng_vlut_system/src/offline_tools/offline_urdf_trainer.cpp)、[max生成ログ](../../artifacts/effectivity_refresh_20260928/topo_dual_arm_max.log)、[long生成ログ](../../artifacts/effectivity_refresh_20260928/topo_dual_arm_max_long.log)。

次の設計候補: 到達マップの実証拠姿勢から未被覆位置を補完し、独立サンプルに対する位置・双腕同時・必要な手先姿勢角の許容誤差を満たす条件で代表を削減。接続は関節空間側の元経路との対応と補間衝突を別途検証。今回の調査では再学習・モデル更新なし。

## 検証・再現

測定: 元GNG基準2条件、独立可動域基準2条件、全て正常終了。最近傍探索の先頭64姿勢を全点対距離計算と照合、全代表の自己距離0、左右同時距離の上下界を検査。元GNG・代表GNG・入力CSV・可動域マップ・URDFは読取りのみ。

結果と終了記録: [artifacts/voxel_coverage_20261001](../../artifacts/voxel_coverage_20261001)。追加サンプル比較前のROS実行状態と終了後状態を保存。新規ROS・コンテナ起動なし。

再現コマンド（ホスト、出力先は未作成ファイル、依存はNumPy・SciPy・PyYAML）:

```bash
python3 /home/uraki/uraki_ws/benchmarks/voxel_pose_compression_20260930/coverage.py --model max --output /tmp/max_saved_coverage.json
python3 /home/uraki/uraki_ws/benchmarks/voxel_pose_compression_20260930/workspace_coverage.py --model max --output /tmp/max_workspace_coverage.json
```

long版は`--model long`へ変更。出力ファイル名も別名へ変更。

今回の起動コマンド（全て終了済み）:

```bash
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_coverage_20261001/cases.json --output /tmp/voxel_coverage_20261001/results --repeats 1 --timeout-sec 90 --max-total-sec 190 --estimate-sec 8
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_coverage_20261001/cases_workspace.json --output /tmp/voxel_coverage_20261001/workspace_results --repeats 1 --timeout-sec 60 --max-total-sec 130 --estimate-sec 4
```

初回予測/実時間: 元GNG基準16秒/31.73秒、独立可動域基準8秒/12.06秒。性能比較を目的とする数値ではなく有限検証の実行時間。過去ログの/tmpパスは成果保存後に同構成でartifactsへ移動。

Viewerで可動域マップを重ねる例（未起動）:

```bash
ros2 launch gng_vlut_system dual_arm_reachability.launch.py \
  model_dir:=/ros2_ws/src/gng_vlut_system/gng_results/topo_dual_arm_max/reachability \
  namespace:=voxel_max_preview frame_id:=voxel_max_preview/base_link
```

対象トピック: `/voxel_max_preview/reachability_left_arm_Tmap`と`/voxel_max_preview/reachability_right_arm_Tmap`。表示点は3 cmセル中心、点に付属する関節角はセル内の実証拠姿勢。既存GNGの点・辺とは別レイヤー。
