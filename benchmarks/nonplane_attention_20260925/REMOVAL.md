# 非平面重点削除後の確認（2026-09-25）

## 要約

追加した非平面重点処理・設定・専用トピックを削除。
通常のunknown学習、非平面成分抽出、把持・境界重点は維持。
CPU回帰21対象、把持・境界・非平面成分抽出のROS単体3対象成功。
通常配布先の2パッケージのビルドと、実bag12フレーム×3条件のスモーク検証成功。

## 条件・検証

実行環境は`gng_cpu_container`のROS Humble。既存のGNG・bag・Viewerには触れず、ROSドメイン178で分離。
入力は交差点bagの`/lidar_points`先頭12件。10万点、総学習4,000回、分類器OFF、入力座標のままで検証。
通常、非平面成分出力OFF、平面OFFの3条件で、非空Tmapの生成を確認。
通常条件では成分情報とノード所属を照合。他2条件では成分Publisherなしを確認。
全条件で専用重点Publisherと専用ROSパラメータの不在を検査。
上位のlaunch全体・実Viewer表示・削除後の性能値は今回の検証対象外。

コンテナ内の起動コマンド：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
ROS_DOMAIN_ID=178 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 240 \
  python3 benchmarks/nonplane_attention_20260925/removal_smoke.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
```

スクリプト内の各ノード起動はログの`start:`行に記録。`finally`で自分が起動したプロセスグループだけを停止。
試験ノードPID 315824・315845・315861はすべて終了コード0。
既存bag PID 293745、GNG launch 305234／node 305235、Viewer launch 305269と既存3コンテナを維持。
新規デーモン・試験プロセスの残存なし。すべてのビルド・試験セッション終了を確認。
配布先の廃止ヘッダーへの壊れたsymlink 2件もバックアップ先へ退避。
ビルド・テストログ：`artifacts/nonplane_attention_removal_20260925/`（Git対象外）。
削除前ソースのバックアップ：同ディレクトリの`source_before.tar.gz`。

通常利用は再ビルド後、使用中launchを終了して同じコマンドで再起動：

```bash
ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

削除済みの旧ベンチ実行コマンドは履歴資料。現行ソースでの再現方法ではない。
撤去判断と互換性：[リリースノート](../../gng_vlut_system/docs/releases/2026-09-25_nonplane_attention.md)。
